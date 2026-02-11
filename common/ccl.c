/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include "ccl.h"
#include "apriltag_simd.h"
#include <string.h>
#include <assert.h>
#include <limits.h>

ccl_component_t *ccl_create(int width, int height) {
    // Check for overflow in size calculations
    size_t num_pixels = (size_t)width * (size_t)height;
    if (num_pixels / (size_t)width != (size_t)height) {
        return NULL; // Overflow detected
    }
    
    // Check if max_labels will fit in uint32_t
    size_t max_labels_size = num_pixels / 2 + 1;
    if (max_labels_size > UINT32_MAX) {
        return NULL; // Too many labels for uint32_t
    }
    
    ccl_component_t *ccl = (ccl_component_t *)calloc(1, sizeof(ccl_component_t));
    if (!ccl) return NULL;
    
    ccl->width = width;
    ccl->height = height;
    
    // Allocate label array for all pixels
    ccl->labels = (uint32_t *)calloc(num_pixels, sizeof(uint32_t));
    if (!ccl->labels) {
        free(ccl);
        return NULL;
    }
    
    // Allocate equivalence table (worst case: every other pixel is a new component)
    ccl->max_labels = (uint32_t)max_labels_size;
    ccl->equiv = (uint32_t *)malloc(ccl->max_labels * sizeof(uint32_t));
    if (!ccl->equiv) {
        free(ccl->labels);
        free(ccl);
        return NULL;
    }
    
    ccl->comp_size = (uint32_t *)calloc(ccl->max_labels, sizeof(uint32_t));
    if (!ccl->comp_size) {
        free(ccl->equiv);
        free(ccl->labels);
        free(ccl);
        return NULL;
    }
    
    ccl->stats = (ccl_component_stats_t *)calloc(ccl->max_labels, sizeof(ccl_component_stats_t));
    if (!ccl->stats) {
        free(ccl->comp_size);
        free(ccl->equiv);
        free(ccl->labels);
        free(ccl);
        return NULL;
    }
    
    // Initialize equivalence table and stats in a single loop for better cache locality
    for (uint32_t i = 0; i < ccl->max_labels; i++) {
        ccl->equiv[i] = i;
        ccl->stats[i].min_x = UINT32_MAX;
        ccl->stats[i].min_y = UINT32_MAX;
        ccl->stats[i].max_x = 0;
        ccl->stats[i].max_y = 0;
    }
    
    ccl->num_labels = 1; // Start at 1 (0 is background)
    
    return ccl;
}

void ccl_destroy(ccl_component_t *ccl) {
    if (ccl) {
        free(ccl->labels);
        free(ccl->equiv);
        free(ccl->comp_size);
        free(ccl->stats);
        free(ccl);
    }
}

// Find root label with path compression
static inline uint32_t find_root(uint32_t *equiv, uint32_t label) {
    uint32_t root = label;
    
    // Find root
    while (equiv[root] != root) {
        root = equiv[root];
    }
    
    // Path compression
    while (equiv[label] != root) {
        uint32_t next = equiv[label];
        equiv[label] = root;
        label = next;
    }
    
    return root;
}

// Merge two labels in equivalence table
static inline void merge_labels(uint32_t *equiv, uint32_t label1, uint32_t label2) {
    uint32_t root1 = find_root(equiv, label1);
    uint32_t root2 = find_root(equiv, label2);
    
    if (root1 != root2) {
        // Always make smaller root the parent (keeps trees flatter)
        if (root1 < root2) {
            equiv[root2] = root1;
        } else {
            equiv[root1] = root2;
        }
    }
}

uint32_t ccl_process(ccl_component_t *ccl, image_u8_t *threshim) {
    int width = ccl->width;
    int height = ccl->height;
    int stride = threshim->stride;
    uint8_t *buf = threshim->buf;
    uint32_t *labels = ccl->labels;
    uint32_t *equiv = ccl->equiv;
    
    uint32_t next_label = 1; // Start at 1 (0 is background)
    
    // Pass 1: Forward scan with label assignment
    // Process first row separately (no top neighbors)
    for (int x = 1; x < width - 1; x++) {
        uint8_t val = buf[x]; // First row, y=0, so offset is just x
        if (val == 127) continue; // Skip mid-gray pixels
        
        // Check left neighbor
        if (x > 0 && buf[x - 1] == val && labels[x - 1] != 0) {
            labels[x] = labels[x - 1];
        } else {
            // New label
            if (next_label >= ccl->max_labels) {
                // This shouldn't happen in practice, but handle gracefully
                labels[x] = next_label - 1;
            } else {
                labels[x] = next_label++;
            }
        }
    }
    
    // Process remaining rows (with top neighbors)
    for (int y = 1; y < height; y++) {
        int row_offset = y * width;
        int prev_row_offset = (y - 1) * width;
        int buf_offset = y * stride;
        int prev_buf_offset = (y - 1) * stride;
        
        // Cache row pointers for better locality
        uint8_t* cur_row = &buf[buf_offset];
        uint8_t* prev_row = &buf[prev_buf_offset];
        uint32_t* label_row = &labels[row_offset];
        uint32_t* prev_label_row = &labels[prev_row_offset];
        
        // Prefetch next row for cache warmup (important for Pi5's cache hierarchy)
        if (__builtin_expect(y + 1 < height, 1)) {
            int next_buf_offset = (y + 1) * stride;
            simd_prefetch(&buf[next_buf_offset]);
            simd_prefetch(&labels[(y + 1) * width]);
        }
        
        for (int x = 1; x < width - 1; x++) {
            uint8_t val = cur_row[x];
            if (__builtin_expect(val == 127, 0)) continue; // Unlikely: skip mid-gray pixels
            
            uint32_t min_label = 0;
            
            // Decision tree approach: Most pixels connect to left or top neighbor
            // Check most common cases first to minimize branches
            // Note: Diagonal checks are duplicated in each branch intentionally -
            // this allows for better branch prediction and avoids extra conditionals
            
            // Pre-load all neighbor values and labels for better pipelining
            uint8_t left_val = cur_row[x - 1];
            uint8_t top_val = prev_row[x];
            uint32_t left_label = label_row[x - 1];
            uint32_t top_label = prev_label_row[x];
            
            // Fast path: Check left neighbor first (most common case)
            if (left_val == val && left_label != 0) {
                min_label = left_label;
                
                // Check if top also matches - need to merge if different
                if (__builtin_expect(top_val == val && top_label != 0 && top_label != left_label, 0)) {
                    merge_labels(equiv, left_label, top_label);
                }
                
                // For white pixels, check diagonals (8-connectivity)
                if (__builtin_expect(val == 255, 1)) {
                    uint8_t top_left_val = prev_row[x - 1];
                    uint32_t top_left_label = prev_label_row[x - 1];
                    if (top_left_val == val && top_left_label != 0 && top_left_label != min_label) {
                        merge_labels(equiv, min_label, top_left_label);
                    }
                    
                    uint8_t top_right_val = prev_row[x + 1];
                    uint32_t top_right_label = prev_label_row[x + 1];
                    if (top_right_val == val && top_right_label != 0 && top_right_label != min_label) {
                        merge_labels(equiv, min_label, top_right_label);
                    }
                }
            } else if (top_val == val && top_label != 0) {
                // Second most common: top neighbor only
                min_label = top_label;
                
                // For white pixels, check diagonals
                if (__builtin_expect(val == 255, 1)) {
                    uint8_t top_left_val = prev_row[x - 1];
                    uint32_t top_left_label = prev_label_row[x - 1];
                    if (top_left_val == val && top_left_label != 0 && top_left_label != min_label) {
                        merge_labels(equiv, min_label, top_left_label);
                    }
                    
                    uint8_t top_right_val = prev_row[x + 1];
                    uint32_t top_right_label = prev_label_row[x + 1];
                    if (top_right_val == val && top_right_label != 0 && top_right_label != min_label) {
                        merge_labels(equiv, min_label, top_right_label);
                    }
                }
            } else if (__builtin_expect(val == 255, 1)) {
                // Rare case: no 4-connected neighbor, check diagonals for white pixels
                uint8_t top_left_val = prev_row[x - 1];
                uint32_t top_left_label = prev_label_row[x - 1];
                if (top_left_val == val && top_left_label != 0) {
                    min_label = top_left_label;
                    
                    uint8_t top_right_val = prev_row[x + 1];
                    uint32_t top_right_label = prev_label_row[x + 1];
                    if (top_right_val == val && top_right_label != 0 && top_right_label != min_label) {
                        merge_labels(equiv, min_label, top_right_label);
                    }
                } else {
                    uint8_t top_right_val = prev_row[x + 1];
                    uint32_t top_right_label = prev_label_row[x + 1];
                    if (top_right_val == val && top_right_label != 0) {
                        min_label = top_right_label;
                    }
                }
            }
            
            // Assign label
            if (__builtin_expect(min_label == 0, 0)) {  // Unlikely: most pixels connect to existing labels
                // New component
                if (__builtin_expect(next_label >= ccl->max_labels, 0)) {
                    label_row[x] = next_label - 1;
                } else {
                    label_row[x] = next_label++;
                }
            } else {
                label_row[x] = min_label;
            }
        }
    }
    
    ccl->num_labels = next_label;
    
    // Pass 2: Resolve equivalences, compute component sizes, and collect statistics
    memset(ccl->comp_size, 0, ccl->max_labels * sizeof(uint32_t));
    
    // Re-initialize stats for clean slate
    for (uint32_t i = 0; i < ccl->max_labels; i++) {
        ccl->stats[i].count = 0;
        ccl->stats[i].sum_x = 0;
        ccl->stats[i].sum_y = 0;
        ccl->stats[i].min_x = UINT32_MAX;
        ccl->stats[i].min_y = UINT32_MAX;
        ccl->stats[i].max_x = 0;
        ccl->stats[i].max_y = 0;
    }
    
    // Flatten equivalence table
    for (uint32_t i = 1; i < next_label; i++) {
        equiv[i] = find_root(equiv, i);
    }
    
    // Count component sizes and collect statistics
    // This is where we merge the clustering step into Pass 2
    // Heavy optimization with prefetching and pointer arithmetic
    for (int y = 0; y < height; y++) {
        int row_offset = y * width;
        uint32_t* label_row = &labels[row_offset];
        
        // Prefetch next row for cache warmup (Cortex-A76 has 64KB L1)
        if (__builtin_expect(y + 1 < height, 1)) {
            simd_prefetch(&labels[(y + 1) * width]);
        }
        
        for (int x = 0; x < width; x++) {
            uint32_t label = label_row[x];
            if (__builtin_expect(label != 0, 1)) {  // Branch hint: most pixels are labeled
                uint32_t root_label = equiv[label];
                label_row[x] = root_label; // Update to root label
                
                // Cache pointer to stats structure to reduce array indexing
                ccl_component_stats_t* stats = &ccl->stats[root_label];
                
                // Prefetch stats for next pixel's likely label (spatial coherence)
                // Bounds check: ensure we don't read past the label array
                if (__builtin_expect(x + 1 < width, 1)) {
                    uint32_t next_label = label_row[x + 1];
                    if (next_label != 0) {
                        uint32_t next_rep = equiv[next_label];
                        simd_prefetch(&ccl->stats[next_rep]);
                    }
                }
                
                // Update statistics (this replaces separate clustering pass)
                ccl->comp_size[root_label]++;
                stats->count++;
                // Don't cast to uint32_t - let it promote to uint64_t for accumulation
                stats->sum_x += x;
                stats->sum_y += y;
                
                // Update bounding box with branchless min/max
                // Cast both sides to maintain type consistency
                stats->min_x = (x < (int)stats->min_x) ? (uint32_t)x : stats->min_x;
                stats->max_x = (x > (int)stats->max_x) ? (uint32_t)x : stats->max_x;
                stats->min_y = (y < (int)stats->min_y) ? (uint32_t)y : stats->min_y;
                stats->max_y = (y > (int)stats->max_y) ? (uint32_t)y : stats->max_y;
            }
        }
    }
    
    // Count unique components
    uint32_t num_components = 0;
    for (uint32_t i = 1; i < next_label; i++) {
        if (equiv[i] == i && ccl->comp_size[i] > 0) {
            num_components++;
        }
    }
    
    return num_components;
}
