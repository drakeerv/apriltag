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
#include <string.h>
#include <assert.h>
#include <stdio.h>

ccl_component_t *ccl_create(int width, int height) {
    ccl_component_t *ccl = (ccl_component_t *)calloc(1, sizeof(ccl_component_t));
    ccl->width = width;
    ccl->height = height;
    
    // Allocate label array for all pixels
    ccl->labels = (uint32_t *)calloc(width * height, sizeof(uint32_t));
    
    // Allocate equivalence table (worst case: every other pixel is a new component)
    ccl->max_labels = (width * height) / 2 + 1;
    ccl->equiv = (uint32_t *)malloc(ccl->max_labels * sizeof(uint32_t));
    ccl->comp_size = (uint32_t *)calloc(ccl->max_labels, sizeof(uint32_t));
    
    // Initialize equivalence table (each label points to itself initially)
    for (uint32_t i = 0; i < ccl->max_labels; i++) {
        ccl->equiv[i] = i;
    }
    
    ccl->num_labels = 1; // Start at 1 (0 is background)
    
    return ccl;
}

void ccl_destroy(ccl_component_t *ccl) {
    if (ccl) {
        free(ccl->labels);
        free(ccl->equiv);
        free(ccl->comp_size);
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
        
        for (int x = 1; x < width - 1; x++) {
            uint8_t val = buf[buf_offset + x];
            if (val == 127) continue; // Skip mid-gray pixels
            
            uint32_t min_label = 0;
            
            // Check 4-connected neighbors: left, top-left, top, top-right
            // For AprilTag, we use 8-connectivity for white pixels
            
            // Left neighbor
            if (x > 0 && buf[buf_offset + x - 1] == val && labels[row_offset + x - 1] != 0) {
                min_label = labels[row_offset + x - 1];
            }
            
            // Top neighbor
            if (buf[prev_buf_offset + x] == val && labels[prev_row_offset + x] != 0) {
                if (min_label == 0) {
                    min_label = labels[prev_row_offset + x];
                } else if (labels[prev_row_offset + x] != min_label) {
                    merge_labels(equiv, min_label, labels[prev_row_offset + x]);
                }
            }
            
            // For white pixels (255), check diagonal neighbors for 8-connectivity
            if (val == 255) {
                // Top-left neighbor
                if (x > 0 && buf[prev_buf_offset + x - 1] == val && labels[prev_row_offset + x - 1] != 0) {
                    if (min_label == 0) {
                        min_label = labels[prev_row_offset + x - 1];
                    } else if (labels[prev_row_offset + x - 1] != min_label) {
                        merge_labels(equiv, min_label, labels[prev_row_offset + x - 1]);
                    }
                }
                
                // Top-right neighbor
                if (x < width - 1 && buf[prev_buf_offset + x + 1] == val && labels[prev_row_offset + x + 1] != 0) {
                    if (min_label == 0) {
                        min_label = labels[prev_row_offset + x + 1];
                    } else if (labels[prev_row_offset + x + 1] != min_label) {
                        merge_labels(equiv, min_label, labels[prev_row_offset + x + 1]);
                    }
                }
            }
            
            // Assign label
            if (min_label == 0) {
                // New component
                if (next_label >= ccl->max_labels) {
                    labels[row_offset + x] = next_label - 1;
                } else {
                    labels[row_offset + x] = next_label++;
                }
            } else {
                labels[row_offset + x] = min_label;
            }
        }
    }
    
    ccl->num_labels = next_label;
    
    // Pass 2: Resolve equivalences and compute component sizes
    memset(ccl->comp_size, 0, ccl->max_labels * sizeof(uint32_t));
    
    // Flatten equivalence table
    for (uint32_t i = 1; i < next_label; i++) {
        equiv[i] = find_root(equiv, i);
    }
    
    // Count component sizes
    for (int y = 0; y < height; y++) {
        int row_offset = y * width;
        for (int x = 0; x < width; x++) {
            uint32_t label = labels[row_offset + x];
            if (label != 0) {
                labels[row_offset + x] = equiv[label]; // Update to root label
                ccl->comp_size[equiv[label]]++;
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
