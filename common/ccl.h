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

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include "image_u8.h"

#ifdef __cplusplus
extern "C" {
#endif

// Two-Pass Connected Components Labeling (CCL)
// This replaces the Union-Find approach with a linear-scan algorithm
// that is cache-friendly and SIMD-optimizable.

typedef struct ccl_component ccl_component_t;

struct ccl_component {
    uint32_t *labels;      // Label for each pixel (w*h array)
    uint32_t *equiv;       // Label equivalence table
    uint32_t *comp_size;   // Size of each component
    uint32_t max_labels;   // Maximum number of labels allocated
    uint32_t num_labels;   // Actual number of labels used
    int width;
    int height;
};

// Create CCL structure
ccl_component_t *ccl_create(int width, int height);

// Destroy CCL structure
void ccl_destroy(ccl_component_t *ccl);

// Run two-pass CCL on thresholded image
// Returns the number of unique components found
uint32_t ccl_process(ccl_component_t *ccl, image_u8_t *threshim);

// Get the root label for a given label (after processing)
static inline uint32_t ccl_get_representative(ccl_component_t *ccl, uint32_t label) {
    // Safety check for invalid labels
    if (label >= ccl->max_labels) {
        return 0; // Return background for invalid label
    }
    while (label < ccl->max_labels && ccl->equiv[label] != label) {
        label = ccl->equiv[label];
    }
    return label;
}

// Get the size of a component
static inline uint32_t ccl_get_component_size(ccl_component_t *ccl, uint32_t label) {
    if (label == 0) return 0; // Background has no size
    uint32_t root = ccl_get_representative(ccl, label);
    return ccl->comp_size[root];
}

// Get the label for a specific pixel
static inline uint32_t ccl_get_label(ccl_component_t *ccl, int x, int y) {
    // Add bounds checking for safety
    if (x < 0 || x >= ccl->width || y < 0 || y >= ccl->height) {
        return 0; // Return background label for out-of-bounds access
    }
    return ccl->labels[y * ccl->width + x];
}

#ifdef __cplusplus
}
#endif
