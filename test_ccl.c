#include <stdio.h>
#include "apriltag.h"
#include "tag36h11.h"
#include "common/image_u8.h"
#include "common/pnm.h"
#include "common/ccl.h"

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <image>\n", argv[0]);
        return 1;
    }

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    image_u8_t *im = image_u8_create_from_pnm(argv[1]);
    if (!im) {
        fprintf(stderr, "Failed to load image\n");
        return 1;
    }

    printf("Image: %dx%d\n", im->width, im->height);
    
    // Test CCL directly
    ccl_component_t *ccl = ccl_create(im->width, im->height);
    printf("CCL created\n");
    
    uint32_t num_components = ccl_process(ccl, im);
    printf("CCL processed: %d components found\n", num_components);
    
    // Test get_label
    printf("Testing get_label...\n");
    for (int y = 0; y < 10 && y < im->height; y++) {
        for (int x = 0; x < 10 && x < im->width; x++) {
            uint32_t label = ccl_get_label(ccl, x, y);
            if (label > 0) {
                uint32_t rep = ccl_get_representative(ccl, label);
                uint32_t size = ccl_get_component_size(ccl, rep);
                printf("  (%d,%d): label=%u, rep=%u, size=%u\n", x, y, label, rep, size);
            }
        }
    }
    
    printf("Cleaning up...\n");
    ccl_destroy(ccl);
    image_u8_destroy(im);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    
    printf("Done!\n");
    return 0;
}
