/* Benchmark program for AprilTag detection performance
 * This program loads a test image and runs detection multiple times
 * to measure performance of the thresholding step.
 */

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

#include "apriltag.h"
#include "tag36h11.h"
#include "common/image_u8.h"
#include "common/pjpeg.h"
#include "common/pnm.h"
#include "common/zarray.h"

static int str_ends_with(const char *str, const char *suffix)
{
    if (!str || !suffix)
        return 0;
    size_t lenstr = strlen(str);
    size_t lensuffix = strlen(suffix);
    if (lensuffix > lenstr)
        return 0;
    return strncmp(str + lenstr - lensuffix, suffix, lensuffix) == 0;
}

double timespec_to_seconds(struct timespec *ts)
{
    return ts->tv_sec + ts->tv_nsec / 1e9;
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        printf("Usage: %s <image_path> [iterations]\n", argv[0]);
        printf("Example: %s test/data/33369213973_9d9bb4cc96_c.jpg 100\n", argv[0]);
        return 1;
    }

    const char *image_path = argv[1];
    int iterations = 100;
    if (argc >= 3) {
        iterations = atoi(argv[2]);
    }

    printf("Benchmark Configuration:\n");
    printf("  Image: %s\n", image_path);
    printf("  Iterations: %d\n", iterations);
    printf("\n");

    // Load image
    image_u8_t *im = NULL;
    if (str_ends_with(image_path, "pnm") || str_ends_with(image_path, "PNM") ||
        str_ends_with(image_path, "pgm") || str_ends_with(image_path, "PGM")) {
        im = image_u8_create_from_pnm(image_path);
    } else if (str_ends_with(image_path, "jpg") || str_ends_with(image_path, "JPG")) {
        int err = 0;
        pjpeg_t *pjpeg = pjpeg_create_from_file(image_path, 0, &err);
        if (pjpeg == NULL) {
            printf("Failed to load JPEG: %s (error %d)\n", image_path, err);
            return 1;
        }
        im = pjpeg_to_u8_baseline(pjpeg);
        pjpeg_destroy(pjpeg);
    }

    if (im == NULL) {
        printf("Failed to load image: %s\n", image_path);
        return 1;
    }

    printf("Image loaded: %dx%d\n\n", im->width, im->height);

    // Create detector
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    
    // Configure detector
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;
    td->nthreads = 1;
    td->debug = 0;
    td->refine_edges = 1;

    // Warm-up run
    printf("Running warm-up detection...\n");
    zarray_t *detections = apriltag_detector_detect(td, im);
    printf("Warm-up complete. Detected %d tags.\n\n", zarray_size(detections));
    apriltag_detections_destroy(detections);

    // Benchmark
    printf("Running benchmark with %d iterations...\n", iterations);
    
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (int i = 0; i < iterations; i++) {
        detections = apriltag_detector_detect(td, im);
        apriltag_detections_destroy(detections);
    }

    clock_gettime(CLOCK_MONOTONIC, &end);

    double elapsed = timespec_to_seconds(&end) - timespec_to_seconds(&start);
    double avg_ms = (elapsed / iterations) * 1000.0;

    printf("\nBenchmark Results:\n");
    printf("  Total time: %.3f seconds\n", elapsed);
    printf("  Average per detection: %.3f ms\n", avg_ms);
    printf("  Detections per second: %.1f\n", iterations / elapsed);

    // Show time profile
    printf("\nTime Profile:\n");
    timeprofile_display(td->tp);

    // Cleanup
    image_u8_destroy(im);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}
