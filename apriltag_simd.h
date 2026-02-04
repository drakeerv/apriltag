/* SIMD utilities for AprilTag
 * Cross-platform SIMD operations supporting ARM NEON and x86 SSE2/AVX2
 * Copyright (C) 2024
 */

#ifndef _APRILTAG_SIMD_H_
#define _APRILTAG_SIMD_H_

#include <stdint.h>
#include <string.h>

// Detect platform and include appropriate headers
#if defined(__ARM_NEON) || defined(__aarch64__)
    #define APRILTAG_USE_NEON 1
    #include <arm_neon.h>
#elif defined(__SSE2__) || defined(__x86_64__) || defined(_M_X64) || defined(_M_IX86)
    #define APRILTAG_USE_SSE2 1
    #include <emmintrin.h>  // SSE2
    #ifdef __SSSE3__
        #include <tmmintrin.h>  // SSSE3
    #endif
#else
    #define APRILTAG_USE_SCALAR 1
#endif

// Define vector types
#ifdef APRILTAG_USE_NEON
    typedef uint8x16_t simd_u8x16_t;
    #define SIMD_VECTOR_SIZE 16
#elif APRILTAG_USE_SSE2
    typedef __m128i simd_u8x16_t;
    #define SIMD_VECTOR_SIZE 16
#else
    // Scalar fallback
    typedef struct {
        uint8_t data[16];
    } simd_u8x16_t;
    #define SIMD_VECTOR_SIZE 16
#endif

// Load 16 bytes from memory
static inline simd_u8x16_t simd_load_u8x16(const uint8_t *ptr)
{
#ifdef APRILTAG_USE_NEON
    return vld1q_u8(ptr);
#elif APRILTAG_USE_SSE2
    return _mm_loadu_si128((const __m128i*)ptr);
#else
    simd_u8x16_t result;
    memcpy(result.data, ptr, 16);
    return result;
#endif
}

// Store 16 bytes to memory
static inline void simd_store_u8x16(uint8_t *ptr, simd_u8x16_t v)
{
#ifdef APRILTAG_USE_NEON
    vst1q_u8(ptr, v);
#elif APRILTAG_USE_SSE2
    _mm_storeu_si128((__m128i*)ptr, v);
#else
    memcpy(ptr, v.data, 16);
#endif
}

// Create a vector with all elements set to the same value
static inline simd_u8x16_t simd_set1_u8(uint8_t value)
{
#ifdef APRILTAG_USE_NEON
    return vdupq_n_u8(value);
#elif APRILTAG_USE_SSE2
    return _mm_set1_epi8((char)value);
#else
    simd_u8x16_t result;
    for (int i = 0; i < 16; i++) {
        result.data[i] = value;
    }
    return result;
#endif
}

// Element-wise minimum
static inline simd_u8x16_t simd_min_u8(simd_u8x16_t a, simd_u8x16_t b)
{
#ifdef APRILTAG_USE_NEON
    return vminq_u8(a, b);
#elif APRILTAG_USE_SSE2
    return _mm_min_epu8(a, b);
#else
    simd_u8x16_t result;
    for (int i = 0; i < 16; i++) {
        result.data[i] = (a.data[i] < b.data[i]) ? a.data[i] : b.data[i];
    }
    return result;
#endif
}

// Element-wise maximum
static inline simd_u8x16_t simd_max_u8(simd_u8x16_t a, simd_u8x16_t b)
{
#ifdef APRILTAG_USE_NEON
    return vmaxq_u8(a, b);
#elif APRILTAG_USE_SSE2
    return _mm_max_epu8(a, b);
#else
    simd_u8x16_t result;
    for (int i = 0; i < 16; i++) {
        result.data[i] = (a.data[i] > b.data[i]) ? a.data[i] : b.data[i];
    }
    return result;
#endif
}

// Compare greater than (returns 0xFF for true, 0x00 for false)
static inline simd_u8x16_t simd_cmpgt_u8(simd_u8x16_t a, simd_u8x16_t b)
{
#ifdef APRILTAG_USE_NEON
    return vcgtq_u8(a, b);
#elif APRILTAG_USE_SSE2
    // SSE2 doesn't have unsigned byte comparison, so we use max trick
    // a > b is equivalent to max(a,b) == a && a != b
    // But simpler: use signed comparison with XOR to flip sign bit
    __m128i sign_bit = _mm_set1_epi8((char)0x80);
    __m128i a_signed = _mm_xor_si128(a, sign_bit);
    __m128i b_signed = _mm_xor_si128(b, sign_bit);
    return _mm_cmpgt_epi8(a_signed, b_signed);
#else
    simd_u8x16_t result;
    for (int i = 0; i < 16; i++) {
        result.data[i] = (a.data[i] > b.data[i]) ? 0xFF : 0x00;
    }
    return result;
#endif
}

// Horizontal minimum - reduce vector to single minimum value
static inline uint8_t simd_reduce_min_u8(simd_u8x16_t v)
{
#ifdef APRILTAG_USE_NEON
    // Use pairwise reduction
    uint8x8_t min8 = vmin_u8(vget_low_u8(v), vget_high_u8(v));
    uint8x8_t min4 = vpmin_u8(min8, min8);
    uint8x8_t min2 = vpmin_u8(min4, min4);
    uint8x8_t min1 = vpmin_u8(min2, min2);
    return vget_lane_u8(min1, 0);
#elif APRILTAG_USE_SSE2
    // Extract to array and find minimum
    uint8_t temp[16];
    _mm_storeu_si128((__m128i*)temp, v);
    uint8_t min_val = temp[0];
    for (int i = 1; i < 16; i++) {
        if (temp[i] < min_val) min_val = temp[i];
    }
    return min_val;
#else
    uint8_t min_val = v.data[0];
    for (int i = 1; i < 16; i++) {
        if (v.data[i] < min_val) min_val = v.data[i];
    }
    return min_val;
#endif
}

// Horizontal maximum - reduce vector to single maximum value
static inline uint8_t simd_reduce_max_u8(simd_u8x16_t v)
{
#ifdef APRILTAG_USE_NEON
    // Use pairwise reduction
    uint8x8_t max8 = vmax_u8(vget_low_u8(v), vget_high_u8(v));
    uint8x8_t max4 = vpmax_u8(max8, max8);
    uint8x8_t max2 = vpmax_u8(max4, max4);
    uint8x8_t max1 = vpmax_u8(max2, max2);
    return vget_lane_u8(max1, 0);
#elif APRILTAG_USE_SSE2
    // Extract to array and find maximum
    uint8_t temp[16];
    _mm_storeu_si128((__m128i*)temp, v);
    uint8_t max_val = temp[0];
    for (int i = 1; i < 16; i++) {
        if (temp[i] > max_val) max_val = temp[i];
    }
    return max_val;
#else
    uint8_t max_val = v.data[0];
    for (int i = 1; i < 16; i++) {
        if (v.data[i] > max_val) max_val = v.data[i];
    }
    return max_val;
#endif
}

#endif // _APRILTAG_SIMD_H_
