// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <immintrin.h>

namespace epic::nls {

#if defined CARBON_ENABLE_SSE && CARBON_ENABLE_SSE
//! \return value[0] + value[1] + value[2] + value[3]
inline float HorizontalSum(__m128 value) {
    const __m128 perm1 = _mm_movehdup_ps(value);
    const __m128 sum1 = _mm_add_ps(value, perm1);
    const __m128 perm2 = _mm_movehl_ps(perm1, sum1);
    const __m128 sum2 = _mm_add_ss(sum1, perm2);
    return _mm_cvtss_f32(sum2);
}

inline __m128i mul_epi32_sse2(const __m128i& a, const __m128i& b)
{
#if defined(__clang__) && !defined(__SSE4_1__)
	__m128i tmp1 = _mm_mul_epu32(a, b); /* mul 2,0*/
	__m128i tmp2 = _mm_mul_epu32(_mm_srli_si128(a, 4), _mm_srli_si128(b, 4)); /* mul 3,1 */
	return _mm_unpacklo_epi32(_mm_shuffle_epi32(tmp1, _MM_SHUFFLE(0, 0, 2, 0)), _mm_shuffle_epi32(tmp2, _MM_SHUFFLE(0, 0, 2, 0))); /* shuffle results to [63..0] and pack */
#else
	return _mm_mullo_epi32(a, b);
#endif
}
#endif

#if defined CARBON_ENABLE_AVX && CARBON_ENABLE_AVX
//! \return value[0] + value[1] + ... + value[7]
inline float HorizontalSum(__m256 value) {
    const __m128 first  = _mm256_castps256_ps128(value);
    const __m128 second = _mm256_extractf128_ps(value, 1);
    return HorizontalSum(_mm_add_ps(first, second));
}
#endif

} // namespace epic::nls
