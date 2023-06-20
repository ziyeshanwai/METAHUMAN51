// Copyright Epic Games, Inc. All Rights Reserved.

/*
    Various useful aliases defined for pixel templates.

    This is an internal header, and it is not meant for API use.
    It is only included through carbon/data/Pixel.h file.
*/


#pragma once

#ifndef _CARBON_PIXEL_IS_INCLUDED
    #error "This is an implementation header of <carbon/data/Pixel.h> and is not supposed to be included on its own."
#endif

namespace epic::carbon {
using Mono8 = Mono<std::uint8_t>;
using MonoAlpha8 = MonoAlpha<std::uint8_t>;
using RGB8 = RGB<std::uint8_t>;
using RGBA8 = RGBA<std::uint8_t>;
using sRGB8 = sRGB<std::uint8_t>;
using sRGBA8 = sRGBA<std::uint8_t>;
using HSV8 = HSV<std::uint8_t>;
using HSL8 = HSL<std::uint8_t>;

using MonoU16 = Mono<std::uint16_t>;
using MonoAlphaU16 = MonoAlpha<std::uint16_t>;
using RGBU16 = RGB<std::uint16_t>;
using RGBAU16 = RGBA<std::uint16_t>;
using sRGBU16 = sRGB<std::uint16_t>;
using sRGBAU16 = sRGBA<std::uint16_t>;
using HSVU16 = HSV<std::uint16_t>;
using HSLU16 = HSL<std::uint16_t>;

using MonoS16 = Mono<std::int16_t>;
using MonoAlphaS16 = MonoAlpha<std::int16_t>;
using RGBS16 = RGB<std::int16_t>;
using RGBAS16 = RGBA<std::int16_t>;
using sRGBS16 = sRGB<std::int16_t>;
using sRGBAS16 = sRGBA<std::int16_t>;
using HSVS16 = HSV<std::int16_t>;
using HSLS16 = HSL<std::int16_t>;

using MonoU32 = Mono<std::uint32_t>;
using MonoAlphaU32 = MonoAlpha<std::uint32_t>;
using RGBU32 = RGB<std::uint32_t>;
using RGBAU32 = RGBA<std::uint32_t>;
using sRGBU32 = sRGB<std::uint32_t>;
using sRGBAU32 = sRGBA<std::uint32_t>;
using HSVU32 = HSV<std::uint32_t>;
using HSLU32 = HSL<std::uint32_t>;

using MonoS32 = Mono<std::int32_t>;
using MonoAlphaS32 = MonoAlpha<std::int32_t>;
using RGBS32 = RGB<std::int32_t>;
using RGBAS32 = RGB<std::int32_t>;
using sRGBS32 = sRGB<std::int32_t>;
using sRGBAS32 = sRGB<std::int32_t>;
using HSVS32 = HSV<std::int32_t>;
using HSLS32 = HSL<std::int32_t>;

using MonoF = Mono<float>;
using MonoAlphaF = MonoAlpha<float>;
using RGBF = RGB<float>;
using RGBAF = RGBA<float>;
using sRGBF = sRGB<float>;
using sRGBAF = sRGBA<float>;
using HSVF = HSV<float>;
using HSLF = HSL<float>;

using MonoD = Mono<double>;
using MonoAlphaD = MonoAlpha<double>;
using RGBD = RGB<double>;
using RGBAD = RGBA<double>;
using sRGBD = sRGB<double>;
using sRGBAD = sRGBA<double>;
using HSVD = HSV<double>;
using HSLD = HSL<double>;


}
