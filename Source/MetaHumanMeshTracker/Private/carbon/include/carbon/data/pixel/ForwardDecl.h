// Copyright Epic Games, Inc. All Rights Reserved.

/*
    Forward declaration of Pixel types.

    This is an internal header, and it is not meant for API use.
    It is only included through carbon/data/Pixel.h file.
*/


#pragma once

#ifndef _CARBON_PIXEL_IS_INCLUDED
    #error "This is an implementation header of <carbon/data/Pixel.h> and is not supposed to be included on its own."
#endif

namespace epic::carbon {

/**
    Generic Pixel type.
*/
template<class T, std::size_t size>
class Pixel;

/**
    Mono (grayscale) pixel specialization.
*/
template<class T>
class Mono;

/**
    Mono (grayscale) with alpha channel pixel specialization.
*/
template<class T>
class MonoAlpha;

/**
    RGB pixel specialization.
*/
template<class T>
class RGB;

/**
    RGB with alpha channel pixel specialization.
*/
template<class T>
class RGBA;

/**
    sRGB pixel specialization.
*/
template<class T>
class sRGB;

/**
    sRGB with alpha channel pixel specialization.
*/
template<class T>
class sRGBA;

/**
    HSV pixel specialization.
*/
template<class T>
class HSV;

/**
    HSL pixel specialization.
*/
template<class T>
class HSL;

/**
    Color space indicator, helpful as a template trait.
*/
enum class ColorSpace {
    Undefined,
    Mono,
    MonoAlpha,
    RGB,
    RGBA,
    sRGB,
    sRGBA,
    HSV,
    HSL
};
}
