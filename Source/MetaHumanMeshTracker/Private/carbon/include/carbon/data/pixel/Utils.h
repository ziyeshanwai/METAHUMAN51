// Copyright Epic Games, Inc. All Rights Reserved.

/*
    Various utilities for carbon::Pixel class.

    This is an internal header, and it is not meant for API use.
    It is only included through carbon/data/Pixel.h file.
*/
#pragma once

#ifndef _CARBON_PIXEL_IS_INCLUDED
    #error "This is an implementation header of <carbon/data/Pixel.h> and is not supposed to be included on its own."
#endif

namespace epic::carbon {

/**
    Initializes pixel with arbitrary constant.

    @param[in] value value to set.
    @return pixel with all veluse set to value.
*/
template<class T, typename std::enable_if<is_pixel<T>::value, int>::type = 0>
T Constant(typename PixelTraits<T>::value_type value) {
    T out;
    for (int i = 0; i < PixelTraits<T>::channels; ++i) {
        out[i] = value;
    }
    return out;
}

/**
    Constant initialization for scalar values.

    This functions is a template specialization to allow generic code,
    which accepts both scalar and pixel type.
*/
template<class T, typename std::enable_if<!is_pixel<T>::value, int>::type = 0>
T Constant(T v) {
    return v;
}

/**
    Initializes pixel with zeros

    @return pixel with all zeros set.
*/
template<class T>
T Zero() {
    return Constant<T>(0);
}

/**
    Initializes pixel with ones

    @return pixel with all ones set.
*/
template<class T>
T Ones() {
    return Constant<T>(1);
}

}
