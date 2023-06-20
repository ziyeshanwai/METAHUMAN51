// Copyright Epic Games, Inc. All Rights Reserved.

/*
    RGB pixel representation overload.

    This is an internal header, and it is not meant for API use.
    It is only included through carbon/data/Pixel.h file.
*/

#pragma once

#ifndef _CARBON_PIXEL_IS_INCLUDED
    #error "This is an implementation header of <carbon/data/Pixel.h> and is not supposed to be included on its own."
#endif

namespace epic::carbon {

template<class T>
template<class U>
RGB<T>::operator RGB<U>() const {
    return RGB<U>{
        detail::rgba_cast<U>(r()),
        detail::rgba_cast<U>(g()),
        detail::rgba_cast<U>(b())
    };
}

template<class T>
template<class U>
RGBA<T>::operator RGBA<U>() const {
    return {
        detail::rgba_cast<U>(r()),
        detail::rgba_cast<U>(g()),
        detail::rgba_cast<U>(b()),
        detail::rgba_cast<U>(Alpha()),
    };
}

template<class T>
template<class U>
sRGB<T>::operator sRGB<U>() const {
    return {
        detail::rgba_cast<U>(r()),
        detail::rgba_cast<U>(g()),
        detail::rgba_cast<U>(b())
    };
}

template<class T>
template<class U>
sRGBA<T>::operator sRGBA<U>() const {
    return {
        detail::rgba_cast<U>(r()),
        detail::rgba_cast<U>(g()),
        detail::rgba_cast<U>(b()),
        detail::rgba_cast<U>(Alpha()),
    };
}

template<class T>
template<class U>
RGB<T>::operator RGBA<U>() const {
    return {
        detail::rgba_cast<U>(r()),
        detail::rgba_cast<U>(g()),
        detail::rgba_cast<U>(b()),
        detail::AlphaWhite<U>::value
    };
}

template<class T>
template<class U>
RGBA<T>::operator RGB<U>() const {
    return {
        detail::rgba_cast<U>(r()),
        detail::rgba_cast<U>(g()),
        detail::rgba_cast<U>(b())
    };
}

template<class T>
template<class U>
sRGB<T>::operator sRGBA<U>() const {
    return {
        detail::rgba_cast<U>(r()),
        detail::rgba_cast<U>(g()),
        detail::rgba_cast<U>(b()),
        detail::AlphaWhite<U>::value
    };
}

template<class T>
template<class U>
sRGBA<T>::operator sRGB<U>() const {
    return {
        detail::rgba_cast<U>(r()),
        detail::rgba_cast<U>(g()),
        detail::rgba_cast<U>(b()),
    };
}

// Color conversion implementation

template<class T>
template<class U>
RGB<T>::operator Mono<U>() const {
    return {
        detail::rgba_cast<U>(T(0.2126f * static_cast<float>(r()) +
                               0.7152f * static_cast<float>(g()) +
                               0.0722f * static_cast<float>(b())))
    };
}

template<class T>
template<class U>
RGBA<T>::operator MonoAlpha<U>() const {
    U v = detail::rgba_cast<U>(T(0.2126f * static_cast<float>(r()) +
                                 0.7152f * static_cast<float>(g()) +
                                 0.0722f * static_cast<float>(b())));
    U a = detail::rgba_cast<U>(Alpha());

    return {v, a, };
}

template<class T>
template<class U>
RGBA<T>::operator Mono<U>() const {
    return {detail::rgba_cast<U>(T(0.2126f * static_cast<float>(r()) +
                                   0.7152f * static_cast<float>(g()) +
                                   0.0722f * static_cast<float>(b())))
    };
}

template<class T>
template<class U>
RGB<T>::operator sRGB<U>() const {
    return {
        detail::rgba_cast<U>(detail::sRGBGamut(r())),
        detail::rgba_cast<U>(detail::sRGBGamut(g())),
        detail::rgba_cast<U>(detail::sRGBGamut(b())),
    };
}

template<class T>
template<class U>
RGBA<T>::operator sRGBA<U>() const {
    return {
        detail::rgba_cast<U>(detail::sRGBGamut(r())),
        detail::rgba_cast<U>(detail::sRGBGamut(g())),
        detail::rgba_cast<U>(detail::sRGBGamut(b())),
        detail::rgba_cast<U>(Alpha())
    };
}

template<class T>
template<class U>
sRGB<T>::operator RGB<U>() const {
    return {
        detail::rgba_cast<U>(detail::Linearize(r())),
        detail::rgba_cast<U>(detail::Linearize(g())),
        detail::rgba_cast<U>(detail::Linearize(b()))
    };
}

template<class T>
template<class U>
sRGBA<T>::operator RGBA<U>() const {
    return {
        detail::rgba_cast<U>(detail::Linearize(r())),
        detail::rgba_cast<U>(detail::Linearize(g())),
        detail::rgba_cast<U>(detail::Linearize(b())),
        detail::rgba_cast<U>(Alpha())
    };
}

template<class T>
template<class U>
RGB<T>::operator HSV<U>() const {
    return detail::RGB2HSV<U>(*this);
}

template<class T>
template<class U>
RGB<T>::operator HSL<U>() const {
    return detail::RGB2HSL<U>(*this);
}

}  // namespace epic::carbon
