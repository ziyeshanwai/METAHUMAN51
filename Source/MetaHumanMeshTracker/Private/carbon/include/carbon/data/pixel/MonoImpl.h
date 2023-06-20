// Copyright Epic Games, Inc. All Rights Reserved.

/*
    Mono (single-channel) pixel representation overload.

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
Mono<T>::operator Mono<U>() const {
    return detail::rgba_cast<U>(Value());
}

template<class T>
template<class U>
Mono<T>::operator MonoAlpha<U>() const {
    return {
        detail::rgba_cast<U>(Value()),
        detail::AlphaWhite<U>::value
    };
}

template<class T>
template<class U>
MonoAlpha<T>::operator MonoAlpha<U>() const {
    return {
        detail::rgba_cast<U>(Value()),
        detail::rgba_cast<U>(Alpha()),
    };
}

template<class T>
template<class U>
MonoAlpha<T>::operator Mono<U>() const {
    return {
        detail::rgba_cast<U>(Value())
    };
}

template<class T>
template<class U>
Mono<T>::operator RGB<U>() const {
    const U v = detail::rgba_cast<U>(Value());
    return {v, v, v};
}

template<class T>
template<class U>
Mono<T>::operator RGBA<U>() const {
    const U v = detail::rgba_cast<U>(Value());
    return {v, v, v, detail::AlphaWhite<U>::value};
}

template<class T>
template<class U>
MonoAlpha<T>::operator RGBA<U>() const {
    const U v = detail::rgba_cast<U>(Value());
    const U a = detail::rgba_cast<U>(Alpha());
    return {v, v, v, a};
}

}  // epic::carbon
