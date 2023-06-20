// Copyright Epic Games, Inc. All Rights Reserved.

/*
    HSV/HSL pixel representation overload.

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
HSV<T>::operator HSV<U>() const {
    return {
        detail::hue_cast<U>(h()),
        detail::svl_cast<U>(s()),
        detail::svl_cast<U>(v()),
    };
}

template<class T>
template<class U>
HSL<T>::operator HSL<U>() const {
    return {
        detail::hue_cast<U>(h()),
        detail::svl_cast<U>(s()),
        detail::svl_cast<U>(l()),
    };
}

template<class T>
template<class U>
HSV<T>::operator RGB<U>() const {
    return detail::HSV2RGB<U>(*this);
}


template<class T>
template<class U>
HSL<T>::operator RGB<U>() const {
    return detail::HSL2RGB<U>(*this);
}

}  // namespace epic::carbon
