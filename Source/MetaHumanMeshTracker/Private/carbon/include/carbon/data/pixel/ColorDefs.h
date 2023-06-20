// Copyright Epic Games, Inc. All Rights Reserved.

/*
    Definitions of specialization of pixel that represent particular color spaces.

    This is an internal header, and it is not meant for API use.
    It is only included through carbon/data/Pixel.h file.
*/

#pragma once

#ifndef _CARBON_PIXEL_IS_INCLUDED
    #error "This is an implementation header of <carbon/data/Pixel.h> and is not supposed to be included on its own."
#endif

namespace epic::carbon {

template<class T>
class Mono final : public Pixel<T, 1> {
    public:
        Mono<T>()  noexcept : Pixel<T, 1>() {
        }

        template<class U, typename std::enable_if<std::is_convertible<U, T>::value, int>::type = 0>
        Mono<T>(U value) noexcept : Pixel<T, 1>(value) {
        }

        T Value() const {
            return this->m_elements[0];
        }

        T& Value() {
            return this->m_elements[0];
        }

        /**
            Implicit cast operator to scalar value.

            This is only valid for Mono i.e. single-channel pixels.

            @return pixel value.
        */
        explicit operator T() const {
            return this->Value();
        }

        /// Adding alpha channel to a grayscale pixel.
        template<class U>
        explicit operator MonoAlpha<U>() const;

        // Data type conversion

        template<class U>
        explicit operator Mono<U>() const;

        // Color conversions

        template<class U>
        explicit operator RGB<U>() const;

        template<class U>
        explicit operator RGBA<U>() const;
};

template<class T>
class MonoAlpha final : public Pixel<T, 2> {
    public:
        MonoAlpha<T>()  noexcept : Pixel<T, 2>() {
        }

        template<class U, typename std::enable_if<std::is_convertible<U, T>::value, int>::type = 0>
        MonoAlpha<T>(U value, U alpha) noexcept : Pixel<T, 2>(value, alpha) {
        }

        T Value() const {
            return this->m_elements[0];
        }

        T Alpha() const {
            return this->m_elements[1];
        }

        T& Value() {
            return this->m_elements[0];
        }

        T& Alpha() {
            return this->m_elements[1];
        }

        template<class U>
        explicit operator MonoAlpha<U>() const;

        /// Strip alpha channel.
        template<class U>
        explicit operator Mono<U>() const;

        template<class U>
        explicit operator RGBA<U>() const;

};

// Color class definitions

#define IMPL_TRIPLET_GETTERS(x0, x1, x2) \
    T x0() const { \
        return this->m_elements[0]; \
    } \
    T x1() const { \
        return this->m_elements[1]; \
    } \
    T x2() const { \
        return this->m_elements[2]; \
    } \
    T& x0() { \
        return this->m_elements[0]; \
    } \
    T& x1() { \
        return this->m_elements[1]; \
    } \
    T& x2() { \
        return this->m_elements[2]; \
    }

template<class T>
class RGB final : public Pixel<T, 3> {
    public:
        RGB<T>()  noexcept : Pixel<T, 3>() {
        }

        template<class U, typename std::enable_if<std::is_convertible<U, T>::value, int>::type = 0>
        RGB<T>(U r, U g, U b) noexcept : Pixel<T, 3>(r, g, b) {
        }

        // convenience getters / setters
        IMPL_TRIPLET_GETTERS(r, g, b)

        template<class U>
        explicit operator RGB<U>() const;

        template<class U>
        explicit operator RGBA<U>() const;

        template<class U>
        explicit operator Mono<U>() const;

        template<class U>
        explicit operator sRGB<U>() const;

        template<class U>
        explicit operator HSV<U>() const;

        template<class U>
        explicit operator HSL<U>() const;


};

template<class T>
class RGBA final : public Pixel<T, 4> {
    public:
        RGBA<T>()  noexcept : Pixel<T, 4>() {
        }

        template<class U, typename std::enable_if<std::is_convertible<U, T>::value, int>::type = 0>
        RGBA<T>(U r, U g, U b, U a) noexcept : Pixel<T, 4>(r, g, b, a) {
        }

        // convenience getters / setters
        IMPL_TRIPLET_GETTERS(r, g, b)

        T Alpha() const {
            return this->m_elements[3];
        }

        T& Alpha() {
            return this->m_elements[3];
        }

        template<class U>
        explicit operator RGBA<U>() const;

        template<class U>
        explicit operator RGB<U>() const;

        template<class U>
        explicit operator MonoAlpha<U>() const;

        template<class U>
        explicit operator Mono<U>() const;

        template<class U>
        explicit operator sRGBA<U>() const;
};

template<class T>
class sRGB final : public Pixel<T, 3> {
    public:
        sRGB<T>()  noexcept : Pixel<T, 3>() {
        }

        template<class U, typename std::enable_if<std::is_convertible<U, T>::value, int>::type = 0>
        sRGB<T>(U r, U g, U b) noexcept : Pixel<T, 3>(r, g, b) {
        }

        // convenience getters / setters
        IMPL_TRIPLET_GETTERS(r, g, b)

        template<class U>
        explicit operator sRGB<U>() const;

        template<class U>
        explicit operator sRGBA<U>() const;

        template<class U>
        explicit operator RGB<U>() const;


};

template<class T>
class sRGBA final : public Pixel<T, 4> {
    public:
        sRGBA<T>()  noexcept : Pixel<T, 4>() {
        }

        template<class U, typename std::enable_if<std::is_convertible<U, T>::value, int>::type = 0>
        sRGBA<T>(U r, U g, U b, U a) noexcept : Pixel<T, 4>(r, g, b, a) {
        }

        // convenience getters / setters
        IMPL_TRIPLET_GETTERS(r, g, b)

        T Alpha() const {
            return this->m_elements[3];
        }

        T& Alpha() {
            return this->m_elements[3];
        }

        template<class U>
        explicit operator sRGBA<U>() const;

        template<class U>
        explicit operator sRGB<U>() const;

        template<class U>
        explicit operator RGBA<U>() const;
};


template<class T>
class HSV final : public Pixel<T, 3> {

    public:
        HSV<T>()  noexcept : Pixel<T, 3>() {
        }

        template<class U,
                 typename std::enable_if<std::is_convertible<U, T>::value && std::is_integral<T>::value, int>::type = 0>
        HSV<T>(U h, U s, U v) : Pixel<T, 3>(h, s, v) {
            const T hmax = sizeof(T) == 1 ? T(180) : T(360);
            CARBON_PRECONDITION(h >= T(0) && h < hmax, "Given hue is out of range [0-{})", hmax);
            CARBON_PRECONDITION(s >= T(0) && s <= T(100), "Given value is out of range [0-100]");
            CARBON_PRECONDITION(v >= T(0) && v <= T(100), "Given saturation is out of range [0-100]");
        }

        template<class U,
                 typename std::enable_if<std::is_convertible<U, T>::value && std::is_floating_point<T>::value, int>::type = 0>
        HSV<T>(U h, U s, U v) : Pixel<T, 3>(h, s, v) {
            CARBON_PRECONDITION(h >= T(0) && h < T(360), "Given hue is out of range [0-360)");
            CARBON_PRECONDITION(s >= T(0) && s <= T(1), "Given value is out of range [0-1]");
            CARBON_PRECONDITION(v >= T(0) && v <= T(1), "Given saturation is out of range [0-1]");
        }

        // convenience getters / setters
        IMPL_TRIPLET_GETTERS(h, s, v)

        template<class U>
        explicit operator HSV<U>() const;

        template<class U>
        explicit operator RGB<U>() const;


};

template<class T>
class HSL final : public Pixel<T, 3> {
    public:
        HSL<T>()  noexcept : Pixel<T, 3>() {
        }

        template<class U, typename std::enable_if<std::is_convertible<U, T>::value && std::is_integral<T>::value, int>::type = 0>
        HSL<T>(U h, U s, U l) : Pixel<T, 3>(h, s, l) {
            const T hmax = sizeof(T) == 1 ? T(180) : T(360);
            CARBON_PRECONDITION(h >= T(0) && h < hmax, "Given hue is out of range [0-{})", hmax);
            CARBON_PRECONDITION(s >= T(0) && s <= T(100), "Given value is out of range [0-100]");
            CARBON_PRECONDITION(l >= T(0) && l <= T(100), "Given lightness is out of range [0-100]");
        }

        template<class U,
                 typename std::enable_if<std::is_convertible<U, T>::value && std::is_floating_point<T>::value, int>::type = 0>
        HSL<T>(U h, U s, U l) : Pixel<T, 3>(h, s, l) {
            CARBON_PRECONDITION(h >= T(0) && h < T(360), "Given hue is out of range [0-360)");
            CARBON_PRECONDITION(s >= T(0) && s <= T(1), "Given value is out of range [0-1]");
            CARBON_PRECONDITION(l >= T(0) && l <= T(1), "Given lightness is out of range [0-1]");
        }

        // convenience getters / setters
        IMPL_TRIPLET_GETTERS(h, s, l)

        template<class U>
        explicit operator HSL<U>() const;

        template<class U>
        explicit operator RGB<U>() const;
};


#undef IMPL_TRIPLET_GETTERS

}
