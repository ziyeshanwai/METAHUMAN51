// Copyright Epic Games, Inc. All Rights Reserved.

/*
    Implementation details of color conversion algorithms.

    This is an internal header, and it is not meant for API use.
    It is only included through carbon/data/Pixel.h file.
*/

#pragma once

#ifndef _CARBON_PIXEL_IS_INCLUDED
    #error "This is an implementation header of <carbon/data/Pixel.h> and is not supposed to be included on its own."
#endif

namespace epic::carbon::detail {

template<class T, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
T sRGBGamut(T x) {
    return (x < T(0.0031308)) ? T(12.92) * x : T(1.055) * std::pow(x, T(1.0 / 2.4)) - T(0.055);
}

template<class T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
T sRGBGamut(T x) {
    double xf = static_cast<double>(x) / std::numeric_limits<T>::max();
    return static_cast<T>(std::floor(sRGBGamut(xf) * std::numeric_limits<T>::max()));
}

template<class T, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
T Linearize(T x) {
    return (x < T(0.04045)) ? x / T(12.92) : std::pow((x + T(0.055)) / T(1.055), T(2.4));
}

template<class T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
T Linearize(T x) {
    double xf = static_cast<double>(x) / std::numeric_limits<T>::max();
    return static_cast<T>(std::ceil(Linearize(xf) * std::numeric_limits<T>::max()));
}

template<class T>
void CalcMinMaxRange(T r, T g, T b, T& M, T& m, T& c) {
    static_assert(std::is_floating_point<T>::value,
                  "DEVELOPER ERROR: epic::carbon::detail::CalcMinMaxRange given non-float RGB.");
    M = std::max(r, std::max(g, b));
    m = std::min(r, std::min(g, b));
    c = M - m;
}

template<class T>
float CalcHue(T r, T g, T b, T M, T c) {
    static_assert(std::is_floating_point<T>::value, "DEVELOPER ERROR: epic::carbon::detail::CalcHue given non-float RGB.");
    float H{};
    if (c == 0.f) {
        // undefined actually, but
        return H;
    } else if (M == r) {
        H = std::fmod(((g - b) / c), T(6));
    } else if (M == g) {
        H = (b - r) / c + T(2);
    } else if (M == b) {
        H = (r - g) / c + T(4);
    }
    H = T(60) * H;
    if (H < 0.f) {
        H += 360.0f;
    }

    return H;
}

template<class U, class T, typename std::enable_if<std::is_same<U, T>::value, int>::type = 0>
U rgba_cast(T in) {
    return in;
}

template<class U, class T, typename std::enable_if<std::is_integral<T>::value && std::is_floating_point<U>::value, int>::type = 0>
U rgba_cast(T in) {
    return static_cast<U>(in) / static_cast<U>(std::numeric_limits<T>::max());
}

template<class U, class T, typename std::enable_if<std::is_floating_point<T>::value && std::is_integral<U>::value, int>::type = 0>
U rgba_cast(T in) {
    return static_cast<U>(in * static_cast<T>(std::numeric_limits<U>::max()));
}

template<class U, class T,
         typename std::enable_if<!std::is_same<T, U>::value &&
                                 std::is_floating_point<T>::value &&
                                 std::is_floating_point<U>::value, int>::type = 0>
U rgba_cast(T in) {
    return static_cast<U>(in);
}

template<class U, class T,
         typename std::enable_if<!std::is_same<T, U>::value &&
                                 std::is_integral<T>::value &&
                                 std::is_integral<U>::value, int>::type = 0>
U rgba_cast(T in) {
    constexpr float tmax = static_cast<float>(std::numeric_limits<T>::max());
    constexpr float umax = static_cast<float>(std::numeric_limits<U>::max());
    float nvalue = static_cast<float>(in) / tmax;
    return static_cast<U>(nvalue * umax);
}

template<class U, class T, typename std::enable_if<std::is_same<T, U>::value, int>::type = 0>
U hue_cast(T h) {
    return h;
}

template<class U, class T,
         typename std::enable_if<std::is_same<T, std::uint8_t>::value &&
                                 !std::is_same<U, std::uint8_t>::value, int>::type = 0>
U hue_cast(T h) {
    return static_cast<U>(static_cast<float>(h) * 2.f);
}

template<class U, class T,
         typename std::enable_if<!std::is_same<T, std::uint8_t>::value &&
                                 std::is_same<U, std::uint8_t>::value, int>::type = 0>
U hue_cast(T h) {
    return static_cast<U>(static_cast<float>(h) * 0.5f);
}

template<class U, class T,
         typename std::enable_if<!std::is_same<T, std::uint8_t>::value &&
                                 !std::is_same<U, std::uint8_t>::value &&
                                 !std::is_same<T, U>::value, int>::type = 0>
U hue_cast(T h) {
    return static_cast<U>(h);
}

template<class U, class T, typename std::enable_if<std::is_same<T, U>::value, int>::type = 0>
U svl_cast(T v) {
    return v;
}

template<class U, class T,
         typename std::enable_if<std::is_integral<T>::value && std::is_floating_point<U>::value, int>::type = 0>
U svl_cast(T v) {
    return static_cast<U>(v) / U(100);
}

template<class U, class T,
         typename std::enable_if<std::is_floating_point<T>::value && std::is_integral<U>::value, int>::type = 0>
U svl_cast(T v) {
    return static_cast<U>(v * T(100));
}

template<class U, class T,
         typename std::enable_if<!std::is_same<T, U>::value &&
                                 std::is_floating_point<T>::value &&
                                 std::is_floating_point<U>::value, int>::type = 0>
U svl_cast(T v) {
    return static_cast<U>(v);
}

template<class U, class T,
         typename std::enable_if<!std::is_same<T, U>::value &&
                                 std::is_integral<T>::value &&
                                 std::is_integral<U>::value, int>::type = 0>
U svl_cast(T v) {
    return static_cast<U>(v);
}

// helper structure to coordinate rgb packing depending on the hue values
// without doing range checking
template<class T>
struct HSV_L_value_stacking {
    using packing_func = std::function<void (T c, T x, T& r, T& g, T& b)>;

    static void pack_01(T c, T x, T& r, T& g, T& b) {
        r = c;
        g = x;
        b = T(0);
    }

    static void pack_12(T c, T x, T& r, T& g, T& b) {
        r = x;
        g = c;
        b = T(0);
    }

    static void pack_23(T c, T x, T& r, T& g, T& b) {
        r = T(0);
        g = c;
        b = x;
    }

    static void pack_34(T c, T x, T& r, T& g, T& b) {
        r = T(0);
        g = x;
        b = c;
    }

    static void pack_45(T c, T x, T& r, T& g, T& b) {
        r = x;
        g = T(0);
        b = c;
    }

    static void pack_56(T c, T x, T& r, T& g, T& b) {
        r = c;
        g = T(0);
        b = x;
    }

    static packing_func select_function(int H) {
        CARBON_PRECONDITION(H >= 0 && H < 6, "Invalid Hue value range - should be 0-360.");

        static packing_func funcs[] = {
            & pack_01,
            & pack_12,
            & pack_23,
            & pack_34,
            & pack_45,
            & pack_56
        };
        return funcs[H];
    }

};

// repeated, per element chunk of the algorithm (below the version with RGB and HSV API)
template<class T>
void RGB2HSV_impl(T r, T g, T b, T& h, T& s, T& v) {
    // DEV NOTE
    // This static assert should never go off on the client side. But just
    // in case we messed up and did let it slide, it should be plainly said it
    // is us, devs, that messed up, and that it was NOT the client input.
    //
    // Explanation - this is a private (detail) function, and it is called only
    // by RGB<T>::operator HSV<T>() cast operator. In case it was miscalled, it
    // has to be the issue of whomever built that cast operator hasn't called this
    // function properly (as it was designed to be called only with floats).
    //
    // This also implies to other similar static_asserts in other _impl functions.

    static_assert(std::is_floating_point<T>::value,
                  "DEVELOPER ERROR: epic::carbon::detail::RGB2HSV_impl called with non-floating point type.");

    T M{}, m{}, c{};
    detail::CalcMinMaxRange(r, g, b, M, m, c);
    h = detail::CalcHue(r, g, b, M, c);
    v = M;
    s = v ? c / v : T(0);
}

template<class T>
void HSV2RGB_impl(T h, T s, T v, T& r, T& g, T& b) {

    static_assert(std::is_floating_point<T>::value,
                  "DEVELOPER ERROR: epic::carbon::detail::HSV2RGB_impl called with non-floating point type.");
    T c = v * s;
    T h_ = h / T(60);
    T x = c * (1 - std::abs(std::fmod(h_, T(2)) - 1));

    auto packf = HSV_L_value_stacking<T>::select_function(static_cast<int>(floor(h_)));
    packf(c, x, r, g, b);

    T m = v - c;

    r += m;
    g += m;
    b += m;
}

template<class T>
void RGB2HSL_impl(T r, T g, T b, T& h, T& s, T& l) {
    static_assert(std::is_floating_point<T>::value,
                  "DEVELOPER ERROR: epic::carbon::detail::RGB2HSL_impl called with non-floating point type.");
    float M{}, m{}, c{};
    detail::CalcMinMaxRange(r, g, b, M, m, c);
    h = detail::CalcHue(r, g, b, M, c);
    l = M - c * 0.5f;
    s = l == 1.0f || l == 0.0f ? 0.0f : c / (1.0f - std::fabs(2.0f * l - 1.0f));
}

template<class T>
void HSL2RGB_impl(T h, T s, T l, T& r, T& g, T& b) {

    static_assert(std::is_floating_point<T>::value,
                  "DEVELOPER ERROR: epic::carbon::detail::HSL2RGB_impl called with non-floating point type.");
    T c = (1.f - std::fabs(2.f * l - 1.f)) * s;
    T h_ = h / T(60);
    T x = c * (1 - std::abs(std::fmod(h_, T(2)) - 1));

    auto packf = HSV_L_value_stacking<T>::select_function(static_cast<int>(floor(h_)));
    packf(c, x, r, g, b);

    T m = l - c * 0.5f;

    r += m;
    g += m;
    b += m;
}

template<class U, class T, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
HSV<U> RGB2HSV(const RGB<T>& rgb) {
    T h, s, v;
    RGB2HSV_impl(rgb.r(), rgb.g(), rgb.b(), h, s, v);
    return {hue_cast<U>(h), svl_cast<U>(s), svl_cast<U>(v)};
}

template<class U, class T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
HSV<U> RGB2HSV(const RGB<T>& rgb) {
    auto const frgb = static_cast<RGB<float> >(rgb);

    float h, s, v;
    RGB2HSV_impl(frgb.r(), frgb.g(), frgb.b(), h, s, v);

    T h_ = T(std::round(sizeof(T) == 1 ? h * 0.5f : h));
    T s_ = T(std::round(s * 100.f));
    T v_ = T(std::round(v * 100.f));

    return {hue_cast<U>(h_), svl_cast<U>(s_), svl_cast<U>(v_)};
}

template<class U, class T, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
RGB<U> HSV2RGB(const HSV<T>& hsv) {
    T r{}, g{}, b{};

    detail::HSV2RGB_impl(hsv.h(), hsv.s(), hsv.v(), r, g, b);

    return {rgba_cast<U>(r), rgba_cast<U>(g), rgba_cast<U>(b)};
}

template<class U, class T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
RGB<U> HSV2RGB(const HSV<T>& hsv) {
    float h_ = static_cast<float>(sizeof(T) == 1 ? hsv.h() * 2.f : hsv.h());
    float s_ = static_cast<float>(hsv.s()) / 100.f;
    float v_ = static_cast<float>(hsv.v()) / 100.f;

    float r{}, g{}, b{};

    detail::HSV2RGB_impl(h_, s_, v_, r, g, b);

    float tmax = static_cast<float>(std::numeric_limits<T>::max());

    return {
        rgba_cast<U>(static_cast<T>(std::round(r * tmax))),
        rgba_cast<U>(static_cast<T>(std::round(g * tmax))),
        rgba_cast<U>(static_cast<T>(std::round(b * tmax)))
    };
}

template<class U, class T, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
HSL<U> RGB2HSL(const RGB<T>& rgb) {
    T h, s, l;
    RGB2HSL_impl(rgb.r(), rgb.g(), rgb.b(), h, s, l);

    return {hue_cast<U>(h), svl_cast<U>(s), svl_cast<U>(l)};
}

template<class U, class T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
HSL<U> RGB2HSL(const RGB<T>& rgb) {
    auto const frgb = static_cast<RGB<float> >(rgb);

    float h, s, l;
    RGB2HSL_impl(frgb.r(), frgb.g(), frgb.b(), h, s, l);

    T h_ = T(std::round(sizeof(T) == 1 ? h * 0.5f : h));
    T s_ = T(std::round(s * 100.f));
    T l_ = T(std::round(l * 100.f));

    return {hue_cast<U>(h_), svl_cast<U>(s_), svl_cast<U>(l_)};
}

template<class U, class T, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
RGB<U> HSL2RGB(const HSL<T>& hsl) {
    T r{}, g{}, b{};

    detail::HSL2RGB_impl(hsl.h(), hsl.s(), hsl.l(), r, g, b);

    return {rgba_cast<U>(r), rgba_cast<U>(g), rgba_cast<U>(b)};
}

template<class U, class T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
RGB<U> HSL2RGB(const HSL<T>& hsl) {
    float h_ = static_cast<float>(sizeof(T) == 1 ? hsl.h() * 2.f : hsl.h());
    float s_ = static_cast<float>(hsl.s()) / 100.f;
    float l_ = static_cast<float>(hsl.l()) / 100.f;

    float r{}, g{}, b{};

    detail::HSL2RGB_impl(h_, s_, l_, r, g, b);

    float tmax = static_cast<float>(std::numeric_limits<T>::max());

    return {
        rgba_cast<U>(static_cast<T>(std::round(r * tmax))),
        rgba_cast<U>(static_cast<T>(std::round(g * tmax))),
        rgba_cast<U>(static_cast<T>(std::round(b * tmax))),
    };
}

template<class T, bool = std::is_integral<T>::value>
struct AlphaWhite;  // trait to help define full (white) value of alpha, depending on T

template<class T>
struct AlphaWhite<T, true> {
    static constexpr T value = std::numeric_limits<T>::max();
};

template<class T>
struct AlphaWhite<T, false> {
    static constexpr T value = T(1);
};

}  // namespace epic::carbon::detail
