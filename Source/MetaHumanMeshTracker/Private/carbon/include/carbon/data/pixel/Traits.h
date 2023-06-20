// Copyright Epic Games, Inc. All Rights Reserved.

/*
    Various trait implementation for carbon::Pixel class.

    This is an internal header, and it is not meant for API use.
    It is only included through carbon/data/Pixel.h file.
*/
#pragma once

#ifndef _CARBON_PIXEL_IS_INCLUDED
    #error "This is an implementation header of <carbon/data/Pixel.h> and is not supposed to be included on its own."
#endif

namespace epic::carbon {

template<class T>
struct EPIC_CARBON_API is_pixel {
    private:
        using yes = char;
        using no = long;

        template<typename U>
        static yes Check(typename U::pixel_type*);

        template<typename U>
        static no Check(...);

    public:
        static constexpr bool value = sizeof(Check<T>(0)) == sizeof(yes);
};
template<class T>
struct PixelTraits {
    using value_type = T;

    template<class U>
    using other_type = U;

    constexpr static std::size_t channels = 1;
    constexpr static std::size_t depth = sizeof(T);
    constexpr static ColorSpace color_space = ColorSpace::Undefined;
};

template<class T, std::size_t N>
struct PixelTraits<Pixel<T, N> > {
    using value_type = T;

    template<class U>
    using other_type = Pixel<U, N>;

    constexpr static std::size_t channels = N;
    constexpr static std::size_t depth = sizeof(T);
    constexpr static ColorSpace color_space = ColorSpace::Undefined;
};

template<class T>
struct PixelTraits<Mono<T> > {
    using value_type = T;

    template<class U>
    using other_type = Mono<U>;

    constexpr static std::size_t channels = 1;
    constexpr static std::size_t depth = sizeof(T);
    constexpr static ColorSpace color_space = ColorSpace::Mono;
};

template<class T>
struct PixelTraits<MonoAlpha<T> > {
    using value_type = T;

    template<class U>
    using other_type = MonoAlpha<U>;

    constexpr static std::size_t channels = 2;
    constexpr static std::size_t depth = sizeof(T);
    constexpr static ColorSpace color_space = ColorSpace::MonoAlpha;
};

template<class T>
struct PixelTraits<RGB<T> > {
    using value_type = T;

    template<class U>
    using other_type = RGB<U>;

    constexpr static std::size_t channels = 3;
    constexpr static std::size_t depth = sizeof(T);
    constexpr static ColorSpace color_space = ColorSpace::RGB;
};

template<class T>
struct PixelTraits<sRGB<T> > {
    using value_type = T;

    template<class U>
    using other_type = sRGB<U>;

    constexpr static std::size_t channels = 3;
    constexpr static std::size_t depth = sizeof(T);
    constexpr static ColorSpace color_space = ColorSpace::sRGB;
};

template<class T>
struct PixelTraits<RGBA<T> > {
    using value_type = T;

    template<class U>
    using other_type = RGBA<U>;

    constexpr static std::size_t channels = 4;
    constexpr static std::size_t depth = sizeof(T);
    constexpr static ColorSpace color_space = ColorSpace::RGBA;
};

template<class T>
struct PixelTraits<sRGBA<T> > {
    using value_type = T;

    template<class U>
    using other_type = sRGBA<U>;

    constexpr static std::size_t channels = 4;
    constexpr static std::size_t depth = sizeof(T);
    constexpr static ColorSpace color_space = ColorSpace::sRGBA;
};

template<class T>
struct PixelTraits<HSV<T> > {
    using value_type = T;

    template<class U>
    using other_type = HSV<U>;

    constexpr static std::size_t channels = 3;
    constexpr static std::size_t depth = sizeof(T);
    constexpr static ColorSpace color_space = ColorSpace::HSV;
};

template<class T>
struct PixelTraits<HSL<T> > {
    using value_type = T;

    template<class U>
    using other_type = HSL<U>;

    constexpr static std::size_t channels = 3;
    constexpr static std::size_t depth = sizeof(T);
    constexpr static ColorSpace color_space = ColorSpace::HSL;
};

/**
    Color / type conversion trait.

    Offer bool flag value for any pair of pixel types, to indicate whether conversion
    from one to another is implemented.

    This trait is important for compile time error messages. If not used, and
    invalid conversion is entered and compiled, compiled messages may be quite
    confusing (invalid / unimplemented code invocation).

    Conversions that are implemented are:
    Mono       -->  MonoAlpha
    Mono       -->  RGB
    MonoAlpha  -->  Mono
    MonoAlpha  -->  RGBA
    RGB        -->  RGBA
    RGB        -->  Mono
    RGB        -->  sRGB
    RGB        -->  HSV
    RGB        -->  HSL
    RGBA       -->  RGB
    RGBA       -->  MonoAlpha
    RGBA       -->  Mono
    RGBA       -->  sRGBA
    sRGB       -->  sRGBA
    sRGB       -->  RGB
    HSV        -->  RGB
    HSL        -->  RGB
*/
template<class U, class T, typename X = void>
struct is_pixel_convertible : std::false_type {};

// Color space conversion

#define is_pixel_convertible_gen(Pix1, Pix2) \
    template<class T, class U> \
    struct is_pixel_convertible< \
        Pix1<T>, \
        Pix2<U>, \
        typename std::enable_if<std::is_convertible<T, U>::value>::type> : std::true_type { \
    };

#define is_pixel_type_convertible_gen(Pix) \
    template<class T, class U> \
    struct is_pixel_convertible< \
        Pix<T>, \
        Pix<U>, \
        typename std::enable_if<std::is_convertible<T, U>::value>::type> : std::true_type { \
    };


is_pixel_convertible_gen(Mono, MonoAlpha)
is_pixel_convertible_gen(Mono, RGB)
is_pixel_convertible_gen(Mono, RGBA)
is_pixel_convertible_gen(MonoAlpha, Mono)
is_pixel_convertible_gen(MonoAlpha, RGBA)
is_pixel_convertible_gen(RGB, RGBA)
is_pixel_convertible_gen(RGB, Mono)
is_pixel_convertible_gen(RGB, sRGB)
is_pixel_convertible_gen(RGB, HSV)
is_pixel_convertible_gen(RGB, HSL)
is_pixel_convertible_gen(RGBA, RGB)
is_pixel_convertible_gen(RGBA, MonoAlpha)
is_pixel_convertible_gen(RGBA, Mono)
is_pixel_convertible_gen(RGBA, sRGBA)
is_pixel_convertible_gen(sRGB, sRGBA)
is_pixel_convertible_gen(sRGB, RGB)
is_pixel_convertible_gen(sRGBA, sRGB)
is_pixel_convertible_gen(sRGBA, RGBA)
is_pixel_convertible_gen(HSV, RGB)
is_pixel_convertible_gen(HSL, RGB)

is_pixel_type_convertible_gen(Mono)
is_pixel_type_convertible_gen(MonoAlpha)
is_pixel_type_convertible_gen(RGB)
is_pixel_type_convertible_gen(sRGB)
is_pixel_type_convertible_gen(RGBA)
is_pixel_type_convertible_gen(sRGBA)
is_pixel_type_convertible_gen(HSV)
is_pixel_type_convertible_gen(HSL)

#undef is_pixel_convertible_gen
#undef is_pixel_type_convertible_gen

}  // namespace epic::carbon
