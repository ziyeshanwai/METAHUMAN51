// Copyright Epic Games, Inc. All Rights Reserved.


/**
    @defgroup Pixel

    Types and utilities to represent and handle colors.

    Module introduces types and utilities to represent and
    handle colors, and in the context of image structure, pixels.

    It is implemented in series of internal headers to better organize
    a header-only implementations of a bit more complex structure
    hierarchy. In a nutshell, it introduces following classes:

    - Pixel<T, N>: templated (almost-abstract) pixel type, base to concrete color type structures.
    - Mono<T>: Single-channel i.e. grayscale pixel representation
    - MonoAlpha<T>: Grayscale pixel representation with alpha channel.
    - RGB<T>: Classic linear RGB color representation.
    - RGBA<T>: Linear RGB with alpha channel.
    - sRGB<T>: Classic sRGB gamut color representation
    - sRGBA<T>: sRGB with alpha channel.
    - HSV<T>: Hue-Saturation-Value color space.
    - HSL<T>: Hue-Saturation-Lightness color space.

    Conversion between color types is implemented through casting operators.
    Therefore, for e.g., conversion between RGB and HSV is done using:

    @code
    RGB<float> rgb; // initialized elsewhere
    // ...
    HSV<float> hsv = static_cast<HSV<float>>(rgb);
    @endcode

    This is designed specifically to allow a simple templated color
    conversion for Image class:

    @code
    Image<RGB<float> > rgbImage;
    // ...
    Image<HSV<float> > hsvImage = rgbImage.ConvertTo<HSV<float> >();
    @endcode

    @see epic::carbon::Image::ConvertTo for more details.

    Conversions that are supported thus far are outlined in the table below.

    <div style="text-align:center">
    <table>
    <tr>
        <th></th>
        <th>Mono</th>
        <th>MonoAlpha</th>
        <th>RGB</th>
        <th>sRGB</th>
        <th>RGBA</th>
        <th>sRGBA</th>
        <th>HSV</th>
        <th>HSL</th>
    </tr>

    <tr>
        <th>Mono</th>

        <td>⬤</td>
        <td>⬤</td>
        <td>⬤</td>
        <td></td>
        <td></td>
        <td></td>
        <td></td>
        <td></td>
    </tr>

    <tr>
        <th>MonoAlpha</th>

        <td>⬤</td>
        <td>⬤</td>
        <td></td>
        <td></td>
        <td>⬤</td>
        <td></td>
        <td></td>
        <td></td>
    </tr>

    <tr>
        <th>RGB</th>

        <td>⬤</td>
        <td></td>
        <td>⬤</td>
        <td>⬤</td>
        <td>⬤</td>
        <td></td>
        <td></td>
        <td></td>
    </tr>

    <tr>
        <th>sRGB</th>

        <td></td>
        <td></td>
        <td>⬤</td>
        <td>⬤</td>
        <td></td>
        <td>⬤</td>
        <td></td>
        <td></td>
    </tr>

    <tr>
        <th>RGBA</th>

        <td></td>
        <td>⬤</td>
        <td>⬤</td>
        <td></td>
        <td>⬤</td>
        <td>⬤</td>
        <td></td>
        <td></td>
    </tr>

    <tr>
        <th>sRGBA</th>

        <td></td>
        <td>⬤</td>
        <td></td>
        <td>⬤</td>
        <td>⬤</td>
        <td>⬤</td>
        <td></td>
        <td></td>
    </tr>

    <tr>
        <th>HSV</th>

        <td></td>
        <td></td>
        <td>⬤</td>
        <td></td>
        <td></td>
        <td></td>
        <td>⬤</td>
        <td></td>
    </tr>

    <tr>
        <th>HSL</th>

        <td></td>
        <td></td>
        <td>⬤</td>
        <td></td>
        <td></td>
        <td></td>
        <td></td>
        <td>⬤</td>
    </tr>
    </table>
    </div>

    @{
*/

#pragma once


#include <cstdlib>
#include <cmath>
#include <limits>
#include <algorithm>
#include <functional>

#include <carbon/common/Common.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
    #define _CARBON_PIXEL_IS_INCLUDED // indicator so that impl headers can be safely included
#endif

// remove max and min defines in case current include trace picked it from somewhere
#pragma push_macro("max")
#pragma push_macro("min")

#undef max
#undef min


#include <carbon/data/pixel/ForwardDecl.h>
#include <carbon/data/pixel/PixelBase.h>
#include <carbon/data/pixel/ColorDefs.h>
#include <carbon/data/pixel/ColorDetailsImpl.h>
#include <carbon/data/pixel/MonoImpl.h>
#include <carbon/data/pixel/RgbImpl.h>
#include <carbon/data/pixel/HsxImpl.h>
#include <carbon/data/pixel/Traits.h>
#include <carbon/data/pixel/Utils.h>
#include <carbon/data/pixel/OperatorsImpl.h>
#include <carbon/data/pixel/Aliases.h>

#ifndef DOXGEN_SHOULD_SKIP_THIS  // this contains external namespace definitions, Doxygen should really not parse this.
    #include <carbon/data/pixel/Interop.h>
#endif

#pragma pop_macro("max")
#pragma pop_macro("min")

/** @} */  // endgroup Pixel
