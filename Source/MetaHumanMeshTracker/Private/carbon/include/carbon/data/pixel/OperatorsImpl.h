// Copyright Epic Games, Inc. All Rights Reserved.

/*
    Arithmetic operator implementation for carbon::Pixel class.

    This is an internal header, and it is not meant for API use.
    It is only included through carbon/data/Pixel.h file.
*/

#pragma once

#ifndef _CARBON_PIXEL_IS_INCLUDED
    #error "This is an implementation header of <carbon/data/Pixel.h> and is not supposed to be included on its own."
#endif

namespace epic::carbon {

///////////////////////////////////////////////
// Operator implementation details

namespace detail {

// condition type to denote valid template combination for arithmetic operators between 2 pixels
template<class PL, class PR, class E = void>
struct IsValidPixelPixelArithOpSetup : std::false_type {};

// condition type to denote valid template combination for arithmetic operators between pixels and scalars
template<class P, class S, class E = void>
struct IsValidPixelScalarArithOpSetup : std::false_type {};

template<class PL, class PR>
struct IsValidPixelPixelArithOpSetup
<
    PL, PR,
    typename std::enable_if<
        std::is_convertible<typename PixelTraits<PL>::value_type, typename PixelTraits<PR>::value_type>::value>::type
> : std::true_type {};

template<class P, class S>
struct IsValidPixelScalarArithOpSetup
<
    P, S,
    typename std::enable_if<
        std::is_convertible<typename PixelTraits<P>::value_type, S>::value>::type
> : std::true_type {};

template<class L, class R>
using PixelBinaryFunc = std::function<typename PixelTraits<L>::value_type(typename PixelTraits<L>::value_type,
                                                                          typename PixelTraits<R>::value_type)>;

template<class P>
using PixelScalarBinaryFunc = std::function<typename PixelTraits<P>::value_type(typename PixelTraits<P>::value_type,
                                                                                typename PixelTraits<P>::value_type)>;


template<class PL, class PR>
constexpr PL PixelPixelBinaryOp(const PL& lhs, const PR& rhs, PixelBinaryFunc<PL, PR> op) {
    static_assert(IsValidPixelPixelArithOpSetup<PL, PR>::value,
                  "Requested arithmetic operation for given pair of pixels is not implemented.");

    static_assert(PL::channels == PR::channels, "Invalid channel count among arithmetic operator operands.");

    using size_type = typename PL::size_type;
    constexpr size_type size = PL::channels;

    PL result;
    for (size_type i = 0; i < size; ++i) {
        result[i] = op(lhs[i], rhs[i]);
    }
    return result;
}

// *InPlace* stands for operators like +=, -= etc..
template<class PL, class PR>
constexpr void PixelPixelInPlaceBinaryOp(PL& lhs, const PR& rhs, PixelBinaryFunc<PL, PR> op) {
    static_assert(IsValidPixelPixelArithOpSetup<PL, PR>::value,
                  "Requested arithmetic operation for given pair of pixels is not implemented.");

    static_assert(PL::channels == PR::channels, "Invalid channel count among arithmetic operator operands.");

    using size_type = typename PL::size_type;
    constexpr size_type size = PL::channels;
    for (size_type i = 0; i < size; ++i) {
        lhs[i] = op(lhs[i], rhs[i]);
    }
}

template<class P, class S>
constexpr P PixelScalarBinaryOp(const P& lhs, S rhs, PixelScalarBinaryFunc<P> op) {
    static_assert(IsValidPixelScalarArithOpSetup<P, S>::value,
                  "Given arithmetic operation for given pixel and scalar is not valid.");
    P result;
    constexpr std::size_t size = PixelTraits<P>::channels;
    using T = typename PixelTraits<P>::value_type;
    for (std::size_t i = 0; i < size; ++i) {
        result[i] = op(lhs[i], static_cast<T>(rhs));
    }
    return result;
}

template<class P, class S>
constexpr void PixelScalarInPlaceBinaryOp(P& lhs, S rhs, PixelScalarBinaryFunc<P> op) {
    static_assert(IsValidPixelScalarArithOpSetup<P, S>::value,
                  "Given arithmetic operation for given pixel and scalar is not valid.");
    constexpr std::size_t size = PixelTraits<P>::channels;
    using T = typename PixelTraits<P>::value_type;
    for (std::size_t i = 0; i < size; ++i) {
        lhs[i] = op(lhs[i], static_cast<T>(rhs));
    }
}

}  // detail


///////////////////////////////////////////////
// Operator API implementation

/**
    + operator. Creates temporary object that hold sum of pixel values of lhs and rhs.

    @param[in] lhs left side operand.
    @param[in] rhs right side operand.

    @return temporary pixel object containing sum.
*/
template<class PL, class PR, typename std::enable_if<is_pixel<PL>::value && is_pixel<PR>::value, int>::type = 0>
PL operator+(const PL& lhs, const PR& rhs) {
    constexpr auto op = std::plus<typename PixelTraits<PL>::value_type>();
    return detail::PixelPixelBinaryOp(lhs, rhs, op);
}

/**
    + operator. Creates temporary object that hold sum of pixel and scalar.

    @param[in] lhs left side operand, pixel.
    @param[in] rhs right side operand, scalar.

    @return temporary pixel object containing the pixel with summed values.
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P operator+(const P& lhs, S rhs) {
    constexpr auto op = std::plus<typename PixelTraits<P>::value_type>();
    return detail::PixelScalarBinaryOp(lhs, rhs, op);
}

/**
    + operator. Creates temporary object that hold sum of scalar and pixel.

    @param[in] lhs left side operand, scalar.
    @param[in] rhs right side operand, pixel.

    @return temporary pixel object containing the pixel with summed values.
*/
template<class S, class P, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P operator+(S lhs, const P& rhs) {
    constexpr auto op = std::plus<typename PixelTraits<P>::value_type>();
    return detail::PixelScalarBinaryOp(rhs, lhs, op);
}

/**
    += operator. In-place sum of pixel values of lhs and rhs.

    @param[in] lhs left side operand.
    @param[in] rhs right side operand.

    @return lhs
*/
template<class PL, class PR, typename std::enable_if<is_pixel<PL>::value && is_pixel<PR>::value, int>::type = 0>
PL& operator+=(PL& lhs, const PR& rhs) {
    constexpr auto op = std::plus<typename PixelTraits<PL>::value_type>();
    detail::PixelPixelInPlaceBinaryOp(lhs, rhs, op);
    return lhs;
}

/**
    += operator. In-place sum of pixel values with a scalar.

    @param[in] lhs left side operand, pixel.
    @param[in] rhs right side operand, scalar.

    @return lhs
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P& operator+=(P& lhs, S rhs) {
    constexpr auto op = std::plus<typename PixelTraits<P>::value_type>();
    detail::PixelScalarInPlaceBinaryOp(lhs, rhs, op);
    return lhs;
}

/**
    - operator. Creates temporary object that holds the difference between pixel values of lhs and rhs.

    @param[in] lhs left side operand.
    @param[in] rhs right side operand.

    @return temporary pixel object containing the difference.
*/
template<class PL, class PR, typename std::enable_if<is_pixel<PL>::value && is_pixel<PR>::value, int>::type = 0>
PL operator-(const PL& lhs, const PR& rhs) {
    constexpr auto op = std::minus<typename PixelTraits<PL>::value_type>();
    return detail::PixelPixelBinaryOp(lhs, rhs, op);
}

/**
    - operator. Creates temporary object that hold the difference of pixel and scalar.

    @param[in] lhs left side operand, pixel.
    @param[in] rhs right side operand, scalar.

    @return temporary pixel object containing the pixel with difference values.
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P operator-(const P& lhs, S rhs) {
    constexpr auto op = std::minus<typename PixelTraits<P>::value_type>();
    return detail::PixelScalarBinaryOp(lhs, rhs, op);
}

/**
    - operator. Creates temporary object that hold the difference of pixel and scalar.

    @param[in] lhs left side operand, scalar.
    @param[in] rhs right side operand, pixel.

    @return temporary pixel object containing the pixel with difference values.
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P operator-(S lhs, const P& rhs) {
    constexpr auto op = std::plus<typename PixelTraits<P>::value_type>();
    return detail::PixelScalarBinaryOp(rhs, lhs, op);
}

/**
    -= operator. In-place difference between pixel values of lhs and rhs.

    @param[in] lhs left side operand.
    @param[in] rhs right side operand.

    @return lhs
*/
template<class PL, class PR, typename std::enable_if<is_pixel<PL>::value && is_pixel<PR>::value, int>::type = 0>
PL& operator-=(PL& lhs, const PR& rhs) {
    constexpr auto op = std::minus<typename PixelTraits<PL>::value_type>();
    detail::PixelPixelInPlaceBinaryOp(lhs, rhs, op);
    return lhs;
}

/**
    -= operator. In-place difference between pixel values and a scalar.

    @param[in] lhs left side operand, pixel.
    @param[in] rhs right side operand, scalar.

    @return lhs
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P& operator-=(P& lhs, S rhs) {
    constexpr auto op = std::minus<typename PixelTraits<P>::value_type>();
    detail::PixelScalarInPlaceBinaryOp(lhs, rhs, op);
    return lhs;
}

/**
    * operator. Creates temporary object that holds the coefficient-wise product of pixel values of lhs and rhs.

    @param[in] lhs left side operand.
    @param[in] rhs right side operand.

    @return temporary pixel object containing the product.
*/
template<class PL, class PR, typename std::enable_if<is_pixel<PL>::value && is_pixel<PR>::value, int>::type = 0>
PL operator*(const PL& lhs, const PR& rhs) {
    constexpr auto op = std::multiplies<typename PixelTraits<PL>::value_type>();
    return detail::PixelPixelBinaryOp(lhs, rhs, op);
}

/**
    * operator. Creates temporary object that hold the product of pixel and scalar.

    @param[in] lhs left side operand, pixel.
    @param[in] rhs right side operand, scalar.

    @return temporary pixel object containing the pixel with product values.
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P operator*(const P& lhs, S rhs) {
    constexpr auto op = std::multiplies<typename PixelTraits<P>::value_type>();
    return detail::PixelScalarBinaryOp(lhs, rhs, op);
}

/**
    * operator. Creates temporary object that hold the product of pixel and scalar.

    @param[in] lhs left side operand, scalar.
    @param[in] rhs right side operand, pixel.

    @return temporary pixel object containing the pixel with product values.
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P operator*(S lhs, const P& rhs) {
    constexpr auto op = std::multiplies<typename PixelTraits<P>::value_type>();
    return detail::PixelScalarBinaryOp(rhs, lhs, op);
}

/**
    *= operator. In-place multiplication of pixel values of lhs and rhs.

    @param[in] lhs left side operand.
    @param[in] rhs right side operand.

    @return lhs
*/
template<class PL, class PR, typename std::enable_if<is_pixel<PL>::value && is_pixel<PR>::value, int>::type = 0>
PL& operator*=(PL& lhs, const PR& rhs) {
    constexpr auto op = std::multiplies<typename PixelTraits<PL>::value_type>();
    detail::PixelPixelInPlaceBinaryOp(lhs, rhs, op);
    return lhs;
}

/**
    *= operator. In-place product between pixel values and a scalar.

    @param[in] lhs left side operand, pixel.
    @param[in] rhs right side operand, scalar.

    @return lhs
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P& operator*=(P& lhs, S rhs) {
    constexpr auto op = std::multiplies<typename PixelTraits<P>::value_type>();
    detail::PixelScalarInPlaceBinaryOp(lhs, rhs, op);
    return lhs;
}

/**
    / operator. Creates temporary object that holds the coefficient-wise division of pixel values of lhs and rhs.

    @param[in] lhs left side operand.
    @param[in] rhs right side operand.

    @return temporary pixel object containing the division.
*/
template<class PL, class PR, typename std::enable_if<is_pixel<PL>::value && is_pixel<PR>::value, int>::type = 0>
PL operator/(const PL& lhs, const PR& rhs) {
    constexpr auto op = std::divides<typename PixelTraits<PL>::value_type>();
    return detail::PixelPixelBinaryOp(lhs, rhs, op);
}

/**
    / operator. Creates temporary object that hold the division of pixel and scalar.

    @param[in] lhs left side operand, pixel.
    @param[in] rhs right side operand, scalar.

    @return temporary pixel object containing the pixel with division values.
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P operator/(const P& lhs, S rhs) {
    constexpr auto op = std::divides<typename PixelTraits<P>::value_type>();
    return detail::PixelScalarBinaryOp(lhs, rhs, op);
}

/**
    / operator. Creates temporary object that hold the division of pixel and scalar.

    @param[in] lhs left side operand, scalar.
    @param[in] rhs right side operand, pixel.

    @return temporary pixel object containing the pixel with division values.
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P operator/(S lhs, const P& rhs) {
    constexpr auto op = std::divides<typename PixelTraits<P>::value_type>();
    return detail::PixelScalarBinaryOp(rhs, lhs, op);
}

/**
    /= operator. In-place division of pixel values of lhs and rhs.

    @param[in] lhs left side operand.
    @param[in] rhs right side operand.

    @return lhs
*/
template<class PL, class PR, typename std::enable_if<is_pixel<PL>::value && is_pixel<PR>::value, int>::type = 0>
PL& operator/=(PL& lhs, const PR& rhs) {
    constexpr auto op = std::divides<typename PixelTraits<PL>::value_type>();
    detail::PixelPixelInPlaceBinaryOp(lhs, rhs, op);
    return lhs;
}

/**
    /= operator. In-place division between pixel values and a scalar.

    @param[in] lhs left side operand, pixel.
    @param[in] rhs right side operand, scalar.

    @return lhs
*/
template<class P, class S, typename std::enable_if<is_pixel<P>::value && std::is_arithmetic<S>::value, int>::type = 0>
P& operator/=(P& lhs, S rhs) {
    constexpr auto op = std::divides<typename PixelTraits<P>::value_type>();
    detail::PixelScalarInPlaceBinaryOp(lhs, rhs, op);
    return lhs;
}

}  // epic::carbon
