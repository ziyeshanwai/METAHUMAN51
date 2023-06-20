// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/algs.h>
RLIBV_RENABLE_WARNINGS


namespace rlibv
{
	/**
	 * @brief Evaluate Legendre polynomials
	 * @tparam T Must be either 'double', 'long double' or 'float'
	 * @param n The order of the Legendre polynomial
	 * @param x The value at which to evaluate the polynomial
	 * @return The value of the Legendre polynomial of order n at value x
	 */
	template<typename T>
	T legendre(unsigned int n, T x);
#	
	/**
	 * @brief evaluate the Halton sequence
	 * @tparam T Must be either 'double', 'long double' or 'float'
	 * @param index
	 * @param base
	 * @return the value of the Halton sequence at index for given base
	 */
	template<typename T>
	T halton(int index, int base);

	/**
	 * @brief Puts an angle in the range 0 to 2*pi
	 * @tparam T Must be either 'double', 'long double' or 'float'
	 * @param angle the input angle
	 * @returns the angle recomputed to be in the range 0 to 2*pi
	 */
	template<typename T>
	T wrap_angle(T angle);
}

#include "impl/maths_functions.hpp"