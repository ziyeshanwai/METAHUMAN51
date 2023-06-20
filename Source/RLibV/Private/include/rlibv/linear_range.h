// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <type_traits>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <iterator>

namespace rlibv
{
	/**
	* \defgroup Utils Utilities
	* @{
	*/

	/**
	 * @brief creates a linear range between (and including) a and b, of N elements
	 * @param a the inclusive start of the range
	 * @param b the inclusive end of the range
	 * @param N the number of elements
	 * @tparam T T must be a type that supports basic arithmetic operations
	 * @pre N must be >= 2
	 */
	template<typename T>
	std::vector<T> linear_range(T a, T b, int N);

	/**
	 * @brief creates a linear range between (and including) a and b, of N elements
	 * @tparam T T must be a type that supports basic arithmetic operations
	 * @tparam N the number of elements
	 * @param a the inclusive start of the range
	 * @param b the inclusive end of the range
	 * @pre N must be >= 2
	 */
	template<typename T, int N> 
	constexpr std::array<T, N> linear_range(T a, T b);

	/**@}*/
}

#include "impl/linear_range.hpp"