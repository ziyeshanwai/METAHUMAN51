// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <type_traits>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <iterator>





#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
RLIBV_RENABLE_WARNINGS


namespace rlibv
{
	/**
	 * @brief Apply a Savitzky-Golay filter to the supplied data.
	 * @details Data is supplied as a std::vector of values, data.
	 *			The filtering considers the number of points, left, prior to the point be filtered;
	 *			and the number of points, right, subsequent to the point be filtered.
	 *			For points at the start and end of the data set, where the full filtering window of
	 *			prior and subsequent points can not be formed, no filtering is performed and the unfiltered
	 *			values are used for the results. This will be the case for the initial, left, and final, right,
	 *			points in the dataset.
	 *			The filtering uses the supplied value, order, for the polynomial order.
	 * @tparam T T must be a type that supports basic arithmetic operations
	 * @param data the original data
	 * @param left the number of previous data points to consider
	 * @param right the number of next data points to consider
	 * @pre order must be > 0
	 */
	template<typename T>
	std::vector<T> savitzky_golay_filter(const std::vector<T>& data, unsigned int left, unsigned int right, unsigned int order);
}

#include "impl/savitzky_golay.hpp"