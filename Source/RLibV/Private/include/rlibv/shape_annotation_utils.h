// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "basic_types.h"
#include "shape_annotation.h"

namespace rlibv
{
	/**
	* \defgroup Utils Utilities
	* @{
	*/

	/**
	 * @brief Returns a fixed length curve for the named curve
	 * @tparam N must be 8,16,32 or 64
	 * @param annotation The shape annotation
	 * @param curve_name The name of the curve
	 * @pre map_contains_key(annotation.keypoint_curves(), curve_name)
	 * @returns An array of points equally spaced along the named curve
	 */
	template<int N> 
	std::array<point2d<double>, N> get_fixed_size_curve(const shape_annotation& annotation, const std::string& curve_name);

	/**@}*/
}

#include "impl/shape_annotation_utils.hpp"