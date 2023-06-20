// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "basic_types.h"
#include "randomized_svd_error.h"
#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
RLIBV_RENABLE_WARNINGS
#include <vector>
#include <iostream>

namespace rlibv
{
	/**
   	* \defgroup Data Image Annotation
   	* @{
   	*/

	using dlib::serialize;
	using dlib::deserialize;

	/**
	 * @brief A struct representing a curve between keypoints 
	 */
	struct keypoint_curve
	{
		std::string start_keypoint_name;
		std::string end_keypoint_name;
		std::vector<point2d<double>> internal_points;
		std::array<unsigned char, 3> color;
	};

	/**
	 * @brief Comparison operator
	 */
	inline bool operator==(const keypoint_curve& lhs, const keypoint_curve& rhs)
	{
		return (
			lhs.start_keypoint_name == rhs.start_keypoint_name &&
			lhs.end_keypoint_name == rhs.end_keypoint_name &&
			lhs.internal_points == rhs.internal_points &&
			lhs.color == rhs.color
			);
	}

	void serialize(const keypoint_curve& item, std::ostream& out);
	void deserialize(keypoint_curve& item, std::istream& in);

	/**@}*/
}


