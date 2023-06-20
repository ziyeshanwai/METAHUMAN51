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
	using dlib::serialize;
	using dlib::deserialize;

	DECLARE_ENUM(vertex_style, smooth, sharp);

	/**
	 * @brief A struct representing a keypoint which can optionally connect curves
	 */
	struct keypoint
	{
		point2d<double> pos = point2d<double>(0., 0.);
		vertex_style style = vertex_style::sharp;
	};

	/**
	 * @brief Comparison operator
	 */
	inline bool operator==(const keypoint& lhs, const keypoint& rhs)
	{
		return (lhs.pos == rhs.pos && lhs.style == rhs.style);
	}

	void serialize(const keypoint& item, std::ostream& out);
	void deserialize(keypoint& item, std::istream& in);
}


