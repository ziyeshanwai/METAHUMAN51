// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "basic_types.h"
#include "geometry.h"

namespace rlibv
{
	template<int N>
	std::array<rlibv::point2d<double>, N> get_fixed_size_curve(const shape_annotation& annotation, const std::string& curve_name)
	{
		static_assert(N == 8 || N == 16 || N == 32 || N == 64);
		DLIB_ASSERT(map_contains_key(annotation.keypoint_curves(), curve_name));
		std::array<rlibv::point2d<double>, N> result;

		const auto& curve = annotation.keypoint_curves().at(curve_name);
		
		point2d<double> start_extension;
		point2d<double> end_extension;
		if (curve.start_keypoint_name == curve.end_keypoint_name) // Closed curve
		{
			start_extension = annotation.first_point_before_end(curve_name);
			end_extension = annotation.first_point_after_start(curve_name);
		}
		else
		{
			start_extension = annotation.dummy_first_point(curve_name);
			end_extension = annotation.dummy_last_point(curve_name);
		}

		std::vector<point2d<double>> extended_control_points(curve.internal_points.size()+4);

		extended_control_points[0] = start_extension;
		extended_control_points[1] = annotation.keypoints().at(curve.start_keypoint_name).pos;
		for (int i=0; i< curve.internal_points.size(); ++i)
		{
			extended_control_points[i+2] = curve.internal_points[i];
		}
		extended_control_points[extended_control_points.size() - 2] = annotation.keypoints().at(curve.end_keypoint_name).pos;
		extended_control_points[extended_control_points.size() -1] = end_extension;

		auto dense_with_ends = approximate_open_catmullrom_spline(extended_control_points, N, 10);
		std::copy_n(dense_with_ends.begin(), N, result.begin());

		return result;
	}

}