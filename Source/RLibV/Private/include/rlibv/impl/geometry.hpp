// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "../include/rlibv/transforms.h"
#include "../include/rlibv/data_utils.h"
#include "../include/rlibv/linear_range.h"

namespace rlibv
{

	template<typename T, typename K>
	std::vector<point2d<T>> cast_shape(const std::vector<point2d<K>>& shape)
	{
		std::vector<point2d<T>> result(shape.size());
		for (size_t i = 0; i < shape.size(); ++i)
		{
			result[i].x() = static_cast<T>(shape[i].x());
			result[i].y() = static_cast<T>(shape[i].y());
		}
		return result;
	}
	
	template<typename T>
	bool in_polygon(const std::vector<point2d<T>>& polygon, const point2d<T>& pt)
	{
		bool c = false;
		int i, j;
		for (i = 0, j = static_cast<int>(polygon.size()) - 1; i < static_cast<int>(polygon.size()); j = i++)
		{
			if (((polygon[i].y() > pt.y()) != (polygon[j].y() > pt.y())) &&
				(pt.x() < (polygon[j].x() - polygon[i].x()) * (pt.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y()) + polygon[i].x()))
			{
				c = !c;
			}
		}
		return c;
	}


	template<typename T>
	T distance_to_line_segment(const point2d<T>& a, const point2d<T>& b, const point2d<T>& p)
	{
		//Find the scalar projection of the line p-a onto b-a
		T seg_length_squared = static_cast<T>((b - a).length_squared());
		if (seg_length_squared < std::numeric_limits<T>::epsilon()) // Line segment has no length return distance to /one of the ends
		{
			return static_cast<T>((p - a).length());
		}
		
		T proj = dot(p - a, b - a) / seg_length_squared;
		
		//If it's greater than one, the nearest point is end b
		if (proj >= 1)
		{
			return static_cast<T>((p - b).length());
		}
		//If it's less than 0, the nearest point is a
		if (proj <= 0)
		{
			return static_cast<T>((p - a).length());
		}
		point2d<T> projected = a + proj * (b - a);
		return static_cast<T>((p - projected).length());
	}

	template<typename T>
	T distance_to_polyline(const std::vector<point2d<T>>& line_pts, const point2d<T>& p)
	{
		DLIB_ASSERT(line_pts.size() > 1);
		T min_dist = std::numeric_limits<T>::max();
		for (size_t i = 1; i < line_pts.size(); ++i)
		{
			min_dist = std::min(min_dist, distance_to_line_segment(line_pts[i], line_pts[i - 1], p));
		}
		return min_dist;
	}


	template<typename T>
	std::vector<std::vector<T>> point_cross_distances(const std::vector<point2d<T>>& points)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		auto n_points = points.size();
		std::vector<std::vector<T>> distances(n_points);
		for (auto p = 0; p < points.size(); ++p)
		{
			distances[p].resize(n_points);
			for (auto q = 0; q < points.size(); ++q)
			{
				distances[p][q] = static_cast<T>((points[p] - points[q]).length());
			}
		}
		return distances;
	}

	template<typename T>
	T quad_area(const std::array<point2d<T>, 4>& verts)
	{
		static_assert(dlib::is_float_type<T>::value, "x must be a float or double type");

		// Initialize area
		T area = 0;

		// Calculate value of shoelace formula
		int j = static_cast<int>(verts.size()) - 1;
		for (auto i = 0; i < 4; i++)
		{
			area += (verts[j].x() + verts[i].x()) * (verts[j].y() - verts[i].y());
			j = i;  // j is previous vertex to i
		}

		// Return absolute value
		return std::abs(area / 2.0f);
	}


	template<typename T>
	std::vector<T> tangent_angles(const std::vector<point2d<T>>& curve, bool closed)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(curve.size() > 0);

		std::vector<T> angles(curve.size());
		for (unsigned int i = 0; i < curve.size(); ++i)
		{
			if (0 == i)
			{
				if (!closed)
				{
					const auto d1 = curve[i + 1] - curve[i];
					const auto d2 = curve[i + 1] - curve[i];
					const auto d = d1 / length(d1) + d2 / length(d2);
					const auto nx = d.y();
					const auto ny = d.x();
					angles[i] = atan2(ny, nx);
				}
				else
				{
					const auto d1 = curve[i] - curve[curve.size() - 1];
					const auto d2 = curve[i + 1] - curve[i];
					const auto d = d1 / length(d1) + d2 / length(d2);
					const auto nx = d.y();
					const auto ny = d.x();
					angles[i] = atan2(ny, nx);
				}
			}
			else if (static_cast<int>(curve.size()) - 1 == static_cast<int>(i))
			{
				if (!closed)
				{
					const auto d1 = curve[i] - curve[i - 1];
					const auto d2 = curve[i] - curve[i - 1];
					const auto d = d1 / length(d1) + d2 / length(d2);
					const auto nx = d.y();
					const auto ny = d.x();
					angles[i] = atan2(ny, nx);
				}
				else
				{
					const auto d1 = curve[i] - curve[i - 1];
					const auto d2 = curve[0] - curve[i];
					const auto d = d1 / length(d1) + d2 / length(d2);
					const auto nx = d.y();
					const auto ny = d.x();
					angles[i] = atan2(ny, nx);
				}
			}
			else
			{
				const auto d1 = curve[i] - curve[i - 1];
				const auto d2 = curve[i + 1] - curve[i];
				const auto d = d1 / length(d1) + d2 / length(d2);
				const auto nx = d.y();
				const auto ny = d.x();
				angles[i] = atan2(ny, nx);
			}
		}

		//This procedure deals with angles that aren't valid by picking the next valid one (or last valid one if it's the last angle)
		for (int a = 0; a < angles.size(); ++a)
		{
			if (a < (angles.size() - 1))
			{
				if (!dlib::is_finite(angles[a]))
				{
					for (int b = 1; b < angles.size(); ++b)
					{
						if (dlib::is_finite(angles[b]))
						{
							angles[a] = angles[b];
							break;
						}
					}
				}
			}
			else
			{
				if (!dlib::is_finite(angles[a]))
				{
					angles[a] = angles[a - 1];
				}
			}
		}
		// End angle clean up procedure

		return angles;
	}
	

	template<typename T>
	std::vector<T> normal_angles(const std::vector<point2d<T>>& curve, bool closed)
	{
		std::vector<T> t_angles = tangent_angles(curve, closed);
		std::vector<T> normal_angles(t_angles.size());
		for (int i = 0; i < t_angles.size(); ++i)
		{
			normal_angles[i] = t_angles[i] - dlib::pi / 2.0;
		}
		return normal_angles;
	}

	template<typename T>
	T signed_angle(point2d<T> a, point2d<T> b, point2d<T> c, point2d<T> d)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT((b - a).length() > 0);
		DLIB_ASSERT((d - c).length() > 0);

		auto v1 = b - a;
		auto v2 = d - c;
		auto dot = v1.x() * v2.x() + v1.y() * v2.y();
		auto det = v1.x() * v2.y() - v1.y() * v2.x();
		return atan2(det, dot);
	}


	template<typename T>
	std::vector<point2d<T>> spread_points_evenly(const std::vector<point2d<T>>& input_points, int n_output_points)
	{
		std::vector<T> cum_distance(input_points.size(), 0.0);
		for (int p = 1; p < input_points.size(); p++)
		{
			cum_distance[p] = cum_distance[p - 1] + static_cast<T>((input_points[p] - input_points[p - 1]).length());
		}
		
		std::vector<T> target_distances = rlibv::linear_range<T>(0.0, cum_distance.back(), n_output_points);
		std::vector<point2d<T>> result(target_distances.size());
		int index_of_last_upper_bound = 0;
		result[0] = input_points.front();
		result[n_output_points - 1] = input_points.back();

		for (int op = 1; op < n_output_points-1; ++op)
		{
			int index_of_upper_bound = index_of_last_upper_bound;
			while (index_of_upper_bound < input_points.size())
			{
				if (cum_distance[index_of_upper_bound] >= target_distances[op]) break;
				++index_of_upper_bound;
			}
			int index_of_lower_bound = index_of_upper_bound - 1;

			T gap_to_lower_bound = target_distances[op] - cum_distance[index_of_lower_bound];
			T gap_to_upper_bound = cum_distance[index_of_upper_bound] - target_distances[op];
			T lower_weight = gap_to_upper_bound / (gap_to_lower_bound + gap_to_upper_bound);
			T upper_weight = 1.0f - lower_weight;

			result[op] = lower_weight * input_points[index_of_lower_bound] + upper_weight * input_points[index_of_upper_bound];
			
			index_of_last_upper_bound = index_of_upper_bound;
		}
		return result;
	}


	template<typename T>
	dlib::matrix<T, 4, 1> minx_maxx_miny_maxy(const shape2d<T>& points)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(points.size() > 0);

		dlib::matrix<T, 4, 1> result;
		T min_x = 0;
		T max_x = 0;
		T min_y = 0;
		T max_y = 0;
		auto is_first = true;
		for (auto item : points)
		{
		if (is_first)
		{
			min_x = item.x();
			max_x = min_x;
			min_y = item.y();
			max_y = min_y;
			is_first = false;
		}
		else
		{
			min_x = std::min(min_x, item.x());
			max_x = std::max(max_x, item.x());
			min_y = std::min(min_y, item.y());
			max_y = std::max(max_y, item.y());
		}
		}

		result(0) = min_x;
		result(1) = max_x;
		result(2) = min_y;
		result(3) = max_y;

		return result;
	}
	
	template<typename T>
	dlib::matrix<T, 4, 1> minx_maxx_miny_maxy(const std::vector<shape2d<T>>& points)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
		DLIB_ASSERT(points.size() > 0);
		DLIB_ASSERT(points[0].size() > 0);

		auto result = minx_maxx_miny_maxy(points[0]);
		for (auto item : points)
		{
			auto this_result = minx_maxx_miny_maxy(item);
			result(0) = std::min(result(0), this_result(0));
			result(1) = std::max(result(1), this_result(1));
			result(2) = std::min(result(2), this_result(2));
			result(3) = std::max(result(3), this_result(3));
		}
		return result;
	}

	template<typename T>
	T fractional_scalar_projection(const point2d<T>& a, const point2d<T>& b)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(b.length() > 0);

		if (b.length_squared() > std::numeric_limits<T>::min())
		{
			return static_cast<T>(dot(a, b) / b.length_squared());
		}
		return std::numeric_limits<T>::max();
	}

	template<typename T>
	T fractional_scalar_projection(const point2d<T>& a, const point2d<T>& b, const point2d<T>& c, const point2d<T>& d)
	{
		static_assert(dlib::is_float_type<T>::value, "x must be a float or double type");
		DLIB_ASSERT((d - c).length() > 0);

		return fractional_scalar_projection(b - a, d - c);
	}

	template<typename T>
	point2d<T> catmullrom_point_on_curve(point2d<T> a, point2d<T> b, point2d<T> c, point2d<T> d, T t, T alpha)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		T t0 = 0.0f;
		T t1 = t0 + static_cast<T>(std::pow((b - a).length_squared(), 0.5f * alpha));
		T t2 = t1 + static_cast<T>(std::pow((c - b).length_squared(), 0.5f * alpha));
		T t3 = t2 + static_cast<T>(std::pow((d - c).length_squared(), 0.5f * alpha));

		if (abs(t1 - t0) < 1.e-8f || abs(t2 - t1) < 1.e-8f || abs(t3 - t2) < 1.e-8f || abs(t2 - t0) < 1.e-8f || abs(t3 - t1) < 1.e-8f)
		{
			return b;
		}
		
		t = t1 + t*(t2 - t1);

		point2d<T> a1 = (t1 - t) / (t1 - t0) * a + (t - t0) / (t1 - t0) * b;
		point2d<T> a2 = (t2 - t) / (t2 - t1) * b + (t - t1) / (t2 - t1) * c;
		point2d<T> a3 = (t3 - t) / (t3 - t2) * c + (t - t2) / (t3 - t2) * d;
		point2d<T> b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2;
		point2d<T> b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3;
		return (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2;
	}

	template<typename T>
	shape2d<T> generate_catmullrom_spline(const shape2d<T>& points, int n_points_per_section, bool closed)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(points.size() >= 2);
		return closed ? generate_catmullrom_spline(points, std::vector<int>(points.size(), n_points_per_section), true)
					  :
			   generate_catmullrom_spline(points, std::vector<int>(points.size() - 1, n_points_per_section), false);
	}

	template<typename T>
	shape2d<T> generate_catmullrom_spline(const shape2d<T>& points,
			const std::vector<int>& n_points_per_section, bool closed)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(points.size() >= 2);
		DLIB_ASSERT((n_points_per_section.size() == (points.size() - 1)) && (closed == false) ||
				(n_points_per_section.size() == points.size()) && (closed == true));
		shape2d<T> spline_points;
		point2d<T> a, b, c, d;

		if (points.size() == 2)   // Straight line
		{
			auto delta = points.back() - points.front();
			auto step = delta / (n_points_per_section[0] - 1);
			for (auto i = 0; i < n_points_per_section[0]; ++i)
			{
				spline_points.emplace_back(points.front() + i * step);
			}
		}
		else
		{
			if (!closed)
			{
				for (auto i_point = 0; i_point < static_cast<int>(points.size()) - 1; i_point++)
				{
					if (0 == i_point)
					{
						a = points[i_point] - (points[i_point + 1] - points[i_point]);
						b = points[i_point];
						c = points[i_point + 1];
						d = points[i_point + 2];
						spline_points.emplace_back(points[i_point]); //Add the first point if this is the first section
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
						}
					}
					else if (i_point == (static_cast<int>(points.size()) - 2))
					{
						a = points[i_point - 1];
						b = points[i_point];
						c = points[i_point + 1];
						d = c - (b - c);
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
						}
					}
					else
					{
						a = points[i_point - 1];
						b = points[i_point];
						c = points[i_point + 1];
						d = points[i_point + 2];
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
						}
					}
				}
			}
			else // closed
			{
				for (auto i_point = 0; i_point < static_cast<int>(points.size()); i_point++)
				{
					if (0 == i_point)
					{
						a = points[static_cast<int>(points.size()) - 1];
						b = points[i_point];
						c = points[i_point + 1];
						d = points[i_point + 2];
						spline_points.emplace_back(points[i_point]); //Add the first point if this is the first section
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
						}
					}
					else if (i_point == (static_cast<int>(points.size()) - 2))
					{
						a = points[i_point - 1];
						b = points[i_point];
						c = points[i_point + 1];
						d = points[0];
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
						}
					}
					else if (i_point == (static_cast<int>(points.size()) - 1))
					{
						a = points[i_point - 1];
						b = points[i_point];
						c = points[0];
						d = points[1];
						for (auto j = 1; j <= n_points_per_section[i_point] - 1; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
						}
					}
					else
					{
						a = points[i_point - 1];
						b = points[i_point];
						c = points[i_point + 1];
						d = points[i_point + 2];
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
						}
					}
				}
			}
		}
		return spline_points;
	}

	template<typename T>
	void generate_catmullrom_spline(const shape2d<T>& points, const std::vector<int>& n_points_per_section,
			bool closed, shape2d<T>& spline_points, std::vector<int>& sections)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(points.size() >= 2);
		DLIB_ASSERT((n_points_per_section.size() == (points.size() - 1)) && (closed == false) ||
				(n_points_per_section.size() == points.size()) && (closed == true));

		spline_points.clear();
		sections.clear();
		point2d<T> a, b, c, d;

		if (!closed && points.size() == 2)   // Straight line
		{
			auto delta = points.back() - points.front();
			auto step = delta / (n_points_per_section[0] - 1);
			for (auto i = 0; i < n_points_per_section[0]; ++i)
			{
				spline_points.emplace_back(points.front() + i * step);
				sections.emplace_back(0);
			}
		}
		else
		{
			if (!closed)
			{
				for (unsigned int i_point = 0; i_point < static_cast<unsigned int>(points.size()) - 1; i_point++)
				{
					if (0 == i_point)
					{
						a = points[i_point] - (points[i_point + 1] - points[i_point]);
						b = points[i_point];
						c = points[i_point + 1];
						d = points[i_point + 2];
						spline_points.emplace_back(points[i_point]); //Add the first point if this is the first section
						sections.emplace_back(i_point);
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
							sections.emplace_back(i_point);
						}
					}
					else if (i_point == (static_cast<int>(points.size()) - 2))
					{
						a = points[i_point - 1];
						b = points[i_point];
						c = points[i_point + 1];
						d = c - (b - c);
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
							sections.emplace_back(i_point);
						}
					}
					else
					{
						a = points[i_point - 1];
						b = points[i_point];
						c = points[i_point + 1];
						d = points[i_point + 2];
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
							sections.emplace_back(i_point);
						}
					}
				}
			}
			else // closed
			{
				for (unsigned int i_point = 0; i_point < static_cast<unsigned int>(points.size()); i_point++)
				{
					if (0 == i_point)
					{
						a = points[static_cast<int>(points.size()) - 1];
						b = points[i_point];
						c = points[i_point + 1];
						d = points[i_point + 2];
						spline_points.emplace_back(points[i_point]); //Add the first point if this is the first section
						sections.emplace_back(i_point);
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
							sections.emplace_back(i_point);
						}
					}
					else if (i_point == (static_cast<int>(points.size()) - 2))
					{
						a = points[i_point - 1];
						b = points[i_point];
						c = points[i_point + 1];
						d = points[0];
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
							sections.emplace_back(i_point);
						}
					}
					else if (i_point == (static_cast<int>(points.size()) - 1))
					{
						a = points[i_point - 1];
						b = points[i_point];
						c = points[0];
						d = points[1];
						for (auto j = 1; j <= n_points_per_section[i_point] - 1; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
							sections.emplace_back(i_point);
						}
					}
					else
					{
						a = points[i_point - 1];
						b = points[i_point];
						c = points[i_point + 1];
						d = points[i_point + 2];
						for (auto j = 1; j <= n_points_per_section[i_point]; j++)
						{
							auto t = (1.0f / n_points_per_section[i_point]) * j;
							spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
							sections.emplace_back(i_point);
						}
					}
				}
			}
		}
	}

	template<typename T>
	shape2d<T> find_catmullrom_inbetween_points(const shape2d<T>& points, int n_intermediates_per_section)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(n_intermediates_per_section >= 0);
		DLIB_ASSERT(points.size() >= 2);

		shape2d<T> spline_points;
		point2d<T> a, b, c, d;

		if (points.size() == 2)   // Straight line
		{
			auto delta = points.back() - points.front();
			auto step = delta / (n_intermediates_per_section + 1);
			for (auto i = 1; i <= n_intermediates_per_section; ++i)
			{
				spline_points.emplace_back(points.front() + i * step);
			}
		}
		else
		{
			for (auto i_point = 0; i_point < static_cast<int>(points.size()) - 1; i_point++)
			{
				if (0 == i_point)
				{
					a = points[i_point] - (points[i_point + 1] - points[i_point]);
					b = points[i_point];
					c = points[i_point + 1];
					d = points[i_point + 2];
				}
				else if (i_point == (static_cast<int>(points.size()) - 2))
				{
					a = points[i_point - 1];
					b = points[i_point];
					c = points[i_point + 1];
					d = c - (b - c);

				}
				else
				{
					a = points[i_point - 1];
					b = points[i_point];
					c = points[i_point + 1];
					d = points[i_point + 2];
				}
				for (auto j = 1; j <= n_intermediates_per_section; j++)
				{
					auto t = (1.0f / (n_intermediates_per_section + 1)) * j;
					spline_points.emplace_back(catmullrom_point_on_curve<T>(a, b, c, d, t));
				}
			}
		}
		return spline_points;
	}

	template<typename T>
	shape2d<T> generate_intermediate_curve(const shape2d<T>& curve_a, const shape2d<T>& curve_b, int n_points,
			T weight, int sample_accuracy)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(n_points >= 2);
		DLIB_ASSERT(curve_a.size() >= 2);
		DLIB_ASSERT(curve_b.size() >= 2);

		auto spline_a = generate_catmullrom_spline(curve_a,
				std::max(1, sample_accuracy / static_cast<int>(curve_a.size())), false);
		auto spline_b = generate_catmullrom_spline(curve_b,
				std::max(1, sample_accuracy / static_cast<int>(curve_b.size())), false);

		std::vector<T> running_dist_a(spline_a.size(), 0);
		std::vector<T> running_dist_b(spline_b.size(), 0);
		for (auto i = 1; i < spline_a.size(); ++i)
		{
			running_dist_a[i] = static_cast<T>(running_dist_a[i - 1] + (spline_a[i] - spline_a[i - 1]).length());
		}
		for (auto i = 1; i < spline_b.size(); ++i)
		{
			running_dist_b[i] = static_cast<T>(running_dist_b[i - 1] + (spline_b[i] - spline_b[i - 1]).length());
		}
		T last_a = running_dist_a.back();
		T last_b = running_dist_b.back();
		std::for_each(running_dist_a.begin(), running_dist_a.end(), [last_a](T& el) {el /= last_a; });
		std::for_each(running_dist_b.begin(), running_dist_b.end(), [last_b](T& el) {el /= last_b; });

		const auto n_a = static_cast<int>(running_dist_a.size());
		const auto n_b = static_cast<int>(running_dist_b.size());

		shape2d<T> average_curve(n_points);
		int low_ind_a = 0;
		int low_ind_b = 0;
		for (auto i = 0; i < n_points; ++i)
		{
			const auto target_dist = static_cast<T>(i) / static_cast<T>(n_points - 1);

			while (running_dist_a[low_ind_a] <= target_dist && (low_ind_a < n_a - 1))
			{
				low_ind_a++;
			}
			low_ind_a--;
			while (running_dist_b[low_ind_b] <= target_dist && (low_ind_b < n_b - 1))
			{
				low_ind_b++;
			}
			low_ind_b--;

			const auto d_low_a = target_dist - running_dist_a[low_ind_a];
			const auto d_high_a = running_dist_a[low_ind_a + 1] - target_dist;
			const auto w_low_a = d_high_a / (d_low_a + d_high_a);
			const auto w_high_a = 1.0 - w_low_a;

			const auto d_low_b = target_dist - running_dist_b[low_ind_b];
			const auto d_high_b = running_dist_b[low_ind_b + 1] - target_dist;
			const auto w_low_b = d_high_b / (d_low_b + d_high_b);
			const auto w_high_b = 1.0 - w_low_b;

			average_curve[i] = (1 - weight) * (w_low_a * spline_a[low_ind_a] + w_high_a * spline_a[low_ind_a + 1]) + weight * (w_low_b * spline_b[low_ind_b] + w_high_b * spline_b[low_ind_b + 1]);
		}
		return average_curve;
	}
	
	template<typename T>
	shape2d<T> centre_shape_at_origin(const shape2d<T>& points)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(points.size() >= 1);

		auto mean_pt = point2d<T>(0, 0);
		for (const auto& pt : points)
		{
			mean_pt += pt;
		}
		mean_pt /= static_cast<T>(points.size());
		shape2d<T> output;
		for (const auto& pt : points)
		{
			output.emplace_back(pt - mean_pt);
		}
		return output;
	}

	template<typename T>
	T shape_norm(const shape2d<T>& points)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(points.size() >= 1);

		T norm_squared = 0.0f;

		for (auto point : points)
		{
			norm_squared += point.x() * point.x() + point.y() * point.y();
		}
		return std::sqrt(norm_squared);
	}

	template<typename T>
	shape2d<T> scale_shape_to_unit_norm(const shape2d<T>& points)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(points.size() >= 1);

		auto norm = shape_norm(points);
		shape2d<T> output;
		for (auto point : points)
		{
			output.emplace_back(point2d<T>(point.x() / norm, point.y() / norm));
		}
		return output;
	}

	template<typename T>
	T norm_between_shapes(const shape2d<T>& points_a, const shape2d<T>& points_b)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(points_a.size() >= 1);
		DLIB_ASSERT(points_a.size() == points_b.size());

		T norm_squared = 0.0f;
		const auto n_points = static_cast<int>(points_a.size());

		for (auto i_point = 0; i_point < n_points; ++i_point)
		{
			auto dx = points_a[i_point].x() - points_b[i_point].x();
			auto dy = points_a[i_point].y() - points_b[i_point].y();
			norm_squared += dx * dx + dy * dy;
		}
		return std::sqrt(norm_squared);
	}

	template<typename T>
	shape2d<T> apply_random_similarity_distortion(const shape2d<T>& points, T scale_coeff, T angle_coeff, T dx_coeff, T dy_coeff, dlib::rand& rng)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(points.size() >= 1);

		dlib::matrix<T, 4, 1> ranges = minx_maxx_miny_maxy(points);
		const auto range_x = ranges(1) - ranges(0);
		const auto range_y = ranges(3) - ranges(2);
		const auto extent = std::sqrt(range_x * range_x + range_y * range_y);
		const auto max_dx = extent * dx_coeff;
		const auto max_dy = extent * dy_coeff;
		const auto mid_x = (ranges(1) + ranges(0)) / 2.0;
		const auto mid_y = (ranges(3) + ranges(2)) / 2.0;
		dlib::matrix<double, 3, 3> m;
		const auto angle = rng.get_double_in_range(-1.0, 1.0) * angle_coeff;
		const auto scale = rng.get_double_in_range(1.0 / (1.0 + scale_coeff), (1.0 + scale_coeff));
		m(0, 0) = std::cos(angle) * scale;
		m(0, 1) = -std::sin(angle) * scale;
		m(1, 0) = std::sin(angle) * scale;
		m(1, 1) = std::cos(angle) * scale;
		m(0, 2) = rng.get_double_in_range(-max_dx, max_dx);
		m(1, 2) = rng.get_double_in_range(-max_dy, max_dy);
		m(2, 0) = 0; m(2, 1) = 0; m(2, 2) = 1;

		shape2d<T> output(points);
		for (auto& item : output)
		{
			item -= point2d<T>(static_cast<T>(mid_x), static_cast<T>(mid_y));
		}
		output = transform_shape(output, dlib::point_transform_projective(m));
		for (auto& item : output)
		{
			item += point2d<T>(static_cast<T>(mid_x), static_cast<T>(mid_y));
		}

		return output;
	}
	
	template<typename T>
	shape3d<T> find_lerp_inbetween_points3d(const point3d<T>& a, const point3d<T>& b, int n_intermediate_points)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(n_intermediate_points >= 0);

		shape3d<T> points3d;

		// Straight line
		auto delta = b - a;
		auto step = delta / (n_intermediate_points + 1);
		for (auto i = 1; i <= n_intermediate_points; ++i)
		{
			points3d.push_back(a + i * step);
		}

		return points3d;
	}

	template<typename T>
	std::vector<std::vector<T>> find_lerp_inbetween_vectors(const std::vector<T>& v0, const std::vector<T>& v1, int n_intermediate_points) 
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(n_intermediate_points >= 0);
		DLIB_ASSERT(v0.size() == v1.size());

		std::vector<std::vector<T>> result(v0.size());

		for (int idx = 0; idx < v0.size(); idx++)
		{
			auto step = (v1[idx] - v0[idx]) / (n_intermediate_points + 1);

			for (auto i = 1; i <= n_intermediate_points; ++i)
				result[idx].emplace_back(v0[idx] + i * step);
		}

		return result;
	}

	template<typename T>
	T sum_of_point_distances(const shape2d<T>& points_a, const shape2d<T>& points_b)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(points_a.size() == points_b.size());

		T sum = 0;
		for (auto i = 0; i < points_b.size(); ++i)
		{
			sum += static_cast<T>((points_a[i] - points_b[i]).length());
		}
		return sum;
	}

	template<typename T>
	shape2d<T> mean_shape(const std::vector<shape2d<T>>& shapes)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(shapes.size() > 0);
		DLIB_ASSERT(all_shapes_are_same_size(shapes));

		shape2d<T> mean_shape(shapes[0].size(), point2d<double>(0, 0));
		for (auto shape : shapes)
		{
			for (auto i : dlib::range(0, static_cast<long>(shape.size()) - 1))
			{
				mean_shape[i] += shape[i];
			}
		}
		for (auto& pt : mean_shape)
		{
			pt /= static_cast<T>(shapes.size());
		}
		return mean_shape;
	}

	template<typename T>
	point2d<T> mean_point(const shape2d<T>& shape)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(shape.size() > 0);

		point2d<T> mean_pt{ 0,0 };
		for (auto pt : shape)
		{
			mean_pt += pt;
		}
		mean_pt /= static_cast<T>(shape.size());
		return mean_pt;
	}

	template<typename T>
	shape2d<double> cast_points_to_double(const shape2d<T>& points)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		std::vector<dlib::vector<double, 2>> double_points(points.size());
		for (size_t i = 0; i < points.size(); ++i)
		{
			double_points[i] = point2d<double>(points[i].x(), points[i].y());
		}
		return double_points;
	}

	template<typename T>
	shape2d<T> reinterpret_open_catmullrom_spline(const shape2d<T>& points, const std::vector<int>& anchor_indices, int n_out_points, const std::vector<int>& out_fixed_indices, int resolution)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
		//DLIB_ASSERT(n_out_points >= points.size());
		DLIB_ASSERT(resolution >= points.size() - 1);
		DLIB_ASSERT(points.size() >= 2);
		DLIB_ASSERT(anchor_indices.size() >= 0);
		DLIB_ASSERT(out_fixed_indices.size() >= 0);
		for (int i = 0; i < anchor_indices.size(); ++i)
		{
			DLIB_ASSERT(anchor_indices[i] >= 0 && anchor_indices[i] < points.size());
			DLIB_ASSERT(out_fixed_indices[i] >= 0 && out_fixed_indices[i] < n_out_points);
		}
	
		int n_sections = static_cast<int>(points.size()) - 1;
		int per_section = resolution / n_sections;
		std::vector<int> n_points_per_section(n_sections, per_section);
		std::vector<point2d<T>> interpolation_spline;
		std::vector<int> sections;
		generate_catmullrom_spline(points, n_points_per_section, false, interpolation_spline, sections);
		std::vector<T> cum_distance(interpolation_spline.size());
		cum_distance[0] = 0;
		for (auto p=1; p<interpolation_spline.size();++p)
		{
			auto dx = interpolation_spline[p].x() - interpolation_spline[p - 1].x();
			auto dy = interpolation_spline[p].y() - interpolation_spline[p - 1].y();
			cum_distance[p] = cum_distance[p - 1] + sqrt(dx * dx + dy * dy);
		}

		auto anchor_indices_and_ends(anchor_indices);
		anchor_indices_and_ends.emplace(anchor_indices_and_ends.begin(), 0);
		anchor_indices_and_ends.emplace_back(points.size() - 1);
		
		auto out_fixed_indices_and_ends(out_fixed_indices);
		out_fixed_indices_and_ends.emplace(out_fixed_indices_and_ends.begin(), 0);
		out_fixed_indices_and_ends.emplace_back(n_out_points - 1);

		std::vector<int> fixed_section_spline_start_inds;
		for(auto fi : anchor_indices_and_ends )
		{
			auto sp = std::min(resolution-1,fi * per_section);
			fixed_section_spline_start_inds.emplace_back(sp);
		}

		shape2d<T> result(n_out_points);

		int out_pt_ind = 0;
		for(int os = 0; os<out_fixed_indices_and_ends.size()-1; ++os)
		{
			int os_plus_1 = os + 1;
			int first = out_fixed_indices_and_ends[os];
			int last = out_fixed_indices_and_ends[os_plus_1];
			int diff = last - first;
			int spline_first = fixed_section_spline_start_inds[os];
			int spline_last = fixed_section_spline_start_inds[os_plus_1];
			T length_of_section = cum_distance[spline_last]-cum_distance[spline_first];
			T step = length_of_section / static_cast<T>(diff);
			T delta = 0;
			for(int i=first; i<last; ++i)
			{
				T target_distance = cum_distance[spline_first] + delta;

				auto higher_ind = std::upper_bound(cum_distance.begin(), cum_distance.end(), target_distance) - cum_distance.begin();
				if (higher_ind >= static_cast<int>(cum_distance.size()))
				{
					result[out_pt_ind] = interpolation_spline.back();
				}
				else if (higher_ind == 0)
				{
					result[out_pt_ind] = interpolation_spline[0];
				}
				else
				{
					auto lower_ind = higher_ind - 1;
					auto range = cum_distance[higher_ind] - cum_distance[lower_ind];
					auto w1 = 1 - (target_distance - cum_distance[lower_ind]) / range;
					auto w2 = 1 - w1;
					result[out_pt_ind] = w1 * interpolation_spline[lower_ind] + w2 * interpolation_spline[higher_ind];
				}
				out_pt_ind++;
				delta+=step;
			}
		}

		//The very last point
		result[n_out_points - 1] = interpolation_spline.back();

		return result;
	}


	template<typename T>
	shape2d<T> approximate_evenly_spaced_curve(const shape2d<T>& points, unsigned int n_output_points, bool is_closed)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
		
		DLIB_ASSERT(points.size() >= 4);
		DLIB_ASSERT(n_output_points >= 2);

		size_t n_input_points = points.size();

		shape2d<T> extended_curve(points);
		shape2d<T> approx_curve;
		if (is_closed)
		{
			extended_curve.insert(extended_curve.begin(), extended_curve[n_input_points - 2]);
			extended_curve.emplace_back(extended_curve[1]);
			approx_curve = approximate_open_catmullrom_spline(extended_curve, n_output_points+1, 50);
			approx_curve.pop_back();
		}
		else
		{
			point2d<T> prev = extended_curve[0] - (extended_curve[1] - extended_curve[0]);
			point2d<T> next = extended_curve[n_input_points-1] + (extended_curve[n_input_points - 1] - extended_curve[n_input_points - 2]);
			extended_curve.insert(extended_curve.begin(), prev);
			extended_curve.emplace_back(next);
			approx_curve = approximate_open_catmullrom_spline(extended_curve, n_output_points, 50);
		}
		return approx_curve;
	}

	template<typename T>
	shape2d<T> approximate_open_catmullrom_spline(const shape2d<T>& extended_points, int n_out_points, int resolution)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
		//DLIB_ASSERT(n_out_points >= extended_points.size());
		DLIB_ASSERT(extended_points.size() >= 4);
	
		int approx_resolution = n_out_points * resolution;
		int n_real_control_points = static_cast<int>(extended_points.size()) - 2;
	
		int n_real_sections = static_cast<int>(extended_points.size()) - 3;
		std::vector<T> p2p_distance(n_real_sections);
		for (int section = 0; section < n_real_sections; ++section)
		{
			auto dist = static_cast<T>((extended_points[section + 2] - extended_points[section + 1]).length());
			if(dist == 0)
			{
				// Prevents a crash from divide by zero when calculating for "invalid" curves
				dist = 0.0000001;
			}
			if (0 == section)
			{
				p2p_distance[section] = dist;
			}
			else
			{
				p2p_distance[section] = dist + p2p_distance[section-1];
			}
		}
		
		std::vector<int> n_internal_dense_pts_per_section(n_real_sections);
		int total_internal_points = 0;
		for (int section = 0; section < n_real_sections; ++section)
		{
			n_internal_dense_pts_per_section[section] = static_cast<int>(approx_resolution * p2p_distance[section] / p2p_distance.back());
			total_internal_points += n_internal_dense_pts_per_section[section];
		}
		std::vector<point2d<T>> dense_spline(total_internal_points+n_real_control_points);
		int pt_count = 0;
		for (int section = 0; section < n_real_sections; ++section)
		{
			auto pt_a = extended_points[section];
			auto pt_b = extended_points[section+1];
			auto pt_c = extended_points[section+2];
			auto pt_d = extended_points[section+3];

			auto t_vals = rlibv::linear_range<T>(0.0f, 1.0f, n_internal_dense_pts_per_section[section] + 2);
			if (section != n_real_sections - 1)
			{
				t_vals.pop_back(); //Unless we're on the last section, don't use t=1.0 because this will 
				                   //be the first point of the next section
			}
			for (const auto& t : t_vals)
			{
				auto pt = catmullrom_point_on_curve(pt_a, pt_b, pt_c, pt_d, t);
				dense_spline[pt_count++] = pt;
			}
		}

		return spread_points_evenly(dense_spline, n_out_points);
	}
	
	template<typename T>
	shape2d<T> reinterpret_closed_catmullrom_spline(const shape2d<T>& points, const std::vector<int>& anchor_indices, int n_out_points, const std::vector<int>& out_fixed_indices, int resolution)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
		//DLIB_ASSERT(n_out_points >= points.size());
		DLIB_ASSERT(resolution >= points.size() - 1);
		DLIB_ASSERT(points.size() >= 2);
		DLIB_ASSERT(anchor_indices.size() >= 0);
		DLIB_ASSERT(out_fixed_indices.size() >= 0);
		for (int i = 0; i < anchor_indices.size(); ++i)
		{
			DLIB_ASSERT(anchor_indices[i] >= 0 && anchor_indices[i] < points.size());
			DLIB_ASSERT(out_fixed_indices[i] >= 0 && out_fixed_indices[i] < n_out_points);
		}

		int n_sections = points.size();
		int per_section = resolution / n_sections;
		std::vector<int> n_points_per_section(n_sections, per_section);
		std::vector<point2d<T>> interpolation_spline;
		std::vector<int> sections;
		generate_catmullrom_spline(points, n_points_per_section, true, interpolation_spline, sections);
		std::vector<T> cum_distance(interpolation_spline.size()+1);
		cum_distance[0] = 0;
		for (auto p = 1; p < interpolation_spline.size()+1; ++p)
		{
			T dx, dy;
			if (p < interpolation_spline.size())
			{
				dx = interpolation_spline[p].x() - interpolation_spline[p - 1].x();
				dy = interpolation_spline[p].y() - interpolation_spline[p - 1].y();
			}
			else
			{
				dx = interpolation_spline[0].x() - interpolation_spline[p - 1].x();
				dy = interpolation_spline[0].y() - interpolation_spline[p - 1].y();
			}
			cum_distance[p] = cum_distance[p - 1] + sqrt(dx * dx + dy * dy);
		}

		auto anchor_indices_and_ends(anchor_indices);
		anchor_indices_and_ends.emplace(anchor_indices_and_ends.begin(), 0);
		anchor_indices_and_ends.emplace_back(points.size());

		auto out_fixed_indices_and_ends(out_fixed_indices);
		out_fixed_indices_and_ends.emplace(out_fixed_indices_and_ends.begin(), 0);
		out_fixed_indices_and_ends.emplace_back(n_out_points);

		std::vector<int> fixed_section_spline_start_inds;
		for (auto fi : anchor_indices_and_ends)
		{
			auto sp = std::min(resolution-1, fi * per_section);
			fixed_section_spline_start_inds.emplace_back(sp);
		}

		shape2d<T> result(n_out_points);

		int out_pt_ind = 0;
		for (int os = 0; os < out_fixed_indices_and_ends.size() - 1; ++os)
		{
			int os_plus_1 = os + 1;
			int first = out_fixed_indices_and_ends[os];
			int last = out_fixed_indices_and_ends[os_plus_1];
			int diff = last - first;
			int spline_first = fixed_section_spline_start_inds[os];
			int spline_last = fixed_section_spline_start_inds[os_plus_1];
			T length_of_section = cum_distance[spline_last] - cum_distance[spline_first];
			T step = length_of_section / static_cast<T>(diff);
			T delta = 0;
			for (int i = first; i < last; ++i)
			{
				T target_distance = cum_distance[spline_first] + delta;

				auto higher_ind = std::upper_bound(cum_distance.begin(), cum_distance.end(), target_distance) - cum_distance.begin();
				if (higher_ind >= static_cast<int>(cum_distance.size()))
				{
					result[out_pt_ind] = interpolation_spline.back();
				}
				else if (higher_ind == 0)
				{
					result[out_pt_ind] = interpolation_spline[0];
				}
				else
				{
					auto lower_ind = higher_ind - 1;
					auto range = cum_distance[higher_ind] - cum_distance[lower_ind];
					auto w1 = 1 - (target_distance - cum_distance[lower_ind]) / range;
					auto w2 = 1 - w1;
					result[out_pt_ind] = w1 * interpolation_spline[lower_ind] + w2 * interpolation_spline[higher_ind];
				}
				out_pt_ind++;
				delta += step;
			}
		}
		return result;
	}

	template<typename T>
	void fit_sparse_catmullrom_spline(const shape2d<T>& dense_points, const std::vector<int>& dense_anchor_inds, int max_total_points, T tolerance, bool closed, shape2d<T>& sparse_points, std::vector<int>& sparse_anchor_inds)
	{
		DLIB_ASSERT(dense_points.size() >= 2);
		DLIB_ASSERT(dense_anchor_inds.size() >= 0);
		for (int i = 0; i < static_cast<int>(dense_anchor_inds.size()); ++i)
		{
			DLIB_ASSERT(dense_anchor_inds[i] >= 0 && dense_anchor_inds[i] < dense_points.size());
		}

		sparse_points.clear();
		sparse_anchor_inds.clear();
		std::vector<int> source_inds;
		std::vector<int> source_anchor_inds;
		int ai_count = 0;
		sparse_points.emplace_back(dense_points.front());
		source_inds.emplace_back(0);
		if (closed && dense_anchor_inds.empty())
		{
			sparse_points.emplace_back(dense_points[static_cast<int>(dense_points.size()) / 3]);
			source_inds.emplace_back(static_cast<int>(dense_points.size()) / 3);
			sparse_points.emplace_back(dense_points[2* static_cast<int>(dense_points.size()) / 3]);
			source_inds.emplace_back(2 * static_cast<int>(dense_points.size()) / 3);
		}
		if(!closed)
		{
			sparse_points.emplace_back(dense_points.back());
			source_inds.emplace_back(static_cast<int>(dense_points.size()) - 1);
		}
		for (int ai : dense_anchor_inds)
		{
			if (std::find(source_inds.begin(), source_inds.end(), ai) == source_inds.end())
			{
				sparse_points.emplace_back(dense_points[ai]);
				source_inds.emplace_back(ai);
			}
			sparse_anchor_inds.emplace_back(ai_count++);
			source_anchor_inds.emplace_back(ai);
		}

		while (sparse_points.size() < max_total_points)
		{
			int resolution = 200;
			int n_sections = static_cast<int>(sparse_points.size()) - 1;
			int per_section = resolution / n_sections;
			std::vector<int> n_points_per_section(n_sections, per_section);
			std::vector<point2d<T>> interpolation_spline;
			std::vector<int> sections;
			generate_catmullrom_spline(sparse_points, n_points_per_section, closed, interpolation_spline, sections);


			T worst_diff = 0.0;
			int worst_ind = 0;
			for (int p = 0; p < static_cast<int>(dense_points.size()); ++p)
			{
				T this_smallest_diff = 9999999;
				for (int q = 0; q < static_cast<int>(interpolation_spline.size()); ++q)
				{
					this_smallest_diff  = std::min(static_cast<T>(this_smallest_diff), static_cast<T>((interpolation_spline[q] - dense_points[p]).length()));
				}
				if (this_smallest_diff > worst_diff)
				{
					worst_diff = this_smallest_diff;
					worst_ind = p;
				}
			}
			if (worst_diff < tolerance)
			{
				break;
			}
			
			source_inds.emplace_back(worst_ind);
			std::sort(source_inds.begin(), source_inds.end());
			sparse_points.clear();
			sparse_points.reserve(source_inds.size());
			for (auto si : source_inds)
			{
				sparse_points.emplace_back(dense_points[si]);
			}
			sparse_anchor_inds.clear();
			sparse_anchor_inds.reserve(source_anchor_inds.size());
			for (auto si : source_anchor_inds)
			{
				int pos = static_cast<int>(std::find(source_inds.begin(), source_inds.end(), si) - source_inds.begin());
				sparse_anchor_inds.emplace_back(pos);
			}
		}
	}

	template<typename T>
	shape2d<T> compute_procrustes_base_shape(const std::vector<shape2d<T>>& points, const alignment_type& align_method)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
		DLIB_ASSERT(shapes_all_same_size_with_at_least_n_elements(points, 2));

		if (align_method == alignment_type::none)
		{
			return mean_shape(points);
		}

		using shape = shape2d<T>;
		using set_of_shapes = std::vector<shape>;

		const T tolerance = 1e-6f;

		set_of_shapes centred_shapes;
		for (const auto& item : points)
		{
			centred_shapes.emplace_back(centre_shape_at_origin(item));
		}
		shape current_mean_shape;
		auto new_mean_shape = centred_shapes[0];
		const auto has_scale = (
			alignment_type::similarity == align_method ||
			alignment_type::scale_translate == align_method ||
			alignment_type::projective == align_method);

		if (has_scale)
		{
			new_mean_shape = scale_shape_to_unit_norm(centred_shapes[0]);
		}

		const auto reference_shape = shape(new_mean_shape);
		auto current_aligned_set_of_shapes(centred_shapes);
		do
		{
			current_mean_shape = shape(new_mean_shape);
			for (auto& this_shape : current_aligned_set_of_shapes)
			{
				dlib::point_transform_projective xform = find_transform_as_projective(this_shape, current_mean_shape, align_method);
				for (auto& pt : this_shape)
				{
					pt = xform(pt);
				}
			}
			new_mean_shape = mean_shape(current_aligned_set_of_shapes);
			dlib::point_transform_projective xform = find_transform_as_projective(new_mean_shape, reference_shape, align_method);
			for (auto& pt : new_mean_shape)
			{
				pt = xform(pt);
			}
			if (has_scale)
			{
				new_mean_shape = scale_shape_to_unit_norm(new_mean_shape);
			}

		} while (norm_between_shapes(new_mean_shape, current_mean_shape) > tolerance);

		return current_mean_shape;
	}
}

