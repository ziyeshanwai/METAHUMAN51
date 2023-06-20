// Copyright Epic Games, Inc. All Rights Reserved.

#include "../include/rlibv/shape_annotation.h"
#include "../include/rlibv/data_utils.h"
#include "../include/rlibv/geometry.h"
#include "../include/rlibv/simple_profiler.h"
#include "../include/rlibv/linear_range.h"
#include <numeric>

namespace rlibv
{

	void rlibv::shape_annotation::initialize(
		const std::string& image_filename, 
		const std::map<std::string, keypoint>& keypoints, 
		const std::map<std::string, keypoint_curve>& keypoint_curves)
	{
		image_filename_ = image_filename;
		keypoints_ = keypoints;
		keypoint_curves_ = keypoint_curves;
	}

	curve_connection shape_annotation::incoming_connection(const std::string& curve_name) const
	{
		//Find the incoming connection
		curve_connection connection;

		auto incoming_keypoint_name = keypoint_curves_.at(curve_name).start_keypoint_name;
		for (const auto& other_keypoint_curve : keypoint_curves_)
		{
			if (other_keypoint_curve.first == curve_name)
			{
				continue;
			}
			if (other_keypoint_curve.second.end_keypoint_name == incoming_keypoint_name)
			{
				connection.other_curve_name = other_keypoint_curve.first;
				connection.direction = curve_connection_direction::to_end_of_other_curve;
				break;
			}
			else if (other_keypoint_curve.second.start_keypoint_name == incoming_keypoint_name)
			{
				connection.other_curve_name = other_keypoint_curve.first;
				connection.direction = curve_connection_direction::to_start_of_other_curve;
				break;
			}
		}
		return connection;
	}

	curve_connection shape_annotation::outgoing_connection(const std::string& curve_name) const
	{

		//Find the outgoing connection
		curve_connection connection;

		auto outgoing_keypoint_name = keypoint_curves_.at(curve_name).end_keypoint_name;
		for (const auto& other_keypoint_curve : keypoint_curves_)
		{
			if (other_keypoint_curve.first == curve_name)
			{
				continue;
			}
			if (other_keypoint_curve.second.start_keypoint_name == outgoing_keypoint_name)
			{
				connection.other_curve_name = other_keypoint_curve.first;
				connection.direction = curve_connection_direction::to_start_of_other_curve;
				break;
			}
			if (other_keypoint_curve.second.start_keypoint_name == outgoing_keypoint_name)
			{
				connection.other_curve_name = other_keypoint_curve.first;
				connection.direction = curve_connection_direction::to_end_of_other_curve;
				break;
			}
		}
		return connection;
	}

	point2d<double> shape_annotation::first_point_before_end(const std::string& curve_name) const
	{
		DLIB_CASSERT(map_contains_key(keypoint_curves_, curve_name));
		point2d<double> result;
		if (keypoint_curves_.at(curve_name).internal_points.empty())
		{
			result = keypoints_.at(keypoint_curves_.at(curve_name).start_keypoint_name).pos;
		}
		else
		{
			result = keypoint_curves_.at(curve_name).internal_points.back();
		}
		return result;
	}

	point2d<double> shape_annotation::first_point_after_start(const std::string& curve_name) const
	{
		DLIB_CASSERT(map_contains_key(keypoint_curves_, curve_name));
		point2d<double> result;
		if (keypoint_curves_.at(curve_name).internal_points.empty())
		{
			result = keypoints_.at(keypoint_curves_.at(curve_name).end_keypoint_name).pos;
		}
		else
		{
			result = keypoint_curves_.at(curve_name).internal_points.front();
		}
		return result;
	}

	point2d<double> shape_annotation::dummy_first_point(const std::string& curve_name) const
	{
		DLIB_CASSERT(map_contains_key(keypoint_curves_, curve_name));
		point2d<double> a = keypoints_.at(keypoint_curves_.at(curve_name).start_keypoint_name).pos;
		point2d<double> b = first_point_after_start(curve_name);
		return a - (b - a);
	}

	point2d<double> shape_annotation::dummy_last_point(const std::string& curve_name) const
	{
		DLIB_CASSERT(map_contains_key(keypoint_curves_, curve_name));
		point2d<double> a = keypoints_.at(keypoint_curves_.at(curve_name).end_keypoint_name).pos;
		point2d<double> b = first_point_before_end(curve_name);
		return a + (a - b);
	}

	const std::map<std::string, rlibv::keypoint_curve>& rlibv::shape_annotation::keypoint_curves() const
	{
		return keypoint_curves_;
	}

	std::map<std::string, rlibv::keypoint_curve>& rlibv::shape_annotation::keypoint_curves()
	{
		return keypoint_curves_;
	}

	shape_annotation::shape_annotation(const std::string& image_filename, const std::map<std::string, keypoint>& keypoints, const std::map<std::string, keypoint_curve>& keypoint_curves)
	{
		initialize(image_filename, keypoints, keypoint_curves);
	}

	shape_annotation::shape_annotation()
	{

	}

	const std::map<std::string, rlibv::keypoint>& shape_annotation::keypoints() const
	{
		return keypoints_;
	}

	std::map<std::string, rlibv::keypoint>& shape_annotation::keypoints()
	{
		return keypoints_;
	}

	void shape_annotation::insert_internal_point(const std::string& name, int insert_before, const rlibv::point2d<double> pos)
	{
		DLIB_CASSERT(rlibv::map_contains_key(keypoint_curves_, name));
		DLIB_CASSERT(insert_before <= keypoint_curves_.at(name).internal_points.size());

		auto& pts = keypoint_curves_[name].internal_points;
		if (insert_before > pts.size())
		{
			pts.emplace_back(pos);
		}
		else
		{
			pts.insert(pts.begin() + insert_before, pos);
		}
	}

	void shape_annotation::clear_internal_points(const std::string& name)
	{
		DLIB_CASSERT(rlibv::map_contains_key(keypoint_curves_, name));
		keypoint_curves_[name].internal_points.clear();
	}

	void shape_annotation::remove_internal_point(const std::string& name, int index)
	{
		DLIB_CASSERT(rlibv::map_contains_key(keypoint_curves_, name));
		DLIB_CASSERT(index < keypoint_curves_.at(name).internal_points.size());

		auto& pts = keypoint_curves_[name].internal_points;
		pts.erase(pts.begin() + index);

	}

	std::map<std::string, std::vector<rlibv::point2d<double>>> shape_annotation::get_drawing_splines(const std::map<std::string, int>& points_per_spline) const
	{
		std::map<std::string, std::vector<rlibv::point2d<double>>> result;
		std::vector<std::vector<int>> inbound_links;
		std::vector<std::vector<int>> outbound_links;
		std::map<std::string, std::vector<int>> curve_lookup;
		std::map<std::string, int> keypoint_lookup;
		std::vector<rlibv::point2d<double>> splines = get_dense_points(1, 1, inbound_links, outbound_links, curve_lookup, keypoint_lookup, points_per_spline);
		
		int count = static_cast<int>(keypoints_.size());

		for (const auto& item : keypoint_curves_)
		{
			std::vector<rlibv::point2d<double>> this_spline_internals(points_per_spline.at(item.first));
			for (int i = count; i < count + points_per_spline.at(item.first); ++i)
			{
				this_spline_internals[i - count] = splines[i];
			}
			result[item.first].emplace_back(keypoints_.at(item.second.start_keypoint_name).pos);
			result[item.first].insert(result[item.first].end(), this_spline_internals.begin(), this_spline_internals.end());
			result[item.first].emplace_back(keypoints_.at(item.second.end_keypoint_name).pos);
	
			count += points_per_spline.at(item.first);
		}
		return result;
	}

	std::map<std::string, std::array<unsigned char, 3>> shape_annotation::get_drawing_colours() const
	{
		std::map<std::string, std::array<unsigned char, 3>> result;
		for (const auto& item : keypoint_curves_)
		{
			result[item.first] = item.second.color;
		}
		return result;
	}

	void shape_annotation::set_drawing_colours(std::map<std::string, std::array<unsigned char, 3>>& colours)
	{
		for (auto& item : keypoint_curves_)
		{
			item.second.color = colours[item.first];
		}
	}

	const std::string& shape_annotation::image_filename() const
	{
		return image_filename_;
	}

	void shape_annotation::set_image_filename(const std::string& filename)
	{
		image_filename_ = filename;
	}

	std::vector<rlibv::point2d<double>> shape_annotation::get_dense_points(
		const int image_width, 
		const int image_height,
		std::vector<std::vector<int>>& inbound_links,
		std::vector<std::vector<int>>& outbound_links,
		std::map<std::string,std::vector<int>>& curve_lookup,
		std::map<std::string, int>& keypoint_lookup,
		const std::map<std::string, int>& internal_densities) const
	{
		std::vector<rlibv::point2d<double>> dense_shape;
		int sum_densities = 0;
		for (const auto& item : internal_densities)
		{
			sum_densities += item.second;
		}
		dense_shape.reserve(sum_densities + keypoints_.size()); // This might be too big, but won't be too small
		std::map<std::string, int> keypoint_inds_in_dense;
		std::map<std::string, std::vector<int>> internal_point_inds_in_dense;

		int dense_point_count = 0;
		for (const auto& [keypoint_name, keypoint_val] : keypoints_)
		{
			dense_shape.emplace_back(keypoint_val.pos);
			keypoint_inds_in_dense[keypoint_name] = dense_point_count;
			keypoint_lookup[keypoint_name] = dense_point_count;
			dense_point_count++;
		}
		
		for (const auto& [curve_name, curve_val] : keypoint_curves_)
		{
			int num_internals = internal_densities.at(curve_name);

			if (num_internals == curve_val.internal_points.size())
			{
				std::vector<int> these_inds(num_internals);
				for (int p = 0; p < num_internals; ++p)
				{
					dense_shape.emplace_back(curve_val.internal_points.at(p));
					these_inds[p] = dense_point_count++;
				}
				internal_point_inds_in_dense[curve_name] = these_inds;
			}
			else
			{
				std::vector<point2d<double>> extended_control_points;
				extended_control_points.reserve(curve_val.internal_points.size() + 4);
				//add the start extension, according to whether or not to use the connected curve
				point2d<double> start_extension;
				if (keypoints_.at(curve_val.start_keypoint_name).style == vertex_style::smooth)
				{
					if (curve_val.start_keypoint_name == curve_val.end_keypoint_name) // Closed curve
					{
						start_extension = first_point_before_end(curve_name);
					}
					else
					{
						auto incoming = incoming_connection(curve_name);
						if (!incoming.other_curve_name.empty())
						{
							bool connected_to_end = (incoming.direction == curve_connection_direction::to_end_of_other_curve);
							start_extension = connected_to_end ?
								first_point_before_end(incoming.other_curve_name) : first_point_after_start(incoming.other_curve_name);
						}
						else
						{
							start_extension = dummy_first_point(curve_name);
						}
					}
				}
				else
				{
					start_extension = dummy_first_point(curve_name);
				}
				point2d<double> end_extension;
				if (keypoints_.at(curve_val.end_keypoint_name).style == vertex_style::smooth)
				{
					if (curve_val.start_keypoint_name == curve_val.end_keypoint_name) // Closed curve
					{
						end_extension = first_point_after_start(curve_name);
					}
					else
					{
						auto outgoing = outgoing_connection(curve_name);
						if (!outgoing.other_curve_name.empty())
						{
							bool connected_to_start = (outgoing.direction == curve_connection_direction::to_start_of_other_curve);
							end_extension = connected_to_start ?
								first_point_after_start(outgoing.other_curve_name) : first_point_before_end(outgoing.other_curve_name);
						}
						else
						{
							end_extension = dummy_last_point(curve_name);
						}
					}
				}
				else
				{
					end_extension = dummy_last_point(curve_name);
				}

				extended_control_points.emplace_back(start_extension);
				extended_control_points.emplace_back(keypoints_.at(curve_val.start_keypoint_name).pos);
				for (const auto& pt : curve_val.internal_points)
				{
					extended_control_points.emplace_back(pt);
				}
				extended_control_points.emplace_back(keypoints_.at(curve_val.end_keypoint_name).pos);
				extended_control_points.emplace_back(end_extension);

				auto dense_with_ends = approximate_open_catmullrom_spline(extended_control_points, num_internals + 2, 5);

				std::vector<int> these_inds(dense_with_ends.size() - 2);
				for (size_t p = 1; p < dense_with_ends.size() - 1; ++p)
				{
					dense_shape.emplace_back(dense_with_ends.at(p));
					these_inds[p - 1L] = dense_point_count++;
				}
				internal_point_inds_in_dense[curve_name] = these_inds;
			}
		}

		// Set the links for each point
		inbound_links.clear();
		inbound_links.reserve(sum_densities + keypoints_.size());
		outbound_links.clear();
		outbound_links.reserve(sum_densities + keypoints_.size());

		for (const auto& [keypoint_name, keypoint_val] : keypoints_)
		{
			std::vector<int> these_inbound_links;
			std::vector<int> these_outbound_links;
			for (const auto& [curve_name, curve_def] : keypoint_curves_)
			{
				const auto& internal_inds = internal_point_inds_in_dense.at(curve_name);
				if (curve_def.start_keypoint_name == keypoint_name)
				{
					these_outbound_links.emplace_back(internal_inds.front());
				}
				if (curve_def.end_keypoint_name == keypoint_name)
				{
					these_inbound_links.emplace_back(internal_inds.back());
				}
			}
			inbound_links.emplace_back(these_inbound_links);
			outbound_links.emplace_back(these_outbound_links);

		}
		for (const auto& [curve_name, curve_val] : keypoint_curves_)
		{
			const auto& internal_inds = internal_point_inds_in_dense.at(curve_name);
			for (size_t i = 0; i < internal_point_inds_in_dense.at(curve_name).size(); ++i)
			{
				if (0 == i)
				{
					inbound_links.emplace_back(std::vector<int>{ keypoint_inds_in_dense[curve_val.start_keypoint_name]});
					outbound_links.emplace_back(std::vector<int>{ internal_inds[i + 1L] });
				}
				else if ((internal_point_inds_in_dense.at(curve_name).size() - 1) == i)
				{
					inbound_links.emplace_back(std::vector<int>{ internal_inds[i - 1L] });
					outbound_links.emplace_back(std::vector<int>{ keypoint_inds_in_dense[curve_val.end_keypoint_name]});
				}
				else
				{
					inbound_links.emplace_back(std::vector<int>{ internal_inds[i - 1] });
					outbound_links.emplace_back(std::vector<int>{ internal_inds[i + 1] });
				}
			}

			std::vector<int> this_curve_lookup;
			this_curve_lookup.emplace_back(keypoint_inds_in_dense[curve_val.start_keypoint_name]);
			this_curve_lookup.insert(this_curve_lookup.end(), internal_inds.begin(), internal_inds.end());
			this_curve_lookup.emplace_back(keypoint_inds_in_dense[curve_val.end_keypoint_name]);
			curve_lookup[curve_name] = this_curve_lookup;

		}

		for (auto& pt : dense_shape)
		{
			pt.x() *= image_width;
			pt.y() *= image_height;
		}

		return dense_shape;
	}

	std::vector<rlibv::point2d<double>> shape_annotation::get_dense_points(const int image_width, const int image_height, const std::map<std::string, int>& internal_densities) const
	{
		std::vector<rlibv::point2d<double>> dense_shape;
		int sum_densities = 0;
		for (const auto& item : internal_densities)
		{
			sum_densities += item.second;
		}
		dense_shape.reserve(sum_densities + keypoints_.size()); // This might be too big, but won't be too small

		for (const auto& [keypoint_name, keypoint_val] : keypoints_)
		{
			dense_shape.emplace_back(keypoint_val.pos);
		}

		for (const auto& [curve_name, curve_val] : keypoint_curves_)
		{
			int num_internals = internal_densities.at(curve_name);
			if (num_internals == curve_val.internal_points.size())
			{
				std::vector<int> these_inds(num_internals);
				for (int p = 0; p < num_internals; ++p)
				{
					dense_shape.emplace_back(curve_val.internal_points.at(p));
				}
			}
			else
			{
				std::vector<point2d<double>> extended_control_points;
				extended_control_points.reserve(curve_val.internal_points.size() + 4);
				//add the start extension, according to whether or not to use the connected curve
				point2d<double> start_extension;
				if (keypoints_.at(curve_val.start_keypoint_name).style == vertex_style::smooth)
				{
					if (curve_val.start_keypoint_name == curve_val.end_keypoint_name) // Closed curve
					{
						start_extension = first_point_before_end(curve_name);
					}
					else
					{
						auto incoming = incoming_connection(curve_name);
						if (!incoming.other_curve_name.empty())
						{
							bool connected_to_end = (incoming.direction == curve_connection_direction::to_end_of_other_curve);
							start_extension = connected_to_end ?
								first_point_before_end(incoming.other_curve_name) : first_point_after_start(incoming.other_curve_name);
						}
						else
						{
							start_extension = dummy_first_point(curve_name);
						}
					}
				}
				else
				{
					start_extension = dummy_first_point(curve_name);
				}
				point2d<double> end_extension;
				if (keypoints_.at(curve_val.end_keypoint_name).style == vertex_style::smooth)
				{
					if (curve_val.start_keypoint_name == curve_val.end_keypoint_name) // Closed curve
					{
						end_extension = first_point_after_start(curve_name);
					}
					else
					{
						auto outgoing = outgoing_connection(curve_name);
						if (!outgoing.other_curve_name.empty())
						{
							bool connected_to_start = (outgoing.direction == curve_connection_direction::to_start_of_other_curve);
							end_extension = connected_to_start ?
								first_point_after_start(outgoing.other_curve_name) : first_point_before_end(outgoing.other_curve_name);
						}
						else
						{
							end_extension = dummy_last_point(curve_name);
						}
					}
				}
				else
				{
					end_extension = dummy_last_point(curve_name);
				}

				extended_control_points.emplace_back(start_extension);
				extended_control_points.emplace_back(keypoints_.at(curve_val.start_keypoint_name).pos);
				for (const auto& pt : curve_val.internal_points)
				{
					extended_control_points.emplace_back(pt);
				}
				extended_control_points.emplace_back(keypoints_.at(curve_val.end_keypoint_name).pos);
				extended_control_points.emplace_back(end_extension);

				auto dense_with_ends = approximate_open_catmullrom_spline(extended_control_points, num_internals + 2, 5);

				for (size_t p = 1; p < dense_with_ends.size() - 1; ++p)
				{
					dense_shape.emplace_back(dense_with_ends.at(p));
				}
			}
		}
		for (auto& pt : dense_shape)
		{
			pt.x() *= image_width;
			pt.y() *= image_height;
		}

		return dense_shape;
	}

	void shape_annotation::set_from_dense_points(const std::vector<point2d<double>>& dense_shape, const std::map<std::string, std::vector<int>>& curve_lookup)
	{
		DLIB_CASSERT(all_curve_names_correct(curve_lookup));

		for (const auto& curve : curve_lookup)
		{
			const std::string& name = curve.first;
			const std::vector<int>& inds = curve.second;
			keypoints_.at(keypoint_curves_.at(name).start_keypoint_name).pos = dense_shape[inds.front()];
			keypoints_.at(keypoint_curves_.at(name).end_keypoint_name).pos = dense_shape[inds.back()];
			keypoint_curves_.at(name).internal_points.clear();
			for (int i = 1; i < inds.size() - 1; ++i)
			{
				keypoint_curves_.at(name).internal_points.emplace_back(dense_shape[inds[i]]);
			}
		}
	}

	void shape_annotation::get_sparse_points(
		const std::vector<rlibv::point2d<double>> dense_shape,
		const int image_width,
		const int image_height,
		const std::map<std::string, std::vector<int>>& curve_lookup,
		std::map<std::string, int>& internal_densities,
		const int max_internal_points,
		int max_n_func_calls,
		const float point_tolerance,
		const double error_func_epsilon)
	{
		DLIB_CASSERT(all_curve_names_correct(curve_lookup));

		for (const auto& curve : curve_lookup)
		{
			const std::string& name = curve.first;
			const std::vector<int>& inds = curve.second;
			keypoints_.at(keypoint_curves_.at(name).start_keypoint_name).pos = dense_shape[inds.front()];
			keypoints_.at(keypoint_curves_.at(name).end_keypoint_name).pos = dense_shape[inds.back()];
			keypoint_curves_.at(name).internal_points.clear();

			auto error_function = [&](rlibv::col_vector<double> inputs)
			{
				int n_internal_points = inputs.size() / 2;
				std::vector<point2d<float>> internal_pts(n_internal_points);
				for (int pt = 0; pt < n_internal_points; ++pt)
				{
					internal_pts[pt].x() = static_cast<float>(inputs(2 * pt));
					internal_pts[pt].y() = static_cast<float>(inputs(2 * pt + 1));
				}

				std::map<std::string, std::vector<int>> lookup;
				std::vector<rlibv::point2d<double>> dense_shape_with_internal_points = get_dense_points(
					image_width, 
					image_height,  
					internal_densities);
				const std::vector<shape2d<double>> dense_shapes = { dense_shape , dense_shape_with_internal_points };

				bool valid = all_shapes_are_same_size(dense_shapes);
				std::vector<float> delta(dense_shape.size());

				if (valid)
				{
					for (int pt = 0; pt < dense_shape.size(); ++pt)
					{
						delta[pt] = static_cast<float>(pow((dense_shape[pt].x() - dense_shape_with_internal_points[pt].x()) / dense_shape[pt].x(), 2.));
						delta[pt] += static_cast<float>(pow((dense_shape[pt].y() - dense_shape_with_internal_points[pt].y()) / dense_shape[pt].y(), 2.));
					}
				}

				double error_deviation = std::accumulate(delta.begin(), delta.end(), decltype(delta)::value_type(0));

				return sqrt(error_deviation / 2.0 / internal_densities.at(name));
			};

			std::vector<rlibv::point2d<double>> internal_points;
			internal_points.reserve(max_internal_points);
			for (int pt = 0; pt < max_internal_points; ++pt)
			{
				rlibv::col_vector<double> lower_bounds(2 * pt + 2);
				rlibv::col_vector<double> upper_bounds(2 * pt + 2);
				if (pt == 0)
				{
					set_all_elements(lower_bounds, 0.0);
					set_all_elements(upper_bounds, 1.0);
				}
				else
				{
					for (int int_pt = 0; int_pt < pt; ++int_pt)
					{
						lower_bounds(2 * int_pt) = internal_points[int_pt].x() - point_tolerance;
						lower_bounds(2 * int_pt + 1) = internal_points[int_pt].y() - point_tolerance;
						upper_bounds(2 * int_pt) = internal_points[int_pt].x() + point_tolerance;
						upper_bounds(2 * int_pt + 1) = internal_points[int_pt].y() + point_tolerance;
					}
					lower_bounds(2 * pt) = 0.0;
					lower_bounds(2 * pt + 1) = 0.0;
					upper_bounds(2 * pt) = 1.0;
					upper_bounds(2 * pt + 1) = 1.0;
				}

				// To optimize this difficult function all we need to do is call
				// find_min_global()
				max_n_func_calls = max_n_func_calls * (pt + 1);
				auto result = dlib::find_min_global(
					error_function, // function to minimize
					lower_bounds, // lower bounds
					upper_bounds, // upper bounds
					dlib::max_function_calls(max_n_func_calls), // run for this many calls
					1e-6 //solver epsilon
				);

				internal_points.emplace_back() = dlib::rowm(result.x, dlib::range(2 * pt, 2 * pt + 1));
				//This loop updates the internal points from the previous iteration
				for (int int_pt = 0; int_pt < pt; ++int_pt)
				{
					internal_points[int_pt] = dlib::rowm(result.x, dlib::range(2 * int_pt, 2 * int_pt + 1));
				}
				if (result.y < error_func_epsilon) { break; }
			}
			for (int i = 0; i < internal_points.size(); ++i)
			{
				keypoint_curves_.at(name).internal_points.emplace_back(internal_points[i]);
			}
		}
	}

	void shape_annotation::sparsify_simple(double factor, const std::map<std::string, int>& internal_densities)
	{
		//If the factor is less than 0.5f as a fast hack remove half the points in any case.
		if (factor < 0.5)
		{
			for (auto& curve : keypoint_curves_)
			{
				auto& internals = curve.second.internal_points;
				auto it_internals = internals.begin();
				while (it_internals != internals.end() && ++it_internals != internals.end())
				{
					it_internals = internals.erase(it_internals);
				}
			}
			factor *= 2.f; // We've chopped half out with a dumb method, so we should double the amount to 
			               // retain in what's left
		}

		//Iteratively find the internal point whose removal has the least effect on the dense result.
		std::vector<std::vector<int>> inbound_links;
		std::vector<std::vector<int>> outbound_links;
		std::map<std::string, std::vector<int>> curve_lookup;
		std::map<std::string, int> keypoint_lookup;
		auto best_dense = get_dense_points(100, 100, inbound_links, outbound_links, curve_lookup, keypoint_lookup, internal_densities);

		//Do it curve by curve
		for (auto& curve : keypoint_curves_)
		{
			int n_to_remove = std::min(
				static_cast<int>(curve.second.internal_points.size()), 
				std::max(0, static_cast<int>(curve.second.internal_points.size() * (1.0f - factor)))
			);

			//Increase the density for closed curves
			if (curve.second.start_keypoint_name == curve.second.end_keypoint_name)
			{
				
				n_to_remove = std::min(
					static_cast<int>(curve.second.internal_points.size()),
					std::max(0, static_cast<int>(curve.second.internal_points.size() * (1.0f - 1.5 * factor)))
				);
			}

			for (int i = 0; i < n_to_remove; ++i)
			{
				double best_error = 99999999.f;
				int best_internal_ind = 0;
				int n_internals = static_cast<int>(curve.second.internal_points.size());
				for (int remove_index = 0; remove_index < n_internals; ++remove_index)
				{
					auto original_internals(curve.second.internal_points);
					auto& internals = curve.second.internal_points;
					internals.erase(internals.begin() + remove_index);
					auto this_dense = get_dense_points(100, 100, internal_densities);
					
					//Only bother checking the points in this curve
					double sq_error = 0.f;
					for (int p : curve_lookup.at(curve.first))
					{
						sq_error += (this_dense[p] - best_dense[p]).length_squared();
					}
					if (sq_error < best_error)
					{
						best_error = sq_error;
						best_internal_ind = remove_index;
					}
					curve.second.internal_points = original_internals;
				}
				auto& internals = curve.second.internal_points;
				internals.erase(internals.begin() + best_internal_ind);
			}
		}
	}

	rlibv::point2d<double>* shape_annotation::get_position_pointer(const std::string& name, int index)
	{
		if (index == -1)
		{
			for (auto& item : keypoints_)
			{
				if (item.first == name)
				{
					return &(item.second.pos);
				}
			}
			return nullptr;
		}
		else if (index >= 0)
		{
			for (auto& item : keypoint_curves_)
			{
				if (item.first == name)
				{
					if (index >= item.second.internal_points.size())
					{
						return nullptr;
					}
					else
					{
						return &(item.second.internal_points[index]);
					}
				}
			}
		}

		return nullptr;
	}

	bool shape_annotation::matches_scheme(const shape_annotation& other, std::string& first_error) const
	{
		first_error.clear();
		for (const auto& item : keypoints_)
		{
			if (!rlibv::map_contains_key(other.keypoints(), item.first))
			{
				first_error = "keypoint " + item.first + " not found.";
				return false;
			}
			if (item.second.style != other.keypoints().at(item.first).style)
			{
				first_error = item.first + " vertex style doesn't match.";
				return false;
			}
		}

		for (const auto& item : keypoint_curves_)
		{
			if (!rlibv::map_contains_key(other.keypoint_curves(), item.first))
			{
				first_error = "curve " + item.first + " not found.";
				return false;
			}
			if (item.second.end_keypoint_name != other.keypoint_curves().at(item.first).end_keypoint_name)
			{
				first_error = "curve " + item.first + " end keypoint name doesn't match.";
				return false;
			}
			if (item.second.start_keypoint_name != other.keypoint_curves().at(item.first).start_keypoint_name)
			{
				first_error = "curve " + item.first + " start keypoint name doesn't match.";
				return false;
			}
		}
	

		return true;
	}

	annotation_state shape_annotation::validate_annotation_data(const std::map<std::string, keypoint>& /*keypoints*/, const std::map<std::string, keypoint_curve>& /*keypoint_curves*/)
	{
		return annotation_state::ok;
	}


	void serialize(const shape_annotation& item, std::ostream& out)
	{
		serialize(item.image_filename_, out);
		serialize(item.keypoints_, out);
		serialize(item.keypoint_curves_, out);
		
	}
	void alt_deserialize(shape_annotation& item, std::istream& in)
	{
		deserialize(item.image_filename_, in);
		deserialize(item.keypoints_, in);
		deserialize(item.keypoint_curves_, in);
	}
	void deserialize(shape_annotation& item, std::istream& in)
	{
		auto pos = in.tellg();
		try {
			deserialize(item.image_filename_, in);
			deserialize(item.keypoints_, in);
			deserialize(item.keypoint_curves_, in);
			std::map<std::string, int> n_internals_for_dense_curves;
			deserialize(n_internals_for_dense_curves, in);
			if (n_internals_for_dense_curves.size() != item.keypoint_curves_.size())
			{
				throw std::runtime_error("out of date sas file");
			}
			for (const auto& item : n_internals_for_dense_curves)
			{
				if (item.first.compare(0, 4, "crv_") != 0)
				{
					throw std::runtime_error("out of date sas file");
				}
			
			}
		}
		catch(...)
		{
			in.clear();
			in.seekg(pos);
			alt_deserialize(item, in);
		}
		
		//Temporary renaming hack
		//if (rlibv::map_contains_key(item.keypoint_curves_, std::string("upper_outer_l")))
		//{
		//	std::map<std::string, keypoint_curve> new_kpcs;
		//	new_kpcs["crv_lip_upper_outer_l"] = item.keypoint_curves_.at("upper_outer_l");
		//	new_kpcs["crv_lip_upper_inner_l"] = item.keypoint_curves_.at("upper_inner_l");
		//	new_kpcs["crv_lip_philtrum_l"] = item.keypoint_curves_.at("upper_philtrum_l");
		//	new_kpcs["crv_lip_lower_outer_l"] = item.keypoint_curves_.at("lower_outer_l");
		//	new_kpcs["crv_lip_lower_inner_l"] = item.keypoint_curves_.at("lower_inner_l");
		//	new_kpcs["crv_lip_upper_outer_r"] = item.keypoint_curves_.at("upper_outer_r");
		//	new_kpcs["crv_lip_upper_inner_r"] = item.keypoint_curves_.at("upper_inner_r");
		//	new_kpcs["crv_lip_philtrum_r"] = item.keypoint_curves_.at("upper_philtrum_r");
		//	new_kpcs["crv_lip_lower_outer_r"] = item.keypoint_curves_.at("lower_outer_r");
		//	new_kpcs["crv_lip_lower_inner_r"] = item.keypoint_curves_.at("lower_inner_r");
		//	new_kpcs["crv_nasolabial_l"] = item.keypoint_curves_.at("naso_l");
		//	new_kpcs["crv_nasolabial_r"] = item.keypoint_curves_.at("naso_r");
		//
		//
		//	for (auto& kpc_item : new_kpcs)
		//	{
		//		if (kpc_item.first.find("lip") != std::string::npos && kpc_item.first.find("_r") != std::string::npos)
		//		{
		//			std::string temp = kpc_item.second.end_keypoint_name;
		//			kpc_item.second.end_keypoint_name = kpc_item.second.start_keypoint_name;
		//			kpc_item.second.start_keypoint_name = temp;
		//			std::reverse(kpc_item.second.internal_points.begin(), kpc_item.second.internal_points.end());
		//		}
		//	}
		//
		//
		//
		//	std::map<std::string, std::string> codebook;
		//	codebook["pt_lip_corner_l"] = "corner_l";
		//	codebook["pt_lip_corner_r"] = "corner_r";
		//	codebook["pt_lip_philtrum_r"] = "upper_philtrum_r";
		//	codebook["pt_lip_upper_outer_m"] = "upper_outer_m";
		//	codebook["pt_lip_philtrum_l"] = "upper_philtrum_l";
		//	codebook["pt_lip_upper_inner_m"] = "upper_inner_m";
		//	codebook["pt_lip_lower_inner_m"] = "lower_inner_m";
		//	codebook["pt_lip_lower_outer_m"] = "lower_outer_m";
		//	codebook["pt_naso_upper_l"] = "upper_naso_l";
		//	codebook["pt_naso_lower_l"] = "lower_naso_l";
		//	codebook["pt_naso_upper_r"] = "upper_naso_r";
		//	codebook["pt_naso_lower_r"] = "lower_naso_r";
		//
		//	std::map<std::string, keypoint> new_keypoints;
		//	std::map<std::string, std::string> reverse_codebook;
		//	for (const auto& cb_item : codebook)
		//	{
		//		new_keypoints[cb_item.first] = item.keypoints_.at(cb_item.second);
		//		reverse_codebook[cb_item.second] = cb_item.first;
		//	}
		//
		//	for (auto& kpc_item : new_kpcs)
		//	{
		//		kpc_item.second.start_keypoint_name = reverse_codebook[kpc_item.second.start_keypoint_name];
		//		kpc_item.second.end_keypoint_name = reverse_codebook[kpc_item.second.end_keypoint_name];
		//	}
		//	item.keypoints_ = new_keypoints;
		//	item.keypoint_curves_ = new_kpcs;
		//
		//
		//}
		//
		////Change the name of the mouth corner if incorrect
		//std::map<std::string, keypoint_curve> renamed_kpcs;
		//std::map<std::string, keypoint> renamed_keypoints;
		//for (const auto& subitem : item.keypoints_)
		//{
		//	std::string name = subitem.first;
		//	name = replace_string(name, "pt_lip_corner", "pt_mouth_corner");
		//	renamed_keypoints[name] = subitem.second;
		//}
		//for (const auto& subitem : item.keypoint_curves_)
		//{
		//	keypoint_curve details = subitem.second;
		//	details.start_keypoint_name = replace_string(details.start_keypoint_name, "pt_lip_corner", "pt_mouth_corner");
		//	details.end_keypoint_name = replace_string(details.end_keypoint_name, "pt_lip_corner", "pt_mouth_corner");
		//	renamed_kpcs[subitem.first] = details;
		//}
		//
		//item.keypoints_ = renamed_keypoints;
		//item.keypoint_curves_ = renamed_kpcs;
	}

	bool refer_to_same_image(const shape_annotation& sa1, const shape_annotation& sa2)
	{
		size_t last_index = sa1.image_filename().find_last_of('.');
		std::string raw_sa1 = sa1.image_filename().substr(0, last_index);
		last_index = sa2.image_filename().find_last_of('.');
		std::string raw_sa2 = sa2.image_filename().substr(0, last_index);
		raw_sa1 = rlibv::strip_string(raw_sa1, { "\\","/" });
		raw_sa2 = rlibv::strip_string(raw_sa2, { "\\","/" });
		std::string sa1_root, sa2_root;
		int sa1_num = -1;
		int sa2_num = -1;
		last_index = raw_sa1.find_last_not_of("0123456789");
		if (last_index < raw_sa1.length() - 1)
		{
			sa1_num = stoi(raw_sa1.substr(last_index + 1));
			sa1_root = raw_sa1.substr(0, last_index + 1);
		}
		else
		{
			sa1_root = raw_sa1;
		}
		last_index = raw_sa2.find_last_not_of("0123456789");
		if (last_index < raw_sa2.length() - 1)
		{
			sa2_num = stoi(raw_sa2.substr(last_index + 1));
			sa2_root = raw_sa2.substr(0, last_index + 1);
		}
		else
		{
			sa2_root = raw_sa2;
		}
		if (sa1_root == sa2_root && sa1_num == sa2_num)
		{
			return true;
		}
		return false;
	}

	bool shape_annotation_less_than::operator()(const shape_annotation& sa1, const shape_annotation& sa2)
	{
		size_t last_index = sa1.image_filename().find_last_of('.');
		std::string raw_sa1 = sa1.image_filename().substr(0, last_index);
		last_index = sa2.image_filename().find_last_of('.');
		std::string raw_sa2 = sa2.image_filename().substr(0, last_index);
		raw_sa1 = rlibv::strip_string(raw_sa1, { "\\","/" });
		raw_sa2 = rlibv::strip_string(raw_sa2, { "\\","/" });
		std::string sa1_root, sa2_root;
		int sa1_num = -1;
		int sa2_num = -1;
		last_index = raw_sa1.find_last_not_of("0123456789");
		if (last_index < raw_sa1.length() - 1)
		{
			sa1_num = stoi(raw_sa1.substr(last_index + 1));
			sa1_root = raw_sa1.substr(0, last_index + 1);
		}
		else
		{
			sa1_root = raw_sa1;
		}
		last_index = raw_sa2.find_last_not_of("0123456789");
		if (last_index < raw_sa2.length() - 1)
		{
			sa2_num = stoi(raw_sa2.substr(last_index + 1));
			sa2_root = raw_sa2.substr(0, last_index + 1);
		}
		else
		{
			sa2_root = raw_sa2;
		}
		if (sa1_root < sa2_root)
		{
			return true;
		}
		if (sa1_root == sa2_root)
		{
			return sa1_num < sa2_num;
		}
		return false;
	}

}


