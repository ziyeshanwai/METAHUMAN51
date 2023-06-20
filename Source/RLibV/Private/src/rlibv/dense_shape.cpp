// Copyright Epic Games, Inc. All Rights Reserved.

#include "../include/rlibv/dense_shape.h"
#include "../include/rlibv/data_utils.h"
#include "../include/rlibv/maths_functions.h"



namespace rlibv
{
	dense_shape::dense_shape(const shape_annotation& annotation, const int image_width, const int image_height, const std::map<std::string, int>& n_dense_internals)
	{
		initialize_from_shape_annotation(annotation, image_width, image_height, n_dense_internals);
	}

	const int dense_shape::n_points() const
	{
		return static_cast<int>(points_.size());
	}

	std::vector<point2d<double>>& dense_shape::points()
	{
		return points_;
	}

	const std::vector<rlibv::point2d<double>>& dense_shape::const_points() const
	{
		return points_;
	}

	void dense_shape::set_points(const std::vector<point2d<double>>& pts)
	{
		points_ = pts;
	}

	const std::vector<int>& dense_shape::inbound_links(int index) const
	{
		DLIB_CASSERT(index >= 0 && index < points_.size());
		return inbound_links_[index];
	}

	const std::vector<int>& rlibv::dense_shape::outbound_links(int index) const
	{
		DLIB_CASSERT(index >= 0 && index < points_.size());
		return outbound_links_[index];
	}


	void dense_shape::initialize_from_shape_annotation(
		const shape_annotation& annotation, 
		const int image_width, 
		const int image_height, 
		const std::map<std::string, int>& n_dense_internals)
	{
		std::map<std::string, std::vector<int>> curve_lookup;
		std::map<std::string, int> keypoint_lookup;
		points_ = annotation.get_dense_points(image_width, image_height, inbound_links_, outbound_links_, curve_lookup, keypoint_lookup, n_dense_internals);
	}



	std::vector<std::pair<point2d<double>, point2d<double>>> dense_shape::curve_oriented_tangent_lines() const
	{
		auto lines = curve_orientied_normal_lines();
		for (auto& line : lines)
		{
			auto midpoint = (line.first + line.second) / 2.f;
			auto new_first_x = midpoint.x() - (line.first.y() - midpoint.y());
			auto new_first_y = midpoint.y() + (line.first.x() - midpoint.x());
			auto new_second_x = midpoint.x() - (line.second.y() - midpoint.y());
			auto new_second_y = midpoint.y() + (line.second.x() - midpoint.x());
			line.first.x() = new_first_x;
			line.first.y() = new_first_y;
			line.second.x() = new_second_x;
			line.second.y() = new_second_y;
		}
		return lines;
	}

	std::vector<std::array<point2d<double>, 4>> dense_shape::curve_oriented_square_patches(double scale) const
	{
		auto lines = curve_orientied_normal_lines();
		std::vector<std::array<point2d<double>, 4>> patches(lines.size());
		int count = 0;
		for (auto& line : lines)
		{
			point2d<double> mid = (line.first + line.second) / 2.0f;
			auto dxn = line.first.x() - line.second.x();
			auto dyn = line.first.y() - line.second.y();
			auto dxt = -dyn;
			auto dyt = dxn;
			point2d<double> normal = { dxn,dyn };
			point2d<double> tangent = { dxt,dyt };

			patches[count][0] = mid - scale*normal / 2.f - scale * tangent / 2.f;
			patches[count][1] = mid - scale * normal / 2.f + scale * tangent / 2.f;
			patches[count][2] = mid + scale * normal / 2.f + scale * tangent / 2.f;
			patches[count][3] = mid + scale * normal / 2.f - scale * tangent / 2.f;
			count++;
		}

		return patches;
	}

	std::vector<std::array<rlibv::point2d<double>, 4>> dense_shape::unaligned_square_patches(double scale) const
	{
		auto lines = curve_orientied_normal_lines();
		double avg_length = 3.0f;
		std::vector<std::array<point2d<double>, 4>> patches(lines.size());
		int count = 0;
		for (auto& line : lines)
		{
			point2d<double> mid = (line.first + line.second) / 2.0f;
			point2d<double> normal = { 0,avg_length };
			point2d<double> tangent = { -avg_length,0 };

			patches[count][0] = mid - scale * normal / 2.f - scale * tangent / 2.f;
			patches[count][1] = mid - scale * normal / 2.f + scale * tangent / 2.f;
			patches[count][2] = mid + scale * normal / 2.f + scale * tangent / 2.f;
			patches[count][3] = mid + scale * normal / 2.f - scale * tangent / 2.f;
			count++;
		}

		return patches;
	}

	std::vector<std::pair<point2d<double>, point2d<double>>> dense_shape::curve_orientied_normal_lines() const
	{
		std::vector<std::pair<point2d<double>, point2d<double>>> result(points_.size());

		std::vector<rlibv::point2d<double>> normals = curve_oriented_normal_vectors();
		for (int p = 0; p < points_.size(); ++p)
		{
			point2d<double> scaled_pt = { points_[p].x(),points_[p].y() };
			point2d<double> end1 = scaled_pt + normals[p];
			point2d<double> end2 = scaled_pt - normals[p];

			result[p] = { end1,end2 };
		}
		return result;
	}

	std::vector<rlibv::point2d<double>> dense_shape::curve_oriented_normal_vectors() const
	{
		std::vector<rlibv::point2d<double>> result(points_.size());
		for (int p = 0; p < points_.size(); ++p)
		{
			auto sum_dist = 0.;
			auto sum_angle = 0.;
			auto n_connections = static_cast<double>(inbound_links_[p].size()) + static_cast<double>(outbound_links_[p].size());
			for (const auto& inbound : inbound_links_[p])
			{
				point2d<double> scaled_inbound = { points_[inbound].x() , points_[inbound].y()  };
				point2d<double> scaled_pt = { points_[p].x(),points_[p].y() };
				auto delta = scaled_pt - scaled_inbound;
				sum_angle += std::atan2(delta.y(), delta.x());
				sum_dist += static_cast<double>(delta.length());
			}
			for (const auto& outbound : outbound_links_[p])
			{
				point2d<double> scaled_outbound = { points_[outbound].x() , points_[outbound].y()  };
				point2d<double> scaled_pt = { points_[p].x() ,points_[p].y()  };
				auto delta = scaled_outbound - scaled_pt;
				sum_angle += std::atan2(delta.y(), delta.x());
				sum_dist += static_cast<double>(delta.length());
			}
			auto angle = dlib::pi/2 + wrap_angle(sum_angle / n_connections);
			auto scale = 0.5f * (sum_dist / n_connections);

			auto dx = scale * cos(angle);
			auto dy = scale * sin(angle);

			result[p] = point2d<double>(static_cast<double>(dx), static_cast<double>(dy));
		}
		return result;
	}

	std::vector<point2d<double>> dense_shape::curve_oriented_tangent_vectors() const
	{
		std::vector<point2d<double>> result = curve_oriented_normal_vectors();
		for (auto& pt : result)
		{
			pt.x() = -pt.y();
			pt.y() = pt.x();
		}
		return result;
	}

	void serialize(const dense_shape& item, std::ostream& out)
	{
		serialize(item.points_, out);
		serialize(item.inbound_links_, out);
		serialize(item.outbound_links_, out);
	}

	void deserialize(dense_shape& item, std::istream& in)
	{
		deserialize(item.points_, in);
		deserialize(item.inbound_links_, in);
		deserialize(item.outbound_links_, in);
	}
}


