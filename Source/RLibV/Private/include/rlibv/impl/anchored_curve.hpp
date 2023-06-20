// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "data_utils.h"

namespace rlibv
{
	template<typename T>
	anchored_curve<T>::anchored_curve(const rlibv::shape2d<T>& control_points,
			const std::vector<int>& extra_anchor_point_indices, bool closed)
	{
		for (auto ind : extra_anchor_point_indices)
		{
			DLIB_ASSERT(ind > 0 && ind < (control_points.size() - 1));
		}

		control_points_ = control_points;
		anchor_point_indices_.resize(extra_anchor_point_indices.size() + 2);
		anchor_point_indices_[0] = 0;
		anchor_point_indices_.back() = static_cast<int>(control_points_.size()) - 1;
		for (int i = 0; i < static_cast<int>(extra_anchor_point_indices.size()); ++i)
		{
			anchor_point_indices_[i + 1] = extra_anchor_point_indices[i];
		}
		closed_ = closed;
	}

	template<typename T>
	void anchored_curve<T>::append_anchor_point(const rlibv::point2d<T>& point)
	{
		control_points_.emplace_back(point);
		anchor_point_indices_.emplace_back(static_cast<int>(control_points_.size()) - 1);
	}

	template<typename T>
	const std::vector<int>& anchored_curve<T>::anchor_point_indices_const_ref() const
	{
		return anchor_point_indices_;
	}

	template<typename T>
	std::vector<int>& anchored_curve<T>::anchor_point_indices_ref()
	{
		return anchor_point_indices_;
	}

	template<typename T>
	const rlibv::shape2d<T>& anchored_curve<T>::control_points_const_ref() const
	{
		return control_points_;
	}

	template<typename T>
	rlibv::shape2d<T>& anchored_curve<T>::control_points_ref()
	{
		return control_points_;
	}

	template<typename T>
	bool anchored_curve<T>::closed() const
	{
		return closed_;
	}

	template<typename T>
	void anchored_curve<T>::make_closed()
	{
		closed_ = true;
		if (static_cast<int>(control_points_.size()) > anchor_point_indices_const_ref().back())
		{
			if ((control_points_.back() - control_points_.front()).length() < 0.00001f)
			{
				control_points_.erase(control_points_.begin());
			}
		}
	}

	template<typename T>
	std::array<unsigned char, 4> anchored_curve<T>::get_preferred_display_colour_uchar() const
	{
		std::array<unsigned char, 4> uchar_result;
		uchar_result[0] = static_cast<unsigned char>(preferred_display_colour_[0] * 255);
		uchar_result[1] = static_cast<unsigned char>(preferred_display_colour_[1] * 255);
		uchar_result[2] = static_cast<unsigned char>(preferred_display_colour_[2] * 255);
		uchar_result[3] = static_cast<unsigned char>(preferred_display_colour_[3] * 255);
		return uchar_result;
	}

	template<typename T>
	inline float* anchored_curve<T>::preferred_display_colour()
	{
		return preferred_display_colour_;
	}

	template<typename T>
	inline void anchored_curve<T>::clear()
	{
		control_points_.clear();
		anchor_point_indices_.clear();
	}

	template<typename T>
	inline bool anchored_curve<T>::visible() const
	{
		return visible_;
	}

	template<typename T>
	inline void anchored_curve<T>::set_visible(bool is_visible)
	{
		visible_ = is_visible;
	}

	template<typename U>
	void serialize(const anchored_curve<U>& item, std::ostream& out)
	{
		serialize(item.control_points_, out);
		serialize(item.anchor_point_indices_, out);
		serialize(item.closed_, out);
		serialize(item.preferred_display_colour_[0], out);
		serialize(item.preferred_display_colour_[1], out);
		serialize(item.preferred_display_colour_[2], out);
		serialize(item.preferred_display_colour_[3], out);
		serialize(item.visible_, out);
	}

	template<typename U>
	void deserialize(anchored_curve<U>& item, std::istream& in)
	{
		auto pos = in.tellg();
		deserialize(item.control_points_, in);
		deserialize(item.anchor_point_indices_, in);
		deserialize(item.closed_, in);
		pos = in.tellg();
		try 
		{
			deserialize(item.preferred_display_colour_[0], in);
			deserialize(item.preferred_display_colour_[1], in);
			deserialize(item.preferred_display_colour_[2], in);
			deserialize(item.preferred_display_colour_[3], in);
		}
		catch(dlib::serialization_error& e)
		{
			e;
			in.clear();
			in.seekg(pos);
			std::array<unsigned char, 4> old_colour;
			deserialize(old_colour, in);
			item.preferred_display_colour_[0] = old_colour[0] / 255.f;
			item.preferred_display_colour_[1] = old_colour[1] / 255.f;
			item.preferred_display_colour_[2] = old_colour[2] / 255.f;
			item.preferred_display_colour_[3] = old_colour[3] / 255.f;
		}
		try
		{
			deserialize(item.visible_, in);
		}
		catch (dlib::serialization_error& e)
		{
			e;
			item.visible_ = true;
		}
	}
}

