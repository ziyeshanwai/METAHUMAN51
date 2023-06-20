// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "enum_ext.h"

#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
RLIBV_RENABLE_WARNINGS


namespace rlibv
{
	using dlib::serialize;
	using dlib::deserialize;

	/// Alias for a 2D std::array
	template <typename T, int ROWS, int COLS>
	using fixed_size_array2d = std::array<std::array<T, COLS>, ROWS>;

	/// Alias for a column vector of type float, double, or long double
	template<typename T, typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>
	using col_vector = dlib::matrix<T, 0, 1>;

	/// Alias for a row vector of type float, double, or long double
	template<typename T, typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>
	using row_vector = dlib::matrix<T, 1, 0>;

	/// Alias for a 2d point of type float, double, or long double
	template<typename T, typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>
	using point2d = dlib::vector<T, 2>;

	/// Alias for a 3d point of type float, double, or long double
	template<typename T, typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>
	using point3d = dlib::vector<T, 3>;

	/// Alias for a vector of point2
	template<typename T>
	using shape2d = std::vector<point2d<T>>;

	/// Alias for a vector of point3d
	template<typename T>
	using shape3d = std::vector<point3d<T>>;

	/// Alias for a shape with N points
	template<typename T, int N>
	using fixed_size_shape = std::array<point2d<T>, N>;

	/// Alias for a triangle
	template<typename T>
	using triangle = fixed_size_shape<T, 3>;

	/// Alias for a quadrilateral
	template<typename T>
	using quad = fixed_size_shape<T, 4>;

	//! Alignment types
	DECLARE_ENUM(alignment_type, none, translation, rigid, similarity, projective, scale_translate);

	/**
	 * @brief Represents a set of multiview shapes
	 * @tparam T T must be either float or double
	 */
	template<typename T>
	struct multiview_shape2d
	{
		std::vector<shape2d<T>> shapes;
		std::vector<std::vector<bool>> visible;
	};

	/**
	 * @brief Supports serialization of multiview_shape2d.
	 * @tparam U U must be either float or double
	 * @param item
	 * @param out
	 */
	template<typename U>
	void serialize(const multiview_shape2d<U>& item, std::ostream& out);
	
	/**
	 * @brief Supports deserialization of multiview_shape2d.
	 * @tparam U U must be either float or double
	 * @param in
	 * @param item
	 */
	template<typename U>
	void deserialize(multiview_shape2d<U>& item, std::istream& in);

	template<typename U>
	void deserialize_old_mvshapes2d(std::vector<multiview_shape2d<U>>& item, std::istream& in);
}

#include "impl/basic_types.hpp"