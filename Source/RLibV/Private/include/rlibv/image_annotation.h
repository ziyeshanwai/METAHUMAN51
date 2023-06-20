// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "anchored_curve.h"
#include "curve_anchor_connection.h"

namespace rlibv
{
	using dlib::serialize;
	using dlib::deserialize;

	/**
	 * @brief Represents annotation on an image, the curves and any anchor point connections between them
	 * @tparam T T must be either float or double
	 */
	template<typename T>
	struct image_annotation
	{
		std::map<std::string, anchored_curve<T>> curves;
		std::vector<curve_anchor_connection> anchor_connections;
	};

	/**
	 * @brief Supports serialization of image_annotation structures.
	 * @tparam U U must be either float or double
	 * @param item
	 * @param out
	 */
	template<typename U>
	void serialize(const image_annotation<U>& item, std::ostream& out);

	/**
	 * @brief Supports deserialization of image_annotation structures.
	 * @tparam U U must be either float or double
	 * @param in
	 * @param item
	 */
	template<typename U>
	void deserialize(image_annotation<U>& item, std::istream& in);

}

#include "impl/image_annotation.hpp"
