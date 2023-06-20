// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <type_traits>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <iterator>

#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/rand.h>
RLIBV_RENABLE_WARNINGS

namespace rlibv
{
	/**
	 * @brief Generate a checkerboard pattern
	 * @tparam T T must be a dlib pixel type
	 * @param rows the total number of rows in the final image
	 * @param cols the total number of cols in the final image
	 * @param patch_rows the number of rows in each patch
	 * @param patch_cols the number of cols in each patch
	 * @param first_colour the colour of the top left rectangle
	 * @param second_colour the alternative colour to make up a checkerboard pattern
	 */
	template<typename pixel_type>
	dlib::array2d<pixel_type> checkerboard_image(
		unsigned int rows, 
		unsigned int cols, 
		unsigned int patch_rows, 
		unsigned int patch_cols,
		pixel_type first_colour,
		pixel_type second_colour);

	/**
	 * @brief Generate a random image
	 * @tparam T T must be a dlib pixel type
	 * @param rows the total number of rows in the final image
	 * @param cols the total number of cols in the final image
	 * @param rng the random number generator
	 */
	template<typename pixel_type>
	dlib::array2d<pixel_type> random_image(
		unsigned int rows,
		unsigned int cols,
		dlib::rand& rng);

}

#include "impl/synthetic_images.hpp"