// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <type_traits>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <iterator>


namespace rlibv
{
	template<typename pixel_type>
	dlib::array2d<pixel_type> checkerboard_image(unsigned int rows, unsigned int cols, unsigned int patch_rows, unsigned int patch_cols, pixel_type first_colour, pixel_type second_colour)
	{
		static_assert(
			std::is_same<pixel_type, dlib::rgb_pixel>::value ||
			std::is_same<pixel_type, dlib::rgb_alpha_pixel>::value ||
			std::is_same<pixel_type, unsigned char>::value);

		dlib::array2d<pixel_type> result(rows, cols);
		for (unsigned int r = 0; r < rows; ++r)
		{
			bool row_flag = (r / patch_rows) % 2 == 0;
			for (unsigned int c = 0; c < cols; ++c)
			{
				bool col_flag = (c / patch_cols) % 2 == 0;
				if (row_flag == col_flag)
				{
					result[r][c] = first_colour;
				}
				else
				{
					result[r][c] = second_colour;
				}
			}
		}

		return result;
	}

	template<typename pixel_type>
	pixel_type random_pixel(dlib::rand& rng);

	template<>
	unsigned char random_pixel<unsigned char>(dlib::rand& rng)
	{
		return rng.get_random_8bit_number();
	}

	template<>
	dlib::rgb_pixel random_pixel<dlib::rgb_pixel>(dlib::rand& rng)
	{
		dlib::rgb_pixel val;
		val.red = rng.get_random_8bit_number();
		val.green = rng.get_random_8bit_number();
		val.blue = rng.get_random_8bit_number();
		return val;
	}

	template<>
	dlib::rgb_alpha_pixel random_pixel<dlib::rgb_alpha_pixel>(dlib::rand& rng)
	{
		dlib::rgb_alpha_pixel val;
		val.red = rng.get_random_8bit_number();
		val.green = rng.get_random_8bit_number();
		val.blue = rng.get_random_8bit_number();
		val.alpha = rng.get_random_8bit_number();
		return val;
	}

	template<typename pixel_type>
	dlib::array2d<pixel_type> random_image(
		unsigned int rows,
		unsigned int cols,
		dlib::rand& rng)
	{
		static_assert(
			std::is_same<pixel_type, dlib::rgb_pixel>::value ||
			std::is_same<pixel_type, dlib::rgb_alpha_pixel>::value ||
			std::is_same<pixel_type, unsigned char>::value);

		dlib::array2d<pixel_type> result(rows, cols);
		for (unsigned int r = 0; r < rows; ++r)
		{
			for (unsigned int c = 0; c < cols; ++c)
			{
				result[r][c] = random_pixel<pixel_type>(rng);
			}
		}

		return result;
	}
}