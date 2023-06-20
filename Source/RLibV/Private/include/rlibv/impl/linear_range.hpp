// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <type_traits>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <iterator>

#include <dlib/assert.h>

namespace rlibv
{
	template<typename T>
	std::vector<T> linear_range(T a, T b, int N)
	{
		DLIB_ASSERT(N >= 2);
		std::vector<T> result(N);
		T step_size = (b - a) / (N - 1);
		result[0] = a;
		for (int i = 1; i < N; ++i)
		{
			result[i] = result[i - 1] + step_size;
		}
		return result;
	}

	template<typename T, int N> 
	constexpr std::array<T, N> linear_range(T a, T b)
	{
		static_assert(N >= 1, "N must be >= 1");
		std::array<T, N> result;
		result[0] = a;
		if (N > 1)
		{
			T step_size = (b - a) / (N - 1);
			for (int i = 1; i < N; ++i)
			{
				result[i] = result[i - 1] + step_size;
			}
		}
		return result;
	}
}