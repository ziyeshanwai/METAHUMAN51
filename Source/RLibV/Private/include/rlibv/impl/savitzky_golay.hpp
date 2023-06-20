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
	template<typename T>
	std::vector<T> savitzky_golay_filter(const std::vector<T>& data, unsigned int left, unsigned int right, unsigned int order)
	{
		COMPILE_TIME_ASSERT(dlib::is_float_type<T>::value);
		DLIB_CASSERT(order > 0);

		const auto rows = left + right + 1;
		const auto cols = order + 1;

		if (data.size() < rows)
		{
			return data;
		}

		dlib::matrix<T> M(rows, cols);
		M = static_cast<T>(1);

		int p = -static_cast<int>(left);
		for (unsigned int r = 0; r < rows; r++)
		{
			for (int c = order - 1; c >= 0; c--)
			{
				M(r, c) = p * M(r, c + 1);
			}
			p++;
		}

		dlib::qr_decomposition<dlib::matrix<T>> qr(M);
		dlib::matrix<T> Q = qr.get_q();
		dlib::matrix<T> R = qr.get_r();

		dlib::matrix<T> coeffs(Q.nr(), 1);
		for (auto r = 0; r < Q.nr(); r++)
		{
			coeffs(r, 0) = Q(r, order) / R(order, order);
		}

		std::vector<T> result(data.size());

		for (auto i = 0; i < static_cast<int>(left); i++)
		{
			result[i] = data[i];
		}

		for (auto i = left; i < data.size() - right; i++)
		{
			dlib::matrix<T> w(rows, 1);
			for (auto r = 0; r < static_cast<int>(rows);  r++) w(r, 0) = data[i - left + r];

			result[i] = dot(coeffs, w);
		}

		for (auto i = data.size() - right; i < data.size(); i++)
		{
			result[i] = data[i];
		}
		return result;
	}
}