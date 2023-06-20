// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
POSEBASEDSOLVER_RENABLE_WARNINGS

namespace cm
{
	using namespace dlib;

	/**
	 *    Apply a Savitzky-Golay filter to the supplied data.
	 *
	 *    Data is supplied as a std::vector of values, data.
	 *	  The filtering considers the number of points, left, prior to the point be filtered;
	 *    and the number of points, right, subsequent to the point be filtered.
 	 *    For points at the start and end of the data set, where the full filtering window of
	 *    prior and subsequent points can not be formed, no filtering is performed and the unfiltered
	 *    values are used for the results. This will be the case for the initial, left, and final, right,
	 *    points in the dataset.
	 *    The filtering uses the supplied value, order, for the polynominal order.
	 *
	 *    - REQUIREMENTS ON T
	 *        - T must be a floating point data type, using an int type will cause a compile time assert
	 *
	 *    - Requires
	 *        - Polynomial order greater than zero
	 *
	 *    - Ensures
	 *        - Returns a vector of values which is the Savitzky-Golay filtering of the given data.
	 *        - The returned vector will contain the same number of values as the supplied data.
	 *
	 */

	template<typename T>
	std::vector<T> savitzky_golay_filter(const std::vector<T> &data, unsigned int left, unsigned int right, unsigned int order)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);
		DLIB_ASSERT(order > 0);

		const unsigned int nr = left + right + 1;
		const unsigned int nc = order + 1;

		if (data.size() < nr) return data; // Not enough data for filtering - return original data

		matrix<T> A(nr, nc);

		for (unsigned int r = 0; r < nr; r++)
			for (unsigned int c = 0; c < nc; c++)
				A(r, c) = 1;

		int t = left;
		t = -t;
		for (unsigned int r = 0; r < nr; r++)
		{
			for (int c = order - 1; c >= 0; c--)
				A(r, c) = t * A(r, c + 1);
      
			t++;
		}

		qr_decomposition<matrix<T> > qr(A);

		matrix<T> Q = qr.get_q();
		matrix<T> R = qr.get_r();

		const unsigned int nrq = Q.nr();

		matrix<T> coeffs(nrq, 1);
		for (unsigned int r = 0; r < nrq; r++)
			coeffs(r, 0) = Q(r, order) / R(order, order);

		const unsigned int n = static_cast<unsigned>(data.size());
		std::vector<T> result;

		for (std::size_t i = 0; i < left; i++) result.push_back(data[i]);
    
		for (std::size_t i = left; i < n - right; i++)
		{
			matrix<T> window(nr, 1);
			for (unsigned int r = 0; r < nr; r++) window(r, 0) = data[i - left + r];
    
			result.push_back(dot(coeffs, window));
		}

		for (std::size_t i = n - right; i < n; i++) result.push_back(data[i]);

		DLIB_ASSERT(result.size() == data.size());

		return result;
	}
}
