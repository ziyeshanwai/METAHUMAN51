// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "basic_types.h"
#include "randomized_svd_error.h"
#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
RLIBV_RENABLE_WARNINGS
#include <vector>
#include <iostream>

namespace rlibv
{
	using dlib::serialize;
	using dlib::deserialize;

	template<typename T, int M, int N>
	void fixed_pca_model<T,M,N>::train(const std::vector<dlib::matrix<T, M, 1>>& data)
	{
		DLIB_ASSERT(data.size() >= N);

		dlib::matrix<T, M, 0> data_as_mat;
		data_as_mat.set_size(M, static_cast<long>(data.size()));
		auto count = 0;
		for (const auto& vec : data)
		{
			set_colm(data_as_mat, count++) = vec;
		}

		dlib::matrix<T> data_as_rows_local(trans(data_as_mat));
		mean_ = sum_rows(data_as_rows_local) / data_as_rows_local.nr();
		for (auto i_col = 0; i_col < data_as_rows_local.nc(); ++i_col)
		{
			set_colm(data_as_rows_local, i_col) = colm(data_as_rows_local, i_col) - mean_(i_col);
		}

		dlib::matrix<T> y = data_as_rows_local / std::sqrt(data_as_rows_local.nr() - 1);
		dlib::matrix<T> u, w, v;

		// despite svd_fast claiming to be randomized, the seed used within it is always the
		// same and it always gives the same results
		dlib::svd_fast(y, u, w, v, N);
		dlib::matrix<T> eigenvalues_unsorted = pointwise_multiply(w, w);
		dlib::matrix<T> eigenvectors_unsorted = v;

		if (eigenvectors_unsorted.size() < N)
		{
			throw(std::runtime_error("fixed_size_pca_model::train -> not enough eigenvectors for the model"));
		}

		// sort according to highest eigenvalues
		std::vector<std::pair<T, int>> eigenvalue_pairs(static_cast<int>(eigenvalues_unsorted.size()));
		for (auto i_eval = 0; i_eval < N; ++i_eval)
		{
			eigenvalue_pairs[i_eval] = std::pair<T, int>(eigenvalues_unsorted(i_eval, 0), i_eval);
		}
		std::sort(eigenvalue_pairs.rbegin(), eigenvalue_pairs.rend());
		eigenvectors_ = eigenvectors_unsorted;
		eigenvalues_ = trans(eigenvalues_unsorted);

		for (auto i_eval = 0; i_eval < N; ++i_eval)
		{
			int index = eigenvalue_pairs[i_eval].second;
			set_colm(eigenvectors_, i_eval) = colm(eigenvectors_unsorted, index);
			eigenvalues_(i_eval) = eigenvalues_unsorted(i_eval);

		}
		sqrt_eigenvalues_ = sqrt(eigenvalues_);
		is_trained_ = true;
		if (eigenvalues_(0) <= std::numeric_limits<T>::min())
		{
			throw randomized_svd_error("While performing rsvd pca model training, the first eigenvalue was zero.");
		}
	}

	template<typename T, int M, int N>
	dlib::matrix<T, N, 1> fixed_pca_model<T,M,N>::parameterize(const dlib::matrix<T, M, 1>& data_as_col) const
	{
		dlib::matrix<T, N, 1> result = trans((trans(data_as_col) - mean_) * eigenvectors_);
		return result;
	} 

	template<typename T, int M, int N>
	bool fixed_pca_model<T,M,N>::is_trained() const 
	{
		return is_trained_;
	}

	template<typename U, int P, int Q>
	void serialize(const fixed_pca_model<U, P, Q>& item, std::ostream& out)
	{
		serialize(item.is_trained_, out);
		serialize(item.eigenvectors_, out);
		serialize(item.eigenvalues_, out);
		serialize(item.sqrt_eigenvalues_, out);
		serialize(item.mean_, out);
	}


	template<typename U, int P, int Q>
	void deserialize(fixed_pca_model<U, P, Q>& item, std::istream& in)
	{
		deserialize(item.is_trained_, in);
		deserialize(item.eigenvectors_, in);
		deserialize(item.eigenvalues_, in);
		deserialize(item.sqrt_eigenvalues_, in);
		deserialize(item.mean_, in);
	}
}


