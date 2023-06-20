// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "../include/rlibv/randomized_svd_error.h"
#include "../include/rlibv/data_utils.h"

namespace rlibv
{
	template<typename T>
	void pca_model<T>::fast_train_to_variance(const std::vector<col_vector<T>>& data, T variance_to_retain,
			unsigned long max_n_modes)
	{
		DLIB_ASSERT(data.size() > 1);
		DLIB_ASSERT(data[0].nr() > 0);
		DLIB_ASSERT(variance_to_retain >= 0);
		DLIB_ASSERT(variance_to_retain <= 1);
		DLIB_ASSERT(max_n_modes > 0);
		DLIB_ASSERT(all_col_vectors_are_same_size(data));

		core_training(data, max_n_modes);
		trim_modes_for_variance(variance_to_retain, max_n_modes);
		find_limits(data);
		is_trained_ = true;
	}

	template<typename T>
	void pca_model<T>::fast_train_to_max_n_modes(const std::vector<col_vector<T>>& data, int max_n_modes)
	{
		core_training(data, max_n_modes);
		trim_modes_for_variance(1.0, max_n_modes);
		find_limits(data);
	}

	template<typename T>
	void pca_model<T>::fast_train_to_rms_error(const std::vector<col_vector<T>>& data, T max_rms_error, int max_n_modes)
	{
		core_training(data, max_n_modes);
		trim_modes_for_target_error(data, max_rms_error, max_n_modes);
		find_limits(data);
	}

	template<typename T>
	col_vector<T> pca_model<T>::parameterize(const col_vector<T>& data_as_col, T param_limit_scaling, int n_modes) const
	{
		DLIB_ASSERT(n_variables() > 0);
		DLIB_ASSERT(data_as_col.nr() == n_variables());
		DLIB_ASSERT(n_modes > 0);

		auto modes = std::min(n_params(), n_modes);

		col_vector<T> params;
		params.set_size(modes);

		col_vector<T> norm_data = dlib::trans(dlib::trans(data_as_col) - mean_);

		for (int c = 0; c < params.nr(); ++c)
		{
			T val = 0.0f;
			for (int r = 0; r < norm_data.nr(); ++r)
			{
				val += eigenvectors_(r, c) * norm_data(r);
			}
			params(c) = val;
		}

		if (param_limit_scaling >= 0)
		{
			T this_dist_sq = 0.0f;
			for (int m = 0; m < modes; ++m)
			{
				T this_delta = params(m) / (parameter_ranges_(m) / 2.0f);
				this_dist_sq += this_delta * this_delta;
			}
			T this_dist = static_cast<T>(std::sqrt(this_dist_sq));
			T target_dist = static_cast<T>(std::sqrt(n_modes));
			T scaling = param_limit_scaling * target_dist / this_dist;

			for (int m = 0; m < modes; ++m)
			{
				auto limit = scaling * parameter_ranges_(m);
				params(m) = std::min(std::max(params(m), -limit), limit);
			}	
		}
		return params;
	}

	template<typename T>
	col_vector<T> pca_model<T>::reconstruct(const col_vector<T>& params) const
	{
		DLIB_ASSERT(params.nr() <= n_params());

		col_vector<T> result;
		result.set_size(n_variables());
		for (int r = 0; r < result.nr(); ++r)
		{
			T val = 0.0f;
			for (int c = 0; c < params.nr(); ++c)
			{
				val += eigenvectors_(r, c) * params(c);
			}
			val += mean_(r);
			result(r) = val;
		}
		return result;
	}

	template<typename T>
	void pca_model<T>::core_training(const std::vector<col_vector<T>>& data, const int max_n_modes)
	{
		dlib::matrix<T> data_as_mat = col_vectors_to_single_mat(data);
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
		dlib::svd_fast(y, u, w, v, max_n_modes);
		dlib::matrix<T> eigenvalues_unsorted = pointwise_multiply(w, w);
		dlib::matrix<T> eigenvectors_unsorted = v;

		// sort according to highest eigenvalues
		std::vector<std::pair<T, int>> eigenvalue_pairs(static_cast<int>(eigenvalues_unsorted.size()));
		for (auto i_eval = 0; i_eval < static_cast<int>(eigenvalues_unsorted.size()); ++i_eval)
		{
			eigenvalue_pairs[i_eval] = std::pair<T, int>(eigenvalues_unsorted(i_eval, 0), i_eval);
		}
		std::sort(eigenvalue_pairs.rbegin(), eigenvalue_pairs.rend());
		eigenvectors_ = eigenvectors_unsorted;
		eigenvalues_ = trans(eigenvalues_unsorted);

		for (auto i_eval = 0; i_eval < static_cast<int>(eigenvalues_unsorted.size()); ++i_eval)
		{
			int index = eigenvalue_pairs[i_eval].second;
			set_colm(eigenvectors_, i_eval) = colm(eigenvectors_unsorted, index);
		}
		sqrt_eigenvalues_ = sqrt(eigenvalues_);

		if (eigenvalues_(0) <= std::numeric_limits<T>::min())
		{
			throw randomized_svd_error("While performing rsvd pca model training, the first eigenvalue was zero.");
		}
	}

	template<typename T>
	void pca_model<T>::trim_modes_for_variance(const T variance, const int max_n_modes)
	{
		const T sum_eigenvalues = sum(eigenvalues_);
		auto mode_count = 0;
		T running_total = 0;
		for (auto i_mode = 0; i_mode < static_cast<int>(eigenvalues_.nc()); ++i_mode)
		{
			mode_count++;
			running_total += eigenvalues_(i_mode);
			if ((running_total / sum_eigenvalues) >= variance)
			{
				break;
			}
		}
		if (mode_count > max_n_modes)
		{
			mode_count = max_n_modes;
		}
		eigenvectors_ = subm(eigenvectors_, 0, 0, eigenvectors_.nr(), mode_count);
		eigenvalues_ = subm(eigenvalues_, 0, 0, 1, mode_count);
		sqrt_eigenvalues_ = sqrt(eigenvalues_);
	}

	template<typename T>
	void pca_model<T>::trim_modes_for_target_error(const std::vector<col_vector<T>>& data, const T target_rms_error,
			const int max_n_modes)
	{
		bool reach_target = false;
		if (target_rms_error > 0)
		{
			auto mode_count = 1;
			for (auto i_mode = 1; i_mode < static_cast<int>(eigenvalues_.nc()); ++i_mode)
			{
				dlib::matrix<T> temp_eigenvectors = subm(eigenvectors_, 0, 0, eigenvectors_.nr(), i_mode);
				T max_rms_error = 0;
				for (const col_vector<T>& data_as_col : data)
				{
					col_vector < T > recon =
							temp_eigenvectors * (trans((trans(data_as_col) - mean_) * temp_eigenvectors))
									+ trans(mean_);
					col_vector < T > diff_vec = recon - data_as_col;
					T rms_error = std::sqrt(dlib::mean(dlib::squared(diff_vec)));
					max_rms_error = std::max(max_rms_error, rms_error);
				}
				if (max_rms_error < target_rms_error)
				{
					mode_count = i_mode;
					reach_target = true;
					break;
				}
			}

			if (mode_count > max_n_modes)
			{
				mode_count = max_n_modes;
			}

			if (!reach_target)
			{
				mode_count = eigenvalues_.size();
			}
			eigenvectors_ = subm(eigenvectors_, 0, 0, eigenvectors_.nr(), mode_count);
			eigenvalues_ = subm(eigenvalues_, 0, 0, 1, mode_count);
			sqrt_eigenvalues_ = sqrt(eigenvalues_);
		}
	}

	template<typename T>
	void pca_model<T>::find_limits(const std::vector<col_vector<T>>& data)
	{
		std::vector<col_vector<T>> params;
		params.reserve(data.size());
		for (const auto& data_item : data)
		{
			params.emplace_back(parameterize(data_item, -1, n_params()));
		}
		col_vector < T > min_params, max_params;
		element_wise_min_max(params, min_params, max_params);
		parameter_mid_values_ = (min_params + max_params) / 2;
		parameter_ranges_ = max_params - min_params;
	}

	template<typename U>
	void serialize(const pca_model<U>& item, std::ostream& out)
	{
		serialize(item.is_trained_, out);
		serialize(item.eigenvectors_, out);
		serialize(item.eigenvalues_, out);
		serialize(item.sqrt_eigenvalues_, out);
		serialize(item.mean_, out);
		serialize(item.parameter_mid_values_, out);
		serialize(item.parameter_ranges_, out);
	}

	template<typename U>
	void deserialize(pca_model<U>& item, std::istream& in)
	{
		deserialize(item.is_trained_, in);
		deserialize(item.eigenvectors_, in);
		deserialize(item.eigenvalues_, in);
		deserialize(item.sqrt_eigenvalues_, in);
		deserialize(item.mean_, in);
		deserialize(item.parameter_mid_values_, in);
		deserialize(item.parameter_ranges_, in);
	}
}