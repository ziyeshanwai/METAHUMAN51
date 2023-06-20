// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "data_utils.h"
#include "pca_model.h"
#include "data_types.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/error.h>
POSEBASEDSOLVER_RENABLE_WARNINGS
#include <vector>

namespace cm
{
    using namespace dlib;

    /*!
        Training error in randomized singular value decompostion - typically the result of a singular matrix
    */
    struct rsvd_error : public error
    {
        /*!
           Training error constructor
        */
        rsvd_error(const std::string& message) : error(message) {}
    };

    /*!
        Trains a PCA Model using randomized SVD to keep up to a target variance.
    
	    - Requires
	        - data_as_cols.size() >= 2
            - all_matrices_are_same_shape(data_as_cols) == true
	        - Every element of data_as_cols must have nr() > 0
	        - 0 <= target_variance_explained <= 1
	        - max_modes > 0

	    - Ensures
	        - Returns a PCA model, trained using the randomized SVD algorithm.

        - Throws
            -  rsvd_error
               This exception is thrown if we are unable to train the model for some 
               reason.  For example, if all the examples were identical, resulting in
               singular matrices.
	*/
    template<typename T>
    pca_model<T> train_pca_model_rsvd_to_target_variance(const std::vector<col_vector<T>>& data_as_cols,
        T target_variance,
        const int max_n_modes)
    {
        DLIB_ASSERT(data_as_cols.size() > 1 &&
			data_as_cols[0].nr() > 0 &&
			target_variance >= 0 && target_variance <= 1 &&
			max_n_modes > 0 &&
            all_col_vectors_are_same_size(data_as_cols),
			"\t randomized_pca_trainer::train_pca_model_rsvd_to_target_variance(data_as_cols,target_variance,max_n_modes)"
			<< "\n\t Invalid inputs were given to this function "
			<< "\n\t data_as_cols.size(): " << data_as_cols.size()
			<< "\n\t data_as_cols[0].nr(): " << data_as_cols[0].nr()
            << "\n\t target_variance: " << target_variance
            << "\n\t max_n_modes: " << max_n_modes
            << "\n\t all_matrices_are_same_shape(data_as_cols): " << all_col_vectors_are_same_size(data_as_cols));

		pca_model<T> model;
		matrix<T> data_as_mat = col_vectors_to_single_mat(data_as_cols);
		matrix<T> data_as_rows_local(trans(data_as_mat));
		model.mean_ = sum_rows(data_as_rows_local) / data_as_rows_local.nr();
		for (auto i_col = 0; i_col < data_as_rows_local.nc(); ++i_col)
		{
			set_colm(data_as_rows_local, i_col) = colm(data_as_rows_local, i_col) - model.mean_(i_col);
		}

        matrix<T> y = data_as_rows_local / std::sqrt(data_as_rows_local.nr() - 1);
		matrix<T> u, w, v;

		// despite svd_fast claiming to be randomized, the seed used within it is always the
		// same and it always gives the same results
		svd_fast(y, u, w, v, static_cast<unsigned long>(max_n_modes));
		matrix<T> eigenvalues_unsorted = pointwise_multiply(w, w);
		matrix<T> eigenvectors_unsorted = v;

		// sort according to highest eigenvalues
		std::vector<std::pair<T, int>> eigenvalue_pairs(static_cast<int>(eigenvalues_unsorted.size()));
		for (auto i_eval = 0; i_eval < static_cast<int>(eigenvalues_unsorted.size()); ++i_eval)
		{
			eigenvalue_pairs[i_eval] = std::pair<T, int>(eigenvalues_unsorted(i_eval, 0), i_eval);
		}
		std::sort(eigenvalue_pairs.rbegin(), eigenvalue_pairs.rend());
		model.eigenvectors_ = eigenvectors_unsorted;
		model.eigenvalues_ = trans(eigenvalues_unsorted);

		for (auto i_eval = 0; i_eval < static_cast<int>(eigenvalues_unsorted.size()); ++i_eval)
		{
			int index = eigenvalue_pairs[i_eval].second;
			set_colm(model.eigenvectors_, i_eval) = colm(eigenvectors_unsorted, index);
		}
		model.sqrt_eigenvalues_ = sqrt(model.eigenvalues_);
		//Now drop modes we don't need to hit the target variance if target_variance > 0
		if (target_variance > 0)
		{
			const T sum_eigenvalues = sum(model.eigenvalues_);
			auto mode_count = 0;
			T running_total = 0;
			for (auto i_mode = 0; i_mode < static_cast<int>(model.eigenvalues_.nc()); ++i_mode)
			{
				mode_count++;
				running_total += model.eigenvalues_(i_mode);
				if ((running_total / sum_eigenvalues) >= target_variance)
				{
					break;
				}
			}
			if (mode_count > max_n_modes)
			{
				mode_count = max_n_modes;
			}
			model.eigenvectors_ = subm(model.eigenvectors_, 0, 0, model.eigenvectors_.nr(), mode_count);
			model.eigenvalues_ = subm(model.eigenvalues_, 0, 0, 1, mode_count);
			model.sqrt_eigenvalues_ = sqrt(model.eigenvalues_);
		}

        if (model.eigenvalues_(0) <= std::numeric_limits<T>::min())
        {
            throw rsvd_error("While performing rsvd pca model training, the first eigenvalue was zero.");
        }
                            
		//Find the mid values and ranges
		std::vector<col_vector<T>> params;
		params.reserve(data_as_cols.size());

		for (const auto& data_item : data_as_cols)
		{
			params.push_back(model.parameterize(data_item, -1));
		}

		col_vector<T> min_params, max_params;
		element_wise_min_max(params, min_params, max_params);

		model.parameter_mid_values_ = (min_params + max_params) / 2;
		model.parameter_ranges_ = max_params - min_params;
		return model;
    }

	/*!
        Trains a PCA Model using randomized SVD to keep up to a target maximum rms reconstuction error
    
	    - Requires
	        - data_as_cols.size() >= 2
            - all_matrices_are_same_shape(data_as_cols) == true
	        - Every element of data_as_cols must have nr() > 0
	        - 0 <= target_rms_error
	        - max_modes > 0

	    - Ensures
	        - Returns a PCA model, trained using the randomized SVD algorithm.

        - Throws
            -  rsvd_error
               This exception is thrown if we are unable to train the model for some 
               reason.  For example, if all the examples were identical, resulting in
               singular matrices.
	*/
	
    template<typename T>
    pca_model<T> train_pca_model_rsvd_to_target_error(const std::vector<col_vector<T>>& data_as_cols,
        T target_rms_error,
        const int max_n_modes)
    {
        DLIB_ASSERT(data_as_cols.size() > 1 &&
			data_as_cols[0].nr() > 0 &&
			target_rms_error >= 0 &&
			max_n_modes > 0 &&
            all_col_vectors_are_same_size(data_as_cols),
			"\t randomized_pca_trainer::train_pca_model_rsvd_to_target_error(data_as_cols,target_rms_error,max_n_modes)"
			<< "\n\t Invalid inputs were given to this function "
			<< "\n\t data_as_cols.size(): " << data_as_cols.size()
			<< "\n\t data_as_cols[0].nr(): " << data_as_cols[0].nr()
            << "\n\t target_rms_error: " << target_rms_error
            << "\n\t max_n_modes: " << max_n_modes
            << "\n\t all_matrices_are_same_shape(data_as_cols): " << all_col_vectors_are_same_size(data_as_cols));

		pca_model<T> model;
		matrix<T> data_as_mat = col_vectors_to_single_mat(data_as_cols);
		matrix<T> data_as_rows_local(trans(data_as_mat));
		model.mean_ = sum_rows(data_as_rows_local) / data_as_rows_local.nr();
		for (auto i_col = 0; i_col < data_as_rows_local.nc(); ++i_col)
		{
			set_colm(data_as_rows_local, i_col) = colm(data_as_rows_local, i_col) - model.mean_(i_col);
		}

        matrix<T> y = data_as_rows_local / std::sqrt(data_as_rows_local.nr() - 1);
		matrix<T> u, w, v;

		// despite svd_fast claiming to be randomized, the seed used within it is always the
		// same and it always gives the same results
		svd_fast(y, u, w, v, static_cast<unsigned long>(max_n_modes));
		matrix<T> eigenvalues_unsorted = pointwise_multiply(w, w);
		matrix<T> eigenvectors_unsorted = v;

		// sort according to highest eigenvalues
		std::vector<std::pair<T, int>> eigenvalue_pairs(static_cast<int>(eigenvalues_unsorted.size()));
		for (auto i_eval = 0; i_eval < static_cast<int>(eigenvalues_unsorted.size()); ++i_eval)
		{
			eigenvalue_pairs[i_eval] = std::pair<T, int>(eigenvalues_unsorted(i_eval, 0), i_eval);
		}
		std::sort(eigenvalue_pairs.rbegin(), eigenvalue_pairs.rend());
		model.eigenvectors_ = eigenvectors_unsorted;
		model.eigenvalues_ = trans(eigenvalues_unsorted);

		for (auto i_eval = 0; i_eval < static_cast<int>(eigenvalues_unsorted.size()); ++i_eval)
		{
			int index = eigenvalue_pairs[i_eval].second;
			set_colm(model.eigenvectors_, i_eval) = colm(eigenvectors_unsorted, index);
		}
		model.sqrt_eigenvalues_ = sqrt(model.eigenvalues_);

		//Now drop modes we don't need to hit the target rms error
        bool reach_target = false;
		if (target_rms_error > 0)
		{
			auto mode_count = 1;
			for (auto i_mode = 1; i_mode < static_cast<int>(model.eigenvalues_.nc()); ++i_mode)
			{
				matrix<T> temp_eigenvectors = subm(model.eigenvectors_, 0, 0, model.eigenvectors_.nr(), i_mode);
				T max_rms_error = 0;
				for (const col_vector<T>& data_as_col : data_as_cols)
				{
					col_vector<T> recon = temp_eigenvectors * (trans((trans(data_as_col) - model.mean_) * temp_eigenvectors)) + trans(model.mean_);
					col_vector<T> diff_vec = recon - data_as_col;
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

            if(!reach_target)
            {
                mode_count = model.eigenvalues_.size();
            }
			model.eigenvectors_ = subm(model.eigenvectors_, 0, 0, model.eigenvectors_.nr(), mode_count);
			model.eigenvalues_ = subm(model.eigenvalues_, 0, 0, 1, mode_count);
			model.sqrt_eigenvalues_ = sqrt(model.eigenvalues_);
		}

        if (model.eigenvalues_(0) <= std::numeric_limits<T>::min())
        {
            throw rsvd_error("While performing rsvd pca model training, the first eigenvalue was zero.");
        }
                            
		//Find the mid values and ranges
		std::vector<col_vector<T>> params;
		params.reserve(data_as_cols.size());

		for (const auto& data_item : data_as_cols)
		{
			params.push_back(model.parameterize(data_item, -1));
		}
		
		col_vector<T> min_params, max_params;
		element_wise_min_max(params, min_params, max_params);

		model.parameter_mid_values_ = (min_params + max_params) / 2;
		model.parameter_ranges_ = max_params - min_params;
		return model;
    }
    
}
