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
	/**
   	* \defgroup Learning Machine learning
   	* @{
   	*/

	using dlib::serialize;
	using dlib::deserialize;

	/**
	 * @brief A class representing a Principal Component Analysis Model
	 * @tparam T T must be either float or double
	 */
	template<typename T>
	class pca_model
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
	public:
		/**
		 * @brief Train model to retain approximately the target variance using randomized svd method.
		 * @details Trains the PCA model to capture at least the target variance
		 * but as little as possible above the target (the model keeps a discrete
		 * number of modes so can't hit EXACTlY the target), provided it can do
		 * so by using no more than max_n_modes, otherwise it uses max_n_modes.
		 * The trainer uses the fast randomized svd method.
		 * @param data The training data as a std::vector of dlib column vectors
		 * @param variance_to_retain The target variance to retain
		 * @param max_n_modes Absolute cap on the number of modes
		 * @pre data.size() > 1
		 * @pre variance_to_retrain >= 0
		 * @pre variance_to_retrain <= 1.0
		 * @throws randomized_svd_error if the SVD falls over (e.g. all vectors equal)
		 */
		void fast_train_to_variance(const std::vector<col_vector<T>>& data, T variance_to_retain,
				unsigned long max_n_modes);

		/**
		 * @brief Train model to have at most n modes using randomized svd method.
		 * @param data The training data as a std::vector of dlib column vectors
		 * @param max_n_modes
		 * @pre data.size() > 1
		 * @pre max_n_modes >= 1
		 * @throws randomized_svd_error if the SVD falls over (e.g. all vectors equal)
		 */
		void fast_train_to_max_n_modes(const std::vector<col_vector<T>>& data, int max_n_modes);

		/**
		 * @brief Train model keeping just enough modes to hit the specified RMS error for training set.
		 * @details The trainer uses the fast randomized svd method.
		 * @param data The training data as a std::vector of dlib column vectors
		 * @param max_rms_error The RMS error to hit
		 * @param max_n_modes Absolute cap on the number of modes
		 * @pre data.size() > 1
		 * @pre max_rms_error >= 0
		 * @throws randomized_svd_error if the SVD falls over (e.g. all vectors equal)
		 */
		void fast_train_to_rms_error(const std::vector<col_vector<T>>& data, T max_rms_error, int max_n_modes);

		/**
		 * @brief Return the best set of parameters for the given data.
		 * @details The parameterization can be clamped so that the values of each parameter do not exceed the
				  corresponding maximum and minimum values for the parameter as seen when parameterizing the
		          training examples. These ranges are stored in the pca_model, having been determined during
		          training by one of the training functions. In calling this function, param_limit_scaling allows
		          you to stretch or shrink these ranges by the given factor. A value of 1.0 will clamp the output
		          such that each parameter never exceeds the training range. A value of 0.9 would clamp the output
		          10 percent more aggressively. If you pass a negative value for param_limit_scaling, no clamping is applied.
			      The default value for param_limit_scaling is -1.
		 * @param data_as_col
		 * @param param_limit_scaling Limits on the parameter values (see detailed explanation).
		 * @param n_modes returns only the first n_modes of the parameterization
		 *                or the maximum number of modes if n_modes > maximum number of modes.
		 * @return A column vector which is the PCA parametrization of the input data.
		 * @pre n_modes > 0
		 */
		col_vector<T> parameterize(const col_vector<T>& data_as_col, T param_limit_scaling, int n_modes) const;

		/**
		 * @brief Generate a reconstructed data vector from the given parameters
		 * @details The params vector can be shorter than the number of parameters in the model, in which case it is
		 *          assumed that the parameters represent the coefficients for the corresponding largest eigenvectors.
		 * @param params
		 * @pre n_params() >= 0
		 * @pre params.nr() <= n_params()
		 * @return The reconstructed data vector from the given parameters
		 */
		col_vector<T> reconstruct(const col_vector<T>& params) const;

		/**
		 * @brief The number of parameters in the model
		 * @return The number of parameters in the model
		 */
		int n_params() const
		{
			return eigenvalues_.nc();
		}

		/**
		 * @brief The number of elements in the original data vectors
		 * @return The number of elements in the original data vectors
		 */
		int n_variables() const
		{
			return mean_.nc();
		}

		/**
		 * @brief Get the mid values of the parameterizations of the training set
		 * @return The mid values of the parameters
		 */
		const col_vector<T>& parameter_mid_values() const
		{
			return parameter_mid_values_;
		}

		/**
		 * @brief Get the ranges of the parameterizations of the training set
		 * @return The ranges of the parameters
		 */
		const col_vector<T>& parameter_ranges() const
		{
			return parameter_ranges_;
		}




		/**
		 * @brief Serialization
		 * @tparam U must be either float or double
		 * @param item The PCA model
		 * @param out The output stream
		 */
		template<typename U>
		friend void serialize(const pca_model<U>& item, std::ostream& out);

		/**
		 * @brief Deserialization
		 * @tparam U must be either float or double
		 * @param item The resulting PCA model
		 * @param in The input stream
		 */
		template<typename U>
		friend void deserialize(pca_model<U>& item, std::istream& in);

	private:
		bool is_trained_ = false;
		dlib::matrix<T> eigenvectors_;
		row_vector<T> eigenvalues_;
		row_vector<T> sqrt_eigenvalues_;
		row_vector<T> mean_;
		col_vector<T> parameter_mid_values_;
		col_vector<T> parameter_ranges_;

		void core_training(const std::vector<col_vector<T>>& data, const int max_n_modes);
		void find_limits(const std::vector<col_vector<T>>& data);
		void trim_modes_for_variance(const T variance, const int max_n_modes);
		void trim_modes_for_target_error(const std::vector<col_vector<T>>& data, const T target_error, const int max_n_modes);

	};


	template<typename U>
	void serialize(const pca_model<U>& item, std::ostream& out);


	template<typename U>
	void deserialize(pca_model<U>& item, std::istream& in);



	/**@}*/
}

#include "impl/pca_model.hpp"

