// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
POSEBASEDSOLVER_RENABLE_WARNINGS
#include "data_types.h"
#include "data_utils.h"
#include "serialization_utils.h"

namespace cm
{
    /*!
        Principal components analysis model.
     
        - REQUIREMENTS ON T
	        - Must be either float, double, or long double, ie:
              is_float_type<T>::value == true
    
	    - INITIAL VALUE
            - n_variables() == 0
            - n_params() == 0
    
	    - WHAT THIS OBJECT REPRESENTS 
            - This object represents a principal components analysis model.
              Models can be trained by specfic PCA trainers such as:
				 pca_model_trainer_rsvd
    */
	template <typename T>
	class pca_model
	{
		friend class cascade_point_mask;

        template<typename U>
        friend class pca_model;

	public:
        static const int version;

	    /**
	     *  pca_model constructor
	     *
		 *   - Ensures
		 *	   - this object is properly initialized
		 */
		pca_model(
		)
		{
			COMPILE_TIME_ASSERT(is_float_type<T>::value);
		}

        /*!
            pca model copy constructor 
	    */
        template<typename U>
        pca_model(const pca_model<U>& other)
        {
            this->operator=<U>(other);
        }

        pca_model& operator = (const pca_model& other) = default;

        template<typename U>
        pca_model& operator=(const pca_model<U>& other)
        {
            eigenvectors_ = matrix_cast<T>(other.eigenvectors_);
            eigenvalues_ = matrix_cast<T>(other.eigenvalues_);
            sqrt_eigenvalues_ = matrix_cast<T>(other.sqrt_eigenvalues_);
            mean_ = matrix_cast<T>(other.mean_);
            parameter_mid_values_ = matrix_cast<T>(other.parameter_mid_values_);
            parameter_ranges_ = matrix_cast<T>(other.parameter_ranges_);
            weights_matrix_ = matrix_cast<T>(other.weights_matrix_);
            
            return *this;
        }

		/*!
		    Return the best set of parameters for the given data.
		
		    - Requires
		        - data_as_rows.nc() == n_variables()  (Number of columns in each training example)
		        - n_variables() > 0

		    - Ensures
		        - returns a column vector which is the PCA parametrization of the input data (unweighted)
		          where this column vector has n_params() rows
		        - The parameterization can be clamped so that the values of each parameter do not exceed the 
				  corresponding maximum and minimum values for the parameter as seen when parameterizing the
		          training examples. These ranges are stored in the pca_model, having been determined during
		          training by one of the training functions. In calling this function, param_limit_scaling allows
		          you to stretch or shrink these ranges by the given factor. A value of 1.0 will clamp the output
		          such that each parameter never exceeds the training range. A value of 0.9 would clamp the output
		          10 percent more aggressively. If you pass a negative value for param_limit_scaling, no clamping is applied.
			      The default value for param_limit_scaling is -1. 
		*/
		col_vector<T> parameterize(const col_vector<T>& data_as_col, T param_limit_scaling = -1) const
		{
			DLIB_ASSERT(n_variables() > 0,
				"\t pca_model::parameterize(data_as_col, param_limit_scaling)"
				<< "\n\t This function was called on what appears to be an untrained model "
				<< "\n\t n_variables(): " << n_variables());

			DLIB_ASSERT(data_as_col.nr() == n_variables(),
				"\t pca_model::parameterize(data_as_col)"
				<< "\n\t Invalid inputs were given to this function "
				<< "\n\t data_as_col.nr(): " << data_as_col.nr());

           
		    if (param_limit_scaling >= 0)
		    {
                return clamp(trans((trans(data_as_col) - mean_) * eigenvectors_),
                    parameter_mid_values() - param_limit_scaling * parameter_ranges() / 2,
                    parameter_mid_values() + param_limit_scaling * parameter_ranges() / 2
                );
		    }
		    else
		    {
			    return trans((trans(data_as_col) - mean_) * eigenvectors_);
		    }
		}

		/*!
			Return the best set of parameters for a subset of the given data.

			- Requires
				- data_as_rows.nc() == n_variables()  (Number of columns in each training example)
				- n_variables() > 0
				- subset_indices.size() <= n_variables()

			- Ensures
				- returns a column vector which is the PCA parametrization of the input data (unweighted)
				  where this column vector has n_params() rows
				- The parameterization can be clamped so that the values of each parameter do not exceed the
				  corresponding maximum and minimum values for the parameter as seen when parameterizing the
				  training examples. These ranges are stored in the pca_model, having been determined during
				  training by one of the training functions. In calling this function, param_limit_scaling allows
				  you to stretch or shrink these ranges by the given factor. A value of 1.0 will clamp the output
				  such that each parameter never exceeds the training range. A value of 0.9 would clamp the output
				  10 percent more aggressively. If you pass a negative value for param_limit_scaling, no clamping is applied.
				  The default value for param_limit_scaling is -1.
		*/
		col_vector<T> parameterize_subset(const col_vector<T>& data_as_col, const std::vector<int>& subset_indices, T param_limit_scaling = -1) const
		{
			DLIB_ASSERT(n_variables() > 0 && subset_indices.size() <= n_variables(),
				"\t pca_model::parameterize(data_as_col, param_limit_scaling)"
				<< "\n\t This function was called on what appears to be an untrained model "
				<< "\n\t subset_indices.size(): " << subset_indices.size()
				<< "\n\t n_variables(): " << n_variables());

			DLIB_ASSERT(data_as_col.nr() == n_variables() && subset_indices.size() <= n_variables(),
				"\t pca_model::parameterize(data_as_col)"
				<< "\n\t Invalid inputs were given to this function "
				<< "\n\t subset_indices.size(): " << subset_indices.size()
				<< "\n\t data_as_col.nr(): " << data_as_col.nr());

			col_vector<T> sub_data = dlib::rowm(data_as_col, dlib::mat(subset_indices));
			row_vector<T> sub_mean = dlib::colm(mean_, dlib::mat(subset_indices));
			matrix<T> sub_evecs = dlib::subm(eigenvectors_, dlib::mat(subset_indices), range(0, eigenvectors_.nc()-1));

			if (param_limit_scaling >= 0)
			{
				return clamp(trans((trans(sub_data) - sub_mean) * sub_evecs),
					parameter_mid_values() - param_limit_scaling * parameter_ranges() / 2,
					parameter_mid_values() + param_limit_scaling * parameter_ranges() / 2
				);
			}
			else
			{
				return trans((trans(sub_data) - sub_mean) * sub_evecs);
			}
		}


		/*!
			Return the best set of parameters for the given data, using pre-computed weight

			- Requires:
				- data_as_col.nr() == n_variables()  (Number of columns in each training example)
				- set_weights has been run with the appropriate sized weights vector ( n_variables() )
				- n_variables() > 0

			- Ensures:
				- Returns a row vector which is the parametrization of the input data, where the inputs are weighted, such that
				  the reconstruction will preferentially try to minimize subsequent reconstruction error per variable according
				  to the weight provided for that variable
				- The parameterization can be clamped so that the values of each parameter do not exceed the
				  corresponding maximum and minimum values for the parameter as seen when parameterizing the
				  training examples. These ranges are stored in the pca_model, having been determined during
				  training by one of the training functions. In calling this function, param_limit_scaling allows
				  you to stretch or shrink these ranges by the given factor. A value of 1.0 will clamp the output
				  such that each parameter never exceeds the training range. A value of 0.9 would clamp the output
				  10 percent more aggressively. If you pass a negative value for param_limit_scaling, no clamping is applied.
				  The default value for param_limit_scaling is -1.
		*/
		col_vector<T> parameterize_with_pre_set_weights(
			const col_vector<T>& data_as_col,
			T param_limit_scaling = -1) const
		{
			DLIB_ASSERT(n_variables() > 0,
				"\t pca_model::parameterize_with_pre_set_weights(data_as_col)"
				<< "\n\t This function was called on what appears to be an untrained model "
				<< "\n\t n_variables(): " << n_variables());

			DLIB_ASSERT(weighted_fit_precomputed_matrix_.nr() == n_variables(),
				"\t pca_model::parameterize_with_pre_set_weights(data_as_col)"
				<< "\n\t This function was called when the model doesn't have a correctly-sized precomputed matrix - have you called set_weights()? "
				<< "\n\t n_variables(): " << n_variables());


			if (param_limit_scaling >= 0)
			{
				return clamp(
					trans((trans(data_as_col) - mean_) * weighted_fit_precomputed_matrix_),
					parameter_mid_values() - param_limit_scaling * (parameter_ranges() / 2),
					parameter_mid_values() + param_limit_scaling * (parameter_ranges() / 2));
			}
			else
			{
				return trans((trans(data_as_col) - mean_) * weighted_fit_precomputed_matrix_);
			}
		}

		/*!
			Set the variable weights.

			- Requires:
				- weights.nr() == n_variables()

			- Ensures:
				- Performs advance computations so that weighted fitting runs faster. This is
				  useful if you're making repeated calls to the function with the same weights.
		*/
		void set_weights(const col_vector<T>& weights) const
		{
			DLIB_ASSERT(
				weights.nr() == n_variables(),
				"\t pca_model::set_weights(weights)"
				<< "\n\t Invalid inputs were given to this function "
				<< "\n\t n_variables(): " << n_variables()
				<< "\n\t weights.nr(): " << weights.nr());

			weighted_fit_precomputed_matrix_ = scale_rows(eigenvectors_, weights) * pinv(trans(eigenvectors_) * scale_rows(eigenvectors_, weights));
		}

		/*!
			Set the variable weights.

			- Requires:
				- weights.size() == n_variables()

			- Ensures:
				- Performs advance computations so that weighted fitting runs faster. This is
				  useful if you're making repeated calls to the function with the same weights.
		*/
		void set_weights(const std::vector<T>& weights) const
		{
			DLIB_ASSERT(
				weights.size() == n_variables(),
				"\t pca_model::set_weights(weights)"
				<< "\n\t Invalid inputs were given to this function "
				<< "\n\t n_variables(): " << n_variables()
				<< "\n\t weights.size(): " << weights.nr());

			weighted_fit_precomputed_matrix_ = scale_rows(eigenvectors_, dlib::mat(weights)) * pinv(trans(eigenvectors_) * scale_rows(eigenvectors_, dlib::mat(weights)));
		}


		/*!
			Return the best set of parameters for the given data (weighted).

			- Requires:
				- data_as_col.nr() == n_variables()  (Number of columns in each training example)
				- weights.nr() == n_variables()
				- n_variables() > 0

			- Ensures:
				- Returns a row vector which is the parametrization of the input data, where the inputs are weighted, such that
				  the reconstruction will preferentially try to minimize subsequent reconstruction error per variable according
				  to the weight provided for that variable
				- The parameterization can be clamped so that the values of each parameter do not exceed the
				  corresponding maximum and minimum values for the parameter as seen when parameterizing the
				  training examples. These ranges are stored in the pca_model, having been determined during
				  training by one of the training functions. In calling this function, param_limit_scaling allows
				  you to stretch or shrink these ranges by the given factor. A value of 1.0 will clamp the output
				  such that each parameter never exceeds the training range. A value of 0.9 would clamp the output
				  10 percent more aggressively. If you pass a negative value for param_limit_scaling, no clamping is applied.
				  The default value for param_limit_scaling is -1.
		*/
		col_vector<T> parameterize(
			const col_vector<T>& data_as_col,
			const col_vector<T>& weights,
			T param_limit_scaling = -1) const
		{
			DLIB_ASSERT(n_variables() > 0,
				"\t pca_model::parameterize(data_as_col)"
				<< "\n\t This function was called on what appears to be an untrained model "
				<< "\n\t n_variables(): " << n_variables());

			DLIB_ASSERT(data_as_col.nr() == n_variables() &&
				weights.nr() == n_variables(),
				"\t pca_model::parameterize(data_as_col,weights)"
				<< "\n\t Invalid inputs were given to this function "
				<< "\n\t data_as_col.nr(): " << data_as_col.nr()
				<< "\n\t weights.nr(): " << weights.nr());


			if (param_limit_scaling >= 0)
			{
				return clamp(
					trans((trans(data_as_col) - mean_) * scale_rows(eigenvectors_, weights) * pinv(trans(eigenvectors_) * scale_rows(eigenvectors_, weights))),
					parameter_mid_values() - param_limit_scaling * (parameter_ranges() / 2),
					parameter_mid_values() + param_limit_scaling * (parameter_ranges() / 2));
			}
			else
			{
				return trans((trans(data_as_col) - mean_) * scale_rows(eigenvectors_, weights) * pinv(trans(eigenvectors_) * scale_rows(eigenvectors_, weights)));
			}
		}

		/*
			Static helper that computes a (non-orthogonal) projector to some PCA-based (low-rank) eigenspace
			given importance weights.

			- Requires
				- weights.size() = eigenvectors.nr()

			- Ensures
				- That the non-orthonormal (due to weighting) projector is returned:

								P = inv(U'*W*U) * U' * W

				where W is the weights diagonal matrix and U is the low-rank matrix of eigenvectors
				stacked as columns.

				Note that the projector is return as transposed, hence,

								( inv(U'*W*U) * U' * W)' =  W * U * inv(U'*W*U)

		 */
		static matrix<T> compute_weighted_projector(const col_vector<T>& weights, const pca_model<T> &pca)
		{
			/*	DLIB_ASSERT(weights.size() == eigenvectors_.nr(),
					"\t pca_model::calculate_weighted_eigenvectors(weights_as_col,weights)"
					<< "\n\t Invalid inputs were given to this eigenvectors "
					<< "\n\t weights.size(): " << weights.size()
					<< "\n\t eigenvectors.nr(): " << eigenvectors_.nr());*/

			const matrix<T>& eigenvectors = pca.eigenvectors();

			matrix<T> projector = tmp(scale_rows(eigenvectors, weights) * pinv(trans(eigenvectors) * scale_rows(eigenvectors, weights)));          

			return projector;
		}

		/*
			Static helper that computes the normal equations (inverse) matrix of the
			Lagrangian of a Linearly Constrained Quadratic program associated with
			a constrained parametrization of a data vector, i.e.:

				  minimize (y - U*x)'*(y - U*x)
					x
					subject to: z - V*x = 0, where V = U(m:end, :) and U low-rank orthonormal basis.

			- Requires:
				- A map of index-to-point representing the constrained locations.
				- tdim <= pca.eigenvectors().nr().
				- constraints.size() > 0.
			 where tdim = dimensionality of constraint vector.

			- Ensures:
				- The projector from the [data ; constrained values] vectors space to
				  the space of [eigenvectors ; Lagrange multipliers] vectors is returned:

					inv([  U'*W*U (m x m)          V' (m x tdim) ]		[  U'*W (m x n) 0 (m x tdim)  ]
																	 *
						[  V (tdim x m)               0(m x m)   ])	    [  0 (tdim x n)  I (tdim x tdim)]

		*/
		static matrix<T> compute_constrained_projector(
			const std::vector<subvector_constraint<T>> &constraints,
			const pca_model<T> &pca,
			const col_vector<T> &weights = col_vector<T>()
		)
		{
			// dimensionality of the constraints
			long tdim = static_cast<long>(subvector_constraint<>::dimensionality(constraints));

			DLIB_ASSERT(
				tdim > 0 &&
				tdim <= pca.eigenvectors().nr() &&
				weights.nr() == 0 ||
				(weights.nr() == pca.eigenvectors().nr())
				,
				"\n\t dimensionality of constraints: " << tdim << " pca.eigenvectors().nr() " << pca.eigenvectors().nr()
			);

			matrix<T> W;
			if (weights.nr() > 0)
			{
				W = diagm(weights);
			}

			const matrix<T>& U = pca.eigenvectors();
			long n = U.nr(), // Original-data space dimension.
				m = U.nc();  // Rank of U (number of eigenvectors/principal components used)

		 // The tdim x m constraint matrix
			matrix<T> V(static_cast<long>(tdim), m);

			// Filling-in V in a loop with chunks of U associated with each constraint
			size_t base_row = 0; // points to the next row of V to be filled
			for (size_t i = 0; i < constraints.size(); i++)
			{
				size_t position = constraints[i].index(), // the index of the constraint in the data vector
					cdim = constraints[i].dimension(); // the dimension of the i-th constraint

			 // Assign submatrix U(position:position+cdim-1, 0, m-1) to V(base_row:base_row+cdim-1, 0:m-1)
				dlib::set_subm(V, dlib::range(static_cast<long>(base_row), static_cast<long>(base_row + cdim - 1)), dlib::range(0, static_cast<long>(m - 1))) =
					dlib::subm(U, dlib::range(static_cast<long>(position), static_cast<long>(position + cdim - 1)), dlib::range(0, static_cast<long>(m - 1)));

				base_row += cdim;
			}

			// Allocate the size following (m + tdim) x (m + tdim) matrix with zeros:
			//
			//              (NOTE: tdim is the dimensionality of the constraint vector)
			//
			// Omega = [  U'*W*U (m x m)         V' (m x tdim);
			//			   V( tdim x m)               0(m x m) ]
			//
			matrix<T> Omega(m + tdim, m + tdim); Omega = 0.0;

			// Block assignments to Omega
			if (weights.nr() == 0)
			{
				dlib::set_subm(Omega, dlib::range(0, static_cast<long>(m - 1)), dlib::range(0, static_cast<long>(m - 1))) = dlib::identity_matrix<double>(m); /* i.e., dlib::trans(U) * U */
			}
			else
			{
				dlib::set_subm(Omega, dlib::range(0, static_cast<long>(m - 1)), dlib::range(0, static_cast<long>(m - 1))) = trans(U) * W * U;
			}
			dlib::set_subm(Omega, dlib::range(0, static_cast<long>(m - 1)), dlib::range(m, static_cast<long>(m + tdim - 1))) = dlib::trans(V);
			dlib::set_subm(Omega, dlib::range(m, static_cast<long>(m + tdim - 1)), dlib::range(0, static_cast<long>(m - 1))) = V;

			// Create a (m + tdim) x (n + tdim) block matrix B = [ U' * W (m x n)      0(m x tdim);
			//											            0(tdim x n)          I(tdim x tdim) ]    ]
			matrix<T> B(m + tdim, n + tdim); B = 0.0;
			if (weights.nr() == 0)
			{
				dlib::set_subm(B, dlib::range(0, static_cast<long>(m - 1)), dlib::range(0, static_cast<long>(n - 1))) = trans(U);
			}
			else
			{
				dlib::set_subm(B, dlib::range(0, static_cast<long>(m - 1)), dlib::range(0, static_cast<long>(n - 1))) = trans(U) * W;
			}

			dlib::set_subm(B, dlib::range(static_cast<long>(m), static_cast<long>(m + tdim - 1)), 
				dlib::range(static_cast<long>(n), static_cast<long>(n + tdim - 1))) = dlib::identity_matrix<double>(tdim);

			// Return the projector
			matrix<T> projector = tmp(pinv(Omega) * B);

			return projector;
		}

		/*!
			Return the best set of parameters for the given data (weighted_eigenvectors).

			- Requires:
				- data_as_col.nr() == n_variables()  (Number of columns in each training example)
				- weighted_projector.nr() == n_variables()
				- n_variables() > 0

			- Ensures:
				- Returns a row vector which is the parametrization of the input data, where the inputs are weighted, (as above, but
				  the weights have been pre-applied to the eigenvectors, which makes it faster) such that
				  the reconstruction will preferentially try to minimize subsequent reconstruction error per variable according
				  to the weight provided for that variable
				- The parameterization can be clamped so that the values of each parameter do not exceed the
				  corresponding maximum and minimum values for the parameter as seen when parameterizing the
				  training examples. These ranges are stored in the pca_model, having been determined during
				  training by one of the training functions. In calling this function, param_limit_scaling allows
				  you to stretch or shrink these ranges by the given factor. A value of 1.0 will clamp the output
				  such that each parameter never exceeds the training range. A value of 0.9 would clamp the output
				  10 percent more aggressively. If you pass a negative value for param_limit_scaling, no clamping is applied.
				  The default value for param_limit_scaling is -1.
		*/
		col_vector<T> parameterize(
			const col_vector<T>& data_as_col,
			const matrix<T>& weighted_projector,
			T param_limit_scaling = -1) const
		{
			DLIB_ASSERT(n_variables() > 0,
				"\t pca_model::parameterize(data_as_col)"
				<< "\n\t This function was called on what appears to be an untrained model "
				<< "\n\t n_variables(): " << n_variables());

			DLIB_ASSERT(data_as_col.nr() == n_variables() &&
				weighted_projector.nr() == n_variables(),
				"\t pca_model::parameterize(data_as_col,weights)"
				<< "\n\t Invalid inputs were given to this function "
				<< "\n\t data_as_col.nr(): " << data_as_col.nr()
				<< "\n\t weighted_projector.nr(): " << weighted_projector.nr());

			if (param_limit_scaling >= 0)
			{
				return clamp(
					trans((trans(data_as_col) - mean_) * weighted_projector),
					parameter_mid_values() - param_limit_scaling * (parameter_ranges() / 2),
					parameter_mid_values() + param_limit_scaling * (parameter_ranges() / 2));
			}
			else
			{
				return trans((trans(data_as_col) - mean_) * weighted_projector);
			}
		}

		/*!
			Return the projection of a datum onto a low-rank eigenbasis with point constraints.
			The cached Lagrangian-based projector as "lagrangian_projector"

			- Requires:
				- data_as_col.nr() == n_variables()
				- data_as_col.nr() + constraints.size() == lagrangian_projector.nc()
				- eigenvectors_.nc() + constraints.size() == constrained_projector.nr() 				- constraints.size() > 0
				- n_variables() > 0

			- Ensures:
				- Returns a row vector of dimension equal to the number of eigenvectors in the PCA basis.
				- The parameterization can be clamped so that the values of each parameter do not exceed the
				  corresponding maximum and minimum values for the parameter as seen when parameterizing the
				  training examples. These ranges are stored in the pca_model, having been determined during
				  training by one of the training functions. In calling this function, param_limit_scaling allows
				  you to stretch or shrink these ranges by the given factor. A value of 1.0 will clamp the output
				  such that each parameter never exceeds the training range. A value of 0.9 would clamp the output
				  10 percent more aggressively. If you pass a negative value for param_limit_scaling, no clamping is applied.
				  The default value for param_limit_scaling is -1.
		*/
		col_vector<T> parameterize(
			const std::vector<subvector_constraint<T>> &constraints,
			const col_vector<T>& data_as_col,
			const matrix<T>& constrained_projector,
			T param_limit_scaling = -1) const
		{
			DLIB_ASSERT(n_variables() > 0,
				"\t pca_model::parameterize(data_as_col)"
				<< "\n\t This function was called on what appears to be an untrained model "
				<< "\n\t n_variables(): " << n_variables());

			DLIB_ASSERT(
				data_as_col.nr() == n_variables() &&
				data_as_col.nr() + subvector_constraint<>::dimensionality(constraints) == constrained_projector.nc() &&
				eigenvectors_.nc() + subvector_constraint<>::dimensionality(constraints) == constrained_projector.nr() &&
				constraints.size() > 0 &&
				subvector_constraint<>::dimensionality(constraints) <= eigenvectors_.nr()
				,
				"\t pca_model::parameterize(data_as_col,weights)"
				<< "\n\t Invalid inputs were given to this function "
				<< "\n\t n_variables(): " << n_variables()
				<< "\n\t data_as_col.nr(): " << data_as_col.nr()
				<< "\n\t constrained_projector.nr(): " << constrained_projector.nr()
				<< "\n\t constrained_projector.nc(): " << constrained_projector.nc()
				<< "\n\t constraints.size(): " << constraints.size()
				<< "\n\t subvector_constraint<>::dimensionality(constraints): " << subvector_constraint<>::dimensionality(constraints)
				<< "\n\t eigenvectors_.nr(): " << eigenvectors_.nr()
				<< "\n\t eigenvectors_.nc(): " << eigenvectors_.nc()

			);
			auto augmented_vector = constrained_data_vector(constraints, data_as_col);
			col_vector<T> temp = constrained_projector * augmented_vector;
			col_vector<T> constrained_params = dlib::subm(temp, dlib::range(0, eigenvectors_.nc() - 1), dlib::range(0, 0));

			if (param_limit_scaling >= 0)
			{
				return clamp(
					constrained_params,
					parameter_mid_values() - param_limit_scaling * (parameter_ranges() / 2),
					parameter_mid_values() + param_limit_scaling * (parameter_ranges() / 2)
				);
			}

			return constrained_params;
		}

		/*!
			Generate a reconstructed data vector from the given parameters

			- Requires:
				 - n_params() >= 0
				 - params.nr() == n_params()

			- Ensures:
				 - Returns a reconstructed data vector from the given parameters, the length of which will be n_variables()
		*/
		col_vector<T> reconstruct(const col_vector<T>& params) const
		{
			DLIB_ASSERT(params.nr() == n_params(),
				"\t pca_model::reconstruct(params)"
				<< "\n\t Invalid inputs were given to this function "
				<< "\n\t params.nr(): " << params.nr()
				<< "\n\t n_params(): " << n_params());

			return eigenvectors_ * params + trans(mean_);
		}

		/*!
			Is the model trained?

			- Ensures
			  - Returns true if trained, false if not
		*/
		bool is_trained() const
		{
			return mean_.nc() > 0;
		}

		/*!
			Get the number of input variables to the pca model.

			- Ensures
				- Returns the expected size of the raw data vector from which pca params are computed.
		*/
		int n_variables() const
		{
			return mean_.nc();
		}

		/*
			Get the size of pca paramater vector

			- Ensures
				- Returns the size of pca parameter vector which parameterize() returns.
		*/
		int n_params() const
		{
			return parameter_mid_values_.nr();
		}

		/*!
			Returns the eigenvectors of the PCA model

			- Ensures
				- Returns the eigenvectors of the PCA model in the order of their corresponding eigenvectors
				  as returned by eigenvalues(). Each eigenvector is a column of the returned matrix
		*/
		const matrix<T>& eigenvectors() const
		{
			return eigenvectors_;
		}

		/*!
			Returns the eigenvalues (column vector) of the PCA model.

			- Ensures
				- Returns the eigenvalues (column vector) of the PCA model, sorted by size, largest to smallest.
		*/
		col_vector<T> eigenvalues() const
		{
			return dlib::trans(eigenvalues_);
		}

		/*!
			Returns the square root of the eigenvalues (column vector) of the PCA model

			- Ensures
				- Returns the square root of the eigenvalues (column vector) of the PCA model, sorted by size, largest to smallest.
		*/
		col_vector<T> sqrt_eigenvalues() const
		{
			return dlib::trans(sqrt_eigenvalues_);
		}

		/*!
			Returns the data mean of the PCA model

			- Ensures
				- Returns the data mean of the PCA model as a row vector.
		*/
		const row_vector<T>& mean() const
		{
			return mean_;
		}

		/*!
			Returns the mid values of each parameter over the parameterization of the training data

			- Ensures
				- Returns the mid values of each parameter over the parameterization of the training data
				  as a row vector.
		*/
		const col_vector<T>& parameter_mid_values() const
		{
			return parameter_mid_values_;
		}

		/*!
			Returns the range of each parameter over the parameterization of the training data

			- Ensures
				- Returns the range each parameter over the parameterization of the training data
				  as a row vector.
		*/
		const col_vector<T>& parameter_ranges() const
		{
			return parameter_ranges_;
		}

		//! Give any trainer functions access to private data
		template<typename U>
		friend pca_model<U> train_pca_model_rsvd_to_target_variance(const std::vector<col_vector<U>>&, U, int);

		template<typename U>
		friend pca_model<U> train_pca_model_rsvd_to_target_error(const std::vector<col_vector<U>>&, U, int);

		/*!
			Provides serialization support
		*/
		friend void serialize(const pca_model<T>& item, std::ostream& out)
		{
			using dlib::serialize;
			serialize(pca_model<T>::version, out);
			serialize(item.eigenvectors_, out);
			serialize(item.eigenvalues_, out);
			serialize(item.sqrt_eigenvalues_, out);
			serialize(item.mean_, out);
			serialize(item.parameter_mid_values_, out);
			serialize(item.parameter_ranges_, out);
			serialize(item.weights_matrix_, out);
		}

		/*!
			Provides deserialization support
		*/
		friend void deserialize(pca_model<T>& item, std::istream& in)
		{
			auto curversion = 0;
			using dlib::deserialize;
			deserialize(curversion, in);
			check_serialization_versioning("pca_model", pca_model<T>::version, curversion);

			deserialize(item.eigenvectors_, in);
			deserialize(item.eigenvalues_, in);
			deserialize(item.sqrt_eigenvalues_, in);
			deserialize(item.mean_, in);
			deserialize(item.parameter_mid_values_, in);
			deserialize(item.parameter_ranges_, in);
			deserialize(item.weights_matrix_, in);
		}

	private:
		matrix<T> eigenvectors_;
		row_vector<T> eigenvalues_;
		row_vector<T> sqrt_eigenvalues_;
		row_vector<T> mean_;
		col_vector<T> parameter_mid_values_;
		col_vector<T> parameter_ranges_;
		mutable matrix<T> weights_matrix_;

		//No need to serialize this
		mutable matrix<T> weighted_fit_precomputed_matrix_;

		/*
			A method that returns the augmented data vector with constraint based data
			in the case where constraints have been specified, in order to use it as
			data vector in the solution of the augmented Lagrangian function optimum.

			- Ensures:
						The return vector is a CENTRALIZED (i.e., mean is subtracted from both the datum and the augmented_datum = [datum; x] where x is the vector of
						constrained coordinates (and is a subvector of datum).
			- Requires:
						Since x is a subvector of datum, it follows that dimensionality(constraints) <= datum.nr()

			*/

		inline col_vector<T> constrained_data_vector(
			const std::vector<subvector_constraint<T>> &constraints,
			const col_vector<T> &datum) const
		{
			size_t tdim = subvector_constraint<>::dimensionality(constraints);

			DLIB_ASSERT(tdim <= datum.nr(),
				"\n\t dimensionality of constraints " << tdim << "datum.nr() "
				<< datum.nr()
			);

			size_t n = datum.nr();
			col_vector<T> mu_ = trans(mean_); // for use as an upright cector
			col_vector<T> augmented_datum(static_cast<long>(n + tdim)); // the augmented data vector (to be filled shortly...)

			// set the augmented_datum submatrix
			dlib::set_subm(augmented_datum, dlib::range(0, static_cast<long>(n - 1)), dlib::range(0, 0)) = datum - mu_;
			// now insert the remaining components from the constraints
			size_t vector_index = n;
			for (size_t i = 0; i < constraints.size(); i++)
			{
				size_t cdim = constraints[i].dimension();
				dlib::set_subm(augmented_datum, dlib::range(static_cast<long>(vector_index), static_cast<long>(vector_index + cdim - 1)), dlib::range(0, 0)) =
					constraints[i].constraint_vector() - dlib::subm(mu_, dlib::range(static_cast<long>(constraints[i].index()), static_cast<long>(constraints[i].index() + cdim - 1)), dlib::range(0, 0));
				vector_index += cdim;
			}
			return augmented_datum;
		}
	};

	template <typename T>
	const int pca_model<T>::version = 1;
}
