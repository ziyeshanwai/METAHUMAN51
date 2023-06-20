// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "pca_model_training.h"
#include "multiple_output_prediction_model_wrapper.h"


namespace cm
{
    using namespace dlib;


	/*!
	 Multiple output prediction model wrapper trainer class.

	 - REQUIREMENTS ON T
		 - Must be either float, double, or long double, ie:
		   is_float_type<T>::value == true

	 - INITIAL VALUE

	 - WHAT THIS OBJECT REPRESENTS
		 - This object essentially groups a set of static functions for training multiple output prediction model wrappers

	*/
	template <typename T>
	class multiple_output_prediction_model_wrapper_trainer
	{
	
	public:


        /*!
    Trains a prediction_model and returns a multiple_output_prediction_model_wrapper such that it contains a prediction_model_pca_linear regressor
    using rr with optional pca basis on inputs and output

    - Requires
        - is_learning_problem(input_examples,input_targets)
        - 0 < variance_input <= 1
        - max_n_modes_input >= 0
        - 0 < variance_output <= 1
        - max_n_modes_output >= 0
    - Ensures
        - Trains the model such that it contains a prediction_model_pca_linear regressor,
        - lambda is the regularization parameter in rr. When lambda is close to zero, the result will be same as conventional regression;
        when the value of lambda is large, the coefficients will approach zero ie large regularization.
        In dlib, lambda =0 has a special meaning and will result in the model searching for an appropriate lambda.
        - If max_n_modes_input is set to 0, no pca will be performed on the input data, otherwise, this specifes the maximum number of PCA
        modes used for input data.
        - variance_input specifies the maximum fractional variance to be included in the PCA modes for the input data (if used)
        - If max_n_modes_output is set to 0, no pca will be performed on the output data, otherwise, this specifes the maximum number of PCA
        modes used for output data.
        - variance_input specifies the maximum fractional variance to be included in the PCA modes for the output data (if used)
        - compute_regression_errors specifies whether to compute the regression errors in the trained model.
    */
        static multiple_output_prediction_model_wrapper<T> train_rr(const std::vector<matrix<T, 0, 1>>& input_examples,
            const std::vector<matrix<T, 0, 1>>& input_targets,
            const std::string& random_seed,
            const double lambda,
            const double variance_input = 1.0,
            const int max_n_modes_input = 0,
            const double variance_output = 1.0,
            const int max_n_modes_output = 0,
            const bool compute_regression_errors = false
        )
        {
            prediction_model_bits<T> bits;

            perform_checks_and_pca(input_examples, input_targets, variance_input, max_n_modes_input, variance_output, max_n_modes_output, bits);

            prediction_model<T, linear_predictor<T> > prediction_model;

            prediction_model.predictors_.resize(bits.reduced_input_targets[0].size());
            bits.copy_into_prediction_model(prediction_model);


            for (unsigned i = 0; i < static_cast<unsigned>(bits.reduced_input_targets[0].size()); i++)
            {
                std::vector<T> reduced_input_targets_single(bits.reduced_input_targets.size());
                for (unsigned j = 0; j < static_cast<unsigned>(bits.reduced_input_targets.size()); j++)
                {
                    reduced_input_targets_single[j] = bits.reduced_input_targets[j](i);
                }

                double estimated_mse;

                prediction_model.predictors_[i] = train_normalized_rr<T>(
                    bits.reduced_input_examples,
                    reduced_input_targets_single,
                    random_seed,
                    lambda,
                    compute_regression_errors,
                    estimated_mse
                    );

                prediction_model.estimated_mses_[i] = estimated_mse;
                prediction_model.final_n_basis_functions_[i] = 0; // no basis functions for rr

            }

            return multiple_output_prediction_model_wrapper<T>(prediction_model);
        }

	private:
		// a private struct for sticking all the bits together prior to creating the final prediction_model
		// everything except the prediction model
		template <typename T2>
		struct prediction_model_bits
		{
			template <typename  P2>
			void copy_into_prediction_model(prediction_model<T2, P2> & model) const
			{
				model.n_variables_ = n_variables;
				model.pca_input_ = pca_input;
				model.pca_output_ = pca_output;
				model.estimated_mses_ = estimated_mses;
				model.final_gamma_multipliers_ = final_gamma_multipliers;
				model.final_cs_ = final_cs;
				model.final_n_basis_functions_ = final_n_basis_functions;
			}

			int n_variables = 0;
			pca_model<T2> pca_input;
			pca_model<T2> pca_output;
			std::vector<double> estimated_mses;
			std::vector<double> final_gamma_multipliers;
			std::vector<double> final_cs;
			std::vector<int> final_n_basis_functions;
			std::vector<matrix<T2, 0, 1>> reduced_input_examples;
			std::vector<matrix<T2, 0, 1>> reduced_input_targets;
		};

		// a helper function for performing checks on input, doing PCA and setting up various 
		// outputs shared by all implementations
		static void perform_checks_and_pca(const std::vector<matrix<T, 0, 1>>& input_examples,
			const std::vector<matrix<T, 0, 1>>& input_targets,
			const T variance_input,
			const int max_n_modes_input,
			const T variance_output,
			const int max_n_modes_output,
			prediction_model_bits<T> & bits)
		{
			DLIB_ASSERT(is_learning_problem(input_examples, input_targets));
			DLIB_ASSERT(variance_input > 0 && variance_input <= 1);
			DLIB_ASSERT(variance_output > 0 && variance_output <= 1);
			DLIB_ASSERT(max_n_modes_input >= 0);
			DLIB_ASSERT(max_n_modes_output >= 0);


			bits.n_variables = input_examples[0].nr();

			bits.reduced_input_examples.resize(input_examples.size());
			if (max_n_modes_input == 0)
			{
				bits.reduced_input_examples = input_examples;
			}
			else
			{
				bits.pca_input = train_pca_model_rsvd_to_target_variance(input_examples, variance_input, max_n_modes_input);
				for (std::size_t i = 0; i < input_examples.size(); ++i)
				{
					bits.reduced_input_examples[i] = bits.pca_input.parameterize(input_examples[i], 1.0);
				}
			}

			bits.reduced_input_targets.resize(input_targets.size());
			if (max_n_modes_output == 0)
			{
				bits.reduced_input_targets = input_targets;
			}
			else
			{
				bits.pca_output = train_pca_model_rsvd_to_target_variance(input_targets, variance_output, max_n_modes_output);
				for (std::size_t i = 0; i < input_targets.size(); ++i)
				{
					bits.reduced_input_targets[i] = bits.pca_output.parameterize(input_targets[i], 1.0);
				}
			}

			bits.estimated_mses.resize(bits.reduced_input_targets[0].size(), 0);
			bits.final_gamma_multipliers.resize(bits.reduced_input_targets[0].size(), 0);
			bits.final_cs.resize(bits.reduced_input_targets[0].size(), 0);
			bits.final_n_basis_functions.resize(bits.reduced_input_targets[0].size(), 0);
		}


	};

	

}

