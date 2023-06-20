// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "learning.h"
#include "prediction_model_wrapper.h"
#include "multiple_output_prediction_model_wrapper_trainer.h"

namespace cm
{
    using namespace dlib;


	/*!
	 Prediction model wrapper trainer class.

	 - REQUIREMENTS ON T
		 - Must be either float, double, or long double, ie:
		   is_float_type<T>::value == true

	 - INITIAL VALUE

	 - WHAT THIS OBJECT REPRESENTS
		 - This object essentially groups a set of static functions for training single output prediction model wrappers

	*/
	template <typename T>
	class prediction_model_wrapper_trainer
	{

	public:

        /*!
        Trains a prediction_model and returns a prediction_model_wrapper such that it contains a prediction_model_pca_linear regressor
        using rr with an optional pca basis on the input data and output data.

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
            - The function tries to optimize for the best number of basis functions from the list provided
            - If max_n_modes_input is set to 0, no pca will be performed on the input data, otherwise, this specifes the maximum number of PCA
            modes used for input data.
            - variance_input specifies the maximum fractional variance to be included in the PCA modes for the input data (if used)
            - If max_n_modes_output is set to 0, no pca will be performed on the output data, otherwise, this specifes the maximum number of PCA
            modes used for output data.
            - variance_input specifies the maximum fractional variance to be included in the PCA modes for the output data (if used)
            - compute_regression_errors specifies whether to compute the regression errors in the trained model.
        */
        static prediction_model_wrapper<T> train_rr(const std::vector<matrix<T, 0, 1>>& input_examples,
            const std::vector<T>& input_targets,
            const std::string& random_seed,
            const double lambda,
            const double variance_input = 1.0,
            const int max_n_modes_input = 0,
            const double variance_output = 1.0,
            const int max_n_modes_output = 0,
            const bool compute_regression_errors = false
        )
        {
            DLIB_ASSERT(is_learning_problem(input_examples, input_targets));

            // call the multiple predictor version of the code with the input converted to a vector
            std::vector<matrix<T, 0, 1>> input_targets_vectors(input_targets.size());
            for (unsigned i = 0; i < input_targets.size(); i++)
            {
                matrix<T, 1, 1> cur_vec;
                cur_vec(0, 0) = input_targets[i];
                input_targets_vectors[i] = cur_vec;
            }
            return prediction_model_wrapper<T>(multiple_output_prediction_model_wrapper_trainer<T>::train_rr(input_examples, input_targets_vectors,
                random_seed, lambda, variance_input, max_n_modes_input, variance_output, max_n_modes_output, compute_regression_errors));

        }


	};

	

}



