// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "learning_method.h"
#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>
#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/global_optimization/find_max_global.h>
#include <dlib/svm.h>
#include <dlib/svm/cm_function.h>
#include <dlib/svm/cm_kernel.h>
#include <dlib/svm/cm_rr_trainer.h>
#include <dlib/statistics/cm_statistics.h>
POSEBASEDSOLVER_RENABLE_WARNINGS


namespace cm
{


    /*!
    Train a ridge regression with built in normalization

    - REQUIREMENTS ON T
        - Must be either float, double, or long double, i.e.:
          is_float_type<T>::value == true
    - Requires
        - is_learning_problem(input_examples,input_targets)

    - Ensures
       - Returns a linear_predictor (a typedef for normalized_function<decision_function<cm_linear_kernel<dlib::matrix<T, 0, 1>>>> )
       - The predictor will automatically normalize input data before regressing.
       - "estimated_mse" will be set to an estimate of the regressors cross-validated error.
    */
    template <typename T>
    linear_predictor<T> train_normalized_rr(
        const std::vector<matrix<T, 0, 1>>& input_examples,
        const std::vector<T>& input_targets,
        const std::string& random_seed,
        double lambda,
        bool get_mse,
        double& estimated_mse
    )
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);
        DLIB_ASSERT(is_learning_problem(input_examples, input_targets)
            ,
            "\t train_normalized_rr(...)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t (input_examples.size(): " << input_examples.size()
            << "\n\t (input_targets.size(): " << input_targets.size());

        auto examples(input_examples);
        auto targets(input_targets);

        if (random_seed.empty())
        {
            randomize_samples(examples, targets);
        }
        else
        {
            auto rnd = dlib::rand(random_seed);
            randomize_samples(examples, targets, rnd);
        }

        cm_vector_normalizer<dlib::matrix<T, 0, 1>> normalizer;
        normalizer.train(examples);
        for (auto& example : examples)
        {
            example = normalizer(example);
        }

        cm_rr_trainer<cm_linear_kernel<matrix<T, 0, 1>>> final_trainer;
        final_trainer.set_lambda(lambda);
        linear_predictor<T> final_model;
        final_model.normalizer = normalizer;
        final_model.function = final_trainer.train(examples, targets);
        if (get_mse)
        {
            const auto diagnostics = cross_validate_regression_trainer(final_trainer, examples, targets, 4);
			estimated_mse = diagnostics(0);
        }
		else
		{
			estimated_mse = 0;
		}
        return final_model;
    }



	/*!
        Train an linear svr regression with built in normalization
    
        - REQUIREMENTS ON T
            - Must be either float, double, or long double, i.e.:
              is_float_type<T>::value == true
        - Requires
            - is_learning_problem(input_examples,input_targets)
    
        - Ensures
           - Returns an rbf_predictor (a typedef for normalized_function<decision_function<cm_radial_basis_kernel<dlib::matrix<T, 0, 1>>>> )
           - The predictor will automatically normalize input data before regressing.
           - "estimated_mse" will be set to an estimate of the regressors cross-validated error.
    */
    template <typename T>
    linear_predictor<T> train_normalized_linear_svr(
        const std::vector<matrix<T, 0, 1>>& input_examples,
        const std::vector<T>& input_targets,
        const std::string& random_seed,
        double c,
        double epsilon_insensitivity,
        bool get_mse,
        double& estimated_mse
    )
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);
        DLIB_ASSERT(is_learning_problem(input_examples, input_targets)
            ,
            "\t train_normalized_rbf_svr(...)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t (input_examples.size(): " << input_examples.size()
            << "\n\t (input_targets.size(): " << input_targets.size());

        auto examples(input_examples);
        auto targets(input_targets);

        if (random_seed.empty())
        {
            randomize_samples(examples, targets);
        }
        else
        {
            auto rnd = dlib::rand(random_seed);
            randomize_samples(examples, targets, rnd);
        }

		cm_vector_normalizer<dlib::matrix<T, 0, 1>> normalizer;
		normalizer.train(examples);
        for (auto& example : examples)
        {
            example = normalizer(example);
        }
   
		svr_linear_trainer<cm_linear_kernel<matrix<T, 0, 1>>> final_trainer;
		final_trainer.set_c(c);
		final_trainer.set_epsilon_insensitivity(epsilon_insensitivity);
        
		linear_predictor<T> final_model;
        final_model.normalizer = normalizer; 
		final_model.function = final_trainer.train(examples, targets);
		if(get_mse)
        {
            const auto diagnostics = cross_validate_regression_trainer(final_trainer, examples, targets, 4);
            estimated_mse = diagnostics(0);
        }
        else
        {
            estimated_mse = 0;
        }
        return final_model;
    }


	/*
		Find the best n features based on the cosine of the angles between each drivers vector and the targets vector

		- REQUIREMENTS ON T
			 - Must be either float, double, or long double, i.e.:
				is_float_type<T>::value == true

		- Requires
			- is_learning_problem(drivers,targets)==true
			- feature_fraction > 0.0
			- feature_fraction <= 1.0

		- Ensures
			- Returns feature_fraction*number of drivers indices indicating the most correlated drivers with the given targets, based on the cosine of the angles between each drivers vector and the targets vector
	*/
	template <typename T>
	std::vector<int> best_dot_features(const std::vector<matrix<T, 0, 1>>& drivers, const std::vector<T>& targets, const int n)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);
		DLIB_ASSERT(is_learning_problem(drivers, targets)
			&& n > 0
			&& n <= drivers[0].nr(),
			"\t best_cubic_merit_features((drivers,targets, selected_features)"
			<< "\n\t Invalid inputs were given to this function "
			<< "\n\t n: " << n
			<< "\n\t is_learning_problem(drivers,targets): " << is_learning_problem(drivers, targets));

		unsigned num_examples = static_cast<unsigned>(drivers.size());
		unsigned num_ordinates = static_cast<unsigned>(drivers.begin()->nr());
		std::vector<int> selected_features(num_ordinates);
		std::iota(selected_features.begin(), selected_features.end(), 0);

		std::vector<T> univariate_coefficients(num_ordinates);
		for (unsigned o = 0; o < num_ordinates; ++o)
		{
			T numerator_dot = 0.0;
			T denominator_dot = 0.0;
			for (unsigned ex = 0; ex < num_examples; ++ex)
			{
				numerator_dot += drivers[ex](o) * targets[ex];
				denominator_dot += drivers[ex](o) * drivers[ex](o);
			}
			univariate_coefficients[o] = std::abs(numerator_dot / std::sqrt(denominator_dot));
		}
		std::sort(selected_features.rbegin(), selected_features.rend(), [&](int lhs, int rhs) -> bool {return univariate_coefficients[lhs] < univariate_coefficients[rhs]; });
		selected_features.resize(static_cast<unsigned>(n));
		return selected_features;
	}
}
