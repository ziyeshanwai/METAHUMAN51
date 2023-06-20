// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "enum_ext.h"
#include "pca_model.h"
#include "data_types.h"
#include "prediction_model_wrapper_trainer.h"
#include <functional>
#include <algorithm>
#include <dlib/error.h>
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_RENABLE_WARNINGS

namespace cm
{
	//! Regressor types
	DECLARE_ENUM(ms_regression_options, ridge_regression)

	//! Kernel types
	DECLARE_ENUM(ms_kernel_options, linear)

	//! Cross-validation metric
	DECLARE_ENUM(ms_cross_validation_metric, absolute_average, absolute_maximum)

    /*
        optimal regressor params struct

        - WHAT THIS OBJECT REPRESENTS
            - This objects represents the combination of training parameters which as an ensemble gives minimised global/max mean square error
    */
    template <typename T>
    struct optimal_regressor_params
    {
        T input_variance;
        int input_max_modes;
        T lambda;
        std::pair<T, std::vector<int>> selected_feature_fraction;
        T cross_validation_error;
		std::vector<T> example_errors;
		optimal_regressor_params():
			input_variance(1.0),
			input_max_modes(std::numeric_limits<int>::max()),
			lambda(0.0),
			cross_validation_error(0.0)
		{
			cross_validation_error = std::numeric_limits<T>::max();
		}

		/*
			friend function for serialization
		*/
		friend void serialize(const optimal_regressor_params& orp, std::ostream& out)
		{
			dlib::serialize(optimal_regressor_params::version, out);
			dlib::serialize(orp.input_variance, out);
			dlib::serialize(orp.input_max_modes, out);
			dlib::serialize(orp.lambda, out);
			dlib::serialize(orp.selected_feature_fraction, out);
			dlib::serialize(orp.cross_validation_error, out);
			dlib::serialize(orp.example_errors, out);
		}

		/*
			friend function for deserialization
		*/
		friend void deserialize(optimal_regressor_params& orp, std::istream& in)
		{
			unsigned cur_version;
			dlib::deserialize(cur_version, in);

			// current version deserialization
			// IMPORTANT: please try and make class serialization / deserialization back-compatible
			// by supporting multiple versions if possible, and if not, function should throw an exception
			if (cur_version == 1u)
			{
				dlib::deserialize(orp.input_variance, in);
				dlib::deserialize(orp.input_max_modes, in);
				dlib::deserialize(orp.lambda, in);
				dlib::deserialize(orp.selected_feature_fraction, in);
				dlib::deserialize(orp.cross_validation_error, in);
				dlib::deserialize(orp.example_errors, in);
			}
			else
			{
				// back-compatibility code for previous versions should go here
				throw serialization_version_error("optimal_regressor_params", optimal_regressor_params::version, cur_version);
			}
		}
	private:
		static const unsigned version;
	};

	template <typename T>
	class modular_solver_regressor_factory
	{
	public:
		struct modular_solver_error : public dlib::error
		{
			modular_solver_error(const std::string& message) : dlib::error(message) {}
		};

	private:
		static const std::string random_seed_;
		static const size_t max_folds_;

		static prediction_model_wrapper<T> train_rr_regressor(ms_kernel_options kernel_type,
			const std::vector<col_vector<T>>& input_examples,
			const std::vector<T>& input_targets,
			const std::vector<T>& input_pca_variances_to_try,
			const std::vector<int>& input_pca_max_modes_to_try,
			const std::vector<T>& feature_fractions_to_try,
			const std::vector<T>& lambdas_to_try,
            optimal_regressor_params<T>& optimal_params,
			bool compute_regression_errors,
			ms_cross_validation_metric metric)
		{
			if (!compute_regression_errors && feature_fractions_to_try.size() * input_pca_variances_to_try.size() * input_pca_max_modes_to_try.size() * lambdas_to_try.size() > 1)
			{
				throw modular_solver_error("Unable to cross validate training parameters when there is only one set of training parameters.");
			}
			const auto num_folds = std::min(input_examples.size(), max_folds_);

			switch (kernel_type)
			{
			case ms_kernel_options::linear:
			{
				if (compute_regression_errors)
				{
					T feature_fraction, variance, lambda;
					int max_modes;
					std::function<prediction_model_wrapper<T>(const std::vector<col_vector<T>>&, const std::vector<T>&)> trainer = [&](const std::vector<col_vector<T>>& train_examples, const std::vector<T>& train_targets) {
						return prediction_model_wrapper_trainer<T>::train_rr(
							train_examples,
							train_targets,
							random_seed_,
							lambda,
							variance,
							max_modes,
							1.0,
							0,
							false);
					};
					T best_error = std::numeric_limits<T>::max();
					for (const auto& ff : feature_fractions_to_try)
					{
						feature_fraction = ff;
						for (const auto& var : input_pca_variances_to_try)
						{
							variance = var;
							for (const auto& mm : input_pca_max_modes_to_try)
							{
								max_modes = mm;
								for (const auto& lam : lambdas_to_try)
								{
									lambda = lam;
									T fold_error = cross_validate_parameters(input_examples, input_targets, feature_fraction, metric, static_cast<int>(num_folds), trainer);
									if (fold_error < best_error)
									{
										best_error = fold_error;
										optimal_params.cross_validation_error = fold_error;
										optimal_params.lambda = lambda;
										optimal_params.input_variance = variance;
										optimal_params.input_max_modes = max_modes;
										optimal_params.selected_feature_fraction.first = feature_fraction;
									}
								}
							}
						}
					}
				}
				else
				{
					optimal_params.input_max_modes = input_pca_max_modes_to_try[0];
					optimal_params.input_variance = input_pca_variances_to_try[0];
					optimal_params.lambda = lambdas_to_try[0];
					optimal_params.selected_feature_fraction.first = feature_fractions_to_try[0];
				}

				const unsigned num_examples = static_cast<unsigned>(input_examples.size());
				const unsigned num_ordinates = static_cast<unsigned>(input_examples.begin()->size());
				const unsigned num_features = static_cast<unsigned>(num_ordinates * optimal_params.selected_feature_fraction.first);
				std::vector<col_vector<T>> feature_selected_examples(num_examples, col_vector<T>(num_features));
				if (optimal_params.selected_feature_fraction.first == 0.0)
				{
					// if feature fraction  == 0.0, use the whole unmodified input example data
					// Note that this is distinct from using a feature fraction of 1.0, which will use all of the input example data, but will re-order them based on calculated correlation
					feature_selected_examples = input_examples;
				}
				else
				{
					// otherwise compute the correlation between the input and output, and store the indices
					optimal_params.selected_feature_fraction.second = best_dot_features(input_examples, input_targets, num_features);
					for (unsigned example = 0; example < num_examples; ++example)
					{
						for (unsigned feature = 0; feature < num_features; ++feature)
						{
							feature_selected_examples[example](feature) = input_examples[example](optimal_params.selected_feature_fraction.second[feature]);
						}
					}
				}
				prediction_model_wrapper<T> pm = prediction_model_wrapper_trainer<T>::train_rr(
					feature_selected_examples,
					input_targets,
					random_seed_,
					optimal_params.lambda,
					optimal_params.input_variance,
					optimal_params.input_max_modes,
					1.0,
					0,
					false);
				optimal_params.example_errors.resize(num_examples);
				for (unsigned ex = 0; ex < num_examples; ++ex)
				{
					optimal_params.example_errors[ex] = pm.predict(feature_selected_examples[ex]) - input_targets[ex];
				}
				return pm;
			}
			default:
				throw modular_solver_error("Unrecognised kernel type");
			}
		}

		static T cross_validate_parameters(const std::vector<col_vector<T>>& input_examples,
			const std::vector<T>& input_targets,
			const T& feature_fraction,
			const ms_cross_validation_metric metric,
			const int num_folds,
			std::function<prediction_model_wrapper<T>(const std::vector<col_vector<T>>&, const std::vector<T>&)> trainer)
		{
			const unsigned num_examples = static_cast<unsigned>(input_examples.size());
			const unsigned num_ordinates = static_cast<unsigned>(input_examples.begin()->size());
			const unsigned chunk = num_examples / num_folds;
			const unsigned num_features = static_cast<unsigned>(num_ordinates * feature_fraction);

			running_stats<T> rs_abs;
			running_stats<T> rs_sq;
			running_scalar_covariance<T> rc;
			for (unsigned fold = 0; fold < static_cast<unsigned>(num_folds); ++fold)
			{
				const unsigned test_start_index = fold * chunk;
				const unsigned test_end_index = fold == static_cast<unsigned>(num_folds - 1) ? num_examples : (fold + 1) * chunk;
				const unsigned num_test_examples = test_end_index - test_start_index;
				std::vector<col_vector<T>> fold_train_examples(static_cast<size_t>(num_examples - num_test_examples), col_vector<T>(num_ordinates));
				std::vector<T> fold_train_targets(static_cast<size_t>(num_examples - num_test_examples));
				std::vector<T> fold_test_targets(static_cast<size_t>(num_test_examples));

				// apply randomization to examples + targets
				std::vector<unsigned> random_indices(num_examples);
				std::vector<unsigned> test_indices(num_test_examples);
				std::iota(random_indices.begin(), random_indices.end(), 0);
				dlib::rand rng(random_seed_);
				randomize_samples(random_indices, rng);
				for (unsigned i = 0; i < test_start_index; ++i)
				{
					fold_train_examples[i] = input_examples[random_indices[i]];
					fold_train_targets[i] = input_targets[random_indices[i]];
				}
				for (unsigned i = test_start_index; i < test_end_index; ++i)
				{
					test_indices[i - test_start_index] = random_indices[i];
					fold_test_targets[i - test_start_index] = input_targets[random_indices[i]];
				}
				for (unsigned i = test_end_index; i < num_examples; ++i)
				{
					fold_train_examples[i - num_test_examples] = input_examples[random_indices[i]];
					fold_train_targets[i - num_test_examples] = input_targets[random_indices[i]];
				}
				std::vector<col_vector<T>> feature_selected_fold_train_examples(num_examples - num_test_examples, col_vector<T>(num_features));
				std::vector<col_vector<T>> feature_selected_fold_test_examples(num_test_examples, col_vector<T>(num_features));
				if (feature_fraction == 0.0)
				{
					feature_selected_fold_train_examples = fold_train_examples;
					for (unsigned ex = 0; ex < num_examples; ++ex)
					{
						feature_selected_fold_test_examples[ex] = input_examples[test_indices[ex]];
					}
				}
				else
				{
					// if feature fraction  == 0.0, use the whole unmodified input example data
					// Note that this is distinct from using a feature fraction of 1.0, which will use all of the input example data, but will re-order them based on calculated correlation
					// otherwise compute the correlation between the input and output, and store the indices
					std::vector<int> feature_selection_indices = best_dot_features(fold_train_examples, fold_train_targets, num_features);
					for (unsigned ex = 0; ex < num_examples - num_test_examples; ++ex)
					{
						for (unsigned f = 0; f < feature_selection_indices.size(); ++f)
						{
							feature_selected_fold_train_examples[ex](f) = fold_train_examples[ex](feature_selection_indices[f]);
						}
					}
					for (unsigned ex = 0; ex < num_test_examples; ++ex)
					{
						for (unsigned f = 0; f < feature_selection_indices.size(); ++f)
						{
							feature_selected_fold_test_examples[ex](f) = input_examples[test_indices[ex]](feature_selection_indices[f]);
						}
					}
				}
				prediction_model_wrapper<T> predictor = trainer(feature_selected_fold_train_examples, fold_train_targets);
				for (unsigned i = 0; i < num_test_examples; ++i)
				{
					T result = predictor.predict(feature_selected_fold_test_examples[i]);
					T diff = fold_test_targets[i] - result;
					rs_abs.add(std::abs(diff));
					rs_sq.add(diff * diff);
					rc.add(result, fold_test_targets[i]);
				}
			}
			switch (metric)
			{
			case ms_cross_validation_metric::absolute_average:
				return rs_abs.mean();
				break;
			case ms_cross_validation_metric::absolute_maximum:
				return rs_abs.max();
				break;
			default:
				throw modular_solver_error("Unrecognised cross-validation error metric option.");
			}
		}
	public:

		static prediction_model_wrapper<T> regressor_training_factory(ms_regression_options regression_type,
			ms_kernel_options kernel_type,
			const std::vector<col_vector<T>>& input_examples,
			const std::vector<T>& input_targets,
			const std::vector<T>& input_pca_variances_to_try,
			const std::vector<int>& input_pca_max_modes_to_try,
			const std::vector<T>& feature_fractions_to_try,
			const std::vector<T>& lambdas_to_try,
			optimal_regressor_params<T>& optimal_params,
			bool compute_regression_errors,
			ms_cross_validation_metric metric)
		{
			try
			{
				switch (regression_type)
				{
				case ms_regression_options::ridge_regression:
					return train_rr_regressor(kernel_type,
						input_examples,
						input_targets,
						input_pca_variances_to_try,
						input_pca_max_modes_to_try,
						feature_fractions_to_try,
						lambdas_to_try,
						optimal_params,
						compute_regression_errors,
						metric);
				default:
					throw modular_solver_error("Unrecognised regression option.");
				}
			}
			catch (std::exception const& err)
			{
				throw modular_solver_error(std::string("Exception in modular_solver_regressor_factory.h: ") + err.what());
			}
		}
	};

	template <typename T>
	const std::string modular_solver_regressor_factory<T>::random_seed_ = "CUBIC";
	template <typename T>
	const size_t modular_solver_regressor_factory<T>::max_folds_ = 4;
	template <typename T>
	const unsigned optimal_regressor_params<T>::version = 1;
}
