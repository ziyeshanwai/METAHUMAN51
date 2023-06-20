// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>

#include "modular_solver.h"
#include "data_types.h"
#include "data_utils.h"
#include "transforms.h"
#include "pca_model_training.h"
#include "modular_solver_regressor_factory.h"
#include "general_geometry.h"
#include <vector>
#include <map>
#include <sstream>
#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/assert.h>
#include <dlib/md5.h>
POSEBASEDSOLVER_RENABLE_WARNINGS

namespace cm
{
	/*
		modular solver trainer class

		- REQUIREMENTS ON T
			- Must be either float, double, or long double, ie:
			is_float_type<T>::value == true

		- INITIAL VALUE

		- WHAT THIS OBJECT REPRESENTS
			- This object essentially groups a set of static functions for training modular_solvers
	*/
	template <typename T, int D = 2>
	class modular_solver_trainer
	{
	public:
		using P = dlib::vector<T, D>;
		struct modular_solver_error : public dlib::error
		{
			/*!
			   Training error constructor
			*/
			modular_solver_error(const std::string& message) : dlib::error(message) {}
		};

		/*
			training diagnostic information
		*/
		struct training_delta
		{
			std::string control_name;
			T fractional_delta = 0.0;
			int example_index = -1;
		};

		/*
			Trains and returns a modular solver with regressors and output PCA trained with parameters that minimise tatal/max regressor mean square error (toggled by
			minimise_global_error).	Type of regressors specified by choice of ms_regression_options and ms_kernel_options

			- Requires
				- frames_shaoes_tracking.size() == frames_controls.size()
				- all elements of frames_shapes_tracking contain the same number of views
				- all examples of any particular view must contain all point IDs from the corresponding view in views_pointids
				- every control values example must contain the same control IDs

			- Ensures
				- Trains the modular solver such that it contains a set of regressors (with inbuilt input PCA), point IDs and base shapes for each view, and IDs,
				minimum, maximum, and default values for each control, and MD5 hash of training tracking examples, training control examples, and training parameters
				- The function tries to optimise for the best:
					- input PCA max modes
					- input	PCA variance
					- feature fraction
					- lambda (as appropriate for choice of regressor type)
					- feature fraction
					that produce the lowest total/max cross-validation error
				- Training deltas for each trained regressor returned in parameter training_delta
		*/
		static modular_solver<T, D> train(const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking,
			const std::vector<std::map<std::string, T>>& frames_controls,
			const std::map<std::string, typename modular_solver<T, D>::control_range>& control_ranges,
			const std::vector<std::pair<typename modular_solver<T, D>::alignmentD_type, std::vector<std::vector<std::string>>>>& shapes_alignment_views_pointids,
			const typename modular_solver<T, D>::training_params& params,
			std::vector<training_delta>& training_deltas)
		{
			modular_solver<T, D> ms = modular_solver<T, D>();
			verify_params(params);
			// define number of output predictions
			ms.n_var_output_ = static_cast<unsigned>(control_ranges.size());
			// set ID and min/max/default values for each control
			ms.control_ranges_ = control_ranges;
			// record md5 of training tracking, controls, and parameters
			compute_training_md5s(ms, frames_shapes_tracking, frames_controls, params, control_ranges);

			ms.cross_validated_params_.orig_training_params = params;

			verify_inputs(ms, frames_shapes_tracking, frames_controls, shapes_alignment_views_pointids);
			ms.num_views_ = static_cast<unsigned>(shapes_alignment_views_pointids.begin()->second.size());

			train_shape_models(ms, shapes_alignment_views_pointids, frames_shapes_tracking, params.compression.shape_pca_variance, 999);

			std::vector<col_vector<T>> input_examples;
			std::vector<col_vector<T>> input_targets;
			// alignment and concatenation of input_examples, and converting from map to col_vector for both input_examples and input_targets
			process_inputs(ms, input_examples, input_targets, frames_shapes_tracking, frames_controls);

			ms.regressors_.resize(ms.n_var_output_);
			ms.cross_validated_params_.regressor_params.resize(ms.n_var_output_);

			train_regressors_cross_validation(ms, input_examples, input_targets, params, ms.cross_validated_params_.regressor_params);

			//imputation
			ms.cross_validated_params_.imputation_regressor_params.resize(params.imputation.num_iterations, std::vector<optimal_regressor_params<T>>(ms.n_var_output_));
			for (unsigned i = 0; i < params.imputation.num_iterations; ++i)
			{
				modify_target_examples(ms, input_examples, input_targets, control_ranges, i);
				train_regressors_cross_validation(ms, input_examples, input_targets, params, ms.cross_validated_params_.imputation_regressor_params[i]);
			}

			ms.output_constraint_pca_ = train_pca_model_rsvd_to_target_variance<T>(input_targets, 0.995, ms.n_var_output_);

			compute_training_diagnostics(ms, static_cast<int>(frames_shapes_tracking.size()), static_cast<int>(control_ranges.size()), training_deltas);

			ms.is_trained_ = true;
			return ms;
		}

		/*
			Trains and returns a modular solver with regressors and output PCA trained with previously established cross-validated parameters

			- Requires
				- frames_shapes_tracking.size() == frames_controls.size()
				- all elements of frames_shapes_tracking contain the same number of views
				- all examples of any particular view must contain the same point IDs
				- every control values example must contain the same control IDs

			- Ensures
				- Trains the modular solver such that it contains a set of regressors (with inbuilt input PCA), point IDs and base shapes for each view, and IDs,
				minimum, maximum, and default values for each control, and MD5 hash of training tracking examples, training control examples, and training parameters
				- regressor parameters are dictated by input prev_optimal_params
				- training deltas for each trained regressor returned in parameter training_delta
		*/
		static modular_solver<T, D> train(const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking,
			const std::vector<std::map<std::string, T>>& frames_controls,
			const std::map<std::string, typename modular_solver<T, D>::control_range>& control_ranges,
			const std::vector<std::pair<typename modular_solver<T, D>::alignmentD_type, std::vector<std::vector<std::string>>>>& shapes_alignment_views_pointids,
			const typename modular_solver<T, D>::optimal_params& prev_optimal_params,
			std::vector<training_delta>& training_deltas)
		{
			modular_solver<T, D> ms = modular_solver<T, D>();
			// define number of output predictions
			ms.n_var_output_ = static_cast<unsigned>(control_ranges.size());
			// set ID and min/max/default values for each control
			ms.control_ranges_ = control_ranges;
			// record md5 of training tracking, controls, and parameters

			ms.cross_validated_params_.orig_training_params = prev_optimal_params.orig_training_params;

			verify_inputs(ms, frames_shapes_tracking, frames_controls, shapes_alignment_views_pointids);
			ms.num_views_ = static_cast<unsigned>(shapes_alignment_views_pointids.begin()->second.size());

			train_shape_models(ms, shapes_alignment_views_pointids, frames_shapes_tracking, prev_optimal_params.orig_training_params.compression.shape_pca_variance, 999);

			std::vector<col_vector<T>> input_examples;
			std::vector<col_vector<T>> input_targets;
			// alignment and concatenation of input_examples, and converting from map to col_vector for both input_examples and input_targets
			process_inputs(ms, input_examples, input_targets, frames_shapes_tracking, frames_controls);

			ms.regressors_.resize(ms.n_var_output_);
			ms.cross_validated_params_.regressor_params.resize(ms.n_var_output_);

			train_regressors_optimal_params(ms, input_examples, input_targets, prev_optimal_params.regressor_params, ms.cross_validated_params_.regressor_params, prev_optimal_params.orig_training_params.num_threads);

			// imputation
			ms.cross_validated_params_.imputation_regressor_params.resize(prev_optimal_params.orig_training_params.imputation.num_iterations, std::vector<optimal_regressor_params<T>>(ms.n_var_output_));
			for (unsigned i = 0; i < prev_optimal_params.orig_training_params.imputation.num_iterations; ++i)
			{
				modify_target_examples(ms, input_examples, input_targets, control_ranges, i);
				train_regressors_optimal_params(ms, input_examples, input_targets, prev_optimal_params.imputation_regressor_params[i], ms.cross_validated_params_.imputation_regressor_params[i], prev_optimal_params.orig_training_params.num_threads);
			}

			ms.output_constraint_pca_ = train_pca_model_rsvd_to_target_variance<T>(input_targets, 0.995, 999);

			compute_training_diagnostics(ms, static_cast<int>(frames_shapes_tracking.size()), static_cast<int>(control_ranges.size()), training_deltas);

			ms.is_trained_ = true;
			return ms;
		}

		static void verify_inputs(const modular_solver<T, D>& ms, const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking, const std::vector<std::map<std::string, T>>& frames_controls, const std::vector<std::pair<typename modular_solver<T, D>::alignmentD_type, std::vector<std::vector<std::string>>>>& shapes_alignment_views_pointids)
		{
			CARBON_SUPPRESS_UNUSED(frames_controls);
			CARBON_SUPPRESS_UNUSED(shapes_alignment_views_pointids);

			for (unsigned view = 0; view < ms.num_views_; ++view)
			{
				DLIB_ASSERT(std::all_of(ms.shapes_data_.begin(), ms.shapes_data_.end(), [&](const modular_solver<T, D>::shape_data& r) { return std::all_of(r.views_point_ids[view].begin(), r.views_point_ids[view].end(), [&](const auto& id) { return std::count(r.views_point_ids[view].begin(), r.views_point_ids[view].end(), id) == 1; }); }),
					"\t modular_solver_trainer::verify_inputs(const modular_solver<T, D>& ms, const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking, const std::vector<std::map<std::string, T>>& frames_controls)"
					<< "\n\t Invalid inputs:"
					<< "\n\t Each view of ms.shapes_data_.point_ids_ must contain only unique point IDs");
			}

			DLIB_ASSERT(frames_shapes_tracking.size() == frames_controls.size(),
				"\t modular_solver_trainer::verify_inputs(const modular_solver<T, D>& ms, const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking, const std::vector<std::map<std::string, T>>& frames_controls)"
				<< "\n\t Invalid inputs:"
				<< "\n\t There must be as many tracking examples as control examples:"
				<< "\n\t frames_shapes_tracking.size(): " << frames_shapes_tracking.size()
				<< "\n\t frames_controls.size(): " << frames_controls.size());

			DLIB_ASSERT(std::all_of(frames_controls.begin(), frames_controls.end(), [&](typename std::vector<std::map<std::string, T>>::const_reference controls) { return controls.size() == ms.n_var_output_; }),
				"\t modular_solver_trainer::verify_inputs(const modular_solver<T, D>& ms, const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking, const std::vector<std::map<std::string, T>>& frames_controls)"
				<< "\n\t Invalid inputs:"
				<< "\n\t Every frame must contain the same number of controls.");

			DLIB_ASSERT(std::all_of(frames_shapes_tracking.begin(), frames_shapes_tracking.end(), [&](typename std::vector<std::vector<std::map<std::string, P>>>::const_reference it) {return it.size() == frames_shapes_tracking.begin()->size(); }),
				"\t modular_solver_trainer::verify_inputs(const modular_solver<T, D>& ms, const std::vector<std::vector<std::vector<std::map<std::string, P>>>>& frames_shapes_tracking, const std::vector<std::map<std::stirng, T>>& frames_controls)"
				<< "\n\t Invalid inputs:"
				<< "\n\t Each frame of example tracking must contain the same number of shapes");

			for (unsigned frame = 0; frame < frames_shapes_tracking.size(); ++frame)
			{
				DLIB_ASSERT(std::all_of(frames_controls[frame].begin(), frames_controls[frame].end(), [&](typename std::map<std::string, T>::const_reference it) { return ms.control_ranges_.find(it.first) != ms.control_ranges_.end(); }),
					"\t modular_solver_trainer::verify_inputs(const modular_solver<T, D>& ms, const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking, const std::vector<std::map<std::string, T>>& frames_controls)"
					<< "\n\t Invalid inputs:"
					<< "\n\t Each frame of example controls must contain all controls indicated in ms.control_ranges_.");

				for (unsigned shape = 0; shape < shapes_alignment_views_pointids.size(); ++shape)
				{
					for (unsigned view = 0; view < ms.num_views_; ++view)
					{
						DLIB_ASSERT(std::all_of(shapes_alignment_views_pointids[shape].second[view].begin(), shapes_alignment_views_pointids[shape].second[view].end(),
							[&](const std::vector<std::string>::const_reference id) {return frames_shapes_tracking[frame][shape].find(id) !=
							frames_shapes_tracking[frame][shape].end(); }),
							"\t modular_solver_trainer::verify_inputs(const modular_solver<T, D>& ms, const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking, const std::vector<std::map<std::string, T>>& frames_controls)"
							<< "\n\t Invalid inputs:"
							<< "\n\t Each view of example tracking must contain all point IDs indicated in shapes_alignment_views_pointids in every frame.");

					}
				}
			}
		}

		static void verify_params(const typename modular_solver<T, D>::training_params& params)
		{
			if (params.compression.input_max_modes_to_try.size() == 0 || std::any_of(params.compression.input_max_modes_to_try.begin(), params.compression.input_max_modes_to_try.end(), [](std::vector<int>::const_reference r) {return r < 0; }))
			{
				throw modular_solver_error("Inappropriate input compression params - input_max_modes_to_try must contain at least one element, and all elements must be greater than or equal to zero.");
			}
			if (std::any_of(params.compression.input_max_modes_to_try.begin(), params.compression.input_max_modes_to_try.end(), [](std::vector<int>::const_reference r) {return r > 0; }))
			{
				if (params.compression.input_variance_to_try.size() == 0 || std::any_of(params.compression.input_variance_to_try.begin(), params.compression.input_variance_to_try.end(), [](typename std::vector<T>::const_reference r) {return r <= 0.0 || r > 1.0; }))
				{
					throw modular_solver_error("Inappropriate input compression params - input_variance_to_try must contain at least one element, and all elements must be greater than zero and less than or equal to one, since input_max_modes_to_try contains at least one non-zero element.");
				}
			}
			switch (params.regression.regression_type)
			{
			case ms_regression_options::ridge_regression:
				if (params.regression.lambdas_to_try.size() == 0 || std::any_of(params.regression.lambdas_to_try.begin(), params.regression.lambdas_to_try.end(), [](typename std::vector<T>::const_reference r) { return r < 0.0; }))
				{
					throw modular_solver_error("Inappropriate input ms_regression_options::ridge_regression params - lambdas_to_try must contain at least one element, and all elements must be greater than or equal to zero");
				}
				break;
			default:
				throw modular_solver_error("Unrecognised ms_regression_options parameter.");
			}

			switch (params.kernel.kernel_type)
			{
			case ms_kernel_options::linear:
				break;
			default:
				throw modular_solver_error("Unrecognised kernel option.");
			}

			if (params.metric == cm::ms_cross_validation_metric::MAX_NUMBER_OF_ms_cross_validation_metric)
			{
				throw modular_solver_error("Unrecognised cross-validation-metric option.");
			}

			if (params.feature_selection.feature_fractions_to_try.size() > 0 && std::any_of(params.feature_selection.feature_fractions_to_try.begin(), params.feature_selection.feature_fractions_to_try.end(), [](typename std::vector<T>::const_reference r) { return r <= 0.0 || r > 1.0; }))
			{
				throw modular_solver_error("Inappropriate input ms_feature_selection_options::feature_selection params - feature_fractions_to_try must contain at least one element, and all elements must be greater than zero and less than or equal to one.");
			}
		}
	private:
		static void train_shape_models(modular_solver<T, 2>& ms, const std::vector<std::pair<alignment_type, std::vector<std::vector<std::string>>>>& shapes_alignment_views_pointids, const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking, const T shape_pca_variance, const int shape_pca_modes)
		{
			const unsigned num_examples = static_cast<unsigned>(frames_shapes_tracking.size());
			const unsigned num_shapes = static_cast<unsigned>(shapes_alignment_views_pointids.size());

			ms.shapes_data_.resize(num_shapes);
			for (unsigned shape = 0; shape < num_shapes; ++shape)
			{
				ms.shapes_data_[shape].alignment = shapes_alignment_views_pointids[shape].first;
				ms.shapes_data_[shape].views_base_shape.resize(ms.num_views_);
				ms.shapes_data_[shape].views_point_ids.resize(ms.num_views_);

				std::vector<std::vector<std::vector<P>>> examples_views(num_examples, std::vector<std::vector<P>>(ms.num_views_));
				for (unsigned view = 0; view < ms.num_views_; ++view)
				{
					ms.shapes_data_[shape].views_point_ids[view] = shapes_alignment_views_pointids[shape].second[view];
					if (ms.shapes_data_[shape].views_point_ids[view].size() > 0)
					{
						std::vector<std::vector<P>> shape_examples(num_examples);
						for (unsigned example = 0; example < num_examples; ++example)
						{
							shape_examples[example] = map_to_vector(frames_shapes_tracking[example][shape], ms.shapes_data_[shape].views_point_ids[view]);
							examples_views[example][view] = shape_examples[example];
						}
						ms.shapes_data_[shape].views_base_shape[view] = compute_procrustes_base_shape(shape_examples, ms.shapes_data_[shape].alignment);
					}
				}

				if (shape_pca_variance > 0.0)
				{
					std::vector<col_vector<T>> examples_concatenated_points(num_examples, col_vector<T>(static_cast<long>( ms.shapes_data_[shape].get_num_inputs())));
					for (unsigned example = 0; example < num_examples; ++example)
					{
						examples_concatenated_points[example] = ms.shapes_data_[shape].concatenate_views_and_points(examples_views[example]);
					}
					ms.shapes_data_[shape].shape_pca = train_pca_model_rsvd_to_target_variance<T>(examples_concatenated_points, shape_pca_variance, shape_pca_modes);
				}
			}
		}

		static void train_shape_models(modular_solver<T, 3>& ms, const std::vector<std::pair<alignment3d_type, std::vector<std::vector<std::string>>>>& shapes_alignment_views_pointids, const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking, const T shape_pca_variance, const int shape_pca_modes)
		{
			const unsigned num_examples = static_cast<unsigned>(frames_shapes_tracking.size());
			const unsigned num_shapes = static_cast<unsigned>(shapes_alignment_views_pointids.size());

			ms.shapes_data_.resize(num_shapes);
			for (unsigned shape = 0; shape < num_shapes; ++shape)
			{
				ms.shapes_data_[shape].alignment = shapes_alignment_views_pointids[shape].first;
				ms.shapes_data_[shape].views_base_shape.resize(ms.num_views_);
				ms.shapes_data_[shape].views_point_ids.resize(ms.num_views_);

				std::vector<std::vector<std::vector<P>>> examples_views(num_examples, std::vector<std::vector<P>>(ms.num_views_));
				for (unsigned view = 0; view < ms.num_views_; ++view)
				{
					ms.shapes_data_[shape].views_point_ids[view] = shapes_alignment_views_pointids[shape].second[view];
					if (ms.shapes_data_[shape].views_point_ids[view].size() > 0)
					{
						std::vector<std::vector<P>> shape_examples(num_examples);
						for (unsigned example = 0; example < num_examples; ++example)
						{
							shape_examples[example] = map_to_vector(frames_shapes_tracking[example][shape], ms.shapes_data_[shape].views_point_ids[view]);
							examples_views[example][view] = shape_examples[example];
						}
						ms.shapes_data_[shape].views_base_shape[view] = compute_procrustes_base_shape3d(shape_examples, ms.shapes_data_[shape].alignment);
					}
				}

				if (shape_pca_variance > 0.0)
				{
					std::vector<col_vector<T>> examples_concatenated_points(num_examples, col_vector<T>(static_cast<long>(ms.shapes_data_[shape].get_num_inputs())));
					for (unsigned example = 0; example < num_examples; ++example)
					{
						examples_concatenated_points[example] = ms.shapes_data_[shape].concatenate_views_and_points(examples_views[example]);
					}
					ms.shapes_data_[shape].shape_pca = train_pca_model_rsvd_to_target_variance<T>(examples_concatenated_points, shape_pca_variance, shape_pca_modes);
				}
			}
		}

		static void process_inputs(const modular_solver<T, D>& ms, std::vector<col_vector<T>>& input_examples, std::vector<col_vector<T>>& input_targets, const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking, const std::vector<std::map<std::string, T>>& frames_controls)
		{
			unsigned num_examples = static_cast<unsigned>(frames_shapes_tracking.size());
			input_examples.assign(num_examples, col_vector<T>(std::accumulate(ms.shapes_data_.begin(), ms.shapes_data_.end(), 0, [](size_t sum, const typename modular_solver<T, D>::shape_data& shape_model) {return sum + shape_model.get_num_outputs(); })));
			input_targets.assign(num_examples, col_vector<T>(ms.n_var_output_));
			for (unsigned ex = 0; ex < num_examples; ++ex)
			{
				std::vector<std::vector<std::vector<P>>> aligned_shapes_views(ms.shapes_data_.size(), std::vector<std::vector<P>>(ms.num_views_));
				for (unsigned view = 0; view < ms.num_views_; ++view)
				{
					for (unsigned shape = 0; shape < ms.shapes_data_.size(); ++shape)
					{
						aligned_shapes_views[shape][view] = map_to_vector(frames_shapes_tracking[ex][shape], ms.shapes_data_[shape].views_point_ids[view]);
						ms.align(aligned_shapes_views[shape][view], ms.shapes_data_[shape].views_base_shape[view], ms.shapes_data_[shape].alignment);
					}
				}
				input_examples[ex] = ms.concatenate_shapes(aligned_shapes_views);
				int control_index = 0;
				for (auto it = frames_controls[ex].begin(); it != frames_controls[ex].end(); ++it, ++control_index)
				{
					input_targets[ex](control_index) = it->second;
				}
			}
		}

		static void train_regressors_optimal_params(modular_solver<T, D>& ms, const std::vector<col_vector<T>>& input_examples, const std::vector<col_vector<T>>& input_targets, const std::vector<optimal_regressor_params<T>>& previous_regressor_params, std::vector<optimal_regressor_params<T>>& this_regressor_params, int num_threads)
		{
			DLIB_ASSERT(is_learning_problem(input_examples, input_targets),
				"\t modular_solver_trainer::train_regressors_optimal_params(modular_solver<T, D>& ms, const std::vector<col_vector<T>>& input_examples, const std::vector<col_vector<T>>& input_targets, const std::vector<optimal_regressor_params<T>>& previous_regressor_params, std::vector<optimal_regressor_params<T>>& this_regressor_params)"
				<< "\n\t Invalid inputs:"
				<< "\n\t is_learning_problem(input_examples, input_targets) must evaluate true");

			DLIB_ASSERT(input_targets.begin()->nr() == previous_regressor_params.size(),
				"\t modular_solver_trainer::train_regressors_optimal_params(modular_solver<T, D>& ms, const std::vector<col_vector<T>>& input_examples, const std::vector<col_vector<T>>& input_targets, const std::vector<optimal_regressor_params<T>>& previous_regressor_params, std::vector<optimal_regressor_params<T>>& this_regressor_params)"
				<< "\n\t Invalid inputs:"
				<< "\n\t There must be as many sets of previous regressor params as number of controls"
				<< "\n\t previous_regressor_params.size(): " << previous_regressor_params.size()
				<< "\n\t input_targets.begin()->nr(): " << input_targets.begin()->nr());

			DLIB_ASSERT(input_targets.begin()->nr() == this_regressor_params.size(),
				"\t modular_solver_trainer::train_regressors_optimal_params(modular_solver<T, D>& ms, const std::vector<col_vector<T>>& input_examples, const std::vector<col_vector<T>>& input_targets, const std::vector<optimal_regressor_params<T>>& previous_regressor_params, std::vector<optimal_regressor_params<T>>& this_regressor_params)"
				<< "\n\t Invalid inputs:"
				<< "\n\t There must be as many sets of current regressor params as number of controls"
				<< "\n\t previous_regressor_params.size(): " << this_regressor_params.size()
				<< "\n\t input_targets.begin()->nr(): " << input_targets.begin()->nr());

			const unsigned num_examples = static_cast<unsigned>(input_examples.size());
			const unsigned num_predictors = static_cast<unsigned>(input_targets.begin()->nr());

			if (num_threads == 1)
			{
				for (unsigned p = 0; p < num_predictors; ++p)
				{
					// input_targets_single contains all examples of this control
					std::vector<T> input_targets_single(num_examples);
					for (unsigned ex = 0; ex < num_examples; ++ex)
					{
						input_targets_single[ex] = input_targets[ex](p);
					}

					ms.regressors_[p] = modular_solver_regressor_factory<T>::regressor_training_factory(ms.cross_validated_params_.orig_training_params.regression.regression_type,
						ms.cross_validated_params_.orig_training_params.kernel.kernel_type,
						input_examples,
						input_targets_single,
						{ previous_regressor_params[p].input_variance },
						{ previous_regressor_params[p].input_max_modes },
						{ previous_regressor_params[p].selected_feature_fraction.first },
						{ previous_regressor_params[p].lambda },
						this_regressor_params[p],
						false,
						ms_cross_validation_metric::MAX_NUMBER_OF_ms_cross_validation_metric);
					std::cout << ".";
				}
			}
			else
			{
				parallel_for(num_threads, 0, num_predictors, [&](long p)
					{
						// input_targets_single contains all examples of this control
						std::vector<T> input_targets_single(num_examples);
						for (unsigned ex = 0; ex < num_examples; ++ex)
						{
							input_targets_single[ex] = input_targets[ex](p);
						}

						ms.regressors_[p] = modular_solver_regressor_factory<T>::regressor_training_factory(ms.cross_validated_params_.orig_training_params.regression.regression_type,
							ms.cross_validated_params_.orig_training_params.kernel.kernel_type,
							input_examples,
							input_targets_single,
							{ previous_regressor_params[p].input_variance },
							{ previous_regressor_params[p].input_max_modes },
							{ previous_regressor_params[p].selected_feature_fraction.first },
							{ previous_regressor_params[p].lambda },
							this_regressor_params[p],
							false,
							ms_cross_validation_metric::MAX_NUMBER_OF_ms_cross_validation_metric);
						std::cout << ".";
					});
			}
			std::cout << std::endl;
		}

		static void train_regressors_cross_validation(modular_solver<T, D>& ms, const std::vector<col_vector<T>>& input_examples, const std::vector<col_vector<T>>& input_targets, const typename modular_solver<T, D>::training_params& params, std::vector<optimal_regressor_params<T>>& optimal_regressor_params)
		{
			DLIB_ASSERT(is_learning_problem(input_examples, input_targets),
				"\t modular_solver_trainer::train_regressors_cross_validation(modular_solver<T, P>& ms, const std::vector<col_vector<T>>& input_examples, const std::vector<col_vector<T>>& input_targets, const training_params& params, std::vector<optimal_regressor_params<T>>& optimal_regressor_params)"
				<< "\n\t Invalid inputs:"
				<< "\n\t is_learning_problem(input_examples, input_targets) must evaluate true");

			DLIB_ASSERT(input_targets.begin()->nr() == optimal_regressor_params.size(),
				"\t modular_solver_trainer::train_regressors_cross_validation(modular_solver<T, P>& ms, const std::vector<col_vector<T>>& input_examples, const std::vector<col_vector<T>>& input_targets, const training_params& params, std::vector<optimal_regressor_params<T>>& optimal_regressor_params)"
				<< "\n\t Invalid inputs:"
				<< "\n\t There must be as many sets of regressor params as number of controls"
				<< "\n\t previous_regressor_params.size(): " << optimal_regressor_params.size()
				<< "\n\t input_targets.begin()->nr(): " << input_targets.begin()->nr());

			const unsigned num_examples = static_cast<unsigned>(input_examples.size());
			const unsigned num_predictors = input_targets.begin()->nr();

			if (params.num_threads == 1)
			{
				for (unsigned p = 0; p < num_predictors; ++p)
				{
					// input_targets_single contains all examples of this control
					std::vector<T> input_targets_single(num_examples);
					for (unsigned ex = 0; ex < num_examples; ++ex)
					{
						input_targets_single[ex] = input_targets[ex](p);
					}
					ms.regressors_[p] = modular_solver_regressor_factory<T>::regressor_training_factory(params.regression.regression_type,
						params.kernel.kernel_type,
						input_examples,
						input_targets_single,
						params.compression.input_variance_to_try,
						params.compression.input_max_modes_to_try,
						params.feature_selection.feature_fractions_to_try,
						params.regression.lambdas_to_try,
						optimal_regressor_params[p],
						true,
						params.metric);
					std::cout << ".";
				}
			}
			else
			{
				parallel_for(params.num_threads, 0, num_predictors, [&](long p)
					{
						// input_targets_single contains all examples of this control
						std::vector<T> input_targets_single(num_examples);
						for (unsigned ex = 0; ex < num_examples; ++ex)
						{
							input_targets_single[ex] = input_targets[ex](p);
						}
						ms.regressors_[p] = modular_solver_regressor_factory<T>::regressor_training_factory(params.regression.regression_type,
							params.kernel.kernel_type,
							input_examples,
							input_targets_single,
							params.compression.input_variance_to_try,
							params.compression.input_max_modes_to_try,
							params.feature_selection.feature_fractions_to_try,
							params.regression.lambdas_to_try,
							optimal_regressor_params[p],
							true,
							params.metric);
						std::cout << ".";
					});
			}
			std::cout << std::endl;
		}

		static void modify_target_examples(const modular_solver<T, D>& ms, const std::vector<col_vector<T>>& input_examples, std::vector<col_vector<T>>& input_targets, const std::map<std::string, typename modular_solver<T, D>::control_range>& control_ranges, const unsigned int& imputation_iteration)
		{
			const unsigned num_examples = static_cast<unsigned>(input_examples.size());
			const unsigned num_regressors = static_cast<unsigned>(ms.regressors_.size());

			for (unsigned ex = 0; ex < num_examples; ++ex)
			{
				col_vector<T> predictions(num_regressors);
				for (unsigned r = 0; r < num_regressors; ++r)
				{
					const std::vector<int>& feature_selected_indices = imputation_iteration == 0 ? ms.cross_validated_params_.regressor_params[r].selected_feature_fraction.second : ms.cross_validated_params_.imputation_regressor_params[imputation_iteration - 1][r].selected_feature_fraction.second;
					const unsigned num_features = static_cast<unsigned>(feature_selected_indices.size());
					col_vector<T> feature_selected_input_example(num_features);
					for (unsigned f = 0; f < num_features; ++f)
					{
						feature_selected_input_example(f) = input_examples[ex](feature_selected_indices[f]);
					}
					predictions(r) = ms.regressors_[r].predict(feature_selected_input_example);
				}
				int c = 0;
				for (auto it = control_ranges.begin(); it != control_ranges.end(); ++it, ++c)
				{
					if (predictions(c) < it->second.min || predictions(c) > it->second.max)
					{
						input_targets[ex](c) = predictions(c);
					}
				}
			}
		}

		static void compute_training_diagnostics(const modular_solver<T, D>& ms, const int& num_examples, const int& num_controls, std::vector<training_delta>& training_deltas)
		{
			training_deltas.resize(static_cast<size_t>(num_examples) * static_cast<size_t>(num_controls));
			unsigned c = 0;
			unsigned delta = 0;
			for (auto control_it = ms.control_ranges_.begin(); control_it != ms.control_ranges_.end(); ++control_it, ++c)
			{
				for (int ex = 0; ex < num_examples; ++ex, ++delta)
				{
					training_deltas[delta].control_name = control_it->first;
					training_deltas[delta].fractional_delta = ms.cross_validated_params_.regressor_params[c].example_errors[static_cast<unsigned>(ex)] / (control_it->second.max - control_it->second.min);
					training_deltas[delta].example_index = ex;
				}
			}
			std::sort(training_deltas.rbegin(), training_deltas.rend(), [](const training_delta& lhs, const training_delta& rhs) -> bool {return std::abs(lhs.fractional_delta) < std::abs(rhs.fractional_delta); });
		}

		static void compute_training_md5s(modular_solver<T, D>& ms, const std::vector<std::vector<std::map<std::string, P>>>& frames_shapes_tracking, const std::vector<std::map<std::string, T>>& frames_controls, const typename modular_solver<T, D>::training_params& params, const std::map<std::string, typename modular_solver<T, D>::control_range>& control_ranges)
		{
			using dlib::serialize;
			std::stringstream ss;
			serialize(frames_shapes_tracking, ss);
			ms.tracking_examples_md5_ = dlib::md5(ss);
			ss.clear();
			serialize(frames_controls, ss);
			ms.control_examples_md5_ = dlib::md5(ss);
			ss.clear();
			serialize(params, ss);
			ms.training_parameters_md5_ = dlib::md5(ss);
			ss.clear();
			serialize(control_ranges, ss);
			ms.control_ranges_md5_ = dlib::md5(ss);
		}
	};
}