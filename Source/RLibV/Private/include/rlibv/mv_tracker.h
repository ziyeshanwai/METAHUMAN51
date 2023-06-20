// Copyright Epic Games, Inc. All Rights Reserved.

#include "../ThirdParty/dlib/serialization_prototypes.h"

#pragma once

#include "enum_ext.h"
#include "basic_types.h"
#include "data_utils.h"
#include "geometry.h"
#include "radial_feature_sampler.h"
#include "linear_pdm.h"
#include "fixed_pca_model.h"
#include "transforms.h"
#include "simple_curve.h"
#include "simple_profiler.h"
#include "maths_functions.h"

#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/svm.h>
#include <dlib/random_forest.h>
#include <dlib/rand.h>
#include <dlib/image_io.h>
#include <dlib/image_transforms.h>
RLIBV_RENABLE_WARNINGS
#include <vector>
#include <iostream>
#include <sstream> 
#include <algorithm> 
#include <random>
#include <chrono>
#include <mutex>

namespace rlibv
{
	using dlib::serialize;
	using dlib::deserialize;

	/**
   	* \defgroup Tracking Tracking
   	* @{
   	*/

	template<typename T>
	using view_vector = std::vector<T>;
	
	/**
	 * The regression model type used by mv_tracker. You shouldn't use these directly.
	 */
	template<int INPUTS, int MODES, int TREES>
	struct point_regression_model
	{
		dlib::vector_normalizer<dlib::matrix<double, INPUTS, 1>> normalizer;
		fixed_pca_model<double, INPUTS, MODES<1? 1 : MODES> compressor;
		dlib::vector_normalizer<dlib::matrix<double, MODES<1 ? 1 : MODES, 1>> normalizer2;
		dlib::decision_function<dlib::linear_kernel<dlib::matrix<double, MODES<1 ? 1 : MODES, 1>>> dn_kernel_regression;
		dlib::decision_function<dlib::linear_kernel<dlib::matrix<double, MODES<1 ? 1 : MODES, 1>>> dt_kernel_regression;
		dlib::random_forest_regression_function<dlib::dense_feature_extractor> dn_rf_regression;
		dlib::random_forest_regression_function<dlib::dense_feature_extractor> dt_rf_regression;
		bool use_rf = false;
	};

	/**
	* @brief Supports serialization of point_regression_model structures.
	* @param item
	* @param out
	*/
	template<int N, int M, int T>
	void serialize(const point_regression_model<N, M, T>& item, std::ostream& out);

	/**
	 * @brief Supports deserialization of point_regression_model structures.
	 * @param item
	 * @param in
	 */
	template<int N, int M, int T>
	void deserialize(point_regression_model<N, M, T>& item, std::istream& in);

	/**
	 * Training data for a confidence model
	 */
	struct confidence_training_data
	{
		std::vector<double> confidence;
		std::vector<std::vector<unsigned char>> feature_samples;
	};
	/**
	* @brief Supports serialization of confidence_training_data structures.
	* @param item
	* @param out
	*/
	inline void serialize(const confidence_training_data& item, std::ostream& out) 
	{
		serialize(item.confidence, out);
		serialize(item.feature_samples, out);
	}

	/**
	* @brief Supports deserialization of confidence_training_data structures.
	* @param item
	* @param in
	*/
	inline void deserialize(confidence_training_data& item, std::istream& in)
	{
		deserialize(item.confidence, in);
		deserialize(item.feature_samples, in);
	}

	/**
	 * Training data for an individual point.
	*/
	struct point_training_data
	{
		int index = -1;
		std::vector<double> dn_data;
		std::vector<double> dt_data;
		std::vector<std::vector<unsigned char>> feature_samples;
	};

	/**
	* @brief Supports serialization of point_training_data structures.
	* @param item
	* @param out
	*/
	inline void serialize(const point_training_data& item, std::ostream& out)
	{
		serialize(item.index, out);
		serialize(item.dn_data, out);
		serialize(item.dt_data, out);
		serialize(item.feature_samples, out);
	}

	/**
	* @brief Supports deserialization of point_training_data structures.
	* @param item
	* @param in
	*/
	inline void deserialize(point_training_data& item, std::istream& in)
	{
		deserialize(item.index, in);
		deserialize(item.dn_data, in);
		deserialize(item.dt_data, in);
		deserialize(item.feature_samples, in);
	}

	/**
	* \brief Checks if the given data is valid data for initializing an mv_tracker.
	* \details In order for initialization data to be valid, all of the following must be true:
	*          mv_shapes.size() > 2
	*			For every mv_shape:
	*				mv_shapes[i].size() == mv_shapes[0].size());
	*
	*			For all shapes 'view' in mv_shapes[i]:
	*              view.size() >= 2
	*				view.size() == mv_shapes[i][0].size()
	*
			every curve in curves[v] contains elements whos values are > 0 and < mv_shapes[0][v].size()
	*			stage_scaling.size() > 0
	*          every element of stage_scaling must be > 0.0
	*
	* \param mv_shapes vector of training shapes (each one a vector of shapes - each entry is a view)
	* \param curves map of curve definitions, per view
	* \param augmentation_factors - only relevant in models without similarity alignment
	* \param stage_scaling The relative scaling of each search stage.
	* \param[out] error_message populates error_message with "ok" or a description of what wasn't valid.
	* \return returns true if and only if the data is valid.
	*/
	inline bool is_mv_tracker_initialization_valid(const std::vector<multiview_shape2d<double>>& mv_shapes,
		const view_vector<std::vector<simple_curve>>& curves,
		const std::array<double, 4>& /*augmentation_factors*/,
		const std::vector<double>& stage_scaling,
		std::string& error_message
	)
	{
		if (mv_shapes.size() < 2)
		{
			error_message = "at least two shapes must be provided.";
			return false;
		}

		if (stage_scaling.empty())
		{
			error_message = "stage_scaling is empty";
			return false;
		}


		bool is_first = true;
		std::vector<int> expected_pts_per_view;
		for (const auto& view_set : mv_shapes)
		{
			if (view_set.shapes.size() != mv_shapes[0].shapes.size())
			{
				error_message = "Every example must provide the same number of views";
				return false;
			}

			int v = 0;
			for (const auto& view : view_set.shapes)
			{
				if (view.size() < 2)
				{
					error_message = "Every shape in every view must have at least two points";
					return false;
				}

				if (is_first)
				{
					expected_pts_per_view.emplace_back(view.size());

				}
				else if (view.size() != static_cast<size_t>(expected_pts_per_view[v]))
				{
					error_message = "All shapes in each view must have the same number of points";
					return false;
				}
				++v;
			}

			for (int v = 0; v < curves.size(); ++v)
			{
				for (const auto& curve : curves[v])
				{
					for (const auto& index : curve.indices)
					{
						if (index < 0 || index >= mv_shapes[0].shapes[v].size())
						{
							error_message = "Curve definitions for each view must only contain indices >0 and less than the number of points in the view";
							return false;
						}
					}
				}
			}
			is_first = false;
		}
		error_message = "ok";
		return true;
	}

	/**
	 * @brief A class representing a Multi-view point-wise Regression Tracker
	 * 
	 * To use an mv_tracker you first need to train it. 
	 * 
	 * The typical training process is as follows: 
	 * model.initialize_training(training_shapes,curves, ...[other args] ..., batch_size );
	 *
	 * 
	 *	for (size_t e = 0; e < training_shapes.size(); ++e)
	 *	{
	 *		model.add_training_example(training_images[e], training_shapes[e], training_shapes.size());
	 *	}
	 *  model.finalize_training(...)
	 *	
	 * 
	 * The above code repeatedly runs through the training set adding examples. On each run, at least
	 * 'batch_size' points are trained. Once every point has been trained with the expected number of 
	 * examples (given by the third argument to .add_training_example) training is finalized.
	 * 
	 * If batch_size is given as -1 the method will attempt to determine an appropriate batch size based
	 * on the available memory.
	 * 
	 * @tparam T the image type
	 * @tparam R the resolution of each sampling line, which also automatically
	 *           sets the number of sampling lines to be R/2. R must be even.
	 * @tparam S the number of sampling scales
	 * @tparam MODES the number of modes in the PCA regression reduction model, or
	 * @tparam TREES the number of trees in the random forest regressors
	 * @tparam args any number of feature types
	 */
	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	class mv_tracker
	{
		static_assert(
			std::is_same<T, dlib::rgb_pixel>::value ||
			std::is_same<T, dlib::rgb_alpha_pixel>::value ||
			std::is_same<T, unsigned char>::value,
			"T must be a pixel type");

		static_assert(R > 0 && (R % 2) == 0,
			"R must be even and greater than 2");

		static_assert(S > 0 && S < 9,
			"S must be between 1 and 9");

	public:
		using sampler = rlibv::radial_feature_sampler<
			T,
			R,
			S,
			args...>;

		/**
		 * The total number of features used by the model.
		 * 
		 * \return number of features
		 */
		static constexpr int total_number_of_features();

		/**
		 * The total number of raw samples used by the model.
		 *
		 * \return number of raw samples
		 */
		static constexpr int number_of_raw_samples();
		
		/**
		 * Get the approximate map of minimum training values of scale,angle,x and y
		 * 
		 * \return approximate map of minimum values in order s,a,x,y
		 */
		const view_vector<std::array<double, 4>>& local_approx_saxy_min() const;
		
		/**
		  * Get the approximate map of maximum training values of scale,angle,x and y
		 *
		 * \return approximate map of maximum values in order s,a,x,y
		 */
		const view_vector<std::array<double, 4>>& local_approx_saxy_max() const;
		
		/**
		  * Get the approximate map of mid training values of scale,angle,x and y
		 *
		 * \return approximate map of mid values in order s,a,x,y
		 */
		const view_vector<std::array<double, 4>>& local_approx_saxy_mid() const;

		/**
		 * Get a constant reference to the map of internal shape models.
		 * 
		 * \return constant ref to the map of internal shape models
		 */
		const view_vector<linear_pdm<double>>& local_pdms() const;

		/**
		 * Get a constant reference to the global shape model.
		 *
		 * \return constant ref to the global shape model
		 */
		const linear_pdm<double>& global_pdm() const;

		/**
		 * Construct a basis image to illustrate the sampling regime.
		 * 
		 * \param pt_indices The point indices for which to show samples.
		 * \param stage The regression stage to illustrate.
		 * \param view The number of the view
		 * \return A dlib image showing the sampling regime for the given view
		 */
		dlib::array2d<dlib::rgb_pixel> sampling_diagnostic_image(const std::vector<int>& pt_indices, int stage, int view);

		/**
		 * @brief Initialize training of the mv_tracker.
		 * @param mv_shapes vector of training shapes (each element contains shapes for each view)
		 * @param curves map of curve definitions, per view
		 * @param augmentation_factors - only relevant in models without similarity alignment
		 * @param stage_scaling The relative scaling of each search stage.
		 * @param align_method Alignment method
		 * @param displacement_size Displacement size, typically in the range 0.2-1.0
		 * @pre is_mv_tracker_initialization_valid(...) == true
		 */
		void initialize_training(
			const std::vector<multiview_shape2d<double>>& mv_shapes,
			const view_vector<std::vector<simple_curve>>& curves,
			std::array<double, 4> augmentation_factors,
			std::vector<double> stage_scaling,
			alignment_type align_method,
			double displacement_size
		);
		
		/**
		 * @brief Add a training example.
		 * 
		 * @param view_imgs The set of images (one per view) to add.
		 * @param mv_shape The multiview shape to add.
		 * @param expected_number_of_images How many images are going to be passed to this?
		 *									This is important because it determines how many 
		 *                                  displacements per image to take.
		 * @param min_target_displacements How many displacements?
		 * @param min_samples_per_node A learning smoothness parameter.
		 * @pre view_imgs.size() == n_views()
		 * @pre mv_shape.shapes.size() = n_views()
		 */
		void add_training_example(const dlib::array<dlib::array2d<dlib::rgb_pixel>>& view_imgs,
			const multiview_shape2d<double>& mv_shape,
			int expected_number_of_images,
			int min_target_displacements,
			int min_samples_per_node);
			
		/**
		 *	Complete the training after collecting all the data
		 *  
		 *  @param min_samples_per_node The minimum samples per node in RF 
		 *  @param generic_training Optional parameter; a pointer to another training set, used to to train
		 *                                              semi-generic trackers.
		 */
		void finalize_training(int min_samples_per_node,
			const view_vector<std::vector<std::vector<point_training_data>>>* generic_training = nullptr);

		/**
		* Get the number of texture modes
		*
		* \return the number of texture modes
		*/
		constexpr int n_texture_modes();

		/**
		 * Get the number of stages
		 *
		 * \return the number of stages
		 */
		const int n_stages() const;
		
		/**
		 * Get the number of shape modes in the global model.
		 * 
		 * \return number of modes
		 */
		const int n_shape_modes() const;

		/**
		 * Get the number of points in each view.
		 *
		 * \return number of points by view
		 */
		const std::vector<int>& n_points() const;
		
		/**
		 * Return the number of views
		 *
		 * @return the number of views
		 */
		int n_views() const;

		/**
		 * Load and preprocess an input image, ready for searching.
		 * 
		 * \param views array pf images, one per view
		 * \pre is_image_map_valid(const view_vector<dlib::array2d<dlib::rgb_pixel>>& image_map)
		 *      i.e. the keys in image map match the elements of view_names()
		 */
		void set_images(const dlib::array<dlib::array2d<dlib::rgb_pixel>>& views);

		/**
		 * Return the first training shapes in the model.
		 * 
		 * \return The first shapes.
		 */
		const multiview_shape2d<double>& first_training_shapes() const;
		
		/**
		 * Return a vector of stage scales at each stage.
		 *
		 * \return The scaling at each stage.
		 */
		const std::vector<double>& stage_scaling() const;
		
		/**
		 * Return a reference to the training shapes in the model.
		 *
		 * \return const reference to the training shapes
		 */
		const std::vector<multiview_shape2d<double>>& training_shapes() const;

		/**
		 * @brief Run one search iteration at a given stage.
		 * 
		 * @param stage The search stage.
		 * @param max_modes Sets the number of modes used by the shape constraint.
		 * @param bilinear Set to true if you want to use bilinear (slower but more precise) sampling.
		 * @param[in,out] mv_shape The starting multiview shape, which will be updated in place.
		 * @pre mv_shape.shapes.size() == n_views()
		 
		 * @return view_vector<std::vector<double>> a per-point fitness measure based on suggested movement
		 */
		view_vector<std::vector<double>> update_shapes(
			int stage,
			int max_modes, 
			bool bilinear, 
			multiview_shape2d<double>& mv_shape) const;

		/**
		 * @brief Search the image, previously set using set_images(...).
		 *
		 
		 * @param iterations_per_stage The number of iterations to run at each stage.
		 * @param max_modes_per_stage The maximum number of modes (actually number used are automatically calculated)
		 * @param bilinear Set to true if you want bilinear sampling (slower but more precise)
		 * @param mv_shape The starting multiview shape, which will be updated in place.
		 * @return view_vector<std::vector<double>> a per-point fitness measure based on suggested movement
		 */
		view_vector<std::vector<double>> autosearch(
			const std::vector<int>& iterations_per_stage,
			int max_modes_per_stage,
			bool bilinear,
			multiview_shape2d<double>& mv_shape
		) ;

		/**
		 * @brief Search the image, previously set using set_image(...).
		 * 
		 * @param iterations_per_stage The number of iterations to run at each stage.
		 * @param bilinear Set to true if you want bilinear sampling (slower but more precise)
		 * @param max_modes_per_stage Sets the number of modes used by the shape constraint
		 *                             for each stage.
		 * @param mv_shape The starting multiview shapes, which will be updated in place.
		 * 
		 * @return view_vector<std::vector<double>> a per-point fitness measure based on suggested movement
		 */
		view_vector<std::vector<double>> search(
			const std::vector<int>& iterations_per_stage,
			const std::vector<int>& max_modes_per_stage,
			bool bilinear,
			multiview_shape2d<double>& mv_shape
		) ;

		/**
		 * @brief Run multistage diagnostic search
		 * 
		 * @param shapes The starting shapes, which will be updated in place.
		 * @param iterations_per_stage The number of iterations to run at each stage.
		 * @param max_modes_per_stage Sets the number of modes used by the shape constraint
		 *                             for each stage.
		 * @param bilinear Set to true if you want bilinear sampling (slower but more precise)
		 * @param[out] local_predictions The local predictions, per view, at each stage before
		 * 								the shape model was applied.
		 * @return std::vector<std::vector<double>> giving a normalized measure of fitness 
		 *         at each iteration of each stage
		 */
		std::vector<std::vector<double>> diagnostic_search(
			view_vector<std::vector<dlib::dpoint>>& shapes,
			const std::vector<int>& iterations_per_stage,
			const std::vector<int>& max_modes_per_stage,
			bool bilinear,
			std::vector<view_vector<std::vector<dlib::dpoint>>>& local_predictions
		);
		
		/**
		 * Return an approximation to a multiview shape where the points are weighted from 0.0 to 1.0 where
		 * 1.0 means the point will be approximated as precisely as possible subject to the constraints of
		 * the model and 0.0 means the shape will be purely imputed
		 *
		 * @param mv_shape the input shape
		 * @param mv_weights the multiview weights
		 * @param max_modes limit the PDM parameterization to this number of modes
		 * @param iterations the number of fitting iterations to run (if caller knows that all weights are equal, this
		 *        should be set to 1)
		 * @return the approximated shape
		 * @pre mv_shape.shapes.size() == mv_weights.size()
		 * @pre mv_shape.shapes[i].size() = mv_weights[i].size()
		 */
		multiview_shape2d<double> weighted_shape_fit(const multiview_shape2d<double>& mv_shape, const std::vector<std::vector<double>>& mv_weights, int max_modes, int iterations) const;
		/**
		 * @brief Save the model
		 * @param out The output stream
		 */
		void save(std::ostream& out) const;

		/**
		 * @brief Load a model
		 * @param in The input stream
		 */
		void load(std::istream& in);

		/**
		* Get the raw sample radius.
		*
		* \return the raw sample radius
		*/
		const double raw_sample_radius() const;
		
		/**
		* Get feature sampling radii for a given shape
		*
		* @param shape the example shape
		* @param view  the index of the view
		*
		* @return a vector (without any stage_scaling adjustment) of radii for each sampler for the given view
		*/
		std::vector<double> feature_radii(shape2d<double>& shape, int view) const;

		/**
		* Get feature sampling angle for all the points
		*
		* @param shape the example shape
		* @param view  the index of the view
		*
		* @return a vector of local rotations for the points
		*/
		std::vector<double> feature_angles(shape2d<double>& shape, int view) const;

		const view_vector<std::vector<std::vector<point_training_data>>>* regressors_training_data();

		const view_vector<std::vector<std::vector<confidence_training_data>>>* classifier_training_data();

	private:
		
		// Don't serialize the sampler
		view_vector<rlibv::radial_feature_sampler<T, R, S, args...>> samplers_;

		// Internal number used for pseudorandom displacements; don't serialize
		view_vector<std::vector<int>> halton_index_;


		view_vector<std::vector<std::vector<point_training_data>>> regressors_training_data_;
		view_vector<std::vector<std::vector<confidence_training_data>>> classifier_training_data_;

		int n_views_ = 0;
		double max_normal_displacement_ = (MODES > 1) ? 0.25 : 0.5;
		double max_tangent_displacement_ = (MODES > 1) ? 0.25 : 0.5;
		view_vector<std::vector<std::vector<std::array<int, total_number_of_features()>>>> selected_feature_inds_norm_;
		view_vector<std::vector<std::vector<std::array<int, total_number_of_features()>>>> selected_feature_inds_tangent_;
		std::vector<double> stage_scaling_;
		view_vector<std::vector<simple_curve>> simple_curves_;

		view_vector<std::array<double, 4>> local_approx_saxy_min_;
		view_vector<std::array<double, 4>> local_approx_saxy_max_;
		view_vector<std::array<double, 4>> local_approx_saxy_mid_;
		view_vector<rlibv::linear_pdm<double>> local_pdms_;
		rlibv::linear_pdm<double> global_pdm_;
		view_vector<int> n_points_;
		view_vector<std::vector<std::vector<point_regression_model<total_number_of_features(), MODES, TREES>>>> regressors_;
		view_vector<std::vector<std::vector<dlib::random_forest_regression_function<dlib::dense_feature_extractor>>>> classifiers_;

		view_vector<std::vector<int>> pt_inds_;
		view_vector<std::vector<double>> residuals_limits_;
		std::vector<double> global_residuals_limits_;
		dlib::array<dlib::array2d<dlib::rgb_pixel>> first_training_images_;
		std::vector<multiview_shape2d<double>> training_shapes_;


		// Private functions
		std::vector<double> compute_local_rotations(int view, const std::vector<dlib::dpoint>& shape, double global_angle) const;

	};
	/**@}*/
}


#include "impl/mv_tracker.hpp"