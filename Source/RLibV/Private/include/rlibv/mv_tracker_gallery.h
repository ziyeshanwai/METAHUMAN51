// Copyright Epic Games, Inc. All Rights Reserved.

#include "../ThirdParty/dlib/serialization_prototypes.h"

#pragma once

#include "enum_ext.h"
#include "mv_tracker.h"
#include "basic_types.h"

#include <variant>

namespace rlibv
{
	typedef mv_tracker<dlib::rgb_pixel,
		14, 3, 30, 0,
		feature_type::moments,
		feature_type::poly2,
		feature_type::gradmag_poly2> standard_colour_tracker;

	typedef mv_tracker<dlib::rgb_pixel,
		12, 4, 0, 10,
		feature_type::moments,
		feature_type::poly2,
		feature_type::gradmag_poly2> tree_colour_tracker;

	typedef mv_tracker<dlib::rgb_pixel,
		14, 4, 30, 0,
		feature_type::moments,
		feature_type::poly2,
		feature_type::gradmag_poly2> four_level_colour_tracker;

	typedef mv_tracker<dlib::rgb_pixel, 8, 3, 30, 0,
		feature_type::moments,
		feature_type::poly2,
		feature_type::gradmag_poly2> fast_colour_tracker;

	typedef mv_tracker<unsigned char,
		14, 3, 30, 0,
		feature_type::moments,
		feature_type::poly2,
		feature_type::gradmag_poly2> standard_grey_tracker;

	typedef mv_tracker<unsigned char,
		14, 3, 0, 20,
		feature_type::moments,
		feature_type::poly2,
		feature_type::gradmag_poly2> tree_grey_tracker;

	typedef mv_tracker<unsigned char, 14, 4, 30, 0,
		feature_type::moments,
		feature_type::poly2,
		feature_type::gradmag_poly2> four_level_grey_tracker;

	typedef mv_tracker<unsigned char, 8, 3, 30, 0,
		feature_type::moments,
		feature_type::poly2,
		feature_type::gradmag_poly2> fast_grey_tracker;

	typedef mv_tracker<dlib::rgb_pixel,
		14, 3, 30, 0,
		feature_type::poly2,
		feature_type::gradmag_poly2> combined_colour_tracker;

	typedef mv_tracker<unsigned char,
		14, 3, 30, 0,
		feature_type::poly2,
		feature_type::gradmag_poly2> combined_grey_tracker;

	/// <summary>
	/// The types of tracker available in the gallery
	/// </summary>
	enum class mv_tracker_gallery_item 
	{
		standard_colour,
		tree_colour,
		four_level_colour,
		fast_colour,
		combined_colour,
		standard_grey,
		tree_grey,
		four_level_grey,
		fast_grey,
		combined_grey
	};

	/**
	 * @brief A class holding one of several present types of multi-view point-wise regression trackers
	 *
	 * To use an mv_tracker_gallery you first need to train it. 
	 *
	 * The typical training process is as follows:
	 * model.initialize_training(gallery_selection, training_shapes,curves, ...[other args] ..., batch_size );
	 *
	 * 
	 *		for (size_t e = 0; e < training_shapes.size(); ++e)
	 *		{
	 *			model.add_training_example(training_images[e], training_shapes[e], training_shapes.size());
	 *		}
	 * model.finalize_training(...);
	 *	
	 *
	 * The above code repeatedly runs through the training set adding examples. On each run, at least
	 * 'batch_size' points are trained. Once every point has been trained with the expected number of
	 * examples (given by the third argument to .add_training_example) training is finalized.
	 *
	 *
	 */
	class mv_tracker_gallery
	{
	public:

		/**
		 * Constructor - creates an untrained standard_colour_tracker by default
		 */
		mv_tracker_gallery()
		{
			tracker_ptr_ = std::make_unique<standard_colour_tracker>();
		}

		/**
		 * @brief Load a model
		 * @param ifs The input stream
		 */
		void load(std::istream& ifs);

		/**
		 * @brief Save a model
		 * @param ofs The output stream
		 */
		void save(std::ostream& ofs) const;

		/**
		 * @brief Initialize training of the mv_tracker_gallery.
		 * @param gallery_selection indicates which type of tracker from the gallery to train
		 * @param mv_shapes vector of training shapes (each element contains shapes for each view)
		 * @param curves map of curve definitions, per view
		 * @param augmentation_factors - only relevant in models without similarity alignment
		 * @param stage_scaling The relative scaling of each search stage.
		 * @param align_method Alignment method
		 * @pre is_mv_tracker_initialization_valid(...) == true
		 */
		void initialize_training(mv_tracker_gallery_item gallery_selection,
			const std::vector<multiview_shape2d<double>>& mv_shapes,
			const view_vector<std::vector<simple_curve>>& curves,
			std::array<double, 4> augmentation_factors,
			std::vector<double> stage_scaling,
			alignment_type align_method);


		/**
		 * @brief Add a training example.
		 *
		 * @param view_imgs The set of images (one per view) to add.
		 * @param mv_shape The multiview shape to add.
		 * @param expected_number_of_examples How many images are going to be passed to this?
		 *									This is important because it determines how many
		 *                                  displacements per image to take.
		 * @param min_target_displacements How many displacements?
		 * @param min_samples_per_node A learning smoothness parameter
		 * @pre view_imgs.size() == n_views()
		 * @pre view_shapes.size() = n_views()
		 */
		bool add_training_example(
			const dlib::array<dlib::array2d<dlib::rgb_pixel>>& view_imgs,
			const  multiview_shape2d<double>& mv_shape,
			int expected_number_of_examples,
			int min_target_displacements,
			int min_samples_per_node);
		
		/**
		 * Return a vector of stage scales at each stage.
		 *
		 * @return The scaling at each stage.
		 */
		const std::vector<double> stage_scaling() const;

		/**
		 * Preprocess a set of input images (one for each view), ready for searching.
		 *
		 * @param view_imgs views array of images, one per view
		 * @pre view_imgs.size() = n_views()
		 */
		bool set_images(const dlib::array<dlib::array2d<dlib::rgb_pixel>>& view_imgs);

		/**
		 *	Complete the training after collecting all the data
		 *
		 *  @param min_samples_per_node - the minimum samples per node in RF models
		 */
		void finalize_training(int min_samples_per_node);

		/**
		 * @brief get the number of views in the tracker
		 * @return the number of views
		 */
		int n_views();

		/**
		 * @brief Search the image, previously set using set_images(...).
		 *

		 * @param iterations_per_stage The number of iterations to run at each stage.
		 * @param max_modes_per_stage The maximum number of modes (actually numbers used at each stage are automatically calculated)
		 * @param bilinear Set to true if you want bilinear sampling (slower but more precise)
		 * @param[in,out] mv_shape The starting multiview shape, which will be updated in place.
		 * @pre mv_shape.shapes.size() == n_views()

		 * @return view_vector<std::vector<double>> a per-point fitness measure based on suggested movement
		 * @return a vector of vectors (double type) giving a normalized measure of fitness
		 *         at each iteration of each stage.
		 */
		std::vector<std::vector<double>> autosearch(
			const std::vector<int>& iterations_per_stage,
			int max_modes_per_stage,
			bool bilinear,
			multiview_shape2d<double>& mv_shape
		) const;

		/**
		 * Get a constant reference to the global shape model.
		 *
		 * @return constant ref to the global shape model
		 */
		const linear_pdm<double>& global_pdm() const;

		/**
		 * Construct a basis image to illustrate the sampling regime.
		 *
		 * @param pt_indices The point indices for which to show samples.
		 * @param stage The regression stage to illustrate.
		 * @param view The number of the view
		 * @return A dlib image showing the sampling regime for the given view
		 */
		dlib::array2d<dlib::rgb_pixel> sampling_diagnostic_image(const std::vector<int>& pt_indices, int stage, int view);

		/**
		 * Get the approximate map of minimum training values of scale,angle,x and y
		 *
		 * @return approximate map of minimum values in order s,a,x,y
		 */
		const view_vector<std::array<double, 4>>& local_approx_saxy_min() const;

		/**
		 * Get the approximate map of mid training values of scale,angle,x and y
		 *
		 * @return approximate map of mid values in order s,a,x,y
		 */
		const view_vector<std::array<double, 4>>& local_approx_saxy_mid() const;

		/**
		 * Get the approximate map of maximum training values of scale,angle,x and y
		 *
		 * @return approximate map of maximum values in order s,a,x,y
		 */
		const view_vector<std::array<double, 4>>& local_approx_saxy_max() const;

		/**
		 * Return a reference to the training shapes in the model.
		 *
		 * @return const reference to the training shapes
		 */
		const std::vector<multiview_shape2d<double>>& training_shapes() const;

		/**
		 * Return a reference to the first training shapes in the model.
		 *
		 * \return const reference to the first shapes.
		 */
		const multiview_shape2d<double>& first_training_shapes() const;

		/**
		 * Get the total number of features per point.
		 * 
		 * @return the total number of features for each point.
		 */
		const int total_number_of_features() const;

		/**
		 * Get the number of texture modes.
		 *
		 * @return the number of texture modes.
		 */
		const int n_texture_modes() const;

		/**
		 * @brief Resets the tracker to an untrained state
		 */
		void reset();

	

	private:
		std::variant<
			std::shared_ptr<standard_colour_tracker>,
			std::shared_ptr<tree_colour_tracker>,
			std::shared_ptr<four_level_colour_tracker>,
			std::shared_ptr<fast_colour_tracker>,
			std::shared_ptr<combined_colour_tracker>,
			std::shared_ptr<standard_grey_tracker>,
			std::shared_ptr<tree_grey_tracker>,
			std::shared_ptr<four_level_grey_tracker>,
			std::shared_ptr<fast_grey_tracker>,
			std::shared_ptr<combined_grey_tracker>>
			tracker_ptr_;
	};
}

