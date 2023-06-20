// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "shape_annotation_set.h"
#include "linear_pdm.h"
#include "fixed_pca_model.h"
#include "data_utils.h"
#include "dense_shape.h"
#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/svm.h>
#include <dlib/random_forest.h>
#include <dlib/image_transforms.h>
#include <dlib/image_keypoint.h>
#include <dlib/dnn.h>
RLIBV_RENABLE_WARNINGS
#include <vector>
#include <iostream>

namespace rlibv
{

	/**
   	* \defgroup Tracking Tracking
   	* @{
   	*/

	using dlib::serialize;
	using dlib::deserialize;
	
	typedef void (*training_progress_callback)(float);
	typedef std::array<rlibv::point2d<double>, 4> patch_corners;
	
	/**
	 * @brief A class representing a model for refining shapes to match image data
	 */
	template<int DIM, int STAGES, int TMODES>
	class shape_refiner
	{
		static_assert(STAGES < 6);

	public:
		shape_refiner();

		/**
		 * @brief Check if the refinement model is trained
		 * @returns true if trained
		 */
		bool is_trained() const;

		/**
		 * @brief Train the refiner with true shape and rough shapes
		 * @param images The set of training images
		 * @param rough_shapes The estimated shapes found by some other predictor we want to polish
		 * @param true_shapes The known hand-corrected truth for the shapes
		 * @param patch_scaling The patch scaling factor which controls the size of per-point sampling regions.
		 *						By default, these are set to half the long dimension of the shape, but can be 
		 *                      scaled by this user-provided factor. 
		 * @param n_dense_internals The number of internal points to generate for each dense curve.
		 *                          This is application dependent and usually stored in a shape_annotation_set.
		 * @param progress_callback  Optional pointer to a function of type void f(float) for reporting progress.
		 * @param tp a thread-pool for parallelization of processing
		 * @pre images.size() > 0
		 * @pre images.size() == rough_shapes.size() || rough_shapes.empty()
		 * @pre images.size() == true_shapes.size()
		 * @pre is_training_data_valid(rough_shapes, true_shapes, n_dense_internals)
		 */
		void train(
			const dlib::array<dlib::array2d<dlib::rgb_alpha_pixel>>& images,
			const std::vector<rlibv::shape_annotation>& rough_shapes,
			const std::vector<rlibv::shape_annotation>& true_shapes,
			const std::map<std::string, int>& n_dense_internals,
			double patch_scaling,
			dlib::thread_pool& tp,
			training_progress_callback progress_callback=nullptr);

		/**
		 * @brief Train the refiner with true shapes only
		 * @param images The set of training images
		 * @param true_shapes The known hand-corrected truth for the shapes
		 * @param patch_scaling The patch scaling factor which controls the size of per-point sampling regions.
		 *						By default, these are set to half the long dimension of the shape, but can be
		 *                      scaled by this user-provided factor.
		 * @param n_dense_internals The number of internal points to generate for each dense curve.
		 *                          This is application dependent and usually stored in a shape_annotation_set.
		 * @param tp a thread-pool for parallelization of processing
		 * @pre images.size() > 0
		 * @pre images.size() == rough_shapes.size() || rough_shapes.empty()
		 * @pre images.size() == true_shapes.size()
		 * @pre is_training_data_valid(true_shapes, n_dense_internals)
		 */
		void train(
			const dlib::array<dlib::array2d<dlib::rgb_alpha_pixel>>& images,
			const std::vector<rlibv::shape_annotation>& true_shapes,
			const std::map<std::string, int>& n_dense_internals,
			double patch_scaling, 
			dlib::thread_pool& tp,
			training_progress_callback progress_callback = nullptr);

		/**
		 * @brief Refine the shape
		 * @param initial_shape The estimated shape
		 * @param n_iterations
		 * @param n_basis_functions The number of basis functions to use in the shape model
		 * @param[out] confidence The search confidence
		 * @param tp a thread-pool for parallelization of processing
		 * @return The refined shape estimate
		 * @pre is_annotation_valid(rough_shape) == true
		 * @pre is_trained() == true
		 */
		rlibv::shape_annotation refine(
			const dlib::array2d<dlib::rgb_alpha_pixel>& image,
			const rlibv::shape_annotation& initial_shape,
			int n_iterations,
			int n_basis_functions,
			dlib::thread_pool& tp,
			double& confidence);

		/**
		 * @brief Overloaded function calling refine() using image pointer as input 
		 * @param rows Number of rows in image 
		 * @param cols Number of cols in image 
		 * @param initial_shape The estimated shape 
		 * @param n_iterations
		 * @param n_basis_functions The number of basis functions to use in the shape model
		 * @param[out] confidence The search confidence
		 * @param tp a thread-pool for parallelization of processing
		 * @return The refined shape estimate
		 * @pre is_annotation_valid(rough_shape) == true
		 * @pre is_trained() == true
		 */
		rlibv::shape_annotation refine(
			const unsigned char* image,
			const int rows,
			const int cols,
			const rlibv::shape_annotation& initial_shape,
			int n_iterations,
			int n_basis_functions,
			dlib::thread_pool& tp,
			double& confidence);

		/**
		 * @brief Return the scaling that the user set for this refiner during training 
		 * @return The overall scaling value
		 */
		double user_scaling();

		/**
		 * @brief Check that all shapes and other info use the same scheme and have the correct sizes
		 * @param rough_shapes The vector of rough training shapes (e.g. found by a generic tracker)
		 * @param true_shapes The vector of true training shapes (typically the user hand-edits)
		 * @param n_dense_internals The number of internal curve points the tracker should generate
		 * @param[out] error_msg A description of the problem, if any
		 */
		bool is_training_data_valid(
			const std::vector<rlibv::shape_annotation>& rough_shapes,
			const std::vector<rlibv::shape_annotation>& true_shapes,
			const std::map<std::string, int>& n_dense_internals,
			std::string& error_msg);

		/**
		 * @brief Check that all shapes and other info use the same scheme and have the correct sizes
		 * @param true_shapes The vector of true training shapes (typically the user hand-edits)
		 * @param n_dense_internals The number of internal curve points the tracker should generate
		 * @param[out] error_msg A description of the problem, if any
		 */
		bool is_training_data_valid(
			const std::vector<rlibv::shape_annotation>& true_shapes,
			const std::map<std::string, int>& n_dense_internals,
			std::string& error_msg);


		/**
		 * @brief This function is exposed for UE use
		 * @return curve lookup.
		*/

		const std::map<std::string, std::vector<int>>& get_curve_lookup() const { return curve_lookup_; }

		/**
		 * @brief Extract the raw feature vectors for a given rgb image
		 * @param image The  image
		 * @param d_shape The dense annotation points for this image
		 * @param stage The regression stage
		 * @param object_scale the global scaling of the object (thus applied to the patches)
		 * @param object_rotation the global rotation of the object (thus applied to the patches)
		 * @param tp a thread-pool for parallelization of processing
		 * @returns a vector of 1-D feature column vectors
		 */
		template<int STAGE>
		std::vector<dlib::matrix<float, (STAGE+1)*DIM*DIM*3, 1>> get_raw_features(
			const std::vector<point2d<double>>& shape,
			double object_scale,
			double object_rotation, 
			dlib::thread_pool& tp
		);

		/**
		 * @brief Get the corners of a given local tower of patches
		 * @param pt the centre point
		 * @param stage the search stage
		 * @param object_scale the global scaling of the object (thus applied to the patches)
		 * @param object_rotation the global rotation of the object (thus applied to the patches)
		 * @return array of patch_corner structures
		 */
		std::vector<patch_corners> patch_tower(const rlibv::point2d<double>& pt, int stage, double object_scale, double object_rotation);

		/**
		 * @brief Get the corners of all the patch towers in the local frame of reference
		 * @param stage the search stage
		 * @return array of patch_corner structures
		 */
		std::vector<patch_corners> reference_patch_tower(int stage);

		/**
		 * @brief Check if the model uses a delta constraint (i.e. it's a rough->sharp refiner
		 * @return True if the model uses delta constraint
		 */
		bool uses_delta_constraint();

		/**
		 * @brief Get the total number of dense points
		 * @return The number of points
		 */
		int n_dense_points();


		/**
		 * @brief Serialization
		 * @param item The shape_annotation
		 * @param out The output stream
		 */
		template<int DIM, int STAGES, int TMODES>
		friend void serialize(const shape_refiner<DIM, STAGES, TMODES>& item, std::ostream& out);

		/**
		 * @brief Deserialization
		 * @param item The resulting shape_annotation
		 * @param in The input stream
		 */
		template<int DIM, int STAGES, int TMODES>
		friend void deserialize(shape_refiner<DIM, STAGES, TMODES>& item, std::istream& in);

	private:
		void set_image(const dlib::array2d<dlib::rgb_alpha_pixel>* img, dlib::thread_pool & tp);

		static constexpr auto relative_patch_scales_ = [] {
			std::array<double, STAGES> a{};
			for (int i = 0; i < STAGES; ++i) {
				a[i] = 1.0;
				for (int j = 0; j < i; ++j)
				{
					a[i] /= 2.0;
				}
			}
			return a;
		}();

		const int lower_res_stack_depth_ = 4;
		std::map<std::string, int> n_dense_internals_;
		bool is_trained_ = false;
		bool use_delta_constraint_ = false;
		rlibv::linear_pdm<double> shape_model_;
		
		std::string input_tracker_name_ = "";
		long long input_tracker_size_check_ = 0;
		std::map<std::string, std::vector<int>> curve_lookup_;
		std::map<std::string, int> keypoint_lookup_;
		double patch_scaling_ = 1.0;
		pca_model<double> delta_model_;
		std::vector<point2d<double>> reference_shape_;
		double default_relative_patch_width_ = 1.0;

		std::vector<rlibv::fixed_pca_model<float, 1 * DIM * DIM * 3, TMODES>> vector_reducers_stage_0_;
		std::vector<rlibv::fixed_pca_model<float, 2 * DIM * DIM * 3, TMODES>> vector_reducers_stage_1_;
		std::vector<rlibv::fixed_pca_model<float, 3 * DIM * DIM * 3, TMODES>> vector_reducers_stage_2_;
		std::vector<rlibv::fixed_pca_model<float, 4 * DIM * DIM * 3, TMODES>> vector_reducers_stage_3_;
		std::vector<rlibv::fixed_pca_model<float, 5 * DIM * DIM * 3, TMODES>> vector_reducers_stage_4_;
		std::vector<rlibv::fixed_pca_model<float, 6 * DIM * DIM * 3, TMODES>> vector_reducers_stage_5_;
		std::array<std::vector<dlib::vector_normalizer<dlib::matrix<float, TMODES,1>>>, STAGES> vector_normalizers_;

		using net_type = dlib::loss_mean_squared_multioutput<
			dlib::fc<2,
			dlib::leaky_relu<
			dlib::fc<TMODES,
			dlib::leaky_relu<
			dlib::fc<TMODES,
			dlib::input<dlib::matrix<float, TMODES,1>>
			>>>>>>;

		std::array<std::vector<net_type>, STAGES> nn_predictors_;
		const dlib::array2d<dlib::rgb_alpha_pixel>* img_ = nullptr;
		dlib::array<dlib::array2d<dlib::rgb_alpha_pixel>> lower_res_stack_;
		dlib::array2d<long> red_ii_;
		dlib::array2d<long> green_ii_;
		dlib::array2d<long> blue_ii_;

		/*
		* @brief A structure to store the training data for each individual point
		*/
		template<int STAGE>
		struct per_point_nn_training_data
		{
			std::vector<dlib::matrix<float>> delta_data;
			std::vector<dlib::matrix<float, (STAGE+1)*DIM* DIM * 3, 1>> feature_samples;
		};
	};



	/**@}*/
}

#include "impl/shape_refiner.hpp"

