// Copyright Epic Games, Inc. All Rights Reserved.

#include "data_utils.h"
#include "simple_profiler.h"
#include "linear_range.h"
#include "dense_shape.h"
#include "transforms.h"
#include "geometry.h"
#include "disable_dlib_warnings.h"
#include "ThirdParty/dlib/fast_image_chips.h"

RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/image_transforms.h>
#include <dlib/image_keypoint.h>
#include <dlib/gui_widgets.h>
#include <dlib/gui_widgets/widgets.h>
#include <dlib/threads.h>
RLIBV_RENABLE_WARNINGS

#include <execution>
#include <thread>


namespace rlibv
{
	inline int ipow(int num, int exp)
	{   
		if (exp == 0)
		{
			return 1;
		}
		int p = ipow(num, exp / 2);
		return (exp % 2 == 0) ? p * p : num * p * p;
	}

	template<int DIM, int STAGES, int TMODES>
	template<int STAGE>
	std::vector<dlib::matrix<float, (STAGE+1)* DIM* DIM * 3, 1>> shape_refiner<DIM, STAGES, TMODES>::get_raw_features(
		const std::vector<point2d<double>>& shape,
		double object_scale,
		double object_rotation,
		dlib::thread_pool & tp
	)
	{
		int n_points = static_cast<int>(shape.size());
		int n_levels = STAGE + 1;
		std::vector<dlib::matrix<float, (STAGE + 1)* DIM* DIM * 3, 1>> result(n_points);
		double d_patch_width = DIM;
		dlib::chip_dims dims(DIM, DIM);

		std::vector<rlibv::point2d<double>> chip_points_{
		{ 0,0 },
		{ d_patch_width,0 },
		{ d_patch_width,d_patch_width },
		{ 0,d_patch_width } };

		std::vector<dlib::chip_details> all_chip_details(n_points * n_levels);
		std::vector<int> pt_inds = rlibv::linear_range<int>(0, n_points - 1, n_points);
		dlib::parallel_for(tp, 0, pt_inds.size(), [&](const long p)
			{
				rlibv::point2d<double> pt(shape[pt_inds[p]].x(), shape[pt_inds[p]].y());
				auto tower = patch_tower(pt, STAGE, object_scale, object_rotation);
				for (int l=0; l<= STAGE; ++l)
				{
					const auto& level = tower[l];
					std::vector<rlibv::point2d<double>> img_points{ level[0], level[1], level[2], level[3] };
					all_chip_details[pt_inds[p] * n_levels + l] = dlib::chip_details(chip_points_, img_points, dlib::chip_dims(DIM, DIM));
				}
			},1);

		dlib::array<dlib::array2d<dlib::rgb_alpha_pixel>> raw_samples(n_points * n_levels);
		rlibv::fast_extract_image_chips(img_, lower_res_stack_, all_chip_details, raw_samples, tp);

		dlib::parallel_for(tp, 0, pt_inds.size(), [&](const long p)
			{
				dlib::matrix<float, (STAGE + 1) * DIM* DIM * 3, 1> sample;
				int step = n_levels;
				int count = 0;
				for (int i = 0; i < n_levels; ++i)
				{
					auto& patch = raw_samples[pt_inds[p] * step + i];
					for (int r = 0; r < DIM; ++r)
					{
						for (int c = 0; c < DIM; ++c)
						{
							sample(count++) = patch[r][c].red;
							sample(count++) = patch[r][c].blue;
							sample(count++) = patch[r][c].green;
						}
					}
				}
				result[pt_inds[p]] = sample;
			},1);
		return result;
	}

	template<int DIM, int STAGES, int TMODES>
	rlibv::shape_refiner<DIM, STAGES, TMODES>::shape_refiner()
	{
		lower_res_stack_.resize(lower_res_stack_depth_);
	}

	template<int DIM, int STAGES, int TMODES>
	void rlibv::shape_refiner<DIM, STAGES, TMODES>::set_image(const dlib::array2d<dlib::rgb_alpha_pixel> *img,
		dlib::thread_pool & tp)
	{
		img_ = img;
		
		if (lower_res_stack_.size() != lower_res_stack_depth_)
		{
			lower_res_stack_.set_size(lower_res_stack_depth_);
		}

		const dlib::array2d<dlib::rgb_alpha_pixel>* prev_image_ = nullptr;
		for (int level = 0; level < lower_res_stack_depth_; ++level)
		{
			if (level>0)
			{
				prev_image_ = &(lower_res_stack_[level - 1]);
			}

			dlib::array2d<dlib::rgb_alpha_pixel>* target_image_ = &(lower_res_stack_[level]);

			if (level > 0)
			{
				target_image_->set_size(prev_image_->nr() / 2, prev_image_->nc() / 2);
			}
			else
			{
				target_image_->set_size(img_->nr() / 2, img_->nc() / 2);
			}
			
			dlib::parallel_for(tp, 0, target_image_->nr(), [&](const long r)
				{
					int source_tl_r = 2*r;
					int source_tl_c = 0;
					for (int c = 0; c < target_image_->nc(); ++c)
					{
						dlib::rgb_alpha_pixel* target_pixel = &(*target_image_)[r][c];

						dlib::rgb_alpha_pixel source_pixel_1;
						dlib::rgb_alpha_pixel source_pixel_2;
						dlib::rgb_alpha_pixel source_pixel_3;
						dlib::rgb_alpha_pixel source_pixel_4;

						if (level > 0)
						{
							source_pixel_1 = (*prev_image_)[source_tl_r][source_tl_c];
							source_pixel_2 = (*prev_image_)[source_tl_r][source_tl_c+1];
							source_pixel_3 = (*prev_image_)[source_tl_r+1][source_tl_c];
							source_pixel_4 = (*prev_image_)[source_tl_r+1][source_tl_c + 1];
						}
						else
						{
							source_pixel_1 = (*img_)[source_tl_r][source_tl_c];
							source_pixel_2 = (*img_)[source_tl_r][source_tl_c + 1];
							source_pixel_3 = (*img_)[source_tl_r + 1][source_tl_c];
							source_pixel_4 = (*img_)[source_tl_r + 1][source_tl_c + 1];
						}

						target_pixel->red = static_cast<unsigned char>(
							( static_cast<int>(source_pixel_1.red) 
							+ static_cast<int>(source_pixel_2.red)
							+ static_cast<int>(source_pixel_3.red)
							+ static_cast<int>(source_pixel_4.red)) / 4
							);
						target_pixel->green = static_cast<unsigned char>(
							(static_cast<int>(source_pixel_1.green)
								+ static_cast<int>(source_pixel_2.green)
								+ static_cast<int>(source_pixel_3.green)
								+ static_cast<int>(source_pixel_4.green)) / 4
							);
						target_pixel->blue = static_cast<unsigned char>(
							(static_cast<int>(source_pixel_1.blue)
								+ static_cast<int>(source_pixel_2.blue)
								+ static_cast<int>(source_pixel_3.blue)
								+ static_cast<int>(source_pixel_4.blue)) / 4
							);
						target_pixel->alpha = static_cast<unsigned char>(
							(static_cast<int>(source_pixel_1.alpha)
								+ static_cast<int>(source_pixel_2.alpha)
								+ static_cast<int>(source_pixel_3.alpha)
								+ static_cast<int>(source_pixel_4.alpha)) / 4
							);
						source_tl_c += 2;
					}
				});
			
		}
		// Sanity-check debug visualization
		//dlib::image_window winm1(*img, "orig");
		//winm1.wait_until_closed();
		//dlib::image_window win0(lower_res_stack_[0], "0");
		//win0.wait_until_closed();
		//dlib::image_window win1(lower_res_stack_[1], "1");
		//win1.wait_until_closed();

	}

	template<int DIM, int STAGES, int TMODES>
	bool rlibv::shape_refiner<DIM, STAGES, TMODES>::uses_delta_constraint()
	{
		return use_delta_constraint_;
	}

	template<int DIM, int STAGES, int TMODES>
	bool shape_refiner<DIM, STAGES, TMODES>::is_trained() const
	{
		return is_trained_;
	}

	template<int DIM, int STAGES, int TMODES>
	void shape_refiner<DIM, STAGES, TMODES>::train(
		const dlib::array<dlib::array2d<dlib::rgb_alpha_pixel>>& training_images,
		const std::vector<rlibv::shape_annotation>& rough_training_shapes,
		const std::vector<rlibv::shape_annotation>& true_training_shapes,
		const std::map<std::string, int>& n_dense_internals,
		double patch_scaling,
		dlib::thread_pool& tp,
		training_progress_callback progress_callback)
	{
		std::string msg;
		DLIB_CASSERT(training_images.size() > 0);
		DLIB_CASSERT(training_images.size() == rough_training_shapes.size() || rough_training_shapes.empty());
		DLIB_CASSERT(training_images.size() == true_training_shapes.size());
		DLIB_CASSERT(is_training_data_valid(rough_training_shapes, true_training_shapes, n_dense_internals, msg));

		n_dense_internals_ = n_dense_internals;
		patch_scaling_ = patch_scaling;

		// We need to call this, just to get the curve_lookup_ and keypoint_lookup_
		std::vector<std::vector<int>> temp1;
		std::vector<std::vector<int>> temp2;
		auto temp3 = true_training_shapes[0].get_dense_points(
			100,
			100,
			temp1,
			temp2,
			curve_lookup_,
			keypoint_lookup_,
			n_dense_internals_);

		// Establish a reference shape (the first true shape will do) and find the
		// sample patches for this shape
		dense_shape dense_reference;
		dense_reference.initialize_from_shape_annotation(true_training_shapes[0], training_images[0].nc(), training_images[0].nr(), n_dense_internals_);
		reference_shape_ = dense_reference.const_points();

		int n_points = static_cast<int>(reference_shape_.size());
		int max_progress = STAGES * n_points + static_cast<int>(training_images.size());
		std::atomic<int> progress;
	
		auto ranges = rlibv::minx_maxx_miny_maxy(reference_shape_);
		default_relative_patch_width_ = std::max(ranges(1) - ranges(0), ranges(3) - ranges(2)) * 0.5;
		
		// Build a shape model, and, if rough points are provided, a model of acceptable delta to the rough shape
		use_delta_constraint_ = !rough_training_shapes.empty();
	
		std::vector<dlib::matrix<double, 0, 1>> delta_data;
		std::vector<shape2d<double>> shape_training_shapes;
		for (size_t i = 0; i < true_training_shapes.size(); ++i)
		{
			dense_shape dense_true;
			dense_true.initialize_from_shape_annotation(
				true_training_shapes[i], 
				training_images[i].nc(), 
				training_images[i].nr(), 
				n_dense_internals_);
			shape2d<double> true_shape = dense_true.points();
			shape_training_shapes.emplace_back(true_shape);

			if (use_delta_constraint_)
			{
				auto true_tform = rlibv::find_similarity_transform_as_projective(reference_shape_, true_shape);
				std::vector<point2d<double>> true_aligned = transform_shape(true_shape, inv(true_tform));

				dense_shape dense_rough;
				dense_rough.initialize_from_shape_annotation(
					rough_training_shapes[i],
					training_images[i].nc(),
					training_images[i].nr(),
					n_dense_internals_);
				shape2d<double> rough_shape = dense_rough.points();

				std::vector<point2d<double>> rough_aligned = transform_shape(rough_shape, inv(true_tform));

				dlib::matrix<double, 0, 1> delta;
				delta.set_size(static_cast<long>(rough_aligned.size() * 2));
				for (int p = 0; p < static_cast<int>(rough_aligned.size()); ++p)
				{
					delta(p * 2) = rough_aligned[p].x() - true_aligned[p].x();
					delta(p * 2 + 1) = rough_aligned[p].y() - true_aligned[p].y();
				}
				delta_data.emplace_back(delta);
			}
			
		}
		shape_model_.train_linear_pdm(shape_training_shapes, 1.0, 99, alignment_type::similarity, std::vector<int>());
		if (use_delta_constraint_)
		{
			delta_model_.fast_train_to_variance(delta_data, 1.0, 99);
		}

		// Prepare things for extracting samples
		std::vector<per_point_nn_training_data<0>> nn_training_data_stage_0; nn_training_data_stage_0.resize(n_points);
		std::vector<per_point_nn_training_data<1>> nn_training_data_stage_1; nn_training_data_stage_1.resize(n_points);
		std::vector<per_point_nn_training_data<2>> nn_training_data_stage_2; nn_training_data_stage_2.resize(n_points);
		std::vector<per_point_nn_training_data<3>> nn_training_data_stage_3; nn_training_data_stage_3.resize(n_points);
		std::vector<per_point_nn_training_data<4>> nn_training_data_stage_4; nn_training_data_stage_4.resize(n_points);
		std::vector<per_point_nn_training_data<5>> nn_training_data_stage_5; nn_training_data_stage_5.resize(n_points);

		// Now make small displacements around each true shape.
		int n_examples = static_cast<int>(true_training_shapes.size());
		int n_displacements_per_stages = std::max(3, 3000 / n_examples);
		int n_total_disps = n_displacements_per_stages * n_examples;
		for (int stage = 0; stage < STAGES; ++stage)
		{
			for (int p = 0; p < n_points; ++p)
			{
				if (stage == 0) 
				{nn_training_data_stage_0[p].delta_data.resize(n_total_disps); nn_training_data_stage_0[p].feature_samples.resize(n_total_disps);}
				else if (stage == 1) 
				{ nn_training_data_stage_1[p].delta_data.resize(n_total_disps); nn_training_data_stage_1[p].feature_samples.resize(n_total_disps); }
				else if (stage == 2) 
				{ nn_training_data_stage_2[p].delta_data.resize(n_total_disps); nn_training_data_stage_2[p].feature_samples.resize(n_total_disps); }
				else if (stage == 3)
				{ nn_training_data_stage_3[p].delta_data.resize(n_total_disps); nn_training_data_stage_3[p].feature_samples.resize(n_total_disps); }
				else if (stage == 4)
				{ nn_training_data_stage_4[p].delta_data.resize(n_total_disps); nn_training_data_stage_4[p].feature_samples.resize(n_total_disps); }
				else if (stage == 5) 
				{ nn_training_data_stage_5[p].delta_data.resize(n_total_disps); nn_training_data_stage_5[p].feature_samples.resize(n_total_disps); }
			}
		}

		std::vector<int> img_inds = rlibv::linear_range<int>(0, n_examples - 1, n_examples);
		for(int i=0; i< n_examples; ++i)
		{
			set_image(&(training_images[i]), tp);

			dlib::rand rng("THIS STRING MEANS WE USE THE SAME SEED EACH TIME");
				
			dense_shape true_dense;
			true_dense.initialize_from_shape_annotation(true_training_shapes[i], training_images[i].nc(), training_images[i].nr(), n_dense_internals_);
			dense_shape displaced_dense = true_dense;

			std::vector<point2d<double>> true_shape = true_dense.const_points();
			auto tform = rlibv::find_similarity_transform_as_projective(reference_shape_, true_shape);
			std::vector<point2d<double>> true_aligned = transform_shape(true_shape, inv(tform));
			std::vector<point2d<double>> displaced_aligned(true_aligned.size());
			auto saxy = rlibv::approx_similarity_transform<double>(tform);

			for (int stage = 0; stage < STAGES; ++stage)
			{
				for (int d = 0; d < n_displacements_per_stages; ++d)
				{
					int index = i * n_displacements_per_stages + d;
					double delta = 0.25 * relative_patch_scales_[stage] * default_relative_patch_width_ * patch_scaling_;
					double dx = rng.get_double_in_range(-delta, delta);
					double dy = rng.get_double_in_range(-delta, delta);
					for (int p = 0; p < n_points; ++p)
					{
						displaced_aligned[p] = true_aligned[p] + point2d<double>(dx, dy);
						dlib::matrix<float,2,1> delta;
						delta(0) = static_cast<float>(dx);
						delta(1) = static_cast<float>(dy);

						if (stage == 0) { nn_training_data_stage_0[p].delta_data[index] = delta; }
						if (stage == 1) { nn_training_data_stage_1[p].delta_data[index] = delta; }
						if (stage == 2) { nn_training_data_stage_2[p].delta_data[index] = delta; }
						if (stage == 3) { nn_training_data_stage_3[p].delta_data[index] = delta; }
						if (stage == 4) { nn_training_data_stage_4[p].delta_data[index] = delta; }
						if (stage == 5) { nn_training_data_stage_5[p].delta_data[index] = delta; }
					}
					displaced_dense.points() = transform_shape(displaced_aligned, tform);

					std::vector<dlib::matrix<float, 1 * DIM * DIM * 3, 1>> nn_features_stage_0;
					std::vector<dlib::matrix<float, 2 * DIM * DIM * 3, 1>> nn_features_stage_1;
					std::vector<dlib::matrix<float, 3 * DIM * DIM * 3, 1>> nn_features_stage_2;
					std::vector<dlib::matrix<float, 4 * DIM * DIM * 3, 1>> nn_features_stage_3;
					std::vector<dlib::matrix<float, 5 * DIM * DIM * 3, 1>> nn_features_stage_4;
					std::vector<dlib::matrix<float, 6 * DIM * DIM * 3, 1>> nn_features_stage_5;

					if (stage == 0) { nn_features_stage_0 = get_raw_features<0>(displaced_dense.const_points(),saxy[0],saxy[1], tp); }
					else if (stage == 1) { nn_features_stage_1 = get_raw_features<1>(displaced_dense.const_points(), saxy[0], saxy[1], tp); }
					else if (stage == 2) { nn_features_stage_2 = get_raw_features<2>(displaced_dense.const_points(), saxy[0], saxy[1], tp); }
					else if (stage == 3) { nn_features_stage_3 = get_raw_features<3>(displaced_dense.const_points(), saxy[0], saxy[1], tp); }
					else if (stage == 4) { nn_features_stage_4 = get_raw_features<4>(displaced_dense.const_points(), saxy[0], saxy[1], tp); }
					else if (stage == 5) { nn_features_stage_5 = get_raw_features<5>(displaced_dense.const_points(), saxy[0], saxy[1], tp); }
					
					for (int p = 0; p < n_points; ++p)
					{
						if (stage == 0) { nn_training_data_stage_0[p].feature_samples[index] = nn_features_stage_0[p]; }
						if (stage == 1) { nn_training_data_stage_1[p].feature_samples[index] = nn_features_stage_1[p]; }
						if (stage == 2) { nn_training_data_stage_2[p].feature_samples[index] = nn_features_stage_2[p]; }
						if (stage == 3) { nn_training_data_stage_3[p].feature_samples[index] = nn_features_stage_3[p]; }
						if (stage == 4) { nn_training_data_stage_4[p].feature_samples[index] = nn_features_stage_4[p]; }
						if (stage == 5) { nn_training_data_stage_5[p].feature_samples[index] = nn_features_stage_5[p]; }
					}
				}
			}
			
			if (progress_callback != nullptr)
			{
				progress_callback(static_cast<float>(progress) / static_cast<float>(max_progress));
				progress++;
			}
		}

		vector_reducers_stage_0_.resize(n_points);
		vector_reducers_stage_1_.resize(n_points);
		vector_reducers_stage_2_.resize(n_points);
		vector_reducers_stage_3_.resize(n_points);
		vector_reducers_stage_4_.resize(n_points);
		vector_reducers_stage_5_.resize(n_points);

		for (int stage = 0; stage < STAGES; ++stage)
		{
			nn_predictors_[stage].resize(n_points);
			vector_normalizers_[stage].resize(n_points);

			std::vector<int> pt_inds = rlibv::linear_range<int>(0, n_points - 1, n_points);
			
			dlib::parallel_for(tp, 0, n_points, [&](int p)
				{
					if (0 == stage) { vector_reducers_stage_0_[p].train(nn_training_data_stage_0[p].feature_samples); }
					else if (1 == stage) { vector_reducers_stage_1_[p].train(nn_training_data_stage_1[p].feature_samples); }
					else if (2 == stage) { vector_reducers_stage_2_[p].train(nn_training_data_stage_2[p].feature_samples); }
					else if (3 == stage) { vector_reducers_stage_3_[p].train(nn_training_data_stage_3[p].feature_samples); }
					else if (4 == stage) { vector_reducers_stage_4_[p].train(nn_training_data_stage_4[p].feature_samples); }
					else if (5 == stage) { vector_reducers_stage_5_[p].train(nn_training_data_stage_5[p].feature_samples); }

					int n_samples = static_cast<int>(nn_training_data_stage_0[p].feature_samples.size());
					std::vector<dlib::matrix<float, TMODES, 1>> reduced(n_samples);
					for (int e = 0; e < n_samples; ++e)
					{
						if (0 == stage) { reduced[e] = vector_reducers_stage_0_[p].parameterize(nn_training_data_stage_0[p].feature_samples[e]); }
						else if (1 == stage) { reduced[e] = vector_reducers_stage_1_[p].parameterize(nn_training_data_stage_1[p].feature_samples[e]); }
						else if (2 == stage) { reduced[e] = vector_reducers_stage_2_[p].parameterize(nn_training_data_stage_2[p].feature_samples[e]); }
						else if (3 == stage) { reduced[e] = vector_reducers_stage_3_[p].parameterize(nn_training_data_stage_3[p].feature_samples[e]); }
						else if (4 == stage) { reduced[e] = vector_reducers_stage_4_[p].parameterize(nn_training_data_stage_4[p].feature_samples[e]); }
						else if (5 == stage) { reduced[e] = vector_reducers_stage_5_[p].parameterize(nn_training_data_stage_5[p].feature_samples[e]); }
					}
					vector_normalizers_[stage][p].train(reduced);
					std::vector<dlib::matrix<float,TMODES,1>> training_inputs(reduced.size());
					for (int e = 0; e < reduced.size(); ++e)
					{
						training_inputs[e] = vector_normalizers_[stage][p](reduced[e]);
					}

					dlib::dnn_trainer<net_type, dlib::adam> nn_trainer(nn_predictors_[stage][p], dlib::adam());
					nn_trainer.set_learning_rate(0.01);
					nn_trainer.set_min_learning_rate(0.0000001);
					nn_trainer.set_mini_batch_size(128);
					nn_trainer.set_max_num_epochs(500);
					//nn_trainer.be_verbose();
					if (0 == stage) { nn_trainer.train(training_inputs, nn_training_data_stage_0[p].delta_data); }
					else if (1 == stage) { nn_trainer.train(training_inputs, nn_training_data_stage_1[p].delta_data); }
					else if (2 == stage) { nn_trainer.train(training_inputs, nn_training_data_stage_2[p].delta_data); }
					else if (3 == stage) { nn_trainer.train(training_inputs, nn_training_data_stage_3[p].delta_data); }
					else if (4 == stage) { nn_trainer.train(training_inputs, nn_training_data_stage_4[p].delta_data); }
					else if (5 == stage) { nn_trainer.train(training_inputs, nn_training_data_stage_5[p].delta_data); }
					
					nn_predictors_[stage][p].clean();

					if (progress_callback != nullptr)
					{
						progress_callback(static_cast<float>(progress) / static_cast<float>(max_progress));
						progress++;
					}
				});
		}
		is_trained_ = true;
	}

	template<int DIM, int STAGES, int TMODES>
	void shape_refiner<DIM, STAGES, TMODES>::train(const dlib::array<dlib::array2d<dlib::rgb_alpha_pixel>>& images, const std::vector<rlibv::shape_annotation>& true_shapes, const std::map<std::string, int>& n_dense_internals, double patch_scaling, dlib::thread_pool& tp,
		training_progress_callback progress_callback)
	{
		std::string msg;
		DLIB_ASSERT(is_training_data_valid(true_shapes, n_dense_internals, msg));
		std::vector<rlibv::shape_annotation> rough_shapes;
		train(images, rough_shapes, true_shapes, n_dense_internals, patch_scaling, progress_callback, tp);
	}


	template<int DIM, int STAGES, int TMODES>
	int rlibv::shape_refiner<DIM, STAGES, TMODES>::n_dense_points()
	{
		return shape_model_.n_points();
	}

	template<int DIM, int STAGES, int TMODES>
	rlibv::shape_annotation shape_refiner<DIM, STAGES, TMODES>::refine(
		const dlib::array2d<dlib::rgb_alpha_pixel>& image,
		const rlibv::shape_annotation& initial_shape,
		int n_iterations,
		int n_basis_functions,
		dlib::thread_pool& tp,
		double& confidence)
	{
		std::string msg;
		DLIB_ASSERT(is_trained());
	
		set_image(&image, tp);
	
		dense_shape dense_rough;
		dense_rough.initialize_from_shape_annotation(initial_shape, img_->nc(), img_->nr(), n_dense_internals_);
		dense_shape dense_current(dense_rough);
		std::vector<point2d<double>> current_aligned;
		std::vector<point2d<double>> original_points = dense_rough.points();
		int n_points = static_cast<int>(reference_shape_.size());
		std::vector<dlib::chip_details> all_chip_details(n_points * relative_patch_scales_.size());

		std::vector<dlib::matrix<float, 1 * DIM * DIM * 3, 1>> nn_features_stage_0;
		std::vector<dlib::matrix<float, 2 * DIM * DIM * 3, 1>> nn_features_stage_1;
		std::vector<dlib::matrix<float, 3 * DIM * DIM * 3, 1>> nn_features_stage_2;
		std::vector<dlib::matrix<float, 4 * DIM * DIM * 3, 1>> nn_features_stage_3;
		std::vector<dlib::matrix<float, 5 * DIM * DIM * 3, 1>> nn_features_stage_4;
		std::vector<dlib::matrix<float, 6 * DIM * DIM * 3, 1>> nn_features_stage_5;

		for (int it = 0; it < n_iterations; ++it)
		{
			int stage = (it * STAGES) / n_iterations;
			dlib::point_transform_projective tform;
			std::array<double, 4> saxy;
			std::vector<point2d<double>> current_shape = dense_current.const_points();
			
			tform = rlibv::find_similarity_transform_as_projective(reference_shape_, current_shape);
			current_aligned = transform_shape(current_shape, inv(tform));
			saxy = approx_similarity_transform<double>(tform);

			if (stage == 0) { nn_features_stage_0 = get_raw_features<0>(current_shape, saxy[0], saxy[1], tp); }
			else if (stage == 1) { nn_features_stage_1 = get_raw_features<1>(current_shape, saxy[0], saxy[1], tp); }
			else if (stage == 2) { nn_features_stage_2 = get_raw_features<2>(current_shape, saxy[0], saxy[1], tp); }
			else if (stage == 3) { nn_features_stage_3 = get_raw_features<3>(current_shape, saxy[0], saxy[1], tp); }
			else if (stage == 4) { nn_features_stage_4 = get_raw_features<4>(current_shape, saxy[0], saxy[1], tp); }
			else if (stage == 5) { nn_features_stage_5 = get_raw_features<5>(current_shape, saxy[0], saxy[1], tp); }
			
			dlib::matrix<double, 0, 1> delta;
			delta.set_size(static_cast<long>(n_points * 2));

			dlib::parallel_for(tp, 0, n_points, [&](const long p)
				{
					dlib::matrix<float, TMODES, 1> normalized;
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 28020) // disabling static analysis warning "The expression <expr> is not true at this call"
#endif
					if (0 == stage) { normalized = vector_normalizers_[stage][p](vector_reducers_stage_0_[p].parameterize(nn_features_stage_0[p])); }
					else if (1 == stage) { normalized = vector_normalizers_[stage][p](vector_reducers_stage_1_[p].parameterize(nn_features_stage_1[p])); }
					else if (2 == stage) { normalized = vector_normalizers_[stage][p](vector_reducers_stage_2_[p].parameterize(nn_features_stage_2[p])); }
					else if (3 == stage) { normalized = vector_normalizers_[stage][p](vector_reducers_stage_3_[p].parameterize(nn_features_stage_3[p])); }
					else if (4 == stage) { normalized = vector_normalizers_[stage][p](vector_reducers_stage_4_[p].parameterize(nn_features_stage_4[p])); }
					else if (5 == stage) { normalized = vector_normalizers_[stage][p](vector_reducers_stage_5_[p].parameterize(nn_features_stage_5[p])); }
#ifdef _MSC_VER
#pragma warning(pop)
#endif					
					dlib::matrix<float,2,1> prediction = nn_predictors_[stage][p](normalized);
					double dx = prediction(0);
					double dy = prediction(1);
					delta(2 * p) = dx;
					delta(2 * p + 1) = dy;
				});

			if (use_delta_constraint_)
			{
				delta = delta_model_.reconstruct(delta_model_.parameterize(delta, 1.0, 999));
			}

			for (size_t p = 0; p < static_cast<size_t>(n_points); p++)
			{
				current_aligned[p].x() -= delta(2 * p);
				current_aligned[p].y() -= delta(2 * p + 1);
			}
			
			current_shape = transform_shape(current_aligned, tform);

			dlib::matrix<double, 0, 1> parameters;
			dlib::point_transform_projective pose;
			std::vector<rlibv::point2d<double>> residual;
			shape_model_.parameterize(current_shape, parameters, pose, residual, 99.9, n_basis_functions);
			shape_model_.reconstruct(parameters, pose, residual, std::vector<double>(residual.size(), 0.0), current_shape);

			dense_current.set_points(current_shape);
			
		}

		std::vector<point2d<double>> normalized_points(dense_current.points());
		for (int p = 0; p < normalized_points.size(); ++p)
		{
			normalized_points[p].x() /= img_->nc();
			normalized_points[p].y() /= img_->nr();
		}

		rlibv::shape_annotation result(initial_shape);
		result.set_from_dense_points(normalized_points, curve_lookup_);
		confidence = 1.0; //TODO Real measure of confidence
		
		return result;
	}

	template<int DIM, int STAGES, int TMODES>
	rlibv::shape_annotation shape_refiner<DIM, STAGES, TMODES>::refine(
		const unsigned char *image,
		const int rows,
		const int cols,
		const rlibv::shape_annotation& initial_shape,
		int n_iterations,
		int n_basis_functions,
		dlib::thread_pool& tp,
		double& confidence)
	{
		const int channels = 4; // number of channels (4 for RGBA pixels)
		dlib::array2d<dlib::rgb_alpha_pixel> dlib_image_generated;
		set_image_size(dlib_image_generated, rows, cols);

		// convert image to dlib image
		for (int r = 0; r < rows; ++r)
			for (int c = 0; c < cols; ++c)
			{
				dlib_image_generated[r][c].red   = reinterpret_cast<const unsigned char&>(image[channels * (r * cols + c) + 0]);
				dlib_image_generated[r][c].green = reinterpret_cast<const unsigned char&>(image[channels * (r * cols + c) + 1]);
				dlib_image_generated[r][c].blue  = reinterpret_cast<const unsigned char&>(image[channels * (r * cols + c) + 2]);
				dlib_image_generated[r][c].alpha = reinterpret_cast<const unsigned char&>(image[channels * (r * cols + c) + 3]);
			}

		auto result = refine(dlib_image_generated, initial_shape, n_iterations, n_basis_functions, tp, confidence);

		return result;
	}

	template<int DIM, int STAGES, int TMODES>
	double shape_refiner<DIM, STAGES, TMODES>::user_scaling()
	{
		return patch_scaling_;
	}


	template<int DIM, int STAGES, int TMODES>
	bool shape_refiner<DIM, STAGES, TMODES>::is_training_data_valid(
		const std::vector<rlibv::shape_annotation>& true_shapes,
		const std::map<std::string, int>& n_dense_internals,
		std::string& error_msg)
	{
		error_msg.clear();

		for (size_t i = 0; i < true_shapes.size(); ++i)
		{
			if (!true_shapes[i].matches_scheme(true_shapes[0], error_msg))
			{
				error_msg = "One of the true shapes doesn't match the annotation scheme.";
				return false;
			}
		}

		for (const auto& item : n_dense_internals)
		{
			if (!map_contains_key(true_shapes[0].keypoint_curves(), item.first))
			{
				error_msg = "keypoint curve name missing from n_dense_internals";
				return false;
			}
		}

		for (const auto& item : true_shapes[0].keypoint_curves())
		{
			if (!map_contains_key(n_dense_internals, item.first))
			{
				error_msg = "unknown keypoint curve name found in n_dense_internals";
				return false;
			}
		}
		return true;
	}

	template<int DIM, int STAGES, int TMODES>
	bool shape_refiner<DIM, STAGES, TMODES>::is_training_data_valid(
		const std::vector<rlibv::shape_annotation>& rough_shapes, 
		const std::vector<rlibv::shape_annotation>& true_shapes, 
		const std::map<std::string, int>& n_dense_internals,
		std::string& error_msg)
	{
		error_msg.clear();
		if (true_shapes.size() != rough_shapes.size() && !rough_shapes.empty())
		{
			error_msg = "true_shapes.size() != rough_shapes.size() && !rough_shapes.empty()";
			return false;
		}

		if(!rough_shapes.empty())
		{
			for (size_t i = 0; i < rough_shapes.size(); ++i)
			{
				if (!rough_shapes[i].matches_scheme(true_shapes[0], error_msg))
				{
					error_msg = "One of the rough shapes doesn't match the annotation scheme.";
					return false;
				}
			}
		}

		for (size_t i = 0; i < true_shapes.size(); ++i)
		{
			if (!true_shapes[i].matches_scheme(true_shapes[0], error_msg))
			{
				error_msg = "One of the true shapes doesn't match the annotation scheme.";
				return false;
			}
		}

		for (const auto& item : n_dense_internals)
		{
			if (!map_contains_key(true_shapes[0].keypoint_curves(), item.first))
			{
				error_msg = "keypoint curve name missing from n_dense_internals";
				return false;
			}
		}

		for (const auto& item : true_shapes[0].keypoint_curves())
		{
			if (!map_contains_key(n_dense_internals, item.first))
			{
				error_msg = "unknown keypoint curve name found in n_dense_internals";
				return false;
			}
		}
		return true;
	}

	template<int DIM, int STAGES, int TMODES>
	std::vector<patch_corners> shape_refiner<DIM, STAGES, TMODES>::patch_tower(
		const rlibv::point2d<double>& pt, 
		int stage, 
		double object_scale, 
		double object_rotation)
	{
		double largest_patch_width = patch_scaling_ * default_relative_patch_width_;
		std::vector<patch_corners> tower(stage+1);
		for (int i = 0; i <= stage; ++i) 
		{
			double dx = largest_patch_width * relative_patch_scales_[i] / 2.0;
			double dy = largest_patch_width * relative_patch_scales_[i] / 2.0;
			rlibv::shape2d<double> corners = {
				rlibv::point2d<double>(-dx, -dy),
				rlibv::point2d<double>(dx, -dy),
				rlibv::point2d<double>(dx, dy),
				rlibv::point2d<double>(-dx, dy),
			};
			dlib::point_transform_projective tform = similarity_params_as_projective(std::array<double, 4>{ object_scale, object_rotation, pt.x(), pt.y() });
			corners = rlibv::transform_shape(corners, tform);
			tower[i][0] = corners[0];
			tower[i][1] = corners[1];
			tower[i][2] = corners[2];
			tower[i][3] = corners[3];
		}
		return tower;
	}

	template<int DIM, int LEVELS, int TMODES>
	std::vector<rlibv::patch_corners> rlibv::shape_refiner<DIM, LEVELS, TMODES>::reference_patch_tower(int stage)
	{
		double largest_patch_width = patch_scaling_ * default_relative_patch_width_;
		std::vector<patch_corners> tower(LEVELS);
		for (int i = 0; i < static_cast<int>(relative_patch_scales_.size()); ++i)
		{
			double dx = largest_patch_width * relative_patch_scales_[i] / 2.0;
			double dy = largest_patch_width * relative_patch_scales_[i] / 2.0;

			rlibv::shape2d<double> corners = {
				rlibv::point2d<double>(-dx, -dy),
				rlibv::point2d<double>(dx, -dy),
				rlibv::point2d<double>(dx, dy),
				rlibv::point2d<double>(-dx, dy),
			};
			tower[i][0] = corners[0];
			tower[i][1] = corners[1];
			tower[i][2] = corners[2];
			tower[i][3] = corners[3];
		}
		return tower;
	}

	template<int DIM, int STAGES, int TMODES>
	void serialize(const shape_refiner<DIM, STAGES, TMODES>& item, std::ostream& out)
	{
		serialize(item.n_dense_internals_, out);
		serialize(item.is_trained_, out);
		serialize(item.use_delta_constraint_, out);
		serialize(item.shape_model_, out);
		serialize(item.input_tracker_name_, out);
		serialize(item.input_tracker_size_check_, out);
		serialize(item.curve_lookup_, out);
		serialize(item.keypoint_lookup_, out);
		serialize(item.patch_scaling_, out);
		serialize(item.delta_model_, out);
		serialize(item.reference_shape_, out);
		serialize(item.vector_reducers_stage_0_, out);
		serialize(item.vector_reducers_stage_1_, out);
		serialize(item.vector_reducers_stage_2_, out);
		serialize(item.vector_reducers_stage_3_, out);
		serialize(item.vector_reducers_stage_4_, out);
		serialize(item.vector_reducers_stage_5_, out);
		serialize(item.vector_normalizers_, out);
		serialize(item.nn_predictors_, out);
		serialize(item.default_relative_patch_width_, out);
	}

	template<int DIM, int STAGES, int TMODES>
	void deserialize(shape_refiner<DIM, STAGES, TMODES>& item, std::istream& in)
	{
		deserialize(item.n_dense_internals_, in);
		deserialize(item.is_trained_, in);
		deserialize(item.use_delta_constraint_, in);
		deserialize(item.shape_model_, in);
		deserialize(item.input_tracker_name_, in);
		deserialize(item.input_tracker_size_check_, in);
		deserialize(item.curve_lookup_, in);
		deserialize(item.keypoint_lookup_, in);
		deserialize(item.patch_scaling_, in);
		deserialize(item.delta_model_, in);
		deserialize(item.reference_shape_, in);
		deserialize(item.vector_reducers_stage_0_, in);
		deserialize(item.vector_reducers_stage_1_, in);
		deserialize(item.vector_reducers_stage_2_, in);
		deserialize(item.vector_reducers_stage_3_, in);
		deserialize(item.vector_reducers_stage_4_, in);
		deserialize(item.vector_reducers_stage_5_, in);
		deserialize(item.vector_normalizers_, in);
		deserialize(item.nn_predictors_, in);
		deserialize(item.default_relative_patch_width_, in);
	}
}


