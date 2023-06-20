// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "mv_tracker.h"

namespace rlibv
{
	using dlib::serialize;
	using dlib::deserialize;

	constexpr int const_abs(int n)
	{
		if (n < 1)
		{
			return -n;
		}
		else
		{
			return n;
		}
	}

	template<int N, int M, int T>
	void serialize(const point_regression_model<N, M, T>& item, std::ostream& out)
	{
		serialize(item.normalizer, out);
		serialize(item.compressor, out);
		serialize(item.normalizer2, out);
		serialize(item.dn_kernel_regression, out);
		serialize(item.dt_kernel_regression, out);
		serialize(item.dn_rf_regression, out);
		serialize(item.dt_rf_regression, out);
		serialize(item.use_rf, out);
	}


	template<int N, int M, int T>
	void deserialize(point_regression_model<N, M, T>& item, std::istream& in)
	{
		deserialize(item.normalizer, in);
		deserialize(item.compressor, in);
		deserialize(item.normalizer2, in);
		deserialize(item.dn_kernel_regression, in);
		deserialize(item.dt_kernel_regression, in);
		deserialize(item.dn_rf_regression, in);
		deserialize(item.dt_rf_regression, in);
		deserialize(item.use_rf, in);
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	constexpr int mv_tracker<T,R,S,MODES,TREES,args...>::total_number_of_features()
	{
		return sampler::total_number_of_features();
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	constexpr int mv_tracker<T,R,S,MODES,TREES,args...>::number_of_raw_samples()
	{
		return sampler::number_of_raw_samples();
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const view_vector<std::array<double, 4>>& mv_tracker<T,R,S,MODES,TREES,args...>::local_approx_saxy_min() const
	{
		return local_approx_saxy_min_;
	}
	
	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const view_vector<std::array<double, 4>>& mv_tracker<T,R,S,MODES,TREES,args...>::local_approx_saxy_max() const
	{
		return local_approx_saxy_max_;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const view_vector<std::array<double, 4>>& mv_tracker<T,R,S,MODES,TREES,args...>::local_approx_saxy_mid() const
	{
		return local_approx_saxy_mid_;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const view_vector<linear_pdm<double>>& mv_tracker<T,R,S,MODES,TREES,args...>::local_pdms() const
	{
		return local_pdms_;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const linear_pdm<double>& mv_tracker<T,R,S,MODES,TREES,args...>::global_pdm() const
	{
		return global_pdm_;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	dlib::array2d<dlib::rgb_pixel> mv_tracker<T,R,S,MODES,TREES,args...>::sampling_diagnostic_image(const std::vector<int>& pt_indices, int stage, int view)
	{
		auto& first_view_sampler = samplers_[view];
		first_view_sampler.set_image(first_training_images_[view]);
		dlib::array2d<dlib::rgb_pixel> display_img;
		dlib::assign_image(display_img, first_view_sampler.tiled_pyr_image());
		auto& shape = first_training_shapes().shapes[view];
		auto this_pose = local_pdms_[view].find_pose(shape);
		auto approx_saxy = rlibv::approx_similarity_transform<double>(this_pose);
		auto sampler_rotations = compute_local_rotations(view, shape, approx_saxy[1]);
		double scaling = stage_scaling_[stage] * approx_saxy[0] / local_approx_saxy_mid_[view][0];

		for (int ind : pt_indices)
		{
			auto centre_pt = shape[ind];
			std::array<std::array<std::array<point2d<double>, R>, first_view_sampler.n_radials()>, S> sample_locations = first_view_sampler.radial_sample_points(centre_pt, sampler_rotations[ind], scaling);
			// constexpr int n_circles = first_view_sampler.n_radials();
			for (int s = 0; s < S; ++s)
			{	
				for (int c = 0; c < first_view_sampler.n_radials(); ++c)
				{
					for (int r = 0; r < first_view_sampler.n_radials(); ++r)
					{
						auto pt = sample_locations[s][r][c];
						dlib::draw_solid_circle(display_img, pt, 2, dlib::rgb_alpha_pixel(30 * c, 15 * r, 0, 255));
					}
					for (int r = first_view_sampler.n_radials(); r < R; ++r)
					{
						auto pt = sample_locations[s][r - first_view_sampler.n_radials()][R-c-1];
						dlib::draw_solid_circle(display_img, pt, 2, dlib::rgb_alpha_pixel(30 * c, 15 * r, 0, 255));
					}
				}
			}
		}
		return display_img;
	}
	
	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	void mv_tracker<T,R,S,MODES,TREES,args...>::initialize_training(
		const std::vector<multiview_shape2d<double>>& mv_shapes,
		const view_vector<std::vector<simple_curve>>& curves,
		std::array<double, 4> augmentation_factors,
		std::vector<double> stage_scaling,
		alignment_type align_method,
		double displacement_size
	)
	{
		std::string msg;
		DLIB_ASSERT(is_mv_tracker_initialization_valid(mv_shapes, curves, augmentation_factors, stage_scaling, msg));
	
		max_normal_displacement_ = displacement_size;
		max_tangent_displacement_ = displacement_size;
		local_pdms_.clear();
		samplers_.clear();
		regressors_training_data_.clear();
		classifier_training_data_.clear();
		selected_feature_inds_norm_.clear();
		selected_feature_inds_tangent_.clear();
		simple_curves_.clear();
		training_shapes_.clear();
		local_approx_saxy_min_.clear();
		local_approx_saxy_max_.clear();
		local_approx_saxy_mid_.clear();
		n_points_.clear();
		regressors_.clear();
		classifiers_.clear();
		pt_inds_.clear();
		residuals_limits_.clear();
		first_training_images_.clear();
		n_views_ = static_cast<int>(curves.size());
		local_pdms_.resize(n_views_);
		samplers_.resize(n_views_);
		regressors_training_data_.resize(n_views_);
		classifier_training_data_.resize(n_views_);
		selected_feature_inds_norm_.resize(n_views_);
		selected_feature_inds_tangent_.resize(n_views_);
		simple_curves_.resize(n_views_);
		local_approx_saxy_min_.resize(n_views_);
		local_approx_saxy_max_.resize(n_views_);
		local_approx_saxy_mid_.resize(n_views_);
		local_pdms_.resize(n_views_);
		n_points_.resize(n_views_);
		regressors_.resize(n_views_);
		classifiers_.resize(n_views_);
		pt_inds_.resize(n_views_);
		residuals_limits_.resize(n_views_);
		first_training_images_.resize(n_views_);
		halton_index_.resize(n_views_);
		samplers_.resize(n_views_);

		dlib::rand rng;
		rng.set_seed("EPICGAMES");

		int n_shapes = static_cast<int>(mv_shapes.size());
		//Train the local PDMs, collecting the locally aligned shapes on the way
		std::vector<shape2d<double>> stacked_shapes;
		for(int v=0; v<n_views_; ++v)
		{
			std::vector<shape2d<double>> shapes_for_this_view(n_shapes);
			for (int i = 0; i < n_shapes; ++i)
			{
				shapes_for_this_view[i] = mv_shapes[i].shapes[v];
			}
			//If and only if no alignment, augment the shapes
			std::vector<shape2d<double>> aug_shapes;
			if (augmentation_factors[0] < 0.0001 &&
				augmentation_factors[1] < 0.0001 &&
				augmentation_factors[2] < 0.0001 &&
				augmentation_factors[3] < 0.0001)
			{
				aug_shapes = shapes_for_this_view;
			}
			else
			{
				rlibv::linear_pdm<double> temp_pdm;
				temp_pdm.train_linear_pdm(shapes_for_this_view, 0.99999, 50, alignment_type::similarity, std::vector<int>());
				double max_dist = 0.0;
				for (const auto& p1 : temp_pdm.base_shape())
				{
					for (const auto& p2 : temp_pdm.base_shape())
					{
						max_dist = std::max(max_dist, (p1 - p2).length());
					}
				}
				for (const auto& shape : shapes_for_this_view)
				{

					auto xform = find_transform_as_projective(
						shape,
						temp_pdm.base_shape(),
						alignment_type::similarity);
					auto aligned_shape = transform_shape(shape, xform);

					auto ds = augmentation_factors[0];
					auto da = augmentation_factors[1];
					auto dx = augmentation_factors[2] * max_dist;
					auto dy = augmentation_factors[3] * max_dist;

					aug_shapes.emplace_back(shape);
					for (int d = 0; d < 5; ++d)
					{
						auto rs = 1 + rng.get_double_in_range(-ds, ds);
						auto ra = rng.get_double_in_range(-da, da);
						auto rx = rng.get_double_in_range(-dx, dx);
						auto ry = rng.get_double_in_range(-dy, dy);
						auto local_xform = similarity_params_as_projective<double>({ rs,ra,rx,ry });
						auto displaced_aligned_shape = transform_shape(aligned_shape, local_xform);
						auto aug_shape = transform_shape(displaced_aligned_shape, inv(xform));
						aug_shapes.emplace_back(aug_shape);
					}
				}
			}

			local_pdms_[v].train_linear_pdm(aug_shapes, 0.99, 50, align_method, std::vector<int>());
			local_approx_saxy_min_[v] = { 10e10,10e10,10e10,10e10 };
			local_approx_saxy_max_[v] = { -10e10,-10e10,-10e10,-10e10 };
			local_approx_saxy_mid_[v] = { -10e10,-10e10,-10e10,-10e10 };
			int s = 0;
			for (const auto& shape : aug_shapes)
			{
				auto this_pose = local_pdms_[v].find_pose(shape);
				auto approx_saxy = rlibv::approx_similarity_transform<double>(this_pose);
				for (int i = 0; i < 4; ++i)
				{
					local_approx_saxy_min_[v][i] = std::min(local_approx_saxy_min_[v][i], approx_saxy[i]);
					local_approx_saxy_max_[v][i] = std::max(local_approx_saxy_max_[v][i], approx_saxy[i]);
				}
				//auto aligned_shape = transform_shape(shape,dlib::inv(this_pose));
				auto aligned_shape(shape);
				if (0 == v)
				{
					stacked_shapes.emplace_back(aligned_shape);
				}
				else
				{
					stacked_shapes[s].insert(stacked_shapes[s].end(), aligned_shape.begin(), aligned_shape.end());
				}
				++s;
			}

			for (int i = 0; i < 4; ++i)
			{
				local_approx_saxy_mid_[v][i] = (local_approx_saxy_min_[v][i] + local_approx_saxy_max_[v][i]) / 2;
			}

			n_points_[v] = static_cast<int>(shapes_for_this_view[0].size());
			halton_index_[v] = std::vector<int>(n_points_[v], 0);
			pt_inds_[v].resize(n_points_[v]);
			std::iota(pt_inds_[v].begin(), pt_inds_[v].end(), 0);
			residuals_limits_[v] = std::vector<double>(n_points_[v], 0.0);
			simple_curves_[v] = curves.at(v);
			stage_scaling_ = stage_scaling;
			regressors_training_data_[v].resize(stage_scaling_
				.size());
			classifier_training_data_[v].resize(stage_scaling_
				.size());
			selected_feature_inds_norm_[v].resize(stage_scaling_.size());
			selected_feature_inds_tangent_[v].resize(stage_scaling_.size());
			regressors_[v].resize(stage_scaling.size());
			classifiers_[v].resize(stage_scaling.size());
			for (auto& item : regressors_[v])
			{
				item.resize(local_pdms_[v].n_points());
			}
			for (auto& item : classifiers_[v])
			{
				item.resize(local_pdms_[v].n_points());
			}

			for (auto& elem : regressors_training_data_[v])
			{
				elem.clear();
				for (int stage = 0; stage < static_cast<int>(stage_scaling_.size()); ++stage)
				{
					regressors_training_data_[v][stage].resize(n_points_[v]);
					classifier_training_data_[v][stage].resize(n_points_[v]);
				}
			}
		}
		global_pdm_.train_linear_pdm(stacked_shapes, 0.99, 50, alignment_type::similarity, std::vector<int>());
		global_residuals_limits_ = std::vector<double>(global_pdm_.n_points(), 0.0f);
	}
	
	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	void mv_tracker<T,R,S,MODES,TREES,args...>::add_training_example(const dlib::array<dlib::array2d<dlib::rgb_pixel>>& view_imgs,
		const multiview_shape2d<double>& input_mv_shape,
		int expected_number_of_images,
		int min_target_displacements,
		int /*min_samples_per_node*/)
	{
		DLIB_ASSERT(view_imgs.size() == n_views());
		DLIB_ASSERT(input_mv_shape.shapes.size() == n_views());		

		multiview_shape2d<double> mv_shape;
		mv_shape.shapes.resize(n_views_);
		mv_shape.visible.resize(n_views_);
		for (int v = 0; v < n_views_; ++v)
		{
			rlibv::shape2d<double> shape;
			dlib::matrix<double, 0, 1> parameters;
			dlib::point_transform_projective pose;
			rlibv::shape2d<double> residuals_in_aligned_frame;
			local_pdms_[v].parameterize(input_mv_shape.shapes[v], parameters, pose, residuals_in_aligned_frame, 1.0, local_pdms_[v].n_params());
			local_pdms_[v].reconstruct(parameters, pose, residuals_in_aligned_frame, std::vector<double>(static_cast<int>(input_mv_shape.shapes[v].size()), 0.0), shape);
			mv_shape.shapes[v] = shape;
			mv_shape.visible[v] = std::vector<bool>(static_cast<int>(shape.size()), true);
		}

		training_shapes_.emplace_back(mv_shape);
		auto min_target_disps_per_image = std::max(2, min_target_displacements / expected_number_of_images);

		std::vector<double> approx_scales(n_views_);
		std::vector<std::vector<double>> sampler_rotations(n_views_);
		for (int v = 0; v < n_views_; ++v)
		{
			//If no images are passed in, use the last samplers.
			if (view_imgs.size() > 0)
			{
				if (first_training_images_[v].nc() == 0)
				{
					assign_image(first_training_images_[v], view_imgs[v]);
				}
				samplers_[v].set_image(view_imgs[v]);
			}
			const auto& shape = mv_shape.shapes[v];
			auto this_pose = local_pdms_[v].find_pose(shape);
			auto approx_saxy = rlibv::approx_similarity_transform<double>(this_pose);
			approx_scales[v] = approx_saxy[0];
			sampler_rotations[v] = compute_local_rotations(v, shape, approx_saxy[1]);
		}

		std::mutex mtx;


		for (int this_v = 0; this_v < n_views_; ++this_v)
		{
			for (int this_s = 0; this_s < static_cast<int>(stage_scaling_.size()); ++this_s)
			{

				dlib::parallel_for(std::thread::hardware_concurrency(), 0, mv_shape.shapes[this_v].size(), [&](long this_p)
					{
						double scaling = std::max(1.0, stage_scaling_[this_s] * approx_scales[this_v] / local_approx_saxy_mid_[this_v][0]);

						for (int id = 0; id < min_target_disps_per_image; ++id)
						{
							auto dn = 2.0 * (0.5 - halton<double>(halton_index_[this_v][this_p], 2)) * max_normal_displacement_;
							auto dt = 2.0 * (0.5 - halton<double>(halton_index_[this_v][this_p], 3)) * max_tangent_displacement_;
							halton_index_[this_v][this_p]++;
							auto centre_pt = mv_shape.shapes[this_v][this_p];
							auto sampler_rotation = sampler_rotations[this_v][this_p];
							auto dx = dn * sin(sampler_rotation) + dt * cos(sampler_rotation);
							auto dy = dn * cos(sampler_rotation) - dt * sin(sampler_rotation);
							centre_pt += R * scaling * dlib::dpoint(dx, dy);
							auto samples = samplers_[this_v].get_radial_samples(centre_pt, sampler_rotation, scaling, true);

							if (mv_shape.visible[this_v][this_p])
							{
								regressors_training_data_[this_v][this_s][this_p].dn_data.emplace_back(dn);
								regressors_training_data_[this_v][this_s][this_p].dt_data.emplace_back(dt);
								
								regressors_training_data_[this_v][this_s][this_p].feature_samples.emplace_back(samples);

								classifier_training_data_[this_v][this_s][this_p].confidence.emplace_back(1.0);
								classifier_training_data_[this_v][this_s][this_p].feature_samples.emplace_back(samples);
							}
							else
							{
								classifier_training_data_[this_v][this_s][this_p].confidence.emplace_back(0.0);
								classifier_training_data_[this_v][this_s][this_p].feature_samples.emplace_back(samples);
							}

						}
					});
			}
		}
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	void mv_tracker<T,R,S,MODES,TREES,args...>::finalize_training(int min_samples_per_node,
		const view_vector<std::vector<std::vector<point_training_data>>>* generic_training)
	{
		for (int this_v = 0; this_v < n_views_; ++this_v)
		{
			for (int this_s = 0; this_s < static_cast<int>(stage_scaling_.size()); ++this_s)
			{
				dlib::parallel_for(std::thread::hardware_concurrency(), 0, static_cast<long>(regressors_training_data_[this_v][this_s].size()), [&](long this_p)
				//for(int this_p=0; this_p< regressors_training_data_[this_v][this_s].size(); ++this_p)
					{
						auto this_training_data(regressors_training_data_[this_v][this_s][this_p]);
						if (static_cast<int>(this_training_data.dn_data.size()) > 0)
						{
							if (generic_training != nullptr)
							{
								this_training_data.dn_data.insert(this_training_data.dn_data.end(), (*generic_training)[this_v][this_s][this_p].dn_data.begin(), (*generic_training)[this_v][this_s][this_p].dn_data.end());
								this_training_data.dt_data.insert(this_training_data.dt_data.end(), (*generic_training)[this_v][this_s][this_p].dt_data.begin(), (*generic_training)[this_v][this_s][this_p].dt_data.end());
								this_training_data.feature_samples.insert(this_training_data.feature_samples.end(), (*generic_training)[this_v][this_s][this_p].feature_samples.begin(), (*generic_training)[this_v][this_s][this_p].feature_samples.end());
							}
							int n_examples = static_cast<int>(this_training_data.feature_samples.size());
							std::vector<dlib::matrix<double, total_number_of_features(), 1>> feature_data(n_examples);
							for (int e = 0; e < n_examples; ++e)
							{
								feature_data[e] = samplers_[this_v].features_from_samples(this_training_data.feature_samples[e]);
							}
							if constexpr (MODES > 1)
							{
								point_regression_model<total_number_of_features(), MODES, 0> this_regressor;
								//Normalize the raw data
								std::vector<dlib::matrix<double, total_number_of_features(), 1>> normalized_features;
								this_regressor.normalizer.train(feature_data);

								normalized_features.resize(n_examples);
								for (int e = 0; e < n_examples; ++e)
								{
									normalized_features[e] = this_regressor.normalizer(feature_data[e]);
								}

								//Compress the data
								std::vector<dlib::matrix<double, MODES, 1>> compressed_data(n_examples);
								std::vector<dlib::matrix<double, MODES, 1>> final_data(n_examples);

								//Make a smaller subset for training compressor
								std::vector<int> series(static_cast<int>(normalized_features.size()));
								std::iota(series.begin(), series.end(), 0);
								std::shuffle(series.begin(), series.end(), std::default_random_engine(0));
								int subset_size = std::min(150, n_examples);
								series.resize(subset_size);

								std::vector<dlib::matrix<double, total_number_of_features(), 1>> subset(subset_size);
								for (int sub = 0; sub < subset_size; ++sub)
								{
									subset[sub] = normalized_features[series[sub]];
								}
								this_regressor.compressor.train(subset);
								for (int e = 0; e < n_examples; ++e)
								{
									compressed_data[e] = this_regressor.compressor.parameterize(normalized_features[e]);
								}
								this_regressor.normalizer2.train(compressed_data);
								for (int e = 0; e < n_examples; ++e)
								{
									final_data[e] = this_regressor.normalizer2(compressed_data[e]);
								}

								//Regress the parameters
								this_regressor.use_rf = false;

								dlib::rr_trainer<dlib::linear_kernel<dlib::matrix<double, MODES, 1>>> norm_trainer;
								this_regressor.dn_kernel_regression = norm_trainer.train(final_data, this_training_data.dn_data);

								dlib::rr_trainer<dlib::linear_kernel<dlib::matrix<double, MODES, 1>>> tangent_trainer;
								this_regressor.dt_kernel_regression = tangent_trainer.train(final_data, this_training_data.dt_data);
								regressors_[this_v][this_s][this_p] = this_regressor;

							}
							else //RF regressors
							{
								point_regression_model<total_number_of_features(), 0, TREES> this_regressor;

								std::vector<dlib::matrix<double, 0, 1>> raw_features;
								raw_features.resize(n_examples);
								for (int e = 0; e < n_examples; ++e)
								{
									raw_features[e] = dlib::matrix<double, 0, 1>(feature_data[e]);
								}

								this_regressor.use_rf = true;
								dlib::random_forest_regression_trainer<dlib::dense_feature_extractor> norm_rf_trainer;
								norm_rf_trainer.set_num_trees(TREES);
								norm_rf_trainer.set_min_samples_per_leaf(min_samples_per_node);
								this_regressor.dn_rf_regression = norm_rf_trainer.train(raw_features, this_training_data.dn_data);

								dlib::random_forest_regression_trainer<dlib::dense_feature_extractor> tangent_rf_trainer;
								tangent_rf_trainer.set_num_trees(TREES);
								tangent_rf_trainer.set_min_samples_per_leaf(min_samples_per_node);

								this_regressor.dt_rf_regression = tangent_rf_trainer.train(raw_features, this_training_data.dt_data);

								regressors_[this_v][this_s][this_p] = this_regressor;
							}

							//Train the classifier

							auto classifer_feature_data(classifier_training_data_[this_v][this_s][this_p].feature_samples);
							std::vector<dlib::matrix<double, 0, 1>> classifier_features;
							int n_classifier_examples = static_cast<int>(classifer_feature_data.size());
							classifier_features.resize(n_classifier_examples);
							for (int e = 0; e < n_classifier_examples; ++e)
							{
								classifier_features[e].set_size(static_cast<int>(classifer_feature_data[e].size()));
								for (int f = 0; f < static_cast<int>(classifer_feature_data[e].size()); ++f)
								{
									classifier_features[e](f) = static_cast<double>(classifer_feature_data[e][f]);
								}
							}

							const auto& conf_labels = classifier_training_data_[this_v][this_s][this_p].confidence;
							auto min_label = *std::min_element(conf_labels.begin(), conf_labels.end());
							auto max_label = *std::max_element(conf_labels.begin(), conf_labels.end());
							if ((max_label - min_label) > 0.5)
							{
								dlib::random_forest_regression_trainer<dlib::dense_feature_extractor> rf_trainer;
								rf_trainer.set_num_trees(TREES);
								rf_trainer.set_min_samples_per_leaf(min_samples_per_node);
								classifiers_[this_v][this_s][this_p] = rf_trainer.train(classifier_features, conf_labels);
							}
						}
					});
			}
		}
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	constexpr int mv_tracker<T,R,S,MODES,TREES,args...>::n_texture_modes() 
	{
		return MODES;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const int mv_tracker<T,R,S,MODES,TREES,args...>::n_stages() const
	{
		return static_cast<int>(regressors_.begin()->size());
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const int mv_tracker<T,R,S,MODES,TREES,args...>::n_shape_modes() const
	{
		return global_pdm_.n_params();
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const std::vector<int>& mv_tracker<T,R,S,MODES,TREES,args...>::n_points() const
	{
		return n_points_;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	int mv_tracker<T,R,S,MODES,TREES,args...>::n_views() const
	{
		return n_views_;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	void mv_tracker<T,R,S,MODES,TREES,args...>::set_images(const dlib::array<dlib::array2d<dlib::rgb_pixel>>& views)
	{
		if (samplers_.size() != views.size())
		{
			samplers_.resize(views.size());
		}
		for (int v=0; v<n_views_; ++v)
		{
			samplers_[v].set_image(views[v]);
		}
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const multiview_shape2d<double>& mv_tracker<T, R, S, MODES, TREES, args...>::first_training_shapes() const
	{
		return training_shapes_[0];
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const std::vector<double>& mv_tracker<T, R, S, MODES, TREES, args...>::stage_scaling() const
	{
		return stage_scaling_;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const std::vector<multiview_shape2d<double>>& mv_tracker<T, R, S, MODES, TREES, args...>::training_shapes() const
	{
		return training_shapes_;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	view_vector<std::vector<double>> mv_tracker<T,R,S,MODES,TREES,args...>::update_shapes(
		int stage,
		int max_modes, 
		bool /*bilinear*/, 
		multiview_shape2d<double>& mv_shape) const
	{
		
		DLIB_ASSERT(mv_shape.shapes.size() == n_views());
		view_vector<dlib::point_transform_projective> local_poses(n_views_);
		multiview_shape2d<double> local_raw_mv_shape;
		local_raw_mv_shape.shapes.resize(n_views_);

		view_vector<std::vector<double>> fitness(n_views_);
		view_vector<std::vector<double>> mv_weights(n_views_);

		double scaling = 0;
		for (int v = 0; v < n_views_; ++v)
		{
			shape2d<double>& this_shape = mv_shape.shapes[v];
			mv_weights[v].resize(static_cast<int>(this_shape.size()));
			fitness[v].resize(static_cast<int>(this_shape.size()));
			auto this_pose = local_pdms_[v].find_pose(this_shape);
			auto approx_saxy = rlibv::approx_similarity_transform<double>(this_pose);
			auto sampler_rotations = compute_local_rotations(v,this_shape, approx_saxy[1]);
			scaling = std::max(1.0, static_cast<double>(stage_scaling_[stage] * approx_saxy[0] / local_approx_saxy_mid_[v][0]));
			const auto& these_regressors = regressors_[v][stage];
			const auto& these_classifiers = classifiers_[v][stage];
			const auto& these_samplers = samplers_[v];
			auto& these_fitness_measures = fitness[v];
			auto& these_mv_weights = mv_weights[v];
			bool use_confidence_shape_fit = false;
			dlib::parallel_for(std::thread::hardware_concurrency(), 0, pt_inds_[v].size(), [&](long i)
				{
					auto centre_pt = this_shape[pt_inds_[v][i]];
					auto sampler_rotation = sampler_rotations[pt_inds_[v][i]];
					auto sin_sr = sin(sampler_rotation);
					auto cos_sr = cos(sampler_rotation);

					auto samples = these_samplers.get_radial_samples(centre_pt, sampler_rotation, scaling, true);
					dlib::matrix<double, total_number_of_features(), 1> features = these_samplers.features_from_samples(samples);

					double dn = 0;
					double dt = 0;
					const auto& this_regressor = these_regressors[pt_inds_[v][i]];
					bool is_trained = true;
					if (this_regressor.use_rf)
					{
						if (this_regressor.dn_rf_regression.get_num_trees() == 0)
						{
							is_trained = false;
						}
					}
					else
					{
						if (!this_regressor.compressor.is_trained())
						{
							is_trained = false;
						}
					}
					if (is_trained)
					{
						if (!this_regressor.use_rf)
						{
							dlib::matrix<double, total_number_of_features(), 1> norm_features = this_regressor.normalizer(features);

							dlib::matrix<double, MODES, 1> compressed_data = this_regressor.compressor.parameterize(norm_features);
							dlib::matrix<double, MODES, 1> final_data = this_regressor.normalizer2(compressed_data);

							dn = -this_regressor.dn_kernel_regression(final_data);
							dt = -this_regressor.dt_kernel_regression(final_data);
						}
						else
						{
							dlib::matrix<double, 0, 1> raw_features(features);
							dn = -this_regressor.dn_rf_regression(raw_features);
							dt = -this_regressor.dt_rf_regression(raw_features);
						}
						if (these_classifiers[pt_inds_[v][i]].get_num_trees() > 0)
						{
							dlib::matrix<double, 0, 1> raw_features(features);
							auto conf = std::max(0.0, std::min(1.0, these_classifiers[pt_inds_[v][i]](raw_features)));
							//TODO This is a hack to control robustness
							auto gap = 1.0 - conf;
							these_fitness_measures[pt_inds_[v][i]] = 2.0 / (1 + std::exp(10 * gap));
							use_confidence_shape_fit = true;
						}
						else
						{
							these_fitness_measures[pt_inds_[v][i]] = 1.0;
						}
					}
					else
					{
						these_fitness_measures[pt_inds_[v][i]] = 0.0;
					}

					auto dx = dn * sin_sr + dt * cos_sr;
					auto dy = dn * cos_sr - dt * sin_sr;
					this_shape[pt_inds_[v][i]].x() += dx * R * scaling;
					this_shape[pt_inds_[v][i]].y() += dy * R * scaling;
					these_mv_weights[pt_inds_[v][i]] = these_fitness_measures[pt_inds_[v][i]];
					
				});

			local_raw_mv_shape.shapes[v] = this_shape;
			dlib::matrix<double, 0, 1> parameters;
			std::vector<dlib::dpoint> residuals_in_aligned_frame;
			

			if(use_confidence_shape_fit)
			{
				auto max_modes_to_use = std::min(max_modes, local_pdms_[v].n_params());
				local_pdms_[v].parameterize_with_confidence(local_raw_mv_shape.shapes[v], mv_weights[v], parameters, local_poses[v], residuals_in_aligned_frame, 1.1, max_modes_to_use, 5);
			}
			else
			{
				auto max_modes_to_use = std::min(max_modes, local_pdms_[v].n_params());

				local_pdms_[v].parameterize(local_raw_mv_shape.shapes[v], parameters, local_poses[v], residuals_in_aligned_frame, 1.0, max_modes_to_use);
			}
			
			//Stop poses going to tiny or huge values which can cause NaN crashes
			auto local_approx_saxy = rlibv::approx_similarity_transform<double>(local_poses[v]);
			if (local_approx_saxy[0] < 0.1 * local_approx_saxy_min_[v][0] || local_approx_saxy[0] > 20 * local_approx_saxy_min_[v][0])
			{
				local_poses[v] = similarity_params_as_projective(local_approx_saxy_mid_[v]);
			}
			local_pdms_[v].reconstruct(parameters, local_poses[v], residuals_in_aligned_frame, residuals_limits_[v], this_shape);
		}

		dlib::matrix<double, 0, 1> parameters;
		dlib::point_transform_projective pose;
		std::vector<dlib::dpoint> residuals_in_aligned_frame;
		auto max_modes_to_use = std::min(max_modes, global_pdm_.n_params());
		mv_shape = weighted_shape_fit(mv_shape, mv_weights, max_modes_to_use, 5);

		return fitness;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	view_vector<std::vector<double>> mv_tracker<T,R,S,MODES,TREES,args...>::autosearch(
		const std::vector<int>& iterations_per_stage,
		int max_modes_per_stage,
		bool bilinear,
		multiview_shape2d<double>& mv_shape
	) 
	{
		
		std::vector<int> modes_per_stage(n_stages(), max_modes_per_stage);
		return search(iterations_per_stage, modes_per_stage, bilinear, mv_shape);
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	view_vector<std::vector<double>> mv_tracker<T,R,S,MODES,TREES,args...>::search(
		const std::vector<int>& iterations_per_stage,
		const std::vector<int>& max_modes_per_stage,
		bool bilinear,
		multiview_shape2d<double>& mv_shape
	) 
	{
		double cur_iteration = 1.0;
		double max_iterations = 0.0;
		for (const auto& its : iterations_per_stage)
		{
			max_iterations += its;
		}
		view_vector<std::vector<double>> fitness(n_views_);
		for (int s = 0; s < static_cast<int>(stage_scaling_.size()); ++s)
		{
			for (int its = 0; its < iterations_per_stage[s]; ++its)
			{
				auto d_modes = (cur_iteration/ max_iterations)*max_modes_per_stage[s];
				auto modes = std::max(2, static_cast<int>(std::round(d_modes)));
				fitness = update_shapes(s, modes, bilinear, mv_shape);
				cur_iteration += 1.0;
			}
		}
		return fitness;
	}

#if 0 // cur_iterations_ undefined. Function seems unused. Remove it? and maybe also mv_tracker.h diagnostic_search?
	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	std::vector<std::vector<double>> mv_tracker<T,R,S,MODES,TREES,args...>::diagnostic_search(
		view_vector<std::vector<dlib::dpoint>>& shapes,
		const std::vector<int>& iterations_per_stage,
		const std::vector<int>& max_modes_per_stage,
		bool bilinear,
		std::vector<view_vector<std::vector<dlib::dpoint>>>& local_predictions
	)
	{
		cur_iterations_ = 0;
		max_iterations_ = 0;
		for (const auto& its : iterations_per_stage)
		{
			max_iterations_ += its;
		}
		std::vector<std::vector<double>> fitness(stage_scaling_.size());
		for (int s = 0; s < static_cast<int>(stage_scaling_.size()); ++s)
		{
			fitness[s] = std::vector<double>(iterations_per_stage[s], 99);
			for (int its = 0; its < iterations_per_stage[s]; ++its)
			{
				auto d_modes = max_modes_per_stage[s] / (1.0 + max_iterations_ - cur_iterations_);
				auto modes = std::max(2, static_cast<int>(std::round(d_modes)));
				fitness[s][its] = update_shapes(s, shapes, modes, bilinear);
				cur_iterations_++;
			}
		}
		return fitness;
	}
#endif

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	multiview_shape2d<double> mv_tracker<T,R,S,MODES,TREES,args...>::weighted_shape_fit(const multiview_shape2d<double>& mv_shape, const std::vector<std::vector<double>>& mv_weights, int max_modes, int iterations) const
	{
		multiview_shape2d<double> result(mv_shape);
		shape2d<double> orig_stacked_shape;
		std::vector<double> stacked_weights;
		for (int v = 0; v < n_views_; ++v)
		{
			if (0 == v)
			{
				orig_stacked_shape = result.shapes[v];
				stacked_weights = mv_weights[v];
			}
			else
			{
				orig_stacked_shape.insert(orig_stacked_shape.end(), result.shapes[v].begin(), result.shapes[v].end());
				stacked_weights.insert(stacked_weights.begin(), mv_weights[v].begin(), mv_weights[v].end());
			}
		}
		shape2d<double> stacked_shape(orig_stacked_shape);

		for (int it = 0; it < iterations; ++it)
		{
			for (int p = 0; p < static_cast<int>(stacked_shape.size()); ++p)
			{
				stacked_shape[p] = (1.0 - stacked_weights[p]) * stacked_shape[p] + stacked_weights[p] * orig_stacked_shape[p];
			}
			dlib::matrix<double, 0, 1> parameters;
			dlib::point_transform_projective pose;
			std::vector<dlib::dpoint> residuals_in_aligned_frame;
			auto max_modes_to_use = std::min(max_modes, global_pdm_.n_params());
			global_pdm_.parameterize(stacked_shape, parameters, pose, residuals_in_aligned_frame, 1.0, max_modes_to_use);
			global_pdm_.reconstruct(parameters, pose, residuals_in_aligned_frame, global_residuals_limits_, stacked_shape);
		}

		//Decompose the stacked shape
		int p_count = 0;
		for (int v = 0; v < n_views_; ++v)
		{
			for (int p = 0; p < static_cast<int>(mv_shape.shapes[v].size()); ++p)
			{
				result.shapes[v][p] = stacked_shape[p_count++];
			}
		}
		return result;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	void mv_tracker<T,R,S,MODES,TREES,args...>::save(std::ostream& out) const
	{
		if (std::is_same<T, dlib::rgb_pixel>::value)
		{
			serialize(std::string("dlib::rgb_pixel"), out);
		}
		else if (std::is_same<T, dlib::rgb_alpha_pixel>::value)
		{
			serialize(std::string("dlib::rgb_alpha_pixel"), out);
		}
		else
		{
			serialize(std::string("unsigned char"), out);
		}
		serialize(R, out);
		serialize(false, out); //Legacy serialization no longer used
		serialize(true, out); //Legacy serialization no longer used
		serialize(S, out);
		auto values = { args... };
		auto n_features = static_cast<int>(values.size());
		serialize(n_features, out);
		for (auto& feature : values)
		{
			std::string feature_as_string = to_string(feature);
			serialize(feature_as_string, out);
		}


		serialize(n_views_, out);
		serialize(stage_scaling_, out);
		serialize(simple_curves_, out);
		serialize(local_pdms_, out);
		serialize(global_pdm_, out);
		serialize(n_points_, out);
		serialize(regressors_, out);
		serialize(pt_inds_, out);
		serialize(residuals_limits_, out);
		serialize(global_residuals_limits_, out);
		serialize(first_training_images_, out);
		serialize(training_shapes_, out);
		serialize(local_approx_saxy_min_, out);
		serialize(local_approx_saxy_mid_, out);
		serialize(local_approx_saxy_max_, out);
		serialize(regressors_training_data_, out);
		serialize(classifier_training_data_, out);
		serialize(classifiers_, out);
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	void mv_tracker<T,R,S,MODES,TREES,args...>::load(std::istream& in)
	{
		std::string pixel_t;
		deserialize(pixel_t, in);
		int Rt, St;
		deserialize(Rt, in);
		bool At, Ct;
		deserialize(At, in); //Legacy serialization no longer used
		deserialize(Ct, in); //Legacy serialization no longer used
		deserialize(St, in);
		int n_features;
		deserialize(n_features, in);
		std::vector<feature_type> features(n_features);
		for (int f = 0; f < n_features; ++f)
		{
			std::string f_str;
			deserialize(f_str, in);
			features[f] = to_enum<feature_type>(f_str);

		}
		deserialize(n_views_, in);
		deserialize(stage_scaling_, in);
		deserialize(simple_curves_, in);
		deserialize(local_pdms_, in);
		deserialize(global_pdm_, in);
		deserialize(n_points_, in);
		deserialize(regressors_, in);
		deserialize(pt_inds_, in);
		deserialize(residuals_limits_, in);
		deserialize(global_residuals_limits_, in);
		deserialize(first_training_images_, in);
		deserialize(training_shapes_, in);
		deserialize(local_approx_saxy_min_, in);
		deserialize(local_approx_saxy_mid_, in);
		deserialize(local_approx_saxy_max_, in);
		auto pos = in.tellg();
		try
		{
			deserialize(regressors_training_data_, in);
		}
		catch (...)
		{
			regressors_training_data_.clear();
			in.seekg(pos);
		}
		pos = in.tellg();
		try
		{
			deserialize(classifier_training_data_, in);
		}
		catch (...)
		{
			in.seekg(pos);
		}
		pos = in.tellg();
		try
		{
			deserialize(classifiers_, in);
		}
		catch (...)
		{
			in.seekg(pos);
		}

		std::stringstream s;
		s << "tracking_model<" << pixel_t << "," << Rt << "," << At << "," << Ct << "," << St;
		for (int f = 0; f < n_features; ++f)
		{
			s << ",feature_type::" << to_string(features[f]);
		}
		s << ">";

		if (std::is_same<T, dlib::rgb_pixel>::value)
		{
			if (pixel_t != "dlib::rgb_pixel")
			{
				throw(dlib::serialization_error("Model you're loading has this structure: " + s.str()));
			}
		}
		else if (std::is_same<T, dlib::rgb_alpha_pixel>::value)
		{
			if (pixel_t != "dlib::rgb_alpha_pixel")
			{
				throw(dlib::serialization_error("Model you're loading has this structure: " + s.str()));
			}
		}
		else
		{
			if (pixel_t != "unsigned char")
			{
				throw(dlib::serialization_error("Saved model has this structure: " + s.str()));
			}
		}

		if (R != Rt || S != St)
		{
			throw(dlib::serialization_error("Saved model has this structure: " + s.str()));
		}
		auto values = { args... };
		int n_features_expected = static_cast<int>(values.size());
		if (n_features != n_features_expected)
		{
			throw(dlib::serialization_error("Saved model has this structure: " + s.str()));
		}

		int f = 0;
		for (auto it = values.begin(); it != values.end(); ++it)
		{
			if (features[f++] != *it)
			{
				throw(dlib::serialization_error("Saved model has this structure: " + s.str()));
			}
		}
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const double mv_tracker<T,R,S,MODES,TREES,args...>::raw_sample_radius() const
	{
		return R / 2.0;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	std::vector<double> mv_tracker<T,R,S,MODES,TREES,args...>::feature_radii(shape2d<double>& shape, int view) const
	{
		auto this_pose = local_pdms_[view].find_pose(shape);
		auto approx_saxy = rlibv::approx_similarity_transform<double>(this_pose);
		//auto sampler_rotations = compute_local_rotations(view, shape, approx_saxy[1]);
		
		double scaling = approx_saxy[0] / local_approx_saxy_mid_[view][0];
		std::vector<double> these_radii;
		for (int s = 0; s < S; ++s)
		{
			double this_scale = 1.0 / (std::pow(2, s) * scaling);
			these_radii.emplace_back(raw_sample_radius() / this_scale);
		}
		return these_radii;

	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	std::vector<double> mv_tracker<T,R,S,MODES,TREES,args...>::feature_angles(shape2d<double>& shape, int view) const
	{
		auto this_pose = local_pdms_[view].find_pose(shape);
		auto approx_saxy = rlibv::approx_similarity_transform<double>(this_pose);
		return compute_local_rotations(view, shape, approx_saxy[1]);
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const view_vector<std::vector<std::vector<point_training_data>>>* mv_tracker<T,R,S,MODES,TREES,args...>::regressors_training_data()
	{
		return &regressors_training_data_;
	}

	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	const view_vector<std::vector<std::vector<confidence_training_data>>>* mv_tracker<T,R,S,MODES,TREES,args...>::classifier_training_data()
	{
		return &classifier_training_data_;
	}
	
	template<typename T, int R, int S, int MODES, int TREES, feature_type... args>
	std::vector<double> mv_tracker<T,R,S,MODES,TREES,args...>::compute_local_rotations(int view, const std::vector<dlib::dpoint>& shape, double global_angle) const
	{
		//First set all points to the global angle - any points not on a curve will just keep this value
		std::vector<double> angles(shape.size(), global_angle);
		for (const auto& curve : simple_curves_[view])
		{
			std::vector<double> curve_angles = normal_angles(subvector(shape, curve.indices), curve.closed);
			int a = 0;
			for (auto ind: curve.indices)
			{
				angles[ind] = curve_angles[a++];
			}
		}
		return angles;
	}

	
}
