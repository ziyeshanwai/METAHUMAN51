// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/io/JsonIO.h>
#include "modular_solver_trainer.h"
#include <vector>
#include <map>
#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/dir_nav/dir_nav_extensions.h>
POSEBASEDSOLVER_RENABLE_WARNINGS


namespace cm
{
	class goreq_parser
	{
	public:
		struct tracking_data
		{
			int idOffset;
			std::string tracking_file_path;
		};

		template <typename T, int N>
		struct shot_data
		{
			typename cm::modular_solver<T, N>::runtime_params rp;
			std::string result_path;
			std::vector<tracking_data> views_tracking;
		};
	private:
		struct req_parse_error : public dlib::error
		{
			req_parse_error(const std::string& message) : error(message) {}
		};
		static const std::string CONFIG_STRING;
		static const std::string SOLVER_NAME_STRING;
		static const std::string PART_NAME_STRING;
		static const std::string POSES_STRING;
		static const std::string USER_PROTOCOL_STRING;
		static const std::string SHAPE_COMPONENTS_STRING;
		static const std::string CONTROLS_STRING;
		static const std::string SHOTS_STRING;
		static const std::string FEATURE_FRACTIONS_STRING;
		static const std::string IMPUTATION_ITERATIONS_STRING;
		static const std::string CROSS_VALIDATION_METRIC_STRING;
		static const std::string REGRESSION_TYPE_STRING;
		static const std::string FEATURE_FRACTION_STRING;
		static const std::string ITERATIVE_STRING;
		static const std::string MODE_FACTORS_STRING;
		static const std::string PCA_VARIANCES_STRING;
		static const std::string SHAPES_PCA_VARIANCE_STRING;
		static const std::string GLOBAL_ERROR_STRING;
		static const std::string ORDER_STRING;
		static const std::string LAMBDAS_STRING;
		static const std::string KERNEL_TYPE_STRING;
		static const std::string ALIGN_METHOD_STRING;
		static const std::string POINT_ID_STRING_STRING;
		static const std::string CONTROL_NAME_STRING;
		static const std::string RANGE_INFO_STRING;
		static const std::string CONTROL_VALUES_STRING;
		static const std::string SHAPE_STRING;
		static const std::string SHAPES_STRING;
		static const std::string X_STRING;
		static const std::string Y_STRING;
		static const std::string FILENAME_STRING;
		static const std::string CONTAINS_SHAPES_STRING;
		static const std::string ID_OFFSET_STRING;
		static const std::string OUTPUT_CONSTRAINT_STRING;
		static const std::string KFR_STRING;
		static const std::string CLEANING_STRING;
		static const std::string CUTOFF_STRING;
		static const std::string SMOOTHING_LEVEL_STRING;
		static const std::string CUT_STARTS_STRING;
		static const std::string REFERENCE_FRAMES_STRING;
		static const std::string FRAMES_STRING;
		static const std::string TRACKING_STRING;
		static const std::string RESULT_FILENAME_STRING;
		static const std::string NEUTRAL_FRAME_STRING;

		static const std::string RIDGE_REGRESSION_STRING;
		static const std::string ABSOLUTE_AVERAGE_STRING;
		static const std::string ABSOLUTE_MAXIMUM_STRING;
		static const std::string LINEAR_STRING;
		static const std::string NONE_STRING;
		static const std::string RIGID2D_STRING;
		static const std::string SIMILARITY2D_STRING;
		static const std::string TRANSLATE_STRING;
		static const std::string TRANSLATION_STRING;
		static const std::string RIGID_STRING;
		static const std::string SIMILARITY_STRING;
		static const std::string NUM_THREADS_STRING;
		// common private functions
	private:
		template <typename T>
		static void parse_mode_factors(const std::vector<epic::carbon::JsonElement>& mode_factors_json, std::vector<int>& mode_factors, const unsigned num_examples)
		{
			try
			{
				mode_factors.resize(mode_factors_json.size());
				int i = 0;
				for (auto factor = mode_factors_json.begin(); factor != mode_factors_json.end(); ++factor, ++i)
				{
					mode_factors[i] = static_cast<int>(std::ceil((*factor).Get<T>() * std::sqrt(num_examples)));
				}
			}
			catch (const std::exception& err)
			{
				throw req_parse_error("parse_mode_factors: Error parsing mode factors: " + std::string(err.what()));
			}
		}

		template <typename T>
		static void parse_json_array(const std::vector<epic::carbon::JsonElement>& json, std::vector<T>& values)
		{
			try
			{
				values.resize(json.size());
				int i = 0;
				for (auto item = json.begin(); item != json.end(); ++item, ++i)
				{
					values[i] = (*item).Get<T>();
				}
			}
			catch (const std::exception& err)
			{
				throw req_parse_error("parse_json_array: Error parsing json array: " + std::string(err.what()));
			}
		}

		static std::vector<std::string> parse_point_id_string(const epic::carbon::JsonElement& point_id_string)
		{
			std::vector<std::string> point_ids;
			auto ranges = split_by_string(point_id_string.Get<std::string>(), ",");
			for (const auto& range : ranges)
			{
				auto parts = split_by_string(range, ":");
				int start, end, step;
				switch (parts.size())
				{
				case 1:
				{
					start = std::stoi(parts[0]);
					end = start;
					step = 1;
					break;
				}
				case 2:
				{
					start = std::stoi(parts[0]);
					end = std::stoi(parts[1]);
					step = 1;
					break;
				}
				case 3:
				{
					start = std::stoi(parts[0]);
					step = std::stoi(parts[1]);
					end = std::stoi(parts[2]);
					break;
				}
				default:
					throw req_parse_error("Unable to handle point ID parsing");
				}
				for (int id = start; id <= end; id += step)
				{
					point_ids.emplace_back(std::to_string(id));
				}
			}
			return point_ids;
		}

		template <typename T, int N>
		static void parse_controls(const std::vector<epic::carbon::JsonElement>& controls, std::map<std::string, typename cm::modular_solver<T, N>::control_range>& control_ranges)
		{
			try
			{
				for (auto control = controls.begin(); control != controls.end(); ++control)
				{
					auto control_name = (*control)[CONTROL_NAME_STRING].Get<std::string>();
					const auto& range_info = (*control)[RANGE_INFO_STRING].Array();
					typename cm::modular_solver<T, N>::control_range& cr = control_ranges[control_name];
					int i = 0;
					for (auto range_it = range_info.begin(); range_it != range_info.end(); ++range_it, ++i)
					{
						if (i == 0)
						{
							cr.min = (*range_it).Get<T>();
						}
						else if (i == 1)
						{
							cr.max = (*range_it).Get<T>();
						}
						else if (i == 2)
						{
							cr.def = (*range_it).Get<T>();
						}
						else
						{
							throw req_parse_error("parse_controls: Encountered unexpected element of control range array");
						}
					}
				}
			}
			catch (const std::exception& err)
			{
				throw req_parse_error("parse_controls: Error parsing controls json: " + std::string(err.what()));
			}
		}
		// .req public functions
	public:
		/*
		Parses a .req file and populates runtime params for a modular solver

		- Requires
			- shots_file_string contains a string containing the contents of a valid .req file (just the shots part)

		- Ensures
			- shots is populated with appropriate post_processing_params, output path, and qp3 paths
			- returns true if .req is successfully parsed, false otherwise
		*/
		template <typename T>
		static bool parse_into_shots(const std::string& shots_file_string,
			std::vector<shot_data<T, 2>>& shots,
			const std::vector<std::vector<point2_map<T>>>& frames_shapes_tracking)
		{
			try
			{
				epic::carbon::JsonElement shots_json = epic::carbon::ReadJson(shots_file_string);

				parse_shots(shots_json[SHOTS_STRING].Array(), shots, frames_shapes_tracking);
			}
			catch (const req_parse_error& e)
			{
				std::cerr << e.what() << std::endl;
				return false;
			}

			return true;
		}

		/*
		Parses a Go solver .req file and populates tracking training data, control training data, control ranges, training params, and runtime params for a modular solver

		- Requires
			- req_string contains a string containing the contents of a .req file

		- Ensures
			- frames_shapes_tracking is returned with the same number of frames as frames_controls
			- frames_controls contains only controls present in control_ranges
			- tp is populated with appropriate compression_params, regression_params, kernel_params, feature_selection_params, imputation_params, alignment_type, minimise_global_error
			- rp is populated with appropriate post_processing_params
			- returns true if .req is successfully parsed, false otherwise
		*/
		template <typename T>
		static bool parse_into_msparams(const std::string& req_string,
			std::string& solver_name,
			std::vector<std::vector<point2_map<T>>>& frames_shapes_tracking,
			std::vector<std::map<std::string, T>>& frames_controls,
			std::map<std::string, typename modular_solver<T>::control_range>& control_ranges,
			typename modular_solver<T>::training_params& tp,
			std::vector<shot_data<T,2>>& shots,
			std::vector<std::pair<alignment_type, std::vector<std::vector<std::string>>>>& shapes_alignment_views_pointids)
		{
			try
			{
				epic::carbon::JsonElement req_json = epic::carbon::ReadJson(req_string);
				auto& config = req_json[CONFIG_STRING];
				solver_name = config[SOLVER_NAME_STRING].Get<std::string>();
				auto& poses = config[POSES_STRING].Array();
				const unsigned num_examples = static_cast<unsigned>(poses.size());

				parse_user_protocol<T>(config[USER_PROTOCOL_STRING], tp, num_examples);

				std::vector<std::pair<std::string, std::pair<alignment_type, std::vector<std::string>>>> shape_alignment_pointids;
				parse_shape_components(config[SHAPE_COMPONENTS_STRING].Array(), shape_alignment_pointids);

				parse_controls<T, 2>(config[CONTROLS_STRING].Array(), control_ranges);

				parse_poses(poses, shape_alignment_pointids, control_ranges, frames_shapes_tracking, frames_controls);

				if (req_json.Contains(SHOTS_STRING))
				{
					parse_shots(req_json[SHOTS_STRING].Array(), shots, frames_shapes_tracking);
				}

				assign_shape_view_pointids(req_json[SHOTS_STRING].Array(), shape_alignment_pointids, shapes_alignment_views_pointids);
				return true;
			}
			catch (const req_parse_error& e)
			{
				std::cerr << e.what() << std::endl;
				return false;
			}
		}

		// .req private functions
	private:
		static void assign_shape_view_pointids(const std::vector<epic::carbon::JsonElement>& json,
			const std::vector<std::pair<std::string, std::pair<alignment_type, std::vector<std::string>>>>& shapes_alignment_pointids,
			std::vector<std::pair<alignment_type, std::vector<std::vector<std::string>>>>& shapes_alignment_views_pointids)
		{
			unsigned shot = 0;
			unsigned num_views = 0;
			shapes_alignment_views_pointids.resize(shapes_alignment_pointids.size());
			for (auto shots_it = json.begin(); shots_it != json.end(); ++shots_it, ++shot)
			{
				const auto& tracking = (*shots_it)[TRACKING_STRING].Array();

				if (shot == 0)
				{
					num_views = static_cast<unsigned>(tracking.size());
					for (unsigned shape = 0; shape < shapes_alignment_views_pointids.size(); ++shape)
					{
						shapes_alignment_views_pointids[shape].second.resize(num_views);
					}
					unsigned view = 0;
					for (auto tracking_obj = tracking.begin(); tracking_obj != tracking.end(); ++tracking_obj, ++view)
					{
						// if "ContainsShapes" field is either absent or empty, assume this view contains point ids of every shape
						if (!(*tracking_obj).Contains(CONTAINS_SHAPES_STRING))
						{
							for (unsigned shape = 0; shape < shapes_alignment_pointids.size(); ++shape)
							{
								shapes_alignment_views_pointids[shape].second[view] = shapes_alignment_pointids[shape].second.second;
							}
						}
						else
						{
							std::vector<std::string> contains_shapes;
							parse_json_array<std::string>((*tracking_obj)[CONTAINS_SHAPES_STRING].Array(), contains_shapes);
							if (contains_shapes.size() == 0)
							{
								for (unsigned shape = 0; shape < shapes_alignment_pointids.size(); ++shape)
								{
									shapes_alignment_views_pointids[shape].second[view] = shapes_alignment_pointids[shape].second.second;
								}
							}
							else
							{
								for (unsigned shape = 0; shape < shapes_alignment_pointids.size(); ++shape)
								{
									for (const std::string& contained_shape : contains_shapes)
									{
										if (contained_shape == shapes_alignment_pointids[shape].first)
										{
											shapes_alignment_views_pointids[shape].second[view] = shapes_alignment_pointids[shape].second.second;
										}
									}
								}
							}
						}
					}
				}
				else if (tracking.size() != num_views)
				{
					throw req_parse_error("All shots must contain the same number of views");
				}
			}
		}

		template <typename T>
		static void parse_user_protocol(const epic::carbon::JsonElement& user_protocol, typename modular_solver<T>::training_params& tp, const unsigned num_examples)
		{
			parse_json_array(user_protocol[FEATURE_FRACTION_STRING].Array(), tp.feature_selection.feature_fractions_to_try);

			tp.imputation.num_iterations = user_protocol[ITERATIVE_STRING].IsTrue() ? 5 : 0;

			parse_mode_factors<T>(user_protocol[MODE_FACTORS_STRING].Array(), tp.compression.input_max_modes_to_try, num_examples);
			tp.compression.input_variance_to_try = { 1.0 };
			if (user_protocol.Contains(SHAPES_PCA_VARIANCE_STRING))
			{
				tp.compression.shape_pca_variance = user_protocol[SHAPES_PCA_VARIANCE_STRING].Get<T>();
			}
			else
			{
				tp.compression.shape_pca_variance = 0.0;
			}

			tp.metric = user_protocol[GLOBAL_ERROR_STRING].IsTrue() ? ms_cross_validation_metric::absolute_average : ms_cross_validation_metric::absolute_maximum;

			tp.num_threads = user_protocol.Contains(NUM_THREADS_STRING) ? user_protocol[NUM_THREADS_STRING].Value<unsigned>() : 1;

			parse_kernel_type(user_protocol[ORDER_STRING].Get<std::string>(), tp.kernel.kernel_type);

			tp.regression = parse_lambdas<T>(user_protocol[LAMBDAS_STRING].Array());
		}

		template <typename T>
		static typename modular_solver<T>::training_params::regression_params parse_lambdas(const std::vector<epic::carbon::JsonElement>& lambdas)
		{
			typename modular_solver<T>::training_params::regression_params rp;
			rp.regression_type = ms_regression_options::ridge_regression;
			rp.lambdas_to_try.resize(lambdas.size());
			unsigned i = 0;
			for (auto lambda_it = lambdas.begin(); lambda_it != lambdas.end(); ++lambda_it, ++i)
			{
				T lambda = (*lambda_it).Get<T>();
				if (lambda == 0.0)
				{
					rp.lambdas_to_try[i] = 1e-10; // lambda == 0.0 signals to dlib to do a parameter optimisation for lambda, which we don't want.
				}
				else
				{
					rp.lambdas_to_try[i] = lambda * lambda; // the Go solver actually squares the values of lambda provided by the user when training its regressors, so we need to do the same here
				}
			}
			return rp;
		}

		static void parse_shape_components(const std::vector<epic::carbon::JsonElement>& shape_components,
			std::vector<std::pair<std::string, std::pair<alignment_type, std::vector<std::string>>>>& shape_alignment_pointids)
		{
			unsigned shape = 0;
			try
			{
				shape_alignment_pointids.resize(shape_components.size());
				for (auto shape_component = shape_components.begin(); shape_component != shape_components.end(); ++shape_component, ++shape)
				{

					auto part_name = (*shape_component)[PART_NAME_STRING].Get<std::string>();
					auto alignment_field = (*shape_component)[ALIGN_METHOD_STRING].Get<std::string>();
					auto alignment = alignment_type::MAX_NUMBER_OF_alignment_type;
					if (alignment_field == NONE_STRING)
					{
						alignment = cm::alignment_type::none;
					}
					else if (alignment_field == RIGID2D_STRING)
					{
						alignment = cm::alignment_type::rigid;
					}
					else if (alignment_field == SIMILARITY2D_STRING)
					{
						alignment = cm::alignment_type::similarity;
					}
					else if (alignment_field == TRANSLATE_STRING || alignment_field == TRANSLATION_STRING)
					{
						alignment = cm::alignment_type::translation;
					}
					else
					{
						throw req_parse_error("Unsupported align type.");
					}
					shape_alignment_pointids[shape].first = part_name;
					shape_alignment_pointids[shape].second.first = alignment;
					shape_alignment_pointids[shape].second.second = parse_point_id_string((*shape_component)[POINT_ID_STRING_STRING]);
				}
			}
			catch (const std::exception& err)
			{
				throw req_parse_error("parse_shape_components: Error parsing shapes json: " + std::string(err.what()));
			}
		}

		template <typename T>
		static void parse_poses(const std::vector<epic::carbon::JsonElement>& poses,
			const std::vector<std::pair<std::string, std::pair<alignment_type, std::vector<std::string>>>>& shape_alignment_pointids,
			const std::map<std::string, typename modular_solver<T>::control_range> control_ranges,
			std::vector<std::vector<cm::point2_map<T>>>& frames_shapes_tracking,
			std::vector<std::map<std::string, T>>& frames_controls)
		{
			frames_controls.resize(poses.size());
			frames_shapes_tracking.assign(poses.size(), std::vector<cm::point2_map<T>>(shape_alignment_pointids.size()));
			unsigned i = 0;
			for (auto poses_it = poses.begin(); poses_it != poses.end(); ++poses_it, ++i)
			{
				const auto& control_values = (*poses_it)[CONTROL_VALUES_STRING];
				for (auto control_range = control_ranges.begin(); control_range != control_ranges.end(); ++control_range)
				{
					if (control_values.Contains(control_range->first))
					{
						frames_controls[i][control_range->first] = control_values[control_range->first].template Get<T>();
					}
					else
					{
						throw req_parse_error("parse_poses: required control not found in json.");
					}
				}

				const auto& shape = (*poses_it)[SHAPE_STRING];
				for (unsigned shape_index = 0; shape_index < shape_alignment_pointids.size(); ++shape_index)
				{
					for (const auto& id : shape_alignment_pointids[shape_index].second.second)
					{
						if (shape[X_STRING].Contains(id) && shape[Y_STRING].Contains(id))
						{
							frames_shapes_tracking[i][shape_index][id](0) = shape[X_STRING][id].Get<T>();
							frames_shapes_tracking[i][shape_index][id](1) = shape[Y_STRING][id].Get<T>();
						}
						else
						{
							throw req_parse_error("parse_poses: required point ID not found in json.");
						}
					}
				}
			}
		}

		template <typename T>
		static void parse_shots(const std::vector<epic::carbon::JsonElement>& json,
			std::vector<shot_data<T, 2>>& shots,
			const std::vector<std::vector<point2_map<T>>>& frames_shapes_tracking)
		{
			try
			{
				unsigned shot = 0;
				unsigned num_views = 0;
				shots.resize(json.size());
				for (auto shots_it = json.begin(); shots_it != json.end(); ++shots_it, ++shot)
				{
					if ((*shots_it).Contains(REFERENCE_FRAMES_STRING) && !(*shots_it).Contains(FRAMES_STRING))
					{
						const auto & reference_frames = (*shots_it)[REFERENCE_FRAMES_STRING].Array();
						if (reference_frames.size() == 2)
						{
							unsigned i = 0;
							for (auto reference_frame = reference_frames.begin(); reference_frame != reference_frames.end(); ++reference_frame, ++i)
							{
								if (i == 0)
								{
									shots[shot].rp.shot_neutral_correction_frame = (*reference_frame).Get<int>();
								}
								else if (i == 1)
								{
									shots[shot].rp.reference_neutral_pose = frames_shapes_tracking[(*reference_frame).Get<int>()];
								}
							}
						}
						else
						{
							shots[shot].rp.shot_neutral_correction_frame = -1;
						}
					}
					else if ((*shots_it).Contains(FRAMES_STRING) && !(*shots_it).Contains(REFERENCE_FRAMES_STRING))
					{
						const auto & frames = (*shots_it)[FRAMES_STRING].Array();
						if (frames.size() == 2)
						{
							unsigned i = 0;
							for (auto frame = frames.begin(); frame != frames.end(); ++frame, ++i)
							{
								if (i == 0)
								{
									shots[shot].rp.shot_neutral_correction_frame = (*frame).Get<int>();
								}
								else if (i == 1)
								{
									shots[shot].rp.reference_neutral_pose = frames_shapes_tracking[(*frame).Get<int>()];
								}
							}
						}
						else
						{
							shots[shot].rp.shot_neutral_correction_frame = -1;
						}
					}
					else if (!(*shots_it).Contains(FRAMES_STRING) && !(*shots_it).Contains(REFERENCE_FRAMES_STRING))
					{
						shots[shot].rp.shot_neutral_correction_frame = -1;
					}
					else
					{
						throw req_parse_error("parse_shots: Error parsing tracking: tracking object contains both a \"ReferenceFrames\" and a \"Frames\" field. Unable to resolve.");
					}
					shots[shot].rp.post_processing.cleaning =(*shots_it)[CLEANING_STRING].Get<T>();
					shots[shot].rp.post_processing.cutoff =(*shots_it)[CUTOFF_STRING].Get<T>();
					const auto& cut_starts =(*shots_it)[CUT_STARTS_STRING].Array();
					unsigned i = 0;
					shots[shot].rp.post_processing.cut_starts.resize(cut_starts.size());
					for (auto cut_start = cut_starts.begin(); cut_start != cut_starts.end(); ++cut_start, ++i)
					{
						shots[shot].rp.post_processing.cut_starts[i] = (*cut_start).Get<int>();
					}
					shots[shot].rp.post_processing.key_frame_reduction =(*shots_it)[KFR_STRING].Get<T>();
					shots[shot].rp.post_processing.output_constraint =(*shots_it)[OUTPUT_CONSTRAINT_STRING].Get<T>();
					shots[shot].rp.post_processing.smoothing_level =(*shots_it)[SMOOTHING_LEVEL_STRING].Get<int>();
					shots[shot].result_path =(*shots_it)[RESULT_FILENAME_STRING].Get<std::string>();
					shots[shot].result_path.replace(shots[shot].result_path.rfind('.'), shots[shot].result_path.length() - shots[shot].result_path.rfind('.'), ".qsa");
					const auto& tracking =(*shots_it)[TRACKING_STRING].Array();
					if (shot == 0)
					{
						num_views = static_cast<unsigned>(tracking.size());
					}
					else if (tracking.size() != num_views)
					{
						throw req_parse_error("parse_shots: Error parsing tracking: number of tracking objects (" + std::to_string(tracking.size()) + ") must match number of registered solver views (" + std::to_string(num_views) + ")");
					}

					shots[shot].views_tracking.resize(tracking.size());
					unsigned view = 0;
					for (auto tracking_obj = tracking.begin(); tracking_obj != tracking.end(); ++tracking_obj, ++view)
					{
						shots[shot].views_tracking[view].tracking_file_path = (*tracking_obj)[FILENAME_STRING].Get<std::string>();
						if (!(*tracking_obj).Contains(ID_OFFSET_STRING))
						{
							shots[shot].views_tracking[view].idOffset = 0;
						}
						else
						{
							shots[shot].views_tracking[view].idOffset = (*tracking_obj)[ID_OFFSET_STRING].Get<int>();
						}
					}
				}
			}
			catch (const std::exception& err)
			{
				throw req_parse_error("parse_shots: Error parsing json array: " + std::string(err.what()));
			}
		}
		// public .re3 functions
	public:
			/*
			Parses a .req file and populates runtime params for a modular solver

			- Requires
				- shots_file_string contains the contents of a valid .req file (just the shots part)

			- Ensures
				- shots is populated with appropriate post_processing_params, output path, and qp3 paths
				- returns true if .re3 is successfully parsed, false otherwise
			*/
			template <typename T>
			static bool parse_into_shots(const std::string& shots_file_string,
				std::vector<shot_data<T, 3>>& shots,
				const std::vector<std::vector<point3_map<T>>>& frames_shapes_tracking)
			{
				try
				{
					epic::carbon::JsonElement shots_json = epic::carbon::ReadJson(shots_file_string);

					parse_shots(shots_json[SHOTS_STRING].Array(), shots, frames_shapes_tracking);
				}
				catch (const req_parse_error& e)
				{
					std::cerr << e.what() << std::endl;
					return false;
				}

				return true;
			}

			/*
			Parses a .re3 file and populates tracking training data, control training data, control ranges, training params, and runtime params for a modular solver

			- Requires
				- re3_path_string contains a string containing the contents of a valid .req file

			- Ensures
				- frames_shapes_tracking is returned with the same number of frames as frames_controls
				- frames_controls contains only controls present in control_ranges
				- tp is populated with appropriate compression_params, regression_params, kernel_params, feature_selection_params, imputation_params, alignment_type, minimise_global_error
				- shots is populated with appropriate post_processing_params, output path, and qp3 paths
				- returns true if .re3 is successfully parsed, false otherwise
			*/
			template <typename T>
			static bool parse_into_msparams(const std::string& re3_path_string,
				std::string& solver_name,
				std::vector<std::vector<point3_map<T>>>& frames_shapes_tracking,
				std::vector<std::map<std::string, T>>& frames_controls,
				std::map<std::string, typename modular_solver<T, 3>::control_range>& control_ranges,
				typename cm::modular_solver<T, 3>::training_params& tp,
				std::vector<shot_data<T, 3>>& shots,
				std::vector<std::pair<alignment3d_type, std::vector<std::vector<std::string>>>>& shapes_alignment_views_pointids)
			{
				try
				{
					epic::carbon::JsonElement re3_json = epic::carbon::ReadJson(re3_path_string);
					auto& config = re3_json[CONFIG_STRING];
					solver_name = config[SOLVER_NAME_STRING].Get<std::string>();
					auto& poses = config[POSES_STRING].Array();
					const unsigned num_examples = static_cast<unsigned>(poses.size());

					parse_user_protocol<T>(config[USER_PROTOCOL_STRING], tp, num_examples);

					std::vector<std::pair<std::string, std::pair<alignment3d_type, std::vector<std::string>>>> shape_alignment_pointids;
					parse_shape_components(config[SHAPE_COMPONENTS_STRING].Array(), shape_alignment_pointids);

					parse_controls<T, 3>(config[CONTROLS_STRING].Array(), control_ranges);

					parse_poses(poses, shape_alignment_pointids, control_ranges, frames_shapes_tracking, frames_controls);

					if (re3_json.Contains(SHOTS_STRING))
					{
						parse_shots(re3_json[SHOTS_STRING].Array(), shots, frames_shapes_tracking);

						assign_shape_view_pointids(re3_json[SHOTS_STRING].Array(), shape_alignment_pointids, shapes_alignment_views_pointids);
					}

					return true;
				}
				catch (const req_parse_error& e)
				{
					std::cerr << e.what() << std::endl;
					return false;
				}

				return true;
			}
		// private .re3 functions
	private:
		static void assign_shape_view_pointids(const std::vector<epic::carbon::JsonElement>& json,
			const std::vector<std::pair<std::string, std::pair<alignment3d_type, std::vector<std::string>>>>& shapes_alignment_pointids,
			std::vector<std::pair<alignment3d_type, std::vector<std::vector<std::string>>>>& shapes_alignment_views_pointids)
		{
			shapes_alignment_views_pointids.resize(shapes_alignment_pointids.size());
			unsigned shot = 0;
			if (json.size() == 0)
			{
				std::cout << "Warning - no shots list found, assuming input will consist of a single view containing all specified shapes." << std::endl;
				for (unsigned shape = 0; shape < shapes_alignment_pointids.size(); ++shape)
				{
					shapes_alignment_views_pointids[shape].second.emplace_back(shapes_alignment_pointids[shape].second.second);
				}
			}
			else
			{
				for (auto shots_it = json.begin(); shots_it != json.end(); ++shots_it, ++shot)
				{
					const auto& tracking = (*shots_it)[TRACKING_STRING].Array();
					unsigned num_views = 0;

					if (shot == 0)
					{
						num_views = static_cast<unsigned>(tracking.size());
						for (unsigned shape = 0; shape < shapes_alignment_views_pointids.size(); ++shape)
						{
							shapes_alignment_views_pointids[shape].second.resize(num_views);
						}
						unsigned view = 0;
						for (auto tracking_obj = tracking.begin(); tracking_obj != tracking.end(); ++tracking_obj, ++view)
						{
							// if "ContainsShapes" field is either absent or empty, assume this view contains point ids of every shape
							if (!(*tracking_obj).Contains(CONTAINS_SHAPES_STRING))
							{
								for (unsigned shape = 0; shape < shapes_alignment_pointids.size(); ++shape)
								{
									shapes_alignment_views_pointids[shape].second[view] = shapes_alignment_pointids[shape].second.second;
								}
							}
							else
							{
								std::vector<std::string> contains_shapes;
								parse_json_array<std::string>((*tracking_obj)[CONTAINS_SHAPES_STRING].Array(), contains_shapes);
								if (contains_shapes.size() == 0)
								{
									for (unsigned shape = 0; shape < shapes_alignment_pointids.size(); ++shape)
									{
										shapes_alignment_views_pointids[shape].second[view] = shapes_alignment_pointids[shape].second.second;
									}
								}
								else
								{
									for (unsigned shape = 0; shape < shapes_alignment_pointids.size(); ++shape)
									{
										for (const std::string& contained_shape : contains_shapes)
										{
											if (contained_shape == shapes_alignment_pointids[shape].first)
											{
												shapes_alignment_views_pointids[shape].second[view] = shapes_alignment_pointids[shape].second.second;
											}
										}
									}
								}
							}
						}
					}
					else if (tracking.size() != num_views)
					{
						throw req_parse_error("All shots must contain the same number of views");
					}
				}
			}
		}


		template <typename T>
		static void parse_user_protocol(const epic::carbon::JsonElement& user_protocol, typename cm::modular_solver<T, 3>::training_params& tp, const unsigned num_examples)
		{
			parse_mode_factors<T>(user_protocol[MODE_FACTORS_STRING].Array(), tp.compression.input_max_modes_to_try, num_examples);
			parse_json_array(user_protocol[PCA_VARIANCES_STRING].Array(), tp.compression.input_variance_to_try);
			if (user_protocol.Contains(SHAPES_PCA_VARIANCE_STRING))
			{
				tp.compression.shape_pca_variance = user_protocol[SHAPES_PCA_VARIANCE_STRING].Get<T>();
			}
			else
			{
				tp.compression.shape_pca_variance = 0.0;
			}
			parse_json_array(user_protocol[FEATURE_FRACTIONS_STRING].Array(), tp.feature_selection.feature_fractions_to_try);
			tp.imputation.num_iterations = user_protocol[IMPUTATION_ITERATIONS_STRING].Value<unsigned>();
			tp.num_threads = user_protocol.Contains(NUM_THREADS_STRING) ? user_protocol[NUM_THREADS_STRING].Value<unsigned>() : 1;
			parse_cross_validation_metric(user_protocol[CROSS_VALIDATION_METRIC_STRING].Get<std::string>(), tp.metric);
			parse_regression_type(user_protocol[REGRESSION_TYPE_STRING].Get<std::string>(), tp.regression.regression_type);
			parse_json_array(user_protocol[LAMBDAS_STRING].Array(), tp.regression.lambdas_to_try);
			parse_kernel_type(user_protocol[KERNEL_TYPE_STRING].Get<std::string>(), tp.kernel.kernel_type);
		}

		static void parse_regression_type(const std::string& regression_type_string, cm::ms_regression_options& regression_type)
		{
			if (regression_type_string == RIDGE_REGRESSION_STRING)
			{
				regression_type = cm::ms_regression_options::ridge_regression;
			}
			else
			{
				throw req_parse_error("parse_regression_type: Unrecognised regression type: " + regression_type_string);
			}
		}

		static void parse_kernel_type(const std::string& kernel_type_string, cm::ms_kernel_options& kernel_type)
		{
			if (kernel_type_string == LINEAR_STRING)
			{
				kernel_type = cm::ms_kernel_options::linear;
			}
			else
			{
				throw req_parse_error("prase_kernel_type: Unrecognised kernel type: " + kernel_type_string);
			}
		}

		static void parse_cross_validation_metric(const std::string& cross_validation_metric_string, cm::ms_cross_validation_metric& metric)
		{
			if (cross_validation_metric_string == ABSOLUTE_AVERAGE_STRING)
			{
				metric = cm::ms_cross_validation_metric::absolute_average;
			}
			else if (cross_validation_metric_string == ABSOLUTE_MAXIMUM_STRING)
			{
				metric = cm::ms_cross_validation_metric::absolute_maximum;
			}
			else
			{
				throw req_parse_error("parse_corss_validation_metric: Unrecognised cross-validation metric: " + cross_validation_metric_string);
			}
		}

		static void parse_shape_components(const std::vector<epic::carbon::JsonElement>& shape_components,
			std::vector<std::pair<std::string, std::pair<alignment3d_type, std::vector<std::string>>>>& shape_alignment_pointids)
		{
			try
			{
				shape_alignment_pointids.resize(shape_components.size());
				unsigned shape = 0;
				for (auto shape_component = shape_components.begin(); shape_component != shape_components.end(); ++shape_component, ++shape)
				{
					auto part_name = (*shape_component)[PART_NAME_STRING].Get<std::string>();
					const auto alignment_field = (*shape_component)[ALIGN_METHOD_STRING].Get<std::string>();
					auto alignment = alignment3d_type::MAX_NUMBER_OF_alignment3d_type;
					if (alignment_field == NONE_STRING)
					{
						alignment = cm::alignment3d_type::none;
					}
					else if (alignment_field == TRANSLATION_STRING)
					{
						alignment = cm::alignment3d_type::translation;
					}
					else if (alignment_field == RIGID_STRING)
					{
						alignment = cm::alignment3d_type::rigid;
					}
					else if (alignment_field == SIMILARITY_STRING)
					{
						alignment = cm::alignment3d_type::similarity;
					}
					else
					{
						throw req_parse_error("parse_shape_component: Unrecognised alignment type : " + alignment_field);
					}
					shape_alignment_pointids[shape].first = part_name;
					shape_alignment_pointids[shape].second.first = alignment;
					shape_alignment_pointids[shape].second.second = parse_point_id_string((*shape_component)[POINT_ID_STRING_STRING]);
				}
			}
			catch (const std::exception& err)
			{
				throw req_parse_error("parse_shape_components: Error parsing shapes json: " + std::string(err.what()));
			}
		}

		template <typename T>
		static void parse_controls(const std::vector<epic::carbon::JsonElement>& controls, std::map<std::string, typename cm::modular_solver<T, 3>::control_range>& control_ranges)
		{
			try
			{
				for (auto control = controls.begin(); control != controls.end(); ++control)
				{
					auto control_name = (*control)[CONTROL_NAME_STRING].Get<std::string>();
					const auto& range_info = (*control)[RANGE_INFO_STRING].Array();
					typename cm::modular_solver<T, 3>::control_range& cr = control_ranges[control_name];
					unsigned i = 0;
					for (auto range_it = range_info.begin(); range_it != range_info.end(); ++range_it, ++i)
					{
						if (i == 0)
						{
							cr.min = (*range_it).Get<T>();
						}
						else if (i == 1)
						{
							cr.max = (*range_it).Get<T>();
						}
						else if (i == 2)
						{
							cr.def = (*range_it).Get<T>();
						}
						else
						{
							throw req_parse_error("parse_controls: Encountered unexpected element of control range array");
						}
					}
				}
			}
			catch (const std::exception& err)
			{
				throw req_parse_error("parse_controls: Error parsing controls json: " + std::string(err.what()));
			}
		}

		template <typename T>
		static void parse_poses(const std::vector<epic::carbon::JsonElement>& poses,
			const std::vector<std::pair<std::string, std::pair<alignment3d_type, std::vector<std::string>>>>& shapes_alignment_pointids,
			const std::map<std::string, typename cm::modular_solver<T, 3>::control_range>& control_ranges,
			std::vector<std::vector<cm::point3_map<T>>>& frames_shapes_tracking_training,
			std::vector<std::map<std::string, T>>& frames_controls_training)
		{
			try
			{
				frames_shapes_tracking_training.assign(poses.size(), std::vector<std::map<std::string, cm::point3<T>>>(shapes_alignment_pointids.size()));
				frames_controls_training.resize(poses.size());
				unsigned example = 0;
				for (auto pose = poses.begin(); pose != poses.end(); ++pose, ++example)
				{
					const auto& control_values = (*pose)[CONTROL_VALUES_STRING];
					for (auto control_it = control_ranges.begin(); control_it != control_ranges.end(); ++control_it)
					{
						frames_controls_training[example][control_it->first] = control_values[control_it->first].template Get<T>();
					}

					const auto& shapes = (*pose)[SHAPES_STRING].Array();
					const auto& shape = shapes.begin();
					for (unsigned shape_index = 0; shape_index < shapes_alignment_pointids.size(); ++shape_index)
					{
						for (const auto& id : shapes_alignment_pointids[shape_index].second.second)
						{
							const auto& point = (*shape)[id].Array();
							if (point.size() != 3)
							{
								throw req_parse_error("parse_poses: Error - all points must be 3D");
							}
							auto& point3d = frames_shapes_tracking_training[example][shape_index][id];

							unsigned i = 0;
							for (auto ordinate = point.begin(); ordinate != point.end(); ++ordinate, ++i)
							{
								point3d(i) = (*ordinate).Get<T>();
							}
						}
					}
				}
			}
			catch (const std::exception& err)
			{
				throw req_parse_error("parse_poses: Error parsing poses json: " + std::string(err.what()));
			}
		}

		template <typename T>
		static void parse_shots(const std::vector<epic::carbon::JsonElement>& json, std::vector<shot_data<T, 3>>& shots,
			const std::vector<std::vector<point3_map<T>>>& frames_shapes_tracking)
		{
			try
			{
				unsigned shot = 0;
				unsigned num_views = 0;
				shots.resize(json.size());
				for (auto shots_it = json.begin(); shots_it != json.end(); ++shots_it, ++shot)
				{
					if ((*shots_it).Contains(REFERENCE_FRAMES_STRING) && !(*shots_it).Contains(FRAMES_STRING))
					{
						if ((*shots_it)[REFERENCE_FRAMES_STRING].Array().size() == 2)
						{
							unsigned i = 0;
							for (auto reference_frame = (*shots_it)[REFERENCE_FRAMES_STRING].Array().begin();
								reference_frame != (*shots_it)[REFERENCE_FRAMES_STRING].Array().end(); ++reference_frame, ++i)
							{
								if (i == 0)
								{
									shots[shot].rp.shot_neutral_correction_frame = (*reference_frame).Get<int>();
								}
								else if (i == 1)
								{
									shots[shot].rp.reference_neutral_pose = frames_shapes_tracking[static_cast<unsigned>((*reference_frame).Get<int>())];
								}
							}
						}
						else
						{
							shots[shot].rp.shot_neutral_correction_frame = -1;
						}
					}
					else if ((*shots_it).Contains(FRAMES_STRING) && !(*shots_it).Contains(REFERENCE_FRAMES_STRING))
					{
						if ((*shots_it)[FRAMES_STRING].Array().size() == 2)
						{
							unsigned i = 0;
							for (auto frame = (*shots_it)[FRAMES_STRING].Array().begin(); frame != (*shots_it)[FRAMES_STRING].Array().begin(); ++frame, ++i)
							{
								if (i == 0)
								{
									shots[shot].rp.shot_neutral_correction_frame = (*frame).Get<int>();
								}
								else if (i == 1)
								{
									shots[shot].rp.reference_neutral_pose = frames_shapes_tracking[(*frame).Get<int>()];
								}
							}
						}
						else
						{
							shots[shot].rp.shot_neutral_correction_frame = -1;
						}
					}
					else if (!(*shots_it).Contains(FRAMES_STRING) && !(*shots_it).Contains(REFERENCE_FRAMES_STRING))
					{
						shots[shot].rp.shot_neutral_correction_frame = -1;
					}
					else
					{
						throw req_parse_error("parse_shots: Error parsing tracking: tracking object contains both a \"ReferenceFrames\" and a \"Frames\" field. Unable to resolve.");
					}
					shots[shot].rp.post_processing.cleaning = (*shots_it)[CLEANING_STRING].Get<T>();
					shots[shot].rp.post_processing.cutoff = (*shots_it)[CUTOFF_STRING].Get<T>();
					const auto& cut_starts = (*shots_it)[CUT_STARTS_STRING].Array();

					unsigned i = 0;
					shots[shot].rp.post_processing.cut_starts.resize(cut_starts.size());
					for (auto cut_start = cut_starts.begin(); cut_start != cut_starts.end(); ++cut_start, ++i)
					{
						shots[shot].rp.post_processing.cut_starts[i] = (*cut_start).Get<int>();
					}
					shots[shot].rp.post_processing.key_frame_reduction = (*shots_it)[KFR_STRING].Get<T>();
					shots[shot].rp.post_processing.output_constraint = (*shots_it)[OUTPUT_CONSTRAINT_STRING].Get<T>();
					shots[shot].rp.post_processing.smoothing_level = (*shots_it)[SMOOTHING_LEVEL_STRING].Get<int>();
					shots[shot].result_path = (*shots_it)[RESULT_FILENAME_STRING].Get<std::string>();
					const auto& tracking = (*shots_it)[TRACKING_STRING].Array();
					if (shot == 0)
					{
						num_views = static_cast<unsigned>(tracking.size());
					}
					else if (tracking.size() != num_views)
					{
						throw req_parse_error("parse_shots: Error parsing tracking: number of tracking objects (" + std::to_string(tracking.size()) + ") must match number of registered solver views (" + std::to_string(num_views) + ")");
					}

					shots[shot].views_tracking.resize(tracking.size());
					unsigned view = 0;
					for (auto tracking_obj = tracking.begin(); tracking_obj != tracking.end(); ++tracking_obj, ++view)
					{
						shots[shot].views_tracking[view].tracking_file_path = (*tracking_obj)[FILENAME_STRING].Get<std::string>();
						if (!tracking_obj->Contains(ID_OFFSET_STRING))
						{
							shots[shot].views_tracking[view].idOffset = 0;
						}
						else
						{
							shots[shot].views_tracking[view].idOffset = (*tracking_obj)[ID_OFFSET_STRING].Get<int>();
						}
					}
				}
			}
			catch (const std::exception& err)
			{
				throw req_parse_error("parse_shots: Error parsing json array: " + std::string(err.what()));
			}
		}
	};
	const std::string goreq_parser::CONFIG_STRING = "Config";
	const std::string goreq_parser::SOLVER_NAME_STRING = "SolverName";
	const std::string goreq_parser::PART_NAME_STRING = "PartName";
	const std::string goreq_parser::POSES_STRING = "Poses";
	const std::string goreq_parser::USER_PROTOCOL_STRING = "UserProtocol";
	const std::string goreq_parser::SHAPE_COMPONENTS_STRING = "ShapeComponents";
	const std::string goreq_parser::CONTROLS_STRING = "Controls";
	const std::string goreq_parser::SHOTS_STRING = "Shots";
	const std::string goreq_parser::FEATURE_FRACTION_STRING = "FeatureFraction";
	const std::string goreq_parser::FEATURE_FRACTIONS_STRING = "FeatureFractions";
	const std::string goreq_parser::IMPUTATION_ITERATIONS_STRING = "ImputationIterations";
	const std::string goreq_parser::CROSS_VALIDATION_METRIC_STRING = "CrossValidationMetric";
	const std::string goreq_parser::REGRESSION_TYPE_STRING = "RegressionType";
	const std::string goreq_parser::RIDGE_REGRESSION_STRING = "ridge_regression";
	const std::string goreq_parser::ABSOLUTE_AVERAGE_STRING = "absolute_average";
	const std::string goreq_parser::ABSOLUTE_MAXIMUM_STRING = "absolute_maximum";
	const std::string goreq_parser::ITERATIVE_STRING = "Iterative";
	const std::string goreq_parser::MODE_FACTORS_STRING = "ModeFactors";
	const std::string goreq_parser::PCA_VARIANCES_STRING = "PCAVariances";
	const std::string goreq_parser::SHAPES_PCA_VARIANCE_STRING = "ShapesPCAVariance";
	const std::string goreq_parser::GLOBAL_ERROR_STRING = "GlobalError";
	const std::string goreq_parser::ORDER_STRING = "Order";
	const std::string goreq_parser::LAMBDAS_STRING = "Lambdas";
	const std::string goreq_parser::KERNEL_TYPE_STRING = "KernelType";
	const std::string goreq_parser::ALIGN_METHOD_STRING = "AlignMethod";
	const std::string goreq_parser::POINT_ID_STRING_STRING = "PointIDString";
	const std::string goreq_parser::CONTROL_NAME_STRING = "ControlName";
	const std::string goreq_parser::RANGE_INFO_STRING = "RangeInfo";
	const std::string goreq_parser::CONTROL_VALUES_STRING = "ControlValues";
	const std::string goreq_parser::SHAPE_STRING = "Shape";
	const std::string goreq_parser::SHAPES_STRING = "Shapes";
	const std::string goreq_parser::X_STRING = "X";
	const std::string goreq_parser::Y_STRING = "Y";
	const std::string goreq_parser::FILENAME_STRING = "Filename";
	const std::string goreq_parser::CONTAINS_SHAPES_STRING = "ContainsShapes";
	const std::string goreq_parser::ID_OFFSET_STRING = "IDOffset";
	const std::string goreq_parser::OUTPUT_CONSTRAINT_STRING = "OutputConstraint";
	const std::string goreq_parser::KFR_STRING = "KFR";
	const std::string goreq_parser::CLEANING_STRING = "Cleaning";
	const std::string goreq_parser::CUTOFF_STRING = "Cutoff";
	const std::string goreq_parser::SMOOTHING_LEVEL_STRING = "SmoothingLevel";
	const std::string goreq_parser::CUT_STARTS_STRING = "CutStarts";
	const std::string goreq_parser::REFERENCE_FRAMES_STRING = "ReferenceFrames";
	const std::string goreq_parser::FRAMES_STRING = "Frames";
	const std::string goreq_parser::TRACKING_STRING = "Tracking";
	const std::string goreq_parser::RESULT_FILENAME_STRING = "ResultFilename";
	const std::string goreq_parser::NEUTRAL_FRAME_STRING = "NeutralFrame";
	const std::string goreq_parser::LINEAR_STRING = "Linear";
	const std::string goreq_parser::NONE_STRING = "none";
	const std::string goreq_parser::RIGID2D_STRING = "2drigid";
	const std::string goreq_parser::SIMILARITY2D_STRING = "2dsimilarity";
	const std::string goreq_parser::TRANSLATE_STRING = "translate";
	const std::string goreq_parser::TRANSLATION_STRING = "translation";
	const std::string goreq_parser::RIGID_STRING = "rigid";
	const std::string goreq_parser::SIMILARITY_STRING = "similarity";
	const std::string goreq_parser::NUM_THREADS_STRING = "NumThreads";
}
