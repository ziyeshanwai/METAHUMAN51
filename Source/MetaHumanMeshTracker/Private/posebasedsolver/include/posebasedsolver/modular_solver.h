// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>

#include "data_types.h"
#include "data_utils.h"
#include "serialization_utils.h"
#include "pca_model_training.h"
#include "general_geometry.h"
#include "prediction_model_wrapper.h"
#include "serialization_utils.h"
#include "modular_solver_regressor_factory.h"
#include "savitzky_golay_filter.h"
#include "misc_utils.h"
#include "transforms.h"
#include "transforms3d.h"
#include <memory>
#include <algorithm>
#include <functional>
#include <map>
#include <type_traits>

namespace cm
{
	/*
		modular solver class

		- REQUIREMENTS ON T
			- Must be either float, double, or long double, ie:
			- is_float_type<T>::value == true

		- INITIAL VALUE
			- is_trained_ = false;
			- n_var_output_ = 0;

		- WHAT THIS OBJECT REPRESENTS
			- This object represents a modular framework for turning tracking data into rig control values
			- The modular solver currently supports:
				multi-view solving in both 2D and 3D
				alignment
				neutral correction
				input PCA
				feature selection
				imputation
				choice of the following kernel types: linear
				choice of the following regression types: ridge-regression
				choice of the following cross-validation metrics: minimise absolute maximum of regressors errors, minimise absolute average of regressors errors
				output constraint
				key-frame-reduction
				savitzky golay smoothing
	*/
	template <typename T, int D = 2>
	class modular_solver
	{
	public:
		using P = dlib::vector<T, D>;
		typedef typename std::conditional<D == 3, alignment3d_type, alignment_type>::type alignmentD_type;
		/*
			runtime params struct

			- REQUIREMENTS ON T
				- Must be either float, double, or long double, ie:
				- is_float_type<T>::value == true

			- WHAT THIS OBJECT REPRESENTS
				- This object represents a container for parameters required by modular_solver during runtime
		*/
		struct runtime_params
		{
			struct post_processing_params
			{
				T output_constraint = 0.0;
				T key_frame_reduction = 0.0;
				T cutoff = 0.0;
				T cleaning = 0.0;
				int smoothing_level = 0;
				std::vector<int> cut_starts;
			};

			post_processing_params post_processing;
			int shot_neutral_correction_frame;
			std::vector<std::map<std::string, P>> reference_neutral_pose;
		};

		/*
			control range struct

			- REQUIREMENTS ON T
				- Must be either float, double, or long double, ie:
				- is_float_type<T>::value == true

			- WHAT THIS OBJECT REPRESENTS
				- This object represents a container for a particular rig control's minimum, maximum, and default values
		*/
		struct control_range
		{
			T min;
			T max;
			T def;

			/*
				friend function for serialization
			*/
			friend void serialize(const control_range& cr, std::ostream& out)
			{
				dlib::serialize(modular_solver<T, D>::control_range::version, out);
				dlib::serialize(cr.min, out);
				dlib::serialize(cr.max, out);
				dlib::serialize(cr.def, out);
			}

			/*
				friend function for deserialization
			*/
			friend void deserialize(control_range& cr, std::istream& in)
			{
				unsigned cur_version;
				dlib::deserialize(cur_version, in);

				// current version deserialization
				// IMPORTANT: please try and make class serialization / deserialization back-compatible
				// by supporting multiple versions if possible, and if not, function should throw an exception
				if (cur_version == 1u)
				{
					throw serialization_error("modular_solver::control_range version 1 incompatible with version " + std::to_string(modular_solver<T, D>::control_range::version));
				}
				else if (cur_version == 2u)
				{
					dlib::deserialize(cr.min, in);
					dlib::deserialize(cr.max, in);
					dlib::deserialize(cr.def, in);
				}
				else
				{
					// back-compatibility code for previous versions should go here
					throw serialization_version_error("modular_solver::control_range", modular_solver<T, D>::control_range::version, cur_version);
				}
			}
		private:
			static const unsigned version;
		};

		/*
			collection of training parameters
		*/
		struct training_params
		{
			/*
				input/output PCA parameters to try during training

				- REQUIRES
					- 0.0 < input_variance_to_try <= 1.0
					- input_max_modes_to_try >= 0
			*/
			struct compression_params
			{
				std::vector<T> input_variance_to_try;
				std::vector<int> input_max_modes_to_try;
				T shape_pca_variance = 0.0;
				static const unsigned version;

				/*
					friend function for serialization
				*/
				friend void serialize(const compression_params& cp, std::ostream& out)
				{
					dlib::serialize(modular_solver::training_params::compression_params::version, out);
					dlib::serialize(cp.input_variance_to_try, out);
					dlib::serialize(cp.input_max_modes_to_try, out);
					dlib::serialize(cp.shape_pca_variance, out);
				}

				/*
					friend function for deserialization
				*/
				friend void deserialize(compression_params& cp, std::istream& in)
				{
					unsigned cur_version;
					dlib::deserialize(cur_version, in);

					// current version deserialization
					// IMPORTANT: please try and make class serialization / deserialization back-compatible
					// by supporting multiple versions if possible, and if not, function should throw an exception
					if (cur_version == 1u)
					{
						throw serialization_error("modular_solver::compression_params version 1 incompatible with version " + std::to_string(modular_solver<T, D>::training_params::compression_params::version));
					}
					else if (cur_version == 2u)
					{
						dlib::deserialize(cp.input_variance_to_try, in);
						dlib::deserialize(cp.input_max_modes_to_try, in);
						cp.shape_pca_variance = 0.0;
					}
					else if (cur_version == 3u)
					{
						dlib::deserialize(cp.input_variance_to_try, in);
						dlib::deserialize(cp.input_max_modes_to_try, in);
						dlib::deserialize(cp.shape_pca_variance, in);
					}
					else
					{
						// back-compatibility code for previous versions should go here
						throw serialization_version_error("modular_solver::training_params::compression_params", modular_solver<T, D>::training_params::compression_params::version, cur_version);
					}
				}
			};

			/*
				regression paramters to try during training

				- REQUIRES
					- lambdas_to_try > 0.0
			*/
			struct regression_params
			{
				ms_regression_options regression_type = ms_regression_options::ridge_regression;
				std::vector<T> lambdas_to_try; // NB: lambda == 0.0 signals dlib to optimise lambda. If the user actually wants lambda = 0.0, then something like lambda = 1.0e-10 should be used
				static const unsigned version;

				/*
					friend function for serialization
				*/
				friend void serialize(const regression_params& rp, std::ostream& out)
				{
					dlib::serialize(modular_solver<T, D>::training_params::regression_params::version, out);
					serialize(rp.regression_type, out);
					dlib::serialize(rp.lambdas_to_try, out);
				}

				/*
					friend function for deserialization
				*/
				friend void deserialize(regression_params& rp, std::istream& in)
				{
					unsigned cur_version;
					dlib::deserialize(cur_version, in);

					// current version deserialization
					// IMPORTANT: please try and make class serialization / deserialization back-compatible
					// by supporting multiple versions if possible, and if not, function should throw an exception
					if (cur_version == 1u)
					{
						throw serialization_error("modular_solver::regression_params version 1 incompatible with version " + std::to_string(modular_solver<T, D>::training_params::regression_params::version));
					}
					else if (cur_version == 2u)
					{
						deserialize(rp.regression_type, in);
						dlib::deserialize(rp.lambdas_to_try, in);
					}
					else
					{
						// back-compatibility code for previous versions should go here
						throw serialization_version_error("modular_solver::training_params::regression_params", modular_solver<T, D>::training_params::regression_params::version, cur_version);
					}
				}
			};

			/*
				kernel parameters to try during training
			*/
			struct kernel_params
			{
				ms_kernel_options kernel_type;
				static const unsigned version;

				/*
					friend function for serialization
				*/
				friend void serialize(const kernel_params& kp, std::ostream& out)
				{
					dlib::serialize(modular_solver<T, D>::training_params::kernel_params::version, out);
					serialize(kp.kernel_type, out);
				}

				/*
					friend function for deserialization
				*/
				friend void deserialize(kernel_params& kp, std::istream& in)
				{
					using dlib::deserialize;
					unsigned cur_version;
					deserialize(cur_version, in);

					// current version deserialization
					// IMPORTANT: please try and make class serialization / deserialization back-compatible
					// by supporting multiple versions if possible, and if not, function should throw an exception
					if (cur_version == 1u)
					{
						throw serialization_error("modular_solver::kernel_params version 1 incompatible with version " + std::to_string(modular_solver<T, D>::training_params::kernel_params::version));
					}
					else if (cur_version == 2u)
					{
						deserialize(kp.kernel_type, in);
					}
					else
					{
						// back-compatibility code for previous versions should go here
						throw serialization_version_error("modular_solver::training_params::kernel_params", modular_solver<T, D>::training_params::kernel_params::version, cur_version);
					}
				}
			};

			/*
				feature selection parameters to try during training

				- REQUIRES
					- 0.0 < feature_fractions_to_try <= 1.0
			*/
			struct feature_selection_params
			{
				std::vector<T> feature_fractions_to_try;
				static const unsigned version;

				/*
					friend function for serialization
				*/
				friend void serialize(const feature_selection_params& fsp, std::ostream& out)
				{
					dlib::serialize(modular_solver<T, D>::training_params::feature_selection_params::version, out);
					dlib::serialize(fsp.feature_fractions_to_try, out);
				}

				/*
					friend function for deserialization
				*/
				friend void deserialize(feature_selection_params& fsp, std::istream& in)
				{
					unsigned cur_version;
					dlib::deserialize(cur_version, in);

					// current version deserialization
					// IMPORTANT: please try and make class serialization / deserialization back-compatible
					// by supporting multiple versions if possible, and if not, function should throw an exception
					if (cur_version == 1u)
					{
						throw serialization_error("modular_solver::feature_selection_params version 1 incompatible with version " + std::to_string(modular_solver<T, D>::training_params::feature_selection_params::version));
					}
					else if (cur_version == 2u)
					{
						dlib::deserialize(fsp.feature_fractions_to_try, in);
					}
					else
					{
						// back-compatibility code for previous versions should go here
						throw serialization_version_error("modular_solver::training_params::feature_selection_params", modular_solver<T, D>::training_params::feature_selection_params::version, cur_version);
					}
				}
			};

			/*
				number of imputation loops to take

				- REQUIRES
					- num_iterations >= 0
			*/
			struct imputation_params
			{
				unsigned num_iterations;
				static const unsigned version;

				/*
					friend function for serialization
				*/
				friend void serialize(const imputation_params& ip, std::ostream& out)
				{
					dlib::serialize(training_params::imputation_params::version, out);
					dlib::serialize(ip.num_iterations, out);
				}

				/*
					friend function for deserialization
				*/
				friend void deserialize(imputation_params& ip, std::istream& in)
				{
					unsigned cur_version;
					dlib::deserialize(cur_version, in);

					// current version deserialization
					// IMPORTANT: please try and make class serialization / deserialization back-compatible
					// by supporting multiple versions if possible, and if not, function should throw an exception
					if (cur_version == 1u)
					{
						throw serialization_error("modular_solver::imputation_params version 1 incompatible with version " + std::to_string(modular_solver<T, D>::training_params::imputation_params::version));
					}
					else if (cur_version == 2u)
					{
						dlib::deserialize(ip.num_iterations, in);
					}
					else
					{
						// back-compatibility code for previous versions should go here
						throw serialization_version_error("modular_solver::training_params::imputation_params", modular_solver<T, D>::training_params::imputation_params::version, cur_version);
					}
				}
			};

			ms_cross_validation_metric metric;
			compression_params compression;
			regression_params regression;
			kernel_params kernel;
			feature_selection_params feature_selection;
			imputation_params imputation;
			unsigned num_threads = 1;
			static const unsigned version;

			training_params()
			{
				metric = ms_cross_validation_metric::MAX_NUMBER_OF_ms_cross_validation_metric;
				compression.input_variance_to_try = { 1.0 };
				compression.input_max_modes_to_try = { 0 };
				regression.regression_type = ms_regression_options::MAX_NUMBER_OF_ms_regression_options;
				regression.lambdas_to_try = {};
				kernel.kernel_type = ms_kernel_options::MAX_NUMBER_OF_ms_kernel_options;
				feature_selection.feature_fractions_to_try = { 1.0 };
				imputation.num_iterations = 0;
			}

			/*
				friend function for serialization
			*/
			friend void serialize(const training_params& tp, std::ostream& out)
			{
				using dlib::serialize;
				dlib::serialize(training_params::version, out);
				serialize(tp.metric, out);
				serialize(tp.compression, out);
				serialize(tp.regression, out);
				serialize(tp.kernel, out);
				serialize(tp.feature_selection, out);
				serialize(tp.imputation, out);
				serialize(tp.num_threads, out);
			}

			/*
				friend function for deserialization
			*/
			friend void deserialize(training_params& tp, std::istream& in)
			{
				using dlib::deserialize;
				unsigned cur_version;
				dlib::deserialize(cur_version, in);

				// current version deserialization
				// IMPORTANT: please try and make class serialization / deserialization back-compatible
				// by supporting multiple versions if possible, and if not, function should throw an exception
				if (cur_version == 4u)
				{
					deserialize(tp.metric, in);
					deserialize(tp.compression, in);
					deserialize(tp.regression, in);
					deserialize(tp.kernel, in);
					deserialize(tp.feature_selection, in);
					deserialize(tp.imputation, in);
					deserialize(tp.num_threads, in);
				}
				else if (cur_version == 3u)
				{
					throw serialization_error("modular_solver::training_params version 3 incompatible with version " + std::to_string(modular_solver<T, D>::training_params::version));
				}
				else if (cur_version == 1u)
				{
					throw serialization_error("modular_solver::training_params version 1 incompatible with version " + std::to_string(modular_solver<T, D>::training_params::version));
				}
				else if (cur_version == 2u)
				{
					throw serialization_error("modular_solver::training_params version 2 incompatible with version " + std::to_string(modular_solver<T, D>::training_params::version));
				}
				else
				{
					// back-compatibility code for previous versions should go here
					throw serialization_version_error("modular_solver::training_params", modular_solver<T, D>::training_params::version, cur_version);
				}
			}
		};

		/*
			optimal params struct

			- WHAT THIS OBJECT REPRESENTS
				- This object represents a container for the choice of input params that produces the lowest (according to the specified metric) cross-validation error per regressor
		*/
		struct optimal_params
		{
			std::vector<optimal_regressor_params<T>> regressor_params;
			std::vector<std::vector<optimal_regressor_params<T>>> imputation_regressor_params;
			training_params orig_training_params;

			optimal_params()
			{
				regressor_params = {};
				imputation_regressor_params = {};
			}

			/*
				friend function for serialization
			*/
			friend void serialize(const optimal_params& op, std::ostream& out)
			{
				using dlib::serialize;
				dlib::serialize(modular_solver<T, D>::optimal_params::version, out);
				serialize(op.regressor_params, out);
				serialize(op.imputation_regressor_params, out);
				serialize(op.orig_training_params, out);
			}

			/*
				friend function for deserialization
			*/
			friend void deserialize(optimal_params& op, std::istream& in)
			{
				using dlib::deserialize;
				unsigned cur_version;
				dlib::deserialize(cur_version, in);

				// current version deserialization
				// IMPORTANT: please try and make class serialization / deserialization back-compatible
				// by supporting multiple versions if possible, and if not, function should throw an exception
				if (cur_version == 1u)
				{
					throw serialization_error("modular_solver::optimal_params version 1 incompatible with version " + std::to_string(modular_solver<T, D>::optimal_params::version));
				}
				else if (cur_version == 2u)
				{
					deserialize(op.regressor_params, in);
					deserialize(op.imputation_regressor_params, in);
					deserialize(op.orig_training_params, in);
				}
				else
				{
					// back-compatibility code for previous versions should go here
					throw serialization_version_error("modular_solver::optimal_params", modular_solver<T, D>::optimal_params::version, cur_version);
				}
			}
		private:
			static const unsigned version;
		};

		/*
			struct containing data specific to each shape; namely expected point IDs, base shape, and alignment method
		*/
		struct shape_data
		{
			std::vector<std::vector<std::string>> views_point_ids;
			std::vector<std::vector<P>> views_base_shape;
			pca_model<T> shape_pca;
			alignmentD_type alignment = alignmentD_type::none;

			/*
				friend function for deserialization
			*/
			friend void deserialize(shape_data& vd, std::istream& in)
			{
				unsigned cur_version;
				dlib::deserialize(cur_version, in);

				// current version deserialization
				// IMPORTANT: please try and make class serialization / deserialization back-compatible
				// by supporting multiple versions if possible, and if not, function should throw an exception
				if (cur_version == 1u)
				{
					throw serialization_error("modular_solver::shape_data version 1 incompatible with version " + std::to_string(modular_solver<T, D>::shape_data::version));
				}
				else if (cur_version == 2u)
				{
					throw serialization_error("modular_solver::shape_data version 2 incompatible with version " + std::to_string(modular_solver<T, D>::shape_data::version));
				}
				else if (cur_version == 3u)
				{
					dlib::deserialize(vd.views_point_ids, in);
					dlib::deserialize(vd.views_base_shape, in);
					deserialize(vd.shape_pca, in);
					deserialize(vd.alignment, in);
				}
				else
				{
					// back-compatibility code for previous versions should go here
					throw serialization_version_error("modular_solver::shape_data", modular_solver<T, D>::shape_data::version, cur_version);
				}
			}

			/*
				friend function for serialization
			*/
			friend void serialize(const shape_data& vd, std::ostream& out)
			{
				dlib::serialize(modular_solver<T, D>::shape_data::version, out);
				dlib::serialize(vd.views_point_ids, out);
				dlib::serialize(vd.views_base_shape, out);
				serialize(vd.shape_pca, out);
				serialize(vd.alignment, out);
			}


			size_t get_num_inputs() const
			{
				return D * std::accumulate(views_point_ids.begin(), views_point_ids.end(), 0, [](size_t sum, const std::vector<std::string>& point_ids) {return sum + point_ids.size(); });
			}

			size_t get_num_outputs() const
			{
				if (shape_pca.is_trained())
				{
					return shape_pca.n_params();
				}
				else
				{
					return get_num_inputs();
				}
			}

			col_vector<T> concatenate_views_and_points(const std::vector<std::vector<P>>& views_points) const
			{
				col_vector<T> concat(static_cast<long>(get_num_inputs()));
				for (unsigned dim = 0, ordinate = 0; dim < D; ++dim)
				{
					for (unsigned view = 0; view < views_point_ids.size(); ++view)
					{
						for (unsigned point = 0; point < views_points[view].size(); ++point, ++ordinate)
						{
							concat(ordinate) = views_points[view][point](dim);
						}
					}
				}
				return concat;
			}

			col_vector<T> process_frame(const std::vector<std::vector<P>>& aligned_views_points) const
			{
				auto concat = concatenate_views_and_points(aligned_views_points);
				if (shape_pca.is_trained())
				{
					return shape_pca.parameterize(concat);
				}
				else
				{
					return concat;
				}
			}
		private:
			static const unsigned version;
		};

	private:
		// regressors and associated feature selection indices
		std::vector<prediction_model_wrapper<T>> regressors_;

		// alignment/training data neutral shape
		std::vector<shape_data> shapes_data_;

		// final training params following cross-validation
		optimal_params cross_validated_params_;

		// post-processing output constraint PCA
		pca_model<T> output_constraint_pca_;

		// control names and allowed ranges/default values
		std::map<std::string, control_range> control_ranges_;
		unsigned n_var_output_;
		unsigned num_views_;
		bool is_trained_;

		// MD5s of training tracking examples, training control examples, and training parameters
		std::string tracking_examples_md5_;
		std::string control_examples_md5_;
		std::string training_parameters_md5_;
		std::string control_ranges_md5_;

		static const unsigned version;

	public:

		/*
			modular_solver constructor

			- Ensures
				- this object is properly initialized
				- is_trained = false
		*/
		modular_solver()
		{
			COMPILE_TIME_ASSERT(is_float_type<T>::value);
			COMPILE_TIME_ASSERT(D == 2 || D == 3); //support solving from 2d tracking or 3d tracking - 1d (control-to-control) solving support to come in future
			is_trained_ = false;
			n_var_output_ = 0;
			num_views_ = 0;
		}

		/*
			returns true if the solver is trained, false otherwise
		*/
		bool is_trained() const
		{
			return is_trained_;
		}

		/*
			returns number of control values predicted per frame
		*/
		const unsigned& get_n_outputs() const
		{
			return n_var_output_;
		}

		/*
			modular solver version getter
		*/
		static const unsigned& get_version()
		{
			return version;
		}

		/*
			returns number of shapes
		*/
		unsigned get_n_shapes() const
		{
			return static_cast<unsigned>(shapes_data_.size());
		}

		/*
			returns the final cross-validated regressor training parameters
		*/
		const optimal_params& get_final_trained_params() const
		{
			return cross_validated_params_;
		}

		/*
			returns the point IDs associated with the requested shape component
		*/
		const std::vector<std::string>& get_shape_view_point_ids(const unsigned& shape, const unsigned& view) const
		{
			DLIB_ASSERT(shape >= 0 && shape < shapes_data_.size(),
				"\t modular_solver::get_shape_view_point_ids(const unsigned& shape, const unsigned& view)"
				<< "\n\t Invalid input:"
				<< "\n\t shape must be between 0 and " << (shapes_data_.size() - 1)
				<< "\n\t shape: " << shape);
			DLIB_ASSERT(view >= 0 && view < shapes_data_[shape].views_point_ids.size(),
				"\t modular_solver::get_shape_view_point_ids(const unsigned& shape, const unsigned& view)"
				<< "\n\t shape must be between 0 and " << (shapes_data_[shape].views_point_ids.size() - 1)
				<< "\n\t view: " << view);
			return shapes_data_[shape].views_point_ids[view];
		}


		/*
			returns the alignment associated with the requested shape component
		*/
		alignmentD_type get_shape_alignment(const unsigned& shape) const
		{
			DLIB_ASSERT(shape >= 0 && shape < shapes_data_.size(),
				"\t modular_solver::get_shape_alignment(const unsigned& shape, const unsigned& view)"
				<< "\n\t Invalid input:"
				<< "\n\t shape must be between 0 and " << (shapes_data_.size() - 1)
				<< "\n\t shape: " << shape);
			return shapes_data_[shape].alignment;
		}

		/*
			returns the expected number of dimensions in each tracking point
		*/
		static int get_n_dimensions()
		{
			return D;
		}

		unsigned get_n_views() const
		{
			return num_views_;
		}

		/*
			returns the MD5 hash of the training tracking examples
		*/
		const std::string& get_tracking_examples_md5() const
		{
			return tracking_examples_md5_;
		}

		/*
			returns the MD5 hash of the training control examples
		*/
		const std::string& get_control_examples_md5() const
		{
			return control_examples_md5_;
		}

		/*
			returns the MD5 hash of the training parameters
		*/
		const std::string& get_training_parameters_md5() const
		{
			return training_parameters_md5_;
		}

		/*
			returns the MD5 hash of the control ranges
		*/
		const std::string& get_control_ranges_md5() const
		{
			return control_ranges_md5_;
		}

		/*
			returns the control names and ranges this solver was trained with
		*/
		const std::map<std::string, control_range>& get_control_ranges() const
		{
			return control_ranges_;
		}

		/*
			friend function for serialization
		*/
		friend void serialize(const modular_solver<T, D>& item, std::ostream& out)
		{
			using dlib::serialize;
			// Output current class version number
			// IMPORTANT: if you add to or change the class member variables, please up the version number
			dlib::serialize(modular_solver<T, D>::version, out);

			serialize(item.regressors_, out);
			serialize(item.shapes_data_, out);
			serialize(item.cross_validated_params_, out);
			serialize(item.output_constraint_pca_, out);
			serialize(item.control_ranges_, out);
			dlib::serialize(item.n_var_output_, out);
			dlib::serialize(item.is_trained_, out);
			dlib::serialize(item.tracking_examples_md5_, out);
			dlib::serialize(item.control_examples_md5_, out);
			dlib::serialize(item.training_parameters_md5_, out);
			dlib::serialize(item.control_ranges_md5_, out);
			dlib::serialize(item.num_views_, out);
		}

		/*
			friend function for deserialization
		*/
		friend void deserialize(modular_solver<T, D>& item, std::istream& in)
		{
			using dlib::deserialize;
			unsigned cur_version;
			dlib::deserialize(cur_version, in);

			// current version deserialization
			// IMPORTANT: please try and make class serialization / deserialization back-compatible
			// by supporting multiple versions if possible, and if not, function should throw an exception
			if (cur_version == 1u)
			{
				throw serialization_error("modular_solver version 1 incompatible with version " + std::to_string(modular_solver<T, D>::version));
			}
			else if (cur_version == 2u)
			{
				throw serialization_error("modular_solver version 2 incompatible with version " + std::to_string(modular_solver<T, D>::version));
			}
			else if (cur_version == 3u)
			{
				using dlib::deserialize;
				deserialize(item.regressors_, in);
				deserialize(item.shapes_data_, in);
				deserialize(item.cross_validated_params_, in);
				deserialize(item.output_constraint_pca_, in);
				deserialize(item.control_ranges_, in);
				dlib::deserialize(item.n_var_output_, in);
				dlib::deserialize(item.is_trained_, in);
				dlib::deserialize(item.tracking_examples_md5_, in);
				dlib::deserialize(item.control_examples_md5_, in);
				dlib::deserialize(item.training_parameters_md5_, in);
				dlib::deserialize(item.control_ranges_md5_, in);
				dlib::deserialize(item.num_views_, in);
			}
			else
			{
				// back-compatibility code for previous versions should go here
				throw serialization_version_error("modular_solver", modular_solver<T, D>::version, cur_version);
			}
		}

		/*
			Predict rig control values for given tracking data, a shot neutral frame, and output constraint, using trained regressors

			- Requires
				- modular_solver::is_trained_ == true
				- views_tracking_data.size() == modular_solver::shapes_data_.size()
				- every view's tracking data contains only and all of the expected point IDs (i.e. those used during training)

			- Ensures
				- input is aligned (if applicable)
				- input is neutral corrected (if applicable)
				- input is feature selected (if applicable)
				- output_constraint is applied (if applicable)
				- output is clamped according to control ranges supplied during training
				- returns map of control ID to control value
		*/
		std::map<std::string, T> solve_frame(const std::vector<std::map<std::string, P>>& views_tracking_data,
			const std::vector<std::map<std::string, P>>& unaligned_views_pose_neutral,
			const T& output_constraint) const
		{
			DLIB_ASSERT(is_trained_, "Modular solver must be trained before solving frames.");

			check_input_data(views_tracking_data);

			std::vector<std::vector<std::vector<P>>> input(shapes_data_.size(), std::vector<std::vector<P>>(num_views_));
			for (unsigned view = 0; view < num_views_; ++view)
			{
				for (unsigned shape = 0; shape < shapes_data_.size(); ++shape)
				{
					input[shape][view] = map_to_vector(views_tracking_data[view], shapes_data_[shape].views_point_ids[view]);
					align(input[shape][view], shapes_data_[shape].views_base_shape[view], shapes_data_[shape].alignment);
				}
			}

			std::vector<std::vector<std::vector<P>>> aligned_views_shapes_shot_neutral, aligned_views_shapes_pose_neutral;
			align_neutral(unaligned_views_pose_neutral, aligned_views_shapes_pose_neutral);
			apply_neutral(input, aligned_views_shapes_shot_neutral, aligned_views_shapes_pose_neutral);

			auto output = predict_controls(input);

			auto constrained_output = apply_output_constraint(output, output_constraint);

			std::map<std::string, T> result;
			int c = 0;
			for (auto it = control_ranges_.begin(); it != control_ranges_.end(); ++it, ++c)
			{
				result[it->first] = constrained_output(c);
			}
			return result;
		}

		/*!
			Predict rig control values for every frame of a shot, given tracking data and runtime params, using trained regressors

			- Requires
				- modular_solver::is_trained_ == true
				- views_tracking_data.size() == modular_solver::shapes_data_.size()
				- every view's tracking data contains only and all of the expected point IDs (i.e. those used during training)

			- Ensures
				- input is aligned (if applicable)
				- input is neutral corrected (if applicable)
				- input is feature selected (if applicable)
				- output_constraint is applied (if applicable)
				- output is clamped according to control ranges supplied during training
				- output is smoothed (if applicable)
				- output is cleaned (if applicable)
				- output has key-frame-reduction applied (if applicable)
				- returns map of control ID to vector of control value at each frame
		*/
		std::map<std::string, std::map<int, T>> solve_shot(const std::vector<std::vector<std::map<std::string, P>>>& frames_views_tracking_data, const runtime_params& params) const
		{
			DLIB_ASSERT(is_trained_, "Modular solver must be trained before solving shots.");

			const unsigned num_frames = static_cast<unsigned>(frames_views_tracking_data.size());
			const unsigned num_controls = static_cast<unsigned>(control_ranges_.size());
			verify_params(params, num_frames);
			std::vector<std::map<std::string, P>> views_shot_neutral;
			if (params.shot_neutral_correction_frame != -1)
			{
				views_shot_neutral = frames_views_tracking_data[params.shot_neutral_correction_frame];
			}
			std::vector<std::vector<std::vector<P>>> aligned_views_shapes_shot_neutral, aligned_views_shapes_pose_neutral;
			align_neutral(views_shot_neutral, aligned_views_shapes_shot_neutral);
			align_neutral(params.reference_neutral_pose, aligned_views_shapes_pose_neutral);

			std::vector<std::vector<T>> frames_controls_predictions(num_frames, std::vector<T>(num_controls));

			for (unsigned frame = 0; frame < num_frames; ++frame)
			{
				check_input_data(frames_views_tracking_data[frame]);

				std::vector<std::vector<std::vector<P>>> aligned_shapes_views(shapes_data_.size(), std::vector<std::vector<P>>(num_views_));
				for (unsigned view = 0; view < num_views_; ++view)
				{
					for (unsigned shape = 0; shape < shapes_data_.size(); ++shape)
					{
						aligned_shapes_views[shape][view] = map_to_vector(frames_views_tracking_data[frame][view], shapes_data_[shape].views_point_ids[view]);
						align(aligned_shapes_views[shape][view], shapes_data_[shape].views_base_shape[view], shapes_data_[shape].alignment);
					}
				}

				apply_neutral(aligned_shapes_views, aligned_views_shapes_shot_neutral, aligned_views_shapes_pose_neutral);
				frames_controls_predictions[frame] = predict_controls(aligned_shapes_views);
			}
			return post_process(frames_controls_predictions, params.post_processing);
		}

		/*
			friend class for producing a trained modular_solver object
		*/
		template <class T2, int D2> friend class modular_solver_trainer;

	private:
#ifdef ENABLE_ASSERTS
		void verify_params(const runtime_params& params, const int& num_frames) const
#else
		void verify_params(const runtime_params& , const int& ) const
#endif
		{
			DLIB_ASSERT(params.shot_neutral_correction_frame > -2 && params.shot_neutral_correction_frame < num_frames,
				"\t modular_solver::verify_params(const runtime_params& params, const int& num_frames)"
				<< "\n\t Invalid solve parameter:"
				<< "\n\t shot_neutral_correction_frame must be bounded between -2 and num_frames exclusive"
				<< "\n\t shot_neutral_correction_frame: " << params.shot_neutral_correction_frame
				<< "\n\t num_frames: " << num_frames);
			DLIB_ASSERT(params.post_processing.output_constraint >= 0.0 && params.post_processing.output_constraint <= 1.0,
				"\t modular_solver::verify_params(const runtime_params& params, const int& num_frames)"
				<< "\n\t Invalid solve parameter:"
				<< "\n\t output_constraint must be bounded between 0.0 and 1.0 inclusive"
				<< "\n\t output_constraint: " << params.post_processing.output_constraint);
			DLIB_ASSERT(params.post_processing.key_frame_reduction >= 0.0 && params.post_processing.key_frame_reduction <= 1.0,
				"\t modular_solver::verify_params(const runtime_params& params, const int& num_frames)"
				<< "\n\t Invalid solve parameter:"
				<< "\n\t key_frame_reduction must be bounded between 0.0 and 1.0 inclusive"
				<< "\n\t key_frame_reduction: " << params.post_processing.key_frame_reduction);
			DLIB_ASSERT(params.post_processing.cutoff >= 0.0 && params.post_processing.cutoff <= 1.0,
				"\t modular_solver::verify_params(const runtime_params& params, const int& num_frames)"
				<< "\n\t Invalid solve parameters:"
				<< "\n\t cutoff must be bounded between 0.0 and 1.0 inclusive"
				<< "\n\t cutoff: " << params.post_processing.cutoff);
			DLIB_ASSERT(params.post_processing.cleaning >= 0.0 && params.post_processing.cleaning <= 1.0,
				"\t modular_solver::verify_params(const runtime_params& params, const int& num_frames)"
				<< "\n\t Invalid solve parameters:"
				<< "\n\t cleaning must be bounded between 0.0 and 1.0 inclusive"
				<< "\n\t cleaning: " << params.post_processing.cleaning);
			DLIB_ASSERT(params.post_processing.smoothing_level >= 0,
				"\t modular_solver::verify_params(const runtime_params& params, const int& num_frames)"
				<< "\n\t Invalid solve parameters:"
				<< "\n\t smoothing_level must be greater than or equal to 0"
				<< "\n\t smoothing_level: " << params.post_processing.smoothing_level);
#ifdef ENABLE_ASSERTS
			int prev = -1;
#endif
			DLIB_ASSERT(std::all_of(params.post_processing.cut_starts.begin(), params.post_processing.cut_starts.end(), [&num_frames, &prev](std::vector<int>::const_reference ref)
			{bool pass = ref > prev && ref <= num_frames; prev = ref; return pass; }),
				"\t modular_solver::verify_params(const runtime_params& params, const int& num_frames)"
				<< "\n\t Invalid solve parameters:"
				<< "\n\t every element of cut_starts must be bounded between 0 and num_frames inclusive, and cut_starts must be strictly monotonic increasing");
		}

		void check_input_data(const std::vector<std::map<std::string, P>>& views_tracking) const
		{
			CARBON_SUPPRESS_UNUSED(views_tracking);
			DLIB_ASSERT(views_tracking.size() == num_views_,
				"\t modular_solver::check_input_data(const std::vector<std::map<std::string, P>>& views_tracking)"
				<< "\n\t Invalid inputs:"
				<< "\n\t Input data must have the expected number of views (" << num_views_ << ")"
				<< "\n\t views_tracking.size(): " << views_tracking.size());

			for (unsigned shape = 0; shape < shapes_data_.size(); ++shape)
			{
				for (unsigned view = 0; view < num_views_; ++view)
				{
					DLIB_ASSERT(std::all_of(shapes_data_[shape].views_point_ids[view].begin(), shapes_data_[shape].views_point_ids[view].end(), [&](std::vector<std::string>::const_reference id) {return views_tracking[view].find(id) != views_tracking[view].end(); }),
						"\t modular_solver::check_input_data(const std::vector<std::vector<std::map<std::string, P>>>& views_shapes)"
						<< "\n\t Invalid inputs:"
						<< "n\t Each view must contain all expected points.");
				}
			}
		}

		col_vector<T> concatenate_shapes(const std::vector<std::vector<std::vector<P>>>& aligned_shapes_views) const
		{
			col_vector<T> ret(std::accumulate(shapes_data_.begin(), shapes_data_.end(), 0, [](size_t sum, const shape_data& shape_model) {return sum + shape_model.get_num_outputs(); }));
			for (unsigned shape = 0, shape_ordinate = 0; shape < shapes_data_.size(); ++shape)
			{
				unsigned num_rows = static_cast<unsigned>(shapes_data_[shape].get_num_outputs());
				dlib::set_subm(ret, shape_ordinate, 0, num_rows, 1) = shapes_data_[shape].process_frame(aligned_shapes_views[shape]);
				shape_ordinate += num_rows;
			}
			return ret;
		}

		void align(std::vector<dlib::vector<T, 2>>& points, const std::vector<dlib::vector<T, 2>>& base_shape, const alignment_type& alignment) const
		{
			auto ptp = find_transform_as_projective(points, base_shape, alignment);
			for (auto& point : points)
			{
				point = ptp(point);
			}
		}

		void align(std::vector<dlib::vector<T, 3>>& points, const std::vector<dlib::vector<T, 3>>& base_shape, const alignment3d_type& alignment) const
		{
			auto ptp = find_transform_as_affine3d(points, base_shape, alignment);
			for (auto& point : points)
			{
				point = ptp(point);
			}
		}

		void align_neutral(const std::vector<std::map<std::string, P>>& views_neutral, std::vector<std::vector<std::vector<P>>>& aligned_views_shapes_neutral) const
		{
			if (views_neutral.size() != 0)
			{
				aligned_views_shapes_neutral.assign(num_views_, std::vector<std::vector<P>>(shapes_data_.size()));
				check_input_data(views_neutral);
				for (unsigned shape = 0; shape < shapes_data_.size(); ++shape)
				{
					for (unsigned view = 0; view < num_views_; ++view)
					{
						aligned_views_shapes_neutral[view][shape] = map_to_vector(views_neutral[view], shapes_data_[shape].views_point_ids[view]);
						align(aligned_views_shapes_neutral[view][shape], shapes_data_[shape].views_base_shape[view], shapes_data_[shape].alignment);
					}
				}
			}
		}

		void apply_neutral(std::vector<std::vector<std::vector<P>>>& aligned_shapes_views, const std::vector<std::vector<std::vector<P>>>& aligned_corrective_views_shapes_neutral, const std::vector<std::vector<std::vector<P>>>& aligned_reference_views_shapes_neutral) const
		{
			if (aligned_corrective_views_shapes_neutral.size() != 0)
			{
				for (unsigned shape = 0; shape < shapes_data_.size(); ++shape)
				{
					for (unsigned view = 0; view < num_views_; ++view)
					{
						if (aligned_corrective_views_shapes_neutral.size() != 0 && aligned_reference_views_shapes_neutral.size() != 0)
						{
							for (unsigned point = 0; point < shapes_data_[shape].views_point_ids[view].size(); ++point)
							{
								aligned_shapes_views[shape][view][point] -= aligned_corrective_views_shapes_neutral[view][shape][point] - aligned_reference_views_shapes_neutral[view][shape][point];
							}
						}
					}
				}
			}
		}

		std::vector<T> predict_controls(const std::vector<std::vector<std::vector<P>>>& aligned_shapes_views) const
		{
			col_vector<T> input1D = concatenate_shapes(aligned_shapes_views);
			std::vector<T> output(regressors_.size());
			unsigned r = 0;
			for (auto control_range_it = control_ranges_.begin(); control_range_it != control_ranges_.end(); ++control_range_it, ++r)
			{
				const std::vector<int>& feature_selection_indices = cross_validated_params_.imputation_regressor_params.size() == 0
					? cross_validated_params_.regressor_params[r].selected_feature_fraction.second
					: cross_validated_params_.imputation_regressor_params[cross_validated_params_.imputation_regressor_params.size() - 1][r].selected_feature_fraction.second;
				// apply feature selection to input (this just makes a copy of the input if this modular solver was trained without feature selection)
				col_vector<T> feature_selected_input(static_cast<unsigned>(feature_selection_indices.size()));
				for (unsigned f = 0; f < feature_selection_indices.size(); ++f)
				{
					feature_selected_input(f) = input1D(static_cast<unsigned>(feature_selection_indices[f]));
				}
				// predict the r'th element of the output
				output[r] = std::min(std::max(regressors_[r].predict(feature_selected_input), control_range_it->second.min), control_range_it->second.max);
			}
			return output;
		}

		col_vector<T> apply_output_constraint(const std::vector<T>& predicted_controls, const T& output_constraint) const
		{
			col_vector<T> constrained_predictions = mat(predicted_controls);
			if (output_constraint > 0.0)
			{
				constrained_predictions *= (1.0 - output_constraint);
				constrained_predictions += output_constraint * output_constraint_pca_.reconstruct(output_constraint_pca_.parameterize(constrained_predictions));
			}
			return constrained_predictions;
		}

		std::map<std::string, std::map<int, T>> post_process(const std::vector<std::vector<T>>& frames_controls_predictions, const typename runtime_params::post_processing_params& ppp) const
		{
			const unsigned num_controls = static_cast<unsigned>(control_ranges_.size());
			const int num_frames = static_cast<int>(frames_controls_predictions.size());

			std::map<std::string, std::map<int, T>> output_qsa;
			std::vector<int> cut_starts = ppp.cut_starts;
			if (ppp.cut_starts.size() == 0)
			{
				cut_starts = { 0, num_frames };
			}
			else
			{
				if (ppp.cut_starts[0] != 0)
				{
					cut_starts.insert(cut_starts.begin(), 0);
				}
				if (ppp.cut_starts[ppp.cut_starts.size() - 1] != num_frames)
				{
					cut_starts.insert(cut_starts.end(), num_frames);
				}
			}
			for (std::size_t cuts_index = 0; cuts_index < cut_starts.size() - 1; ++cuts_index)
			{
				const auto diff = cut_starts[cuts_index + 1] - cut_starts[cuts_index];
				const std::size_t cut_frames = static_cast<std::size_t>(diff);
				std::vector<std::vector<T>> controls_frames(num_controls, std::vector<T>(cut_frames));
				for (std::size_t frame = 0; frame < cut_frames; ++frame)
				{
					auto output_constrained = apply_output_constraint(frames_controls_predictions[static_cast<std::size_t>(ppp.cut_starts[cuts_index]) + frame], ppp.output_constraint);
					for (std::size_t control = 0; control < num_controls; ++control)
					{
						controls_frames[control][frame] = output_constrained(static_cast<unsigned>(control));
					}
				}
				int control = 0;
				for (auto control_range_it = control_ranges_.begin(); control_range_it != control_ranges_.end(); ++control_range_it, control++)
				{
					controls_frames[control] = savitzky_golay_filter<T>(controls_frames[control], ppp.smoothing_level + 1, ppp.smoothing_level + 1, 2);
					T range = control_range_it->second.max - control_range_it->second.min;
					T diff_limit = range * ppp.cutoff;
					if (control_range_it->second.def - control_range_it->second.min < ppp.cleaning * range || control_range_it->second.max - control_range_it->second.def < ppp.cleaning * range)
					{
						for (std::size_t frame = 0; frame < cut_frames; ++frame)
						{
							T control_diff_prev = frame == 0 ? 0.0 : std::abs(controls_frames[control][frame - 1] - control_range_it->second.def);
							T control_diff = std::abs(controls_frames[control][frame] - control_range_it->second.def);
							T control_diff_next = frame == cut_frames - 1 ? 0.0 : std::abs(controls_frames[control][static_cast<size_t>(frame) + 1] - control_range_it->second.def);
							if (control_diff_prev < diff_limit && control_diff < diff_limit && control_diff_next < diff_limit)
							{
								controls_frames[control][frame] = control_range_it->second.def;
							}
						}
					}
					T tolerance = range * ppp.key_frame_reduction;
					auto key_frames = dense_data_to_key_frames<T>(controls_frames[control], tolerance);
					for (auto key_frame_it = key_frames.begin(); key_frame_it != key_frames.end(); ++key_frame_it)
					{
						output_qsa[control_range_it->first][cut_starts[cuts_index] + key_frame_it->first] = std::min(std::max(key_frame_it->second, control_range_it->second.min), control_range_it->second.max);
					}
				}
			}
			return output_qsa;
		}
	};

	template <typename T, int D>
	const unsigned modular_solver<T, D>::version = 3;
	template <typename T, int D>
	const unsigned modular_solver<T, D>::control_range::version = 2;
	template <typename T, int D>
	const unsigned modular_solver<T, D>::shape_data::version = 3;
	template <typename T, int D>
	const unsigned modular_solver<T, D>::optimal_params::version = 2;
	template <typename T, int D>
	const unsigned modular_solver<T, D>::training_params::version = 4;
	template <typename T, int D>
	const unsigned modular_solver<T, D>::training_params::compression_params::version = 3;
	template <typename T, int D>
	const unsigned modular_solver<T, D>::training_params::regression_params::version = 2;
	template <typename T, int D>
	const unsigned modular_solver<T, D>::training_params::kernel_params::version = 2;
	template <typename T, int D>
	const unsigned modular_solver<T, D>::training_params::feature_selection_params::version = 2;
	template <typename T, int D>
	const unsigned modular_solver<T, D>::training_params::imputation_params::version = 2;
}