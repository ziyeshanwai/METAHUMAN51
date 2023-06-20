// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <rlibv/basic_types.h>
#include <type_traits>
#include <cmath>

#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/svm.h>
#include <dlib/global_optimization.h>
#include <dlib/matrix.h>
RLIBV_RENABLE_WARNINGS

namespace rlibv
{
	template <class regressor_type>
	class predictor
	{
	private:
		typedef typename regressor_type regressor_type;
		typedef typename regressor_type::T T;
		typedef typename regressor_type::kernel_type kernel_type;
		dlib::vector_normalizer<col_vector<T>> normalizer;
		dlib::decision_function<typename regressor_type::kernel_type::dlib_kernel> function;
		T training_error;
		typename regressor_type::training_params trained_regressor_params;
		typename kernel_type::training_params trained_kernel_params;
	public:
		predictor(const dlib::vector_normalizer<col_vector<T>>& normalizer_,
			const dlib::decision_function<typename regressor_type::kernel_type::dlib_kernel>& function_,
			const typename regressor_type::training_params& regressor_trained_params_,
			const typename kernel_type::training_params& kernel_trained_params_,
			const T& training_error_
		) : normalizer(normalizer_), function(function_), trained_regressor_params(regressor_trained_params_), trained_kernel_params(kernel_trained_params_), training_error(training_error_)
		{}
		T predict(const col_vector<T>& input) const
		{
			return function(normalizer(input));
		}
		const typename regressor_type::training_params& get_trained_regressor_params() const
		{
			return trained_regressor_params;
		}
		const typename kernel_type::training_params& get_trained_kernel_params() const
		{
			return trained_kernel_params;
		}
		const T& get_training_error() const
		{
			return training_error;
		}
	};

	template <typename T>
	class regressor_trainer
	{
	private:
		static const size_t max_cross_validation_folds;
		static const int num_threads;
	public:
		struct regressor_error : public dlib::error
		{
			regressor_error(const std::string& message) : dlib::error(message) {}
		};
		enum class cross_validation_metric
		{
			sum_absolute_max,
			sum_absolute_mean,
			sum_square_max,
			sum_square_mean,
			covariance_correlation
		};
		template <class regressor_type>
		static predictor<regressor_type> train_regressor(const std::vector<col_vector<T>>& input_examples,
			const std::vector<T>& input_targets,
			const std::string& random_seed,
			const typename regressor_type::training_params& regressor_training_params,
			const typename regressor_type::kernel_type::training_params& kernel_training_params,
			const T& training_error = std::numeric_limits<T>::max())
		{
			if (input_examples.size() != input_targets.size())
			{
				throw regressor_error("Bad input data - input_examples and output_examples should contain the same number of examples.");
			}

			return regressor_type::train(input_examples, input_targets, random_seed, regressor_training_params, kernel_training_params, training_error);
		}

		template <class regressor_type>
		static predictor<regressor_type> train_regressor_cross_validation(const std::vector<col_vector<T>>& input_examples,
			const std::vector<T>& input_targets,
			const std::string& random_seed,
			const typename regressor_type::cross_validation_training_params& regressor_cv_params,
			const typename regressor_type::kernel_type::cross_validation_training_params& kernel_cv_params,
			cross_validation_metric metric,
			std::vector<T>& residuals)
		{
			if (input_examples.size() != input_targets.size())
			{
				throw regressor_error("Bad input data - input_examples and output_examples should contain the same number of examples.");
			}
			T best_cv_error = std::numeric_limits<T>::max();
			typename regressor_type::training_params cross_validated_regressor_params;
			typename regressor_type::kernel_type::training_params cross_validated_kernel_params;
			regressor_type::iterate_regressor_params(input_examples, input_targets, random_seed, regressor_cv_params, kernel_cv_params, metric, std::min(max_cross_validation_folds, input_examples.size()), cross_validated_regressor_params, cross_validated_kernel_params, best_cv_error);
			predictor<regressor_type> predictor = train_regressor<regressor_type>(input_examples, input_targets, random_seed, cross_validated_regressor_params, cross_validated_kernel_params, best_cv_error);
			residuals.resize(input_examples.size());
			for (int i = 0; i < input_examples.size(); ++i)
			{
				residuals[i] = predictor.predict(input_examples[i]) - input_targets[i];
			}
			return predictor;
		}

		template <class regressor_type>
		static predictor<regressor_type> train_regressor_find_min_global(const std::vector<col_vector<T>>& input_examples,
			const std::vector<T>& input_targets,
			const std::string& random_seed,
			const cross_validation_metric metric,
			const T& optimisation_tolerance,
			const typename regressor_type::training_params& min_regressor_training_params,
			const typename regressor_type::training_params& max_regressor_training_params,
			const typename regressor_type::kernel_type::training_params& min_kernel_training_params,
			const typename regressor_type::kernel_type::training_params& max_kernel_training_params,
			std::vector<T>& residuals)
		{
			if (input_examples.size() != input_targets.size())
			{
				throw regressor_error("Bad input data - input_examples and output_examples should contain the same number of examples.");
			}
			col_vector<T> lower_bound(regressor_type::num_regressor_params + regressor_type::kernel_type::num_kernel_params);
			col_vector<T> upper_bound(regressor_type::num_regressor_params + regressor_type::kernel_type::num_kernel_params);
			std::vector<bool> is_integer_param(regressor_type::num_regressor_params + regressor_type::kernel_type::num_kernel_params);

			regressor_type::package_parameters(lower_bound, upper_bound, min_regressor_training_params, max_regressor_training_params, is_integer_param);
			regressor_type::kernel_type::template package_parameters<regressor_type>(lower_bound, upper_bound, min_kernel_training_params, max_kernel_training_params, is_integer_param);

			dlib::thread_pool tp(num_threads);
			dlib::max_function_calls num_calls = dlib::max_function_calls(5000);

			auto find_min_global_metric = [&](const col_vector<T>& params)
			{
				typename regressor_type::training_params regressor_params(dlib::subm(params, 0, 0, regressor_type::num_regressor_params, 1));
				typename regressor_type::kernel_type::training_params kernel_params(dlib::subm(params, regressor_type::num_regressor_params, 0, regressor_type::kernel_type::num_kernel_params, 1));
				return regressor_trainer<T>::cross_validate<regressor_type>(input_examples, input_targets, random_seed, regressor_params, kernel_params, std::min(max_cross_validation_folds, input_examples.size()), metric);
			};

			auto result = dlib::find_min_global(tp, find_min_global_metric, lower_bound, upper_bound, num_calls, optimisation_tolerance);

			typename regressor_type::training_params optimised_regressor_params = regressor_type::training_params(dlib::subm(result.x, 0, 0, regressor_type::num_regressor_params, 1));
			typename regressor_type::kernel_type::training_params optimised_kernel_params = regressor_type::kernel_type::training_params(dlib::subm(result.x, regressor_type::num_regressor_params, 0, regressor_type::kernel_type::num_kernel_params, 1));
			
			predictor<regressor_type> predictor = train_regressor<regressor_type>(input_examples, input_targets, random_seed, optimised_regressor_params, optimised_kernel_params, result.y);
			residuals.resize(input_examples.size());
			for (int i = 0; i < input_examples.size(); ++i)
			{
				residuals[i] = predictor.predict(input_examples[i]) - input_targets[i];
			}
			return predictor;
		}

		template <class kernel_type>
		class kernel_ridge_regression
		{
		public:
			typedef kernel_type kernel_type;
			typedef typename T T;
			struct training_params
			{
				int max_basis_functions;
				T lambda;
				training_params() = default;
				training_params(const col_vector<T>& params) : max_basis_functions(params(0)), lambda(params(1)) {}
			};
			struct cross_validation_training_params
			{
				std::vector<int> max_basis_functions_to_try;
				std::vector<T> lambdas_to_try;
			};

			template <typename T2> friend class regressor_trainer;
		private:
			static const int num_regressor_params;

			static predictor<kernel_ridge_regression> train(const std::vector<col_vector<T>>& input_examples,
				const std::vector<T>& input_targets,
				const std::string& random_seed,
				const training_params& regressor_params,
				const typename kernel_type::training_params& kernel_params,
				const T& training_error)
			{
				auto examples(input_examples);
				auto targets(input_targets);
				if (random_seed.empty())
				{
					dlib::randomize_samples(examples, targets);
				}
				else
				{
					auto rnd = dlib::rand(random_seed);
					dlib::randomize_samples(examples, targets, rnd);
				}

				dlib::vector_normalizer<col_vector<T>> normalizer;
				normalizer.train(examples);
				for (auto& example : examples)
				{
					example = normalizer(example);
				}

				dlib::krr_trainer<typename kernel_type::dlib_kernel> final_trainer;
				final_trainer.set_kernel(kernel_type::get_kernel(kernel_params));
				final_trainer.set_max_basis_size(regressor_params.max_basis_functions);
				final_trainer.set_lambda(regressor_params.lambda);

				return predictor<kernel_ridge_regression>(normalizer, final_trainer.train(examples, targets), regressor_params, kernel_params, training_error);
			}

			static void iterate_regressor_params(const std::vector<col_vector<T>>& input_examples,
				const std::vector<T>& input_targets,
				const std::string& random_seed,
				const cross_validation_training_params& regressor_training_params,
				const typename kernel_type::cross_validation_training_params& kernel_training_params,
				const cross_validation_metric metric,
				const int num_folds,
				typename training_params& cross_validated_regressor_params,
				typename kernel_type::training_params& cross_validated_kernel_params,
				T& cross_validation_error)
			{
				if (regressor_training_params.max_basis_functions_to_try.size() == 0 || regressor_training_params.lambdas_to_try.size() == 0)
				{
					throw regressor_error("Every regressor parameter must have at least one value to try for cross-validation.");
				}
				training_params this_regressor_params;
				for (const auto& mbf : regressor_training_params.max_basis_functions_to_try)
				{
					this_regressor_params.max_basis_functions = mbf;
					for (const auto& l : regressor_training_params.lambdas_to_try)
					{
						this_regressor_params.lambda = l;
						kernel_type::template iterate_kernel_params<kernel_ridge_regression>(input_examples, input_targets, random_seed, this_regressor_params, kernel_training_params, num_folds, metric, cross_validated_regressor_params, cross_validated_kernel_params, cross_validation_error);
					}
				}
			}

			static void package_parameters(col_vector<T>& lower_bound, col_vector<T>& upper_bound, const training_params& min_training_params, const training_params& max_training_params, std::vector<bool>& is_integer_param)
			{
				lower_bound(0) = min_training_params.max_basis_functions;
				lower_bound(1) = min_training_params.lambda;
				upper_bound(0) = max_training_params.max_basis_functions;
				upper_bound(1) = max_training_params.lambda;
				is_integer_param[0] = true;
				is_integer_param[1] = false;
			}

		};

		 template <class kernel_type>
		 class support_vector_regression
		 {
		 public:
			 typedef kernel_type kernel_type;
			 typedef typename T T;
			 struct training_params
			 {
				 T c;
				 T epsilon_insensitivity;
				 training_params() = default;
				 training_params(const col_vector<T>& params) : c(params(0)), epsilon_insensitivity(params(1)) {}
			 };
			 struct cross_validation_training_params
			 {
				 std::vector<T> c_to_try;
				 std::vector<T> epsilon_to_try;
			 };

			 template <typename T2> friend class regressor_trainer;
		 private:
			 static const int num_regressor_params;

			 static predictor<support_vector_regression> train(const std::vector<col_vector<T>>& input_examples,
				 const std::vector<T>& input_targets,
				 const std::string& random_seed,
				 const training_params& regressor_params,
				 const typename kernel_type::training_params& kernel_params,
				 const T& training_error)
			 {
				 auto examples(input_examples);
				 auto targets(input_targets);
				 if (random_seed.empty())
				 {
					 dlib::randomize_samples(examples, targets);
				 }
				 else
				 {
					 auto rnd = dlib::rand(random_seed);
					 dlib::randomize_samples(examples, targets, rnd);
				 }

				 dlib::vector_normalizer<col_vector<T>> normalizer;
				 normalizer.train(examples);
				 for (auto& example : examples)
				 {
					 example = normalizer(example);
				 }

				 dlib::svr_trainer<typename kernel_type::dlib_kernel> final_trainer;
				 final_trainer.set_kernel(kernel_type::get_kernel(kernel_params));
				 final_trainer.set_c(regressor_params.c);
				 final_trainer.set_epsilon_insensitivity(regressor_params.epsilon_insensitivity);

				 return predictor<support_vector_regression>(normalizer, final_trainer.train(examples, targets), regressor_params, kernel_params, training_error);
			 }

			 static void iterate_regressor_params(const std::vector<col_vector<T>>& input_examples,
				 const std::vector<T>& input_targets,
				 const std::string& random_seed,
				 const cross_validation_training_params& regressor_training_params,
				 const typename kernel_type::cross_validation_training_params& kernel_training_params,
				 const cross_validation_metric metric,
				 const int num_folds,
				 typename training_params& cross_validated_regressor_params,
				 typename kernel_type::training_params& cross_validated_kernel_params,
				 T& cross_validation_error)
			 {
				 if (regressor_training_params.c_to_try.size() == 0 || regressor_training_params.epsilon_to_try.size() == 0)
				 {
					 throw regressor_error("Every regressor parameter must have at least one value to try for cross-validation.");
				 }
				 training_params this_regressor_params;
				 for (const auto& c : regressor_training_params.c_to_try)
				 {
					 this_regressor_params.c = c;
					 for (const auto& e : regressor_training_params.epsilon_to_try)
					 {
						 this_regressor_params.epsilon_insensitivity = e;
						 kernel_type::template iterate_kernel_params<support_vector_regression>(input_examples, input_targets, random_seed, this_regressor_params, kernel_training_params, num_folds, metric, cross_validated_regressor_params, cross_validated_kernel_params, cross_validation_error);
					 }
				 }
			 }

			 static void package_parameters(col_vector<T>& lower_bound, col_vector<T>& upper_bound, const training_params& min_training_params, const training_params& max_training_params, std::vector<bool>& is_integer_param)
			 {
				 lower_bound(0) = min_training_params.c;
				 lower_bound(1) = min_training_params.epsilon_insensitivity;
				 upper_bound(0) = max_training_params.c;
				 upper_bound(1) = max_training_params.epsilon_insensitivity;
				 is_integer_param[0] = false;
				 is_integer_param[1] = false;
			 }
		 };

		class linear_kernel
		{
		public:
			typedef typename dlib::linear_kernel<col_vector<T>> dlib_kernel;
			struct training_params
			{
				training_params() = default;
				training_params(const col_vector<T>& params) {}
			};
			struct cross_validation_training_params
			{
			};
			static dlib_kernel get_kernel(const training_params& params)
			{
				return dlib_kernel();
			}

			template <typename T2> friend class regressor_trainer;
		private:
			static const int num_kernel_params;

			template <class regressor_type>
			static void iterate_kernel_params(const std::vector<col_vector<T>>& input_examples,
				const std::vector<T>& input_targets,
				const std::string& random_seed,
				const typename regressor_type::training_params& regressor_training_params,
				const cross_validation_training_params& kernel_cv_params,
				const int num_folds,
				const cross_validation_metric metric,
				typename regressor_type::training_params& cross_validated_regressor_params,
				training_params& cross_validated_kernel_params,
				T& best_cv_error)
			{
				training_params kernel_training_params;
				T cv_error = regressor_trainer<T>::cross_validate<regressor_type>(input_examples, input_targets, random_seed, regressor_training_params, kernel_training_params, num_folds, metric);
				if (cv_error < best_cv_error)
				{
					cross_validated_regressor_params = regressor_training_params;
					cross_validated_kernel_params = kernel_training_params;
					best_cv_error = cv_error;
				}
			}

			template <class regressor_type>
			static void package_parameters(col_vector<T>& lower_bound, col_vector<T>& upper_bound, const training_params& min_training_params, const training_params& max_training_params, std::vector<bool>& is_integer_param)
			{
			}
		};

		class polynomial_kernel
		{
		public:
			typedef typename dlib::polynomial_kernel<col_vector<T>> dlib_kernel;
			struct training_params
			{
				T gamma;
				T coeff;
				T degree;
				training_params() = default;
				training_params(const col_vector<T>& params) : gamma(params(0)), coeff(params(1)), degree(params(2)) {}
			};
			struct cross_validation_training_params
			{
				std::vector<T> gamma_to_try;
				std::vector<T> coeff_to_try;
				std::vector<T> degree_to_try;
			};
			template <typename T2> friend class regressor_trainer;
		private:
			static const int num_kernel_params;

			static dlib_kernel get_kernel(const training_params& params)
			{
				return dlib_kernel(params.gamma, params.coeff, params.degree);
			}

			template <class regressor_type>
			static void iterate_kernel_params(const std::vector<col_vector<T>>& input_examples,
				const std::vector<T>& input_targets,
				const std::string& random_seed,
				const typename regressor_type::training_params& regressor_training_params,
				const cross_validation_training_params& kernel_cv_params,
				const int num_folds,
				const cross_validation_metric metric,
				typename regressor_type::training_params& cross_validated_regressor_params,
				training_params& cross_validated_kernel_params,
				T& best_cv_error)
			{
				if (kernel_cv_params.gamma_to_try.size() == 0 || kernel_cv_params.coeff_to_try.size() == 0 || kernel_cv_params.degree_to_try.size() == 0)
				{
					throw regressor_error("Every kernel parameter must have at least one value to try for cross-validation.");
				}
				training_params kernel_training_params;
				for (const auto& g : kernel_cv_params.gamma_to_try)
				{
					kernel_training_params.gamma = g;
					for (const auto& c : kernel_cv_params.coeff_to_try)
					{
						kernel_training_params.coeff = c;
						for (const auto& d : kernel_cv_params.degree_to_try)
						{
							kernel_training_params.degree = d;

							T cv_error = regressor_trainer<T>::cross_validate<regressor_type>(input_examples, input_targets, random_seed, regressor_training_params, kernel_training_params, num_folds, metric);
							if (cv_error < best_cv_error)
							{
								cross_validated_regressor_params = regressor_training_params;
								cross_validated_kernel_params = kernel_training_params;
								best_cv_error = cv_error;
							}
						}
					}
				}
			}

			template <class regressor_type>
			static void package_parameters(col_vector<T>& lower_bound, col_vector<T>& upper_bound, const training_params& min_training_params, const training_params& max_training_params, std::vector<bool>& is_integer_param)
			{
				lower_bound(regressor_type::num_regressor_params) = min_training_params.gamma;
				lower_bound(regressor_type::num_regressor_params + 1) = min_training_params.coeff;
				lower_bound(regressor_type::num_regressor_params + 2) = min_training_params.degree;
				upper_bound(regressor_type::num_regressor_params) = max_training_params.gamma;
				upper_bound(regressor_type::num_regressor_params + 1) = max_training_params.coeff;
				upper_bound(regressor_type::num_regressor_params + 2) = max_training_params.degree;
				is_integer_param[regressor_type::num_regressor_params] = false;
				is_integer_param[regressor_type::num_regressor_params + 1] = false;
				is_integer_param[regressor_type::num_regressor_params + 2] = false;
			}
		};

		class radial_basis_kernel
		{
		public:
			typedef typename dlib::radial_basis_kernel<col_vector<T>> dlib_kernel;
			struct training_params
			{
				T gamma;
				training_params() = default;
				training_params(const col_vector<T>& params) : gamma(params(0)) {}
			};
			struct cross_validation_training_params
			{
				std::vector<T> gamma_to_try;
			};
			template <typename T2> friend class regressor_trainer;
		private:
			static const int num_kernel_params;

			static dlib_kernel get_kernel(const training_params& params)
			{
				return dlib_kernel(params.gamma);
			}

			template <class regressor_type>
			static void iterate_kernel_params(const std::vector<col_vector<T>>& input_examples,
				const std::vector<T>& input_targets,
				const std::string& random_seed,
				const typename regressor_type::training_params& regressor_training_params,
				const cross_validation_training_params& kernel_cv_params,
				const int num_folds,
				const cross_validation_metric metric,
				typename regressor_type::training_params& cross_validated_regressor_params,
				training_params& cross_validated_kernel_params,
				T& best_cv_error)
			{
				if (kernel_cv_params.gamma_to_try.size() == 0)
				{
					throw regressor_error("Every kernel parameter must have at least one value to try for cross-validation.");
				}
				training_params kernel_training_params;
				for (const auto& g : kernel_cv_params.gamma_to_try)
				{
					kernel_training_params.gamma = g;

					T cv_error = regressor_trainer<T>::cross_validate<regressor_type>(input_examples, input_targets, random_seed, regressor_training_params, kernel_training_params, num_folds, metric);
					if (cv_error < best_cv_error)
					{
						cross_validated_regressor_params = regressor_training_params;
						cross_validated_kernel_params = kernel_training_params;
						best_cv_error = cv_error;
					}
				}
			}

			template <class regressor_type>
			static void package_parameters(col_vector<T>& lower_bound, col_vector<T>& upper_bound, const training_params& min_training_params, const training_params& max_training_params, std::vector<bool>& is_integer_param)
			{
				lower_bound(regressor_type::num_regressor_params) = min_training_params.gamma;
				upper_bound(regressor_type::num_regressor_params) = max_training_params.gamma;
				is_integer_param[regressor_type::num_regressor_params] = false;
			}
		};

		class sigmoid_kernel
		{
		public:
			typedef typename dlib::sigmoid_kernel<col_vector<T>> dlib_kernel;
			struct training_params
			{
				T gamma;
				T coeff;
				training_params() = default;
				training_params(const col_vector<T>& params) : gamma(params(0)), coeff(params(1)) {}
			};
			struct cross_validation_training_params
			{
				std::vector<T> gamma_to_try;
				std::vector<T> coeff_to_try;
			};
			template <typename T2> friend class regressor_trainer;
		private:
			static const int num_kernel_params;

			static dlib_kernel get_kernel(const training_params& params)
			{
				return dlib_kernel(params.gamma, params.coeff);
			}

			template <class regressor_type>
			static void iterate_kernel_params(const std::vector<col_vector<T>>& input_examples,
				const std::vector<T>& input_targets,
				const std::string& random_seed,
				const typename regressor_type::training_params& regressor_training_params,
				const cross_validation_training_params& kernel_cv_params,
				const int num_folds,
				const cross_validation_metric metric,
				typename regressor_type::training_params& cross_validated_regressor_params,
				training_params& cross_validated_kernel_params,
				T& best_cv_error)
			{
				if (kernel_cv_params.gamma_to_try.size() == 0 || kernel_cv_params.coeff_to_try.size() == 0)
				{
					throw regressor_error("Every kernel parameter must have at least one value to try for cross-validation.");
				}
				training_params kernel_training_params;
				for (const auto& g : kernel_cv_params.gamma_to_try)
				{
					kernel_training_params.gamma = g;
					for (const auto& c : kernel_cv_params.coeff_to_try)
					{
						kernel_training_params.coeff = c;
						T cv_error = regressor_trainer<T>::cross_validate<regressor_type>(input_examples, input_targets, random_seed, regressor_training_params, kernel_training_params, num_folds, metric);
						if (cv_error < best_cv_error)
						{
							cross_validated_regressor_params = regressor_training_params;
							cross_validated_kernel_params = kernel_training_params;
							best_cv_error = cv_error;
						}
					}
				}
			}

			template <class regressor_type>
			static void package_parameters(col_vector<T>& lower_bound, col_vector<T>& upper_bound, const training_params& min_training_params, const training_params& max_training_params, std::vector<bool>& is_integer_param)
			{
				lower_bound(regressor_type::num_regressor_params) = min_training_params.gamma;
				lower_bound(regressor_type::num_regressor_params + 1) = min_training_params.coeff;
				upper_bound(regressor_type::num_regressor_params) = max_training_params.gamma;
				upper_bound(regressor_type::num_regressor_params + 1) = max_training_params.coeff;
				is_integer_param[regressor_type::num_regressor_params] = false;
				is_integer_param[regressor_type::num_regressor_params + 1] = false;
			}
		};
	private:
		template <class regressor_type>
		static typename T cross_validate(const std::vector<col_vector<T>>& input_examples,
			const std::vector<T>& input_targets,
			const std::string& random_seed,
			const typename regressor_type::training_params& regressor_training_params,
			const typename regressor_type::kernel_type::training_params& kernel_training_params,
			const size_t num_folds,
			cross_validation_metric metric)
		{
			const int num_examples = input_examples.size();
			const int num_ordinates = input_examples.begin()->size();
			const int chunk = num_examples / num_folds;

			dlib::running_stats<T> rs_abs;
			dlib::running_stats<T> rs_sq;
			dlib::running_scalar_covariance<T> rc;
			for (int fold = 0; fold < num_folds; ++fold)
			{
				const int test_start_index = fold * chunk;
				const int test_end_index = fold == num_folds - 1 ? num_examples : (fold + 1) * chunk;
				const int num_test_examples = test_end_index - test_start_index;
				std::vector<col_vector<T>> fold_train_examples(num_examples - num_test_examples, col_vector<T>(num_ordinates));
				std::vector<T> fold_train_targets(num_examples - num_test_examples);
				std::vector<col_vector<T>> fold_test_examples(num_test_examples, col_vector<T>(num_ordinates));
				std::vector<T> fold_test_targets(num_test_examples);

				// apply randomization to examples + targets
				std::vector<int> random_indices(num_examples);
				std::iota(random_indices.begin(), random_indices.end(), 0);
				dlib::rand rng(random_seed);
				randomize_samples(random_indices, rng);
				for (int i = 0; i < test_start_index; ++i)
				{
					fold_train_examples[i] = input_examples[random_indices[i]];
					fold_train_targets[i] = input_targets[random_indices[i]];
				}
				for (int i = test_start_index; i < test_end_index; ++i)
				{
					fold_test_examples[i - test_start_index] = input_examples[random_indices[i]];
					fold_test_targets[i - test_start_index] = input_targets[random_indices[i]];
				}
				for (int i = test_end_index; i < num_examples; ++i)
				{
					fold_train_examples[i - num_test_examples] = input_examples[random_indices[i]];
					fold_train_targets[i - num_test_examples] = input_targets[random_indices[i]];
				}
				
				predictor<regressor_type> predictor = regressor_trainer<T>::train_regressor<regressor_type>(fold_train_examples, fold_train_targets, random_seed, regressor_training_params, kernel_training_params);
				for (int i = 0; i < num_test_examples; ++i)
				{
					T result = predictor.predict(fold_test_examples[i]);

					T diff = result - fold_test_targets[i];
					rs_abs.add(std::abs(diff));
					rs_sq.add(diff * diff);
					rc.add(result, fold_test_targets[i]);
				}
			}
			switch (metric)
			{
			case cross_validation_metric::sum_square_max:
				return rs_sq.max();
			case cross_validation_metric::sum_square_mean:
				return rs_sq.mean();
			case cross_validation_metric::sum_absolute_max:
				return rs_abs.max();
			case cross_validation_metric::sum_absolute_mean:
				return rs_abs.mean();
			case cross_validation_metric::covariance_correlation:
				return rc.correlation();
			default:
				throw regressor_error("Unrecognised cross-validation error metric option.");
			}
		}
	};
	template <typename T>
	const size_t regressor_trainer<T>::max_cross_validation_folds = 4;
	template <typename T>
	const int regressor_trainer<T>::num_threads = 8;
	template <typename T> template <class kernel_type>
	const int regressor_trainer<T>::kernel_ridge_regression<kernel_type>::num_regressor_params = 2;
	template <typename T> template <class kernel_type>
	const int regressor_trainer<T>::support_vector_regression<kernel_type>::num_regressor_params = 2;
	template <typename T>
	const int regressor_trainer<T>::linear_kernel::num_kernel_params = 0;
	template <typename T>
	const int regressor_trainer<T>::polynomial_kernel::num_kernel_params = 3;
	template <typename T>
	const int regressor_trainer<T>::radial_basis_kernel::num_kernel_params = 1;
	template <typename T>
	const int regressor_trainer<T>::sigmoid_kernel::num_kernel_params = 2;
}