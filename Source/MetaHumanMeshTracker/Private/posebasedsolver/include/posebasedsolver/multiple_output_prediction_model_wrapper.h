// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "learning.h"
#include "prediction_model.h"
#include "prediction_model_factory.h"

namespace cm
{
    using namespace dlib;


	/*!
	 Multiple output prediction model wrapper class.

	 - REQUIREMENTS ON T
		 - Must be either float, double, or long double, ie:
		   is_float_type<T>::value == true

	 - INITIAL VALUE
		 - n_variables() == 0
		 - is_trained() = false

	 - WHAT THIS OBJECT REPRESENTS
		 - This object represents a general prediction (regression) model which predicts multiple outputs
		 The prediction model may optionally have a PCA basis applied to the input data.

	*/
	template <typename T>
	class multiple_output_prediction_model_wrapper
	{
        template<typename U>
        friend class multiple_output_prediction_model_wrapper;

	public:

		static const int version;


		/*
		 multiple_output_prediction_model_wrapper constructor

		   - Ensures
			   - is_trained() = false
		 */
		multiple_output_prediction_model_wrapper()
		{
			COMPILE_TIME_ASSERT(is_float_type<T>::value);
		}

		/*
		 multiple_output_prediction_model_wrapper construct from a prediction_model_base
		*/
		multiple_output_prediction_model_wrapper(const prediction_model_base<T> & pred)
		{
			prediction_model_ptr_ = std::unique_ptr< prediction_model_base<T> >(pred.clone());
		}

		/*
		 multiple_output_prediction_model_wrapper default destructor
		*/
		~multiple_output_prediction_model_wrapper() = default;

		/*
		multiple_output_prediction_model_wrapper copy constructor
		*/
        multiple_output_prediction_model_wrapper(const multiple_output_prediction_model_wrapper& other)
        {
            this->operator=(other);
        }

        /*
        multiple_output_prediction_model_wrapper copy constructor for type conversion
        */
        template<typename U>
        multiple_output_prediction_model_wrapper(const multiple_output_prediction_model_wrapper<U>& other)
		{
            COMPILE_TIME_ASSERT((std::is_convertible<U, T>::value));
            this->operator=<U>(other);
		}

        /*
        multiple_output_prediction_model_wrapper assignment operator for instances of the same type
        */
        multiple_output_prediction_model_wrapper& operator=(const multiple_output_prediction_model_wrapper& other)
        {
            if (this != &other)
            {
                prediction_model_ptr_.reset(other.prediction_model_ptr_->clone());
            }

            return *this;
        }

        /*
        multiple_output_prediction_model_wrapper assignment operator for type conversion from double to float
        */
        template<typename U, typename std::enable_if<std::is_same<float, T>::value && std::is_same<double, U>::value>::type * = nullptr>
        multiple_output_prediction_model_wrapper& operator=(const multiple_output_prediction_model_wrapper<U>& other)
        {
            prediction_model_ptr_.reset(other.prediction_model_ptr_->clone_as_float());
            return *this;
        }

        /*
        multiple_output_prediction_model_wrapper assignment operator for type conversion from float to double
        */
        template<typename U, typename std::enable_if<std::is_same<double, T>::value && std::is_same<float, U>::value>::type * = nullptr>
        multiple_output_prediction_model_wrapper& operator=(const multiple_output_prediction_model_wrapper<U>& other)
        {
            prediction_model_ptr_.reset(other.prediction_model_ptr_->clone_as_double());
            return *this;
        }

		/*
			default move copy constructor
		*/
		multiple_output_prediction_model_wrapper(multiple_output_prediction_model_wrapper &&) = default;

		/*
			default move assignment operator
		*/
		multiple_output_prediction_model_wrapper & operator= (multiple_output_prediction_model_wrapper &&) = default;


		/*!
		- Ensures
			- Returns the number of outputs of the prediction model
		!*/
		int n_outputs() const
		{
			return prediction_model_ptr_->n_outputs();
		}


		/*!
		 Predict the output

		 - Requires
			- input.nr() == n_variables()
			 - n_variables() > 0 (i.e. the predictor is trained)
			 - pca_limit >= 0

		 - Ensures
			 - Predict the outputs

            input:
                A reference to the input column vector used for prediction.
            pca_limit:
                Limit value used to parametrize the input vector (see cm::pca_model<T>::parametrize(...))
            max_pca_components_to_predict:
                Integer value specifying the maximal number of output pca modes to predict.
		*/
		matrix<T, 0, 1> predict(const matrix<T, 0, 1 >& input, T pca_limit = 99, const int max_pca_components_to_predict = -1) const
		{
			return prediction_model_ptr_->predict(input, pca_limit, max_pca_components_to_predict);
		}



		/*!
		- Ensures
			- Returns the number of input variables required for the prediction model
		!*/
		int n_variables() const
		{
			return prediction_model_ptr_->n_variables();
		}

		/*!
		- Ensures
			- Return the estimated mean square error for each predictor output (from cross-validation)
		*/
		const std::vector<double> & estimated_mses() const
		{
			return prediction_model_ptr_->estimated_mses();
		}

		/*!
		- Ensures
			- Return the final gamma multipler for each predictor output (NB only applicable for predictors which have a gamma parameter
			eg SVR, KRR)
		*/
		const std::vector<double> & final_gamma_multipliers() const
		{
			return prediction_model_ptr_->final_gamma_multipliers();
		}

		/*!
		- Ensures
			- Return the final c for the predictor output (NB only applicable for predictors which have a c parameter
			eg SVR, KRR)
		*/
		const std::vector<double>& final_cs() const
		{
			return prediction_model_ptr_->final_cs();
		}

		/*!
		- Ensures
			- Return the final number of basis functions used for each predictor output (NB only applicable for predictors which have
			basis functions eg SVR, KRR)
		*/
		const std::vector<int>& final_n_basis_functions() const
		{
			return prediction_model_ptr_->final_n_basis_functions();
		}

		// friend for construction of a single output prediction model
		template <typename T2> friend class prediction_model_wrapper;

		// friend for incremental training.
		template <typename T2> friend class multiple_output_prediction_model_wrapper_trainer;


		// Friend functions for serialization
		friend void serialize(const multiple_output_prediction_model_wrapper<T>& item, std::ostream& out)
		{
			// Output current class version number
			// IMPORTANT: if you add to or change the class member variables, please up the version number
			dlib::serialize(multiple_output_prediction_model_wrapper<T>::version, out);

			serialize(item.prediction_model_ptr_->name(), out);
			item.prediction_model_ptr_->serialize_class(out);
		}

		friend void deserialize(multiple_output_prediction_model_wrapper<T> & item, std::istream& in)
		{
			unsigned cur_version;
			dlib::deserialize(cur_version, in);

			// current version deserialization
			// IMPORTANT: please try and make class serialization / deserialization back-compatible
			// by supporting multiple versions if possible, and if not, function should throw an exception
			if (cur_version == 4)
			{
				std::string name;
				dlib::deserialize(name, in);
				item.prediction_model_ptr_ = std::unique_ptr<prediction_model_base<T>>(prediction_model_factory<T>::create(name));
				item.prediction_model_ptr_->deserialize_class(in);
			}
			else if (cur_version == 3 || cur_version == 2 || cur_version == 1)
			{
				// legacy back-compatibility before the re-factor
				int n_variables;
				std::vector<prediction_model_method> trained_methods;
				pca_model<T> output_pca;
				pca_model<T> pca_for_krr_linear;
				pca_model<T> pca_for_krr_rbf;
				pca_model<T> pca_for_krr_poly;
				std::vector<linear_predictor<T>> krr_linear_predictors;
				std::vector<rbf_predictor<T, 0>> krr_rbf_predictors;
				std::vector < rbf_predictor<T, 16>> krr_rbf_predictors_16;
				std::vector<double> estimated_mses = {};
				std::vector<double> final_gamma_multipliers = {};
				std::vector<double> final_cs = {};
				std::vector<int> final_n_basis_functions = {};

				
				dlib::deserialize(n_variables, in);
				dlib::deserialize(trained_methods, in);
				deserialize(output_pca, in);
				deserialize(pca_for_krr_linear, in);
				deserialize(pca_for_krr_rbf, in);
				deserialize(pca_for_krr_poly, in);
				deserialize(krr_linear_predictors, in);
				deserialize(krr_rbf_predictors, in);

				if (cur_version >= 2)
				{
					deserialize(krr_rbf_predictors_16, in);
				}

				if (cur_version == 3)
				{
					dlib::deserialize(estimated_mses, in);
					dlib::deserialize(final_gamma_multipliers, in);
					dlib::deserialize(final_cs, in);
					dlib::deserialize(final_n_basis_functions, in);
				}


				// now convert to the right type. Note we are only supporting types we know to be in the field, but others can be added if needed
				switch (trained_methods[0])
				{
				case prediction_model_method::pca_krr_rbf:
				{
					std::vector < rbf_predictor<T, 0 >> dummy_predictors;
					item.prediction_model_ptr_ = std::unique_ptr<prediction_model_base<T>>(prediction_model_factory<T>::create("prediction_model" + std::string(typeid(dummy_predictors).name())));
					auto * concrete_prediction_model_ptr = dynamic_cast<prediction_model<T, rbf_predictor<T, 0 >> *>(item.prediction_model_ptr_.get());
					concrete_prediction_model_ptr->set_contents_legacy(n_variables, pca_for_krr_rbf, output_pca, krr_rbf_predictors,
						estimated_mses, final_gamma_multipliers, final_cs, final_n_basis_functions);
					break;
				}
				case prediction_model_method::pca_krr_rbf_16:
				{
					std::vector < rbf_predictor<T, 16> > dummy_predictors;
					item.prediction_model_ptr_ = std::unique_ptr<prediction_model_base<T>>(prediction_model_factory<T>::create("prediction_model" + std::string(typeid(dummy_predictors).name())));
					auto * concrete_prediction_model_ptr = dynamic_cast<prediction_model<T, rbf_predictor<T, 16 >> *>(item.prediction_model_ptr_.get());
					concrete_prediction_model_ptr->set_contents_legacy(n_variables, pca_for_krr_rbf, output_pca, krr_rbf_predictors_16,
						estimated_mses, final_gamma_multipliers, final_cs, final_n_basis_functions);
					break;
				}
				default:
					throw std::runtime_error("multiple_output_prediction_model_wrapper legacy deserialization error, unsupported trained method");
				}
			}
			else
			{
				throw serialization_version_error("multiple_output_prediction_model_wrapper", multiple_output_prediction_model_wrapper<T>::version, cur_version);
			}

		}



	private:

		std::unique_ptr<prediction_model_base<T>> prediction_model_ptr_;
	};

	template <typename T>
	const int  multiple_output_prediction_model_wrapper<T>::version = 4;



}



