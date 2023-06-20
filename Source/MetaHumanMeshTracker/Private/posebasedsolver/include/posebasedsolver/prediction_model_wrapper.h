// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "multiple_output_prediction_model_wrapper.h"
#include "prediction_model_factory.h"

namespace cm
{
    using namespace dlib;


	/*!
	 Prediction model wrapper class.

	 - REQUIREMENTS ON T
		 - Must be either float, double, or long double, ie:
		   is_float_type<T>::value == true

	 - INITIAL VALUE
		 - n_variables() == 0
		 - is_trained() = false

	 - WHAT THIS OBJECT REPRESENTS
		 - This object represents a general prediction (regression) model which predicts a scalar output.
		 The prediction model may optionally have a PCA basis applied to its input data.

	*/
	template <typename T>
	class prediction_model_wrapper
	{

	public:

		static const int version;


		/*
		 prediction_model_wrapper constructor
		 
		   - Ensures
			   - is_trained() = false
		 */
		prediction_model_wrapper()
		{
			COMPILE_TIME_ASSERT(is_float_type<T>::value);
		}

		/*
		 prediction_model_wrapper construct from a prediction_model_base

		 Expects:
			pred.n_outputs() == 1
		*/
		explicit prediction_model_wrapper(const prediction_model_base<T> & pred)
		{
			DLIB_ASSERT(pred.n_outputs() == 1);
			prediction_model_ptr_ = std::unique_ptr< prediction_model_base<T> >(pred.clone());
		}

		/*
		 prediction_model_wrapper construct from a multiple_output_prediction_model_wrapper

		 Expects:
			pred.n_outputs() == 1
		*/
		explicit prediction_model_wrapper(const multiple_output_prediction_model_wrapper<T> & pred)
		{
			DLIB_ASSERT(pred.n_outputs() == 1);
			prediction_model_ptr_ = std::unique_ptr< prediction_model_base<T> >(pred.prediction_model_ptr_->clone());
		}

		/*
		 prediction_model_wrapper default destructor
		*/
		~prediction_model_wrapper() = default;

		/*
		prediction_model_wrapper copy constructor
		*/
		prediction_model_wrapper(const prediction_model_wrapper & other)
		{
			this->operator=(other);
		}

		/*
		prediction_model_wrapper assignment operator
		*/
		prediction_model_wrapper& operator=(const prediction_model_wrapper & other)
		{
			if (this != &other)
			{
				prediction_model_ptr_ = std::unique_ptr<prediction_model_base<T>>(other.prediction_model_ptr_->clone());
			}

			return *this;
		}

		/* 
			default move copy constructor
		*/
		prediction_model_wrapper(prediction_model_wrapper &&) = default;

		/*
			default move assignment operator
		*/
		prediction_model_wrapper & operator= (prediction_model_wrapper &&) = default;


		/*!
		 Predict the output

		 - Requires
			 - input.nr() == n_variables()
			 - n_variables() > 0 (i.e. the predictor is trained)
			 - pca_limit >= 0

		 - Ensures
			 - Predict the output
		*/
		T predict(const matrix<T, 0, 1>& input, T pca_limit = 99) const
		{
			return prediction_model_ptr_->predict({ input }, pca_limit)(0);
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
			- Return the estimated mean square error for the predictor output (from cross-validation)
		*/
		double estimated_mse() const 
		{
			return prediction_model_ptr_->estimated_mses()[0];
		}

		/*!
		- Ensures
			- Return the final gamma multipler for the predictor output (NB only applicable for predictors which have a gamma parameter
			eg SVR, KRR, KRLS)
		*/
		double final_gamma_multiplier() const 
		{
			return prediction_model_ptr_->final_gamma_multipliers()[0];
		}

		/*!
		- Ensures
			- Return the final c for the predictor output (NB only applicable for predictors which have a c parameter
			eg SVR, KRR, KRLS)
		*/
		double final_c() const 
		{
			return prediction_model_ptr_->final_cs()[0];
		}

		/*!
		- Ensures
			- Return the final number of basis functions used for the predictor output (NB only applicable for predictors which have
			basis functions eg SVR, KRR, KRLS)
		*/
		int final_n_basis_functions() const
		{
			return prediction_model_ptr_->final_n_basis_functions()[0];
		}


		// Friend functions for serialization
		friend void serialize(const prediction_model_wrapper<T>& item, std::ostream& out)
		{
			using dlib::serialize;
			// Output current class version number
			// IMPORTANT: if you add to or change the class member variables, please up the version number
			serialize(prediction_model_wrapper<T>::version, out);

			serialize(item.prediction_model_ptr_->name(), out);
			item.prediction_model_ptr_->serialize_class(out);
		}

		friend void deserialize(prediction_model_wrapper<T> & item, std::istream& in)
		{
			using dlib::deserialize;
			unsigned cur_version;
			deserialize(cur_version, in);

			// current version deserialization
			// IMPORTANT: please try and make class serialization / deserialization back-compatible
			// by supporting multiple versions if possible, and if not, function should throw an exception
			if (cur_version == 2)
			{
				std::string name;
				deserialize(name, in);
				item.prediction_model_ptr_ = std::unique_ptr<prediction_model_base<T>>(prediction_model_factory<T>::create(name));
				item.prediction_model_ptr_->deserialize_class(in);
			}
			// NB there is no back-compatibility for version 1 as this should not be needed for any models in the field
			else
			{
				// back-compatibility code for previous versions should go here
				throw serialization_version_error("prediction_model_wrapper", prediction_model_wrapper<T>::version, cur_version);
			}
		}
private:

		std::unique_ptr<prediction_model_base<T>> prediction_model_ptr_;
	};


	template <typename T>
	const int  prediction_model_wrapper<T>::version = 2;


}



