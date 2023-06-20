// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
POSEBASEDSOLVER_RENABLE_WARNINGS

namespace cm
{
    using namespace dlib;

    /*!
        Prediction model base class.
     
        - REQUIREMENTS ON T
	        - Must be either float, double, or long double, ie:
              is_float_type<T>::value == true
    
	    - INITIAL VALUE
            - n_variables() == 0
			- is_trained() = false
    
	    - WHAT THIS OBJECT REPRESENTS 
            - This object represents a general prediction (regression) model which predicts one or more scalar results ie
			multiple outputs.
			The prediction model may optionally have a PCA basis applied to its input data.
  
    */
	template <typename T>
	class prediction_model_base
	{
	public:

	    /**
	     *  prediction_model_base constructor
	     *
		 *   - Ensures
		 *	   - is_trained() = false
		 */
        prediction_model_base
		(
		)
		{
			COMPILE_TIME_ASSERT(is_float_type<T>::value);
		}

	
		/*
		 virtual destructor
		 */
		virtual ~prediction_model_base() {}

		/*
		 Clone the object onto the heap.
		 */
		virtual prediction_model_base * clone() const = 0;

        /*
         Clone the object onto the heap forcing it to be of type float.
         */
        virtual prediction_model_base<float> * clone_as_float() const = 0;

        /*
         Clone the object onto the heap forcing it to be of type double.
         */
        virtual prediction_model_base<double> * clone_as_double() const = 0;

		/*!
		- Ensures
			- Returns the number of input variables required for the prediction model
		!*/
		virtual int n_variables() const = 0;


		/*!
		- Ensures
			- Returns the number of outputs of the prediction model
		!*/
		virtual int n_outputs() const = 0;

        /*!
        - Requires
            - All samples in the input vector must have the same dimensionality as the samples used to train the pca_input_ 
	          model.
        - Ensures
            - if (pca_input_.is_trained())
                - returns a vector v_out such that: v_out[i] = pca_input_.parametrize(input[i]) for all valid i
            - else
                - returns a vector v_out which is a copy of input
        !*/
        virtual std::vector<matrix<T, 0, 1>> project_pca_in(const std::vector<matrix<T, 0, 1>> & input) const = 0;

        /*!
        - Requires
            - All samples in the input vector must have the same dimensionality as the samples used to train the pca_output_
              model.
        - Ensures
            - if (pca_output_.is_trained())
                - returns a vector v_out such that: v_out[i] = pca_output_.parametrize(input[i]) for all valid i
            - else
                - returns a vector v_out which is a copy of input
        !*/
        virtual std::vector<matrix<T, 0, 1>> project_pca_out(const std::vector<matrix<T, 0, 1>> & input) const = 0;

        /*!
		    Predict the outputs
		
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
		virtual matrix<T, 0, 1> predict(const matrix<T, 0, 1>& input, T pca_limit = 99, const int max_pca_components_to_predict = -1) const = 0;

		/*!
			- Ensures
				- Return the estimated mean square error for each predictor output (from cross-validation)
		*/
		virtual const std::vector<double> & estimated_mses() const = 0;

		/*!
		- Ensures
			- Return the final gamma multipler for each predictor output (NB only applicable for predictors which have a gamma parameter
			eg SVR, KRR)
		*/
		virtual const std::vector<double>& final_gamma_multipliers() const = 0;

		/*!
		- Ensures
			- Return the final c for each predictor output (NB only applicable for predictors which have a c parameter
			eg SVR, KRR) 
		*/		
		virtual const std::vector<double> & final_cs() const = 0;

		/*!
		- Ensures
			- Return the final number of basis functions used for each predictor output (NB only applicable for predictors which have
			basis functions eg SVR, KRR)
		*/
		virtual const std::vector<int> & final_n_basis_functions() const = 0;

   
		/*
		- Ensures
			- returns a unique name for the class
		*/
		virtual std::string name() const = 0;

		/*
			virtual method for serialization
		*/
		virtual void serialize_class(std::ostream& out) const = 0;

		/*
			virtual method for deserialization
		*/
		virtual void deserialize_class(std::istream& in) = 0;

	};


}



