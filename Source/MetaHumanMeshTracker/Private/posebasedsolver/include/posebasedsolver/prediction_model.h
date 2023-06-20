// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "learning.h"
#include "pca_model.h"
#include "prediction_model_base.h"

namespace cm
{
    using namespace dlib;

    /*!
        Templated prediction model class which consists of regression with an optional PCA subspace on the input and the output.
     
        - REQUIREMENTS ON T
	        - Must be either float, double, or long double, ie:
              is_float_type<T>::value == true
    
	    - INITIAL VALUE
            - n_variables() == 0
    
	    - WHAT THIS OBJECT REPRESENTS 
            - This object represents a general linear prediction (regression) model which predicts one or more scalar results ie
			multiple outputs.
			The prediction model may optionally have a PCA basis applied to its input and/or output data.
			The class is templated both on the type of input / output T and also on the type of underlying predictor P
  
    */
	template <typename T, typename P >
	class prediction_model : public prediction_model_base<T>
	{
        template<typename T2, typename P2>
        friend class prediction_model;

	public:

		static const int version;


	    /**
	     *  prediction_model constructor
	     *
		 *   - Ensures
		 */
		prediction_model():
			n_variables_(0)
		{
			COMPILE_TIME_ASSERT(is_float_type<T>::value);
		}

        prediction_model(const prediction_model& other) = default;

        template<typename T2, typename P2>
        prediction_model(const prediction_model<T2, P2>& other)
        {
            n_variables_ = other.n_variables_;
            pca_input_ = other.pca_input_;
            pca_output_ = other.pca_output_;
            predictors_.resize(other.predictors_.size());
            for (std::size_t i = 0; i < predictors_.size(); ++i)
            {
                predictors_[i] = other.predictors_[i];
            }
            estimated_mses_ = other.estimated_mses_;
            final_gamma_multipliers_ = other.final_gamma_multipliers_;
            final_cs_ = other.final_cs_;
            final_n_basis_functions_ = other.final_n_basis_functions_;
        }

		/*
		 Clone the object onto the heap.
		 */
		virtual prediction_model_base<T> * clone() const override
		{
			return new prediction_model<T, P>(*this);
		}

        /*
         Clone the object onto the heap forcing it to be of type float.
         */
        virtual prediction_model_base<float> * clone_as_float() const override
        {
            using P_new = typename predictor_traits<P>::template as<float>;
            return new prediction_model<float, P_new>(*this);
        }

        /*
         Clone the object onto the heap forcing it to be of type double.
         */
        virtual prediction_model_base<double> * clone_as_double() const override
        {
            using P_new = typename predictor_traits<P>::template as<double>;
            return new prediction_model<double, P_new>(*this);
        }


		/*!
		- Ensures
			- Returns the number of input variables required for the prediction model
		!*/
		virtual int n_variables() const override
		{
			return n_variables_;
		}


		/*!
		- Ensures
			- Returns the number of outputs of the prediction model
		!*/
		virtual int n_outputs() const override
		{
			return static_cast<int>(predictors_.size());
		}

        /*!
        - Requires
            - All samples in the input vector must have the same dimensionality as the samples used to train the pca_in
              model.
        - Ensures
            - if (pca_input_.is_trained())
                - returns a vector v_out such that: v_out[i] = pca_input_.parametrize(input[i]) for all valid i
            - else
                - returns a vector v_out which is a copy of input
        !*/
        virtual std::vector<matrix<T, 0, 1>> project_pca_in(const std::vector<matrix<T, 0, 1>>& input) const override
        {
            DLIB_ASSERT(
                !pca_input_.is_trained() ||
                    ( 
                        pca_input_.is_trained() &&
                        std::all_of(input.begin(), input.end(), 
                            [this](const auto& s) { return s.size() == pca_input_.n_variables(); })
                    ),
                "\n\t prediction_model::project_pca_in(...), invalid samples where given, expected samples of size "
                    << pca_input_.n_variables()
            );
            if (pca_input_.is_trained())
            {
                std::vector<matrix<T, 0, 1>> output;
                for (const auto& s : input)
                {
                    output.emplace_back(pca_input_.parameterize(s));
                }
                return output;
            }
            else
            {
                return input;
            }
        }

        /*!
        - Requires
            - All samples in the input vector must have the same dimensionality as the samples used to train the pca_out
              model.
        - Ensures
            - if (pca_output_.is_trained())
                - returns a vector v_out such that: v_out[i] = pca_output_.parametrize(input[i]) for all valid i
            - else
                - returns a vector v_out which is a copy of input
        !*/
        virtual std::vector<matrix<T, 0, 1>> project_pca_out(const std::vector<matrix<T, 0, 1>>& input) const override
        {
            DLIB_ASSERT(
                !pca_output_.is_trained() ||
                    (
                        pca_output_.is_trained() &&
                        std::all_of(input.begin(), input.end(),
                            [this](const auto& s) { return s.size() == pca_output_.n_variables(); })
                    ),
                "\n\t prediction_model::project_pca_out(...), invalid samples where given, expected samples of size "
                    << pca_output_.n_variables()
            );
            if (pca_output_.is_trained())
            {
                std::vector<matrix<T, 0, 1>> output;
                for (const auto& s : input)
                {
                    output.emplace_back(pca_output_.parameterize(s));
                }
                return output;
            }
            else
            {
                return input;
            }
        }

		/*!
			Predict the output for each regressor

			- Requires
				- input.nr() == n_variables()
				 - n_variables() > 0 (i.e. the predictor is trained)

			- Ensures
				- Predict the output

                input:
                    A reference to the input column vector used for prediction.
                pca_limit:
                    Limit value used to parametrize the input vector (see cm::pca_model<T>::parametrize(...))
                max_pca_components_to_predict:
                    Integer value specifying the maximal number of output pca modes to predict.
                    If this value is negative, or greater-equal to n_outputs(), all pca modes will be predicted.
                    If this value is 0 < max_pca_components_to_predict < n_outputs(), only the first
                    max_pca_components_to_predict will be computed, while all others will be set to 0.
                    This parameter has only an influence when pca_output_.is_trained() == true.
		*/
		virtual matrix<T, 0, 1>  predict(const matrix<T, 0, 1>& input, T pca_limit, const int max_pca_components_to_predict) const override
		{
			DLIB_ASSERT(input.nr() == n_variables() &&
				n_variables() > 0
				,
				"\t T predict(const matrix<T, 0, 1>& input)"
				<< "\n\t Invalid inputs were given to this function "
				<< "\n\t (input.nr(): " << input.nr()
				<< "\n\t (n_variables(): " << n_variables());

			matrix<T, 0, 1>  outputs;
			outputs.set_size(n_outputs());
            
		    // Set all entries of the output vector to 0.
            // In this way, if pca_output_.is_trained() and if we only predict its principal component,
		    // the other components will have a meaningful value.
            for (unsigned i = 0; i < static_cast<unsigned>(outputs.size()); ++i)
            {
                outputs(i) = T(0);
            }

            unsigned num_outputs_to_predict = n_outputs();

            if (pca_output_.is_trained()) {
                // Predict the correct number of pca components, based on the parameters passed as argument and the output size.
                // Note that if max_pca_components_to_predict has value -1, casting it to an unsigned value is equivalent of doing:
                // static_cast<unsigned>(-1) == std::numeric_limits<unsigned>::max().
                // Thus, setting max_pca_components_to_predict to -1 will end up using n_outputs() as num_outputs_to_predict.
                num_outputs_to_predict =
                    std::max(1u, std::min(static_cast<unsigned>(n_outputs()), static_cast<unsigned>(max_pca_components_to_predict)));
            }

			if (pca_input_.is_trained())
			{
				matrix<T, 0, 1> reduced = pca_input_.parameterize(input, pca_limit);     
				for (unsigned i = 0; i < num_outputs_to_predict; i++)
				{
					outputs(i) = predictors_[i](reduced);
				}
			}
			else
			{
				for (unsigned i = 0; i < num_outputs_to_predict; i++)
				{
					outputs(i) = predictors_[i](input);
				}
			}

			if (pca_output_.is_trained())
			{
				outputs = pca_output_.reconstruct(outputs);
			}

			return outputs;
		}

		/*!
		- Ensures
			- Return the estimated mean square error for each predictor output (from cross-validation)
		*/
		virtual const std::vector<double> & estimated_mses() const override
		{
			return estimated_mses_;
		}

		/*!
		- Ensures
            - Return the final gamma multipler for each predictor output (NB only applicable for predictors which have a
        gamma parameter eg SVR, KRR)
		*/
		virtual const std::vector<double> & final_gamma_multipliers() const override
		{
			return final_gamma_multipliers_;
		}

		/*!
		- Ensures
			- Return the final c for each predictor output (NB only applicable for predictors which have a c parameter
			eg SVR, KRR)
		*/
		virtual const std::vector<double>& final_cs() const override
		{
			return final_cs_;
		}

		/*!
		- Ensures
            - Return the final number of basis functions used for each predictor output (NB only applicable for
        predictors which have basis functions eg SVR, KRR)
		*/
		virtual const std::vector<int>& final_n_basis_functions() const override
		{
			return final_n_basis_functions_;
		}

		/*
		- Ensures
			- returns a unique name for the class
		*/
		virtual std::string name() const override
		{
			return "prediction_model" + std::string(typeid(predictors_).name());
		}

		/*
			virtual method for serialization
		*/
		virtual void serialize_class(std::ostream& out) const override
		{
			serialize(*this, out);
		}

		/*
			virtual method for deserialization
		*/
		virtual void deserialize_class(std::istream& in) override
		{
			deserialize(*this, in);
		}

		/*
			a function for helping legacy IO
		*/
		void set_contents_legacy(int n_variables, const pca_model<T> & pca_input, const pca_model<T> & pca_output, 
			const std::vector< P > & predictors, const std::vector<double> & estimated_mses,
			const std::vector<double> & final_gamma_multipliers , const std::vector<double> & final_cs,
			const std::vector<int> & final_n_basis_functions)
		{
			n_variables_ = n_variables;
			pca_input_ = pca_input;
			pca_output_ = pca_output;
			predictors_ = predictors;
			estimated_mses_ = estimated_mses;
			final_gamma_multipliers_ = final_gamma_multipliers;
			final_cs_ = final_cs;
			final_n_basis_functions_ = final_n_basis_functions;
		}


		// Friend functions for serialization 
		friend void serialize(const prediction_model<T, P>& item, std::ostream& out)
		{
			using dlib::serialize;
			// Output current class version number
			// IMPORTANT: if you add to or change the class member variables, please up the version number
			serialize(prediction_model<T, P>::version, out);

			serialize(item.n_variables_, out);
			serialize(item.pca_input_, out);
			serialize(item.pca_output_, out);
			serialize(item.predictors_, out);
			serialize(item.estimated_mses_, out);
			serialize(item.final_gamma_multipliers_, out);
			serialize(item.final_cs_, out);
			serialize(item.final_n_basis_functions_, out);
		}

		friend void deserialize(prediction_model<T, P>& item, std::istream& in)
		{
			unsigned cur_version;
			using dlib::deserialize;
			deserialize(cur_version, in);

			// current version deserialization
			// IMPORTANT: please try and make class serialization / deserialization back-compatible
			// by supporting multiple versions if possible, and if not, function should throw an exception
			if (cur_version == 2)
			{
				deserialize(item.n_variables_, in);
				deserialize(item.pca_input_, in);
				deserialize(item.pca_output_, in);
				deserialize(item.predictors_, in);
				deserialize(item.estimated_mses_, in);
				deserialize(item.final_gamma_multipliers_, in);
				deserialize(item.final_cs_, in);
				deserialize(item.final_n_basis_functions_, in);
			}
            else if (cur_version == 1)
            {
                deserialize(item.n_variables_, in);
                deserialize(item.pca_input_, in);
                deserialize(item.pca_output_, in);
                deserialize(item.predictors_, in);
                deserialize(item.estimated_mses_, in);
                deserialize(item.final_gamma_multipliers_, in);
                deserialize(item.final_cs_, in);
                deserialize(item.final_n_basis_functions_, in);
            }
			else
			{
				// back-compatibility code for previous versions should go here
				throw serialization_version_error("prediction_model", prediction_model<T, P>::version, cur_version);
			}
		}

		// friend class for building the model
		template <class T2> friend class multiple_output_prediction_model_wrapper_trainer;

	private:

		int n_variables_;
		pca_model<T> pca_input_;
		pca_model<T> pca_output_;
		std::vector< P > predictors_;
		std::vector<double> estimated_mses_;
		std::vector<double> final_gamma_multipliers_;
		std::vector<double> final_cs_;
		std::vector<int> final_n_basis_functions_;
	};

	template <typename T, typename P>
	const int prediction_model<T, P>::version = 2;


	// -------------------------------------------
	
}



