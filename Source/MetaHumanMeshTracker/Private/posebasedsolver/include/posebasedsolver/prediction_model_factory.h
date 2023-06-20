// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "prediction_model.h"

namespace cm
{
    using namespace dlib;


	/*!

		- WHAT THIS OBJECT REPRESENTS
			- This object represents a factory class for a prediction_model
	!*/
	template <typename T>
	class prediction_model_factory
	{

	public:

		/*
		- Ensures
			- creates a prediction_model_base ptr on the heap
		*/
		static prediction_model_base<T> * create(const std::string & )
		{
			// Since the general framework for prediction models has been stripped down to
			// just a single class for inclusion in nonlinearsolver repo, this factory method
			// no longer needs to be general, as only the prediction_model<T, linear_predictor<T> >
			// class is supported. If this should be resurrected in future, this factory method 
			// would need to be re-generalized

			return new prediction_model<T, linear_predictor<T> >();

		}

	};

}



