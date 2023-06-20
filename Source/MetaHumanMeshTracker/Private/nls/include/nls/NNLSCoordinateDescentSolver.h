// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Context.h>
#include <nls/DiffData.h>

namespace epic {
namespace nls {

	/*Coordinate descent solver for non-negative least squares problem: ||J*dx+fx||^2 s.t. dx>=0
	Reference paper: https://www.researchgate.net/publication/220914195_Sequential_Coordinate-Wise_Algorithm_for_the_Non-negative_Least_Squares_Problem */

template <class T>
class NNLSCoordinateDescentSolver
{
public:
	NNLSCoordinateDescentSolver() = default;
	bool Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
		int outerIterations=1,
		int innerIterations=200) const
	{
		Context<T> context;
		return Solve(evaluationFunction, context, outerIterations, innerIterations);
	}
	bool Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			   Context<T>& context,
		int outerIterations = 1,
		int innerIterations = 200) const;
private:
};
} // namespace nls
} //namespace epic
