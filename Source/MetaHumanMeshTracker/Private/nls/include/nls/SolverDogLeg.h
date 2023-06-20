// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Context.h>
#include <nls/DiffData.h>

namespace epic {
namespace nls {

template <class T> T DiagonalRegularization();
template <class T> T ResidualErrorStoppingCriterion();

template <class T>
class DogLegSolver
{
public:
	DogLegSolver() = default;

	bool Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			   int iterations = 1,
			   T reg = DiagonalRegularization<T>()) const
	{
		Context<T> context;
		return Solve(evaluationFunction, context, iterations, reg);
	}

	bool Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			   Context<T>& context,
			   int iterations = 1,
			   T reg = DiagonalRegularization<T>()) const;
};

} // namespace nls
} //namespace epic
