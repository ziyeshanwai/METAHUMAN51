// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Context.h>
#include <nls/DiffData.h>

namespace epic {
namespace nls {

template <class T>
class LMSolver
{
public:
	LMSolver() = default;

	bool Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			   int iterations = 1) const
	{
		Context<T> context;
		return Solve(evaluationFunction, context, iterations);
	}

	bool Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			   Context<T>& context,
			   int iterations = 1) const;
};

} // namespace nls
} //namespace epic
