// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Context.h>
#include <nls/DiffData.h>

namespace epic {
namespace nls {

template <class T>
class LBFGSSolver
{
public:
	LBFGSSolver()
	{
		// make sure storage is set up correctly
		s.resize(historySize);
		y.resize(historySize);
		p = Vector<T>::Zero(historySize);
		a = Vector<T>::Zero(historySize);
	}

	bool Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			   int iterations = 1) const
	{
		Context<T> context;
		return Solve(evaluationFunction, context, iterations);
	}

	bool Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			   Context<T>& context,
			   int iterations = 1) const;

private:
	// LBFGS storage
	int historySize = 50;
	mutable std::vector<Vector<T>> s;
	mutable std::vector<Vector<T>> y;
	mutable Vector<T> p;
	mutable Vector<T> a;
};

} // namespace nls
} //namespace epic
