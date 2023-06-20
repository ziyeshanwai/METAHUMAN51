// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/BoundedVectorVariable.h>
#include <nls/Context.h>
#include <nls/DiffData.h>

namespace epic {
namespace nls {

/**
 * Special coordinate descent solver with bounds and L1 regularization i.e. LASSO with box constraints.
 * see also https://www.stat.cmu.edu/~ryantibs/convexopt-S15/lectures/22-coord-desc.pdf.
 * https://www.jstatsoft.org/article/view/v033i01/v33i01.pdf
 *
 * Status: prototype - single bounded vector variable that is used for L1 regularization and bounds. hardcoded number of iterations, no settings such as number of line search iterations etc.
 */
template <class T>
class BoundedCoordinateDescentSolver
{
public:
	BoundedCoordinateDescentSolver() = default;

	/**
	 * Run the solve on the evaluation function with additional L1 regularization.
	 * @param [in] boundedVectorVariable			The vector variable for which the bounds are applied. The evaluationFunction needs to use this bounded variable, and should not use any
	 * 												other bounded varyable.
	 * @param [in] l1Reg 							The L1 regularization term.
	 * @param [in] coordinateDescentIterations  	The number of inner coordinate descent iterations.
	 *
	 * Solves the function:
	 * min 1/2 || evaluationFunction() ||_2^2 + | l1Reg boundedVectorVariable |_1
	 */
	bool Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			   Context<T>& context,
			   int iterations,
			   BoundedVectorVariable<T>* boundedVectorVariable,
			   T l1Reg,
			   int coordinateDescentIterations = 100) const;

	/**
	 * Run the solve on the evaluation function with additional L1 regularization.
	 * @param [in] boundedVectorVariables			A set of vector variable for which the bounds are applied. The evaluationFunction needs to use this bounded variable, and should not use any
	 * 												other bounded varyable.
	 * @param [in] l1Reg 							The L1 regularization term.
	 * @param [in] coordinateDescentIterations  	The number of inner coordinate descent iterations.
	 *
	 * Solves the function:
	 * min 1/2 || evaluationFunction() ||_2^2 + | l1Reg boundedVectorVariable |_1
	 */
	bool Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			   Context<T>& context,
			   int iterations,
			   const std::vector<BoundedVectorVariable<T>*>& boundedVectorVariables,
			   T l1Reg,
			   int coordinateDescentIterations = 100) const;

	//! Evaluate the total energy for the function: 1/2 || evaluationFunction() ||_2^2 + sum_i | l1Reg boundedVectorVariable_i |_1
	T Evaluate(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction, const std::vector<BoundedVectorVariable<T>*>& boundedVectorVariables, T l1Reg) const;

private:

	//! Evaluate the L1 norm of the bounded vector variables sum_i | l1Reg boundedVectorVariable_i |_1
	T EvaluateL1(const std::vector<BoundedVectorVariable<T>*>& boundedVectorVariables, T l1Reg) const;

private:

};

} // namespace nls
} //namespace epic
