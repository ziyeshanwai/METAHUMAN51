// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>
#include <carbon/utils/Profiler.h>

namespace epic {
namespace nls {


/**
 * Function to add a vector to a matrix colwise f(x).colwise() += t(x)
 */
template <class T>
class ColwiseAddFunction
{
public:
	ColwiseAddFunction() {}

    template <int R, int C>
	DiffDataMatrix<T, R, C> colwiseAddFunction(const DiffDataMatrix<T, R, C>& matA, const DiffDataMatrix<T, R, 1>& vecB) const
	{
        PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

        CARBON_PRECONDITION(matA.Rows() == vecB.Size(), "row size needs to match the vector size that is added per column");

        PROFILING_BLOCK("add");
        VectorPtr<T> output = std::make_shared<Vector<T>>(matA.Size());
		Eigen::Map<Eigen::MatrixX<T>>(output->data(), matA.Rows(), matA.Cols()) = matA.Matrix().colwise() + vecB.Value();
        PROFILING_END_BLOCK;

		JacobianConstPtr<T> outputJacobian = matA.JacobianPtr();

		if (vecB.HasJacobian() && vecB.Jacobian().NonZeros() > 0) {
            // the translation jacobian is with respect to a single vertex, so we need to extend
            // it to all vertices
            PROFILING_BLOCK("repeat rows");
            JacobianConstPtr<T> repeatedJacobian = vecB.Jacobian().Repeat(matA.Cols());
            PROFILING_END_BLOCK;

            if (outputJacobian)
            {
                PROFILING_BLOCK("add matrices");
                // TODO: potential optimization: don't repeat rows above but repeat and add directly
                outputJacobian = outputJacobian->Add(repeatedJacobian);
                PROFILING_END_BLOCK;
            }
            else
            {
                outputJacobian = repeatedJacobian;
            }
		}

		return DiffDataMatrix<T, R, C>(matA.Rows(), matA.Cols(), DiffData<T>(output, outputJacobian));
	}
};


} // namespace nls
} //namespace epic
