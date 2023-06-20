// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/utils/Profiler.h>
#include <nls/DiffData.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic {
namespace nls {


/**
 * Function to add gather values from diff data
 */
template <class T>
class GatherFunction
{
public:
	GatherFunction() {}

	static DiffData<T> Gather(const DiffData<T>& a, const Eigen::VectorX<int>& blockIndices, int blockSize = 1)
    {
        PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

        const int numElements = int(blockIndices.size()) * blockSize;
		VectorPtr<T> result = std::make_shared<Vector<T>>(numElements);
        for (int i = 0; i < int(blockIndices.size()); i++) {
            const int blockIdx = blockIndices[i];
            for (int k = 0; k < blockSize; k++) {
                const int idx = blockIdx * blockSize + k;
                (*result)[i * blockSize + k] = a.Value()[idx];
            }
        }

        JacobianConstPtr<T> Jacobian;
        if (a.HasJacobian() && a.Jacobian().NonZeros() > 0) {
            PROFILING_BLOCK("jacobian");
            Jacobian = a.Jacobian().RowGather(blockIndices, blockSize);
            CARBON_ASSERT(Jacobian->Rows() == numElements, "jacobian row size needs to match the number of elements that are gathered");
            CARBON_ASSERT(Jacobian->Cols() == a.Jacobian().Cols(), "jacobian column size needs to match the number of column of the input DiffData");
            PROFILING_END_BLOCK;
        }

		return DiffData<T>(result, Jacobian);
	}

	template <int R, int C1, int C2>
	static DiffDataMatrix<T, R, C1> GatherColumns(const DiffDataMatrix<T, R, C2>& a, const Eigen::VectorX<int>& colIndices)
	{
        if constexpr (C1 > 0) {
            CARBON_PRECONDITION(int(colIndices.size()) == C1, "for a fixed size of output columns the input indices need to match");
        }
		return DiffDataMatrix<T, R, C1>(a.Rows(), int(colIndices.size()), Gather(a, colIndices, a.Rows()));
	}

    template <int R, int C1, int C2>
	static DiffDataMatrix<T, R, C1> GatherColumns(const DiffDataMatrix<T, R, C2>& a, const std::vector<int>& colIndices)
	{
        return GatherColumns<R, C1, C2>(a, Eigen::Map<const Eigen::VectorXi>(colIndices.data(), colIndices.size()));
	}
};


} // namespace nls
} //namespace epic
