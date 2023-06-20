// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/utils/Profiler.h>
#include <nls/DiffData.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/BarycentricCoordinates.h>
#include <nls/math/Math.h>
#include <nls/math/SparseMatrixBuilder.h>

namespace epic::nls {
    /**
     * Function to add gather values from diff data
     */
    template<class T, int R, int C=3>
    class BarycentricCoordinatesFunction {
        static_assert(C >= 1);
        public:
            /**
             * Evaluates the barycentric coordinates on the input vertices
             */
            static DiffDataMatrix<T, R, -1> Evaluate(const DiffDataMatrix<T, R, -1>& vertices,
                                                     const std::vector<BarycentricCoordinates<T, C> >& barycentricCoordinates) {
                PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

                const int numElements = int(barycentricCoordinates.size());

                VectorPtr<T> result = std::make_shared<Vector<T> >(numElements  * R);
                for (int i = 0; i < numElements; ++i) {
                    result->segment(i * R, R) = barycentricCoordinates[i].template Evaluate<R>(vertices.Matrix());
                }

                JacobianConstPtr<T> Jacobian;
                if (vertices.HasJacobian() && (vertices.Jacobian().NonZeros() > 0)) {
                    PROFILING_BLOCK("jacobian (1)");
                    SparseMatrix<T> localJacobian(R * numElements, vertices.Size());
                    localJacobian.reserve(numElements * C * R);
                    for (int i = 0; i < numElements; ++i) {
                        for (int k = 0; k < R; ++k) {
                            localJacobian.startVec(i * R + k);
                            for (int j = 0; j < C; ++j) {
                                const T w = barycentricCoordinates[i].Weight(j);
                                const int vID = barycentricCoordinates[i].Index(j);
                                if (w != T(0.0)) { // negative barycentric weights are allowed
                                    // we are (likely) filling the sparse matrix row out of order, but the Jacobian.Premultiply() function
                                    // below can handle this
                                    localJacobian.insertBackByOuterInnerUnordered(i * R + k, vID * R + k) = w;
                               }
                            }
                        }
                    }
                    localJacobian.finalize();
                    PROFILING_END_BLOCK;
                    PROFILING_BLOCK("jacobian (2)");
                    Jacobian = vertices.Jacobian().Premultiply(localJacobian);
                    PROFILING_END_BLOCK;
                }

                return DiffDataMatrix<T, R, -1>(R, numElements, DiffData<T>(result, Jacobian));
            }
    };
}  // namespace epic::nls
