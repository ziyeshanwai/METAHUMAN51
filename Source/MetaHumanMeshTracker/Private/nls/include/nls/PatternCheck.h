// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {

/**
 * Helper class to check whether the sparsity pattern of a matrix has changed.
 */
template <class T>
class PatternCheck {

public:
    template <int Options>
    bool checkAndUpdatePattern(const Eigen::SparseMatrix<T, Options>& mat) {
        if (mat.rows() != rows
            || mat.cols() != cols
            || mat.nonZeros() != numNonZeros
            || Options != options) {
            rows = mat.rows();
            cols = mat.cols();
            numNonZeros = mat.nonZeros();
            options = Options;
            jtjPattern.resize(numNonZeros);
            memcpy(jtjPattern.data(), mat.innerIndexPtr(), numNonZeros * sizeof(int));
        } else {
            if (memcmp(jtjPattern.data(), mat.innerIndexPtr(), numNonZeros * sizeof(int)) == 0) {
                // same Pattern
                return false;
            }
            memcpy(jtjPattern.data(), mat.innerIndexPtr(), numNonZeros * sizeof(int));
        }
        return true;
    }

private:
    Eigen::VectorXi jtjPattern;
    int options = 0;
    int64_t rows = 0;
    int64_t cols = 0;
    int64_t numNonZeros = 0;
};

} // namespace nls
} //namespace epic
