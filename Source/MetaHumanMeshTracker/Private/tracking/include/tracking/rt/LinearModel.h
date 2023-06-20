// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>

#include <string>

namespace epic::nls::rt {

//! Simple linear model.
template <class T>
struct LinearModel
{
    Eigen::VectorX<T> mean;
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> modes;

    const Eigen::VectorX<T> Evaluate(const Eigen::VectorX<T>& coeffs) const {
        return mean + modes * coeffs;
    }
};

} // namespace epic::nls::rt
