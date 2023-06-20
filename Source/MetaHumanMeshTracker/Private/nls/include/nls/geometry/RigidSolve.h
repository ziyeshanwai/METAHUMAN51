// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/simd/Simd.h>
#include <nls/geometry/QRigidMotion.h>
#include <nls/math/Math.h>

namespace epic::nls {

/**
 * Solves the rigid transformation minimizing the point to surface error i.e. sum(weight * targetNormal.dot(R * src + t - targetPt)) with
 * R = dR * initR
 * t = dT * initT
 * It only runs a single iteration of the linearization, and it optimizes from input transformation @p inputRigidMotion.
 * @param[in] outputDeltaTransform If True, outputs {dR, dT}, otherwise {R, t}.
 */
template <class T>
QRigidMotion<T> SolveRigid(const Eigen::Matrix<T, 3, -1>& srcPts,
                           const Eigen::Matrix<T, 3, -1>& targetPts,
                           const Eigen::Matrix<T, 3, -1>& targetNormals,
                           const Eigen::Vector<T, -1>& weights,
                           const QRigidMotion<T>& inputRigidMotion,
                           bool outputDeltaTransform = false);

/**
 * Solves the rigid transformation minimizing the point to surface error i.e. sum(weight * targetNormal.dot(R * src + t - targetPt)) using
 * R = dR * initR
 * t = dT * initT
 * It only runs a single iteration of the linearization, and it optimizes from input transformation @p inputRigidMotion.
 * Fast implementation using SIMD.
 * @param[in] outputDeltaTransform If True, outputs {dR, dT}, otherwise {R, t}.
 */

QRigidMotion<float> SolveRigid(
    const SimdType* srcPtsX,
    const SimdType* srcPtsY,
    const SimdType* srcPtsZ,
    const SimdType* targetPtsX,
    const SimdType* targetPtsY,
    const SimdType* targetPtsZ,
    const SimdType* targetNormalsX,
    const SimdType* targetNormalsY,
    const SimdType* targetNormalsZ,
    const SimdType* weights,
    const int numSimdElements,
    const QRigidMotion<float>& inputRigidMotion,
    bool outputDeltaTransform = false);

} // namespace epic::nls
