// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/Camera.h>
#include <nls/geometry/RigidSolve.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic::nls {

QRigidMotion<float> RigidICP(const Eigen::Matrix<float, 3, -1>& srcPts,
                             const Eigen::Matrix<float, 3, -1>& srcNormals,
                             const std::vector<int>& srcIndices,
                             const Camera<float>& targetCamera,
                             const Eigen::Matrix<float, 4, -1>& targetDepthAndNormals,
                             const QRigidMotion<float>& inputRigidMotion,
                             const int numIterations);

QRigidMotion<float> RigidICPFast(const Eigen::Matrix<float, 3, -1>& srcPts,
                             const Eigen::Matrix<float, 3, -1>& srcNormals,
                             const std::vector<int>& srcIndices,
                             const Camera<float>& targetCamera,
                             const Eigen::Matrix<float, 4, -1>& targetDepthAndNormals,
                             const QRigidMotion<float>& inputRigidMotion,
                             const int numIterations);

} // namespace epic::nls
