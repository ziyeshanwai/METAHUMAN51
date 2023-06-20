// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "../OpenCVCamera.h"

#include <nls/geometry/MetaShapeCamera.h>

namespace titan {
namespace api {

template <class T>
epic::nls::MetaShapeCamera<T> OpenCVCamera2MetaShapeCamera(const char* InCameraName, const OpenCVCamera& InCameraParameters)
{
    epic::nls::MetaShapeCamera<T> camera;
    camera.SetLabel(InCameraName);
    camera.SetWidth(InCameraParameters.width);
    camera.SetHeight(InCameraParameters.height);
    Eigen::Matrix<T, 3, 3> intrinsics = Eigen::Matrix3f::Identity();
    intrinsics(0, 0) = T(InCameraParameters.fx);
    intrinsics(1, 1) = T(InCameraParameters.fy);
    intrinsics(0, 2) = T(InCameraParameters.cx);
    intrinsics(1, 2) = T(InCameraParameters.cy);
    camera.SetIntrinsics(intrinsics);
    const Eigen::Matrix4f extrinsics = Eigen::Map<const Eigen::Matrix4f>(InCameraParameters.Extrinsics);
    camera.SetExtrinsics(extrinsics.template cast<T>());
    camera.SetRadialDistortion(Eigen::Vector4f(InCameraParameters.k1, InCameraParameters.k2, InCameraParameters.k3, 0.0f).template cast<T>());
    // note that metashape camera has swapped tangential distortion compared to opencv
    camera.SetTangentialDistortion(Eigen::Vector4f(InCameraParameters.p2, InCameraParameters.p1, 0.0f, 0.0f).template cast<T>());

    return camera;
}

} // namespace api
} // namespace titan
