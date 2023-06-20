// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/Camera.h>
#include <nls/math/Math.h>

namespace epic::nls {

template <class T>
struct DepthmapData {
    Camera<T> camera;
    Eigen::Matrix<T, 4, -1> depthAndNormals;

    void Create(const Camera<T>& cam);
    void Create(const Camera<T>& cam, const T* depthPtr);
};

} // namespace epic::nls