// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/data/CameraModelPinhole.h>

namespace epic {
namespace carbon {

/**
@brief Camera related functions.

*/

namespace camera {

/**
    @brief Projects a 3d point onto the image plane.

    @param cameraModel
        Camera with its intrinsic and extrinsic parameters.

    @param pt
        Point in referent 3D coordinate system.
*/

template<class T>
Eigen::Vector2<T> Project(const CameraModelPinhole<T>& cameraModel, const Eigen::Vector3<T>& pt);

}
}
}
