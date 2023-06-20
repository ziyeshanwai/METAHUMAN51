// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/algorithm/Camera.h>

namespace epic {
namespace carbon {
namespace camera {

template<class T>
Eigen::Vector2<T> Project(const CameraModelPinhole<T>& cameraModel, const Eigen::Vector3<T>& pt) {
    Eigen::Vector3<T> projected;
    Eigen::Vector4<T> ptH = Eigen::Vector4<T>(pt[0], pt[1], pt[2], T(1));
    Eigen::Vector3<T> transformed = cameraModel.GetExtrinsics() * ptH;

    projected = cameraModel.GetIntrinsics() * transformed;

    if (projected[2] != 0) {
        projected /= projected[2];
    }
    return Eigen::Vector2<T>(projected[0], projected[1]);
}

template Eigen::Vector2<float> Project(const CameraModelPinhole<float>& cameraModel, const Eigen::Vector3<float>& pt);
template Eigen::Vector2<double> Project(const CameraModelPinhole<double>& cameraModel, const Eigen::Vector3<double>& pt);

}
}
}
