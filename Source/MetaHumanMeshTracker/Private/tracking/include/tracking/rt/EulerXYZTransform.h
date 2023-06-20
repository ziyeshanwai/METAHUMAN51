// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/EulerAngles.h>
#include <nls/math/Math.h>

namespace epic::nls::rt {

//! Class representing a rigid motion as an EulerXYZ rotation and a translation.
template <class T>
struct EulerXYZTransform : public Eigen::Vector<T, 6>
{
    EulerXYZTransform() : Eigen::Vector<T, 6>() { this->setZero(); }

    Eigen::Ref</* */ Eigen::Vector3<T>> Rotation() /* */ { return this->segment(0, 3); };
    Eigen::Ref<const Eigen::Vector3<T>> Rotation() const { return this->segment(0, 3); };
    Eigen::Ref</* */ Eigen::Vector3<T>> Translation() /* */ { return this->segment(3, 3); };
    Eigen::Ref<const Eigen::Vector3<T>> Translation() const { return this->segment(3, 3); };

    Eigen::Transform<T, 3, Eigen::Affine> Transform() const {
        Eigen::Transform<T, 3, Eigen::Affine> transform;
        transform.linear() = EulerXYZ<T>(Rotation()[0], Rotation()[1], Rotation()[2]);
        transform.translation() = Translation();
        return transform;
    }

    static EulerXYZTransform FromTransform(const Eigen::Transform<T, 3, Eigen::Affine>& transform) {
        EulerXYZTransform eulerXYZTransform;
        eulerXYZTransform.Rotation() = RotationMatrixToEulerXYZ<T>(transform.linear());
        eulerXYZTransform.Translation() = transform.translation();
        return eulerXYZTransform;
    }
};

} // namespace epic::nls::rt
