// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>
#include <tracking/rt/EulerXYZTransform.h>

namespace epic::nls::rt {

//! Rigid body with a set of vertices and a base transform.
template <class T>
struct RigidBody
{
    Eigen::Matrix<T, 3, -1> baseVertices;
    Eigen::Transform<T, 3, Eigen::Affine> baseTransform;

    Eigen::Transform<T, 3, Eigen::Affine> EvaluateTransform(const EulerXYZTransform<T>& deltaTransform) const {
        return baseTransform * deltaTransform.Transform();
    }

    Eigen::Matrix<T, 3, -1> EvaluateVertices(const EulerXYZTransform<T>& deltaTransform) const {
        return EvaluateTransform(deltaTransform) * baseVertices;
    }

    // get the derivative info for the delta transform
    Eigen::Matrix<T, 3, 6> Derivative(const EulerXYZTransform<T>& deltaTransform) const
    {
        Eigen::Matrix<T, 3, 6> derivative;

        // delta in translation
        derivative.rightCols(3) = baseTransform.linear();

        Eigen::Quaternion<T> r(baseTransform.linear());

        derivative.col(2).noalias() = r._transformVector(Eigen::Vector3<T>::UnitZ());
        r *= Eigen::Quaternion<T>(Eigen::AngleAxis<T>(deltaTransform.Rotation().z(), Eigen::Vector3<T>::UnitZ()));

        derivative.col(1).noalias() = r._transformVector(Eigen::Vector3<T>::UnitY());
        r *= Eigen::Quaternion<T>(Eigen::AngleAxis<T>(deltaTransform.Rotation().y(), Eigen::Vector3<T>::UnitY()));

        derivative.col(0).noalias() = r._transformVector(Eigen::Vector3<T>::UnitX());

        return derivative;
    }
};

} // namespace epic::nls::rt
