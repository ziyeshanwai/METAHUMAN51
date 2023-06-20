// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/simd/Simd.h>
#include <nls/math/Math.h>

namespace epic::nls {

//! dVertex/dQuaternion assuming quaternion is the identity quaternion
template <class T>
Eigen::Matrix<T, 3, 3> QuaternionDerivativeMatrix(const Eigen::Vector3<T>& pt)
{
    Eigen::Matrix<T, 3, 3> mat;
    mat(0, 0) = 0;
    mat(1, 0) = - T(2) * pt[2];
    mat(2, 0) = T(2) * pt[1];
    mat(0, 1) = T(2) * pt[2];
    mat(1, 1) = 0;
    mat(2, 1) = - T(2) * pt[0];
    mat(0, 2) = - T(2) * pt[1];
    mat(1, 2) = T(2) * pt[0];
    mat(2, 2) = 0;
    return mat;
}

CARBON_SUPRESS_MS_WARNING(4324)
//! Rigid transform using Quaternion rotation.
template <class T>
struct QRigidMotion {
    Eigen::Quaternion<T> q;
    Eigen::Vector3<T> t;

    QRigidMotion() : q(Eigen::Quaternion<T>::Identity()), t(Eigen::Vector3<T>::Zero()) {}
    QRigidMotion(const Eigen::Quaternion<T>& q, const Eigen::Vector3<T>& t) : q(q), t(t) {}
    QRigidMotion(const Eigen::Matrix<T, 4, 4>& mat) : q(mat.template block<3, 3>(0, 0)), t(mat.template block<3, 1>(0, 3)) {}
    QRigidMotion(const Eigen::Matrix<T, 3, 3>& rot, const Eigen::Vector<T, 3>& t) : q(rot), t(t) {}


    Eigen::Transform<T, 3, Eigen::Affine> ToEigenTransform() const {
        Eigen::Transform<T, 3, Eigen::Affine> transform;
        transform.linear() = q.toRotationMatrix();
        transform.translation() = t;
        return transform;
    }

    void ToEigenTransform(Eigen::Transform<T, 3, Eigen::Affine>& transform) const {
        transform.linear() = q.toRotationMatrix();
        transform.translation() = t;
    }

    QRigidMotion operator*(const QRigidMotion& other) const
    {
        Eigen::Quaternion<T> qNew = q * other.q;
        Eigen::Vector3<T> tNew = q._transformVector(other.t) + t;
        return QRigidMotion{qNew, tNew};
    }

    QRigidMotion Inverse() const
    {
        Eigen::Quaternion<T> qNew = q.inverse();
        Eigen::Vector3<T> tNew = qNew._transformVector(-t);
        return QRigidMotion{qNew, tNew};
    }
};
CARBON_REENABLE_MS_WARNING

} // namespace epic::nls
