// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>
#include <nls/geometry/EulerAngles.h>
#include <tracking/rt/EulerXYZTransform.h>
#include <tracking/rt/HeadVertexState.h>
#include <tracking/rt/RigidBody.h>

namespace epic::nls::rt {

//! Head state with vertices for face and teeth, and a delta transformation for the left and right eye.
template <class T>
struct HeadState
{
    Eigen::Matrix<T, 3, -1> faceVertices;
    Eigen::Matrix<T, 3, -1> teethVertices;
    EulerXYZTransform<T> eyeLeftDeltaTransform;
    EulerXYZTransform<T> eyeRightDeltaTransform;

    //! @returns the data of the head state flattened to a vector
    Eigen::VectorX<T> Flatten() const {
        Eigen::VectorX<T> output(faceVertices.size() + teethVertices.size() + eyeLeftDeltaTransform.size() + eyeRightDeltaTransform.size());
        int offset = 0;
        output.segment(offset, faceVertices.size()) = Eigen::Map<const Eigen::VectorX<T>>(faceVertices.data(), faceVertices.size());
        offset += static_cast<int>(faceVertices.size());
        output.segment(offset, teethVertices.size()) = Eigen::Map<const Eigen::VectorX<T>>(teethVertices.data(), teethVertices.size());
        offset += static_cast<int>(teethVertices.size());
        output.segment(offset, eyeLeftDeltaTransform.size()) = eyeLeftDeltaTransform;
        offset += static_cast<int>(eyeLeftDeltaTransform.size());
        output.segment(offset, eyeRightDeltaTransform.size()) = eyeRightDeltaTransform;
        offset += static_cast<int>(eyeRightDeltaTransform.size());
        return output;
    }

    /**
     * @brief  Converts the head vertex state to a state with a rigid transform for the left and right eye. Calculates
     * the rigid transform using Eigen::umeyama().
     */
    static HeadState FromHeadVertexState(const HeadVertexState<T>& headVertexState,
                                         const RigidBody<T>& eyeLeftRigidBody,
                                         const RigidBody<T>& eyeRightRigidBody)
    {
        HeadState headState;
        headState.faceVertices = headVertexState.faceVertices;
        headState.teethVertices = headVertexState.teethVertices;
        {
            const Eigen::Transform<T, 3, Eigen::Affine> eyeLeftTransform(Eigen::umeyama(eyeLeftRigidBody.baseVertices, headVertexState.eyeLeftVertices));
            const Eigen::Transform<T, 3, Eigen::Affine> eyeLeftDeltaTransform = eyeLeftRigidBody.baseTransform.inverse() * eyeLeftTransform;
            headState.eyeLeftDeltaTransform.Translation() = eyeLeftDeltaTransform.translation();
            headState.eyeLeftDeltaTransform.Rotation() = RotationMatrixToEulerXYZ<T>(eyeLeftDeltaTransform.linear());
        }
        {
            const Eigen::Transform<T, 3, Eigen::Affine> eyeRightTransform(Eigen::umeyama(eyeRightRigidBody.baseVertices, headVertexState.eyeRightVertices));
            const Eigen::Transform<T, 3, Eigen::Affine> eyeRightDeltaTransform = eyeRightRigidBody.baseTransform.inverse() * eyeRightTransform;
            headState.eyeRightDeltaTransform.Translation() = eyeRightDeltaTransform.translation();
            headState.eyeRightDeltaTransform.Rotation() = RotationMatrixToEulerXYZ<T>(eyeRightDeltaTransform.linear());
        }
        return headState;
    }

    //! Converts the HeadState back to vertex state.
    HeadVertexState<T> ToHeadVertexState(const RigidBody<T>& eyeLeftRigidBody, const RigidBody<T>& eyeRightRigidBody) const
    {
        HeadVertexState<T> headVertexState;
        headVertexState.faceVertices = faceVertices;
        headVertexState.teethVertices = teethVertices;
        headVertexState.eyeLeftVertices = eyeLeftRigidBody.EvaluateVertices(eyeLeftDeltaTransform);
        headVertexState.eyeRightVertices = eyeRightRigidBody.EvaluateVertices(eyeRightDeltaTransform);
        return headVertexState;
    }
};

} // namespace epic::nls::rt
