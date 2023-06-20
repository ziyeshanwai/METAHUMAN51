// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>
#include <tracking/rt/EulerXYZTransform.h>

namespace epic::nls::rt {

//! Head state with vertices for face, teeth, left eye, and right eye.
template <class T>
struct HeadVertexState
{
    Eigen::Matrix<T, 3, -1> faceVertices;
    Eigen::Matrix<T, 3, -1> teethVertices;
    Eigen::Matrix<T, 3, -1> eyeLeftVertices;
    Eigen::Matrix<T, 3, -1> eyeRightVertices;

    Eigen::VectorX<T> Flatten() const {
        Eigen::VectorX<T> output(faceVertices.size() + teethVertices.size() + eyeLeftVertices.size() + eyeRightVertices.size());
        int offset = 0;
        output.segment(offset, faceVertices.size()) = Eigen::Map<const Eigen::VectorX<T>>(faceVertices.data(), faceVertices.size());
        offset += int(faceVertices.size());
        output.segment(offset, teethVertices.size()) = Eigen::Map<const Eigen::VectorX<T>>(teethVertices.data(), teethVertices.size());
        offset += int(teethVertices.size());
        output.segment(offset, eyeLeftVertices.size()) = eyeLeftVertices;
        offset += int(eyeLeftVertices.size());
        output.segment(offset, eyeRightVertices.size()) = eyeRightVertices;
        offset += int(eyeRightVertices.size());
        return output;
    }
};

} // namespace epic::nls::rt
