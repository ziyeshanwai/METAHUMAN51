// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/QRigidMotion.h>
#include <nls/math/Math.h>
#include <tracking/rt/LinearModel.h>
#include <tracking/rt/LinearVertexModel.h>
#include <tracking/rt/RigidBody.h>

namespace epic::nls::rt {

//! Eye model with a rigid body for the eye mesh and a linear model representing rotation and translation.
template <class T>
struct LinearEyeModel
{
    RigidBody<T> eyeBody;
    LinearModel<T> linearTransformModel;

    /**
     * @brief Evaluates the eye vertices using the linear model coefficients. Optionally a linearized rigid transform
     * and a scale can be passed as part of the coefficients.
     *
     * @param coeffs     Parameters for the local linear rotation/translation model as well as optinoally a linearized rigid transform and scale.
     * @param evaluationMode Whether to just evaluate the model, apply also global rigid transformation, or a rigid transformation and scale.
     * @return Eigen::Matrix<T, 3, -1> The eye vertices.
     *
     * The transform of the eye ball is:
     * vertices = eyeBody.baseTransform * deltaTransform * eyeBody.baseVertices
     * with rigid: vertices = globalDeltaTransform * vertices
     * with rigid and scale: vertices = globalDeltaTransform * scale * vertices
     */
    Eigen::Matrix<T, 3, -1> EvaluateVertices(const Eigen::VectorX<T>& coeffs, typename LinearVertexModel<T>::EvaluationMode evaluationMode) const {
        Verify(coeffs, evaluationMode);
        const int numCoeffsNoRigid = int(linearTransformModel.modes.cols());
        const Eigen::VectorX<T> rotationAndTranslation = linearTransformModel.mean + linearTransformModel.modes * coeffs.head(numCoeffsNoRigid);
        EulerXYZTransform<T> deltaTransform;
        deltaTransform.Rotation() = rotationAndTranslation.segment(0, 3);
        deltaTransform.Translation() = rotationAndTranslation.segment(3, 3);
        switch (evaluationMode) {
            default:
            case LinearVertexModel<T>::EvaluationMode::STATIC: {
                return eyeBody.EvaluateVertices(deltaTransform);
            } break;
            case LinearVertexModel<T>::EvaluationMode::RIGID: {
                QRigidMotion<T> qrm(Eigen::Quaternion<T>(T(1), coeffs[6], coeffs[7], coeffs[8]).normalized(), Eigen::Vector3<T>(coeffs[9], coeffs[10], coeffs[11]));
                return qrm.ToEigenTransform() * eyeBody.EvaluateVertices(deltaTransform);
            } break;
            case LinearVertexModel<T>::EvaluationMode::RIGID_SCALE: {
                const T scale = coeffs[12];
                QRigidMotion<T> qrm(Eigen::Quaternion<T>(T(1), coeffs[6], coeffs[7], coeffs[8]).normalized(), Eigen::Vector3<T>(coeffs[9], coeffs[10], coeffs[11]));
                return qrm.ToEigenTransform() * (scale * eyeBody.EvaluateVertices(deltaTransform));
            } break;
        }
    }

    /**
     * @brief Evaluates the eye vertices and also calculates the Jacobian of the vertices with respect to the coefficients.
     */
    void EvaluateVerticesAndJacobian(const Eigen::VectorX<T>& coeffs, typename LinearVertexModel<T>::EvaluationMode evaluationMode, Eigen::Matrix<T, 3, -1>& vertices, Eigen::Matrix<T, -1, -1, Eigen::RowMajor>& jac) const
    {
        Verify(coeffs, evaluationMode);
        const int numCoeffsNoRigid = int(linearTransformModel.modes.cols());
        vertices.resize(3, eyeBody.baseVertices.size());
        jac.resize(vertices.size(), coeffs.size());
        const Eigen::VectorX<T> rotationAndTranslation = linearTransformModel.mean + linearTransformModel.modes * coeffs.head(numCoeffsNoRigid);
        EulerXYZTransform<T> deltaTransform;
        deltaTransform.Rotation() = rotationAndTranslation.segment(0, 3);
        deltaTransform.Translation() = rotationAndTranslation.segment(3, 3);
        vertices.noalias() = eyeBody.EvaluateVertices(deltaTransform);
        const Eigen::Matrix<T, 3, 6> deriv = eyeBody.Derivative(deltaTransform);
        const Eigen::Vector3<T> rotationCenter = eyeBody.EvaluateTransform(deltaTransform).translation();
        Eigen::Matrix<T, 3, 6> vertexDeriv = deriv;
        for (int i = 0; i < vertices.cols(); ++i) {
            vertexDeriv.col(0) = deriv.col(0).cross(vertices.col(i) - rotationCenter);
            vertexDeriv.col(1) = deriv.col(1).cross(vertices.col(i) - rotationCenter);
            vertexDeriv.col(2) = deriv.col(2).cross(vertices.col(i) - rotationCenter);

            jac.block(3 * i, 0, 3, numCoeffsNoRigid).noalias() = vertexDeriv * linearTransformModel.modes;

            if (evaluationMode ==  LinearVertexModel<T>::EvaluationMode::RIGID || evaluationMode == LinearVertexModel<T>::EvaluationMode::RIGID_SCALE) {
                // with rigid will also evaluate the jacobian for an additional global transformation
                jac.block(3 * i, numCoeffsNoRigid + 0, 3, 3).noalias() = QuaternionDerivativeMatrix<T>(vertices.col(i));
                jac.block(3 * i, numCoeffsNoRigid + 3, 3, 3).setIdentity();
            }
        }
        if (evaluationMode == LinearVertexModel<T>::EvaluationMode::RIGID_SCALE) {
            // with scale
            jac.rightCols(1).noalias() = Eigen::Map<const Eigen::VectorX<T>>(vertices.data(), vertices.size());
        }
    }

    /**
     * @brief Evaluates the eye vertices and also calculates the Jacobian of the vertices with respect to the coefficients.
     */
    void EvaluateVerticesAndJacobian(const Eigen::VectorX<T>& coeffs, typename LinearVertexModel<T>::EvaluationMode evaluationMode, LinearVertexModel<T>& vertexModel) const
    {
        Verify(coeffs, evaluationMode);
        const int numCoeffsNoRigid = int(linearTransformModel.modes.cols());
        vertexModel.MutableBase().resize(3, eyeBody.baseVertices.size());
        vertexModel.MutableModes().resize(vertexModel.NumVertices() * 3, numCoeffsNoRigid + 7);
        const Eigen::VectorX<T> rotationAndTranslation = linearTransformModel.mean + linearTransformModel.modes * coeffs.head(numCoeffsNoRigid);
        EulerXYZTransform<T> deltaTransform;
        deltaTransform.Rotation() = rotationAndTranslation.segment(0, 3);
        deltaTransform.Translation() = rotationAndTranslation.segment(3, 3);
        vertexModel.MutableBase().noalias() = eyeBody.EvaluateVertices(deltaTransform);
        const Eigen::Matrix<T, 3, 6> deriv = eyeBody.Derivative(deltaTransform);
        const Eigen::Vector3<T> rotationCenter = eyeBody.EvaluateTransform(deltaTransform).translation();
        Eigen::Matrix<T, 3, 6> vertexDeriv = deriv;
        for (int i = 0; i < vertexModel.MutableBase().cols(); ++i) {
            vertexDeriv.col(0) = deriv.col(0).cross(vertexModel.MutableBase().col(i) - rotationCenter);
            vertexDeriv.col(1) = deriv.col(1).cross(vertexModel.MutableBase().col(i) - rotationCenter);
            vertexDeriv.col(2) = deriv.col(2).cross(vertexModel.MutableBase().col(i) - rotationCenter);

            vertexModel.MutableModes().block(3 * i, 0, 3, numCoeffsNoRigid).noalias() = vertexDeriv * linearTransformModel.modes;

            if (evaluationMode ==  LinearVertexModel<T>::EvaluationMode::RIGID || evaluationMode == LinearVertexModel<T>::EvaluationMode::RIGID_SCALE) {
                // with rigid will also evaluate the jacobian for an additional global transformation
                vertexModel.MutableModes().block(3 * i, numCoeffsNoRigid + 0, 3, 3).noalias() = QuaternionDerivativeMatrix<T>(vertexModel.MutableBase().col(i));
                vertexModel.MutableModes().block(3 * i, numCoeffsNoRigid + 3, 3, 3).setIdentity();
            }
        }
        if (evaluationMode == LinearVertexModel<T>::EvaluationMode::RIGID_SCALE) {
            // with scale
            vertexModel.MutableModes().rightCols(1).noalias() = vertexModel.BaseAsVector();
        }
    }

    /**
     * @brief Evaluates the eye vertices and also calculates the Jacobian of the vertices with respect to the coefficients.
     */
    std::pair<Eigen::Matrix<T, 3, -1>, Eigen::Matrix<T, -1, -1, Eigen::RowMajor>> EvaluateVerticesAndJacobian(const Eigen::VectorX<T>& coeffs, typename LinearVertexModel<T>::EvaluationMode evaluationMode) const
    {
        Eigen::Matrix<T, 3, -1> vertices;
        Eigen::Matrix<T, -1, -1, Eigen::RowMajor> jac;
        EvaluateVerticesAndJacobian(coeffs, evaluationMode, vertices, jac);
        return {vertices, jac};
    }

    // translates the eye model
    void Translate(const Eigen::Vector3<T>& translation)
    {
        eyeBody.baseTransform = Eigen::Translation<T, 3>(translation) * eyeBody.baseTransform;
    }

    //! Resample the model keeping only the vertices in @p newToOldMap
    void Resample(const std::vector<int>& newToOldMap) {
        const int numNewVertices = int(newToOldMap.size());
        RigidBody<T> newEyeBody = eyeBody;
        newEyeBody.baseVertices.resize(3, numNewVertices);
        for (size_t newId = 0; newId < newToOldMap.size(); ++newId) {
            const int oldId = newToOldMap[newId];
            newEyeBody.baseVertices.col(newId) = eyeBody.baseVertices.col(oldId);
        }
        eyeBody = newEyeBody;
    }

private:
    void Verify(const Eigen::VectorX<T>& coeffs, typename LinearVertexModel<T>::EvaluationMode evaluationMode) const
    {
        int expectedNumCoeffs = 0;
        switch (evaluationMode) {
            case LinearVertexModel<T>::EvaluationMode::STATIC: expectedNumCoeffs = int(linearTransformModel.modes.cols()); break;
            case LinearVertexModel<T>::EvaluationMode::RIGID: expectedNumCoeffs = int(linearTransformModel.modes.cols()) + 6; break;
            case LinearVertexModel<T>::EvaluationMode::RIGID_SCALE: expectedNumCoeffs = int(linearTransformModel.modes.cols()) + 7; break;
        }
        if (expectedNumCoeffs != int(coeffs.size())) {
            LOG_ERROR("Number of coefficients passted to linear eye model are incorrect");
        }

    }

};

} // namespace epic::nls::rt
