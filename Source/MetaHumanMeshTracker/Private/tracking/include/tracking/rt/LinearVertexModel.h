// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>
#include <nls/math/ParallelBLAS.h>
#include <tracking/rt/LinearModel.h>

namespace epic::nls::rt {

/**
 * @brief Dense linear vertex model extended with the last 7 modes representing rigid deformation and scale.
 *
 * v = dR * dscale * (base + modes * coeffs) + dT
 * The rotation mode is a linearlized rotation (differential rotation using quaternion i.e. see QuaternionDerivativeMatrix) and in principle
 * this linearized rotation depends on the current vertices. In practice it is also ok to approximate it with the mean vertices
 * and therefore does not need to be constantly updated. The same applies to scale: the scale coefficient is the scale offset relative to a scale
 * of 1, and the linearization of the scale depends on the current vertices.
 */
template <class T>
struct LinearVertexModel {
public:
    enum class EvaluationMode {
        STATIC,
        RIGID,
        RIGID_SCALE
    };

    LinearVertexModel() = default;
    LinearVertexModel(const Eigen::Matrix<T, 3, -1>& base, const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>& modes)
    {
        Create(base, modes);
    }

    void Create(const Eigen::Matrix<T, 3, -1>& base, const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>& modes)
    {
        m_base = base;
        m_modes = modes;
        m_modes.conservativeResize(m_modes.rows(), m_modes.cols() + 7);
        m_modes.rightCols(7).setZero();
        SetTranslationModes();
        SetRotationModes(m_base);
        SetScaleMode(m_base);
    }

    int NumVertices() const { return int(m_base.cols()); }
    int NumTotalModes() const { return int(m_modes.cols()); }
    int NumPCAModes() const { return NumTotalModes() - 7; }

    void EvaluateLinearized(const Eigen::VectorX<T>& coeffs, EvaluationMode evaluationMode, Eigen::Matrix<T, 3, -1>& output) const
    {
        output.resize(3, m_base.cols());
        ParallelNoAliasGEMV<T>(Eigen::Map<Eigen::VectorX<T>>(output.data(), output.size()), BaseAsVector(), Modes(evaluationMode), coeffs);
    }

    Eigen::Matrix<T, 3, -1> EvaluateLinearized(const Eigen::VectorX<T>& coeffs, EvaluationMode evaluationMode) const
    {
        Eigen::Matrix<T, 3, -1> output;
        EvaluateLinearized(coeffs, evaluationMode, output);
        return output;
    }

    /**
     * @brief Evaluate the linear vertex model and store the result as a vertex model. The modes are only copied if the output modes size is different.
     *
     * @param coeffs The coefficients of the linear model (either with or without the rigid component).
     * @param evaluationMode Whether to evaluate the rigid component of the model.
     * @param output The LinearVertexModel that will contain the result.
     * @param forceCopyModes Force copying the modes even if the size is the same.
     */
    void EvaluateLinearized(const Eigen::VectorX<T>& coeffs, EvaluationMode evaluationMode, LinearVertexModel& output, bool forceCopyModes = false) const
    {
        EvaluateLinearized(coeffs, evaluationMode, output.MutableBase());
        if (forceCopyModes || output.MutableModes().cols() != m_modes.cols() || output.MutableModes().rows() != m_modes.rows()) {
            output.MutableModes() = m_modes;
        }
    }

    void SetTranslationModes()
    {
        for (int vID = 0; vID < int(m_base.cols()); ++vID) {
            m_modes.block(3 * vID, NumPCAModes() + 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
        }
    }

    void SetRotationModes(const Eigen::Matrix<T, 3, -1>& vertices)
    {
        T* matPtr = (T*)m_modes.data();
        const T* vertPtr = (const T*)vertices.data();
        for (int vID = 0; vID < int(vertices.cols()); ++vID) {
            // m_modes.block(3 * vID, NumPCAModes(), 3, 3).noalias() = QuaternionDerivativeMatrix<T>(vertices.col(vID));
            matPtr[(3 * vID + 0) * NumTotalModes() + NumPCAModes() + 1] =   T(2) * vertPtr[3 * vID + 2];
            matPtr[(3 * vID + 0) * NumTotalModes() + NumPCAModes() + 2] = - T(2) * vertPtr[3 * vID + 1];
            matPtr[(3 * vID + 1) * NumTotalModes() + NumPCAModes() + 0] = - T(2) * vertPtr[3 * vID + 2];
            matPtr[(3 * vID + 1) * NumTotalModes() + NumPCAModes() + 2] =   T(2) * vertPtr[3 * vID + 0];
            matPtr[(3 * vID + 2) * NumTotalModes() + NumPCAModes() + 0] =   T(2) * vertPtr[3 * vID + 1];
            matPtr[(3 * vID + 2) * NumTotalModes() + NumPCAModes() + 1] = - T(2) * vertPtr[3 * vID + 0];
        }
    }

    void SetScaleMode(const Eigen::Matrix<T, 3, -1>& vertices)
    {
        m_modes.block(0, NumTotalModes() - 1, m_modes.rows(), 1) = Eigen::Map<const Eigen::VectorX<T>>(vertices.data(), vertices.size());
    }

    Eigen::Ref<const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>> Modes(EvaluationMode evaluationMode) const {
        switch (evaluationMode) {
            default:
            case EvaluationMode::STATIC: return m_modes.leftCols(NumPCAModes());
            case EvaluationMode::RIGID: return m_modes.leftCols(NumTotalModes() - 1);
            case EvaluationMode::RIGID_SCALE: return m_modes;
        }
    }

    const Eigen::Matrix<T, 3, -1>& Base() const { return m_base; }
    Eigen::Ref<const Eigen::VectorX<T>> BaseAsVector() const { return Eigen::Map<const Eigen::VectorX<T>>(m_base.data(), m_base.size()); }

    //! Direct access to the base vertices of the linear model. Caller needs to know what (s)he is doing.
    Eigen::Matrix<T, 3, -1>& MutableBase() { return m_base; }

    //! Direct access to the modes of the linear model. Caller needs to know what (s)he is doing.
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor>& MutableModes() { return m_modes; }

    //! Translates the vertex model. Also updates the rotation modes using the new base.
    void Translate(const Eigen::Vector3<T>& translation)
    {
        m_base.colwise() += translation;
        SetRotationModes(m_base);
        SetScaleMode(m_base);
    }

    //! Rebsample the model keeping only the vertices in @p newToOldMap
    void Resample(const std::vector<int>& newToOldMap) {
        const int numNewVertices = int(newToOldMap.size());
        Eigen::Matrix<T, 3, -1> subsampledBase(3, numNewVertices);
        Eigen::Matrix<T, -1, -1, Eigen::RowMajor> subsampledModes(3 * numNewVertices, m_modes.cols());
        for (size_t newId = 0; newId < newToOldMap.size(); ++newId) {
            int oldId = newToOldMap[newId];
            subsampledBase.col(newId) = m_base.col(oldId);
            subsampledModes.block(3 * newId, 0, 3, m_modes.cols()) = m_modes.block(3 * oldId, 0, 3, m_modes.cols());
        }
        m_base = subsampledBase;
        m_modes = subsampledModes;
    }

private:
    Eigen::Matrix<T, 3, -1> m_base;
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> m_modes;
};

} // namespace epic::nls::rt
