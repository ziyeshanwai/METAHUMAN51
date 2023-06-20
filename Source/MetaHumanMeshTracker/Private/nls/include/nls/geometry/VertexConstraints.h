// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/utils/TaskThreadPool.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic::nls {


/**
 * @brief Class encapsulating a set of linearized vertex constraints.
 *
 * @tparam T                      Scalar type, typically float or double.
 * @tparam ResidualSize           Number of constraints per vertex.
 * @tparam NumConstraintVertices  Number of involved vertices.
 */
template <class T, int ResidualSize, int NumConstraintVertices>
class VertexConstraints
{
public:
    int64_t NumberOfConstraints() const { return m_numConstraints; }

    int64_t NumberOfReservedConstraints() const { return m_reservedConstraints; }

    void Clear(bool clearMemory = false)
    {
        m_numConstraints = 0;
        if (clearMemory) {
            m_vIDs.resize(NumConstraintVertices, 0);
            m_residuals.resize(ResidualSize, 0);
            m_jacobians.resize(0);
            m_reservedConstraints = 0;
        }
    }

    void ResizeToFitAdditionalConstraints(int64_t numAdditionalConstraints)
    {
        const int64_t requiredAdditionalConstraints = (NumberOfConstraints() + numAdditionalConstraints) - m_reservedConstraints;
        if (requiredAdditionalConstraints > 0) {
            m_reservedConstraints += requiredAdditionalConstraints;
            m_vIDs.conservativeResize(NumConstraintVertices, m_reservedConstraints);
            m_residuals.conservativeResize(ResidualSize, m_reservedConstraints);
            m_jacobians.resize(m_reservedConstraints);
        }
    }

    void AddConstraint(const Eigen::Vector<int, NumConstraintVertices>& vIDs,
                       const Eigen::Vector<T, ResidualSize>& residual,
                       const Eigen::Matrix<T, ResidualSize, NumConstraintVertices * 3>& jacobian)
    {
        CARBON_ASSERT(NumberOfConstraints() < m_reservedConstraints, "the number of constraints need to be reserved before adding them to VertexConstraints");
        m_vIDs.col(NumberOfConstraints()) = vIDs;
        m_residuals.col(NumberOfConstraints()) = residual;
        m_jacobians[NumberOfConstraints()] = jacobian;
        m_numConstraints++;
    }

    void AddConstraint(int vID,
                       const T residual,
                       const Eigen::Matrix<T, ResidualSize, NumConstraintVertices * 3>& jacobian)
    {
        static_assert(NumConstraintVertices == 1);
        static_assert(ResidualSize == 1);
        CARBON_ASSERT(NumberOfConstraints() < m_reservedConstraints, "the number of constraints need to be reserved before adding them to VertexConstraints");
        m_vIDs(0, NumberOfConstraints()) = vID;
        m_residuals(0, NumberOfConstraints()) = residual;
        m_jacobians[NumberOfConstraints()] = jacobian;
        m_numConstraints++;
    }

    void AddConstraint(int vID,
                       const T residual,
                       Eigen::Matrix<T, ResidualSize, NumConstraintVertices * 3>&& jacobian)
    {
        static_assert(NumConstraintVertices == 1);
        static_assert(ResidualSize == 1);
        CARBON_ASSERT(NumberOfConstraints() < m_reservedConstraints, "the number of constraints need to be reserved before adding them to VertexConstraints");
        m_vIDs(0, NumberOfConstraints()) = vID;
        m_residuals(0, NumberOfConstraints()) = residual;
        m_jacobians[NumberOfConstraints()] = std::move(jacobian);
        m_numConstraints++;
    }

    Eigen::Ref<const Eigen::VectorX<T>> Residual() const
    {
        return Eigen::Map<const Eigen::VectorX<T>>(m_residuals.data(), NumberOfConstraints() * ResidualSize);
    }

    Eigen::VectorX<T> EvaluateResidual(const Eigen::Matrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& baseVertices) const
    {
        Eigen::VectorX<T> residual = Eigen::Map<const Eigen::VectorX<T>>(m_residuals.data(), NumberOfConstraints() * ResidualSize);

        for (int i = 0; i < NumberOfConstraints(); ++i) {
            for (int k = 0; k < NumConstraintVertices; ++k) {
                residual.segment(i * ResidualSize, ResidualSize).noalias() += m_jacobians[i].block(0, k * 3, ResidualSize, 3) * (vertices.col(m_vIDs(k, i)) - baseVertices.col(m_vIDs(k, i)));
            }
        }

        return residual;
    }

    Eigen::Ref<const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>> EvaluateJacobian(const Eigen::Ref<const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>>& denseVertexJacobian,
                                                                                 Eigen::Matrix<T, -1, -1, Eigen::RowMajor>& denseVertexConstraintJacobian,
                                                                                 epic::carbon::TaskThreadPool* taskThreadPool = nullptr) const
    {
        if (denseVertexConstraintJacobian.rows() < NumberOfConstraints() * ResidualSize || denseVertexConstraintJacobian.cols() < denseVertexJacobian.cols()) {
            denseVertexConstraintJacobian.resize(NumberOfConstraints() * ResidualSize, denseVertexJacobian.cols());
        }

        auto process = [&](int start, int end) {
            for (int i = start; i < end; ++i) {
                denseVertexConstraintJacobian.block(i * ResidualSize, 0, ResidualSize, denseVertexJacobian.cols()).noalias() = m_jacobians[i].block(0, 0 * 3, ResidualSize, 3) * denseVertexJacobian.block(3 * m_vIDs(0, i), 0, 3, denseVertexJacobian.cols());
                if constexpr (NumConstraintVertices > 1)
                {
                    for (int k = 1; k < NumConstraintVertices; ++k) {
                        denseVertexConstraintJacobian.block(i * ResidualSize, 0, ResidualSize, denseVertexJacobian.cols()).noalias() += m_jacobians[i].block(0, k * 3, ResidualSize, 3) * denseVertexJacobian.block(3 * m_vIDs(k, i), 0, 3, denseVertexJacobian.cols());
                    }
                }
            }
        };

        if (taskThreadPool) {
            taskThreadPool->AddTaskRangeAndWait(int(NumberOfConstraints()), process);
        } else {
            process(0, int(NumberOfConstraints()));
        }

        return denseVertexConstraintJacobian.block(0, 0, NumberOfConstraints() * ResidualSize, denseVertexJacobian.cols());
    }

    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> EvaluateJacobian(const Eigen::Ref<const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>>& denseVertexJacobian) const
    {
        Eigen::Matrix<T, -1, -1, Eigen::RowMajor> denseVertexConstraintJacobian;
        EvaluateJacobian(denseVertexJacobian, denseVertexConstraintJacobian);
        return denseVertexConstraintJacobian;
    }


    SparseMatrix<T> SparseJacobian(int numVertices) const {
        SparseMatrix<T> localJacobian(NumberOfConstraints() * ResidualSize, numVertices * 3);
        localJacobian.reserve(NumberOfConstraints() * ResidualSize * NumConstraintVertices * 3);
        for (int i = 0; i < NumberOfConstraints(); ++i) {
            for (int j = 0; j < ResidualSize; ++j) {
                localJacobian.startVec(i * ResidualSize + j);
                for (int k = 0; k < NumConstraintVertices; ++k) {
                    for (int l = 0; l < 3; ++l) {
                        localJacobian.insertBackByOuterInnerUnordered(i * ResidualSize + j, 3 * m_vIDs(k, i) + l) = m_jacobians[i](j, 3 * k + l);
                    }
                }
            }
        }
        localJacobian.finalize();
        return localJacobian;
    }

private:
    Eigen::Matrix<int, NumConstraintVertices,-1> m_vIDs;
    Eigen::Matrix<T, ResidualSize, -1> m_residuals;
    std::vector<Eigen::Matrix<T, ResidualSize, NumConstraintVertices * 3>> m_jacobians;
    int64_t m_numConstraints = 0;
    int64_t m_reservedConstraints = 0;
};

} // namespace epic::nls
