// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/TriangleStrain.h>
#include <carbon/utils/Profiler.h>
#include <nls/math/Math.h>

#include <iostream>

namespace epic {
namespace nls {


template <class T>
void TriangleStrain<T>::SetTopology(const Eigen::Matrix<int, 3, -1>& triangles)
{
    m_triangles = triangles;
}


template <class T>
void TriangleStrain<T>::SetRestPose(const Eigen::Matrix<T, 3, -1>& vertices)
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    m_numVertices = int(vertices.cols());

    // verify that all triangles are valid i.e. have an area larger than 0
    for (int i = 0; i < int(m_triangles.cols()); i++) {
        const int vID0 = m_triangles(0, i);
        const int vID1 = m_triangles(1, i);
        const int vID2 = m_triangles(2, i);
        const Eigen::Vector3<T>& v0 = vertices.col(vID0);
        const Eigen::Vector3<T>& v1 = vertices.col(vID1);
        const Eigen::Vector3<T>& v2 = vertices.col(vID2);
        if ((v1 - v0).cross(v2- v0).norm() <= T(1e-10)) {
            throw std::runtime_error("Strain energy is not possible if triangles have degenerate triangles.");
        }
        if (vID0 < 0 || vID0 >= m_numVertices || vID1 < 0 || vID1 >= m_numVertices || vID2 < 0 || vID2 > m_numVertices) {
            throw std::runtime_error("Triangle index out of boudns.");
        }
    }

    const int numTriangles = int(m_triangles.cols());
    m_invRestFrame2D.clear();
    m_invRestFrame2D.reserve(numTriangles);

    for (int i = 0; i < int(m_triangles.cols()); i++) {
        // for each triangle
        const Eigen::Vector3<T>& v0 = vertices.col(m_triangles(0, i));
        const Eigen::Vector3<T>& v1 = vertices.col(m_triangles(1, i));
        const Eigen::Vector3<T>& v2 = vertices.col(m_triangles(2, i));

        // calculate local frame
        Eigen::Matrix<T, 3, 2> restFrame;
        restFrame.col(0) = v1 - v0;
        restFrame.col(1) = v2 - v0;

        // calculate (arbitrary) local orthogonal 2D frame
        Eigen::Matrix<T,3,2> coord2D;
        coord2D.col(0) = restFrame.col(0).normalized();
        coord2D.col(1) = (restFrame.col(1) - restFrame.col(1).dot(coord2D.col(0)) * coord2D.col(0)).normalized();
        const Eigen::Matrix<T,2,3> proj2D = coord2D.transpose();

        m_invRestFrame2D.push_back((proj2D * restFrame).inverse());
    }
}


template <class T>
DiffData<T> TriangleStrain<T>::EvaluateProjectiveStrain(const DiffDataMatrix<T, 3, -1>& vertices, const T strainWeight) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    const int numTriangles = int(m_triangles.cols());
    VectorPtr<T> outputData = std::make_shared<Vector<T>>(numTriangles * 6);

    std::vector<Eigen::Triplet<T>> triplets;
    if (vertices.HasJacobian()) {
        triplets.reserve(numTriangles * 18);
    }

    for (int i = 0; i < numTriangles; i++) {
        // Stretch Energy = || CurrFrame3D * inv(Proj2D * RestFrame3D) - Proj2D.tranpose() * F' ||_2^2
        const int vID0 = m_triangles(0, i);
        const int vID1 = m_triangles(1, i);
        const int vID2 = m_triangles(2, i);

        const Eigen::Vector3<T>& v0 = vertices.Matrix().col(vID0);
        const Eigen::Vector3<T>& v1 = vertices.Matrix().col(vID1);
        const Eigen::Vector3<T>& v2 = vertices.Matrix().col(vID2);

        // calculate local frame
        Eigen::Matrix<T, 3, 2> currFrame;
        currFrame.col(0) = v1 - v0;
        currFrame.col(1) = v2 - v0;

        // calculate deformation gradient and project to closest admissible deformation gradient
        const Eigen::Matrix<T, 3, 2> F = currFrame * m_invRestFrame2D[i];
        const Eigen::JacobiSVD<Eigen::Matrix<T, -1, -1>> svd(F, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Vector2<T> S = svd.singularValues();
        S[0] = T(1.0);
        S[1] = T(1.0);
        const Eigen::Matrix<T, 3, 2> Fdash = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();

        const Eigen::Matrix<T, 3, 2> residual = strainWeight * (currFrame * m_invRestFrame2D[i] - Fdash);

        outputData->segment(6 * i, 6) = Eigen::Map<const Eigen::Vector<T, 6>>(residual.data(), residual.size());

        if (vertices.HasJacobian()) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 3; k++) {
                    triplets.push_back(Eigen::Triplet<T>(6 * i + 3 * j + k, 3 * vID0 + k, - strainWeight * (m_invRestFrame2D[i](0, j) + m_invRestFrame2D[i](1, j))));
                    triplets.push_back(Eigen::Triplet<T>(6 * i + 3 * j + k, 3 * vID1 + k, strainWeight * m_invRestFrame2D[i](0, j)));
                    triplets.push_back(Eigen::Triplet<T>(6 * i + 3 * j + k, 3 * vID2 + k, strainWeight * m_invRestFrame2D[i](1, j)));
                }
            }
        }
    }

    JacobianConstPtr<T> Jacobian;

    if (vertices.HasJacobian())
    {
        PROFILING_BLOCK("jacobian (1)");
        SparseMatrix<T> localJacobian(int(outputData->size()), vertices.Size());
        localJacobian.setFromTriplets(triplets.begin(), triplets.end());
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("jacobian (2)");
        Jacobian = vertices.Jacobian().Premultiply(localJacobian);
        PROFILING_END_BLOCK;
    }

    return DiffData<T>(outputData, Jacobian);
}


template <class T>
DiffData<T> TriangleStrain<T>::EvaluateGreenStrain(const DiffDataMatrix<T, 3, -1>& vertices, const T strainWeight) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    const int numTriangles = int(m_triangles.cols());
    VectorPtr<T> outputData = std::make_shared<Vector<T>>(numTriangles * 3);

    std::vector<Eigen::Triplet<T>> triplets;
    if (vertices.HasJacobian()) {
        triplets.reserve(numTriangles * 27);
    }

    for (int i = 0; i < numTriangles; i++) {
        // F = [v1+d1 - v0+d0, v2+d2 - v0+d0] inv(proj2D * [v1 - v0, v2 - v0])
        // K = inv(proj2D * [v1 - v0, v2 - v0]) : 2x2 matrix
        // F = [v1+d1 - v0+d0, v2+d2 - v0+d0] K
        // Green = 1/2 [F^t F - I]
        // F = [v1+d1 - v0+d0, v2+d2 - v0+d0] K
        // F^t F = K^t [e1.e1 e1.e2] K
        //             [e2.e1 e2.e2]
        //
        //
        // F^t F = [e1.e1 K.row(0) + e2.e1 K.row(1),  e2.e1 K.row(0) + e2.e2 K.row(1)] * K
        //
        // FtF00 = (e1.e1 K(0,0) + e2.e1 K(1,0)) * K(0,0) + (e2.e1 K(0,0) + e2.e2 K(1,0)) * K(1,0)
        // FtF01 = (e1.e1 K(0,0) + e2.e1 K(1,0)) * K(0,1) + (e2.e1 K(0,0) + e2.e2 K(1,0)) * K(1,1)

        // FtF10 = (e1.e1 K(0,1) + e2.e1 K(1,1)) * K(0,0) + (e2.e1 K(0,1) + e2.e2 K(1,1)) * K(1,0)
        // FtF11 = (e1.e1 K(0,1) + e2.e1 K(1,1)) * K(0,1) + (e2.e1 K(0,1) + e2.e2 K(1,1)) * K(1,1)

        // FtF00 = e1.e1 K(0,0)^2 + 2 e1.e2 K(1,0) K(0,0) + e2.e2 K(1,0)^2
        // FtF11 = e1.e1 K(0,1)^2 + 2 e1.e2 K(1,1) K(0,1) + e2.e2 K(1,1)^2
        // FtF01 = FtF10 = e1.e1 K(0,0) K(0,1) + e1.e2 K(1,0) K(0,1) + e1.e2 K(0,0) K(1,1) + e2.e2 K(1,0) K(1,1)

        const int vID0 = m_triangles(0, i);
        const int vID1 = m_triangles(1, i);
        const int vID2 = m_triangles(2, i);

        const Eigen::Vector3<T>& v0 = vertices.Matrix().col(vID0);
        const Eigen::Vector3<T>& v1 = vertices.Matrix().col(vID1);
        const Eigen::Vector3<T>& v2 = vertices.Matrix().col(vID2);

        // calculate local frame
        //Eigen::Matrix<T, 3, 2> currFrame;
        //currFrame.col(0) = v1 - v0;
        //currFrame.col(1) = v2 - v0;

        //const Eigen::Matrix<T, 3, 2> F = currFrame * m_invRestFrame2D[i];
        //const Eigen::Matrix<T, 2, 2> GreenStrain2D = F.transpose() * F - Eigen::Matrix<T, 2, 2>::Identity();
        //std::cout << "GreenStrain2D:\n" << GreenStrain2D << std::endl;
        //printf("GreenStrain2D norm: %f\n", GreenStrain2D.squaredNorm());

        const T e1e1 = (v1 - v0).dot(v1 - v0); // v1x^2 + v0x^2 - 2 v1x v0x + ...
        const T e2e2 = (v2 - v0).dot(v2 - v0); // v2x^2 + v0x^2 - 2 v2x v0x + ...
        const T e1e2 = (v1 - v0).dot(v2 - v0); // v1x v2x + v0x^2 - v1x v0x - v2x v0x + ...

        const Eigen::Matrix<T, 2, 2> K = m_invRestFrame2D[i];
        Eigen::Vector<T, 3> GreenStrain2Db;
        GreenStrain2Db[0] = strainWeight * (e1e1 * K(0,0) * K(0,0) + T(2) * e1e2 * K(1,0) * K(0,0) + e2e2 * K(1,0) * K(1,0) - T(1.0));
        GreenStrain2Db[1] = strainWeight * (e1e1 * K(0,1) * K(0,1) + T(2) * e1e2 * K(1,1) * K(0,1) + e2e2 * K(1,1) * K(1,1) - T(1.0));
        GreenStrain2Db[2] = strainWeight * (std::sqrt(T(2)) * (e1e1 * K(0,0) * K(0,1) + e1e2 * K(1,0) * K(0,1) + e1e2 * K(0,0) * K(1,1) + e2e2 * K(1,0) * K(1,1)));
        //std::cout << "GreenStrain2Db:\n" << GreenStrain2Db << std::endl;
        //printf("GreenStrain2Db norm: %f\n", GreenStrain2Db.squaredNorm());

        outputData->segment(3 * i, 3) = GreenStrain2Db;

        if (vertices.HasJacobian()) {
            Eigen::Matrix<T, 3, 3> dG_dE;
            dG_dE(0, 0) = strainWeight * K(0,0) * K(0,0);
            dG_dE(0, 1) = strainWeight * K(1,0) * K(1,0);
            dG_dE(0, 2) = strainWeight * T(2) * K(1,0) * K(0,0);

            dG_dE(1, 0) = strainWeight * K(0,1) * K(0,1);
            dG_dE(1, 1) = strainWeight * K(1,1) * K(1,1);
            dG_dE(1, 2) = strainWeight * T(2) * K(1,1) * K(0,1);

            dG_dE(2, 0) = strainWeight * std::sqrt(T(2)) * K(0,0) * K(0,1);
            dG_dE(2, 1) = strainWeight * std::sqrt(T(2)) * K(1,0) * K(1,1);
            dG_dE(2, 2) = strainWeight * std::sqrt(T(2)) * (K(1,0) * K(0,1) + K(0,0) * K(1,1));

            // const T de1e1_dv0x = 2 v0x - 2 v1x;
            // const T de1e1_dv1x = 2 v1x - 2 v0x = -de1e1_dv-x;
            // const T de2e2_dv0x = 2 v0x - 2 v2x;
            // const T de2e2_dv2x = 2 v2x - 2 v0x = -de2e2_dv0x;
            // const T de1e2_dv0x = 2 v0x - v1x - v2x;
            // const T de1e2_dv1x = v2x - v0x;
            // const T de1e2_dv2x = v1x - v0x;
            // equivalent for de1e1_dv0y, de1e1_dv0z
            Eigen::Matrix<T, 3, 9> dE_dV;
            for (int k = 0; k < 3; k++) {
                // de1e1
                dE_dV(0, 0 + k) = T(2) * v0[k] - T(2) * v1[k];
                dE_dV(0, 3 + k) = - dE_dV(0, 0 + k);
                dE_dV(0, 6 + k) = T(0);
                // de2e2
                dE_dV(1, 0 + k) = T(2) * v0[k] - T(2) * v2[k];
                dE_dV(1, 3 + k) = T(0);
                dE_dV(1, 6 + k) = - dE_dV(1, 0 + k);
                // de1e2
                dE_dV(2, 0 + k) = T(2) * v0[k] - v1[k] - v2[k];
                dE_dV(2, 3 + k) = v2[k] - v0[k];
                dE_dV(2, 6 + k) = v1[k] - v0[k];
            }

            const Eigen::Matrix<T, 3, 9> dG_dV = dG_dE * dE_dV;
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    triplets.push_back(Eigen::Triplet<T>(3 * i + j, 3 * vID0 + k, dG_dV(j, 0 + k)));
                    triplets.push_back(Eigen::Triplet<T>(3 * i + j, 3 * vID1 + k, dG_dV(j, 3 + k)));
                    triplets.push_back(Eigen::Triplet<T>(3 * i + j, 3 * vID2 + k, dG_dV(j, 6 + k)));
                }
            }
        }
    }

    JacobianConstPtr<T> Jacobian;

    if (vertices.HasJacobian() && triplets.size() > 0)
    {
        PROFILING_BLOCK("jacobian (1)");
        SparseMatrix<T> localJacobian(int(outputData->size()), vertices.Size());
        localJacobian.setFromTriplets(triplets.begin(), triplets.end());
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("jacobian (2)");
        Jacobian = vertices.Jacobian().Premultiply(localJacobian);
        PROFILING_END_BLOCK;
    }

    return DiffData<T>(outputData, Jacobian);
}


template <class T>
Eigen::VectorX<T> TriangleStrain<T>::EvaluateProjectiveStrainPerVertex(const DiffDataMatrix<T, 3, -1>& vertices) const
{
    const Eigen::VectorX<T> projectiveStrain = EvaluateProjectiveStrain(vertices, T(1)).Value();
    Eigen::VectorX<T> verticesProjectiveStrain = Eigen::VectorX<T>::Zero(m_numVertices);

    for (int i = 0; i < int(m_triangles.cols()); i++) {
        const int vID0 = m_triangles(0, i);
        const int vID1 = m_triangles(1, i);
        const int vID2 = m_triangles(2, i);
        verticesProjectiveStrain[vID0] = std::max<T>(verticesProjectiveStrain[vID0], projectiveStrain[i]);
        verticesProjectiveStrain[vID1] = std::max<T>(verticesProjectiveStrain[vID1], projectiveStrain[i]);
        verticesProjectiveStrain[vID2] = std::max<T>(verticesProjectiveStrain[vID2], projectiveStrain[i]);
    }

    return verticesProjectiveStrain;
}


template <class T>
Eigen::VectorX<T> TriangleStrain<T>::EvaluateGreenStrainPerVertex(const DiffDataMatrix<T, 3, -1>& vertices) const
{
    const Eigen::VectorX<T> greenStrain = EvaluateGreenStrain(vertices, T(1)).Value();
    Eigen::VectorX<T> verticesGreenStrain = Eigen::VectorX<T>::Zero(m_numVertices);

    for (int i = 0; i < int(m_triangles.cols()); i++) {
        const int vID0 = m_triangles(0, i);
        const int vID1 = m_triangles(1, i);
        const int vID2 = m_triangles(2, i);
        verticesGreenStrain[vID0] = std::max<T>(verticesGreenStrain[vID0], greenStrain[i]);
        verticesGreenStrain[vID1] = std::max<T>(verticesGreenStrain[vID1], greenStrain[i]);
        verticesGreenStrain[vID2] = std::max<T>(verticesGreenStrain[vID2], greenStrain[i]);
    }

    return verticesGreenStrain;
}


template class TriangleStrain<float>;
template class TriangleStrain<double>;


} // namespace nls
} // namespace epic
