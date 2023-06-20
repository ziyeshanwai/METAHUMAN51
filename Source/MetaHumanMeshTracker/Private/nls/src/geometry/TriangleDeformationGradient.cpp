// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/TriangleDeformationGradient.h>
#include <carbon/utils/Profiler.h>

#include <iostream>

namespace epic {
namespace nls {


template <class T>
void TriangleDeformationGradient<T>::SetTopology(const Eigen::Matrix<int, 3, -1>& triangles)
{
    m_triangles = triangles;
}


template <class T>
void TriangleDeformationGradient<T>::SetRestPose(const Eigen::Matrix<T, 3, -1>& vertices)
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    // verify that all triangles are valid i.e. have an area larger than 0
    for (int i = 0; i < NumTriangles(); i++) {
        const int vID0 = m_triangles(0, i);
        const int vID1 = m_triangles(1, i);
        const int vID2 = m_triangles(2, i);
        const Eigen::Vector3<T>& v0 = vertices.col(vID0);
        const Eigen::Vector3<T>& v1 = vertices.col(vID1);
        const Eigen::Vector3<T>& v2 = vertices.col(vID2);
        if ((v1 - v0).cross(v2- v0).norm() <= T(1e-10)) {
            throw std::runtime_error("Deformation gradients are invalid if triangles have degenerate triangles.");
        }
    }

    m_invRestFrame3D.clear();
    m_invRestFrame3D.reserve(NumTriangles());

    for (int i = 0; i < NumTriangles(); i++) {
        // for each triangle
        const Eigen::Vector3<T>& v0 = vertices.col(m_triangles(0, i));
        const Eigen::Vector3<T>& v1 = vertices.col(m_triangles(1, i));
        const Eigen::Vector3<T>& v2 = vertices.col(m_triangles(2, i));
        Eigen::Vector3<T> n3 = (v1 - v0).cross((v2 - v0));
        n3 /= std::sqrt(n3.norm());

        Eigen::Matrix<T, 3, 3> restFrame3D;
        restFrame3D.col(0) = v1 - v0;
        restFrame3D.col(1) = v2 - v0;
        restFrame3D.col(2) = n3;

        m_invRestFrame3D.push_back(restFrame3D.inverse());
    }
}


template <class T>
Eigen::Matrix<T, 9, -1> TriangleDeformationGradient<T>::DeformationGradient3D(const Eigen::Matrix<T, 3, -1>& vertices) const
{
    if (int(m_invRestFrame3D.size()) != NumTriangles()) {
        throw std::runtime_error("first call SetRestPose before calculating the deformation gradient");
    }

    Eigen::Matrix<T, 9, -1> gradients(9, NumTriangles());

    for (int i = 0; i < NumTriangles(); i++) {
        // for each triangle
        const Eigen::Vector3<T>& v0 = vertices.col(m_triangles(0, i));
        const Eigen::Vector3<T>& v1 = vertices.col(m_triangles(1, i));
        const Eigen::Vector3<T>& v2 = vertices.col(m_triangles(2, i));
        Eigen::Vector3<T> n3 = (v1 - v0).cross((v2 - v0));
        if (n3.norm() > 0) {
            n3 /= std::sqrt(n3.norm());
        }

        Eigen::Matrix<T, 3, 3> currFrame3D;
        currFrame3D.col(0) = v1 - v0;
        currFrame3D.col(1) = v2 - v0;
        currFrame3D.col(2) = n3;

        Eigen::Matrix<T, 3, 3> G = currFrame3D * m_invRestFrame3D[i];
        gradients.col(i) = Eigen::Map<const Eigen::Vector<T, 9>>(G.data(), G.size());
    }

    return gradients;
}


template <class T>
DiffDataMatrix<T, 9, -1> TriangleDeformationGradient<T>::DeformationGradient3D(const DiffDataMatrix<T, 3, -1>& vertices) const
{
    if (int(m_invRestFrame3D.size()) != NumTriangles()) {
        throw std::runtime_error("first call SetRestPose before calculating the deformation gradient");
    }

    VectorPtr<T> gradients = std::make_shared<Vector<T>>(9 * NumTriangles());
    std::vector<Eigen::Triplet<T>> tripletsVertices;    const bool calculateJacobian = vertices.HasJacobian();

    for (int i = 0; i < NumTriangles(); i++) {
        // for each triangle
        const int vID0 = m_triangles(0, i);
        const int vID1 = m_triangles(1, i);
        const int vID2 = m_triangles(2, i);
        const Eigen::Vector3<T>& v0 = vertices.Matrix().col(vID0);
        const Eigen::Vector3<T>& v1 = vertices.Matrix().col(vID1);
        const Eigen::Vector3<T>& v2 = vertices.Matrix().col(vID2);
        Eigen::Vector3<T> n3 = (v1 - v0).cross((v2 - v0));
        if (n3.norm() > 0) {
            n3 /= std::sqrt(n3.norm());
        }

        Eigen::Matrix<T, 3, 3> currFrame3D;
        currFrame3D.col(0) = v1 - v0;
        currFrame3D.col(1) = v2 - v0;
        currFrame3D.col(2) = n3;

        Eigen::Matrix<T, 3, 3> G = currFrame3D * m_invRestFrame3D[i];
        gradients->segment(9 * i, 9) = Eigen::Map<const Eigen::Vector<T, 9>>(G.data(), G.size());

        if (calculateJacobian) {
            for (int k = 0; k < 3; k++) {
                for (int j = 0; j < 3; j++) {
                    const int row = 9 * i + 3 * k + j;
                    tripletsVertices.push_back(Eigen::Triplet<T>(row, 3 * vID0 + j, - m_invRestFrame3D[i](0, k) - m_invRestFrame3D[i](1, k)));
                    tripletsVertices.push_back(Eigen::Triplet<T>(row, 3 * vID1 + j, m_invRestFrame3D[i](0, k)));
                    tripletsVertices.push_back(Eigen::Triplet<T>(row, 3 * vID2 + j, m_invRestFrame3D[i](1, k)));
                }
            }
        }
    }

    JacobianConstPtr<T> Jacobian;

    if (calculateJacobian) {
        SparseMatrix<T> localJacobian(9 * NumTriangles(), vertices.Size());
        localJacobian.setFromTriplets(tripletsVertices.begin(), tripletsVertices.end());
        Jacobian = vertices.Jacobian().Premultiply(localJacobian);
    }

    return DiffDataMatrix<T, 9, -1>(9, NumTriangles(), DiffData<T>(gradients, Jacobian));
}


template class TriangleDeformationGradient<float>;
template class TriangleDeformationGradient<double>;


} // namespace nls
} // namespace epic
