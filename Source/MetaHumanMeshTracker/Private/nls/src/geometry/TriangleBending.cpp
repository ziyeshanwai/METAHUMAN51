// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/TriangleBending.h>
#include <carbon/utils/Profiler.h>
#include <nls/math/Math.h>


#include <map>

namespace epic {
namespace nls {


template <class T>
void TriangleBending<T>::SetTopology(const Eigen::Matrix<int, 3, -1>& triangles)
{
    m_triangles = triangles;
}


template <class T>
void TriangleBending<T>::SetRestPose(const Eigen::Matrix<T, 3, -1>& vertices)
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
            throw std::runtime_error("Bending energy is not possible if triangles have degenerate triangles.");
        }

        if (vID0 < 0 || vID0 >= m_numVertices || vID1 < 0 || vID1 >= m_numVertices || vID2 < 0 || vID2 > m_numVertices) {
            throw std::runtime_error("Triangle index out of boudns.");
        }
    }

    // set up half edge mapping
    typedef std::pair<int,int> HalfEdge;

    std::map<HalfEdge, int> halfEdgeMapping;
    for (int i = 0; i < int(m_triangles.cols()); i++) {
        const int vID0 = m_triangles(0, i);
        const int vID1 = m_triangles(1, i);
        const int vID2 = m_triangles(2, i);
        halfEdgeMapping[HalfEdge(vID0, vID1)] = vID2;
        halfEdgeMapping[HalfEdge(vID1, vID2)] = vID0;
        halfEdgeMapping[HalfEdge(vID2, vID0)] = vID1;
    }

    // evaluate the number of constraints
    int numConstraints = 0;

    for (auto&& edgeToVertex : halfEdgeMapping) {
        const int vID0 = edgeToVertex.first.first;
        const int vID1 = edgeToVertex.first.second;

        if (vID0 < vID1) {
            auto it = halfEdgeMapping.find(HalfEdge(vID1, vID0));
            if (it != halfEdgeMapping.end()) {
                numConstraints++;
            }
        }
    }

    // set up data structures
    m_trianglePairs.resize(4, numConstraints);
    m_K.resize(4, numConstraints);
    m_restBendingEnergy.resize(numConstraints);
    m_restAngle.resize(numConstraints);
    m_restEdgeAreaSqrt.resize(numConstraints);
    m_restOneOverEdgeAreaSqrt.resize(numConstraints);
    m_restEdgeLength.resize(numConstraints);

    int constraint = 0;
    for (auto&& edgeToVertex : halfEdgeMapping) {
        const int vID0 = edgeToVertex.first.first;
        const int vID1 = edgeToVertex.first.second;
        const int vID2 = edgeToVertex.second;

        if (vID0 < vID1) {
            auto it = halfEdgeMapping.find(HalfEdge(vID1, vID0));
            if (it != halfEdgeMapping.end()) {
                const int vID3 = it->second;

                const Eigen::Vector3<T>& v0 = vertices.col(vID0);
                const Eigen::Vector3<T>& v1 = vertices.col(vID1);
                const Eigen::Vector3<T>& v2 = vertices.col(vID2);
                const Eigen::Vector3<T>& v3 = vertices.col(vID3);

                const Eigen::Vector3<T> e0 = v1 - v0;
                const Eigen::Vector3<T> e1 = v2 - v0;
                const Eigen::Vector3<T> e2 = v3 - v0;
                const Eigen::Vector3<T> e3 = v2 - v1;
                const Eigen::Vector3<T> e4 = v3 - v1;

                // Rest pose data for EvaluateQuadratic(), see derivation in
                // Bergou et al, "A Quadratic Bending Model for Inextensible Surfaces", see Implementation Details
                const T A1 = e0.cross(e1).norm() / T(2);
                const T A2 = e0.cross(e2).norm() / T(2);
                const T c01 = cotangent<T>(e0, e1);
                const T c02 = cotangent<T>(e0, e2);
                const T c03 = cotangent<T>(-e0, e3);
                const T c04 = cotangent<T>(-e0, e4);
                const T K0 = c03 + c04;
                const T K1 = c01 + c02;
                const T K2 = - c01 - c03;
                const T K3 = - c02 - c04;
                const T oneOverEdgeAreaSqrt = std::sqrt(T(3)/(A1 + A2));
                const T w = oneOverEdgeAreaSqrt;

                m_trianglePairs.col(constraint) = Eigen::Vector4i(vID0, vID1, vID2, vID3);
                m_K.col(constraint) = Eigen::Vector4<T>(K0 * w, K1 * w, K2 * w, K3 * w);

                const Eigen::Vector3<T>& bendingTerm = m_K(0, constraint) * v0 +
                                                       m_K(1, constraint) * v1 +
                                                       m_K(2, constraint) * v2 +
                                                       m_K(3, constraint) * v3;

                m_restBendingEnergy[constraint] = bendingTerm.norm();

                // Rest pose data for EvaluateDihedral()
                const Eigen::Vector3<T>& n1 = e0.cross(e1);
                const Eigen::Vector3<T>& n2 = e2.cross(e0);
                T angle = clamped_acos(n1.normalized().dot(n2.normalized()));
                if (n1.cross(n2).dot(e0) > 0) {
                    angle = -angle;
                }
                m_restAngle[constraint] = angle;
                m_restEdgeLength[constraint] = e0.norm();
                m_restOneOverEdgeAreaSqrt[constraint] = oneOverEdgeAreaSqrt;
                m_restEdgeAreaSqrt[constraint] = T(1) / oneOverEdgeAreaSqrt;

                constraint++;
            }
        }
    }
}


template <class T>
DiffData<T> TriangleBending<T>::EvaluateQuadratic(const DiffDataMatrix<T, 3, -1>& vertices,
                                                  const T bendingWeight) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    VectorPtr<T> outputData = std::make_shared<Vector<T>>(NumBendingTerms() * 3);

    const T bendingWeightSqrt = std::sqrt(bendingWeight);

    std::vector<Eigen::Triplet<T>> triplets;
    if (vertices.HasJacobian()) {
        triplets.reserve(NumBendingTerms() * 12);
    }

    for (int i = 0; i < NumBendingTerms(); i++) {
        const int vID0 = m_trianglePairs(0, i);
        const int vID1 = m_trianglePairs(1, i);
        const int vID2 = m_trianglePairs(2, i);
        const int vID3 = m_trianglePairs(3, i);

        const Eigen::Vector3<T>& v0 = vertices.Matrix().col(vID0);
        const Eigen::Vector3<T>& v1 = vertices.Matrix().col(vID1);
        const Eigen::Vector3<T>& v2 = vertices.Matrix().col(vID2);
        const Eigen::Vector3<T>& v3 = vertices.Matrix().col(vID3);

        const Eigen::Vector3<T>& bendingTerm = m_K(0,i) * v0 +
                                               m_K(1,i) * v1 +
                                               m_K(2,i) * v2 +
                                               m_K(3,i) * v3;
        const T bendingEnergy = bendingTerm.norm();
        const T restBendingEnergy = m_restBendingEnergy[i];

        Eigen::Vector3<T> targetBendingTerm = bendingTerm;

        if (bendingEnergy > 0) {
            const T ratio = restBendingEnergy / bendingEnergy;
            targetBendingTerm = ratio * bendingTerm;
        } else {
            // bending enery is 0, so we use the normals of the triangles
            targetBendingTerm = (v1 - v0).cross(v2 - v0) + (v3 - v0).cross(v1 - v0);
            const T l = targetBendingTerm.norm();
            if (l > T(1e-6)) {
                targetBendingTerm *= restBendingEnergy / l;
            } else {
                targetBendingTerm = Eigen::Vector3<T>(restBendingEnergy, 0, 0);
            }
        }

        outputData->segment(3 * i, 3) = bendingWeightSqrt * (bendingTerm - targetBendingTerm);

        if (vertices.HasJacobian()) {
            // We use the K matrix for the Jacobian, but to be fully correct it would also need to to take into
            // account the target bending term which is calculated by the normalization above.
            // This way the Jacobian is simply capturing the projection.
            // TODO: this matrix is constant and could be cached
            for (int j = 0; j < 3; j++) {
                triplets.push_back(Eigen::Triplet<T>(3 * i + j, 3 * vID0 + j, bendingWeightSqrt * m_K(0,i)));
                triplets.push_back(Eigen::Triplet<T>(3 * i + j, 3 * vID1 + j, bendingWeightSqrt * m_K(1,i)));
                triplets.push_back(Eigen::Triplet<T>(3 * i + j, 3 * vID2 + j, bendingWeightSqrt * m_K(2,i)));
                triplets.push_back(Eigen::Triplet<T>(3 * i + j, 3 * vID3 + j, bendingWeightSqrt * m_K(3,i)));
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
DiffData<T> TriangleBending<T>::EvaluateDihedral(const DiffDataMatrix<T, 3, -1>& vertices,
                                                 const T bendingWeight) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    VectorPtr<T> outputData = std::make_shared<Vector<T>>(NumBendingTerms());

    const T bendingWeightSqrt = std::sqrt(bendingWeight);

    std::vector<Eigen::Triplet<T>> triplets;
    if (vertices.HasJacobian()) {
        triplets.reserve(NumBendingTerms() * 12);
    }

    for (int i = 0; i < NumBendingTerms(); i++) {
        const int vID0 = m_trianglePairs(0, i);
        const int vID1 = m_trianglePairs(1, i);
        const int vID2 = m_trianglePairs(2, i);
        const int vID3 = m_trianglePairs(3, i);

        const Eigen::Vector3<T>& v0 = vertices.Matrix().col(vID0);
        const Eigen::Vector3<T>& v1 = vertices.Matrix().col(vID1);
        const Eigen::Vector3<T>& v2 = vertices.Matrix().col(vID2);
        const Eigen::Vector3<T>& v3 = vertices.Matrix().col(vID3);

        const Eigen::Vector3<T> e0 = v1 - v0;
        const Eigen::Vector3<T> e1 = v2 - v0;
        const Eigen::Vector3<T> e2 = v3 - v0;
        const Eigen::Vector3<T> e3 = v2 - v1;
        const Eigen::Vector3<T> e4 = v3 - v1;

        const Eigen::Vector3<T>& n1 = e0.cross(e1);
        const Eigen::Vector3<T>& n2 = e2.cross(e0);
        T angle = clamped_acos(n1.normalized().dot(n2.normalized()));
        if (n1.cross(n2).dot(e0) > 0) {
            angle = -angle;
        }

        // Dihedral bending error as described in Fröhlich and Botsch, "Example-Driven Deformations Based on Discrete Shells"
        // and Grinspun et al, "Discrete Shells", in each case Equation (2).
        const T w = bendingWeightSqrt * m_restEdgeLength[i] * m_restOneOverEdgeAreaSqrt[i];
        (*outputData)[i] = w * (angle - m_restAngle[i]);

        if (vertices.HasJacobian()) {
            // Jacobian as described in Fröhlich and Botsch, "Example-Driven Deformations Based on Discrete Shells"
            // and Bridson et al, "Simulation of Clothing with Folds and Wrinkles".
            // TODO: potential optimization: the number of constraints do not change, so the matrix could be set up
            //       and only the values could be inserted.
            const Eigen::Vector3<T> ddx1 = w * (e3.dot(e0)/(e0.norm() * n1.squaredNorm()) * n1 + e4.dot(e0)/(e0.norm() * n2.squaredNorm()) * n2);
            const Eigen::Vector3<T> ddx2 = - w * (e1.dot(e0)/(e0.norm() * n1.squaredNorm()) * n1 + e2.dot(e0)/(e0.norm() * n2.squaredNorm()) * n2);
            const Eigen::Vector3<T> ddx3 = w * e0.norm() / n1.squaredNorm() * n1;
            const Eigen::Vector3<T> ddx4 = w * e0.norm() / n2.squaredNorm() * n2;
            for (int k = 0; k < 3; k++) triplets.push_back(Eigen::Triplet<T>(i, 3 * vID0 + k, ddx1[k]));
            for (int k = 0; k < 3; k++) triplets.push_back(Eigen::Triplet<T>(i, 3 * vID1 + k, ddx2[k]));
            for (int k = 0; k < 3; k++) triplets.push_back(Eigen::Triplet<T>(i, 3 * vID2 + k, ddx3[k]));
            for (int k = 0; k < 3; k++) triplets.push_back(Eigen::Triplet<T>(i, 3 * vID3 + k, ddx4[k]));
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


template<class T>
Eigen::VectorX<T> TriangleBending<T>::EvaluateAngles(const Eigen::Matrix<T, 3, -1>& vertices) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    const int numBendingTerms = int(m_trianglePairs.cols());
    Eigen::VectorX<T> angles(numBendingTerms);

    for (int i = 0; i < NumBendingTerms(); i++) {
        const int vID0 = m_trianglePairs(0, i);
        const int vID1 = m_trianglePairs(1, i);
        const int vID2 = m_trianglePairs(2, i);
        const int vID3 = m_trianglePairs(3, i);

        const Eigen::Vector3<T>& v0 = vertices.col(vID0);
        const Eigen::Vector3<T>& v1 = vertices.col(vID1);
        const Eigen::Vector3<T>& v2 = vertices.col(vID2);
        const Eigen::Vector3<T>& v3 = vertices.col(vID3);

        const Eigen::Vector3<T>& n1 = (v1 - v0).cross(v2 - v0);
        const Eigen::Vector3<T>& n2 = (v3 - v0).cross(v1 - v0);
        angles[i] = clamped_acos(n1.normalized().dot(n2.normalized()));
        if (n1.cross(n2).dot(v1 - v0) > 0) {
            angles[i] = -angles[i];
        }
    }

    return angles;
}


template <class T>
Eigen::VectorX<T> TriangleBending<T>::EvaluateQuadraticPerVertex(const DiffDataMatrix<T, 3, -1>& vertices) const
{
    const Eigen::VectorX<T> quadraticBending = EvaluateQuadratic(vertices, T(1)).Value();

    Eigen::VectorX<T> verticesQuadraticBending = Eigen::VectorX<T>::Zero(m_numVertices);
    for (int i = 0; i < int(m_trianglePairs.cols()); i++) {
        const int vID0 = m_trianglePairs(0, i);
        const int vID1 = m_trianglePairs(1, i);
        verticesQuadraticBending[vID0] = std::max<T>(verticesQuadraticBending[vID0], quadraticBending[i]);
        verticesQuadraticBending[vID1] = std::max<T>(verticesQuadraticBending[vID1], quadraticBending[i]);
    }

    return verticesQuadraticBending;
}


template <class T>
Eigen::VectorX<T> TriangleBending<T>::EvaluateDihedralPerVertex(const DiffDataMatrix<T, 3, -1>& vertices) const
{
    const Eigen::VectorX<T> dihedralBending = EvaluateDihedral(vertices, T(1)).Value();

    Eigen::VectorX<T> verticesDihedralBending = Eigen::VectorX<T>::Zero(m_numVertices);
    for (int i = 0; i < int(m_trianglePairs.cols()); i++) {
        const int vID0 = m_trianglePairs(0, i);
        const int vID1 = m_trianglePairs(1, i);
        verticesDihedralBending[vID0] = std::max<T>(verticesDihedralBending[vID0], dihedralBending[i]);
        verticesDihedralBending[vID1] = std::max<T>(verticesDihedralBending[vID1], dihedralBending[i]);
    }

    return verticesDihedralBending;
}


template class TriangleBending<float>;
template class TriangleBending<double>;

} // namespace nls
} //namespace epic
