// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/VertexLaplacian.h>
#include <carbon/utils/Profiler.h>

namespace epic::nls {

template <class T>
SparseMatrix<T> VertexLaplacian<T>::LaplacianMatrix(const Mesh<T>& mesh, Type type, int dim)
{
    Mesh<T> triMesh = mesh;
    triMesh.Triangulate();

    const Eigen::VectorX<T> vertexAreas = triMesh.VertexAreas();
    Eigen::VectorX<T> normalizationWeights = Eigen::VectorX<T>::Zero(triMesh.NumVertices());

    std::vector<Eigen::Triplet<T>> triplets;
    triplets.reserve(triMesh.NumTriangles() * 12);

    for (int i = 0; i < triMesh.NumTriangles(); ++i) {
        const int vID0 = triMesh.Triangles()(0, i);
        const int vID1 = triMesh.Triangles()(1, i);
        const int vID2 = triMesh.Triangles()(2, i);
        const Eigen::Vector3<T> v0 = triMesh.Vertices().col(vID0);
        const Eigen::Vector3<T> v1 = triMesh.Vertices().col(vID1);
        const Eigen::Vector3<T> v2 = triMesh.Vertices().col(vID2);
        const Eigen::Vector3<T> e01 = v1 - v0;
        const Eigen::Vector3<T> e12 = v2 - v1;
        const Eigen::Vector3<T> e20 = v0 - v2;
        if ((type == Type::COTAN) || (type == Type::UNIFORM)) {
            T w0 = (type == Type::COTAN) ? cotangent<T>(e01, -e20) : T(1);
            T w1 = (type == Type::COTAN) ? cotangent<T>(e12, -e01) : T(1);
            T w2 = (type == Type::COTAN) ? cotangent<T>(e20, -e12) : T(1);

            normalizationWeights[vID0] += (w1 + w2);
            triplets.push_back(Eigen::Triplet<T>(vID0, vID0,  w1));
            triplets.push_back(Eigen::Triplet<T>(vID0, vID2, -w1));
            triplets.push_back(Eigen::Triplet<T>(vID0, vID0,  w2));
            triplets.push_back(Eigen::Triplet<T>(vID0, vID1, -w2));

            normalizationWeights[vID1] += (w0 + w2);
            triplets.push_back(Eigen::Triplet<T>(vID1, vID1,  w0));
            triplets.push_back(Eigen::Triplet<T>(vID1, vID2, -w0));
            triplets.push_back(Eigen::Triplet<T>(vID1, vID1,  w2));
            triplets.push_back(Eigen::Triplet<T>(vID1, vID0, -w2));

            normalizationWeights[vID2] += (w0 + w1);
            triplets.push_back(Eigen::Triplet<T>(vID2, vID2,  w0));
            triplets.push_back(Eigen::Triplet<T>(vID2, vID1, -w0));
            triplets.push_back(Eigen::Triplet<T>(vID2, vID2,  w1));
            triplets.push_back(Eigen::Triplet<T>(vID2, vID0, -w1));
        }
        else if (type == Type::MEANVALUE) {
            T cos0 = e01.dot(-e20);
            if (cos0 > 0) cos0 /= e01.norm() * e20.norm();
            T cos1 = -e01.dot(e12);
            if (cos1 > 0) cos1 /= e01.norm() * e12.norm();
            T cos2 =  e20.dot(-e12);
            if (cos2 > 0) cos2 /= e20.norm() * e12.norm();

            auto safe_tan_half = [] (T cosAngle) {
                if (cosAngle >= 1) {
                    return T(0);
                } else if (cosAngle <= -1) {
                    return T(1e6); // arbitrary high number
                } else {
                    return T(sqrt((T(1) - cosAngle) / (T(1) + cosAngle)));
                }
            };

            constexpr T eps = T(1e-6);
            const T w01 = safe_tan_half(cos0) / std::max<T>(eps, e01.norm());
            const T w02 = safe_tan_half(cos0) / std::max<T>(eps, e20.norm());

            const T w10 = safe_tan_half(cos1) / std::max<T>(eps, e01.norm());
            const T w12 = safe_tan_half(cos1) / std::max<T>(eps, e12.norm());

            const T w20 = safe_tan_half(cos2) / std::max<T>(eps, e20.norm());
            const T w21 = safe_tan_half(cos2) / std::max<T>(eps, e12.norm());

            normalizationWeights[vID0] += (w01 + w02);
            triplets.push_back(Eigen::Triplet<T>(vID0, vID0,  w02));
            triplets.push_back(Eigen::Triplet<T>(vID0, vID2, -w02));
            triplets.push_back(Eigen::Triplet<T>(vID0, vID0,  w01));
            triplets.push_back(Eigen::Triplet<T>(vID0, vID1, -w01));

            normalizationWeights[vID1] += (w10 + w12);
            triplets.push_back(Eigen::Triplet<T>(vID1, vID1,  w12));
            triplets.push_back(Eigen::Triplet<T>(vID1, vID2, -w12));
            triplets.push_back(Eigen::Triplet<T>(vID1, vID1,  w10));
            triplets.push_back(Eigen::Triplet<T>(vID1, vID0, -w10));

            normalizationWeights[vID2] += (w20 + w21);
            triplets.push_back(Eigen::Triplet<T>(vID2, vID2,  w21));
            triplets.push_back(Eigen::Triplet<T>(vID2, vID1, -w21));
            triplets.push_back(Eigen::Triplet<T>(vID2, vID2,  w20));
            triplets.push_back(Eigen::Triplet<T>(vID2, vID0, -w20));
        }
    }

    std::vector<Eigen::Triplet<T>> extendedDimensionTriplets;
    extendedDimensionTriplets.reserve(triplets.size() * dim);
    for (const Eigen::Triplet<T>& triplet : triplets) {
        const int vID0 = triplet.row();
        const int vID1 = triplet.col();
        // normalize the values i.e. sum of weights = 1
        const T normalizedValue = triplet.value() / normalizationWeights[vID0];
        // weight the laplacian by the vertex area - in the least square sense this means that Ldx is scaled appropriately and therefore global scale changes do not influence the result
        const T areaWeightedValue = (vertexAreas[vID0] > 0) ? (normalizedValue / sqrt(vertexAreas[vID0])) : 0;
        // extend the Laplacian to dim dimensions
        for (int k = 0; k < dim; k++) {
            extendedDimensionTriplets.push_back(Eigen::Triplet<T>(dim * vID0 + k, dim * vID1 + k, areaWeightedValue));
        }
    }

    SparseMatrix<T> laplacianMatrix(triMesh.NumVertices() * dim, triMesh.NumVertices() * dim);
    laplacianMatrix.setFromTriplets(extendedDimensionTriplets.begin(), extendedDimensionTriplets.end());
    return laplacianMatrix;
}

template <class T>
void VertexLaplacian<T>::SetRestPose(const Mesh<T>& mesh, Type type)
{
    m_laplacianMatrix = LaplacianMatrix(mesh, type, 3);
}


template <class T>
DiffDataMatrix<T, 3, -1> VertexLaplacian<T>::EvaluateLaplacianOnOffsets(const DiffDataMatrix<T, 3, -1>& vertexOffsets, const T laplacianWeight) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    if (vertexOffsets.Size() != m_laplacianMatrix.rows()) {
        CARBON_CRITICAL("size of vertex offsets and laplacian matrix does not match: {} vs {}", vertexOffsets.Size(), m_laplacianMatrix.rows());
    }

    VectorPtr<T> outputData = std::make_shared<Vector<T>>(int(m_laplacianMatrix.rows()));

    const T laplacianWeightSqrt = std::sqrt(laplacianWeight);
    *outputData = laplacianWeightSqrt * (m_laplacianMatrix * vertexOffsets.Value());

    JacobianConstPtr<T> Jacobian;

    if (vertexOffsets.HasJacobian())
    {
        PROFILING_BLOCK("jacobian");
        if (laplacianWeightSqrt != T(1)) {
            SparseMatrix<T> localJacobian = laplacianWeightSqrt * m_laplacianMatrix;
            Jacobian = vertexOffsets.Jacobian().Premultiply(localJacobian);
        } else {
            Jacobian = vertexOffsets.Jacobian().Premultiply(m_laplacianMatrix);
        }
        PROFILING_END_BLOCK;
    }

    return DiffDataMatrix<T, 3, -1>(3, vertexOffsets.Cols(), DiffData<T>(outputData, Jacobian));
}


template class VertexLaplacian<float>;
template class VertexLaplacian<double>;

} // namespace epic::nls
