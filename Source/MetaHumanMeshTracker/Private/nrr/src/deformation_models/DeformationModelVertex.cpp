// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/deformation_models/DeformationModelVertex.h>
#include <nrr/deformation_models/DeformationModelRigid.h>

#include <nls/geometry/AffineVariable.h>
#include <nls/geometry/QuaternionVariable.h>
#include <nls/geometry/TriangleBending.h>
#include <nls/geometry/TriangleStrain.h>
#include <nls/geometry/VertexLaplacian.h>

#include <vector>

namespace epic::nls {

template <class T>
struct DeformationModelVertex<T>::Private {

    DeformationModelRigid<T> defModelRigid;
    Eigen::Matrix<T, 3, -1> restVertices;
    std::unique_ptr<MatrixVariable<T, 3, -1>> varOffsets;
    TriangleStrain<T> triangleStrain;
    TriangleBending<T> triangleBending;
    VertexLaplacian<T> vertexLaplacian;

    Configuration config = {DeformationModelVertex<T>::ConfigName(), {
        //!< weight on regularizing the per-vertex offset
        { "vertexLaplacian", ConfigurationParameter(T(1.0), T(0), T(1)) },
        //!< weight on regularizing the per-vertex offset
        { "vertexOffsetRegularization", ConfigurationParameter(T(0.1), T(0), T(1)) },
        //!< projective strain weight (stable, but incorrect Jacobian)
        { "projectiveStrain", ConfigurationParameter(T(0), T(0), T(0.5)) },
        //!< green strain (unstable???, correct Jacobian)
        { "greenStrain", ConfigurationParameter(T(0.0), T(0), T(0.5)) },
        //!< quadratic bending (stable, but incorrect Jacobian, and also has strain component)
        { "quadraticBending", ConfigurationParameter(T(0), T(0), T(0.5)) },
        //!< dihedral bending (unstable???, correct Jacobian)
        { "dihedralBending", ConfigurationParameter(T(0.0), T(0), T(0.5)) },
        //!< whether to optimize the pose when doing fine registration
        { "optimizePose", ConfigurationParameter(false) }
    }};

};

template <class T>
DeformationModelVertex<T>::DeformationModelVertex()
    : m(std::make_unique<Private>())
{}

template <class T> DeformationModelVertex<T>::~DeformationModelVertex() = default;
template <class T> DeformationModelVertex<T>::DeformationModelVertex(DeformationModelVertex&& other) = default;
template <class T> DeformationModelVertex<T>& DeformationModelVertex<T>::operator=(DeformationModelVertex&& other) = default;

template <class T>
std::pair<DiffDataMatrix<T, 3, -1>, DiffDataMatrix<T, 3, -1>> DeformationModelVertex<T>::EvaluateBothStabilizedAndTransformedVertices(Context<T>* context)
{
    if  (!m->varOffsets) {
        CARBON_CRITICAL("no vertices have been defined");
    }

    const DiffDataMatrix<T, 3, -1> baseVertices(m->restVertices);
    const DiffDataMatrix<T, 3, -1> offsets = m->varOffsets->EvaluateMatrix(context);
    const DiffDataMatrix<T, 3, -1> vertices = baseVertices + offsets;
    const bool optimizePose = m->config["optimizePose"].template Value<bool>();
    return {vertices, m->defModelRigid.EvaluateAffine(optimizePose ? context : nullptr).Transform(vertices)};
}

template <class T>
DiffDataMatrix<T, 3, -1> DeformationModelVertex<T>::EvaluateVertices(Context<T>* context)
{
    return EvaluateBothStabilizedAndTransformedVertices(context).second;
}

template <class T>
Cost<T> DeformationModelVertex<T>::EvaluateModelConstraints(Context<T>* context)
{
    Cost<T> cost;

    const T vertexOffsetRegularization = m->config["vertexOffsetRegularization"].template Value<T>();
    const T projectiveStrain = m->config["projectiveStrain"].template Value<T>();
    const T greenStrain = m->config["greenStrain"].template Value<T>();
    const T quadraticBending = m->config["quadraticBending"].template Value<T>();
    const T dihedralBending = m->config["dihedralBending"].template Value<T>();
    const T vertexLaplacian = m->config["vertexLaplacian"].template Value<T>();

    const DiffDataMatrix<T, 3, -1> baseVertices(m->restVertices);
    const DiffDataMatrix<T, 3, -1> offsets = m->varOffsets->EvaluateMatrix(context);
    const DiffDataMatrix<T, 3, -1> vertices = baseVertices + offsets;

    if (vertexOffsetRegularization > 0) {
        cost.Add(m->varOffsets->EvaluateMatrix(context), vertexOffsetRegularization, "vertexOffsetRegularization");
    }

    if (projectiveStrain > 0) {
        cost.Add(m->triangleStrain.EvaluateProjectiveStrain(vertices, projectiveStrain), T(1), "projectiveStrain");
    }

    if (greenStrain > 0) {
        cost.Add(m->triangleStrain.EvaluateGreenStrain(vertices, greenStrain), T(1), "greenStrain");
    }

    if (quadraticBending > 0) {
        cost.Add(m->triangleBending.EvaluateQuadratic(vertices, quadraticBending), T(1), "quadraticBending");
    }

    if (dihedralBending > 0) {
        cost.Add(m->triangleBending.EvaluateDihedral(vertices, dihedralBending), T(1), "dihedralBending");
    }

    if (vertexLaplacian > 0) {
        cost.Add(m->vertexLaplacian.EvaluateLaplacianOnOffsets(offsets, T(1)), vertexLaplacian, "vertexLaplacian");
    }

    return cost;
}

template <class T>
const Configuration& DeformationModelVertex<T>::GetConfiguration() const
{
    return m->config;
}

template <class T>
void DeformationModelVertex<T>::SetConfiguration(const Configuration& config)
{
    m->config.Set(config);
}


template <class T>
void DeformationModelVertex<T>::SetMeshTopology(const Mesh<T>& mesh)
{
    m->triangleStrain.SetTopology(mesh.Triangles());
    m->triangleBending.SetTopology(mesh.Triangles());

    if (int(m->restVertices.cols()) == mesh.NumVertices()) {
        m->triangleStrain.SetRestPose(m->restVertices);
        m->triangleBending.SetRestPose(m->restVertices);
    }

    m->vertexLaplacian.SetRestPose(mesh, VertexLaplacian<T>::Type::MEANVALUE);
}


template <class T>
void DeformationModelVertex<T>::SetRestVertices(const Eigen::Matrix<T, 3, -1>& vertices)
{
    m->restVertices = vertices;
    if (!m->varOffsets || m->varOffsets->Cols() != vertices.cols()) {
        m->varOffsets = std::make_unique<MatrixVariable<T, 3, -1>>(3, int(vertices.cols()));
        m->varOffsets->SetZero();
    }

    if (m->triangleStrain.Triangles().cols() > 0) {
        m->triangleStrain.SetRestPose(m->restVertices);
        m->triangleBending.SetRestPose(m->restVertices);
    }

    m->defModelRigid.SetVertices(m->restVertices);
}

template <class T>
void DeformationModelVertex<T>::SetVertexOffsets(const Eigen::Matrix<T, 3, -1>& offsets)
{
    if (!m->varOffsets || m->varOffsets->Cols() != offsets.cols()) {
        m->varOffsets = std::make_unique<MatrixVariable<T, 3, -1>>(3, int(offsets.cols()));
    }
    m->varOffsets->SetMatrix(offsets);
}

template <class T>
Eigen::Matrix<T, 3, -1> DeformationModelVertex<T>::VertexOffsets() const
{
    if  (!m->varOffsets) {
        throw std::runtime_error("no vertices have been defined");
    }
    return m->varOffsets->Matrix();
}

template <class T>
Eigen::Matrix<T, 3, -1> DeformationModelVertex<T>::DeformedVertices() const
{
    return VertexOffsets() + m->restVertices;
}

template <class T>
void DeformationModelVertex<T>::SetRigidTransformation(const Affine<T, 3, 3>& affine)
{
    m->defModelRigid.SetRigidTransformation(affine);
}

template <class T>
Affine<T, 3, 3> DeformationModelVertex<T>::RigidTransformation() const
{
    return m->defModelRigid.RigidTransformation();
}

template <class T>
void DeformationModelVertex<T>::MakeVerticesConstant(const std::vector<int>& constantVertices)
{
    if (!m->varOffsets) {
        throw std::runtime_error("no vertices have been defined");
    }
    std::vector<int> constantIndices;
    // make all 3 coordinates constant
    for (auto vID : constantVertices) {
        constantIndices.push_back(3 * vID + 0);
        constantIndices.push_back(3 * vID + 1);
        constantIndices.push_back(3 * vID + 2);
    }
    m->varOffsets->MakeIndividualIndicesConstant(constantIndices);
}

template <class T>
void DeformationModelVertex<T>::MakeVerticesMutable()
{
    if (!m->varOffsets) {
        throw std::runtime_error("no vertices have been defined");
    }
    m->varOffsets->MakeMutable();
}

// explicitly instantiate the DeformationModelVertex classes
template class DeformationModelVertex<float>;
template class DeformationModelVertex<double>;

} // namespace epic::nls
