// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/SmoothSurfaceProjection.h>

#include <nls/Cost.h>
#include <nls/MatrixVariable.h>
#include <nls/Solver.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>
#include <nls/geometry/MeshCorrespondenceSearch.h>
#include <nls/geometry/TriangleBending.h>
#include <nls/geometry/TriangleStrain.h>
#include <nls/serialization/EigenSerialization.h>
#include <nls/serialization/MeshSerialization.h>
#include <carbon/io/JsonIO.h>
#include <carbon/utils/Profiler.h>

namespace epic::nls {

template <class T>
struct SmoothSurfaceProjection<T>::Private
{
    Mesh<T> targetMesh;
    Mesh<T> sourceMesh;

    MeshCorrespondenceSearch<T> correspondenceSearch;

    TriangleStrain<T> triangleStrain;
    TriangleBending<T> triangleBending;
};


template <class T>
SmoothSurfaceProjection<T>::SmoothSurfaceProjection() : m(std::make_unique<Private>())
{
}

template <class T> SmoothSurfaceProjection<T>::~SmoothSurfaceProjection() = default;
template <class T> SmoothSurfaceProjection<T>::SmoothSurfaceProjection(SmoothSurfaceProjection&& other) = default;
template <class T> SmoothSurfaceProjection<T>& SmoothSurfaceProjection<T>::operator=(SmoothSurfaceProjection&& other) = default;


template <class T>
void SmoothSurfaceProjection<T>::SetTarget(Mesh<T>& mesh)
{
    m->targetMesh = mesh;
    m->targetMesh.Triangulate();
    m->targetMesh.CalculateVertexNormals();
    m->correspondenceSearch.Init(m->targetMesh);
    m->triangleStrain.SetTopology(m->targetMesh.Triangles());
    m->triangleStrain.SetRestPose(m->targetMesh.Vertices());
    m->triangleBending.SetTopology(m->targetMesh.Triangles());
    m->triangleBending.SetRestPose(m->targetMesh.Vertices());
}


template <class T>
void SmoothSurfaceProjection<T>::SetTargetMeshJsonBased(const std::string& meshDefinitionJson)
{
    Mesh<T> mesh;
    MeshFromJson(carbon::ReadJson(meshDefinitionJson), mesh);
    SetTarget(mesh);
}


template <class T>
void SmoothSurfaceProjection<T>::SetTargetMesh(const Eigen::VectorXi& polygons, const Eigen::VectorXi& vIDs, const Eigen::Matrix<T, 3, -1>& vertices)
{
    Mesh<T> mesh;
    mesh.SetTopology(polygons, vIDs);
    mesh.SetVertices(vertices);
    SetTarget(mesh);
}


template <class T>
void SmoothSurfaceProjection<T>::SetSourceMeshJsonBased(const std::string& meshDefinitionJson)
{
    MeshFromJson(carbon::ReadJson(meshDefinitionJson), m->sourceMesh);
    m->sourceMesh.Triangulate();
}


template <class T>
void SmoothSurfaceProjection<T>::SetSourceMesh(const Eigen::VectorXi& polygons, const Eigen::VectorXi& vIDs, const Eigen::Matrix<T, 3, -1>& vertices)
{
    m->sourceMesh = Mesh<T>();
    m->sourceMesh.SetTopology(polygons, vIDs);
    m->sourceMesh.SetVertices(vertices);
    m->sourceMesh.Triangulate();
}


template <class T>
std::string SmoothSurfaceProjection<T>::SmoothProjectionJsonBased(const std::string& perVertexWeightsJson,
                                                                  T projectionWeight,
                                                                  T point2pointWeight,
                                                                  T projectiveStrainWeight,
                                                                  T greenStrainWeight,
                                                                  T quadraticBendingWeight,
                                                                  T dihedralBendingWeight,
                                                                  int iterations)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

    Eigen::Matrix<T, -1, 1> perVertexWeights;
    FromJson(carbon::ReadJson(perVertexWeightsJson), perVertexWeights);
    Eigen::Matrix<T, 3, -1> vertices;
    vertices = SmoothProjection(perVertexWeights, projectionWeight, point2pointWeight, projectiveStrainWeight, greenStrainWeight, quadraticBendingWeight, dihedralBendingWeight, iterations);

    const carbon::JsonElement jOutput = ToJson2(vertices);
    return carbon::WriteJson(jOutput);
}


template <class T>
Eigen::Matrix<T, 3, -1> SmoothSurfaceProjection<T>::SmoothProjection(const Eigen::Matrix<T, -1, 1>& perVertexWeights,
                                                                     T projectionWeight,
                                                                     T point2pointWeight,
                                                                     T projectiveStrainWeight,
                                                                     T greenStrainWeight,
                                                                     T quadraticBendingWeight,
                                                                     T dihedralBendingWeight,
                                                                     int iterations)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

    MatrixVariable<T, 3, -1> varVertices(3, int(m->sourceMesh.NumVertices()));
    varVertices.SetMatrix(m->sourceMesh.Vertices());

    typename MeshCorrespondenceSearch<T>::Result correspondences;
    PointPointConstraintFunction<T, 3> pointConstraint;
    PointSurfaceConstraintFunction<T, 3> surfaceConstraint;

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {

        DiffDataMatrix<T, 3, -1> evaluatedVertices = varVertices.EvaluateMatrix(context);
        Cost<T> cost;

        if (projectionWeight > T(0)) {
            if (context || int(correspondences.targetVertices.cols()) != evaluatedVertices.Cols()) {
                m->sourceMesh.SetVertices(evaluatedVertices.Matrix());
                m->sourceMesh.CalculateVertexNormals();
                m->correspondenceSearch.Search(m->sourceMesh, correspondences);
            }
            const DiffData<T> residual = surfaceConstraint.Evaluate(evaluatedVertices, correspondences.targetVertices, correspondences.targetNormals, correspondences.weights, projectionWeight);
            cost.Add(residual, T(1));
        }
        if (point2pointWeight > T(0)){
            const DiffData<T> residual = pointConstraint.Evaluate(evaluatedVertices, m->sourceMesh.Vertices(), perVertexWeights, point2pointWeight);
            cost.Add(residual, T(1));
        }
        if (projectiveStrainWeight > 0) {
            const DiffData<T> strain = m->triangleStrain.EvaluateProjectiveStrain(evaluatedVertices, projectiveStrainWeight);
            cost.Add(strain, T(1));
        }
        if (greenStrainWeight > 0) {
            const DiffData<T> strain = m->triangleStrain.EvaluateGreenStrain(evaluatedVertices, greenStrainWeight);
            cost.Add(strain, T(1));
        }
        if (quadraticBendingWeight > 0) {
            const DiffData<T> bending = m->triangleBending.EvaluateQuadratic(evaluatedVertices, quadraticBendingWeight);
            cost.Add(bending, T(1));
        }
        if (dihedralBendingWeight > 0) {
            const DiffData<T> bending = m->triangleBending.EvaluateDihedral(evaluatedVertices, dihedralBendingWeight);
            cost.Add(bending, T(1));
        }

        return cost.CostToDiffData();
    };

    PROFILING_BLOCK("solve");
    GaussNewtonSolver<T> solver;
    const T startEnergy = evaluationFunction(nullptr).Value().norm();
    if (!solver.Solve(evaluationFunction, iterations)) {
        printf("could not solve optimization problem\n");
        return m->sourceMesh.Vertices();
    }
    const T finalEnergy = evaluationFunction(nullptr).Value().norm();
    printf("energy reduced from %f to %f\n", double(startEnergy), double(finalEnergy));
    PROFILING_END_BLOCK;

    return varVertices.Matrix();
}


// explicitly instantiate the joint rig optimization classes
template class SmoothSurfaceProjection<float>;
template class SmoothSurfaceProjection<double>;

} // namespace epic::nls
