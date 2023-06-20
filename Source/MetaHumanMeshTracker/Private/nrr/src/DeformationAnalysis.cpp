// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/DeformationAnalysis.h>

#include <nls/geometry/Mesh.h>
#include <nls/geometry/TriangleStrain.h>
#include <nls/geometry/TriangleBending.h>
#include <nls/serialization/EigenSerialization.h>
#include <nls/serialization/MeshSerialization.h>
#include <carbon/io/JsonIO.h>

namespace epic {
namespace nls {

template <class T>
struct DeformationAnalysis<T>::Private
{
    Mesh<T> mesh;
};


template <class T>
DeformationAnalysis<T>::DeformationAnalysis() : m(std::make_unique<Private>())
{
}

template <class T> DeformationAnalysis<T>::~DeformationAnalysis() = default;
template <class T> DeformationAnalysis<T>::DeformationAnalysis(DeformationAnalysis&&) = default;
template <class T> DeformationAnalysis<T>& DeformationAnalysis<T>::operator=(DeformationAnalysis&&) = default;


template <class T>
void DeformationAnalysis<T>::SetMeshJsonBased(const std::string& meshDefinitionJson)
{
    MeshFromJson(carbon::ReadJson(meshDefinitionJson), m->mesh);
    m->mesh.Triangulate();
}

template <class T>
std::string DeformationAnalysis<T>::AnalyseDeformationJsonBased(const std::string& vertices1Json,
                                                                const std::string& vertices2Json)
{
    const carbon::JsonElement jVertices1 = carbon::ReadJson(vertices1Json);
    const carbon::JsonElement jVertices2 = carbon::ReadJson(vertices2Json);

    Eigen::Matrix<T, 3, -1> vertices1;
    Eigen::Matrix<T, 3, -1> vertices2;

    FromJson(jVertices1, vertices1);
    FromJson(jVertices2, vertices2);

    if (vertices1.cols() != vertices2.cols()) {
        throw std::runtime_error("incompatible number of vertices");
    }

    // distance
    const Eigen::VectorX<T> distances = (vertices1 - vertices2).colwise().norm();

    // strain
    TriangleStrain<T> triangleStrain;
    triangleStrain.SetTopology(m->mesh.Triangles());
    triangleStrain.SetRestPose(vertices1);
    const T projectiveStrainNorm = triangleStrain.EvaluateProjectiveStrain(vertices2, T(1)).Value().norm();
    const T greenStrainNorm = triangleStrain.EvaluateGreenStrain(vertices2, T(1)).Value().norm();
    const Eigen::VectorX<T> verticesProjectiveStrain = triangleStrain.EvaluateProjectiveStrainPerVertex(vertices2);
    const Eigen::VectorX<T> verticesGreenStrain = triangleStrain.EvaluateGreenStrainPerVertex(vertices2);


    // bending
    TriangleBending<T> triangleBending;
    triangleBending.SetTopology(m->mesh.Triangles());
    triangleBending.SetRestPose(vertices1);
    const T quadraticBendingNorm = triangleBending.EvaluateQuadratic(vertices2, T(1)).Value().norm();
    const T dihedralBendingNorm = triangleBending.EvaluateDihedral(vertices2, T(1)).Value().norm();
    const Eigen::VectorX<T> verticesQuadraticBending = triangleBending.EvaluateQuadraticPerVertex(vertices2);
    const Eigen::VectorX<T> verticesDihedralBending = triangleBending.EvaluateDihedralPerVertex(vertices2);


    carbon::JsonElement jOutputDict(carbon::JsonElement::JsonType::Object);
    jOutputDict.Insert("distances", ToJson2(distances));
    jOutputDict.Insert("projective strain", ToJson2(verticesProjectiveStrain));
    jOutputDict.Insert("green strain", ToJson2(verticesGreenStrain));
    jOutputDict.Insert("quadratic bending", ToJson2(verticesQuadraticBending));
    jOutputDict.Insert("dihedral", ToJson2(verticesDihedralBending));
    // add the norms as the visualization is on the vertices
    jOutputDict.Insert("projective strain norm", carbon::JsonElement(projectiveStrainNorm));
    jOutputDict.Insert("green strain norm", carbon::JsonElement(greenStrainNorm));
    jOutputDict.Insert("quadratic bending norm", carbon::JsonElement(quadraticBendingNorm));
    jOutputDict.Insert("dihedral bending norm", carbon::JsonElement(dihedralBendingNorm));

    return carbon::WriteJson(jOutputDict);
}


// explicitly instantiate the joint rig optimization classes
template class DeformationAnalysis<float>;
template class DeformationAnalysis<double>;

} // namespace nls
} //namespace epic
