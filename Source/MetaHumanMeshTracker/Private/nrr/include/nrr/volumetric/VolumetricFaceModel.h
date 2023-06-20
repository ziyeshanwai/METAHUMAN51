// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/BarycentricEmbedding.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/TetMesh.h>
#include <nrr/VertexWeights.h>

#include <string>

namespace epic::nls {

template<class T>
class VolumetricFaceModel {
public:
    bool Load(const std::string& directory);
    bool Save(const std::string& directory) const;

    const Mesh<T>& GetSkinMesh() const { return m_skinMesh; }
    const Mesh<T>& GetFleshMesh() const { return m_fleshMesh; }
    const Mesh<T>& GetCraniumMesh() const { return m_craniumMesh; }
    const Mesh<T>& GetMandibleMesh() const { return m_mandibleMesh; }
    const Mesh<T>& GetTeethMesh() const { return m_teethMesh; }
    const TetMesh<T>& GetTetMesh() const { return m_tetMesh; }
    const BarycentricEmbedding<T>& Embedding() const { return m_embedding; }
    const std::vector<std::pair<int, int> >& SkinFleshMapping() const { return m_skinFleshMapping; }
    const std::vector<std::pair<int, int> >& CraniumFleshMapping() const { return m_craniumFleshMapping; }
    const std::vector<std::pair<int, int> >& MandibleFleshMapping() const { return m_mandibleFleshMapping; }
    const VertexWeights<T>& LowerTeethRigidityMask() const { return m_lowerTeethRigidityMask; }
    const VertexWeights<T>& UpperTeethRigidityMask() const { return m_upperTeethRigidityMask; }

    void SetSkinMeshVertices(const Eigen::Matrix<T, 3, -1>& skinVertices);
    void SetFleshMeshVertices(const Eigen::Matrix<T, 3, -1>& fleshVertices);
    void SetCraniumMeshVertices(const Eigen::Matrix<T, 3, -1>& craniumVertices);
    void SetMandibleMeshVertices(const Eigen::Matrix<T, 3, -1>& mandibleVertices);
    void SetTeethMeshVertices(const Eigen::Matrix<T, 3, -1>& teethVertices);
    void SetTetMeshVertices(const Eigen::Matrix<T, 3, -1>& tetVertices);

    void UpdateFleshMeshVerticesFromSkinCraniumAndMandible();

private:
    Mesh<T> m_skinMesh;
    Mesh<T> m_fleshMesh;
    Mesh<T> m_craniumMesh;
    Mesh<T> m_mandibleMesh;
    Mesh<T> m_teethMesh;

    TetMesh<T> m_tetMesh;

    BarycentricEmbedding<T> m_embedding;

    std::vector<std::pair<int, int> > m_skinFleshMapping;
    std::vector<std::pair<int, int> > m_craniumFleshMapping;
    std::vector<std::pair<int, int> > m_mandibleFleshMapping;

    VertexWeights<T> m_lowerTeethRigidityMask; // !< mask on which to calculate the rigid motion of the lowe rteeth
    VertexWeights<T> m_upperTeethRigidityMask; // !< mask on which to calculate the rigid motion of the upper teeth
};

}  // namespace epic::nls
