// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic {
namespace nls {

/**
 * Class representing tetrahedral meshes.
 *
 */
template <class T>
class TetMesh
{
public:
  	TetMesh() = default;
	TetMesh(const TetMesh& o) = default;
	TetMesh& operator=(const TetMesh& o) = default;

    int NumVertices() const { return int(m_vertices.cols()); }
    int NumTets() const { return int(m_tets.cols()); }

    const Eigen::Matrix<T, 3, -1>& Vertices() const { return m_vertices; }
    void SetVertices(const Eigen::Matrix<T, 3, -1>& vertices) { m_vertices = vertices; }

    const Eigen::Matrix<int, 4, -1>& Tets() const { return m_tets; }
    void SetTets(const Eigen::Matrix<int, 4, -1>& tets) { m_tets = tets; }

    //! Loads m_vertices and m_tets from NPY files
    void LoadFromNPY(std::string vertices_fname, std::string tets_fname);

    //! Saves m_vertices and m_tets to NPY files
    void SaveToNPY(std::string vertices_fname, std::string tets_fname) const;

    void BoundingBox(Eigen::Vector3<T>& bbmin, Eigen::Vector3<T>& bbmax) const;

    //! Sets to false the tetMask tets which have at least one vertex below a plane specified by normal and offset
    void CropByPlane(std::vector<bool>& tetMask, const Eigen::Vector<T, 3>& normal, T offset) const;

    //! Sets a full visualization mesh: 12 triangles per tetrahedron
    void FullVisualizationMesh(Eigen::Matrix<T, 3, -1>& visVertices, Eigen::Matrix<int, 3, -1>& visTriangles) const;

    //! Sets a boundary visualization mesh: one triangle per boundary face. tetPresent can flag tets to be ignored.
    void BoundaryMesh(Eigen::Matrix<T, 3, -1>& visVertices, Eigen::Matrix<int, 3, -1>& visTriangles, std::vector<bool> *tetPresent = nullptr) const;

    void TetVolumeStatistics(T& minVol, T& avgVol, T& maxVol, bool absValue = false) const;

    Eigen::VectorX<T> TetVolumes() const;

private:
    Eigen::Matrix<T, 3, -1> m_vertices;
	Eigen::Matrix<int, 4, -1> m_tets;
};

} // namespace nls
} //namespace epic
