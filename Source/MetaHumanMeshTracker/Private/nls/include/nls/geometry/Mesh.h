// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

#include <vector>
namespace epic::nls {

enum class VertexNormalComputationType {
    AreaWeighted,
    VoronoiAreaWeighted
};

/**
 * Mesh class representing quad and triangle meshes.
 *
 * Important note: Texture coordinates are stored with the origin being at the top left corner. Any import/export of
 *                 meshes, e.g. when writing OBJs, needs to take this into account and flip the y coordinate accordingly.
 *                 The reason to store texture coordinates with the top left corner is so that this is consistent with
 *                 image, windows, and normalized coordinates (Vulkan).
 */
template <class T>
class Mesh
{
public:
  	Mesh() = default;
	Mesh(const Mesh& o) = default;
	Mesh& operator=(const Mesh& o) = default;

    int NumVertices() const { return int(m_vertices.cols()); }
    int NumTriangles() const { return int(m_tris.cols()); }
    int NumQuads() const { return int(m_quads.cols()); }
    const Eigen::Matrix<T, 3, -1>& Vertices() const { return m_vertices; }
    //! Sets the vertices and clears the normals
    void SetVertices(const Eigen::Matrix<T, 3, -1>& vertices) { m_vertices = vertices; ClearVertexNormals(); }
    const Eigen::Matrix<int, 3, -1>& Triangles() const { return m_tris; }
    //! Sets the triangles and clears the normals
    void SetTriangles(const Eigen::Matrix<int, 3, -1>& tris) { m_tris = tris; ClearVertexNormals();  }
    const Eigen::Matrix<int, 4, -1>& Quads() const { return m_quads; }
    //! Sets the quads and clears the normals
    void SetQuads(const Eigen::Matrix<int, 4, -1>& quads) { m_quads = quads; ClearVertexNormals();  }

    /**
     * Set the topology using the polygon format (as used in Maya).
     * @pre There should not be any prior tris and quads assigned to the mesh.
     * @param polygons      The number of vertices per polygons (only 3 for tris and 4 for quads are supported)
     * @param vIDs          The vertex IDs for the \p polygons (concatenated)
     */
    void SetTopology(const Eigen::VectorXi& topology, const Eigen::VectorXi& vIDs);

    /**
     * Validates that all indices are correct (in bound)
     * @checkVertexUsage  Checks whether all vertices are being used
     */
    bool Validate(bool checkVertexUsage) const;

    /**
     * Triangulates the Mesh. Quads will be empty afterwards.
     */
    void Triangulate();

    //! Calculates the triangle areas. (only works for pure triangle meshes)
    Eigen::Vector<T, -1> TriangleAreas() const;

    /**
     * Calculates the vertex areas (only works for pure triangle meshes) using the mixed area term as described in Fig. 4
     * Meyer et al, "Discrete Differential-Geometry Operators for Triangulated 2-Manifolds")
     */
    Eigen::Vector<T, -1> VertexAreas() const;

    /**
     * Calculates the vertex mean curvature normal based on
     * Meyer et al, "Discrete Differential-Geometry Operators for Triangulated 2-Manifolds").
     * The mean curvature value is half the magnitude.
     */
    Eigen::Matrix<T, 3, -1> VertexMeanCurvatureNormals() const;

    //! @returns True if the mesh has vertex normals
    bool HasVertexNormals() const { return m_normals.cols() == m_vertices.cols(); }

    //! Clears the current normals
    void ClearVertexNormals() { m_normals.resize(3, 0); }

    /**
     * Calculates the vertex normals weighted by the mixed area term as described in Fig. 4
     * Meyer et al, "Discrete Differential-Geometry Operators for Triangulated 2-Manifolds")
     * @param recompute  If false then the normals are only calculated if they are not already there, if True then calculation is forced
     */
    const Eigen::Matrix<T, 3, -1>& CalculateVertexNormals(bool recompute = false);

    /**
     * Calculates the vertex normals using the topology of this mesh and the vertices of @p vertices. Stores the result
     * in @p normals.
     * @param[in] accurate  If True, it uses the mixed area term (see above), otherwise it uses a faster version by weighing using the triangle area.
     */
    void CalculateVertexNormals(const Eigen::Matrix<T, 3, -1>& vertices,
                                Eigen::Matrix<T, 3, -1>& normals,
                                VertexNormalComputationType vertexNormalComputationType,
                                bool stableNormalize) const;

    //! @returns the vertex normals
    const Eigen::Matrix<T, 3, -1>& VertexNormals() const { return m_normals; };

    //! Sets the vertex normals.
    void SetVertexNormals(const Eigen::Matrix<T, 3, -1>& vertexNormals) { m_normals = vertexNormals; }

    //! @returns the border vertices of the mesh
    std::vector<int> CalculateBorderVertices() const;

    //! @returns True if the mesh has UV coordinates
    bool HasTexcoords() const { return m_texcoords.size() > 0; }

    int NumTexcoords() const { return int(m_texcoords.cols()); }
    int NumTexTriangles() const { return int(m_tex_tris.cols()); }
    int NumTexQuads() const { return int(m_tex_quads.cols()); }

    //! Sets the texture coordinates. Note that the origin of texture coordinates is at the top left corner (as opposed to how it is stored in OBJs)
    void SetTexcoords(const Eigen::Matrix<T, 2, -1>& texcoords) { m_texcoords = texcoords; }
    //! @returns the texture coordinates. Note that the origin of texture coordinates is at the top left corner (as opposed to how it is stored in OBJs)
    const Eigen::Matrix<T, 2, -1>& Texcoords() const { return m_texcoords; }

    const Eigen::Matrix<int, 3, -1>& TexTriangles() const { return m_tex_tris; }
    void SetTexTriangles(const Eigen::Matrix<int, 3, -1>& tris) { m_tex_tris = tris;  }
    const Eigen::Matrix<int, 4, -1>& TexQuads() const { return m_tex_quads; }
    void SetTexQuads(const Eigen::Matrix<int, 4, -1>& quads) { m_tex_quads = quads;  }

    //! @returns all the edges of the mesh. If vIDs.size() > 0 then only edges which are part of vIDs are returned.
    std::vector<std::pair<int, int>> GetEdges(const std::vector<int>& vIDs) const;

    template <class S>
    Mesh<S> Cast() const
    {
        Mesh<S> other;
        other.SetVertices(m_vertices.template cast<S>());
        other.SetTriangles(m_tris);
        other.SetQuads(m_quads);
        other.SetVertexNormals(m_normals.template cast<S>());
        other.SetTexcoords(m_texcoords.template cast<S>());
        other.SetTexTriangles(m_tex_tris);
        other.SetTexQuads(m_tex_quads);
        return other;
    }

    /**
     * @brief Resample the mesh such that new vertex i will correspond to previous vertex newToOldMap[i]. In the current implementation texture coordinates and texture faces are discarded.
     * @pre newToOldMap Each new vertex needs to point to an existing vertex, and it needs to be unique.
     */
    void Resample(const std::vector<int>& newToOldMap);

private:
    Eigen::Matrix<T, 3, -1> m_vertices;
	Eigen::Matrix<int, 3, -1> m_tris;
	Eigen::Matrix<int, 4, -1> m_quads;

    //! the normals of the mesh (one normal per vertex)
    Eigen::Matrix<T, 3, -1> m_normals;

    //! UVs of the mesh (may be more than one normal per vertex)
    Eigen::Matrix<T, 2, -1> m_texcoords;
    Eigen::Matrix<int, 3, -1> m_tex_tris;
    Eigen::Matrix<int, 4, -1> m_tex_quads;
};


} // namespace epic::nls
