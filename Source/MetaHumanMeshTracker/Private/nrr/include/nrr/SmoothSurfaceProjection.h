// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/math/Math.h>
#include <nls/geometry/Mesh.h>

#include <string>
#include <vector>

namespace epic::nls {

/**
 * The SmoothSurfaceProjection class is used to project a source mesh back onto a target
 * surface while also staying as close as possible to the source starting position.
 * As an additional constrain the triangles should be as similar as possible to the target
 * surface in terms of stretching and bending.
 */
template <class T>
class SmoothSurfaceProjection
{
public:
    SmoothSurfaceProjection();
    ~SmoothSurfaceProjection();
    SmoothSurfaceProjection(SmoothSurfaceProjection&& other);
    SmoothSurfaceProjection(const SmoothSurfaceProjection& other) = delete;
    SmoothSurfaceProjection& operator=(SmoothSurfaceProjection&& other);
    SmoothSurfaceProjection& operator=(const SmoothSurfaceProjection& other) = delete;

    /**
     * Set projection and smoothness to be relative to the target pose.
     */
    void SetTarget(Mesh<T>& mesh);

    /**
     * Set the mesh using a json-based mesh definition.
     * @param meshDefinitionJson  The definition of the mesh in json format (@see MeshSerialization.h)
     */
    void SetTargetMeshJsonBased(const std::string& meshDefinitionJson);

    /**
     * Set the mesh
     * @param polygons      The number of vertices per polygons (only 3 for tris and 4 for quads are supported)
     * @param vIDs          The vertex IDs for the \p polygons (concatenated)
     * @param vertices      The vertices of the target mesh
     */
    void SetTargetMesh(const Eigen::VectorXi& polygons, const Eigen::VectorXi& vIDs, const Eigen::Matrix<T, 3, -1>& vertices);

    /**
    * Set the mesh using a json-based mesh definition.
    * @param meshDefinitionJson  The definition of the mesh in json format (@see MeshSerialization.h)
    */
    void SetSourceMeshJsonBased(const std::string& meshDefinitionJson);

    /**
     * Set the mesh
     * @param polygons      The number of vertices per polygons (only 3 for tris and 4 for quads are supported)
     * @param vIDs          The vertex IDs for the \p polygons (concatenated)
     * @param vertices      The vertices of the source mesh
     */
    void SetSourceMesh(const Eigen::VectorXi& polygons, const Eigen::VectorXi& vIDs, const Eigen::Matrix<T, 3, -1>& vertices);

    /**
     * @param perVertexWeightsJson      The vertcies weights used to influence whether vertices should be close to the source position
     *                                  or project back onto the target surface.
     *
     * @return Projects the vertices while also stays close as possible to the source position such the resulting geometry is smooth
     *         with respect to stretching and bending constraints.
     */
    std::string SmoothProjectionJsonBased(const std::string& perVertexWeightsJson,
                                          T projectionWeight,
                                          T point2pointWeight,
                                          T projectiveStrainWeight,
                                          T greenStrainWeight,
                                          T quadraticBendingWeight,
                                          T dihedralBendingWeight,
                                          int iterations);
    /**
     * @param perVertexWeights      The vertcies weights used to influence whether vertices should be close to the source position or project back onto the target surface.
     *                              or project back onto the target surface.
     *
     * @return Projects the vertices while also stays close as possible to the source position such the resulting geometry is smooth
     *         with respect to stretching and bending constraints.
     */
    Eigen::Matrix<T, 3, -1> SmoothProjection(const Eigen::Matrix<T, -1, 1>& perVertexWeights,
                                             T projectionWeight,
                                             T point2pointWeight,
                                             T projectiveStrainWeight,
                                             T greenStrainWeight,
                                             T quadraticBendingWeight,
                                             T dihedralBendingWeight,
                                             int iterations);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
