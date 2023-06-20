// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/Cost.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/Camera.h>
#include <nls/geometry/DepthmapData.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/MeshCorrespondenceSearch.h>
#include <nls/geometry/VertexConstraints.h>
#include <nls/utils/Configuration.h>
#include <nrr/VertexWeights.h>

namespace epic::nls {

/**
 * Class to evaluate ICP correspondences (point2point and/or point2surface)
 */
template <class T>
class GeometryConstraints
{
public:
    GeometryConstraints();
    ~GeometryConstraints();
    GeometryConstraints(GeometryConstraints&& o);
    GeometryConstraints(const GeometryConstraints& o) = delete;
    GeometryConstraints& operator=(GeometryConstraints&& o);
    GeometryConstraints& operator=(const GeometryConstraints& o) = delete;

    //! Sets the correspondence search vertex weights
    void SetSourceWeights(const VertexWeights<T>& vertexWeights);

    //! Set the target mesh
    void SetTargetMesh(const std::shared_ptr<const Mesh<T>>& targetMesh);

    //! Set the target depthmap and normals
    void AddTargetDepthAndNormals(const std::shared_ptr<const DepthmapData<T>>& depthData);

	//! Set custom target vertex weights for the target mesh.
	void SetTargetWeights(const Eigen::Vector<T, -1>& targetWeights);

    const Configuration& GetConfiguration() const;

    void SetConfiguration(const Configuration& config, bool ignoreUnknownKeys = false);

    //! Clears any correspondences.
    void ClearPreviousCorrespondences();

    //! @returns True if ICP correspondences have been set up using SetupCorrespondences() or EvaluateICP()
    bool HasCorrespondences() const;

    //! Find the correspondences and store the result internally (which is then used for EvaluateICP)
    void SetupCorrespondences(const Mesh<T>& sourceMesh, bool useTarget2Source);

    /**
     * Evaluate the ICP constraint.
     * @param[in] vertices  The differentiable input vertices.
     */
    Cost<T> EvaluateICP(const DiffDataMatrix<T, 3, -1>& vertices);

    //! Finds the correspondences and stores the result in @p correspondences
    void FindCorrespondences(const Eigen::Matrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals, typename MeshCorrespondenceSearch<T>::Result& correspondences) const;

    //! Creates the constraints
    void SetupConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                          const Eigen::Matrix<T, 3, -1>& vertices,
                          const Eigen::Matrix<T, 3, -1>& normals,
                          VertexConstraints<T, 1, 1>& point2SurfaceVertexConstraints,
                          VertexConstraints<T, 3, 1>& point2PointVertexConstraints);
private:
    void FindTarget2SourceCorrespondences(const Mesh<T>& sourceMesh);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
