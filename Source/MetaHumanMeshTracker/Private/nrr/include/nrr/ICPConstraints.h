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
#include <nls/utils/ConfigurationParameter.h>
#include <nrr/VertexWeights.h>


namespace epic::nls {

/**
 * Class to evaluate ICP correspondences (point2point and/or point2surface)
 */
template <class T>
class ICPConstraints
{
public:
    static constexpr const char* ConfigName() { return "ICP Constraints Configuration"; }

    ICPConstraints();
    ~ICPConstraints();
    ICPConstraints(ICPConstraints&&);
    ICPConstraints(ICPConstraints&) = delete;
    ICPConstraints& operator=(ICPConstraints&&);
    ICPConstraints& operator=(const ICPConstraints&) = delete;

    //! Sets the correspondence search vertex weights
    void SetCorrespondenceSearchVertexWeights(const VertexWeights<T>& vertexWeights);

    //! Set the target mesh
    void SetTargetMesh(const Mesh<T>& targetMesh);

    //! @returns the target vertex weights for the target mesh.
	const Eigen::Vector<T, -1>& TargetWeights() const;

	//! Set custom target vertex weights for the target mesh.
	void SetTargetWeights(const Eigen::Vector<T, -1>& targetWeights);

    //! Set the target depthmap and normals
    void AddTargetDepthAndNormals(const std::shared_ptr<const DepthmapData<T>>& depthmapData);

    const Configuration& GetConfiguration() const;

    void SetConfiguration(const Configuration& config, bool ignoreUnknownKeys = false);

    //! Clears any correspondences.
    void ClearPreviousCorrespondences();

    //! @returns True if ICP correspondences have been set up using SetupCorrespondences() or EvaluateICP()
    bool HasCorrespondences() const;

    //! Find the correspondences and store the result internally (which is then used for EvaluateICP)
    void SetupCorrespondences(const Eigen::Matrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals);

    /**
     * Evaluate the ICP constraint.
     * @param[in] vertices  The differentiable input vertices.
     */
    Cost<T> EvaluateICP(const DiffDataMatrix<T, 3, -1>& vertices);

    /**
     * Evaluate the ICP constraint. First finds correspondences if @p searchCorrespondences is True or if no correspondences hav been set up.
     * @param[in] vertices  The differentiable input vertices.
     * @param[in] normals   The normals that should be used when searching correspondences (only needs to be set when searchCorrespondences=true)
     * @param[in] searchCorrespondences  If True then new correspendences are found using SetupCorrespondences, otherwise the current correspondences are used.
     */
    Cost<T> EvaluateICP(const DiffDataMatrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals, bool searchCorrespondences);

    //! Finds the correspondences and stores the result in @p correspondences
    void FindCorrespondences(const Eigen::Matrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals, typename MeshCorrespondenceSearch<T>::Result& correspondences) const;

    /**
     * Alternative way to set up ICP constraints
     */
    void SetupICPConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                             const Eigen::Matrix<T, 3, -1>& vertices,
                             const Eigen::Matrix<T, 3, -1>& normals,
                             VertexConstraints<T, 1, 1>& point2SurfaceVertexConstraints,
                             VertexConstraints<T, 3, 1>& point2PointVertexConstraints);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
