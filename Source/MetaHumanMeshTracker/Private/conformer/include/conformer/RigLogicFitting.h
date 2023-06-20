// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/Camera.h>
#include <nls/geometry/DepthmapData.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/MeshCorrespondenceSearch.h>
#include <nls/utils/Configuration.h>
#include <nrr/MeshLandmarks.h>
#include <nrr/VertexWeights.h>
#include <nrr/landmarks/LandmarkConfiguration.h>
#include <nrr/landmarks/LandmarkConstraints2D.h>
#include <nrr/landmarks/LandmarkConstraints3D.h>
#include <nrr/landmarks/LandmarkInstance.h>
#include <nrr/deformation_models/DeformationModelRigLogic.h>

#include <dna/StreamReader.h>

#include <map>
#include <memory>
#include <string>

namespace epic::nls {

/**
 * Module to align a template eye mesh with target landmarks.
 *
 * Implemented for T=float and T=double
 */
template <class T>
class RigLogicFitting
{
public:
    RigLogicFitting();
    ~RigLogicFitting();
    RigLogicFitting(RigLogicFitting&& o);
    RigLogicFitting(const RigLogicFitting& o) = delete;
    RigLogicFitting& operator=(RigLogicFitting&& o);
    RigLogicFitting& operator=(const RigLogicFitting& o) = delete;

    void LoadRig(dna::StreamReader* dnaRig);

    void SetRig(std::shared_ptr<Rig<T>> rig);

    //! get/set the model registration settings (identity PCA model)
    const Configuration& RigLogicRegistrationConfiguration() const { return m_rigLogicFittingConfig; }
    Configuration& RigLogicRegistrationConfiguration() { return m_rigLogicFittingConfig; }

    //! Set the target mesh
    void SetTargetMeshes(const std::vector<std::shared_ptr<const Mesh<T>>>& targetMeshes,
                            const std::vector<Eigen::VectorX<T>>& targetWeights);

    //! Set the target depth
    void SetTargetDepths(const std::vector<std::vector<std::shared_ptr<const DepthmapData<T>>>> & targetDepths);

    //! Set starting controls
    void SetGuiControls(const Eigen::VectorX<T>& currentControls);

    //! Sets the mesh landmarks that are use for registration
    void SetMeshLandmarks(const MeshLandmarks<T>& meshLandmarks);

    //! Set the target 2D landmarks
    void SetTarget2DLandmarks(const std::vector<std::vector<std::pair<LandmarkInstance<T, 2>, Camera<T>>>>& landmarks);

    //! Set the target 3D landmarks
    void SetTarget3DLandmarks(const std::vector<LandmarkInstance<T, 3>>& landmarks);

    //! Set the lip collision vertices mask
    void SetLipCollisionMasks(const VertexWeights<T>& maskUpperLip, const VertexWeights<T>& maskLowerLip);

    //! @return the estimated gui riglogic controls
    Eigen::VectorX<T> CurrentGuiControls() const;

    //! @return the estimated vertices
    Eigen::Matrix<T, 3, -1> CurrentVertices();

    /**
     * Nonrigid registration using the identity model (discard the per-vertex offsets)
     * @param initialCorrespondencesVertices  The vertices that should be used for the initial correspondences search.
     * @param sourceAffine                    The (current) affine transformation of the source mesh.
     * @param targetAffine                    The target affine transformation of the target mesh.
     * @param numIterations                   The number of iterations for registration.
     */
    std::vector<Affine<T, 3, 3>> RegisterRigLogic(const std::vector<Affine<T, 3, 3>>& source2target,
                                                    const VertexWeights<T>& searchWeights,
                                                    int numIterations = 10);

private:
    void InitIcpConstraints(int numOfObservations);
    void Init2DLandmarksConstraints(int numOfObservations);
    void Init3DLandmarksConstraints(int numOfObservations);

    void UpdateIcpConfiguration(const Configuration& targetConfig);
    void Update2DLandmarkConfiguration(const Configuration& targetConfig);
    void Update3DLandmarkConfiguration(const Configuration& targetConfig);
    void UpdateIcpWeights(const VertexWeights<T>& weights);

private:
    Configuration m_rigLogicFittingConfig = { std::string("RigLogic Fitting Configuration") , {
        // ! whether to use distance threshold
        {"useDistanceThreshold", ConfigurationParameter(true)},
        // ! whether to optimize rigid transform
        {"optimizePose", ConfigurationParameter(true)},
        //!< how much weight to use on geometry constraint
        { "geometryWeight", ConfigurationParameter(T(1), T(0), T(1)) },
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< how much weight to use on landmark constraints
        { "landmarksWeight", ConfigurationParameter(T(0.001), T(0), T(0.1)) },
        //!< how much weight to use on landmark constraints
        { "3DlandmarksWeight", ConfigurationParameter(T(100.0f), T(50.0f), T(200.0f)) },
        //!< how much weight to use on inner lip constraints
        { "innerLipWeight", ConfigurationParameter(T(0.01), T(0), T(0.1)) },
        //!< minimum distance threshold value - if used
        { "minimumDistanceThreshold", ConfigurationParameter(T(5), T(0), T(10)) },
        //!< minimum distance threshold value - if used
        { "l1regularization", ConfigurationParameter(T(1), T(0), T(10)) },
        //!< lip colision weight
        { "collisionWeight", ConfigurationParameter(T(1), T(0), T(10)) },
        //!< resampling of curves
        { "curveResampling", ConfigurationParameter(1, 5, 1) }
    } };

    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
