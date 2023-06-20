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
#include <tracking/FlowConstraints.h>

#include <map>
#include <memory>
#include <string>

namespace epic {
namespace nls {

//! Debug Info for current state
template <class T>
struct FaceFittingConstraintsDebugInfo {
    typename MeshCorrespondenceSearch<T>::Result correspondences;
    LandmarkConstraintsData<T> landmarkConstraints;
    LandmarkConstraintsData<T> curveConstraints;
    LandmarkConstraintsData<T> lipConstraintsUpper;
    LandmarkConstraintsData<T> lipConstraintsLower;
};

/**
 * Module to align a template mesh with a high resolution 3D scan.
 *
 * Implemented for T=float and T=double
 */
template <class T>
class FaceFitting
{
public:
    FaceFitting();
    ~FaceFitting();
    FaceFitting(FaceFitting&& o);
    FaceFitting(const FaceFitting& o) = delete;
    FaceFitting& operator=(FaceFitting&& o);
    FaceFitting& operator=(const FaceFitting& o) = delete;

    //! get/set the model registration settings (identity PCA model)
    const Configuration& RigidRegistrationConfiguration() const { return m_rigidFittingConfig; }
    Configuration& RigidRegistrationConfiguration() { return m_rigidFittingConfig; }

    //! get/set the model registration settings (identity PCA model)
    const Configuration& ModelRegistrationConfiguration() const { return m_modelFittingConfig; }
    Configuration& ModelRegistrationConfiguration() { return m_modelFittingConfig; }

    //! get/set the fine registration settings (per-vertex deformation)
    const Configuration& FineRegistrationConfiguration() const { return m_fineFittingConfig; }
    Configuration& FineRegistrationConfiguration() { return m_fineFittingConfig; }

    void SetUserDefinedLandmarkAndCurveWeights(const std::map<std::string, T>& userDefinedLandmarkAndCurveWeights);

    //! Setup rest pose for eyeball constraint with current deformed mesh
    void SetupEyeballConstraint(const Mesh<T>& leftEyeball, const Mesh<T>& rightEyeball);

    //! Set the source mesh.
    void SetSourceMesh(const Mesh<T>& mesh);

    void SetSourceAndDeformedMesh(const Mesh<T>& baseMesh, const Mesh<T>& deformedMesh);

    //! Sets the model flow constraint vertex weights
    void SetModelFlowVertexWeights(const VertexWeights<T>& vertexWeights);

    //! Sets the uv flow constraint vertex weights
    void SetUVFlowVertexWeights(const VertexWeights<T>& vertexWeights);

    //! Sets the eyeball constraint vertex weights
    void SetEyeConstraintVertexWeights(const VertexWeights<T>& vertexWeightsLeftEye, const VertexWeights<T>& vertexWeightsRightEye);

    //! Sets the collision constraint masks
    void SetLipCollisionMasks(const VertexWeights<T>& maskUpperLip, const VertexWeights<T>& maskLowerLip);

    //! Sets the mesh landmarks that are use for registration
    void SetMeshLandmarks(const MeshLandmarks<T>& meshLandmarks);

    //! Set the target meshes from multiple frames
    void SetTargetMeshes(const std::vector<std::shared_ptr<const Mesh<T>>>& targetMeshes,
                         const std::vector<Eigen::VectorX<T>>& targetWeights);

    void SetTargetDepths(const std::vector<std::vector<std::shared_ptr<const DepthmapData<T>>>>& targetDepths);

    //! Set the target 2D landmarks
    void SetTarget2DLandmarks(const std::vector<std::vector<std::pair<LandmarkInstance<T, 2>, Camera<T>>>>& landmarks);

    //! Set the target 3D landmarks
    void SetTarget3DLandmarks(const std::vector<LandmarkInstance<T, 3>>& landmarks);

    //! Set model flow constraints
    void SetModelFlowConstraints(const std::map<std::string, FlowConstraintsData<T>>& flowConstraintsData);

    //! Set uv flow constraints
    void SetUVFlowConstraints(const std::map<std::string, FlowConstraintsData<T>>& flowConstraintsData);

    //! @see IdentityBlendModel::LoadModel()
    void LoadModel(const std::string& identityModelFile);

    //! Set the current deformed vertices
    void SetCurrentDeformedVertices(const Eigen::Matrix<T, 3, -1>& deformedVertices);

    //! @return the current deformed vertices
    const Eigen::Matrix<T, 3 ,-1>& CurrentDeformedVertices() const;

    std::map<std::string, Eigen::Vector3<T>> CurrentMeshLandmarks();

    /**
     * Run rigid registration.
     * @param sourceAffine                    The (current) affine transformation of the source mesh to the target mesh.
     * @param numIterations                   The number of iterations for rigid registration.
     */
    Affine<T, 3, 3> RegisterRigid(const Affine<T, 3, 3>& source2target,
                                  const VertexWeights<T>& searchWeights,
                                  int numIterations = 10,
                                  int scanFrame = 0);

    std::vector<Affine<T, 3, 3>> RegisterRigid(const std::vector<Affine<T, 3, 3>>& source2target,
                                               const VertexWeights<T>& searchWeights,
                                               int numIterations = 10);

    //! Resets the identity model parameters as well as the per vertex offsets
    void ResetNonrigid();

    /**
     * Nonrigid registration using the identity model (discard the per-vertex offsets)
     * @param sourceAffine                    The (current) affine transformation of the source mesh to the target mesh.
     * @param numIterations                   The number of iterations for rigid registration.
     */
    std::vector<Affine<T, 3, 3>> RegisterNonRigid(const std::vector<Affine<T, 3, 3>>& source2target,
                                                  const VertexWeights<T>& searchWeights,
                                                  int numIterations = 10);


    //! Resets the fine registration (per vertex offsets)
    void ResetFine();

    /**
     * Nonrigid registration with per-vertex displacement
     * @param source2target                   The (current) affine transformation of the source mesh to the target mesh.
     * @param numIterations                   The number of iterations for rigid registration.
     */
    std::vector<Affine<T, 3, 3>> RegisterFine(const std::vector<Affine<T, 3, 3>>& source2target,
                                              const VertexWeights<T>& searchWeights,
                                              int numIterations = 10);


    //! @return debug information to visualize the data constraints such as ICP and landmarks
    std::shared_ptr<const FaceFittingConstraintsDebugInfo<T>> CurrentDebugConstraints(const Affine<T, 3, 3>& source2target, int frame);

private:
    void InitIcpConstraints(int numOfObservations);
    void Init2DLandmarksConstraints(int numOfObservations);
    void Init3DLandmarksConstraints(int numOfObservations);

    //! Loads the vertices that are used for the initial correspondences search. It does *not* update the deformation model.
    void LoadInitialCorrespondencesVertices(const Eigen::Matrix<T, 3, -1>& sourceVertices);

    void UpdateIcpConfiguration(const Configuration& targetConfig);
    void Update2DLandmarkConfiguration(const Configuration& targetConfig);
    void Update3DLandmarkConfiguration(const Configuration& targetConfig);
    void UpdateIcpWeights(const VertexWeights<T>& weights);

private:
    Configuration m_rigidFittingConfig = { std::string("Rigid Fitting Configuration") , {
        // ! whether to use distance threshold
        {"useDistanceThreshold", ConfigurationParameter(false)},
        //!< regularization of model parameters
        { "geometryWeight", ConfigurationParameter(T(1), T(0), T(1)) },
        //!< how much weight to use on inner lip constraints
        { "innerLipWeight", ConfigurationParameter(T(0), T(0), T(0.1)) },
        //!< regularization of model parameters
        { "landmarksWeight", ConfigurationParameter(T(0.001), T(0), T(0.1)) },
        //!<  how much weight to use on 3d landmark constraint
        { "3DlandmarksWeight", ConfigurationParameter(T(100.0f), T(0.0f), T(200.0f)) },
        //!< how much weight to use on geometry constraint
        { "point2point", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< minimum distance threshold value - if used
        { "minimumDistanceThreshold", ConfigurationParameter(T(0.5), T(0), T(10)) },
        //!< resampling of curves
        { "curveResampling", ConfigurationParameter(1, 1, 5) }
    } };

    Configuration m_modelFittingConfig = {std::string("Model Fitting Configuration") , {
        // ! whether to use distance threshold
        {"useDistanceThreshold", ConfigurationParameter(true)},
        // ! whether to optimize the scale of the model
        {"optimizeScale", ConfigurationParameter(true)},
        //!< regularization of model parameters
        { "modelRegularization", ConfigurationParameter(T(100), T(0), T(1000)) },
        //!< how much weight to use on geometry constraint
        { "geometryWeight", ConfigurationParameter(T(1), T(0), T(1)) },
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< how much weight to use on landmark constraints
        { "landmarksWeight", ConfigurationParameter(T(0.01), T(0), T(0.1)) },
        //!<  how much weight to use on 3d landmark constraint
        { "3DlandmarksWeight", ConfigurationParameter(T(100.0f), T(0.0f), T(200.0f)) },
        //!< how much weight to use on inner lip constraints
        { "innerLipWeight", ConfigurationParameter(T(0.01), T(0), T(0.1)) },
        //!< weight on smoothness between patches i.e. neighoring patches should evaluate to the same vertex position
        { "patchSmoothness", ConfigurationParameter(T(1), T(0), T(10))},
        //!< minimum distance threshold value - if used
        { "minimumDistanceThreshold", ConfigurationParameter(T(5), T(0), T(10)) },
        //!< only to use user landmarks while solving
        { "justUserLandmarks", ConfigurationParameter(false) },
        //!< resampling of curves
        { "curveResampling", ConfigurationParameter(1, 1, 5) }
    }};

    Configuration m_fineFittingConfig = {std::string("Fine Fitting Configuration") , {
        // ! whether to use distance threshold
        {"useDistanceThreshold", ConfigurationParameter(true)},
        //!< whether to optimize the pose when doing fine registration
        { "optimizePose", ConfigurationParameter(false) },
        //!< whether to use model optical flow constraint in fitting
        { "useModelOpticalFlow", ConfigurationParameter(false) },
        //!< whether to use uv optical flow constraint in fitting
        { "useUVOpticalFlow", ConfigurationParameter(false) },
        //!< whether to use eyeball constraint when fitting
        { "useEyeballConstraint", ConfigurationParameter(false) },
        //!< projective strain weight (stable, but incorrect Jacobian)
        { "projectiveStrain", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< green strain (unstable???, correct Jacobian)
        { "greenStrain", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< quadratic bending (stable, but incorrect Jacobian, and also has strain component)
        { "quadraticBending", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< dihedral bending (unstable???, correct Jacobian)
        { "dihedralBending", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< weight on regularizing the per-vertex offset
        { "vertexOffsetRegularization", ConfigurationParameter(T(0.01), T(0), T(1)) },
        //!< weight on regularizing the per-vertex offset
        { "vertexLaplacian", ConfigurationParameter(T(1.0), T(0), T(1)) },
        //!< how much weight to use on geometry constraint
        { "geometryWeight", ConfigurationParameter(T(1), T(0), T(1)) },
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(0.1), T(0), T(1)) },
        //!< whether to sample the scan instead of the model for constraints
        { "sampleScan", ConfigurationParameter(false) },
        //!< how much weight to use on landmark constraints
        { "landmarksWeight", ConfigurationParameter(T(0.01), T(0), T(0.1)) },
        //!<  how much weight to use on 3d landmark constraint
        { "3DlandmarksWeight", ConfigurationParameter(T(100.0f), T(0.0f), T(200.0f)) },
        //!< how much weight to use on inner lip constraints
        { "innerLipWeight", ConfigurationParameter(T(0.01), T(0), T(0.1)) },
        //!< resampling of curves
        { "curveResampling", ConfigurationParameter(5, 1, 5) },
        //!< model flow weight for expression fitting
        { "modelFlowWeight", ConfigurationParameter(T(1e-2), T(1e-5), T(0.1)) },
        //!< uv flow weight for expression fitting
        { "uvFlowWeight", ConfigurationParameter(T(1e-2), T(1e-5), T(0.1)) },
        //!< only to use user landmarks while solving
        { "justUserLandmarks", ConfigurationParameter(false) },
        //!< minimum distance threshold value - if used
        { "minimumDistanceThreshold", ConfigurationParameter(T(2), T(0), T(10)) },
        //!< weight for collisions
        { "collisionWeight", ConfigurationParameter(T(0), T(0), T(1)) }
    }};

    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace nls
} //namespace epic
