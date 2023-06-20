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
class IdentityModelFitting
{
public:
    IdentityModelFitting();
    ~IdentityModelFitting();
    IdentityModelFitting(IdentityModelFitting&& o);
    IdentityModelFitting(const IdentityModelFitting& o) = delete;
    IdentityModelFitting& operator=(IdentityModelFitting&& o);
    IdentityModelFitting& operator=(const IdentityModelFitting& o) = delete;

    void LoadModel(const std::string& identityModelFile);

    //! get/set the rigid registration settings
    const Configuration& RigidRegistrationConfiguration() const { return m_rigidFittingConfig; }
    Configuration& RigidRegistrationConfiguration() { return m_rigidFittingConfig; }

    //! get/set the model registration settings (identity PCA model)
    const Configuration& ModelRegistrationConfiguration() const { return m_modelFittingConfig; }
    Configuration& ModelRegistrationConfiguration() { return m_modelFittingConfig; }

    //! get/set the fine registration settings (per-vertex deformation)
    const Configuration& FineRegistrationConfiguration() const { return m_fineFittingConfig; }
    Configuration& FineRegistrationConfiguration() { return m_fineFittingConfig; }

    //! Set the source mesh.
    void SetSourceMesh(const Mesh<T>& mesh);

    //! Set the target meshes
    void SetTargetMeshes(const std::vector<std::shared_ptr<const Mesh<T>>>& targetMeshes,
                         const std::vector<Eigen::VectorX<T>>& targetWeights, bool alignedInputs = false);

    //! Set the target depth
    void SetTargetDepths(const std::vector<std::vector<std::shared_ptr<const DepthmapData<T>>>>& targetDepths);

    //! Sets the mesh landmarks that are use for registration
    void SetMeshLandmarks(const MeshLandmarks<T>& meshLandmarks);

    //! Set the target 2D landmarks
    void SetTarget2DLandmarks(const std::vector<std::vector<std::pair<LandmarkInstance<T, 2>, Camera<T>>>>& landmarks);

    //! Set the target 3D landmarks
    void SetTarget3DLandmarks(const std::vector<LandmarkInstance<T, 3>>& landmarks);

    //! Sets the collision constraint masks
    void SetCollisionMasks(const VertexWeights<T>& maskSource, const VertexWeights<T>& maskTarget);

    //! @return the current deformed vertices
    const Eigen::Matrix<T, 3, -1>& CurrentDeformedVertices() const;

    //! Resets the identity model parameters as well as the per vertex offsets
    void ResetNonrigid();

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
                                               const std::vector<VertexWeights<T>>& searchWeights,
                                               int numIterations = 10);

    /**
     * Nonrigid registration using the identity model (discard the per-vertex offsets)
     * @param source2target                    The (current) affine transformation from source to target.
     * @param numIterations                   The number of iterations for registration.
     */
    std::vector<Affine<T, 3, 3>> RegisterNonRigid(const std::vector<Affine<T, 3, 3>>& source2target,
                                                    const std::vector<VertexWeights<T>>& searchWeights,
                                                    int numIterations = 10);

    //! Resets the fine registration (per vertex offsets)
    void ResetFine();

    /**
     * Nonrigid registration with per-vertex displacement
     * @param source2target                   The (current) affine transformation of the source mesh to the target mesh.
     * @param numIterations                   The number of iterations for rigid registration.
     */
    std::vector<Affine<T, 3, 3>> RegisterFine(const std::vector<Affine<T, 3, 3>>& source2target,
                                              const std::vector<VertexWeights<T>>& searchWeights,
                                              int numIterations = 10);

private:
    void InitIcpConstraints(int numOfObservations);
    void Init2DLandmarksConstraints(int numOfObservations);
    void Init3DLandmarksConstraints(int numOfObservations);

    void UpdateIcpConfiguration(const Configuration& targetConfig);
    void Update2DLandmarkConfiguration(const Configuration& targetConfig);
    void Update3DLandmarkConfiguration(const Configuration& targetConfig);
    void UpdateIcpWeights(const std::vector<VertexWeights<T>>& weights);

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

    Configuration m_modelFittingConfig = { std::string("Model Fitting Configuration") , {
        // ! whether to use distance threshold
        {"useDistanceThreshold", ConfigurationParameter(false)},
        // ! whether to optimize the scale of the model
        {"optimizeScale", ConfigurationParameter(false)},
        //!< regularization of model parameters
        { "modelRegularization", ConfigurationParameter(T(100), T(0), T(1000)) },
        //!< how much weight to use on geometry constraint
        { "geometryWeight", ConfigurationParameter(T(1), T(0), T(1)) },
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< how much weight to use on landmark constraints
        { "landmarksWeight", ConfigurationParameter(T(0.001), T(0), T(0.1)) },
        //!< how much weight to use on 3D landmark constraints
        { "3DlandmarksWeight", ConfigurationParameter(T(100.0f), T(50.0f), T(200.0f)) },
        //!< minimum distance threshold value - if used
        { "minimumDistanceThreshold", ConfigurationParameter(T(5), T(0), T(10)) },
        //!< resampling of curves
        { "curveResampling", ConfigurationParameter(1, 1, 5) }
    } };

    Configuration m_fineFittingConfig = { std::string("Fine Fitting Configuration") , {
        // ! whether to use distance threshold
        {"useDistanceThreshold", ConfigurationParameter(true)},
        //!< whether to optimize the pose when doing fine registration
        { "optimizePose", ConfigurationParameter(false) },
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
        //!< resampling of curves
        { "curveResampling", ConfigurationParameter(5, 1, 5) },
        //!< only to use user landmarks while solving
        { "justUserLandmarks", ConfigurationParameter(false) },
        //!< minimum distance threshold value - if used
        { "minimumDistanceThreshold", ConfigurationParameter(T(2), T(0), T(10)) },
        //!< weight for collisions
        { "collisionWeight", ConfigurationParameter(T(0), T(0), T(1)) }
    } };

    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
