// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/Cost.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/BarycentricCoordinates.h>
#include <nls/geometry/Camera.h>
#include <nls/geometry/VertexConstraints.h>
#include <nls/utils/ConfigurationParameter.h>

#include <nrr/MeshLandmarks.h>
#include <nrr/landmarks/LandmarkConfiguration.h>
#include <nrr/landmarks/LandmarkInstance.h>

#include <map>
#include <string>
#include <vector>

namespace epic::nls {

template <class T>
class LandmarkConstraintsData
{
public:
    struct ConstraintData {
        BarycentricCoordinates<T> barycentricCoordinate; //!< The barycentric coordinate for which the constraint is applicable
        Eigen::Vector<T, 2> vertexProjection; //!< The projection of the mesh vertex into the camera.
        Eigen::Vector<T, 2> target; //!< The target position.
        Eigen::Vector<T, 2> targetNormal; //!< The target normal.
        T weight; //!< The weight of the constraint.
    };
    std::map<std::string, std::vector<ConstraintData>> constraintDataPerCamera;
};

template <class T>
class LandmarkConstraintsBase
{
public:
    enum class MeshType {
        Face,
        EyeLeft,
        EyeRight,
        Teeth
    };

    static constexpr const char* ConfigName() { return "Landmark Constraints Configuration"; }

public:

    virtual ~LandmarkConstraintsBase() {}

    /**
     * Evaluates landmarks and curves according to the configuration parameters.
     */
    virtual Cost<T> Evaluate(const DiffDataMatrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals, bool enforceConsistentSparsityPattern = true) const = 0;

    //! evaluates the eye curves according to the configuration parameters
    virtual Cost<T> EvaluateEyeConstraints(const DiffDataMatrix<T, 3, -1>& eyeLeftVertices, const DiffDataMatrix<T, 3, -1>& eyeRightVertices) const = 0;

    //! evaluates the teeth landmarks according to the configuration parameters
    virtual Cost<T> EvaluateTeethConstraints(const DiffDataMatrix<T, 3, -1>& teethVertices) const = 0;

public:
    /**
    * Evaluates the landmark point2point constraint for all current camera/landmark pairs (does *not* scale it by the configuration weight, use Evaluate() instead)
    */
    virtual Cost<T> EvaluateLandmarks(const DiffDataMatrix<T, 3, -1>& vertices, MeshType meshType = MeshType::Face, LandmarkConstraintsData<T>* debugInfo = nullptr) const = 0;

    /**
    * Evaluates the curve point2surface constraint for all current camera/curve pairs (does *not* scale it by the configuration weight, use Evaluate() instead)
    */
    virtual Cost<T> EvaluateCurves(const DiffDataMatrix<T, 3, -1>& vertices, MeshType meshType = MeshType::Face, LandmarkConstraintsData<T>* debugInfo = nullptr) const = 0;

public:

    //! @returns vector of ints for each submesh {EyeLeft,EyeRight,Teeth}, where the meshIndex is replaced with -1 if not tracked.
    const std::vector<int> ExcludeInactiveMeshIndices(std::vector<int>) const;

    //! @returns mesh name string for given MeshType
    static std::string MeshTypeToName(MeshType meshType);

    //! Set head mesh landmarks to be used in constraint evaluation
    void SetMeshLandmarks(const MeshLandmarks<T>& meshLandmarks) { m_meshLandmarks = meshLandmarks; m_zeroWeightAuxillaryMatrix.reset(); }

    //! Set eye meshes landmarks to be used in constraint evaluation
    void SetEyeMeshLandmarks(const MeshLandmarks<T>& eyeLeftMeshLandmarks, const MeshLandmarks<T>& eyeRightMeshLandmarks)
    {
        m_eyeLeftMeshLandmarks = eyeLeftMeshLandmarks;
        m_eyeRightMeshLandmarks = eyeRightMeshLandmarks;
    }

    //! Set teeth mesh landmarks to be used in constraint evaluation
    void SetTeethMeshLandmarks(const MeshLandmarks<T>& teethMeshLandmarks) { m_teethMeshLandmarks = teethMeshLandmarks; }

    //! Set individual weights for landmarks and/or curves
    void SetUserDefinedLandmarkAndCurveWeights(const std::map<std::string, T>& userDefinedLandmarkAndCurveWeights) { m_userDefinedLandmarkAndCurveWeights = userDefinedLandmarkAndCurveWeights; }

    //! Get parameters configuration
    const Configuration& GetConfiguration() const;

    //! Set parameters configuration
    void SetConfiguration(const Configuration& config);

protected:
    //! @returns the MeshLandmarks structure that belongs to the mesh type
    const MeshLandmarks<T>* MeshLandmarksForType(MeshType type) const;

    //! @returns the user defined landmark or curve weight or 1.0 if not defined
    T UserDefinedWeight(const std::string& landmarkOrCurveName) const;

    //! @returns true if submesh {EyeLeft,EyeRight,Teeth} is provided with tracked landmarks or curves.
    virtual bool EvaluateMeshActivity(const MeshLandmarks<T>* meshLandmarksPtr) const = 0;

protected:
    //! Landmarks and curves on the face mesh corresponding to landmarks and curves as annotated in 2D images
    MeshLandmarks<T> m_meshLandmarks;

    //! Curves on the left eye mesh corresponding to landmarks and curves as annotated in 2D images
    MeshLandmarks<T> m_eyeLeftMeshLandmarks;

    //! Curves on the right eye mesh corresponding to landmarks and curves as annotated in 2D images
    MeshLandmarks<T> m_eyeRightMeshLandmarks;

    //! Landarmks on the teeth mesh corresponding to landmarks and curves as annotated in 2D images
    MeshLandmarks<T> m_teethMeshLandmarks;

    //! optional weighting of landmarks and curves
    std::map<std::string, T> m_userDefinedLandmarkAndCurveWeights;

    Configuration m_config = { ConfigName(), {
                             //!< how much weight to use on landmark constraints
                             { "landmarksWeight", ConfigurationParameter(T(0.005), T(0), T(0.5)) },
                             //!< flag whether to use the contour constraint on the latest contour line
                             { "constrainContourBorder", ConfigurationParameter(true) },
                             //!< how much weight to use on inner lip constraints
                             { "innerLipWeight", ConfigurationParameter(T(0.005), T(0), T(0.5)) },
                             //!< flag whether to use the inner lip constraint on the latest contour line
                             { "constraintInnerLipBorder", ConfigurationParameter(true) },
                             //!< how much weight to use on eye constraints
                             { "eyesWeight", ConfigurationParameter(T(0.005), T(0), T(1.0)) },
                             //!< how much weight to use on teeth constraints
                             { "teethWeight", ConfigurationParameter(T(0.1), T(0), T(1.0)) },
                             //!< resampling of curves
                             { "curveResampling", ConfigurationParameter(1, 1, 5) }
    } };

    /**
    * This is a zero weight pattern matrix that makes sure that when calculating JtJ on the Jacobian of the output diffdata then the sparsity pattern of JtJ stays the same.
    * The problem is that for inner lip constraints the constraint is set on an edge between two vertices, therefore the output Jacobian matrix will change based on what edge
    * is selected. By adding all(!) possible inner lip constraints as zero terms the sparsity structure will stay consistent. However, it adds cost as there are more terms and the
    * Jacobian density increases.
    */
    mutable SparseMatrixConstPtr<T> m_zeroWeightAuxillaryMatrix;
};

} // namespace epic::nls
