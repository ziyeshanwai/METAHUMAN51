// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nrr/landmarks/LandmarkConstraintsBase.h>

namespace epic::nls {

template <class T>
class LandmarkConstraints2D : public LandmarkConstraintsBase<T>
{
public:
    using typename LandmarkConstraintsBase<T>::MeshType;

    virtual ~LandmarkConstraints2D() {}

    //! Clear target landmarks
    void Clear() { m_targetLandmarks.clear(); }

    //! Set target landmarks - in case of 2D landmarks, input is a vector of 2D LandmarkInstance paired with a corresponding camera
    void SetTargetLandmarks(const std::vector<std::pair<LandmarkInstance<T, 2>, Camera<T>>>& landmarks) { m_targetLandmarks = landmarks; }

    /**
    * Evaluates landmarks, curves, and inner lips according to the configuration parameters.
    */
    virtual Cost<T> Evaluate(const DiffDataMatrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals, bool enforceConsistentSparsityPattern = true) const override;

    //! evaluates the eye curves according to the configuration parameters
    virtual Cost<T> EvaluateEyeConstraints(const DiffDataMatrix<T, 3, -1>& eyeLeftVertices, const DiffDataMatrix<T, 3, -1>& eyeRightVertices) const override;

    //! evaluates the teeth landmarks according to the configuration parameters
    virtual Cost<T> EvaluateTeethConstraints(const DiffDataMatrix<T, 3, -1>& teethVertices) const override;

    //! Creates the landmark constraints as vertex constraints into @p landmarkVertexConstraints
    void SetupLandmarkConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                  const Eigen::Matrix<T, 3, -1>& vertices,
                                  const MeshLandmarks<T>* meshLandmarks,
                                  const MeshType meshType,
                                  VertexConstraints<T, 2, 3>& landmarkVertexConstraints) const;

    //! Creates the curves constraints as vertex constraints into @p curveVertexConstraints
    void SetupCurveConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                               const Eigen::Matrix<T, 3, -1>& vertices,
                               const MeshLandmarks<T>* meshLandmarks,
                               const MeshType meshType,
                               VertexConstraints<T, 1, 3>& curveVertexConstraints) const;

    //! Creates the contour constraints as vertex constraints into @p contourVertexConstraints
    void SetupContourConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                 const Eigen::Matrix<T, 3, -1>& vertices,
                                 const Eigen::Matrix<T, 3, -1>& normals,
                                 const MeshLandmarks<T>* meshLandmarks,
                                 const MeshType meshType,
                                 VertexConstraints<T, 1, 2>& contourVertexConstraints) const;

    //! Creates the inner lips constraints as vertex constraints into @p innerLipVertexConstraints
    void SetupInnerLipConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                  const Eigen::Matrix<T, 3, -1>& vertices,
                                  const Eigen::Matrix<T, 3, -1>& normals,
                                  const MeshLandmarks<T>* meshLandmarks,
                                  VertexConstraints<T, 1, 2>& innerLipVertexConstraints) const;

public:
    /**
    * Evaluates the landmark point2point constraint for all current camera/landmark pairs (does *not* scale it by the configuration weight, use Evaluate() instead)
    */
    virtual Cost<T> EvaluateLandmarks(const DiffDataMatrix<T, 3, -1>& vertices, MeshType meshType = MeshType::Face, LandmarkConstraintsData<T>* debugInfo = nullptr) const override;

    /**
    * Evaluates the curve point2surface constraint for all current camera/curve pairs (does *not* scale it by the configuration weight, use Evaluate() instead)
    */
    virtual Cost<T> EvaluateCurves(const DiffDataMatrix<T, 3, -1>& vertices, MeshType meshType = MeshType::Face, LandmarkConstraintsData<T>* debugInfo = nullptr) const override;

    /**
    * Evaluates the contour point2surface constraint for all current camera/curve pairs (does *not* scale it by the configuration weight, use Evaluate() instead)
    * For this, for each contour line it finds first the position on the contour line where the normal switches from front facing to back facing
    * in the camera, and then evaluates a point2surface constraint to the 2D landmark contour.
    */
    Cost<T> EvaluateContours(const DiffDataMatrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals, MeshType meshType = MeshType::Face, LandmarkConstraintsData<T>* debugInfo = nullptr) const;

    /**
    * Evaluates the inner lip point2surface constraint for all current camera/inner lip contour curve pairs (does *not* scale it by the configuration weight, use Evaluate() instead)
    * For this, for each contour line it finds first the position on the contour line where the normal switches from front facing to back facing
    * in the camera, and then evaluates a point2surface constraint to the 2D landmark contour.
    */
    Cost<T> EvaluateInnerLips(const DiffDataMatrix<T, 3, -1>& vertices,
                              const Eigen::Matrix<T, 3, -1>& normals,
                              LandmarkConstraintsData<T>* debugInfoUpper = nullptr,
                              LandmarkConstraintsData<T>* debugInfoLower = nullptr) const;

private:

    virtual bool EvaluateMeshActivity(const MeshLandmarks<T>* meshLandmarksPtr) const override;

private:

    /**
    * A set of landmark instances adhering to the landmark configuration and cameras.
    * The cameras describe the projection of a 3D point into a 2D image, and the landmarks are assumed to be in that camera space.
    */
    std::vector<std::pair<LandmarkInstance<T, 2>, Camera<T>>> m_targetLandmarks;

};

} // namespace epic::nls
