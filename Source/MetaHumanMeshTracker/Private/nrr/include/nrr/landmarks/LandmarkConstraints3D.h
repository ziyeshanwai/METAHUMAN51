// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nrr/landmarks/LandmarkConstraintsBase.h>

namespace epic::nls {

template <class T>
class LandmarkConstraints3D : public LandmarkConstraintsBase<T>
{
public:
    using typename LandmarkConstraintsBase<T>::MeshType;

    virtual ~LandmarkConstraints3D() {}

    //! Set target landmarks - in case of 3D landmarks input is LandmarkInstance3D
    void SetTargetLandmarks(const LandmarkInstance<T, 3>& landmarks) { m_targetLandmarks = landmarks; }

    /**
    * Evaluates landmarks and curves according to the configuration parameters.
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
                                  VertexConstraints<T, 3, 3>& landmarkVertexConstraints) const;

    //! Creates the curves constraints as vertex constraints into @p curveVertexConstraints
    void SetupCurveConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                               const Eigen::Matrix<T, 3, -1>& vertices,
                               const MeshLandmarks<T>* meshLandmarks,
                               const MeshType meshType,
                               VertexConstraints<T, 2, 3>& curveVertexConstraints) const;

public:

    /**
    * Evaluates the landmark point2point constraint for all current camera/landmark pairs (does *not* scale it by the configuration weight, use Evaluate() instead)
    */
    virtual Cost<T> EvaluateLandmarks(const DiffDataMatrix<T, 3, -1>& vertices, MeshType meshType = MeshType::Face, LandmarkConstraintsData<T>* debugInfo = nullptr) const override;

    /**
    * Evaluates the curve point2surface constraint for all current camera/curve pairs (does *not* scale it by the configuration weight, use Evaluate() instead)
    */
    virtual Cost<T> EvaluateCurves(const DiffDataMatrix<T, 3, -1>& vertices, MeshType meshType = MeshType::Face, LandmarkConstraintsData<T>* debugInfo = nullptr) const override;

private:
    //! @returns true if submesh {EyeLeft,EyeRight,Teeth} is provided with tracked landmarks or curves.
    virtual bool EvaluateMeshActivity(const MeshLandmarks<T>* meshLandmarksPtr) const override;

private:

    LandmarkInstance<T, 3> m_targetLandmarks;

};

} // namespace epic::nls
