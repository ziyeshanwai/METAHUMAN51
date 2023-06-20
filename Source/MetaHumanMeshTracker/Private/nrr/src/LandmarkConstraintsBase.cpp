// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/landmarks/LandmarkConstraintsBase.h>
#include <nrr/MeshContourPoint.h>

#include <nls/Cost.h>
#include <nls/functions/BarycentricCoordinatesFunction.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>
#include <nls/geometry/CatmullRom.h>
#include <nls/geometry/Polyline.h>

namespace epic::nls {

    template <class T>
    const Configuration& LandmarkConstraintsBase<T>::GetConfiguration() const
    {
        return m_config;
    }

    template <class T>
    const std::vector<int> LandmarkConstraintsBase<T>::ExcludeInactiveMeshIndices(std::vector<int> meshIndices) const{

        const MeshLandmarks<T>* EyeLeftLandmarksPtr = MeshLandmarksForType(MeshType::EyeLeft);
        if (!EvaluateMeshActivity(EyeLeftLandmarksPtr)) {
            meshIndices[1] = -1;
        }

        const MeshLandmarks<T>* EyeRightLandmarksPtr = MeshLandmarksForType(MeshType::EyeRight);
        if (!EvaluateMeshActivity(EyeRightLandmarksPtr)) {
            meshIndices[2] = -1;
        }

        const MeshLandmarks<T>* TeethLandmarksPtr = MeshLandmarksForType(MeshType::Teeth);
        if (!EvaluateMeshActivity(TeethLandmarksPtr)) {
            meshIndices[3] = -1;
        }

        return meshIndices;

    }

    template <class T>
    void LandmarkConstraintsBase<T>::SetConfiguration(const Configuration& config)
    {
        m_config.Set(config);
    }

    template <class T>
    const MeshLandmarks<T>* LandmarkConstraintsBase<T>::MeshLandmarksForType(MeshType type) const
    {
        switch (type) {
            default:
            case MeshType::Face: return &m_meshLandmarks;
            case MeshType::EyeLeft: return &m_eyeLeftMeshLandmarks;
            case MeshType::EyeRight: return &m_eyeRightMeshLandmarks;
            case MeshType::Teeth: return &m_teethMeshLandmarks;
        }
    }

    template <class T>
    T LandmarkConstraintsBase<T>::UserDefinedWeight(const std::string& landmarkOrCurveName) const
    {
        auto it = m_userDefinedLandmarkAndCurveWeights.find(landmarkOrCurveName);
        if (it != m_userDefinedLandmarkAndCurveWeights.end()) {
            // return sqrt as it is implicitly squared in the loss function
            return sqrt(it->second);
        } else {
            return T(1);
        }
    }

    template <class T>
    std::string LandmarkConstraintsBase<T>::MeshTypeToName(MeshType meshType)
    {
        switch (meshType) {
            case MeshType::Face: return "face";
            case MeshType::EyeLeft: return "eye_left";
            case MeshType::EyeRight: return "eye_right";
            case MeshType::Teeth: return "teeth";
        }
        return "unknown";
    }

    template class LandmarkConstraintsBase<float>;
    template class LandmarkConstraintsBase<double>;

}  // namespace epic::nls
