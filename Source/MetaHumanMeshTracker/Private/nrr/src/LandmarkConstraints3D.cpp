// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/landmarks/LandmarkConstraints3D.h>
#include <nrr/MeshContourPoint.h>

#include <nls/Cost.h>
#include <nls/functions/BarycentricCoordinatesFunction.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>
#include <nls/geometry/CatmullRom.h>
#include <nls/geometry/Polyline.h>

namespace epic::nls {

    template <class T>
    bool LandmarkConstraints3D<T>::EvaluateMeshActivity(const MeshLandmarks<T>* meshLandmarksPtr) const {

        bool activeFlag = false;
        const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = m_targetLandmarks.GetLandmarkConfiguration();
        for (auto&& [landmarkName, _] : meshLandmarksPtr->LandmarksBarycentricCoordinates()) {
            if (landmarkConfiguration->HasLandmark(landmarkName)) {
                activeFlag = true;
            }
        }
        for (auto&& [curveName, _] : meshLandmarksPtr->MeshCurvesBarycentricCoordinates()) {
            if (landmarkConfiguration->HasCurve(curveName)) {
                activeFlag = true;
            }
        }

        return activeFlag;
    }

    template<class T>
    Cost<T> LandmarkConstraints3D<T>::EvaluateLandmarks(const DiffDataMatrix<T, 3, -1>& vertices, MeshType meshType, LandmarkConstraintsData<T>* debugInfo) const {

        if (debugInfo) {
            LOG_INFO("Debug info for 3D landmarks still not implemented.");
        }

        const MeshLandmarks<T>* meshLandmarksPtr = this->MeshLandmarksForType(meshType);

        Cost<T> cost;

        const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = m_targetLandmarks.GetLandmarkConfiguration();

        std::vector<int> landmarkIndices;
        std::vector<BarycentricCoordinates<T>> barycentricCoordinates;
        std::vector<T> userDefinedWeights;
        for (auto&& [landmarkName, bc] : meshLandmarksPtr->LandmarksBarycentricCoordinates()) {
            if (landmarkConfiguration->HasLandmark(landmarkName)) {
                const T userDefinedWeight = this->UserDefinedWeight(landmarkName);
                if (userDefinedWeight > 0) {
                    landmarkIndices.push_back(landmarkConfiguration->IndexForLandmark(landmarkName));
                    barycentricCoordinates.push_back(bc);
                    userDefinedWeights.push_back(sqrt(userDefinedWeight));
                }
            }
        }

        const DiffDataMatrix<T, 3, -1> meshLandmarkPositions = BarycentricCoordinatesFunction<T, 3>::Evaluate(vertices, barycentricCoordinates);

        Eigen::Matrix<T, 3, -1> targetLandmarkPositions = m_targetLandmarks.Points(landmarkIndices);

        Eigen::Vector<T, -1> landmarkWeights(int(landmarkIndices.size()));
        for (int k = 0; k < int(targetLandmarkPositions.cols()); k++) {
            landmarkWeights[k] = userDefinedWeights[k] * m_targetLandmarks.Confidence()[landmarkIndices[k]];
        }

        cost.Add(PointPointConstraintFunction<T, 3>::Evaluate(meshLandmarkPositions, targetLandmarkPositions,
            landmarkWeights, T(1)), T(1), this->MeshTypeToName(meshType) + "_landmarks");

        return cost;
    }

    template<class T>
    Cost<T> LandmarkConstraints3D<T>::Evaluate(const DiffDataMatrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals, bool enforceConsistentSparsityPattern) const
    {
        (void)normals;
        (void)enforceConsistentSparsityPattern;
        PROFILING_FUNCTION(PROFILING_COLOR_RED);

        Cost<T> cost;

        const T landmarksWeight = this->m_config["landmarksWeight"];

        if (landmarksWeight > T(0))
        {
            PROFILING_BLOCK("landmarks");
            cost.Add(EvaluateLandmarks(vertices), landmarksWeight);
            PROFILING_END_BLOCK;

            PROFILING_BLOCK("curves");
            cost.Add(EvaluateCurves(vertices), landmarksWeight);
            PROFILING_END_BLOCK;
        }

        return cost;
    }

    template<class T>
    Cost<T> LandmarkConstraints3D<T>::EvaluateEyeConstraints(const DiffDataMatrix<T, 3, -1>& , const DiffDataMatrix<T, 3, -1>& ) const {
        // not implemented yet
        return Cost<T>();
    }

    template<class T>
    Cost<T> LandmarkConstraints3D<T>::EvaluateTeethConstraints(const DiffDataMatrix<T, 3, -1>& ) const {
        // not implemented yet
        return Cost<T>();
    }

    template<class T>
    Cost<T> LandmarkConstraints3D<T>::EvaluateCurves(const DiffDataMatrix<T, 3, -1>& vertices, MeshType meshType, LandmarkConstraintsData<T>* debugInfo) const
    {

        if (debugInfo)
        {
            debugInfo->constraintDataPerCamera.clear();
        }

        const int curveResampling = this->m_config["curveResampling"];

        const MeshLandmarks<T>* meshLandmarksPtr = this->MeshLandmarksForType(meshType);

        Cost<T> cost;


        const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = m_targetLandmarks.GetLandmarkConfiguration();

        for (auto&& [curveName, bcs] : meshLandmarksPtr->MeshCurvesBarycentricCoordinates())
        {
            if (landmarkConfiguration->HasCurve(curveName))
            {
                const T userDefinedWeight = this->UserDefinedWeight(curveName);
                if (userDefinedWeight > 0)
                {
                    // get the 3d positions of the curve points on the mesh
                    const DiffDataMatrix<T, 3, -1> meshCurvePositions = BarycentricCoordinatesFunction<T, 3>::Evaluate(vertices, bcs);

                    const int numConstraintsForCurve = meshCurvePositions.Cols();

                    // find the correspondences
                    Eigen::Matrix<T, 3, -1> targetPixelPositions(3, numConstraintsForCurve);
                    Eigen::Matrix<T, 3, -1> targetPixelNormals(3, numConstraintsForCurve);
                    Eigen::Matrix<T, 3, -1> targetPixelNormals2(3, numConstraintsForCurve);
                    Eigen::VectorX<T> correspondenceWeights(numConstraintsForCurve);

                    Polyline<T, 3> polyline(m_targetLandmarks.Points(landmarkConfiguration->IndicesForCurve(curveName)));

                    // Special handling of loop curves: we discard curve correspondences where the correspondence is looking at the "wrong" side of the loop.
                    // In order to not rely on the specific ordering of the landmarks or the mesh landmarks we can use the center of gravity of both
                    // the model and landmark points and compare direction this way.
                    const bool isLoop = meshLandmarksPtr->IsLoop(curveName);
                    Eigen::Vector3<T> gravityOfCurve(0, 0, 0);
                    Eigen::Vector3<T> gravityOfModel(0, 0, 0);
                    if (isLoop && numConstraintsForCurve > 0)
                    {
                        gravityOfCurve = polyline.ControlPoints().rowwise().mean();
                        gravityOfModel = meshCurvePositions.Matrix().rowwise().mean();
                    }

                    if (curveResampling > 1)
                    {
                        polyline = CatmullRom<T, 3>(polyline.ControlPoints(), curveResampling, /*closed=*/false).SampledPoints();
                    }
                    for (int k = 0; k < numConstraintsForCurve; k++)
                    {
                        Eigen::Vector3<T> target;
                        Eigen::Vector3<T> direction;
                        Eigen::Vector3<T> normal1;

                        T confidence;
                        polyline.ClosestPointAndNormal(meshCurvePositions.Matrix().col(k), target, normal1, direction, confidence);
                        targetPixelPositions.col(k) = target;
                        targetPixelNormals.col(k) = normal1;
                        correspondenceWeights[k] = userDefinedWeight * confidence;
                        if (isLoop && (meshCurvePositions.Matrix().col(k) - gravityOfModel).dot(target - gravityOfCurve) < 0)
                            correspondenceWeights[k] = 0;

                        Eigen::Vector3<T> normal2 = normal1.cross(direction).normalized();
                        targetPixelNormals2.col(k) = normal2;
                        if (debugInfo)
                        {

                        }
                    }

                    cost.Add(PointSurfaceConstraintFunction<T, 3>::Evaluate(meshCurvePositions, targetPixelPositions,
                                 targetPixelNormals, correspondenceWeights, T(1)),
                        T(1), this->MeshTypeToName(meshType) + "_N1_"+ curveName);

                    cost.Add(PointSurfaceConstraintFunction<T, 3>::Evaluate(meshCurvePositions, targetPixelPositions,
                                 targetPixelNormals2, correspondenceWeights, T(1)),
                        T(1), this->MeshTypeToName(meshType) + "_N2_" + curveName);
                }
            }
        }

        return cost;
    }

    template<class T>
    void LandmarkConstraints3D<T>::SetupLandmarkConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                                            const Eigen::Matrix<T, 3, -1>& vertices,
                                                            const MeshLandmarks<T>* meshLandmarksPtr,
                                                            const MeshType meshType,
                                                            VertexConstraints<T, 3, 3>& landmarkVertexConstraints) const {
        if (!meshLandmarksPtr) return;

        T landmarksWeight = this->m_config["landmarksWeight"].template Value<T>();

        switch (meshType) {
            default:
            case MeshType::Face: landmarksWeight = this->m_config["landmarksWeight"]; break;
            case MeshType::Teeth: landmarksWeight = this->m_config["teethWeight"]; break;
            case MeshType::EyeLeft: landmarksWeight = this->m_config["eyesWeight"]; break;
            case MeshType::EyeRight: landmarksWeight = this->m_config["eyesWeight"]; break;
        }

        if (landmarksWeight <= 0) return;

        const int maxNumLandmarks = int(meshLandmarksPtr->LandmarksBarycentricCoordinates().size());
        landmarkVertexConstraints.ResizeToFitAdditionalConstraints(maxNumLandmarks);

        Eigen::Matrix<T, 3, 9> drdV = Eigen::Matrix<T, 3, 9>::Zero();

        const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = m_targetLandmarks.GetLandmarkConfiguration();
        const Eigen::Transform<T, 3, Eigen::Affine> invTransform = rigidTransform.inverse();

        for (const auto& [landmarkName, bc] : meshLandmarksPtr->LandmarksBarycentricCoordinates()) {
            if (landmarkConfiguration->HasLandmark(landmarkName)) {
                const T userDefinedWeight = this->UserDefinedWeight(landmarkName);
                if (userDefinedWeight > 0) {
                    const int landmarkIndex = landmarkConfiguration->IndexForLandmark(landmarkName);
                    const T weight = sqrt(userDefinedWeight) * sqrt(landmarksWeight) * m_targetLandmarks.Confidence()[landmarkIndex];
                    if (weight <= 0) continue;
                    const Eigen::Vector3<T> targetLandmark = invTransform * m_targetLandmarks.Points().col(landmarkIndex);
                    const Eigen::Vector3<T> residual = weight * (bc.template Evaluate<3>(vertices) - targetLandmark);

                    for (int k = 0; k < 3; ++k) {
                        drdV(0, 3 * k + 0) = weight * bc.Weight(k);
                        drdV(1, 3 * k + 1) = weight * bc.Weight(k);
                        drdV(2, 3 * k + 2) = weight * bc.Weight(k);
                    }
                    landmarkVertexConstraints.AddConstraint(bc.Indices(), residual, drdV);
                }
            }
        }
    }

    template<class T>
    void LandmarkConstraints3D<T>::SetupCurveConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                                        const Eigen::Matrix<T, 3, -1>& vertices,
                                                        const MeshLandmarks<T>* meshLandmarksPtr,
                                                        const MeshType meshType,
                                                        VertexConstraints<T, 2, 3>& curveVertexConstraints) const {
        if (!meshLandmarksPtr) return;

        T landmarksWeight = this->m_config["landmarksWeight"];

        switch (meshType) {
            default:
            case MeshType::Face: landmarksWeight = this->m_config["landmarksWeight"]; break;
            case MeshType::Teeth: landmarksWeight = this->m_config["teethWeight"]; break;
            case MeshType::EyeLeft: landmarksWeight = this->m_config["eyesWeight"]; break;
            case MeshType::EyeRight: landmarksWeight = this->m_config["eyesWeight"]; break;
        }
        const int curveResampling = this->m_config["curveResampling"];

        if (landmarksWeight <= 0) return;

        int maxNumCurveConstraintsPerCamera = 0;
        for (auto&& [_, bcs] : meshLandmarksPtr->MeshCurvesBarycentricCoordinates()) {
            maxNumCurveConstraintsPerCamera += int(bcs.size());
        }
        curveVertexConstraints.ResizeToFitAdditionalConstraints(maxNumCurveConstraintsPerCamera);

        const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = m_targetLandmarks.GetLandmarkConfiguration();
        const Eigen::Transform<T, 3, Eigen::Affine> invTransform = rigidTransform.inverse();

        for (auto&& [curveName, bcs] : meshLandmarksPtr->MeshCurvesBarycentricCoordinates()) {
            if (landmarkConfiguration->HasCurve(curveName)) {
                const T userDefinedWeight = this->UserDefinedWeight(curveName);
                if (userDefinedWeight > 0) {
                    const int numConstraintsForCurve = int(bcs.size());

                    // get the 3d positions of the curve points on the mesh
                    Eigen::Matrix<T, 3, -1> meshCurvePositions(3, numConstraintsForCurve);
                    for (int i = 0; i < numConstraintsForCurve; i++) {
                        meshCurvePositions.col(i) = bcs[i].template Evaluate<3>(vertices);
                    }

                    Polyline<T, 3> polyline(m_targetLandmarks.Points(landmarkConfiguration->IndicesForCurve(curveName)));

                    // Special handling of loop curves: we discard curve correspondences where the correspondence is looking at the "wrong" side of the loop.
                    // In order to not rely on the specific ordering of the landmarks or the mesh landmarks we can use the center of gravity of both
                    // the model and landmark points and compare direction this way.
                    const bool isLoop = meshLandmarksPtr->IsLoop(curveName);
                    Eigen::Vector3<T> gravityOfCurve(0, 0, 0);
                    Eigen::Vector3<T> gravityOfModel(0 ,0, 0);
                    if (isLoop && numConstraintsForCurve > 0) {
                        gravityOfCurve = polyline.ControlPoints().rowwise().mean();
                        gravityOfModel = meshCurvePositions.rowwise().mean();
                    }

                    if (curveResampling > 1) {
                        polyline = CatmullRom<T, 3>(polyline.ControlPoints(), curveResampling, /*closed=*/false).SampledPoints();
                    }
                    for (int i = 0; i < numConstraintsForCurve; i++) {
                        Eigen::Vector3<T> target;
                        Eigen::Vector3<T> direction;
                        Eigen::Vector3<T> normal1;

                        const Eigen::Vector3<T> meshCurvePosition = meshCurvePositions.col(i);
                        const Eigen::Vector3<T> transformedMeshCurvePosition = rigidTransform * meshCurvePosition;

                        T confidence;
                        polyline.ClosestPointAndNormal(transformedMeshCurvePosition, target, normal1, direction, confidence);
                        T weight = userDefinedWeight * confidence;
                        if (isLoop && (transformedMeshCurvePosition - gravityOfModel).dot(target - gravityOfCurve) < 0) {
                            weight = 0;
                        }

                        Eigen::Vector3<T> normal2 = normal1.cross(direction).normalized();

                        // put into coordinate system of input vertices
                        target = invTransform * target;
                        normal1 = invTransform.linear() * normal1;
                        normal2 = invTransform.linear() * normal2;

                        const T residual1 = weight * normal1.dot(meshCurvePosition - target);
                        const T residual2 = weight * normal2.dot(meshCurvePosition - target);

                        Eigen::Matrix<T, 2, 9> drdV;
                        for (int k = 0; k < 3; ++k) {
                            drdV(0, 3 * k + 0) = weight * bcs[i].Weight(k) * normal1[0];
                            drdV(0, 3 * k + 1) = weight * bcs[i].Weight(k) * normal1[1];
                            drdV(0, 3 * k + 2) = weight * bcs[i].Weight(k) * normal1[2];
                            drdV(1, 3 * k + 0) = weight * bcs[i].Weight(k) * normal2[0];
                            drdV(1, 3 * k + 1) = weight * bcs[i].Weight(k) * normal2[1];
                            drdV(1, 3 * k + 2) = weight * bcs[i].Weight(k) * normal2[2];
                        }

                        curveVertexConstraints.AddConstraint(bcs[i].Indices(), Eigen::Vector<T, 2>(residual1, residual2), drdV);
                    }
                }
            }
        }
    }

    template class LandmarkConstraints3D<float>;
    template class LandmarkConstraints3D<double>;
}  // namespace epic::nls
