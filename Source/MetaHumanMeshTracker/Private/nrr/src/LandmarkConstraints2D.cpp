// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/landmarks/LandmarkConstraints2D.h>
#include <nrr/MeshContourPoint.h>

#include <nls/Cost.h>
#include <nls/functions/BarycentricCoordinatesFunction.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>
#include <nls/geometry/CatmullRom.h>
#include <nls/geometry/Polyline.h>

namespace epic::nls {

    template <class T>
    bool LandmarkConstraints2D<T>::EvaluateMeshActivity(const MeshLandmarks<T>* meshLandmarksPtr) const {

        bool activeFlag = false;
        for (int i = 0; i < (int)m_targetLandmarks.size(); ++i) {
            const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = m_targetLandmarks[i].first.GetLandmarkConfiguration();
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
        }

        return activeFlag;
    }

    template<class T>
    Cost<T> LandmarkConstraints2D<T>::Evaluate(const DiffDataMatrix<T, 3, -1>& vertices,
                                               const Eigen::Matrix<T, 3, -1>& normals,
                                               bool enforceConsistentSparsityPattern) const {
        PROFILING_FUNCTION(PROFILING_COLOR_RED);

        Cost<T> cost;

        const T landmarksWeight = this->m_config["landmarksWeight"].template Value<T>();
        const T innerLipWeight = this->m_config["innerLipWeight"].template Value<T>();

        if (landmarksWeight > T(0)) {
            PROFILING_BLOCK("landmarks");
            cost.Add(EvaluateLandmarks(vertices), landmarksWeight);
            PROFILING_END_BLOCK;

            PROFILING_BLOCK("curves");
            cost.Add(EvaluateCurves(vertices), landmarksWeight);
            PROFILING_END_BLOCK;
        }

        if (innerLipWeight > T(0) && (this->m_meshLandmarks.InnerLowerLipContourLines().size() > 0 || this->m_meshLandmarks.InnerUpperLipContourLines().size() > 0)) {
            PROFILING_BLOCK("inner lips");
            cost.Add(EvaluateInnerLips(vertices, normals), innerLipWeight);
            PROFILING_END_BLOCK;

            if (vertices.HasJacobian() && enforceConsistentSparsityPattern) {
                PROFILING_BLOCK("auxillary");
                if (!this->m_zeroWeightAuxillaryMatrix) {
                    std::vector<Eigen::Triplet<T>> triplets;
                    int count = 0;
                    for (const std::vector<int>& vIDs : this->m_meshLandmarks.InnerLowerLipContourLines()) {
                        for (size_t i = 0; i < vIDs.size() - 1; ++i) {
                            int vID1 = vIDs[i];
                            int vID2 = vIDs[i + 1];
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID1 + 0, T(0)));
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID1 + 1, T(0)));
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID1 + 2, T(0)));
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID2 + 0, T(0)));
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID2 + 1, T(0)));
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID2 + 2, T(0)));
                            count++;
                        }
                    }
                    for (const std::vector<int>& vIDs : this->m_meshLandmarks.InnerUpperLipContourLines()) {
                        for (size_t i = 0; i < vIDs.size() - 1; ++i) {
                            int vID1 = vIDs[i];
                            int vID2 = vIDs[i + 1];
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID1 + 0, T(0)));
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID1 + 1, T(0)));
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID1 + 2, T(0)));
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID2 + 0, T(0)));
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID2 + 1, T(0)));
                            triplets.push_back(Eigen::Triplet<T>(count, 3 * vID2 + 2, T(0)));
                            count++;
                        }
                    }
                    for (const auto& [_, contour] : this->m_meshLandmarks.Contours()) {
                        for (const std::vector<int>& vIDs : contour) {
                            for (size_t i = 0; i < vIDs.size() - 1; ++i) {
                                int vID1 = vIDs[i];
                                int vID2 = vIDs[i + 1];
                                triplets.push_back(Eigen::Triplet<T>(count, 3 * vID1 + 0, T(0)));
                                triplets.push_back(Eigen::Triplet<T>(count, 3 * vID1 + 1, T(0)));
                                triplets.push_back(Eigen::Triplet<T>(count, 3 * vID1 + 2, T(0)));
                                triplets.push_back(Eigen::Triplet<T>(count, 3 * vID2 + 0, T(0)));
                                triplets.push_back(Eigen::Triplet<T>(count, 3 * vID2 + 1, T(0)));
                                triplets.push_back(Eigen::Triplet<T>(count, 3 * vID2 + 2, T(0)));
                                count++;
                            }
                        }
                    }
                    SparseMatrixPtr<T> smat = std::make_shared<SparseMatrix<T>>(count, vertices.Size());
                    smat->setFromTriplets(triplets.begin(), triplets.end());
                    this->m_zeroWeightAuxillaryMatrix = smat;
                }
                Eigen::VectorX<T> zeroVector = Eigen::VectorX<T>::Zero(this->m_zeroWeightAuxillaryMatrix->rows());
                cost.Add(DiffData<T>(std::make_shared<Vector<T>>(zeroVector), vertices.Jacobian().Premultiply(*this->m_zeroWeightAuxillaryMatrix)), T(1));
                PROFILING_END_BLOCK;
            }
        }

        return cost;
    }

    template <class T>
    Cost<T> LandmarkConstraints2D<T>::EvaluateEyeConstraints(const DiffDataMatrix<T, 3, -1>& eyeLeftVertices, const DiffDataMatrix<T, 3, -1>& eyeRightVertices) const
    {
        PROFILING_FUNCTION(PROFILING_COLOR_RED);

        Cost<T> cost;

        const T weight = this->m_config["eyesWeight"].template Value<T>();
        if (weight > T(0)) {
            cost.Add(EvaluateCurves(eyeLeftVertices, MeshType::EyeLeft), weight);
            cost.Add(EvaluateCurves(eyeRightVertices, MeshType::EyeRight), weight);
        }

        return cost;
    }

    template <class T>
    Cost<T> LandmarkConstraints2D<T>::EvaluateTeethConstraints(const DiffDataMatrix<T, 3, -1>& teethVertices) const
    {
        PROFILING_FUNCTION(PROFILING_COLOR_RED);

        Cost<T> cost;

        const T weight = this->m_config["teethWeight"].template Value<T>();
        if (weight > T(0)) {
            cost.Add(EvaluateLandmarks(teethVertices, MeshType::Teeth), weight);
        }

        return cost;
    }

    template<class T>
    Cost<T> LandmarkConstraints2D<T>::EvaluateLandmarks(const DiffDataMatrix<T, 3, -1>& vertices,
                                                        MeshType meshType,
                                                        LandmarkConstraintsData<T>* debugInfo) const {
        if (debugInfo) {
            debugInfo->constraintDataPerCamera.clear();
        }

        const MeshLandmarks<T>* meshLandmarksPtr = this->MeshLandmarksForType(meshType);

        Cost<T> cost;

        for (const auto& [landmarkInstance, camera] : m_targetLandmarks) {
            const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = landmarkInstance.GetLandmarkConfiguration();

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

            // project the mesh landmarks into the image
            DiffDataMatrix<T, 2, -1> projectedPixelPosition = camera.Project(meshLandmarkPositions,  /*withExtrinsics=*/ true);
            Eigen::Matrix<T, 2, -1> targetPixelPositions(2, int(landmarkIndices.size()));
            Eigen::Vector<T, -1> landmarkWeights(int(landmarkIndices.size()));
            for (int k = 0; k < int(targetPixelPositions.cols()); k++) {
                targetPixelPositions.col(k) = landmarkInstance.Points().col(landmarkIndices[k]);
                landmarkWeights[k] = userDefinedWeights[k] * landmarkInstance.Confidence()[landmarkIndices[k]];

                if (debugInfo) {
                    typename LandmarkConstraintsData<T>::ConstraintData constraintData{barycentricCoordinates[k],
                                                                                       projectedPixelPosition.Matrix().col(k),
                                                                                       targetPixelPositions.col(k),
                                                                                       Eigen::Vector2<T>(0,0),
                                                                                       landmarkWeights[k]};
                    debugInfo->constraintDataPerCamera[camera.Label()].push_back(constraintData);
                }
            }
            cost.Add(PointPointConstraintFunction<T, 2>::Evaluate(projectedPixelPosition, targetPixelPositions,
                                                                        landmarkWeights, T(1)), T(1), camera.Label() + "_" + this->MeshTypeToName(meshType) + "_landmarks");
        }

        return cost;
    }

    template<class T>
    Cost<T> LandmarkConstraints2D<T>::EvaluateCurves(const DiffDataMatrix<T, 3, -1>& vertices,
                                                     MeshType meshType,
                                                     LandmarkConstraintsData<T>* debugInfo) const {
        if (debugInfo) {
            debugInfo->constraintDataPerCamera.clear();
        }

        const int curveResampling = this->m_config["curveResampling"].template Value<int>();

        const MeshLandmarks<T>* meshLandmarksPtr = this->MeshLandmarksForType(meshType);

        Cost<T> cost;

        for (const auto& [landmarkInstance, camera] : m_targetLandmarks) {
            const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = landmarkInstance.GetLandmarkConfiguration();

            for (auto&& [curveName, bcs] : meshLandmarksPtr->MeshCurvesBarycentricCoordinates()) {
                if (landmarkConfiguration->HasCurve(curveName)) {
                    const T userDefinedWeight = this->UserDefinedWeight(curveName);
                    if (userDefinedWeight > 0) {
                        // get the 3d positions of the curve points on the mesh
                        const DiffDataMatrix<T, 3, -1> meshCurvePositions = BarycentricCoordinatesFunction<T, 3>::Evaluate(vertices, bcs);

                        const int numConstraintsForCurve = meshCurvePositions.Cols();

                        // project the mesh points into the image
                        DiffDataMatrix<T, 2, -1> projectedPixelPosition = camera.Project(meshCurvePositions,  /*withExtrinsics=*/ true);

                        // find the correspondences in the 2D curve
                        Eigen::Matrix<T, 2, -1> targetPixelPositions(2, numConstraintsForCurve);
                        Eigen::Matrix<T, 2, -1> targetPixelNormals(2, numConstraintsForCurve);
                        Eigen::VectorX<T> correspondenceWeights(numConstraintsForCurve);

                        Polyline<T, 2> polyline(landmarkInstance.Points(landmarkConfiguration->IndicesForCurve(curveName)));

                        // Special handling of loop curves: we discard curve correspondences where the correspondence is looking at the "wrong" side of the loop.
                        // In order to not rely on the specific ordering of the landmarks or the mesh landmarks we can use the center of gravity of both
                        // the model and landmark points and compare direction this way.
                        const bool isLoop = meshLandmarksPtr->IsLoop(curveName);
                        Eigen::Vector2<T> gravityOfCurve(0, 0);
                        Eigen::Vector2<T> gravityOfModel(0, 0);
                        if (isLoop && numConstraintsForCurve > 0)
                        {
                            gravityOfCurve = polyline.ControlPoints().rowwise().mean();
                            gravityOfModel = projectedPixelPosition.Matrix().rowwise().mean();
                        }

                        if (curveResampling > 1) {
                            polyline = CatmullRom<T, 2>(polyline.ControlPoints(), curveResampling, /*closed=*/false).SampledPoints();
                        }
                        for (int k = 0; k < numConstraintsForCurve; k++) {
                            Eigen::Vector2<T> target;
                            Eigen::Vector2<T> normal;
                            T confidence;
                            polyline.ClosestPointAndNormal(projectedPixelPosition.Matrix().col(k), target, normal, confidence);
                            targetPixelPositions.col(k) = target;
                            targetPixelNormals.col(k) = normal;
                            correspondenceWeights[k] = userDefinedWeight * confidence;
                            if (isLoop && (projectedPixelPosition.Matrix().col(k) - gravityOfModel).dot(target - gravityOfCurve) < 0)
                                correspondenceWeights[k] = 0;

                            if (debugInfo) {
                                typename LandmarkConstraintsData<T>::ConstraintData constraintData{bcs[k], projectedPixelPosition.Matrix().col(k), targetPixelPositions.col(k), targetPixelNormals.col(k), correspondenceWeights[k]};
                                debugInfo->constraintDataPerCamera[camera.Label()].push_back(constraintData);
                            }
                        }

                        cost.Add(PointSurfaceConstraintFunction<T, 2>::Evaluate(projectedPixelPosition, targetPixelPositions,
                                                                                    targetPixelNormals, correspondenceWeights, T(1)), T(1), camera.Label() + "_" + this->MeshTypeToName(meshType) + curveName);
                    }
                }
            }
        }

        return cost;
    }

    template<class T>
    Cost<T> LandmarkConstraints2D<T>::EvaluateContours(const DiffDataMatrix<T, 3, -1>& vertices,
                                                       const Eigen::Matrix<T, 3, -1>& normals,
                                                       MeshType meshType,
                                                       LandmarkConstraintsData<T>* debugInfo) const {
        if (debugInfo) {
            debugInfo->constraintDataPerCamera.clear();
        }

        const int curveResampling = this->m_config["curveResampling"].template Value<int>();
        const bool constrainContourBorder = this->m_config["constrainContourBorder"].template Value<bool>();
        const MeshLandmarks<T>* meshLandmarksPtr = this->MeshLandmarksForType(meshType);

        Cost<T> cost;

        for (const auto& [landmarkInstance, camera] : m_targetLandmarks) {
            const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = landmarkInstance.GetLandmarkConfiguration();

            for (auto&& [contourName, contourVertexIDs] : meshLandmarksPtr->Contours()) {
                if (!landmarkConfiguration->HasCurve(contourName)) continue;
                const T userDefinedWeight = this->UserDefinedWeight(contourName);
                if (userDefinedWeight <= 0) continue;

                // get the contour points
                const std::vector<MeshContourPoint<T> >& meshContourPoints =  MeshContourPoint<T>::FindContourChanges(contourVertexIDs, vertices.Matrix(), normals, camera);

                // get the corresponding landmark curve
                const Eigen::Matrix<T, 2, -1> curvePts = landmarkInstance.Points(landmarkConfiguration->IndicesForCurve(contourName));

                if (meshContourPoints.size() > 0 && curvePts.cols() > 0) {
                    // convert to baryentric coordinates
                    std::vector<BarycentricCoordinates<T> > barycentricCoordinates;
                    for (const MeshContourPoint<T>&  meshContourPoint : meshContourPoints) {
                        barycentricCoordinates.push_back(BarycentricCoordinates<T>(Eigen::Vector3i(meshContourPoint.vID1, meshContourPoint.vID2, meshContourPoint.vID2),
                                                                                    Eigen::Vector3<T>(meshContourPoint.w1, T(1) - meshContourPoint.w1, T(0))));
                    }

                    // evaluate the contour points on the mesh
                    const DiffDataMatrix<T, 3, -1> evaluatedMeshContourPoints = BarycentricCoordinatesFunction<T, 3>::Evaluate(vertices, barycentricCoordinates);

                    // project the mesh points into the image
                    const DiffDataMatrix<T, 2, -1> projectedPixelPositions = camera.Project(evaluatedMeshContourPoints, /*withExtrinsics=*/ true);
                    const int numConstraintsForCurve = projectedPixelPositions.Cols();

                    // find the correspondences in the 2D curve
                    Eigen::Matrix<T, 2, -1> targetPixelPositions(2, numConstraintsForCurve);
                    Eigen::Matrix<T, 2, -1> targetPixelNormals(2, numConstraintsForCurve);
                    Eigen::VectorX<T> correspondenceWeights(numConstraintsForCurve);

                    Polyline<T, 2> polyline(curvePts);
                    if (curveResampling > 1) {
                        polyline = CatmullRom<T, 2>(polyline.ControlPoints(), curveResampling, /*closed=*/false).SampledPoints();
                    }
                    for (int k = 0; k < numConstraintsForCurve; k++) {
                        Eigen::Vector2<T> target;
                        Eigen::Vector2<T> normal;
                        T confidence;
                        polyline.ClosestPointAndNormal(projectedPixelPositions.Matrix().col(k), target, normal, confidence);
                        targetPixelPositions.col(k) = target;
                        targetPixelNormals.col(k) = normal;
                        const bool isValid = (meshContourPoints[k].type == MeshContourPoint<T>::Type::CONTOUR ||
                            (constrainContourBorder && meshContourPoints[k].type == MeshContourPoint<T>::Type::BORDER_BACK));
                        correspondenceWeights[k] = isValid ? (userDefinedWeight * confidence) : T(0);

                        if (debugInfo) {
                            typename LandmarkConstraintsData<T>::ConstraintData constraintData{barycentricCoordinates[k],
                                                                                                projectedPixelPositions.Matrix().col(k),
                                                                                                targetPixelPositions.col(k),
                                                                                                targetPixelNormals.col(k),
                                                                                                correspondenceWeights[k]};
                            debugInfo->constraintDataPerCamera[camera.Label()].push_back(constraintData);
                        }
                    }

                    // create the actual 2D point-line constraints
                    cost.Add(PointSurfaceConstraintFunction<T, 2>::Evaluate(projectedPixelPositions,
                                                                            targetPixelPositions,
                                                                            targetPixelNormals,
                                                                            correspondenceWeights,
                                                                            T(1)),
                                                                            T(1),
                                                                            camera.Label() + "_" + contourName);
                }
            }
        }

        return cost;
    }

    template<class T>
    Cost<T> LandmarkConstraints2D<T>::EvaluateInnerLips(const DiffDataMatrix<T, 3, -1>& vertices,
                                                        const Eigen::Matrix<T, 3, -1>& normals,
                                                        LandmarkConstraintsData<T>* debugInfoUpper,
                                                        LandmarkConstraintsData<T>* debugInfoLower) const {
        if (debugInfoUpper) {
            debugInfoUpper->constraintDataPerCamera.clear();
        }
        if (debugInfoLower) {
            debugInfoLower->constraintDataPerCamera.clear();
        }

        const int curveResampling = this->m_config["curveResampling"].template Value<int>();
        const bool constraintInnerLipBorder = this->m_config["constraintInnerLipBorder"].template Value<bool>();

        Cost<T> cost;

        for (const auto& [landmarkInstance, camera] : m_targetLandmarks) {
            const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = landmarkInstance.GetLandmarkConfiguration();

            const auto [meshContourPointsLower, meshContourPointsUpper] = MeshContourPoint<T>::FindContourChangesAndOcclusions(this->m_meshLandmarks.InnerLowerLipContourLines(),
                                                                                                                               this->m_meshLandmarks.InnerUpperLipContourLines(),
                                                                                                                               vertices.Matrix(),
                                                                                                                               normals,
                                                                                                                               camera);

            // evaluate lower and upper lip contours
            for (bool evaluateLower : {true, false}) {
                const std::vector<MeshContourPoint<T> >& meshContourPoints = evaluateLower ? meshContourPointsLower : meshContourPointsUpper;

                // get the curve points
                Eigen::Matrix<T, 2, -1> curvePts;
                const std::string curveName = evaluateLower ? "crv_lip_lower_inner" : "crv_lip_upper_inner";
                const std::string curveNameL = curveName + "_l";
                const std::string curveNameR = curveName + "_r";
                const bool hasCurveCombined = landmarkConfiguration->HasCurve(curveName);
                const bool hasCurveL = landmarkConfiguration->HasCurve(curveNameL);
                const bool hasCurveR = landmarkConfiguration->HasCurve(curveNameR);
                if (hasCurveCombined) {
                    curvePts = landmarkInstance.Points(landmarkConfiguration->IndicesForCurve(curveName));
                } else if (hasCurveL) {
                    curvePts = landmarkInstance.Points(landmarkConfiguration->IndicesForCurve(curveNameL));
                } else if (hasCurveR) {
                    curvePts = landmarkInstance.Points(landmarkConfiguration->IndicesForCurve(curveNameR));
                }

                if (meshContourPoints.size() > 0 && curvePts.cols() > 0) {
                    // convert to baryentric coordinates
                    std::vector<BarycentricCoordinates<T> > barycentricCoordinates;
                    for (const MeshContourPoint<T>&  meshContourPoint : meshContourPoints) {
                        barycentricCoordinates.push_back(BarycentricCoordinates<T>(Eigen::Vector3i(meshContourPoint.vID1, meshContourPoint.vID2, meshContourPoint.vID2),
                                                                                   Eigen::Vector3<T>(meshContourPoint.w1, T(1) - meshContourPoint.w1, T(0))));
                    }

                    // evaluate the contour points on the mesh
                    const DiffDataMatrix<T, 3, -1> evaluatedMeshContourPoints = BarycentricCoordinatesFunction<T, 3>::Evaluate(vertices, barycentricCoordinates);

                    // project the mesh points into the image
                    const DiffDataMatrix<T, 2, -1> projectedPixelPositions = camera.Project(evaluatedMeshContourPoints,
                                                                                            /*withExtrinsics=*/ true);
                    const int numConstraintsForCurve = projectedPixelPositions.Cols();

                    // find the correspondences in the 2D curve
                    Eigen::Matrix<T, 2, -1> targetPixelPositions(2, numConstraintsForCurve);
                    Eigen::Matrix<T, 2, -1> targetPixelNormals(2, numConstraintsForCurve);
                    Eigen::VectorX<T> correspondenceWeights(numConstraintsForCurve);

                    Polyline<T, 2> polyline(curvePts);
                    if (curveResampling > 1) {
                        polyline = CatmullRom<T, 2>(polyline.ControlPoints(), curveResampling, /*closed=*/false).SampledPoints();
                    }
                    for (int k = 0; k < numConstraintsForCurve; k++) {
                        Eigen::Vector2<T> target;
                        Eigen::Vector2<T> normal;
                        T confidence;
                        polyline.ClosestPointAndNormal(projectedPixelPositions.Matrix().col(k), target, normal, confidence);
                        targetPixelPositions.col(k) = target;
                        targetPixelNormals.col(k) = normal;
                        const bool isValid = (meshContourPoints[k].type == MeshContourPoint<T>::Type::CONTOUR ||
                            (constraintInnerLipBorder && meshContourPoints[k].type == MeshContourPoint<T>::Type::BORDER_BACK));
                        correspondenceWeights[k] = isValid ? confidence : T(0);

                        if (evaluateLower && debugInfoLower) {
                            typename LandmarkConstraintsData<T>::ConstraintData constraintData{barycentricCoordinates[k],
                                                                                               projectedPixelPositions.Matrix().col(k),
                                                                                               targetPixelPositions.col(k),
                                                                                               targetPixelNormals.col(k),
                                                                                               correspondenceWeights[k]};
                            debugInfoLower->constraintDataPerCamera[camera.Label()].push_back(constraintData);
                        } else if (!evaluateLower && debugInfoUpper) {
                            typename LandmarkConstraintsData<T>::ConstraintData constraintData{barycentricCoordinates[k],
                                                                                               projectedPixelPositions.Matrix().col(k),
                                                                                               targetPixelPositions.col(k),
                                                                                               targetPixelNormals.col(k),
                                                                                               correspondenceWeights[k]};
                            debugInfoUpper->constraintDataPerCamera[camera.Label()].push_back(constraintData);
                        }
                    }

                    // create the actual 2D point-line constraints
                    cost.Add(PointSurfaceConstraintFunction<T, 2>::Evaluate(projectedPixelPositions,
                                                                            targetPixelPositions,
                                                                            targetPixelNormals,
                                                                            correspondenceWeights,
                                                                            T(1)),
                                                                            T(1),
                                                                            camera.Label() + "_" + curveName);
                }
            }
        }

        return cost;
    }


    template<class T>
    void LandmarkConstraints2D<T>::SetupLandmarkConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                                            const Eigen::Matrix<T, 3, -1>& vertices,
                                                            const MeshLandmarks<T>* meshLandmarksPtr,
                                                            const MeshType meshType,
                                                            VertexConstraints<T, 2, 3>& landmarkVertexConstraints) const
    {
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

        const int maxNumLandmarks = int(m_targetLandmarks.size() * meshLandmarksPtr->LandmarksBarycentricCoordinates().size());
        landmarkVertexConstraints.ResizeToFitAdditionalConstraints(maxNumLandmarks);

        for (const auto& [landmarkInstance, camera] : m_targetLandmarks) {
            const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = landmarkInstance.GetLandmarkConfiguration();
            const Eigen::Matrix<T, 3, 3> K = camera.Intrinsics();
            const Eigen::Matrix<T, 4, 4> totalTransform = camera.Extrinsics().Matrix() * rigidTransform.matrix();
            const Eigen::Matrix<T, 3, 3> KR = K * totalTransform.block(0, 0, 3, 3);
            const Eigen::Vector<T, 3> Kt = K * totalTransform.block(0, 3, 3, 1);

            for (const auto& [landmarkName, bc] : meshLandmarksPtr->LandmarksBarycentricCoordinates()) {
                if (landmarkConfiguration->HasLandmark(landmarkName)) {
                    const T userDefinedWeight = this->UserDefinedWeight(landmarkName);
                    if (userDefinedWeight > 0) {
                        const int landmarkIndex = landmarkConfiguration->IndexForLandmark(landmarkName);
                        const T weight = sqrt(userDefinedWeight) * sqrt(landmarksWeight) * landmarkInstance.Confidence()[landmarkIndex];
                        if (weight <= 0) continue;
                        const Eigen::Vector2<T> targetPixelPosition = landmarkInstance.Points().col(landmarkIndex);
                        const Eigen::Vector3<T> pix = KR * bc.template Evaluate<3>(vertices) + Kt;
                        const T x = pix[0];
                        const T y = pix[1];
                        const T z = pix[2];
                        const T invZ = T(1) / z;
                        const Eigen::Vector2<T> residual = weight * (invZ * pix.template head<2>() - targetPixelPosition);

                        // dpix[0] / d(x, y, z) = [1/z,   0, -x/z^2]
                        // dpix[1] / d(x, y, z) = [  0, 1/z, -y/z^2]
                        // dpix[0] / d(vx, vy, vz) = dpix[0] / d(x, y, z) * d(x, y, z) / d(vx, vy, vz)
                        // d(x, y, z) / d(vx, vy, vz) = KR
                        Eigen::Matrix<T, 2, 9> drdV;
                        for (int k = 0; k < 3; ++k) {
                            drdV(0, 3 * k + 0) = weight * invZ * bc.Weight(k) * (KR(0, 0) - (x * invZ) * KR(2, 0));
                            drdV(0, 3 * k + 1) = weight * invZ * bc.Weight(k) * (KR(0, 1) - (x * invZ) * KR(2, 1));
                            drdV(0, 3 * k + 2) = weight * invZ * bc.Weight(k) * (KR(0, 2) - (x * invZ) * KR(2, 2));
                            drdV(1, 3 * k + 0) = weight * invZ * bc.Weight(k) * (KR(1, 0) - (y * invZ) * KR(2, 0));
                            drdV(1, 3 * k + 1) = weight * invZ * bc.Weight(k) * (KR(1, 1) - (y * invZ) * KR(2, 1));
                            drdV(1, 3 * k + 2) = weight * invZ * bc.Weight(k) * (KR(1, 2) - (y * invZ) * KR(2, 2));
                        }
                        landmarkVertexConstraints.AddConstraint(bc.Indices(), residual, drdV);
                    }
                }
            }
        }
    }

    template<class T>
    void LandmarkConstraints2D<T>::SetupCurveConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                                         const Eigen::Matrix<T, 3, -1>& vertices,
                                                         const MeshLandmarks<T>* meshLandmarksPtr,
                                                         const MeshType meshType,
                                                         VertexConstraints<T, 1, 3>& curveVertexConstraints) const
    {
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
        curveVertexConstraints.ResizeToFitAdditionalConstraints(m_targetLandmarks.size() * maxNumCurveConstraintsPerCamera);

        for (const auto& [landmarkInstance, camera] : m_targetLandmarks) {
            const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = landmarkInstance.GetLandmarkConfiguration();
            const Eigen::Matrix<T, 3, 3> K = camera.Intrinsics();
            const Eigen::Matrix<T, 4, 4> totalTransform = camera.Extrinsics().Matrix() * rigidTransform.matrix();
            const Eigen::Matrix<T, 3, 3> KR = K * totalTransform.block(0, 0, 3, 3);
            const Eigen::Vector<T, 3> Kt = K * totalTransform.block(0, 3, 3, 1);

            for (auto&& [curveName, bcs] : meshLandmarksPtr->MeshCurvesBarycentricCoordinates()) {
                if (landmarkConfiguration->HasCurve(curveName)) {
                    const T userDefinedWeight = this->UserDefinedWeight(curveName);
                    if (userDefinedWeight > 0) {
                        const int numConstraintsForCurve = int(bcs.size());

                        Polyline<T, 2> polyline(landmarkInstance.Points(landmarkConfiguration->IndicesForCurve(curveName)));

                        // Special handling of loop curves: we discard curve correspondences where the correspondence is looking at the "wrong" side of the loop.
                        // In order to not rely on the specific ordering of the landmarks or the mesh landmarks we can use the center of gravity of both
                        // the model and landmark points and compare direction this way.
                        const bool isLoop = meshLandmarksPtr->IsLoop(curveName);
                        Eigen::Vector2<T> gravityOfCurve(0, 0);
                        Eigen::Vector2<T> gravityOfModel(0 ,0);
                        if (isLoop && numConstraintsForCurve > 0) {
                            gravityOfCurve = polyline.ControlPoints().rowwise().mean();
                            for (int i = 0; i < numConstraintsForCurve; ++i) {
                                const Eigen::Vector3<T> pix = KR * bcs[i].template Evaluate<3>(vertices) + Kt;
                                gravityOfModel += pix.template head<2>() / pix[2];
                            }
                            gravityOfModel /= T(numConstraintsForCurve);
                        }

                        if (curveResampling > 1) {
                            polyline = CatmullRom<T, 2>(polyline.ControlPoints(), curveResampling, /*closed=*/false).SampledPoints();
                        }
                        for (int i = 0; i < numConstraintsForCurve; i++) {
                            const Eigen::Vector3<T> pix = KR * bcs[i].template Evaluate<3>(vertices) + Kt;
                            const T x = pix[0];
                            const T y = pix[1];
                            const T z = pix[2];
                            const T invZ = T(1) / z;
                            const Eigen::Vector2<T> p = pix.template head<2>() * invZ;
                            Eigen::Vector2<T> target;
                            Eigen::Vector2<T> normal;
                            T confidence;
                            polyline.ClosestPointAndNormal(p, target, normal, confidence);

                            const T weight = sqrt(userDefinedWeight * landmarksWeight) * confidence;
                            if (weight <= 0) continue;
                            if (isLoop && (p - gravityOfModel).dot(target - gravityOfCurve) < 0) continue;

                            const T residual = weight * normal.dot(invZ * pix.template head<2>() - target);

                            Eigen::Matrix<T, 1, 9> drdV;
                            for (int k = 0; k < 3; ++k) {
                                drdV(0, 3 * k + 0) = weight * invZ * bcs[i].Weight(k) * (normal[0] * (KR(0, 0) - (x * invZ) * KR(2, 0)) + normal[1] * (KR(1, 0) - (y * invZ) * KR(2, 0)));
                                drdV(0, 3 * k + 1) = weight * invZ * bcs[i].Weight(k) * (normal[0] * (KR(0, 1) - (x * invZ) * KR(2, 1)) + normal[1] * (KR(1, 1) - (y * invZ) * KR(2, 1)));
                                drdV(0, 3 * k + 2) = weight * invZ * bcs[i].Weight(k) * (normal[0] * (KR(0, 2) - (x * invZ) * KR(2, 2)) + normal[1] * (KR(1, 2) - (y * invZ) * KR(2, 2)));
                            }

                            curveVertexConstraints.AddConstraint(bcs[i].Indices(), Eigen::Vector<T, 1>(residual), drdV);
                        }
                    }
                }
            }
        }
    }

    template<class T>
    void LandmarkConstraints2D<T>::SetupContourConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                                            const Eigen::Matrix<T, 3, -1>& vertices,
                                                            const Eigen::Matrix<T, 3, -1>& normals,
                                                            const MeshLandmarks<T>* meshLandmarksPtr,
                                                            const MeshType meshType,
                                                            VertexConstraints<T, 1, 2>& contourVertexConstraints) const
    {
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

        const int curveResampling = this->m_config["curveResampling"].template Value<int>();
        const bool constrainContourBorder = this->m_config["constrainContourBorder"].template Value<bool>();

        int numContourConstraints = 0;
        for (const auto& [landmarkInstance, camera] : m_targetLandmarks) {
            const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = landmarkInstance.GetLandmarkConfiguration();
            for (auto&& [contourName, contourVertexIDs] : meshLandmarksPtr->Contours()) {
                if (!landmarkConfiguration->HasCurve(contourName)) continue;
                numContourConstraints += int(contourVertexIDs.size());
            }
        }


        contourVertexConstraints.ResizeToFitAdditionalConstraints(numContourConstraints);

        for (const auto& [landmarkInstance, camera] : m_targetLandmarks) {
            const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = landmarkInstance.GetLandmarkConfiguration();
            const Eigen::Matrix<T, 3, 3> K = camera.Intrinsics();
            const Eigen::Matrix<T, 4, 4> totalTransform = camera.Extrinsics().Matrix() * rigidTransform.matrix();
            const Eigen::Matrix<T, 3, 3> KR = K * totalTransform.block(0, 0, 3, 3);
            const Eigen::Vector<T, 3> Kt = K * totalTransform.block(0, 3, 3, 1);
            Camera<T> viewCamera = camera;
            viewCamera.SetExtrinsics(Affine<T, 3, 3>(totalTransform.matrix()));


            for (auto&& [contourName, contourVertexIDs] : meshLandmarksPtr->Contours()) {
                if (!landmarkConfiguration->HasCurve(contourName)) continue;

                // get the contour points
                const std::vector<MeshContourPoint<T> >& meshContourPoints =  MeshContourPoint<T>::FindContourChanges(contourVertexIDs, vertices, normals, viewCamera);

                // get the corresponding landmark curve
                const Eigen::Matrix<T, 2, -1> curvePts = landmarkInstance.Points(landmarkConfiguration->IndicesForCurve(contourName));

                if (meshContourPoints.size() > 0 && curvePts.cols() > 0) {
                    Polyline<T, 2> polyline(curvePts);
                    if (curveResampling > 1) {
                        polyline = CatmullRom<T, 2>(polyline.ControlPoints(), curveResampling, /*closed=*/false).SampledPoints();
                    }
                    for (size_t i = 0; i < meshContourPoints.size(); i++) {
                        const int vID1 = meshContourPoints[i].vID1;
                        const int vID2 = meshContourPoints[i].vID2;
                        const T w1 = meshContourPoints[i].w1;
                        const T w2 = T(1) - w1;

                        const Eigen::Vector3<T> pix = KR * (vertices.col(vID1) * w1 + vertices.col(vID2) * w2) + Kt;
                        const T x = pix[0];
                        const T y = pix[1];
                        const T z = pix[2];
                        const T invZ = T(1) / z;
                        Eigen::Vector2<T> target;
                        Eigen::Vector2<T> normal;
                        T confidence;
                        polyline.ClosestPointAndNormal(pix.template head<2>() * invZ, target, normal, confidence);

                        const T weight = sqrt(landmarksWeight) * confidence;
                        const bool isValid = (meshContourPoints[i].type == MeshContourPoint<T>::Type::CONTOUR ||
                            (constrainContourBorder && meshContourPoints[i].type == MeshContourPoint<T>::Type::BORDER_BACK));
                        if (weight <= 0 || !isValid) continue;

                        const T residual = weight * normal.dot(invZ * pix.template head<2>() - target);

                        Eigen::Matrix<T, 1, 6> drdV;
                        for (int k = 0; k < 2; ++k) {
                            drdV(0, 3 * k + 0) = weight * invZ * (k == 0 ? w1 : w2) * (normal[0] * (KR(0, 0) - (x * invZ) * KR(2, 0)) + normal[1] * (KR(1, 0) - (y * invZ) * KR(2, 0)));
                            drdV(0, 3 * k + 1) = weight * invZ * (k == 0 ? w1 : w2) * (normal[0] * (KR(0, 1) - (x * invZ) * KR(2, 1)) + normal[1] * (KR(1, 1) - (y * invZ) * KR(2, 1)));
                            drdV(0, 3 * k + 2) = weight * invZ * (k == 0 ? w1 : w2) * (normal[0] * (KR(0, 2) - (x * invZ) * KR(2, 2)) + normal[1] * (KR(1, 2) - (y * invZ) * KR(2, 2)));
                        }

                        contourVertexConstraints.AddConstraint(Eigen::Vector2i(vID1, vID2), Eigen::Vector<T, 1>(residual), drdV);
                    }
                }
            }
        }
    }

    template<class T>
    void LandmarkConstraints2D<T>::SetupInnerLipConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                                            const Eigen::Matrix<T, 3, -1>& vertices,
                                                            const Eigen::Matrix<T, 3, -1>& normals,
                                                            const MeshLandmarks<T>* meshLandmarksPtr,
                                                            VertexConstraints<T, 1, 2>& innerLipVertexConstraints) const
    {
        if (!meshLandmarksPtr) return;

        const T innerLipWeight = this->m_config["innerLipWeight"].template Value<T>();
        const bool constraintInnerLipBorder = this->m_config["constraintInnerLipBorder"].template Value<bool>();
        const int curveResampling = this->m_config["curveResampling"].template Value<int>();

        if (innerLipWeight <= 0) return;

        innerLipVertexConstraints.ResizeToFitAdditionalConstraints(m_targetLandmarks.size() *
                                                                   int(meshLandmarksPtr->InnerLowerLipContourLines().size() +
                                                                       meshLandmarksPtr->InnerUpperLipContourLines().size()));

        for (const auto& [landmarkInstance, camera] : m_targetLandmarks) {
            const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration = landmarkInstance.GetLandmarkConfiguration();
            const Eigen::Matrix<T, 3, 3> K = camera.Intrinsics();
            const Eigen::Matrix<T, 4, 4> totalTransform = camera.Extrinsics().Matrix() * rigidTransform.matrix();
            const Eigen::Matrix<T, 3, 3> KR = K * totalTransform.block(0, 0, 3, 3);
            const Eigen::Vector<T, 3> Kt = K * totalTransform.block(0, 3, 3, 1);

            Camera<T> lipCamera = camera;
            lipCamera.SetExtrinsics(Affine<T, 3, 3>(totalTransform.matrix()));
            const auto [meshContourPointsLower, meshContourPointsUpper] = MeshContourPoint<T>::FindContourChangesAndOcclusions(meshLandmarksPtr->InnerLowerLipContourLines(),
                                                                                                                               meshLandmarksPtr->InnerUpperLipContourLines(),
                                                                                                                               vertices,
                                                                                                                               normals,
                                                                                                                               lipCamera);

            // evaluate lower and upper lip contours
            for (bool evaluateLower : {true, false}) {
                const std::vector<MeshContourPoint<T> >& meshContourPoints = evaluateLower ? meshContourPointsLower : meshContourPointsUpper;

                // get the curve points
                Eigen::Matrix<T, 2, -1> curvePts;
                const std::string curveName = evaluateLower ? "crv_lip_lower_inner" : "crv_lip_upper_inner";
                const std::string curveNameL = curveName + "_l";
                const std::string curveNameR = curveName + "_r";
                const bool hasCurveCombined = landmarkConfiguration->HasCurve(curveName);
                const bool hasCurveL = landmarkConfiguration->HasCurve(curveNameL);
                const bool hasCurveR = landmarkConfiguration->HasCurve(curveNameR);
                if (hasCurveCombined) {
                    curvePts = landmarkInstance.Points(landmarkConfiguration->IndicesForCurve(curveName));
                } else if (hasCurveL) {
                    curvePts = landmarkInstance.Points(landmarkConfiguration->IndicesForCurve(curveNameL));
                } else if (hasCurveR) {
                    curvePts = landmarkInstance.Points(landmarkConfiguration->IndicesForCurve(curveNameR));
                }

                if (meshContourPoints.size() > 0 && curvePts.cols() > 0) {
                    Polyline<T, 2> polyline(curvePts);
                    if (curveResampling > 1) {
                        polyline = CatmullRom<T, 2>(polyline.ControlPoints(), curveResampling, /*closed=*/false).SampledPoints();
                    }
                    for (size_t i = 0; i < meshContourPoints.size(); i++) {
                        const int vID1 = meshContourPoints[i].vID1;
                        const int vID2 = meshContourPoints[i].vID2;
                        const T w1 = meshContourPoints[i].w1;
                        const T w2 = T(1) - w1;

                        const Eigen::Vector3<T> pix = KR * (vertices.col(vID1) * w1 + vertices.col(vID2) * w2) + Kt;
                        const T x = pix[0];
                        const T y = pix[1];
                        const T z = pix[2];
                        const T invZ = T(1) / z;
                        Eigen::Vector2<T> target;
                        Eigen::Vector2<T> normal;
                        T confidence;
                        polyline.ClosestPointAndNormal(pix.template head<2>() * invZ, target, normal, confidence);

                        const T weight = sqrt(innerLipWeight) * confidence;
                        const bool isValid = (meshContourPoints[i].type == MeshContourPoint<T>::Type::CONTOUR ||
                            (constraintInnerLipBorder && meshContourPoints[i].type == MeshContourPoint<T>::Type::BORDER_BACK));
                        if (weight <= 0 || !isValid) continue;

                        const T residual = weight * normal.dot(invZ * pix.template head<2>() - target);

                        Eigen::Matrix<T, 1, 6> drdV;
                        for (int k = 0; k < 2; ++k) {
                            drdV(0, 3 * k + 0) = weight * invZ * (k == 0 ? w1 : w2) * (normal[0] * (KR(0, 0) - (x * invZ) * KR(2, 0)) + normal[1] * (KR(1, 0) - (y * invZ) * KR(2, 0)));
                            drdV(0, 3 * k + 1) = weight * invZ * (k == 0 ? w1 : w2) * (normal[0] * (KR(0, 1) - (x * invZ) * KR(2, 1)) + normal[1] * (KR(1, 1) - (y * invZ) * KR(2, 1)));
                            drdV(0, 3 * k + 2) = weight * invZ * (k == 0 ? w1 : w2) * (normal[0] * (KR(0, 2) - (x * invZ) * KR(2, 2)) + normal[1] * (KR(1, 2) - (y * invZ) * KR(2, 2)));
                        }

                        innerLipVertexConstraints.AddConstraint(Eigen::Vector2i(vID1, vID2), Eigen::Vector<T, 1>(residual), drdV);
                    }
                }
            }
        }
    }

    template class LandmarkConstraints2D<float>;
    template class LandmarkConstraints2D<double>;
}  // namespace epic::nls
