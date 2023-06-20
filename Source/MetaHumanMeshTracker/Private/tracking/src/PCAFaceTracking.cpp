// Copyright Epic Games, Inc. All Rights Reserved.

#include <tracking/PCAFaceTracking.h>
#include <nls/math/ParallelBLAS.h>
namespace epic::nls {

void PCAFaceTracking::FitPCAData(const Mesh<float>& topology,
                std::vector<DepthmapConstraints>& vectorOfDepthmapConstraints,
                ICPConstraints<float>* icpConstraints,
                LandmarkConstraints2D<float>* landmarkConstraints,
                const std::vector<FlowConstraints<float>*>& vectorOfFlowConstraints,
                Affine<float, 3, 3>& rigidMotion,
                Eigen::VectorXf& pcaCoeffs,
                const std::vector<Eigen::VectorXf>& pcaCoeffsPrevFrames,
                const Settings& settings,
                std::vector<State>& states) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_NAVY);

    // Timer timer;

    if (states.empty()) {
        states.emplace_back(State());
    }
    const int stateId = 0;

    const int numPcaParameters = int(pcaCoeffs.size());
    const int numTotalParameters = numPcaParameters + (settings.withRigid ? 6 : 0);

    Eigen::VectorXf params(numTotalParameters);
    params.segment(0, numPcaParameters) = pcaCoeffs;
    if (settings.withRigid) params.tail(6).setZero();

    QRigidMotion<float> qrm(rigidMotion.Matrix());

    Eigen::Matrix<float, 3, -1> faceNormals;

    Eigen::Matrix<float, -1, -1> AtA(numTotalParameters, numTotalParameters);
    Eigen::VectorXf Atb(numTotalParameters);
    // LOG_INFO("start: {}", timer.Current()); timer.Restart();

    const auto evaluationMode = settings.withRigid ? rt::LinearVertexModel<float>::EvaluationMode::RIGID : rt::LinearVertexModel<float>::EvaluationMode::STATIC;

    for (int iter = 0; iter < settings.iterations; ++iter) {

        AtA.setZero();
        Atb.setZero();
        // LOG_INFO("zero: {}", timer.Current()); timer.Restart();

        // evaluate the vertices for the current parameters
        m_pcaRig.facePCA.EvaluateLinearized(params, evaluationMode, states[stateId].face);
        // LOG_INFO("eval: {}", timer.Current()); timer.Restart();
        //m_pcaRig.teethPCA.Evaluate(params, evaluationMode, states[stateId].teeth);
        m_pcaRigSubsampled.teethPCA.EvaluateLinearized(params, evaluationMode, states[stateId].teeth);
        // LOG_INFO("eval: {}", timer.Current()); timer.Restart();
        // m_pcaRig.eyeLeftPCA.Evaluate(params, evaluationMode, states[stateId].eyeLeft);
        // m_pcaRig.eyeRightPCA.Evaluate(params, evaluationMode, states[stateId].eyeRight);
        m_pcaRigSubsampled.eyeLeftTransformPCA.EvaluateVerticesAndJacobian(params, evaluationMode, states[stateId].eyeLeft);
        // LOG_INFO("eval: {}", timer.Current()); timer.Restart();
        m_pcaRigSubsampled.eyeRightTransformPCA.EvaluateVerticesAndJacobian(params, evaluationMode, states[stateId].eyeRight);
        // LOG_INFO("eval: {}", timer.Current()); timer.Restart();

        // // update rigid transform jacobian
        // states[stateId].face.SetRotationModes(states[stateId].face.Base());
        // states[stateId].teeth.SetRotationModes(states[stateId].teeth.Base());
        // states[stateId].eyeLeft.SetRotationModes(states[stateId].eyeLeft.Base());
        // states[stateId].eyeRight.SetRotationModes(states[stateId].eyeRight.Base());
        // instead keep rotation jacobian using the cross product matrix with the mean pca face. it is not correct, but should be sufficient as approximation

        // calculate the normals for the head mesh
        topology.CalculateVertexNormals(states[stateId].face.Base(), faceNormals, VertexNormalComputationType::AreaWeighted, /*stableNormalize=*/false);
        // LOG_INFO("normals: {}", timer.Current()); timer.Restart();

        // clear all vertex constraints
        for (auto& state : states) {
            state.cache.Clear();
        }

        // setup depth constraints (point2surface)
        if (vectorOfDepthmapConstraints.size() > 0) {
            for (size_t i = 0; i < vectorOfDepthmapConstraints.size(); ++i) {
                vectorOfDepthmapConstraints[i].SetupDepthConstraints(qrm.ToEigenTransform(), states[stateId].face.Base(), faceNormals, states[stateId].cache.point2SurfaceVertexConstraints);
            }
            // LOG_INFO("iter depth: {}", timer.Current()); timer.Restart();
        }

        // setup icp constraints (point2surface and point2point)
        if (icpConstraints) {
            icpConstraints->SetupICPConstraints(qrm.ToEigenTransform(), states[stateId].face.Base(), faceNormals, states[stateId].cache.point2SurfaceVertexConstraints, states[stateId].cache.point2PointVertexConstraints);
            // LOG_INFO("iter icp: {}", timer.Current()); timer.Restart();
        }

        // evaluate point2surface vertex constraints
        if (states[stateId].cache.point2SurfaceVertexConstraints.NumberOfConstraints() > 0) {
            const auto jacobian = states[stateId].cache.point2SurfaceVertexConstraints.EvaluateJacobian(states[stateId].face.Modes(evaluationMode), states[stateId].cache.point2SurfaceVertexConstraintsJacobian, m_globalThreadPool.get());
            ParallelAtALowerAdd(AtA, jacobian);
            Atb.noalias() += - jacobian.transpose() * states[stateId].cache.point2SurfaceVertexConstraints.Residual();
            // LOG_INFO("iter p2s: {}", timer.Current()); timer.Restart();
        }
        // evaluate point2point vertex constraints
        if (states[stateId].cache.point2PointVertexConstraints.NumberOfConstraints() > 0) {
            const auto jacobian = states[stateId].cache.point2PointVertexConstraints.EvaluateJacobian( states[stateId].face.Modes(evaluationMode), states[stateId].cache.point2PointVertexConstraintsJacobian);
            ParallelAtALowerAdd(AtA, jacobian);
            Atb.noalias() += - jacobian.transpose() * states[stateId].cache.point2PointVertexConstraints.Residual();
            // LOG_INFO("iter p2p: {}", timer.Current()); timer.Restart();
        }

        if (landmarkConstraints) {
            landmarkConstraints->SetupLandmarkConstraints(qrm.ToEigenTransform(), states[stateId].face.Base(), &m_subsampledFaceMeshLandmarks, LandmarkConstraintsBase<float>::MeshType::Face, states[stateId].cache.landmarksVertexConstraints);
            landmarkConstraints->SetupCurveConstraints(qrm.ToEigenTransform(), states[stateId].face.Base(), &m_subsampledFaceMeshLandmarks, LandmarkConstraintsBase<float>::MeshType::Face, states[stateId].cache.curvesVertexConstraints);
            landmarkConstraints->SetupContourConstraints(qrm.ToEigenTransform(), states[stateId].face.Base(), faceNormals, &m_subsampledFaceMeshLandmarks, LandmarkConstraintsBase<float>::MeshType::Face, states[stateId].cache.contourVertexConstraints);
            landmarkConstraints->SetupInnerLipConstraints(qrm.ToEigenTransform(), states[stateId].face.Base(), faceNormals, &m_subsampledFaceMeshLandmarks, states[stateId].cache.contourVertexConstraints);
            landmarkConstraints->SetupCurveConstraints(qrm.ToEigenTransform(), states[stateId].eyeLeft.Base(), &m_subsampledEyeLeftMeshLandmarks, LandmarkConstraintsBase<float>::MeshType::EyeLeft, states[stateId].cache.eyeLeftCurvesVertexConstraints);
            landmarkConstraints->SetupCurveConstraints(qrm.ToEigenTransform(), states[stateId].eyeRight.Base(), &m_subsampledEyeRightMeshLandmarks, LandmarkConstraintsBase<float>::MeshType::EyeRight, states[stateId].cache.eyeRightCurvesVertexConstraints);
            landmarkConstraints->SetupLandmarkConstraints(qrm.ToEigenTransform(), states[stateId].teeth.Base(), &m_subsampledTeethMeshLandmarks, LandmarkConstraintsBase<float>::MeshType::Teeth, states[stateId].cache.teethVertexConstraints);

            if (states[stateId].cache.landmarksVertexConstraints.NumberOfConstraints() > 0) {
                const auto jacobian = states[stateId].cache.landmarksVertexConstraints.EvaluateJacobian(states[stateId].face.Modes(evaluationMode), states[stateId].cache.landmarksVertexConstraintsJacobian);
                AtA.template triangularView<Eigen::Lower>() += jacobian.transpose() * jacobian;
                Atb.noalias() += - jacobian.transpose() * states[stateId].cache.landmarksVertexConstraints.Residual();
            }
            if (states[stateId].cache.curvesVertexConstraints.NumberOfConstraints() > 0) {
                const auto jacobian = states[stateId].cache.curvesVertexConstraints.EvaluateJacobian(states[stateId].face.Modes(evaluationMode), states[stateId].cache.curvesVertexConstraintsJacobian);
                AtA.template triangularView<Eigen::Lower>() += jacobian.transpose() * jacobian;
                Atb.noalias() += - jacobian.transpose() * states[stateId].cache.curvesVertexConstraints.Residual();
            }
            if (states[stateId].cache.contourVertexConstraints.NumberOfConstraints() > 0) {
                const auto jacobian = states[stateId].cache.contourVertexConstraints.EvaluateJacobian(states[stateId].face.Modes(evaluationMode), states[stateId].cache.contourVertexConstraintsJacobian);
                AtA.template triangularView<Eigen::Lower>() += jacobian.transpose() * jacobian;
                Atb.noalias() += - jacobian.transpose() * states[stateId].cache.contourVertexConstraints.Residual();
            }
            if (states[stateId].cache.eyeLeftCurvesVertexConstraints.NumberOfConstraints() > 0) {
                const auto jacobian = states[stateId].cache.eyeLeftCurvesVertexConstraints.EvaluateJacobian(states[stateId].eyeLeft.Modes(evaluationMode), states[stateId].cache.eyeLeftCurvesVertexConstraintsJacobian);
                AtA.template triangularView<Eigen::Lower>() += jacobian.transpose() * jacobian;
                Atb.noalias() += - jacobian.transpose() * states[stateId].cache.eyeLeftCurvesVertexConstraints.Residual();
            }
            if (states[stateId].cache.eyeRightCurvesVertexConstraints.NumberOfConstraints() > 0) {
                const auto jacobian = states[stateId].cache.eyeRightCurvesVertexConstraints.EvaluateJacobian(states[stateId].eyeRight.Modes(evaluationMode), states[stateId].cache.eyeRightCurvesVertexConstraintsJacobian);
                AtA.template triangularView<Eigen::Lower>() += jacobian.transpose() * jacobian;
                Atb.noalias() += - jacobian.transpose() * states[stateId].cache.eyeRightCurvesVertexConstraints.Residual();
            }
            if (states[stateId].cache.teethVertexConstraints.NumberOfConstraints() > 0) {
                const auto jacobian = states[stateId].cache.teethVertexConstraints.EvaluateJacobian(states[stateId].teeth.Modes(evaluationMode), states[stateId].cache.teethVertexConstraintsJacobian);
                AtA.template triangularView<Eigen::Lower>() += jacobian.transpose() * jacobian;
                Atb.noalias() += - jacobian.transpose() * states[stateId].cache.teethVertexConstraints.Residual();
            }
            // LOG_INFO("iter landmarks: {}", timer.Current()); timer.Restart();
        }

        // setup flow constraints
        if (vectorOfFlowConstraints.size() > 0) {
            for (size_t i = 0; i < vectorOfFlowConstraints.size(); ++i) {
                vectorOfFlowConstraints[i]->SetupFlowConstraints(qrm.ToEigenTransform(), states[stateId].face.Base(), states[stateId].cache.flowVertexConstraints);
            }
            if (states[stateId].cache.flowVertexConstraints.NumberOfConstraints() > 0) {
                const auto jacobian = states[stateId].cache.flowVertexConstraints.EvaluateJacobian(states[stateId].face.Modes(evaluationMode), states[stateId].cache.flowVertexConstraintsJacobian);
                ParallelAtALowerAdd(AtA, jacobian);
                Atb.noalias() += - jacobian.transpose() * states[stateId].cache.flowVertexConstraints.Residual();
            }
            // LOG_INFO("iter flow: {}", timer.Current()); timer.Restart();
        }

        // add regularization
        for (int k = 0; k < numPcaParameters; ++k) {
            AtA(k, k) += settings.pcaRegularization;
            Atb[k] -= settings.pcaRegularization * params[k];
        }

        // add temporal constraints on pca coefficients
        if (pcaCoeffsPrevFrames.size() > 0 && settings.pcaVelocityRegularization > 0) {
            for (int k = 0; k < numPcaParameters; ++k) {
                AtA(k, k) += settings.pcaVelocityRegularization;
                Atb[k] -= settings.pcaVelocityRegularization * (params[k] - pcaCoeffsPrevFrames[0][k]);
            }
        }
        if (pcaCoeffsPrevFrames.size() > 1 && settings.pcaAccelerationRegularization > 0) {
            for (int k = 0; k < numPcaParameters; ++k) {
                AtA(k, k) += settings.pcaAccelerationRegularization;
                Atb[k] -= settings.pcaAccelerationRegularization * (params[k] - 2 * pcaCoeffsPrevFrames[0][k] + pcaCoeffsPrevFrames[1][k]);
            }
        }
        // LOG_INFO("iter regularization: {}", timer.Current()); timer.Restart();

        // solve
        const Eigen::VectorX<float> dx = AtA.template selfadjointView<Eigen::Lower>().llt().solve(Atb);

        params.segment(0, numPcaParameters) += dx.segment(0, numPcaParameters);

        if (settings.withRigid) {
            // extract rigid motion from params
            qrm.t += qrm.q._transformVector(dx.tail(3));
            qrm.q = (qrm.q * Eigen::Quaternion<float>(1.0f, dx[numPcaParameters + 0], dx[numPcaParameters + 1], dx[numPcaParameters + 2])).normalized();
            // set rigid motion explicitly to zero
            params.tail(6).setZero();
        }
        // LOG_INFO("iter solve: {}", timer.Current()); timer.Restart();
    }

    rigidMotion.SetMatrix(qrm.ToEigenTransform().matrix());
    pcaCoeffs = params.segment(0, numPcaParameters);
    // LOG_INFO("total: {}", timer.Current()); timer.Restart();
}

PCAFaceTracking::PCAFaceTracking()
    : m_globalThreadPool(epic::carbon::TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/ true))
{}

PCAFaceTracking::~PCAFaceTracking() = default;

bool PCAFaceTracking::LoadPCARig(const std::string& pcaFilename)
{
    if (!m_pcaRig.LoadFromDNA(pcaFilename)) {
        LOG_ERROR("failed to read pca model from {}", pcaFilename);
        return false;
    }

    LOG_INFO("number of pca coeffs: {}", m_pcaRig.NumCoeffs());
    // move the midpoint of the eyes to the origin so that rotation is optimal for optimization
    const Eigen::Vector3f pcaRigEyesMidPoint = m_pcaRig.EyesMidpoint();
    m_pcaRig.Translate(-pcaRigEyesMidPoint);
    m_pcaRigToMesh.SetTranslation(pcaRigEyesMidPoint);

    UpdateSubsampled();

    return true;
}

bool PCAFaceTracking::LoadPCARig(dna::StreamReader* dnaStream)
{
    if (!m_pcaRig.LoadFromDNA(dnaStream)) {
        LOG_ERROR("could not load PCA rig");
        return false;
    }

    LOG_INFO("number of pca coeffs: {}", m_pcaRig.NumCoeffs());
    // move the midpoint of the eyes to the origin so that rotation is optimal for optimization
    const Eigen::Vector3f pcaRigEyesMidPoint = m_pcaRig.EyesMidpoint();
    m_pcaRig.Translate(-pcaRigEyesMidPoint);
    m_pcaRigToMesh.SetTranslation(pcaRigEyesMidPoint);

    UpdateSubsampled();

    return true;
}

bool PCAFaceTracking::SavePCARig(dna::StreamWriter* dnaStream) const
{
    // translate the PCA rig back to its original position before saving it
    auto pcaRigCopy = m_pcaRig;
    pcaRigCopy.Translate(m_pcaRigToMesh.Translation());
    pcaRigCopy.SaveAsDNA(dnaStream);

    return true;
}

void PCAFaceTracking::SavePCARigAsNpy(const std::string& filename) const
{
    auto pcaRigCopy = m_pcaRig;
    pcaRigCopy.Translate(m_pcaRigToMesh.Translation());
    pcaRigCopy.SaveAsNpy(filename);
}

void PCAFaceTracking::UpdateSubsampled()
{
    m_pcaRigSubsampled = m_pcaRig;

    LoadFaceMeshLandmarks(m_faceMeshLandmarks);
    LoadEyeLeftMeshLandmarks(m_eyeLeftMeshLandmarks);
    LoadEyeRightMeshLandmarks(m_eyeRightMeshLandmarks);
    LoadTeethMeshLandmarks(m_teethMeshLandmarks);
}
namespace {
    auto setToVec = [](const std::set<int>& set) {
        return std::vector<int>(set.begin(), set.end());
    };

    auto invertMap = [](const std::vector<int>& vec) {
        std::map<int, int> map;
        for (int newId = 0; newId < int(vec.size()); ++newId) {
            map[vec[newId]] = newId;
        }
        return map;
    };
}

void PCAFaceTracking::LoadFaceMeshLandmarks(const MeshLandmarks<float>& faceMeshLandmarks)
{
    m_faceMeshLandmarks = faceMeshLandmarks;
    m_subsampledFaceMeshLandmarks = faceMeshLandmarks;
}

void PCAFaceTracking::LoadEyeLeftMeshLandmarks(const MeshLandmarks<float>& eyeLeftMeshLandmarks)
{
    m_eyeLeftMeshLandmarks = eyeLeftMeshLandmarks;
    const auto eyeLeftSubsampledToFullMap = setToVec(eyeLeftMeshLandmarks.GetAllVertexIndices());
    m_subsampledEyeLeftMeshLandmarks = eyeLeftMeshLandmarks;
    m_subsampledEyeLeftMeshLandmarks.Remap(invertMap(eyeLeftSubsampledToFullMap));
    m_pcaRigSubsampled.eyeLeftTransformPCA = m_pcaRig.eyeLeftTransformPCA;
    m_pcaRigSubsampled.eyeLeftTransformPCA.Resample(eyeLeftSubsampledToFullMap);
    // LOG_INFO("reduce eye left size from {} to {}", m_pcaRig.eyeLeftTransformPCA.eyeBody.baseVertices.cols(), m_pcaRigSubsampled.eyeLeftTransformPCA.eyeBody.baseVertices.cols());
}

void PCAFaceTracking::LoadEyeRightMeshLandmarks(const MeshLandmarks<float>& eyeRightMeshLandmarks)
{
    m_eyeRightMeshLandmarks = eyeRightMeshLandmarks;
    const auto eyeRightSubsampledToFullMap = setToVec(eyeRightMeshLandmarks.GetAllVertexIndices());
    m_subsampledEyeRightMeshLandmarks = eyeRightMeshLandmarks;
    m_subsampledEyeRightMeshLandmarks.Remap(invertMap(eyeRightSubsampledToFullMap));
    m_pcaRigSubsampled.eyeRightTransformPCA = m_pcaRig.eyeRightTransformPCA;
    m_pcaRigSubsampled.eyeRightTransformPCA.Resample(eyeRightSubsampledToFullMap);
    // LOG_INFO("reduce eye right size from {} to {}", m_pcaRig.eyeRightTransformPCA.eyeBody.baseVertices.cols(), m_pcaRigSubsampled.eyeRightTransformPCA.eyeBody.baseVertices.cols());
}

void PCAFaceTracking::LoadTeethMeshLandmarks(const MeshLandmarks<float>& teethMeshLandmarks)
{
    m_teethMeshLandmarks = teethMeshLandmarks;
    const auto teethSubsampledToFullMap = setToVec(teethMeshLandmarks.GetAllVertexIndices());
    m_subsampledTeethMeshLandmarks = teethMeshLandmarks;
    m_subsampledTeethMeshLandmarks.Remap(invertMap(teethSubsampledToFullMap));
    m_pcaRigSubsampled.teethPCA = m_pcaRig.teethPCA;
    m_pcaRigSubsampled.teethPCA.Resample(teethSubsampledToFullMap);
    // LOG_INFO("reduce teeth size from {} to {}", m_pcaRig.teethPCA.NumVertices(), m_pcaRigSubsampled.teethPCA.NumVertices());
}

} // namespace epic::nls
