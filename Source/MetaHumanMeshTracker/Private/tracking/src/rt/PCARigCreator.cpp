// Copyright Epic Games, Inc. All Rights Reserved.

#include <tracking/rt/PCARigCreator.h>
#include <nls/geometry/PCA.h>
#include <carbon/io/JsonIO.h>
#include <carbon/io/Utils.h>
#include <carbon/utils/Timer.h>

namespace epic::nls::rt {

    bool PCARigCreator::LoadConfigFile(const std::string& configFilename) {

        try
        {
            const epic::carbon::JsonElement configJson = epic::carbon::ReadJson(epic::carbon::ReadFile(configFilename));
            m_rawControlDefaults = configJson["default_raw_controls"].Get<std::map<std::string, float>>();
            const auto& fixedControls = configJson["fixed_raw_controls"].Get<std::vector<std::string>>();
            m_fixedControls = std::set(fixedControls.begin(), fixedControls.end());
            const auto& neckControls = configJson["neck_raw_controls"].Get<std::vector<std::string>>();
            m_neckControls = std::set(neckControls.begin(), neckControls.end());
            if (configJson.Contains("default_max_modes")) {
                m_defaultMaxModes = configJson["default_max_modes"].Get<int>();
            }

            m_headMeshIndex = m_rig->GetRigGeometry()->GetMeshIndex(rt::HeadMeshName());
            m_teethMeshIndex = m_rig->GetRigGeometry()->GetMeshIndex(rt::TeethMeshName());
            m_eyeLeftMeshIndex = m_rig->GetRigGeometry()->GetMeshIndex(rt::EyeLeftMeshName());
            m_eyeRightMeshIndex = m_rig->GetRigGeometry()->GetMeshIndex(rt::EyeRightMeshName());

            m_meshIndices.clear();
            m_meshIndices = { m_headMeshIndex, m_teethMeshIndex, m_eyeLeftMeshIndex, m_eyeRightMeshIndex };

            {
                const int eyeLeftJointIndex = m_rig->GetRigGeometry()->GetJointRig().GetJointIndex(rt::EyeLeftJointName());
                const Eigen::Transform<float, 3, Eigen::Affine> eyeLeftBindPose(m_rig->GetRigGeometry()->GetBindMatrix(eyeLeftJointIndex));
                m_eyeLeftBody.baseTransform = eyeLeftBindPose;
                m_eyeLeftBody.baseVertices = eyeLeftBindPose.inverse() * m_rig->GetRigGeometry()->GetMesh(m_eyeLeftMeshIndex).Vertices();
            }
            {
                const int eyeRightJointIndex = m_rig->GetRigGeometry()->GetJointRig().GetJointIndex(rt::EyeRightJointName());
                const Eigen::Transform<float, 3, Eigen::Affine> eyeRightBindPose(m_rig->GetRigGeometry()->GetBindMatrix(eyeRightJointIndex));
                m_eyeRightBody.baseTransform = eyeRightBindPose;
                m_eyeRightBody.baseVertices = eyeRightBindPose.inverse() * m_rig->GetRigGeometry()->GetMesh(m_eyeRightMeshIndex).Vertices();
            }

            m_offsetsPerModel.clear();
            m_offsetsPerModel.push_back(0);
            m_offsetsPerModel.push_back(m_rig->GetRigGeometry()->GetMesh(m_headMeshIndex).NumVertices() + m_offsetsPerModel.back());
            m_offsetsPerModel.push_back(m_rig->GetRigGeometry()->GetMesh(m_teethMeshIndex).NumVertices() + m_offsetsPerModel.back());
            m_offsetsPerModel.push_back(m_rig->GetRigGeometry()->GetMesh(m_eyeLeftMeshIndex).NumVertices() + m_offsetsPerModel.back());
            m_offsetsPerModel.push_back(m_rig->GetRigGeometry()->GetMesh(m_eyeRightMeshIndex).NumVertices() + m_offsetsPerModel.back());
        }
        catch (std::exception &)
        {
            return false;
        }

        return true;
    }


    PCARigCreator::PCARigCreator(const std::string& dnaFilename, const std::string& configFilename) {
        // load the DNA
        std::shared_ptr<Rig<float>> rig = std::make_shared<Rig<float>>();
        if (!rig->LoadRig(dnaFilename, /*withJointScaling=*/false)) {
            CARBON_CRITICAL("failed to load riglogic from dnafile {}", dnaFilename);
        }
        m_rig = rig;

        if (!LoadConfigFile(configFilename)){
            CARBON_CRITICAL("failed to load PCA rig creator config file {}", configFilename);
        }
    }

    PCARigCreator::PCARigCreator(std::shared_ptr<const Rig<float>> rig){
        m_rig = rig;
    }


    bool PCARigCreator::Create(int maxModes) {

        Timer timer;

        maxModes = (maxModes >= 0 ? maxModes : m_defaultMaxModes);

        const std::vector<Eigen::VectorXf> rawControlsForAllExpressions = AllFaceExpressions();
        std::vector<rt::HeadVertexState<float>> headVertexStates;
        for (size_t i = 0; i < rawControlsForAllExpressions.size(); ++i) {
            headVertexStates.emplace_back(EvaluateExpression(rawControlsForAllExpressions[i]));
        }
        LOG_INFO("time to evalute expressions: {}", timer.Current()); timer.Restart();

        Eigen::Matrix<float, -1, -1> dataMatrix2(rawControlsForAllExpressions.size(), 0);
        for (size_t i = 0; i < headVertexStates.size(); ++i) {
            const rt::HeadState<float> headState = rt::HeadState<float>::FromHeadVertexState(headVertexStates[i], m_eyeLeftBody, m_eyeRightBody);
            if (i == 0) {
                dataMatrix2.resize(rawControlsForAllExpressions.size(), headState.Flatten().size());
            }
            dataMatrix2.row(i) = headState.Flatten().transpose();
        }

        const float varianceToKeep = 0.9999;

        Eigen::VectorX<float> mean2 = dataMatrix2.colwise().mean();
        mean2.segment(3 * m_offsetsPerModel[2], 6).setZero();
        mean2.segment(3 * m_offsetsPerModel[2] + 6, 6).setZero();
        const Eigen::Matrix<float, -1, -1> meanCenteredDataMatrix = dataMatrix2.rowwise() - mean2.transpose();
        const auto modes2 = CreatePCAWithMeanCenteredData<float>(meanCenteredDataMatrix, varianceToKeep, maxModes);

        // const auto [mean2, modes2] = CreatePCA<float>(dataMatrix2, varianceToKeep, maxModes);
        LOG_INFO("time to create PCA 2: {}", timer.Current()); timer.Restart();

        const int numFaceVertices = (m_offsetsPerModel[1] - m_offsetsPerModel[0]);
        const int numTeethVertices = (m_offsetsPerModel[2] - m_offsetsPerModel[1]);
        // const int numEyeLeftVertices = (m_offsetsPerModel[3] - m_offsetsPerModel[2]);
        // const int numEyeRightVertices = (m_offsetsPerModel[4] - m_offsetsPerModel[3]);

        const int facialRootIndex = m_rig->GetRigGeometry()->GetJointRig().GetJointIndex(rt::FacialRootJointName());
        const Eigen::Transform<float, 3, Eigen::Affine> defaultRootTransform(m_rig->GetRigGeometry()->GetBindMatrix(facialRootIndex));
        m_pcaRig.rootBindPose = defaultRootTransform;

        const auto faceMean2 = Eigen::Map<const Eigen::Matrix<float, 3, -1>>(mean2.segment(3 * m_offsetsPerModel[0], 3 * numFaceVertices).data(), 3, numFaceVertices);
        m_pcaRig.facePCA.Create(faceMean2, modes2.block(3 * m_offsetsPerModel[0], 0, 3 * numFaceVertices, modes2.cols()));
        const auto teethMean2 = Eigen::Map<const Eigen::Matrix<float, 3, -1>>(mean2.segment(3 * m_offsetsPerModel[1], 3 * numTeethVertices).data(), 3, numTeethVertices);
        m_pcaRig.teethPCA.Create(teethMean2, modes2.block(3 * m_offsetsPerModel[1], 0, 3 * numTeethVertices, modes2.cols()));
        m_pcaRig.eyeLeftTransformPCA.eyeBody = m_eyeLeftBody;
        m_pcaRig.eyeLeftTransformPCA.linearTransformModel.mean = mean2.segment(3 * m_offsetsPerModel[2], 6);
        m_pcaRig.eyeLeftTransformPCA.linearTransformModel.modes = modes2.block(3 * m_offsetsPerModel[2], 0, 6, modes2.cols());
        m_pcaRig.eyeRightTransformPCA.eyeBody = m_eyeRightBody;
        m_pcaRig.eyeRightTransformPCA.linearTransformModel.mean = mean2.segment(3 * m_offsetsPerModel[2] + 6, 6);
        m_pcaRig.eyeRightTransformPCA.linearTransformModel.modes = modes2.block(3 * m_offsetsPerModel[2] + 6, 0, 6, modes2.cols());
        m_pcaRig.meshes[0] = m_rig->GetRigGeometry()->GetMesh(m_headMeshIndex);
        m_pcaRig.meshes[1] = m_rig->GetRigGeometry()->GetMesh(m_teethMeshIndex);
        m_pcaRig.meshes[2] = m_rig->GetRigGeometry()->GetMesh(m_eyeLeftMeshIndex);
        m_pcaRig.meshes[3] = m_rig->GetRigGeometry()->GetMesh(m_eyeRightMeshIndex);

        // Eigen::Matrix<float, -1, -1> dataMatrix(rawControlsForAllExpressions.size(), 0);
        // for (size_t i = 0; i < headVertexStates.size(); ++i) {
        //     if (i == 0) {
        //         dataMatrix.resize(rawControlsForAllExpressions.size(), headVertexStates[i].Flatten().size());
        //     }
        //     dataMatrix.row(i) = headVertexStates[i].Flatten().transpose();
        // }

        // const auto [mean, modes] = CreatePCA<float>(dataMatrix, varianceToKeep, maxModes);
        // LOG_INFO("time to create PCA: {}", timer.Current()); timer.Restart();

        // m_pcaVertexRig.rootBindPose = defaultRootTransform;
        // const auto faceMean = Eigen::Map<const Eigen::Matrix<float, 3, -1>>(mean.segment(3 * m_offsetsPerModel[0], 3 * numFaceVertices).data(), 3, numFaceVertices);
        // m_pcaVertexRig.facePCA.Create(faceMean, modes.block(3 * m_offsetsPerModel[0], 0, 3 * numFaceVertices, modes.cols()));
        // const auto teethMean = Eigen::Map<const Eigen::Matrix<float, 3, -1>>(mean.segment(3 * m_offsetsPerModel[1], 3 * numTeethVertices).data(), 3, numTeethVertices);
        // m_pcaVertexRig.teethPCA.Create(teethMean, modes.block(3 * m_offsetsPerModel[1], 0, 3 * numTeethVertices, modes.cols()));
        // const auto eyeLeftMean = Eigen::Map<const Eigen::Matrix<float, 3, -1>>(mean.segment(3 * m_offsetsPerModel[2], 3 * numEyeLeftVertices).data(), 3, numEyeLeftVertices);
        // m_pcaVertexRig.eyeLeftPCA.Create(eyeLeftMean, modes.block(3 * m_offsetsPerModel[2], 0, 3 * numEyeLeftVertices, modes.cols()));
        // const auto eyeRightMean = Eigen::Map<const Eigen::Matrix<float, 3, -1>>(mean.segment(3 * m_offsetsPerModel[3], 3 * numEyeRightVertices).data(), 3, numEyeRightVertices);
        // m_pcaVertexRig.eyeRightPCA.Create(eyeRightMean, modes.block(3 * m_offsetsPerModel[3], 0, 3 * numEyeRightVertices, modes.cols()));
        // m_pcaVertexRig.meshes[0] = m_rig->GetRigGeometry()->GetMesh(m_headMeshIndex);
        // m_pcaVertexRig.meshes[1] = m_rig->GetRigGeometry()->GetMesh(m_teethMeshIndex);
        // m_pcaVertexRig.meshes[2] = m_rig->GetRigGeometry()->GetMesh(m_eyeLeftMeshIndex);
        // m_pcaVertexRig.meshes[3] = m_rig->GetRigGeometry()->GetMesh(m_eyeRightMeshIndex);
        // m_pcaVertexRig.solver = modes.colPivHouseholderQr();

#if 0
        // calculate neck PCA
        {
            const std::vector<Eigen::VectorXf> rawControlsForNeckExpressions = AllNeckExpressions();
            Eigen::Matrix<float, -1, -1> neckDataMatrix(rawControlsForNeckExpressions.size(), totalVertices * 3);
            const std::string facialRootName = "FACIAL_C_FacialRoot";
            const int facialRootIndex = m_rig->GetRigGeometry()->GetJointRig().GetJointIndex(facialRootName);
            if (facialRootIndex < 0) {
                CARBON_CRITICAL("no facial root index");
            }
            const Eigen::Matrix<float, 3, -1> defaultExpression = EvaluateExpression(DefaultRawControls());
            const Eigen::Matrix<float, 4, 4> defaultRootTransform = m_rig->GetRigGeometry()->GetWorldMatrix(facialRootIndex);

            for (size_t i = 0; i < rawControlsForNeckExpressions.size(); ++i) {
                const Eigen::Matrix<float, 3, -1> vertices = EvaluateExpression(rawControlsForNeckExpressions[i]);
                const Eigen::Matrix<float, 4, 4> rootTransform = m_rig->GetRigGeometry()->GetWorldMatrix(facialRootIndex);
                const Eigen::Matrix<float, 4, 4> toDefault = defaultRootTransform * rootTransform.inverse();
                Affine<float, 3, 3> rm(toDefault);
                const Eigen::Matrix<float, 3, -1> stabilizedVertices = rm.Transform(vertices);
                const Eigen::Matrix<float, 3, -1> deltaVertices = stabilizedVertices - defaultExpression;
                neckDataMatrix.row(i) = Eigen::Map<const Eigen::VectorXf>(deltaVertices.data(), deltaVertices.size());
            }

            const float neckVarianceToKeep = 0.995;
            const auto neckModes = CreatePCAWithMeanCenteredData<float>(neckDataMatrix, neckVarianceToKeep, maxModes);
            m_modes.conservativeResize(m_modes.rows(), m_modes.cols() + neckModes.cols());
            m_modes.rightCols(neckModes.cols()) = neckModes;
            LOG_INFO("time to create neck PCA: {}", timer.Current()); timer.Restart();
        }
#endif

        return true;
    }

    int PCARigCreator::RawControlIndex(const std::string& rawControlName) const {
        for (size_t i = 0; i < m_rig->GetRigLogic()->RawControlNames().size(); ++i) {
            if (m_rig->GetRigLogic()->RawControlNames()[i] == rawControlName) {
                return int(i);
            }
        }
        return -1;
    }

    Eigen::VectorXf PCARigCreator::DefaultRawControls() const {
        const int numRawControls = m_rig->GetRigLogic()->NumRawControls();

        Eigen::VectorXf defaultRawControls = Eigen::VectorXf::Zero(numRawControls);
        for (const auto& [defaultRawControlName, value] : m_rawControlDefaults) {
            defaultRawControls[RawControlIndex(defaultRawControlName)] = value;
        }

        return defaultRawControls;
    }

    std::vector<Eigen::VectorXf> PCARigCreator::AllFaceExpressions() const {
        std::vector<Eigen::VectorXf> allExpressions;
        allExpressions.reserve(1000);

        // default expression
        const Eigen::VectorXf defaultRawControls = DefaultRawControls();
        allExpressions.emplace_back(defaultRawControls);

        // evaluate individual raw controls (that have not been fixed)
        const int numRawControls = m_rig->GetRigLogic()->NumRawControls();
        for (int k = 0; k < numRawControls; ++k) {
            const std::string& rawControlName = m_rig->GetRigLogic()->RawControlNames()[k];
            if (m_fixedControls.count(rawControlName) > 0) continue;
            Eigen::VectorXf rawControls = defaultRawControls;
            rawControls[k] = 1.0f;
            allExpressions.emplace_back(rawControls);
        }

        // evaluate corrective controls that are not based on fixed controls
        const SparseMatrix<float> psdToRawMap = m_rig->GetRigLogic()->PsdToRawMap();
        for (int k = 0; k < int(psdToRawMap.outerSize()); ++k) {
            Eigen::VectorXf rawControls = defaultRawControls;
            bool isFixed = false;
            for (typename SparseMatrix<float>::InnerIterator it(psdToRawMap, k); it; ++it)
            {
                rawControls[it.col()] = float(1) / it.value();
                isFixed |= (m_fixedControls.count(m_rig->GetRigLogic()->RawControlNames()[it.col()]) > 0);
            }
            if (isFixed) continue;
            allExpressions.emplace_back(rawControls);
        }

        return allExpressions;
    }

    std::vector<Eigen::VectorXf> PCARigCreator::AllNeckExpressions() const {
        std::vector<Eigen::VectorXf> neckExpressions;
        neckExpressions.reserve(1000);

        // default expression
        const Eigen::VectorXf defaultRawControls = DefaultRawControls();

        // evaluate neck controls
        for (const std::string& neckControlName : m_neckControls) {
            Eigen::VectorXf rawControls = defaultRawControls;
            rawControls[RawControlIndex(neckControlName)] = 1.0f;
            neckExpressions.emplace_back(rawControls);
        }

        return neckExpressions;
    }

    rt::HeadVertexState<float> PCARigCreator::EvaluateExpression(const Eigen::VectorXf& rawControls) const {
        rt::HeadVertexState<float> headVertexState;
        const DiffData<float> psdValues = m_rig->GetRigLogic()->EvaluatePSD(DiffData<float>(rawControls));
        const int lod = 0;
        const DiffData<float> diffJoints = m_rig->GetRigLogic()->EvaluateJoints(psdValues, lod);
        RigGeometry<float>::State rigState;
        m_rig->GetRigGeometry()->EvaluateRigGeometry(DiffDataAffine<float, 3, 3>(), diffJoints, psdValues, lod, {m_headMeshIndex, m_teethMeshIndex, m_eyeLeftMeshIndex, m_eyeRightMeshIndex}, rigState);
        headVertexState.faceVertices = rigState.Vertices()[0].Matrix();
        headVertexState.teethVertices = rigState.Vertices()[1].Matrix();
        headVertexState.eyeLeftVertices = rigState.Vertices()[2].Matrix();
        headVertexState.eyeRightVertices = rigState.Vertices()[3].Matrix();

        // get rid of any neck motion that changes the facial root
        const int facialRootIndex = m_rig->GetRigGeometry()->GetJointRig().GetJointIndex(rt::FacialRootJointName());
        if (facialRootIndex < 0) {
            CARBON_CRITICAL("no facial root index");
        }
        const Eigen::Matrix<float, 4, 4> defaultRootTransform = m_rig->GetRigGeometry()->GetBindMatrix(facialRootIndex);
        const Eigen::Matrix<float, 4, 4> rootTransform = rigState.GetWorldMatrix(facialRootIndex);
        const Eigen::Matrix<float, 4, 4> toDefault = defaultRootTransform * rootTransform.inverse();
        const Eigen::Transform<float, 3, Eigen::Affine> rm(toDefault);

        headVertexState.faceVertices = rm * headVertexState.faceVertices;
        headVertexState.teethVertices = rm * headVertexState.teethVertices;
        headVertexState.eyeLeftVertices = rm * headVertexState.eyeLeftVertices;
        headVertexState.eyeRightVertices = rm * headVertexState.eyeRightVertices;

        return headVertexState;
    }

}