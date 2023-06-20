// Copyright Epic Games, Inc. All Rights Reserved.

#include <tracking/rt/PCARig.h>
#include <nls/geometry/EulerAngles.h>
#include <nls/rig/Rig.h>
#include <nls/rig/RigLogicDNAResource.h>
#include <nls/serialization/NpyFileFormat.h>

#include <riglogic/RigLogic.h>

#include <array>
#include <string>

namespace epic::nls::rt {

const char* EyeLeftJointName() { return "FACIAL_L_Eye"; }
const char* EyeRightJointName() { return "FACIAL_R_Eye"; }
const char* FacialRootJointName() { return "FACIAL_C_FacialRoot"; }

const char* HeadMeshName() { return "head_lod0_mesh"; }
const char* TeethMeshName() { return "teeth_lod0_mesh"; }
const char* EyeLeftMeshName() { return "eyeLeft_lod0_mesh"; }
const char* EyeRightMeshName() { return "eyeRight_lod0_mesh"; }


HeadVertexState<float> PCARig::EvaluatePCARig(const Eigen::VectorX<float>& pcaCoeffs) const
{
    HeadVertexState<float> headVertexState;
    headVertexState.faceVertices = facePCA.EvaluateLinearized(pcaCoeffs, LinearVertexModel<float>::EvaluationMode::STATIC);
    headVertexState.teethVertices = teethPCA.EvaluateLinearized(pcaCoeffs, LinearVertexModel<float>::EvaluationMode::STATIC);
    headVertexState.eyeLeftVertices = eyeLeftTransformPCA.EvaluateVertices(pcaCoeffs, LinearVertexModel<float>::EvaluationMode::STATIC);
    headVertexState.eyeRightVertices = eyeRightTransformPCA.EvaluateVertices(pcaCoeffs, LinearVertexModel<float>::EvaluationMode::STATIC);
    return headVertexState;
}


Eigen::VectorX<float> PCARig::Project(const HeadVertexState<float>& headVertexState, const Eigen::VectorX<float>& coeffs) const
{
    const int numCoeffs = int(coeffs.size());
    Eigen::Matrix<float, -1, -1> AtA = Eigen::Matrix<float, -1, -1>::Zero(numCoeffs, numCoeffs);
    Eigen::VectorX<float> Atb = Eigen::VectorX<float>::Zero(numCoeffs);

    const auto evaluationMode = LinearVertexModel<float>::EvaluationMode::STATIC;

    {
        AtA.noalias() += facePCA.Modes(evaluationMode).transpose() * facePCA.Modes(evaluationMode);
        const auto verticesVec = Eigen::Map<const Eigen::VectorXf>(headVertexState.faceVertices.data(), headVertexState.faceVertices.size());
        Atb.noalias() += - facePCA.Modes(evaluationMode).transpose() * (facePCA.BaseAsVector() + facePCA.Modes(evaluationMode) * coeffs - verticesVec);
    }
    {
        AtA.noalias() += teethPCA.Modes(evaluationMode).transpose() * teethPCA.Modes(evaluationMode);
        const auto verticesVec = Eigen::Map<const Eigen::VectorXf>(headVertexState.teethVertices.data(), headVertexState.teethVertices.size());
        Atb.noalias() += - teethPCA.Modes(evaluationMode).transpose() * (teethPCA.BaseAsVector() + teethPCA.Modes(evaluationMode) * coeffs - verticesVec);
    }
    {
        const auto [eyeLeftVertices, eyeLeftJacobian] = eyeLeftTransformPCA.EvaluateVerticesAndJacobian(coeffs, evaluationMode);
        const auto eyeLeftVerticesVec = Eigen::Map<const Eigen::VectorXf>(eyeLeftVertices.data(), eyeLeftVertices.size());
        const auto targetVerticesVec = Eigen::Map<const Eigen::VectorXf>(headVertexState.eyeLeftVertices.data(), headVertexState.eyeLeftVertices.size());
        AtA.noalias() += eyeLeftJacobian.transpose() * eyeLeftJacobian;
        Atb.noalias() += - eyeLeftJacobian.transpose() * (eyeLeftVerticesVec - targetVerticesVec);
    }
    {
        const auto [eyeRightVertices, eyeRightJacobian] = eyeRightTransformPCA.EvaluateVerticesAndJacobian(coeffs, evaluationMode);
        const auto eyeRightVerticesVec = Eigen::Map<const Eigen::VectorXf>(eyeRightVertices.data(), eyeRightVertices.size());
        const auto targetVerticesVec = Eigen::Map<const Eigen::VectorXf>(headVertexState.eyeRightVertices.data(), headVertexState.eyeRightVertices.size());
        AtA.noalias() += eyeRightJacobian.transpose() * eyeRightJacobian;
        Atb.noalias() += - eyeRightJacobian.transpose() * (eyeRightVerticesVec - targetVerticesVec);
    }

    return coeffs + AtA.ldlt().solve(Atb);
}


void PCARig::Translate(const Eigen::Vector3f& translation)
{
    facePCA.Translate(translation);
    teethPCA.Translate(translation);
    eyeLeftTransformPCA.Translate(translation);
    eyeRightTransformPCA.Translate(translation);
    rootBindPose = Eigen::Translation3f(translation) * rootBindPose;
}

Eigen::Vector3f PCARig::EyesMidpoint() const
{
    const auto evaluationMode = LinearVertexModel<float>::EvaluationMode::STATIC;

    const Eigen::Vector3f eyeLeftCenter = eyeLeftTransformPCA.EvaluateVertices(Eigen::VectorXf::Zero(facePCA.NumPCAModes()), evaluationMode).rowwise().mean();
    const Eigen::Vector3f eyeRightCenter = eyeRightTransformPCA.EvaluateVertices(Eigen::VectorXf::Zero(facePCA.NumPCAModes()), evaluationMode).rowwise().mean();
    return 0.5f * (eyeLeftCenter + eyeRightCenter);
}

void PCARig::SaveAsDNA(dna::StreamWriter* writer) const
{
    const std::vector<std::string> allMeshNames = { HeadMeshName(), TeethMeshName(), EyeLeftMeshName(), EyeRightMeshName() };
    const std::vector<std::string> bsMeshNames = { HeadMeshName(), TeethMeshName() };
    const uint16_t numPCAcoeffs = uint16_t(NumCoeffs());
    constexpr float rad2deg = float(180.0 / CARBON_PI);

    // we save the meshes with the mean
    HeadVertexState<float> meanHeadVertexState = EvaluatePCARig(Eigen::VectorXf::Zero(NumCoeffs()));
    std::vector<Eigen::Matrix<float, 3, -1>> allMeanVertices;
    allMeanVertices.push_back(meanHeadVertexState.faceVertices);
    allMeanVertices.push_back(meanHeadVertexState.teethVertices);
    allMeanVertices.push_back(meanHeadVertexState.eyeLeftVertices);
    allMeanVertices.push_back(meanHeadVertexState.eyeRightVertices);

    // DescriptorWriter methods
    writer->setName("pca_model");
    writer->setLODCount(1);

    // definition
    writer->clearGUIControlNames();
    writer->clearRawControlNames();

    for (uint16_t bsIndex = 0; bsIndex < numPCAcoeffs; ++bsIndex) {
        writer->setGUIControlName(bsIndex, (std::string("gui_pca_") + std::to_string(bsIndex)).c_str());
        writer->setRawControlName(bsIndex, (std::string("raw_pca_") + std::to_string(bsIndex)).c_str());
    }

    writer->clearMeshNames();
    std::vector<std::uint16_t> meshIndices;
    for (size_t i = 0; i < allMeshNames.size(); ++i) {
        const uint16_t meshIndex = static_cast<uint16_t>(i);
        writer->setMeshName(meshIndex, allMeshNames[i].c_str());
        meshIndices.push_back(meshIndex);
    }
    writer->setMeshIndices(0, meshIndices.data(), std::uint16_t(meshIndices.size()));
    writer->setLODMeshMapping(0, 0);

    writer->clearBlendShapeChannelNames();
    writer->clearMeshBlendShapeChannelMappings();
    std::vector<uint16_t> controlInputIndices;
    std::vector<uint16_t> blendshapeOutputIndices;
    for (size_t meshIndex = 0; meshIndex < bsMeshNames.size(); ++meshIndex) {
        for (uint16_t bsIndex = 0; bsIndex < numPCAcoeffs; ++bsIndex) {
            const uint16_t meshBsIndex = uint16_t(numPCAcoeffs * meshIndex + bsIndex);
            writer->setBlendShapeChannelName(meshBsIndex, (bsMeshNames[meshIndex] + std::string("_bs_pca_") + std::to_string(bsIndex)).c_str());
            writer->setMeshBlendShapeChannelMapping(0, uint16_t(meshIndex), meshBsIndex);
            controlInputIndices.push_back(bsIndex);
            blendshapeOutputIndices.push_back(meshBsIndex);
        }
    }
    writer->setBlendShapeChannelIndices(0, blendshapeOutputIndices.data(), uint16_t(blendshapeOutputIndices.size()));
    writer->clearLODBlendShapeChannelMappings();
    writer->setLODBlendShapeChannelMapping(0, 0);

    writer->clearJointNames();
    writer->setJointName(0, FacialRootJointName());
    writer->setJointName(1, EyeLeftJointName());
    writer->setJointName(2, EyeRightJointName());
    std::array<std::uint16_t, 3> jointIndices{ 0, 1, 2 };
    writer->clearJointIndices();
    writer->setJointIndices(0, jointIndices.data(), uint16_t(jointIndices.size()));
    writer->clearLODJointMappings();
    writer->setLODJointMapping(0, 0);
    std::array<std::uint16_t, 3> jointHierarchy{ 0, 0, 0 };
    writer->setJointHierarchy(jointHierarchy.data(), uint16_t(jointHierarchy.size()));

    Eigen::Matrix<float, 3, -1> jointTranslations(3, 3);
    Eigen::Matrix<float, 3, -1> jointRotations(3, 3);
    {
        const Eigen::Transform<float, 3, Eigen::Affine> eyeLeftTransform = rootBindPose.inverse() * eyeLeftTransformPCA.eyeBody.baseTransform;
        const Eigen::Transform<float, 3, Eigen::Affine> eyeRightTransform = rootBindPose.inverse() * eyeRightTransformPCA.eyeBody.baseTransform;
        jointRotations.col(0) = (RotationMatrixToEulerXYZ<float>(rootBindPose.linear()) * rad2deg).template cast<float>();
        jointTranslations.col(0) = rootBindPose.translation();
        jointRotations.col(1) = (RotationMatrixToEulerXYZ<float>(eyeLeftTransform.linear()) * rad2deg).template cast<float>();
        jointTranslations.col(1) = eyeLeftTransform.translation();
        jointRotations.col(2) = (RotationMatrixToEulerXYZ<float>(eyeRightTransform.linear()) * rad2deg).template cast<float>();
        jointTranslations.col(2) = eyeRightTransform.translation();
    }
    writer->setNeutralJointTranslations((const dna::Vector3*)jointTranslations.data(), 3);
    writer->setNeutralJointRotations((const dna::Vector3*)jointRotations.data(), 3);

    // BehaviorWriter methods
    {
        const Eigen::VectorX<uint16_t> inputIndices = Eigen::VectorX<uint16_t>::LinSpaced(numPCAcoeffs, 0, numPCAcoeffs - 1);
        const Eigen::VectorX<uint16_t> outputIndices = Eigen::VectorX<uint16_t>::LinSpaced(numPCAcoeffs, 0, numPCAcoeffs - 1);
        const Eigen::VectorX<float> fromValues = Eigen::VectorX<float>::Constant(numPCAcoeffs, -3);
        const Eigen::VectorX<float> toValues = Eigen::VectorX<float>::Constant(numPCAcoeffs, 3);
        const Eigen::VectorX<float> slopeValues = Eigen::VectorX<float>::Constant(numPCAcoeffs, 1);
        const Eigen::VectorX<float> cutValues = Eigen::VectorX<float>::Zero(numPCAcoeffs);
        writer->setGUIToRawInputIndices(inputIndices.data(), numPCAcoeffs);
        writer->setGUIToRawOutputIndices(outputIndices.data(), numPCAcoeffs);
        writer->setGUIToRawFromValues(fromValues.data(), numPCAcoeffs);
        writer->setGUIToRawToValues(toValues.data(), numPCAcoeffs);
        writer->setGUIToRawSlopeValues(slopeValues.data(), numPCAcoeffs);
        writer->setGUIToRawCutValues(cutValues.data(), numPCAcoeffs);
    }

    {
        const Eigen::VectorX<uint16_t> inputIndices = Eigen::VectorX<uint16_t>::LinSpaced(numPCAcoeffs, 0, numPCAcoeffs - 1);
        std::uint16_t rows = 12;
        Eigen::VectorX<uint16_t> outputIndices(12);
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 6; ++j) {
                outputIndices[6 * i + j] = uint16_t(9 + 9 * i + j);
            }
        }
        writer->setJointRowCount(3 * 9);
        writer->setJointColumnCount(numPCAcoeffs);
        writer->clearJointGroups();
        writer->setJointGroupLODs(0, &rows, 1);
        writer->setJointGroupInputIndices(0, inputIndices.data(), uint16_t(inputIndices.size()));
        writer->setJointGroupOutputIndices(0, outputIndices.data(), uint16_t(outputIndices.size()));
        Eigen::Matrix<float, -1, -1, Eigen::RowMajor> rotationsAndTranslations(rows, numPCAcoeffs);
        rotationsAndTranslations.block(0, 0, 3, numPCAcoeffs) = eyeLeftTransformPCA.linearTransformModel.modes.block(3, 0, 3, numPCAcoeffs);
        rotationsAndTranslations.block(3, 0, 3, numPCAcoeffs) = eyeLeftTransformPCA.linearTransformModel.modes.block(0, 0, 3, numPCAcoeffs) * rad2deg;
        rotationsAndTranslations.block(6, 0, 3, numPCAcoeffs) = eyeRightTransformPCA.linearTransformModel.modes.block(3, 0, 3, numPCAcoeffs);
        rotationsAndTranslations.block(9, 0, 3, numPCAcoeffs) = eyeRightTransformPCA.linearTransformModel.modes.block(0, 0, 3, numPCAcoeffs) * rad2deg;
        writer->setJointGroupValues(0, rotationsAndTranslations.data(), uint16_t(rotationsAndTranslations.size()));
    }

    {
        const uint16_t blendshapeChannelLODs = uint16_t(bsMeshNames.size() * numPCAcoeffs);
        writer->setBlendShapeChannelLODs(&blendshapeChannelLODs, 1);
        writer->setBlendShapeChannelInputIndices(controlInputIndices.data(), uint16_t(controlInputIndices.size()));
        writer->setBlendShapeChannelOutputIndices(blendshapeOutputIndices.data(), uint16_t(blendshapeOutputIndices.size()));
    }

    // GeometryWriter methods
    writer->clearMeshes();
    for (uint16_t meshIndex = 0; meshIndex < uint16_t(allMeshNames.size()); ++meshIndex) {
        writer->setVertexPositions(meshIndex, (const dna::Position*)allMeanVertices[meshIndex].data(), uint32_t(allMeanVertices[meshIndex].cols()));
        const Mesh<float>& quadMesh = meshes[meshIndex];
        Mesh<float> triMesh = quadMesh;
        triMesh.Triangulate();
        triMesh.SetVertices(allMeanVertices[meshIndex]);
        triMesh.CalculateVertexNormals();
        writer->setVertexNormals(meshIndex, (const dna::Normal*)triMesh.VertexNormals().data(), uint32_t(triMesh.NumVertices()));
        Eigen::Matrix<float, 2, -1> texcoords = quadMesh.Texcoords();
        for (int k = 0; k < texcoords.cols(); ++k) {
            texcoords(1, k) = 1.0f - texcoords(1, k);
        }
        writer->setVertexTextureCoordinates(meshIndex, (const dna::TextureCoordinate*)texcoords.data(), uint32_t(texcoords.cols()));
        {
            int totalFaces = quadMesh.NumQuads() + quadMesh.NumTriangles();
            writer->setFaceVertexLayoutIndices(uint16_t(meshIndex), totalFaces - 1, nullptr, 0);

            int faceCount = 0;
            std::vector<dna::VertexLayout> vertexLayouts;
            for (int quadIndex = 0; quadIndex < quadMesh.NumQuads(); ++quadIndex)
            {
                std::vector<uint32_t> layoutIndices;
                for (int k = 0; k < 4; ++k) {
                    dna::VertexLayout vertexLayout;
                    vertexLayout.position = quadMesh.Quads()(k, quadIndex);
                    vertexLayout.normal = quadMesh.Quads()(k, quadIndex);
                    vertexLayout.textureCoordinate = quadMesh.TexQuads()(k, quadIndex);
                    layoutIndices.push_back(uint32_t(vertexLayouts.size()));
                    vertexLayouts.push_back(vertexLayout);
                }
                writer->setFaceVertexLayoutIndices(meshIndex, faceCount, layoutIndices.data(), uint32_t(layoutIndices.size()));
                faceCount++;
            }
            for (int triIndex = 0; triIndex < quadMesh.NumTriangles(); ++triIndex)
            {
                std::vector<uint32_t> layoutIndices;
                for (int k = 0; k < 3; ++k) {
                    dna::VertexLayout vertexLayout;
                    vertexLayout.position = quadMesh.Triangles()(k, triIndex);
                    vertexLayout.normal = quadMesh.Triangles()(k, triIndex);
                    vertexLayout.textureCoordinate = quadMesh.TexTriangles()(k, triIndex);
                    layoutIndices.push_back(uint32_t(vertexLayouts.size()));
                    vertexLayouts.push_back(vertexLayout);
                }
                writer->setFaceVertexLayoutIndices(meshIndex, faceCount, layoutIndices.data(), uint32_t(layoutIndices.size()));
                faceCount++;
            }
            writer->setVertexLayouts(meshIndex, vertexLayouts.data(), uint32_t(vertexLayouts.size()));
        }
    }

    {
        for (uint16_t meshIndex = 0; meshIndex < 4; ++meshIndex) {
            writer->setMaximumInfluencePerVertex(meshIndex, 1);
            writer->clearSkinWeights(meshIndex);
            const int numVertices = meshes[meshIndex].NumVertices();
            const float weight = 1.0f;
            const std::uint16_t jointIndex = std::uint16_t(std::max(0, (meshIndex - 1)));
            for (int vID = numVertices - 1; vID >= 0; --vID) {
                writer->setSkinWeightsValues(meshIndex, vID, &weight, 1);
                writer->setSkinWeightsJointIndices(meshIndex, vID, &jointIndex, 1);
            }
        }
    }

    for (uint16_t meshIndex = 0; meshIndex < uint16_t(bsMeshNames.size()); ++meshIndex) {
        writer->clearBlendShapeTargets(meshIndex);
        const LinearVertexModel<float>& linearModel = (meshIndex == 0) ? facePCA : teethPCA;
        const int numVertices = linearModel.NumVertices();
        Eigen::VectorX<std::uint32_t> vertexIndices = Eigen::VectorX<std::uint32_t>::LinSpaced(numVertices, 0, numVertices - 1);
        for (uint16_t bsIndex = 0; bsIndex < numPCAcoeffs; ++bsIndex) {
            const uint16_t meshBsIndex = numPCAcoeffs * meshIndex + bsIndex;
            writer->setBlendShapeChannelIndex(meshIndex, bsIndex, meshBsIndex);
            const Eigen::VectorXf mode = linearModel.Modes(LinearVertexModel<float>::EvaluationMode::STATIC).col(bsIndex);
            writer->setBlendShapeTargetDeltas(meshIndex, bsIndex, (const dna::Delta*)mode.data(), numVertices);
            writer->setBlendShapeTargetVertexIndices(meshIndex, bsIndex, vertexIndices.data(), numVertices);
        }
    }

    writer->write();

}

void PCARig::SaveAsDNA(const std::string& filename) const
{

    pma::ScopedPtr<dna::FileStream> stream = pma::makeScoped<dna::FileStream>(filename.c_str(),
        dna::FileStream::AccessMode::Write,
        dna::FileStream::OpenMode::Binary);
    pma::ScopedPtr<dna::StreamWriter> writer = pma::makeScoped<dna::StreamWriter>(stream.get());

    SaveAsDNA(writer.operator->());


    // verify that we get the same reconstruction using the rig
    Rig<float> testDnaRig;
    if (!testDnaRig.LoadRig(filename)) {
        LOG_ERROR("failed to read dna rig");
    }
    PCARig testPcaRig;
    if (!testPcaRig.LoadFromDNA(filename)) {
        LOG_ERROR("failed to read pca rig from dna");
    }

    const uint16_t numPCAcoeffs = uint16_t(NumCoeffs());

    for (int k = 0; k < numPCAcoeffs; ++k) {
        Eigen::VectorXf pcaCoeffs = Eigen::VectorXf::Zero(numPCAcoeffs);
        pcaCoeffs[k] = 1;
        const HeadVertexState<float> headVertexState = EvaluatePCARig(pcaCoeffs);

        {
            const std::vector<Eigen::Matrix<float, 3, -1>> rigVertices = testDnaRig.EvaluateVertices(pcaCoeffs, 0, {0, 1, 2, 3});
            const float diffHead = (rigVertices[0] - headVertexState.faceVertices).cwiseAbs().maxCoeff();
            const float diffTeeth = (rigVertices[1] - headVertexState.teethVertices).cwiseAbs().maxCoeff();
            const float diffEyeLeft = (rigVertices[2] - headVertexState.eyeLeftVertices).cwiseAbs().maxCoeff();
            const float diffEyeRight = (rigVertices[3] - headVertexState.eyeRightVertices).cwiseAbs().maxCoeff();
            if (diffHead > 1e-4) {
                LOG_ERROR("{}: rig face error: {}", k, diffHead);
            }
            if (diffTeeth > 1e-4) {
                LOG_ERROR("{}: rig teeth error: {}", k, diffTeeth);
            }
            if (diffEyeLeft > 1e-4) {
                LOG_ERROR("{}: rig eye left error: {}", k, diffEyeLeft);
            }
            if (diffEyeRight > 1e-4) {
                LOG_ERROR("{}: rig eye right error: {}", k, diffEyeRight);
            }
        }
        {
            const HeadVertexState<float> headVertexStatePCA = testPcaRig.EvaluatePCARig(pcaCoeffs);
            const float diffHead = (headVertexStatePCA.faceVertices - headVertexState.faceVertices).cwiseAbs().maxCoeff();
            const float diffTeeth = (headVertexStatePCA.teethVertices - headVertexState.teethVertices).cwiseAbs().maxCoeff();
            const float diffEyeLeft = (headVertexStatePCA.eyeLeftVertices - headVertexState.eyeLeftVertices).cwiseAbs().maxCoeff();
            const float diffEyeRight = (headVertexStatePCA.eyeRightVertices - headVertexState.eyeRightVertices).cwiseAbs().maxCoeff();
            if (diffHead > 1e-4) {
                LOG_ERROR("{}: pca rig face error: {}", k, diffHead);
            }
            if (diffTeeth > 1e-4) {
                LOG_ERROR("{}: pca rig teeth error: {}", k, diffTeeth);
            }
            if (diffEyeLeft > 1e-4) {
                LOG_ERROR("{}: pca rig eye left error: {}", k, diffEyeLeft);
            }
            if (diffEyeRight > 1e-4) {
                LOG_ERROR("{}: pca rig eye right error: {}", k, diffEyeRight);
            }
        }
    }
}


bool PCARig::LoadFromDNA(const std::string& filename)
{
    std::shared_ptr<const RigLogicDNAResource> dnaResource = RigLogicDNAResource::LoadDNA(filename, /*retain=*/false);
    if (!dnaResource) {
        LOG_ERROR("failed to open dnafile {}", filename);
        return false;
    }

    return LoadFromDNA(dnaResource->Stream());
}



bool PCARig::LoadFromDNA(dna::StreamReader* reader)
{
    if (reader->getLODCount() != 1) {
        LOG_ERROR("dna does not seem to be a PCA rig");
        return false;
    }

    int numPCACoeffs = reader->getGUIControlCount();
    if (reader->getRawControlCount() != numPCACoeffs || reader->getPSDCount() != 0) {
        LOG_ERROR("dna does not seem to be a PCA rig");
        return false;
    }

    if (reader->getBlendShapeChannelLODs()[0] != numPCACoeffs * 2
       || reader->getMeshCount() != 4
       || reader->getBlendShapeTargetCount(0) != numPCACoeffs
       || reader->getBlendShapeTargetCount(1) != numPCACoeffs) {
        LOG_ERROR("number of PCA blendshape channels incorrect");
        return false;
    }

    if (reader->getJointCount() != 3
        || reader->getJointCount() * 9 != reader->getJointRowCount()
        || reader->getJointColumnCount() != numPCACoeffs
        || reader->getJointGroupCount() != 1
        || int(reader->getJointGroupInputIndices(0).size()) != numPCACoeffs
        || reader->getJointGroupOutputIndices(0).size() != 12) {
        LOG_ERROR("unexpected number of joints in pca rig");
        return false;
    }

    // load meshes
    meshes[0] = RigGeometry<float>::ReadMesh(reader, 0); // face
    meshes[1] = RigGeometry<float>::ReadMesh(reader, 1); // teeth
    meshes[2] = RigGeometry<float>::ReadMesh(reader, 2); // left eye
    meshes[3] = RigGeometry<float>::ReadMesh(reader, 3); // right eye

    std::vector<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> modes(2);
    modes[0].resize(meshes[0].NumVertices() * 3, numPCACoeffs);
    modes[1].resize(meshes[1].NumVertices() * 3, numPCACoeffs);

    // read blendshape data
    for (std::uint16_t meshIndex = 0; meshIndex < 2; ++meshIndex) {
        // auto& modes = (meshIndex == 0) ? facePCA.Modes(/*withRigid*/false) : teethPCA.Modes(/*withRigid*/false);
        const int numVertices = meshes[meshIndex].NumVertices();
        for (std::uint16_t blendShapeTargetIndex = 0; blendShapeTargetIndex < numPCACoeffs; blendShapeTargetIndex++) {
            // const std::uint16_t channelIndex = reader->getBlendShapeChannelIndex(meshIndex, blendShapeTargetIndex);
            const int numDeltas = reader->getBlendShapeTargetDeltaCount(meshIndex, blendShapeTargetIndex);
            if (numDeltas != numVertices) {
                LOG_ERROR("invalid number of vertices for blendshape");
                return false;
            }
            rl4::ConstArrayView<std::uint32_t> vertexIndices = reader->getBlendShapeTargetVertexIndices(meshIndex, blendShapeTargetIndex);
            for (int deltaIndex = 0; deltaIndex < numDeltas; deltaIndex++) {
                if (deltaIndex != int(vertexIndices[deltaIndex])) {
                    LOG_ERROR("invalid blendshape delta vertex index");
                    return false;
                }
                const dna::Delta delta = reader->getBlendShapeTargetDelta(meshIndex, blendShapeTargetIndex, deltaIndex);
                modes[meshIndex](3 * vertexIndices[deltaIndex] + 0, blendShapeTargetIndex) = delta.x;
                modes[meshIndex](3 * vertexIndices[deltaIndex] + 1, blendShapeTargetIndex) = delta.y;
                modes[meshIndex](3 * vertexIndices[deltaIndex] + 2, blendShapeTargetIndex) = delta.z;
            }
        }
    }

    facePCA.Create(meshes[0].Vertices(), modes[0]);
    teethPCA.Create(meshes[1].Vertices(), modes[1]);

    // load eye transforms
    {
        constexpr float deg2rad = float(CARBON_PI / 180.0);

        Eigen::Transform<float, 3, Eigen::Affine> rootTransform = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
        Eigen::Transform<float, 3, Eigen::Affine> eyeLeftLocalTransform = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
        Eigen::Transform<float, 3, Eigen::Affine> eyeRightLocalTransform = Eigen::Transform<float, 3, Eigen::Affine>::Identity();

        const dna::Vector3 t0 = reader->getNeutralJointTranslation(0);
        const dna::Vector3 rot0 = reader->getNeutralJointRotation(0) * deg2rad;
        rootTransform.linear() = EulerXYZ<float>(rot0.x, rot0.y, rot0.z);
        rootTransform.translation() = Eigen::Vector3f(t0.x, t0.y, t0.z);

        const dna::Vector3 t1 = reader->getNeutralJointTranslation(1);
        const dna::Vector3 rot1 = reader->getNeutralJointRotation(1) * deg2rad;
        eyeLeftLocalTransform.linear() = EulerXYZ<float>(rot1.x, rot1.y, rot1.z);
        eyeLeftLocalTransform.translation() = Eigen::Vector3f(t1.x, t1.y, t1.z);

        const dna::Vector3 t2 = reader->getNeutralJointTranslation(2);
        const dna::Vector3 rot2 = reader->getNeutralJointRotation(2) * deg2rad;
        eyeRightLocalTransform.linear() = EulerXYZ<float>(rot2.x, rot2.y, rot2.z);
        eyeRightLocalTransform.translation() = Eigen::Vector3f(t2.x, t2.y, t2.z);

        rootBindPose = rootTransform;
        eyeLeftTransformPCA.eyeBody.baseTransform = rootTransform * eyeLeftLocalTransform;
        eyeRightTransformPCA.eyeBody.baseTransform = rootTransform * eyeRightLocalTransform;

        eyeLeftTransformPCA.eyeBody.baseVertices = eyeLeftTransformPCA.eyeBody.baseTransform.inverse() * meshes[2].Vertices();
        eyeRightTransformPCA.eyeBody.baseVertices = eyeRightTransformPCA.eyeBody.baseTransform.inverse() * meshes[3].Vertices();

        // we expect the eye joint info to be stored as [translations, rotations]
        auto rotationsAndTranslations = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(reader->getJointGroupValues(0).data(), 12, numPCACoeffs);
        eyeLeftTransformPCA.linearTransformModel.mean.setZero(6);
        eyeLeftTransformPCA.linearTransformModel.modes.resize(6, numPCACoeffs);
        eyeLeftTransformPCA.linearTransformModel.modes.block(0, 0, 3, numPCACoeffs) = rotationsAndTranslations.block(3, 0, 3, numPCACoeffs) * deg2rad;
        eyeLeftTransformPCA.linearTransformModel.modes.block(3, 0, 3, numPCACoeffs) = rotationsAndTranslations.block(0, 0, 3, numPCACoeffs);
        eyeRightTransformPCA.linearTransformModel.mean.setZero(6);
        eyeRightTransformPCA.linearTransformModel.modes.resize(6, numPCACoeffs);
        eyeRightTransformPCA.linearTransformModel.modes.block(0, 0, 3, numPCACoeffs) = rotationsAndTranslations.block(9, 0, 3, numPCACoeffs) * deg2rad;
        eyeRightTransformPCA.linearTransformModel.modes.block(3, 0, 3, numPCACoeffs) = rotationsAndTranslations.block(6, 0, 3, numPCACoeffs);
    }

    return true;
}

void PCARig::SaveAsNpy(const std::string& filename) const
{
    std::vector<int> offsets;
    offsets.push_back(0);
    offsets.push_back(offsets.back() + facePCA.NumVertices() * 3);
    offsets.push_back(offsets.back() + teethPCA.NumVertices() * 3);
    offsets.push_back(offsets.back() + int(eyeLeftTransformPCA.linearTransformModel.modes.rows()));
    offsets.push_back(offsets.back() + int(eyeRightTransformPCA.linearTransformModel.modes.rows()));
    const int totalCols = facePCA.NumPCAModes() + 1;

    Eigen::Matrix<float, -1, -1> matrix(offsets.back(), totalCols);
    matrix.block(offsets[0], 0, offsets[1] - offsets[0], 1) = facePCA.BaseAsVector();
    matrix.block(offsets[0], 1, offsets[1] - offsets[0], totalCols - 1) = facePCA.Modes(LinearVertexModel<float>::EvaluationMode::STATIC);
    matrix.block(offsets[1], 0, offsets[2] - offsets[1], 1) = teethPCA.BaseAsVector();
    matrix.block(offsets[1], 1, offsets[2] - offsets[1], totalCols - 1) = teethPCA.Modes(LinearVertexModel<float>::EvaluationMode::STATIC);
    matrix.block(offsets[2], 0, offsets[3] - offsets[2], 1) = eyeLeftTransformPCA.linearTransformModel.mean;
    matrix.block(offsets[2], 1, offsets[3] - offsets[2], totalCols - 1) = eyeLeftTransformPCA.linearTransformModel.modes;
    matrix.block(offsets[2], 0, offsets[4] - offsets[3], 1) = eyeRightTransformPCA.linearTransformModel.mean;
    matrix.block(offsets[2], 1, offsets[4] - offsets[3], totalCols - 1) = eyeRightTransformPCA.linearTransformModel.modes;

    npy::SaveMatrixAsNpy(filename, matrix);
}

} // namespace epic::nls::rt
