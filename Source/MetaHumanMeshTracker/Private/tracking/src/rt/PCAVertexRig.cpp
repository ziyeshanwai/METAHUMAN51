// Copyright Epic Games, Inc. All Rights Reserved.

#include <tracking/rt/PCAVertexRig.h>
#include <nls/geometry/EulerAngles.h>
#include <nls/rig/Rig.h>
#include <nls/rig/RigLogicDNAResource.h>

#include <riglogic/RigLogic.h>

#include <array>
#include <string>

namespace epic::nls::rt {

HeadVertexState<float> PCAVertexRig::EvaluatePCARig(const Eigen::VectorX<float>& pcaCoeffs) const
{
    HeadVertexState<float> headVertexState;
    headVertexState.faceVertices = facePCA.EvaluateLinearized(pcaCoeffs, LinearVertexModel<float>::EvaluationMode::STATIC);
    headVertexState.teethVertices = teethPCA.EvaluateLinearized(pcaCoeffs, LinearVertexModel<float>::EvaluationMode::STATIC);
    headVertexState.eyeLeftVertices = eyeLeftPCA.EvaluateLinearized(pcaCoeffs, LinearVertexModel<float>::EvaluationMode::STATIC);
    headVertexState.eyeRightVertices = eyeRightPCA.EvaluateLinearized(pcaCoeffs, LinearVertexModel<float>::EvaluationMode::STATIC);
    return headVertexState;
}

Eigen::VectorX<float> PCAVertexRig::Project(const HeadVertexState<float>& headVertexState, const Eigen::VectorX<float>&/*coeffs*/) const
{
    HeadVertexState<float> mean;
    mean.faceVertices = facePCA.Base();
    mean.teethVertices = teethPCA.Base();
    mean.eyeLeftVertices = eyeLeftPCA.Base();
    mean.eyeRightVertices = eyeRightPCA.Base();
    return solver.solve(headVertexState.Flatten() - mean.Flatten());
}

void PCAVertexRig::Translate(const Eigen::Vector3f& translation)
{
    facePCA.Translate(translation);
    teethPCA.Translate(translation);
    eyeLeftPCA.Translate(translation);
    eyeRightPCA.Translate(translation);
    rootBindPose = Eigen::Translation3f(translation) * rootBindPose;
}

Eigen::Vector3f PCAVertexRig::EyesMidpoint() const
{
    return (0.5f * (eyeLeftPCA.Base() + eyeRightPCA.Base())).rowwise().mean();
}

void PCAVertexRig::SaveAsDNA(const std::string& filename) const
{
    const std::vector<std::string> allMeshNames = {HeadMeshName(), TeethMeshName(), EyeLeftMeshName(), EyeRightMeshName()};
    const uint16_t numPCAcoeffs = uint16_t(NumCoeffs());
    constexpr float rad2deg = float(180.0 / CARBON_PI);

    // we save the meshes with the mean
    HeadVertexState<float> meanHeadVertexState = EvaluatePCARig(Eigen::VectorXf::Zero(NumCoeffs()));
    std::vector<Eigen::Matrix<float, 3, -1>> allMeanVertices;
    allMeanVertices.push_back(meanHeadVertexState.faceVertices);
    allMeanVertices.push_back(meanHeadVertexState.teethVertices);
    allMeanVertices.push_back(meanHeadVertexState.eyeLeftVertices);
    allMeanVertices.push_back(meanHeadVertexState.eyeRightVertices);

    pma::ScopedPtr<dna::FileStream> stream = pma::makeScoped<dna::FileStream>(filename.c_str(),
                                                                dna::FileStream::AccessMode::Write,
                                                                dna::FileStream::OpenMode::Binary);
    pma::ScopedPtr<dna::StreamWriter> writer = pma::makeScoped<dna::StreamWriter>(stream.get());

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
    writer->setMeshIndices(0, meshIndices.data(), uint16_t(meshIndices.size()));
    writer->setLODMeshMapping(0, 0);

    writer->clearBlendShapeChannelNames();
    writer->clearMeshBlendShapeChannelMappings();
    std::vector<uint16_t> controlInputIndices;
    std::vector<uint16_t> blendshapeOutputIndices;
    for (uint16_t meshIndex = 0; meshIndex < uint16_t(allMeshNames.size()); ++meshIndex) {
        for (uint16_t bsIndex = 0; bsIndex < numPCAcoeffs; ++bsIndex) {
            const uint16_t meshBsIndex = numPCAcoeffs * meshIndex + bsIndex;
            writer->setBlendShapeChannelName(meshBsIndex, (allMeshNames[meshIndex] + std::string("_bs_pca_") + std::to_string(bsIndex)).c_str());
            writer->setMeshBlendShapeChannelMapping(0, meshIndex, meshBsIndex);
            controlInputIndices.push_back(bsIndex);
            blendshapeOutputIndices.push_back(meshBsIndex);
        }
    }
    writer->setBlendShapeChannelIndices(0, blendshapeOutputIndices.data(), uint16_t(blendshapeOutputIndices.size()));
    writer->clearLODBlendShapeChannelMappings();
    writer->setLODBlendShapeChannelMapping(0, 0);

    writer->clearJointNames();
    writer->setJointName(0, FacialRootJointName());
    std::array<std::uint16_t, 3> jointIndices{0};
    writer->clearJointIndices();
    writer->setJointIndices(0, jointIndices.data(), uint16_t(jointIndices.size()));
    writer->clearLODJointMappings();
    writer->setLODJointMapping(0, 0);
    std::array<std::uint16_t, 3> jointHierarchy{0};
    writer->setJointHierarchy(jointHierarchy.data(), uint16_t(jointHierarchy.size()));

    Eigen::Matrix<float, 3, -1> jointTranslations(3, 1);
    Eigen::Matrix<float, 3, -1> jointRotations(3, 1);
    {
        jointRotations.col(0) = (RotationMatrixToEulerXYZ<float>(rootBindPose.linear()) * rad2deg).template cast<float>();
        jointTranslations.col(0) = rootBindPose.translation();
    }
    writer->setNeutralJointTranslations((const dna::Vector3*)jointTranslations.data(), 1);
    writer->setNeutralJointRotations((const dna::Vector3*)jointRotations.data(), 1);

    writer->setJointRowCount(9);
    writer->setJointColumnCount(numPCAcoeffs);


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
        const uint16_t blendshapeChannelLODs = uint16_t(allMeshNames.size() * numPCAcoeffs);
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
            writer->setFaceVertexLayoutIndices(meshIndex, totalFaces - 1, nullptr, 0);

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
            const std::uint16_t jointIndex = 0;
            for (int vID = numVertices - 1; vID >= 0; --vID) {
                writer->setSkinWeightsValues(meshIndex, vID, &weight, 1);
                writer->setSkinWeightsJointIndices(meshIndex, vID, &jointIndex, 1);
            }
        }
    }

    for (uint16_t meshIndex = 0; meshIndex < uint16_t(allMeshNames.size()); ++meshIndex) {
        writer->clearBlendShapeTargets(meshIndex);
        const LinearVertexModel<float>* linearModelPtr = nullptr;
        switch (meshIndex) {
            case 0: linearModelPtr = &facePCA; break;
            case 1: linearModelPtr = &teethPCA; break;
            case 2: linearModelPtr = &eyeLeftPCA; break;
            case 3: linearModelPtr = &eyeRightPCA; break;
        }
        const int numVertices = meshes[meshIndex].NumVertices();
        Eigen::VectorX<std::uint32_t> vertexIndices = Eigen::VectorX<std::uint32_t>::LinSpaced(numVertices, 0, numVertices - 1);
        for (uint16_t bsIndex = 0; bsIndex < numPCAcoeffs; ++bsIndex) {
            const uint16_t meshBsIndex = uint16_t(numPCAcoeffs * meshIndex + bsIndex);
            writer->setBlendShapeChannelIndex(meshIndex, bsIndex, meshBsIndex);
            const Eigen::VectorXf mode = linearModelPtr->Modes(LinearVertexModel<float>::EvaluationMode::STATIC).col(bsIndex);
            writer->setBlendShapeTargetDeltas(meshIndex, bsIndex, (const dna::Delta*)mode.data(), numVertices);
            writer->setBlendShapeTargetVertexIndices(meshIndex, bsIndex, vertexIndices.data(), numVertices);
        }
    }

    writer->write();


    // verify that we get the same reconstruction using the rig
    Rig<float> testDnaRig;
    if (!testDnaRig.LoadRig(filename)) {
        LOG_ERROR("failed to read dna rig");
    }
    PCAVertexRig testPcaRig;
    if (!testPcaRig.LoadFromDNA(filename)) {
        LOG_ERROR("failed to read pca rig from dna");
    }


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

bool PCAVertexRig::LoadFromDNA(const std::string& filename)
{
    std::shared_ptr<const RigLogicDNAResource> dnaResource = RigLogicDNAResource::LoadDNA(filename, /*retain=*/false);
    if (!dnaResource) {
        LOG_ERROR("failed to open dnafile {}", filename);
        return false;
    }

    return LoadFromDNA(dnaResource->Stream());
}


bool PCAVertexRig::LoadFromDNA(dna::StreamReader* reader)
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

    if (reader->getBlendShapeChannelLODs()[0] != numPCACoeffs * 4
       || reader->getMeshCount() != 4
       || reader->getBlendShapeTargetCount(0) != numPCACoeffs
       || reader->getBlendShapeTargetCount(1) != numPCACoeffs
       || reader->getBlendShapeTargetCount(2) != numPCACoeffs
       || reader->getBlendShapeTargetCount(3) != numPCACoeffs) {
        LOG_ERROR("number of PCA blendshape channels incorrect");
        return false;
    }

    if (reader->getJointCount() != 1) {
        LOG_ERROR("unexpected number of joints in pca rig");
        return false;
    }

    // load meshes
    meshes[0] = RigGeometry<float>::ReadMesh(reader, 0); // face
    meshes[1] = RigGeometry<float>::ReadMesh(reader, 1); // teeth
    meshes[2] = RigGeometry<float>::ReadMesh(reader, 2); // left eye
    meshes[3] = RigGeometry<float>::ReadMesh(reader, 3); // right eye

    std::vector<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> modes(4);
    modes[0].resize(meshes[0].NumVertices() * 3, numPCACoeffs);
    modes[1].resize(meshes[1].NumVertices() * 3, numPCACoeffs);
    modes[2].resize(meshes[2].NumVertices() * 3, numPCACoeffs);
    modes[3].resize(meshes[3].NumVertices() * 3, numPCACoeffs);

    // read blendshape data
    for (std::uint16_t meshIndex = 0; meshIndex < 4; ++meshIndex) {
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
    eyeLeftPCA.Create(meshes[2].Vertices(), modes[2]);
    eyeRightPCA.Create(meshes[3].Vertices(), modes[3]);

    return true;
}

} // namespace epic::nls::rt
