// Copyright Epic Games, Inc. All Rights Reserved.

#include <rigposebasedsolver/RigGenerateTrainingData.h>
#include <rigposebasedsolver/FramePoseData.h>
#include <posebasedsolver/serialization_utils.h>
#include <nls/geometry/Procrustes.h>
#include <set>

namespace epic
{
    namespace rigposebasedsolver
    {

        using namespace epic::carbon;
        using namespace epic::nls;

        const unsigned RigGenerateTrainingData::MeshVertexIdData::version = 2;

        void serialize(const RigGenerateTrainingData::MeshVertexIdData& item, std::ostream& out)
        {
            using dlib::serialize;
            // Output current class version number
            // IMPORTANT: if you add to or change the class member variables, please up the version number
            dlib::serialize(RigGenerateTrainingData::MeshVertexIdData::version, out);
            dlib::serialize(item.vertexIdsAndShapeIndices, out);
            dlib::serialize(item.vertexIdOutputOffset, out);
        }

        void deserialize(RigGenerateTrainingData::MeshVertexIdData& item, std::istream& in)
        {
            using dlib::deserialize;
            unsigned curVersion;
            dlib::deserialize(curVersion, in);

            // current version deserialization
            // IMPORTANT: please try and make class serialization / deserialization back-compatible
            // by supporting multiple versions if possible, and if not, function should throw an exception
            if (curVersion == 2u)
            {
                using dlib::deserialize;
                dlib::deserialize(item.vertexIdsAndShapeIndices, in);
                dlib::deserialize(item.vertexIdOutputOffset, in);
            }
            else
            {
                throw cm::serialization_error("RigGenerateTrainingData::MeshVertexIdData is not compatible with version: " + std::to_string(curVersion));
            }
        }

        const unsigned RigGenerateTrainingData::PoseBasedSolverTrainingDataParameters::version = 1;


        void serialize(const RigGenerateTrainingData::PoseBasedSolverTrainingDataParameters& item, std::ostream& out)
        {
            using dlib::serialize;
            // Output current class version number
            // IMPORTANT: if you add to or change the class member variables, please up the version number
            dlib::serialize(RigGenerateTrainingData::PoseBasedSolverTrainingDataParameters::version, out);
            dlib::serialize(item.randomSeed, out);
            dlib::serialize(item.vertexNoiseSd, out);
            dlib::serialize(item.rigAlignmentVertexSd, out);
            dlib::serialize(item.numAugmentationsPerFrame, out);
            dlib::serialize(item.numControlsPerAugmentation, out);
            dlib::serialize(item.fractionalAugmentationSd, out);
            dlib::serialize(item.solverControlNames, out);
            dlib::serialize(item.controlMappers, out);
        }


        void deserialize(RigGenerateTrainingData::PoseBasedSolverTrainingDataParameters& item, std::istream& in)
        {
            using dlib::deserialize;
            unsigned curVersion;
            dlib::deserialize(curVersion, in);

            // current version deserialization
            // IMPORTANT: please try and make class serialization / deserialization back-compatible
            // by supporting multiple versions if possible, and if not, function should throw an exception
            if (curVersion == 1u)
            {
                using dlib::deserialize;
                dlib::deserialize(item.randomSeed, in);
                dlib::deserialize(item.vertexNoiseSd, in);
                dlib::deserialize(item.rigAlignmentVertexSd, in);
                dlib::deserialize(item.numAugmentationsPerFrame, in);
                dlib::deserialize(item.numControlsPerAugmentation, in);
                dlib::deserialize(item.fractionalAugmentationSd, in);
                dlib::deserialize(item.solverControlNames, in);
                dlib::deserialize(item.controlMappers, in);
            }
            else
            {
                throw cm::serialization_error("RigGenerateTrainingData::PoseBasedSolverTrainingDataParameters is not compatible with version: " + std::to_string(curVersion));
            }
        }


        const std::vector<std::string> RigGenerateTrainingData::m_meshNames = { "head_lod0_mesh", "teeth_lod0_mesh", "eyeLeft_lod0_mesh", "eyeRight_lod0_mesh" };

        bool RigGenerateTrainingData::IsInitialized() const
        {
            return m_rig && m_rig->GetRigLogic().get() != nullptr;
        }

        const Eigen::Matrix<float, 2, -1>& RigGenerateTrainingData::GuiControlRanges() const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before function GuiControlRanges can be called");
            return m_rig->GetRigLogic()->GuiControlRanges();
        }

        const std::vector<std::string>& RigGenerateTrainingData::GuiControlNames() const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before function GuiControlNames can be called");
            return m_guiControlNames;
        }

        const std::vector<std::string>& RigGenerateTrainingData::SolverControlNames() const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before function SolverControlNames can be called");
            return m_poseSolverTrainingDataParameters.solverControlNames;
        }

        const Eigen::VectorX<float>& RigGenerateTrainingData::DefaultUIControlValues() const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before function DefaultUIControlValues can be called");
            return m_defaultUIControlValues;
        }

        const std::string& RigGenerateTrainingData::GetMeshName(int meshRigIndex) const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before function GetMeshName can be called");
            CARBON_ASSERT(meshRigIndex < m_rig->GetRigGeometry()->NumMeshes(), "meshRigIndex is out of range");
            return m_rig->GetRigGeometry()->GetMeshName(meshRigIndex);
        }

        int RigGenerateTrainingData::VertexOffset(const std::string& meshName) const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before function VertexOffset can be called");
            CARBON_ASSERT(m_rigMeshOutputVertexIdData.find(meshName) != m_rigMeshOutputVertexIdData.end(), "meshName is not a valid mesh for the solver");
            return m_rigMeshOutputVertexIdData.at(meshName).vertexIdOutputOffset;
        }

        const std::vector<std::string>& RigGenerateTrainingData::MeshNames()
        {
            return m_meshNames;
        }

        const Mesh<float>& RigGenerateTrainingData::GetMesh(const std::string& meshName) const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before function GetMesh can be called");
            const int meshIndex = m_rig->GetRigGeometry()->GetMeshIndex(meshName);
            CARBON_ASSERT(meshIndex != -1, "Invalid meshName {} specified for call to GetMesh", meshName);
            return m_rig->GetRigGeometry()->GetMesh(meshIndex);
        }

        const std::map< std::string, RigGenerateTrainingData::MeshVertexIdData>& RigGenerateTrainingData::RigMeshOutputVertexIdData() const
        {
            return m_rigMeshOutputVertexIdData;
        }


        unsigned RigGenerateTrainingData::GetNumAnimationFrames(unsigned animationIndex) const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before function GetNumAnimationFrames can be called");
            CARBON_ASSERT(animationIndex < m_animationData.size(), "animationIndex {} is out of range", animationIndex);
            if (!m_animationData[animationIndex].empty())
            {
                return static_cast<unsigned>(m_animationData[animationIndex].begin()->second.size());
            }

            return 0;
        }

        std::vector<std::pair<int, std::vector<unsigned>>> RigGenerateTrainingData::OutputVertexIdsAndShapeIndices(const std::string& meshName) const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before function OutputVertexIdsAndShapeIndices can be called");
            std::vector<std::pair<int, std::vector<unsigned>>> vertexIdsAndShapeIndices;
            auto it = m_rigMeshOutputVertexIdData.find(meshName);
            if (it != m_rigMeshOutputVertexIdData.end())
            {
                vertexIdsAndShapeIndices = it->second.vertexIdsAndShapeIndices;
            }

            return vertexIdsAndShapeIndices;
        }


        void RigGenerateTrainingData::GetRigControlData(unsigned currentAnimationFile, unsigned currentFrameNumber, Eigen::VectorX<float>& rigControlValues) const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before calling GetRigControlData");
            CARBON_ASSERT(currentAnimationFile < m_animationData.size(), "currentAnimationFile is out of range");
            CARBON_ASSERT(currentFrameNumber < GetNumAnimationFrames(currentAnimationFile), "currentFrameNumber is out of range");

            rigControlValues = m_defaultUIControlValues;
            for (const auto& values : m_animationData[currentAnimationFile])
            {
                auto curControlName = values.first;
                auto it = std::find(m_guiControlNames.begin(), m_guiControlNames.end(), curControlName);

                if (it != m_guiControlNames.end())
                {
                    unsigned index = static_cast<unsigned>(it - m_guiControlNames.begin());
                    rigControlValues[index] = static_cast<float>(values.second.at(currentFrameNumber));
                }
                else
                {
                    printf("WARNING: control %s is not used in solver training\n", curControlName.c_str());
                }
            }
        }

        void RigGenerateTrainingData::GetSolverControlMinMaxDefault(const std::string& controlName, float& minVal, float& maxVal, float& defaultVal) const
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before calling GetSolverControlMinMaxDefault");
            auto it = std::find(m_guiControlNames.begin(), m_guiControlNames.end(), controlName);
            CARBON_ASSERT(it != m_guiControlNames.end(), "Control {} is not a valid rig control", controlName);
            unsigned controlIndex = static_cast<unsigned>(it - m_guiControlNames.begin());
            const Eigen::Matrix<float, 2, -1>& controlRanges = m_rig->GetRigLogic()->GuiControlRanges();
            minVal = controlRanges(0, controlIndex);
            maxVal = controlRanges(1, controlIndex);
            defaultVal = m_defaultUIControlValues(controlIndex);
        }



        void RigGenerateTrainingData::EvaluateRigVerticesAndPerturbedControlValues(Eigen::VectorX<float>& curRigControlValues, std::vector<Eigen::Matrix<float, 3, -1>>& vertices, bool addControlPerturbation, bool addVertexNoise, bool addRigidAlignmentNoise)
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before calling EvaluateRigVerticesAndPerturbedControlValues");
            CARBON_ASSERT(curRigControlValues.size() == static_cast<int>(m_guiControlNames.size()), "curRigControlValues must be the same size as the number of rig UI controls");

            std::normal_distribution<> distNormal(0, 1);
            std::uniform_int_distribution<> distUniformInt(0, static_cast<int>(m_poseSolverTrainingDataParameters.solverControlNames.size()) - 1);

            // randomly perturb the control values
            if (addControlPerturbation)
            {
                const Eigen::Matrix<float, 2, -1>& controlRanges = m_rig->GetRigLogic()->GuiControlRanges();

                for (unsigned i = 0; i < m_poseSolverTrainingDataParameters.numControlsPerAugmentation; i++)
                {
                    // pick a control at random from the list of solver controls, add a random offset and clamp it to the control range
                    const int solverControlIndex = distUniformInt(m_randomEngine);
                    auto it = std::find(m_guiControlNames.begin(), m_guiControlNames.end(), m_poseSolverTrainingDataParameters.solverControlNames[unsigned(solverControlIndex)]);
                    assert(it != m_guiControlNames.end());
                    unsigned controlIndex = static_cast<unsigned>(it - m_guiControlNames.begin());
                    const auto controlRange = controlRanges(1, controlIndex) - controlRanges(0, controlIndex);
                    curRigControlValues[controlIndex] += static_cast<float>(distNormal(m_randomEngine) * m_poseSolverTrainingDataParameters.fractionalAugmentationSd * controlRange);
                    curRigControlValues[controlIndex] = std::clamp(curRigControlValues[controlIndex], controlRanges(0, controlIndex), controlRanges(1, controlIndex));
                }
            }


            vertices = m_rig->EvaluateVertices(curRigControlValues, /*lod=*/0, m_meshIndices);

            // randomly perturb each vertex
            for (int i = 0; i < int(vertices.size()); ++i) {
                if (addVertexNoise)
                {
                    for (unsigned j = 0; j < vertices[i].cols(); j++)
                    {
                        vertices[i](0, j) += static_cast<float>(distNormal(m_randomEngine) * m_poseSolverTrainingDataParameters.vertexNoiseSd);
                        vertices[i](1, j) += static_cast<float>(distNormal(m_randomEngine) * m_poseSolverTrainingDataParameters.vertexNoiseSd);
                        vertices[i](2, j) += static_cast<float>(distNormal(m_randomEngine) * m_poseSolverTrainingDataParameters.vertexNoiseSd);
                    }
                }
            }

            // randomly perturb rigid alignment of rig
            if (addRigidAlignmentNoise)
            {
                // get three mesh points; rather arbitrary; use outer corners of eyes and just below nose
                // 18723, 21682, 2916
                auto headVertices = m_rig->EvaluateVertices(curRigControlValues, /*lod=*/0, { 0 });

                Eigen::Matrix<float, 3, 3> currentMeshPoints, targetMeshPoints;
                currentMeshPoints(0, 0) = headVertices[0](0, 18723);
                currentMeshPoints(1, 0) = headVertices[0](1, 18723);
                currentMeshPoints(2, 0) = headVertices[0](2, 18723);
                currentMeshPoints(0, 1) = headVertices[0](0, 21682);
                currentMeshPoints(1, 1) = headVertices[0](1, 21682);
                currentMeshPoints(2, 1) = headVertices[0](2, 21682);
                currentMeshPoints(0, 2) = headVertices[0](0, 2916);
                currentMeshPoints(1, 2) = headVertices[0](1, 2916);
                currentMeshPoints(2, 2) = headVertices[0](2, 2916);
                targetMeshPoints = currentMeshPoints;

                for (unsigned i = 0; i < targetMeshPoints.rows(); i++)
                {
                    for (unsigned j = 0; j < targetMeshPoints.cols(); j++)
                    {
                        targetMeshPoints(i, j) += static_cast<float>(distNormal(m_randomEngine) * m_poseSolverTrainingDataParameters.rigAlignmentVertexSd);
                    }
                }

                auto transform = Procrustes<float, 3>::AlignRigid(currentMeshPoints, targetMeshPoints);
                for (int i = 0; i < int(vertices.size()); ++i) {
                    vertices[i] = transform.Transform(vertices[i]);
                }
            }
        }

        bool RigGenerateTrainingData::IsValidMeshData(std::shared_ptr<const Rig<float>> rig, const std::string& meshName,
            const std::vector<std::pair<int, std::vector<unsigned>>>& vertexIdsAndShapeIndices, unsigned numShapes, std::string& errorString)
        {
            errorString = "";
            auto it = std::find(m_meshNames.begin(), m_meshNames.end(), meshName);
            if (it == m_meshNames.end())
            {
                errorString = "Mesh name " + meshName + " specified in solver config file is not a valid mesh name";
                return false;
            }

            const int meshIndex = rig->GetRigGeometry()->GetMeshIndex(meshName);
            const Mesh<float>& quadMesh = rig->GetRigGeometry()->GetMesh(meshIndex);

            for (const auto& id : vertexIdsAndShapeIndices)
            {
                if (id.first< 0 || id.first >= quadMesh.NumVertices())
                {
                    errorString = "solver config mesh_vertices data for mesh " + meshName + " contains an invalid vertex id " + std::to_string(id.first);
                    return false;
                }

                for (const auto& index : id.second)
                {
                    if (index >= numShapes)
                    {
                        errorString = "solver config mesh_vertices data for mesh " + meshName + " contains an invalid shape number " + std::to_string(index) +
                            " for vertex id " + std::to_string(id.first);
                        return false;
                    }

                }
            }

            std::set<int> vertexIdsSet;
            for (const auto& id : vertexIdsAndShapeIndices)
            {
                if (vertexIdsSet.insert(id.first).second == false)
                {
                    errorString = "Solver config contains duplicate vertex id: " + std::to_string(id.first);
                    return false;
                }
            }

            return true;
        }

        bool RigGenerateTrainingData::IsRigGenerationDataConsistent(std::shared_ptr<const Rig<float>> rig, const std::vector< std::map<std::string, std::map<int, double>> >& animationData,
            const std::map< std::string, MeshVertexIdData>& rigMeshOutputVertexIdData, const PoseBasedSolverTrainingDataParameters& poseSolverTrainingDataParameters,
            unsigned numShapes, std::string & errorString)
        {
            const auto guiControlNames = rig->GetGuiControlNames();

            // check the training data parameters
            for (const auto& controlName : poseSolverTrainingDataParameters.solverControlNames)
            {
                auto it = std::find(guiControlNames.begin(), guiControlNames.end(), controlName);
                if (it == guiControlNames.end())
                {
                    errorString =  "rig control " + controlName + " specified in solver config file is not a valid control";
                    return false;
                }
            }

            // check that the vertex id data is valid
            for (const auto& vertexIdData : rigMeshOutputVertexIdData)
            {
                if (!RigGenerateTrainingData::IsValidMeshData(rig, vertexIdData.first, vertexIdData.second.vertexIdsAndShapeIndices, numShapes, errorString))
                {
                    return false;
                }
            }

            // check the values of the animation data
            unsigned count = 0;
            for (const auto& animationDataSet : animationData)
            {
                // check if all animation controls are present
                // TODO need to check whether this is a strict requirement; for now just display as warning
                for (const auto& controlName : poseSolverTrainingDataParameters.solverControlNames)
                {
                    if (animationDataSet.find(controlName) == animationDataSet.end())
                    {
                        std::cout << "WARNING: Control " + controlName + " not present in animation data-set " + std::to_string(count + 1) << std::endl;
                    }
                }

                // check that the same frames are present for all controls in each file
                bool first = true;
                std::vector<int> allFrameNumbers;
                for (const auto & controlData : animationDataSet)
                {
                    if (first)
                    {
                        for (const auto& val : controlData.second)
                        {
                            allFrameNumbers.emplace_back(val.first);
                        }
                        first = false;
                    }
                    else
                    {
                        std::vector<int> curFrameNumbers;
                        for (const auto& val : controlData.second)
                        {
                            curFrameNumbers.emplace_back(val.first);
                        }

                        if (curFrameNumbers != allFrameNumbers)
                        {
                            errorString = "Control data for data-set " + std::to_string(count + 1) + " does not contain a consistent number of frames across all controls";
                            return false;
                        }
                    }
                }
                count++;
            }

            return true;
        }


        void RigGenerateTrainingData::Init(std::shared_ptr<const Rig<float>> rig, const std::vector< std::map<std::string, std::map<int, double>> > & animationData,
            const std::map< std::string, MeshVertexIdData> & rigMeshOutputVertexIdData, const PoseBasedSolverTrainingDataParameters& poseSolverTrainingDataParameters,
            unsigned numShapes)
        {
            std::string errorString = "";
            CARBON_ASSERT(IsRigGenerationDataConsistent(rig, animationData, rigMeshOutputVertexIdData, poseSolverTrainingDataParameters, numShapes, errorString),
                "Rig generation data is inconsistent, with error: {}", errorString);
            m_rig = rig;
            m_deformationModelRigLogic.SetRig(m_rig);
            m_defaultUIControlValues = m_deformationModelRigLogic.GuiControls();

            m_meshIndices.clear();
            for (const std::string& meshName : m_meshNames) {
                const int meshIndex = m_rig->GetRigGeometry()->GetMeshIndex(meshName);
                m_meshIndices.emplace_back(meshIndex);
            }
            m_guiControlNames = rig->GetGuiControlNames();

            m_rigMeshOutputVertexIdData = rigMeshOutputVertexIdData;
            m_animationData = animationData;
            m_poseSolverTrainingDataParameters = poseSolverTrainingDataParameters;
            m_numShapes = numShapes;
        }

        void RigGenerateTrainingData::GenerateTrainingData(std::vector<FramePoseData>& poseData, std::function<void(float)> progressCallback, float progress, float progressStep, std::function<bool(void)> cancelledCallback)
        {
            CARBON_ASSERT(IsInitialized(), "RigGenerateTrainingData must be initialized before calling GenerateTrainingData");
            poseData.clear();
            m_randomEngine.seed(m_poseSolverTrainingDataParameters.randomSeed);

            size_t nsteps = 0;
            for (unsigned i = 0; i < m_animationData.size(); i++)
            {
                nsteps += (m_animationData[i].begin()->second.size() * (m_poseSolverTrainingDataParameters.numAugmentationsPerFrame + 1));
            }
            progressStep /= nsteps;

            for (unsigned i = 0; i < m_animationData.size(); i++)
            {
                std::vector<unsigned> frameNumbers(m_animationData[i].begin()->second.size());
                unsigned  f = 0;
                for (const auto& item : m_animationData[i].begin()->second)
                {
                    frameNumbers[f] = static_cast<unsigned>(item.first);
                    f++;
                }

                for (unsigned j = 0; j < frameNumbers.size(); j++)
                {
                    const auto currentAnimationFile = static_cast<unsigned>(i);
                    const auto curFrameNumber = frameNumbers[j];

                    for (unsigned l = 0; l < m_poseSolverTrainingDataParameters.numAugmentationsPerFrame + 1; l++)
                    {
                        FramePoseData curFrameData;
                        curFrameData.shapes.resize(m_numShapes);

                        Eigen::VectorX<float> rigControlValues;
 
                        GetRigControlData(currentAnimationFile, curFrameNumber, rigControlValues);
                        std::vector<Eigen::Matrix<float, 3, -1>> vertices;

                        if (m_rig->GetRigLogic()) {
                            if (l == 0)
                            {
                                EvaluateRigVerticesAndPerturbedControlValues(rigControlValues, vertices, false, false, false);
                            }
                            else
                            {
                                EvaluateRigVerticesAndPerturbedControlValues(rigControlValues, vertices, true, true, true);
                            }

                            for (unsigned k = 0; k < m_poseSolverTrainingDataParameters.solverControlNames.size(); k++)
                            {
                                auto it = std::find(m_guiControlNames.begin(), m_guiControlNames.end(), m_poseSolverTrainingDataParameters.solverControlNames[k]);
                                assert(it != m_guiControlNames.end());
                                const unsigned controlIndex = static_cast<unsigned>(it - m_guiControlNames.begin());
                                curFrameData.controlValues[m_poseSolverTrainingDataParameters.solverControlNames[k]] = rigControlValues[controlIndex];
                            }
 
                            // remap the control data
                            for (unsigned c = 0; c < m_poseSolverTrainingDataParameters.controlMappers.size(); c++)
                            {
                                 bool mapped = m_poseSolverTrainingDataParameters.controlMappers[c].Map(curFrameData.controlValues);
                                 if (!mapped)
                                 {
                                     CARBON_CRITICAL("Failed to map control mapper {}", m_poseSolverTrainingDataParameters.controlMappers[c].toName);
                                 }
                            }

                            // extract the required vertices
                            // offsetting the point id by the amount specified in the config so that we don't get clashes between point ids from different
                            // meshes (if we have them)
                            for (unsigned shape = 0; shape < m_numShapes; shape++)
                            {
                                for (unsigned k = 0; k < m_meshNames.size(); k++)
                                {
                                    auto it = m_rigMeshOutputVertexIdData.find(m_meshNames[k]);
                                    if (it != m_rigMeshOutputVertexIdData.end())
                                    {
                                        for (const auto& vertexId : it->second.vertexIdsAndShapeIndices)
                                        {
                                            const int curVertexId = vertexId.first + it->second.vertexIdOutputOffset;
                                            if (std::find(vertexId.second.begin(), vertexId.second.end(), shape) != vertexId.second.end())
                                            {
                                                curFrameData.shapes[shape][std::to_string(curVertexId)] = vertices[k].col(vertexId.first);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        poseData.emplace_back(curFrameData);

						progress += progressStep;
						progressCallback(progress);
						if (cancelledCallback())
						{
							return;
						}
                    }
                }
            }
        }
    }
}
