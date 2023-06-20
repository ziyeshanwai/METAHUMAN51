// Copyright Epic Games, Inc. All Rights Reserved.

#include <tracking/FaceTracking.h>
#include <rigposebasedsolver/RigPoseBasedSolverData.h>
#include <rigposebasedsolver/RigGenerateTrainingData.h>
#include <carbon/io/JsonIO.h>
#include <carbon/io/Utils.h>
#include <nls/BoundedCoordinateDescentSolver.h>
#include <nls/functions/VertexConstraintsFunction.h>
#include <nls/geometry/EulerAngles.h>
#include <nls/geometry/Polyline.h>
#include <nls/geometry/RigidICP.h>
#include <nls/rig/RigLogicDNAResource.h>
#include <nrr/deformation_models/DeformationModelRigidScale.h>
#include <nrr/DepthmapConstraints.h>
#include <tracking/LandmarkTriangulation.h>
#include <tracking/rt/PCARig.h>
#include <tracking/rt/PCAVertexRig.h>
#include <carbon/common/External.h>

#include <execution>

namespace epic::nls {

using namespace pma;

static int magicNumber_v1 = 54321;
static int magicNumber_v2 = 54322;

bool SaveFaceTrackingStates(const std::string& filename, const std::map<int, std::shared_ptr<const FaceTrackingState>>& faceTrackingStates)
{
    int numFrames = 0;
    for (const auto& [_, state] : faceTrackingStates) {
        if (state) numFrames++;
    }

    if (numFrames == 0) {
        return false;
    }

    // const int version = 1; // version 1 added pca coeffs
    const int version = 2; // version 2 added postprocessing result

    FILE* pFile = nullptr;
    bool success = true;
#ifdef _MSC_VER
	fopen_s(&pFile, filename.c_str(), "wb");
#else
    pFile = fopen(filename.c_str(), "wb");
#endif
    if (pFile) {
        success &= (fwrite(&magicNumber_v2, sizeof(magicNumber_v2), 1, pFile) == 1);
        success &= (fwrite(&version, sizeof(version), 1, pFile) == 1);
        success &= (fwrite(&numFrames, sizeof(numFrames), 1, pFile) == 1);
        for (const auto& [frameIndex, state] : faceTrackingStates) {
            if (state) {
                success &= (fwrite(&frameIndex, sizeof(frameIndex), 1, pFile) == 1);
                success &= ToBinaryFile(pFile, state->m_mesh2scan.Matrix());
                success &= ToBinaryFile(pFile, state->m_guiControls);
                success &= ToBinaryFile(pFile, state->m_rigVertices);
                success &= ToBinaryFile(pFile, state->m_vertexOffsets);
                success &= ToBinaryFile(pFile, state->m_vertices);
                success &= ToBinaryFile(pFile, state->m_pcaCoeffs);
                if (state->m_postProcessedGuiControls) {
                    success &= ToBinaryFile(pFile, *state->m_postProcessedGuiControls);
                } else {
                    Eigen::VectorXf postProcessedGuiControls;
                    success &= ToBinaryFile(pFile, postProcessedGuiControls);
                }
            } else {
                fclose(pFile);
                CARBON_CRITICAL("only valid face tracking states can be saved");
            }
        }
        fclose(pFile);

        if (!success) {
            LOG_ERROR("failed to save mesh solve states");
        }
    } else {
        LOG_ERROR("failed to open file {} for writing", filename);
        success = false;
    }
    return success;
}

bool LoadFaceTrackingStates(const std::string& filename, std::map<int, std::shared_ptr<FaceTrackingState>>& faceTrackingStates, int& version)
{
    bool success = true;
    version = -1;
    FILE* pFile = nullptr;
#ifdef _MSC_VER
	fopen_s(&pFile, filename.c_str(), "rb");
#else
    pFile = fopen(filename.c_str(), "rb");
#endif
    if (pFile) {
        std::map<int, std::shared_ptr<FaceTrackingState>> frameStatesMap;
        int readMagicNumber = 0;
        success &= (fread(&readMagicNumber, sizeof(magicNumber_v2), 1, pFile) == 1);
        if (success) {
            if (readMagicNumber == magicNumber_v1) {
                version = 0;
            } else if (readMagicNumber == magicNumber_v2) {
                success &= (fread(&version, sizeof(version), 1, pFile) == 1);
            } else {
                success = false;
                version = -1;
            }
        }
        int numFrames = 0;
        success &= (fread(&numFrames, sizeof(numFrames), 1, pFile) == 1);
        for (int i = 0; i < numFrames && success; ++i) {
            std::shared_ptr<FaceTrackingState> frameState = std::make_shared<FaceTrackingState>();
            int frameIndex = 0;
            success &= (fread(&frameIndex, sizeof(i), 1, pFile) == 1);
            Eigen::Matrix4f mat;
            success &= FromBinaryFile(pFile, mat);
            frameState->m_mesh2scan.SetMatrix(mat);
            success &= FromBinaryFile(pFile, frameState->m_guiControls);
            success &= FromBinaryFile(pFile, frameState->m_rigVertices);
            success &= FromBinaryFile(pFile, frameState->m_vertexOffsets);
            success &= FromBinaryFile(pFile, frameState->m_vertices);
            if (version >= 1) {
                success &= FromBinaryFile(pFile, frameState->m_pcaCoeffs);
            }
            if (version >= 2) {
                Eigen::VectorXf postProcessedGuiControls;
                success &= FromBinaryFile(pFile, postProcessedGuiControls);
                if (postProcessedGuiControls.size() > 0) {
                    frameState->m_postProcessedGuiControls = std::make_shared<Eigen::VectorXf>(postProcessedGuiControls);
                }
            }
            frameStatesMap[frameIndex] = frameState;
        }
        fclose(pFile);

        if (success) {
            faceTrackingStates = frameStatesMap;
        }
    }

    return success;
}

epic::carbon::JsonElement FaceTrackingStateToJson(const FaceTrackingState& state, const TrackingRig<float>& trackingRig)
{
    using namespace epic::carbon;

    const auto currentRigid = state.m_mesh2scan.Matrix();
    const auto& guiControlNames = trackingRig.GetRigLogic()->GuiControlNames();
    const auto& guiControlValues = state.m_postProcessedGuiControls ? *state.m_postProcessedGuiControls : state.m_guiControls;

    JsonElement jOutput(JsonElement::JsonType::Object);
    JsonElement currentRigidJson(JsonElement::JsonType::Array);
    for (int r = 0; r < int(currentRigid.rows()); r++) {
        for (int c = 0; c < int(currentRigid.cols()); c++) {
            currentRigidJson.Append(JsonElement(currentRigid(r, c)));
        }
    }
    jOutput.Insert("currentRigid", std::move(currentRigidJson));

    if (static_cast<size_t>(guiControlValues.size()) != guiControlNames.size()) {
        CARBON_CRITICAL("control values has to be equal to control names vector size");
    }

    for (size_t i = 0; i < guiControlNames.size(); ++i) {
        jOutput.Insert(guiControlNames[i], JsonElement(guiControlValues[i]));
    }

    if (state.m_pcaCoeffs.size() > 0) {
        jOutput.Insert("pca", ToJson2(state.m_pcaCoeffs));
    }

    return jOutput;
}

std::shared_ptr<FaceTrackingState> FaceTrackingStateFromJson(const epic::carbon::JsonElement& data, const TrackingRig<float>& trackingRig)
{
    const auto& guiControlNames = trackingRig.GetRigLogic()->GuiControlNames();
    std::shared_ptr<FaceTrackingState> state = std::make_shared<FaceTrackingState>();
    state->m_guiControls = trackingRig.GetDefaultGuiControlValues();

    // current values
    for (const auto& [controlName, value] : data.Map()) {
        const auto& it = std::find(guiControlNames.begin(), guiControlNames.end(), std::string(controlName));
        if (it != guiControlNames.end()) {
            state->m_guiControls[std::distance(guiControlNames.begin(), it)] = value.Get<float>();
        }
    }
    // current rigid
    Eigen::Matrix<float, 4, 4> currentRigid = Eigen::Matrix<float, 4, 4>::Identity();
    for (int r = 0; r < int(currentRigid.rows()); r++) {
        for (int c = 0; c < int(currentRigid.cols()); c++) {
            currentRigid(r, c) = data["currentRigid"][currentRigid.rows() * r + c].Get<float>();
        }
    }
    state->m_mesh2scan.SetMatrix(currentRigid);

    // current pca
    if (data.Contains("pca")) {
        state->m_pcaCoeffs = FromJson<Eigen::VectorXf>(data["pca"]);
    }

    state->m_rigVertices = trackingRig.EvaluateVertices(state->m_guiControls, /*lod=*/0, /*meshIndex=*/0);
    state->m_vertexOffsets = Eigen::Matrix<float, 3, -1>::Zero(3, state->m_rigVertices.cols());
    state->m_vertices = state->m_rigVertices;

    return state;
}


/**
 * General fitting using GaussNewton with a deformation model, landmark constraints, and ICP constraints.
 */
template <class T>
void FitData(const Mesh<T>& topoplogy,
             DeformationModel<T>* deformationModel,
             LandmarkConstraints2D<T>* landmarkConstraints,
             ICPConstraints<T>* icpConstraints,
             const std::vector<FlowConstraints<T>*>& vectorOfFlowConstraints,
             const std::vector<EyeballConstraints<T>*>& vectorOfEyeballConstraints,
             int iterations)
{
    PROFILING_FUNCTION(PROFILING_COLOR_NAVY);

    if (icpConstraints) {
        icpConstraints->ClearPreviousCorrespondences();
    }

    Mesh<T> mesh = topoplogy;
    mesh.Triangulate();

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {

        Cost<T> cost;

        DiffDataMatrix<T, 3, -1> vertices = deformationModel->EvaluateVertices(context);
        mesh.SetVertices(vertices.Matrix());
        mesh.CalculateVertexNormals();

        cost.Add(deformationModel->EvaluateModelConstraints(context), T(1));

        if (icpConstraints) {
            // calculate closest point constraints only when also evaluating the Jacobian, otherwise it
            // uses the previous targets. This ensures constant correspondences during line search
            cost.Add(icpConstraints->EvaluateICP(vertices, mesh.VertexNormals(), /*searchCorrespondences=*/bool(context)), T(1));
        }

        if (landmarkConstraints) {
            cost.Add(landmarkConstraints->Evaluate(vertices, mesh.VertexNormals()), T(1));
        }

        for (FlowConstraints<T>* flowConstraints : vectorOfFlowConstraints) {
            cost.Add(flowConstraints->Evaluate(vertices), T(1));
        }

        for (EyeballConstraints<T>* eyeballConstraints : vectorOfEyeballConstraints) {
            cost.Add(eyeballConstraints->EvaluateEyeballConstraints(vertices), T(1), "eyeball");
        }

        return cost.CostToDiffData();
    };

    GaussNewtonSolver<T> solver;
    // LMSolver<T> solver;
    const T startEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    if (!solver.Solve(evaluationFunction, iterations)) {
        LOG_ERROR("could not solve optimization problem");
        return;
    }
    const T finalEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    LOG_INFO("energy changed from {} to {}", startEnergy, finalEnergy);
}

/**
 * RigLogic fitting using a bounded coordinate descent with L1 regularization, landmark constraints, and ICP constraints.
 */
template <class T>
void FitDataL1AndBounds(const Mesh<T>& topoplogy,
                        DeformationModelRigLogic<T>* deformationModelRigLogic,
                        LandmarkConstraints2D<T>* landmarkConstraints,
                        ICPConstraints<T>* icpConstraints,
                        // std::vector<DepthmapConstraints>& vectorOfDepthmapConstraints,
                        const std::vector<FlowConstraints<T>*>& vectorOfFlowConstraints,
                        int iterations,
                        T l1Reg)
{
    PROFILING_FUNCTION(PROFILING_COLOR_NAVY);

    if (icpConstraints) {
        icpConstraints->ClearPreviousCorrespondences();
    }

    Mesh<T> mesh = topoplogy;
    mesh.Triangulate();

    const std::vector<int> meshIndicesDefault = {0, deformationModelRigLogic->LeftEyeMeshIndex(), deformationModelRigLogic->RightEyeMeshIndex(),  deformationModelRigLogic->TeethMeshIndex() };
    const std::vector<int> meshIndices = landmarkConstraints ? landmarkConstraints->ExcludeInactiveMeshIndices(meshIndicesDefault) : std::vector<int>{0};

    //std::vector<VertexConstraints<float, 1, 1>> depthmapVertexConstraints(vectorOfDepthmapConstraints.size());

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {

        Cost<T> cost;

        const std::vector<DiffDataMatrix<T, 3, -1>> vectorOfVertices = deformationModelRigLogic->EvaluateVertices(context, /*lod=*/0, meshIndices, /*withRigid=*/true);
        const DiffDataMatrix<T, 3, -1> vertices = vectorOfVertices[0];
        mesh.SetVertices(vertices.Matrix());
        mesh.CalculateVertexNormals();

        cost.Add(deformationModelRigLogic->EvaluateModelConstraints(context), T(1));

        if (icpConstraints) {
            // calculate closest point constraints only when also evaluating the Jacobian, otherwise it
            // uses the previous targets. This ensures constant correspondences during line search
            cost.Add(icpConstraints->EvaluateICP(vertices, mesh.VertexNormals(), /*searchCorrespondences=*/bool(context)), T(1));
        }

        // for (size_t i = 0; i < vectorOfDepthmapConstraints.size(); ++i)
        // {
        //     DepthmapConstraints& depthmapConstraints = vectorOfDepthmapConstraints[i];
        //     Timer timer;
        //     if (context || depthmapVertexConstraints[i].NumberOfConstraints() == 0) {
        //         depthmapVertexConstraints[i].Clear();
        //         depthmapVertexConstraints[i].ResizeToFitAdditionalConstraints(vertices.Cols());
        //         depthmapConstraints.SetupDepthConstraints(Eigen::Transform<float, 3, Eigen::Affine>::Identity(), mesh.Vertices(), mesh.VertexNormals(), depthmapVertexConstraints[i]);
        //         // LOG_INFO("time to set up depth constraints: {}", timer.Current()); timer.Restart();
        //         // if (context) {
        //         //     const Eigen::Matrix<T, -1, -1, Eigen::RowMajor> denseVertexJacobian = *(vertices.Jacobian().AsSparseMatrix());
        //         //     LOG_INFO("time to make dense vertex jacobian: {}", timer.Current()); timer.Restart();
        //         //     const Eigen::Matrix<T, -1, -1, Eigen::RowMajor> newJacobian = depthmapVertexConstraints[i].EvaluateJacobian(denseVertexJacobian);
        //         //     LOG_INFO("time to calculate vertex constraint jacobian: {}", timer.Current()); timer.Restart();
        //         //     const Eigen::Matrix<T, -1, -1, Eigen::RowMajor> AtA1 = newJacobian.transpose() * newJacobian;
        //         //     LOG_INFO("time for AtA v1: {}", timer.Current()); timer.Restart();
        //         //     const Eigen::Matrix<T, -1, -1> AtA2 = newJacobian.transpose() * newJacobian;
        //         //     LOG_INFO("time for AtA v2: {}", timer.Current()); timer.Restart();
        //         // }
        //     }
        //     cost.Add(ApplyVertexConstraints(vertices, depthmapVertexConstraints[i]), T(1));
        //     LOG_INFO("time to apply vertex constraints: {}", timer.Current()); timer.Restart();
        // }

        if (landmarkConstraints) {
            cost.Add(landmarkConstraints->Evaluate(vertices, mesh.VertexNormals()), T(1));

            if (meshIndices[1] > -1 && meshIndices[2] > -1) {
                const DiffDataMatrix<T, 3, -1> leftEyeVertices = vectorOfVertices[1];
                const DiffDataMatrix<T, 3, -1> rightEyeVertices = vectorOfVertices[2];
                cost.Add(landmarkConstraints->EvaluateEyeConstraints(leftEyeVertices, rightEyeVertices), T(1));
            }

            if (meshIndices[3] > -1) {
                const DiffDataMatrix<T, 3, -1> teethVertices = vectorOfVertices[3];
                cost.Add(landmarkConstraints->EvaluateTeethConstraints(teethVertices), T(1));
            }
        }

        for (FlowConstraints<T>* flowConstraints : vectorOfFlowConstraints) {
            cost.Add(flowConstraints->Evaluate(vertices), T(1));
        }

        return cost.CostToDiffData();
    };

    BoundedCoordinateDescentSolver<T> solver;
    const T startEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    Context<T> context;
    if (!solver.Solve(evaluationFunction, context, iterations, deformationModelRigLogic->SolveControlVariable(), l1Reg)) {
        LOG_ERROR("could not solve optimization problem");
        return;
    }
    const T finalEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    LOG_INFO("energy changed from {} to {}", startEnergy, finalEnergy);
}


const std::vector<std::string>& FaceTrackingFittingModeNames()
{
    static std::vector<std::string> names = {"Coarse Align", "Rigid", "PCA", "PoseBasedSolve", "RigLogic", "Vertex Offsets"};
    return names;
}

FaceTracking::~FaceTracking() = default;

FaceTracking::FaceTracking():
    m_rigPoseBasedSolversData(new std::vector<rigposebasedsolver::RigPoseBasedSolverData>())
{
}


bool FaceTracking::HasPoseBasedSolverModel() const
{
    return m_rigPoseBasedSolversData->size() > 0;
}

const std::vector<rigposebasedsolver::RigPoseBasedSolverData>& FaceTracking::GetPoseBasedSolversData() const
{
    return *m_rigPoseBasedSolversData;
}

void FaceTracking::SetPoseBasedSolversData(const std::vector<rigposebasedsolver::RigPoseBasedSolverData>& solverData)
{
    *m_rigPoseBasedSolversData = solverData;
}

bool FaceTracking::Init(const TemplateDescription& templateDescription,
                        const std::string& configurationPath,
                        const std::string& controlsConfigurationPath)
{
    m_templateDescription = templateDescription;

    if (!m_templateDescription.HasVertexWeights("eyeball_interface_left")) {
        LOG_WARNING("No vertex weights \"eyeball_interface_left\", so eyeball constraints will not be used.");
    }
    if (!m_templateDescription.HasVertexWeights("eyeball_interface_right")) {
        LOG_WARNING("No vertex weights \"eyeball_interface_right\", so eyeball constraints will not be used.");
    }

    MeshLandmarks<float> meshLandmarks = templateDescription.GetMeshLandmarks();
    // merge mesh curves - hardcoded, should that be in a config file?
    // merge outer lips so that point-line constraints are not separated by the sub segments
    meshLandmarks.MergeCurves({"crv_lip_upper_outer_l", "crv_lip_philtrum_l", "crv_lip_upper_outer_r", "crv_lip_philtrum_r"}, "crv_lip_upper_outer", /*removePreviousCurves=*/true); // merge upper outer lip
    meshLandmarks.MergeCurves({"crv_lip_lower_outer_l", "crv_lip_lower_outer_r"}, "crv_lip_lower_outer", /*removePreviousCurves=*/true); // merge upper outer lip
    if (meshLandmarks.HasCurve("crv_nose_l") && meshLandmarks.HasCurve("crv_nose_r")) {
        meshLandmarks.MergeCurves({"crv_nose_l", "crv_nose_r"}, "crv_nose", /*removePreviousCurves=*/true); // merge nose
    }
    SetMeshLandmarks(meshLandmarks);

    // the curves of the landmarks that should be merged. it is the callers responsibility in SetLandmarks to make sure the curves have been merged
    m_curvesToMerge = {
        // merge the outer lip contour lines
        {"crv_lip_upper_outer", {"crv_lip_upper_outer_l", "crv_lip_philtrum_l", "crv_lip_upper_outer_r", "crv_lip_philtrum_r"}},
        {"crv_lip_lower_outer", {"crv_lip_lower_outer_l", "crv_lip_lower_outer_r"}},
        // merge the inner lip contour lines for zippering
        {"crv_lip_lower_inner", {"crv_lip_lower_inner_l", "crv_lip_lower_inner_r"}},
        {"crv_lip_upper_inner", {"crv_lip_upper_inner_l", "crv_lip_upper_inner_r"}},
        // merge nose and chin lines
        {"crv_nose", {"crv_nose_l", "crv_nose_r"}},
        {"crv_chin", {"crv_chin_l", "crv_chin_r"}}
    };

    // setup the deformation models
    m_defModelRigid = std::make_unique<DeformationModelRigid<float>>();
    m_defModelRigLogic = std::make_unique<DeformationModelRigLogic<float>>();
    m_defModelVertex = std::make_unique<DeformationModelVertex<float>>();

    // get the configurations per model
    const Configuration icpConfiguration = ICPConstraints<float>().GetConfiguration();
    const Configuration landmarkConfiguration = LandmarkConstraints2D<float>().GetConfiguration();
    const Configuration flowConfiguration = {ConfigNameFlow, {{"temporal flow", ConfigurationParameter(0.f, 0.f, 1.f)},
                                                              {"model flow", ConfigurationParameter(0.f, 0.f, 1.f)},
                                                              {"uv flow", ConfigurationParameter(0.f, 0.f, 1.f)}}};
    const Configuration general = {ConfigNameGeneral, {{"iterations", ConfigurationParameter(10, 1, 20)}}};
    const Configuration generalPCA = {ConfigNameGeneral, {{"iterations", ConfigurationParameter(5, 1, 20)},
                                                          {"regularization", ConfigurationParameter((1.f), (0.f), (100.f))},
                                                          {"optimizePose", ConfigurationParameter(true)},
                                                          {"velocity", ConfigurationParameter((0.f), (0.f), (1.f))},
                                                          {"acceleration", ConfigurationParameter((0.f), (0.f), (1.f))}}};
    const Configuration generalRigLogic = {ConfigNameGeneral, {{"iterations", ConfigurationParameter(8, 1, 20)}, {"l1Regularization", ConfigurationParameter((1.f), (0.f), (100.f))}}};
    const Configuration generalFine = {ConfigNameGeneral, {{"iterations", ConfigurationParameter(5, 1, 20)}, {"eyeball", ConfigurationParameter(1.0f, 0.0f, 10.0f)}}};

    const Configuration defModelRigidConfig = m_defModelRigid->GetConfiguration();
    const Configuration defModelRigLogicConfig = m_defModelRigLogic->GetConfiguration();
    const Configuration defModelVertexConfig = m_defModelVertex->GetConfiguration();

    Configuration rigidConfiguration{"rigid", {}};
    rigidConfiguration.AddConfigurations({general, icpConfiguration, landmarkConfiguration, flowConfiguration, defModelRigidConfig});
    rigidConfiguration[landmarkConfiguration.Name()]["innerLipWeight"].Set(0.0f);
    rigidConfiguration[landmarkConfiguration.Name()]["eyesWeight"].Set(0.0f);
    rigidConfiguration[landmarkConfiguration.Name()]["teethWeight"].Set(0.0f);
    m_fittingConfigurations.insert_or_assign(FaceTrackingFittingMode::RIGID, rigidConfiguration);

    Configuration pcaConfiguration{"pca", {}};
    pcaConfiguration.AddConfigurations({generalPCA, icpConfiguration, landmarkConfiguration, flowConfiguration});
    pcaConfiguration[ConfigNameFlow]["temporal flow"].Set(0.0f);
    m_fittingConfigurations.insert_or_assign(FaceTrackingFittingMode::PCA, pcaConfiguration);

    Configuration riglogicConfiguration{"riglogic", {}};
    riglogicConfiguration.AddConfigurations({generalRigLogic, icpConfiguration, landmarkConfiguration, flowConfiguration, defModelRigLogicConfig});
    riglogicConfiguration[flowConfiguration.Name()]["temporal flow"].Set(0.005f);
    m_fittingConfigurations.insert_or_assign(FaceTrackingFittingMode::RIGLOGIC, riglogicConfiguration);

    Configuration perVertexConfiguration{"pervertex", {}};
    perVertexConfiguration.AddConfigurations({generalFine, icpConfiguration, landmarkConfiguration, flowConfiguration, defModelVertexConfig});
    perVertexConfiguration[ConfigNameFlow]["temporal flow"].Set(0.005f);
    perVertexConfiguration[ConfigNameFlow]["uv flow"].Set(0.005f);
    perVertexConfiguration[landmarkConfiguration.Name()]["eyesWeight"].Set(0.0f);
    perVertexConfiguration[landmarkConfiguration.Name()]["teethWeight"].Set(0.0f);
    perVertexConfiguration[defModelVertexConfig.Name()]["projectiveStrain"].Set(0.0f);
    perVertexConfiguration[defModelVertexConfig.Name()]["quadraticBending"].Set(0.0f);
    perVertexConfiguration[defModelVertexConfig.Name()]["vertexLaplacian"].Set(1.0f);
    m_fittingConfigurations.insert_or_assign(FaceTrackingFittingMode::FINE, perVertexConfiguration);

    m_fittingConfigurations.insert_or_assign(FaceTrackingFittingMode::COARSE, Configuration{"coarse", {}});
    m_fittingConfigurations.insert_or_assign(FaceTrackingFittingMode::POSEBASEDSOLVE, Configuration{"posebased", {}});

    if (!configurationPath.empty()) {
        LoadConfiguration(configurationPath);
    }

    if (!controlsConfigurationPath.empty()) {
        LoadControlsConfiguration(controlsConfigurationPath);
    }

    // set up all user defined landmark and curve weights that are not yet set by the configuration
    SetupDefaultUserDefinedLandmarkAndCurveWeights();

    // clear all input data
    ResetInputData();

    return true;
}

bool FaceTracking::LoadTrackingRig(const std::string& dnaFilename, const std::string& solveDefinitionFilename)
{
    std::shared_ptr<const RigLogicDNAResource> dnaResource = RigLogicDNAResource::LoadDNA(dnaFilename, /*retain=*/false);
    if (!dnaResource) {
        LOG_ERROR("failed to open dnafile {}", dnaFilename);
        return false;
    }

    return LoadTrackingRig(dnaResource->Stream(), solveDefinitionFilename);
}

bool FaceTracking::LoadTrackingRig(dna::StreamReader* dnaStream, const std::string& solveDefinitionFilename)
{
    if (m_templateDescription.Topology().NumVertices() == 0) {
        LOG_ERROR("first initialize the tracking module before loading a rig");
        return false;
    }

    if (!m_trackingRig.LoadRig(dnaStream, solveDefinitionFilename)) {
        LOG_ERROR("could not load tracking rig");
        return false;
    }
    m_trackingRigState = std::make_unique<TrackingRigState<float>>(m_trackingRig);

    if (!m_trackingRig.VerifyTopology(m_templateDescription.Topology())) {
        LOG_ERROR("the tracking rig is not compatible with the base mesh topology");
        return false;
    }

    m_defModelRigLogic->SetRig(m_trackingRig.GetRig());
    m_defaultGuiControlValues = m_defModelRigLogic->GuiControls();
    m_defModelVertex->SetMeshTopology(m_trackingRig.GetBaseMeshTriangulated());

    // create a default state
    Affine<float, 3, 3> rot;
    rot.SetLinear(EulerX(float(CARBON_PI)));
    m_currentState.m_mesh2scan = rot; // rotate by 180 degrees by default to be visible by the camera
    m_currentState.m_guiControls = m_trackingRig.GetDefaultGuiControlValues();
    m_currentState.m_rigVertices = m_trackingRig.GetBaseMesh().Vertices();
    m_currentState.m_vertexOffsets = Eigen::Matrix<float, 3, -1>::Zero(3, m_trackingRig.GetBaseMesh().NumVertices());
    m_currentState.m_vertices = m_trackingRig.GetBaseMesh().Vertices();
    UpdateCurrentMesh();

    // clear all input data
    ResetInputData();

    // update fixed rig controls
    if (m_fixedRigLogicGuiControls.size() > 0 && m_trackingRig.GetNumSolveControlSets() > 0) {
        const std::vector<std::string>& controlNames = m_trackingRig.GetRigSolveControlNames(m_trackingRig.GetNumSolveControlSets() - 1);
        std::vector<bool> optimizeControlValues(controlNames.size(), true);
        for (const std::string& guiControlName : m_fixedRigLogicGuiControls) {
            const auto& it = std::find(controlNames.begin(), controlNames.end(), guiControlName);
            if (it != controlNames.end()) {
                optimizeControlValues[std::distance(controlNames.begin(), it)] = false;
            }
        }
        SetRigSolveControlsToOptimize(m_trackingRig.GetNumSolveControlSets() - 1, optimizeControlValues);
    }

    return true;
}

bool FaceTracking::LoadPCARig(const std::string& pcaFilename)
{
    if (!m_pcaFaceTracking.LoadPCARig(pcaFilename)) {
        return false;
    }

    m_pcaFaceTracking.LoadFaceMeshLandmarks(m_meshLandmarks);
    m_pcaFaceTracking.LoadEyeLeftMeshLandmarks( GetTemplateDescription().GetEyeLeftMeshLandmarks());
    m_pcaFaceTracking.LoadEyeRightMeshLandmarks( GetTemplateDescription().GetEyeRightMeshLandmarks());
    m_pcaFaceTracking.LoadTeethMeshLandmarks( GetTemplateDescription().GetTeethMeshLandmarks());

    const std::vector<std::string> meshNames = { "head_lod0_mesh", "eyeLeft_lod0_mesh", "eyeRight_lod0_mesh", "teeth_lod0_mesh"};
    for (int k = 0; k < 4; ++k) {
        m_meshIndexToPcaRig[m_trackingRig.GetRigGeometry()->GetMeshIndex(meshNames[k])] = k;
    }

    return true;
}

bool FaceTracking::LoadPCARig(dna::StreamReader* dnaStream)
{
    if (!m_pcaFaceTracking.LoadPCARig(dnaStream)) {
        return false;
    }

    m_pcaFaceTracking.LoadFaceMeshLandmarks(m_meshLandmarks);
    m_pcaFaceTracking.LoadEyeLeftMeshLandmarks( GetTemplateDescription().GetEyeLeftMeshLandmarks());
    m_pcaFaceTracking.LoadEyeRightMeshLandmarks( GetTemplateDescription().GetEyeRightMeshLandmarks());
    m_pcaFaceTracking.LoadTeethMeshLandmarks( GetTemplateDescription().GetTeethMeshLandmarks());

    const std::vector<std::string> meshNames = { "head_lod0_mesh", "eyeLeft_lod0_mesh", "eyeRight_lod0_mesh", "teeth_lod0_mesh" };
    for (int k = 0; k < 4; ++k) {
        m_meshIndexToPcaRig[m_trackingRig.GetRigGeometry()->GetMeshIndex(meshNames[k])] = k;
    }

    return true;
}

bool FaceTracking::SavePCARig(dna::StreamWriter* dnaStream) const
{
    return m_pcaFaceTracking.SavePCARig(dnaStream);
}

void FaceTracking::SavePCARigAsNpy(const std::string& filename) const
{
    return m_pcaFaceTracking.SavePCARigAsNpy(filename);
}

bool FaceTracking::LoadPoseBasedSolversData(const std::vector<std::string>& poseBasedSolverFilenames)
{
    m_rigPoseBasedSolversData->resize(poseBasedSolverFilenames.size());
    unsigned i = 0;
    for (const auto& filename : poseBasedSolverFilenames)
    {
        if (std::filesystem::exists(filename))
        {
            try
            {
                std::ifstream ifs(filename, std::ios_base::binary);
                deserialize((*m_rigPoseBasedSolversData)[i], ifs);
                ifs.close();
                i++;
            }
            catch (const std::exception& e)
            {
                LOG_ERROR("error loading pose-based solver file {}: {}", filename, e.what());
                m_rigPoseBasedSolversData->clear();
                return false;
            }
        }
        else
        {
            LOG_ERROR("pose-based solver file {} does not exist", filename);
            return false;
        }
    }

    return true;
}


void FaceTracking::SetCameraSetup(const MultiCameraSetup<float>& cameraSetup)
{
    m_cameraSetup = cameraSetup;
    if (m_cameraSetup.GetCameras().size() > 0 &&
        m_cameraSetup.GetCameras().size() != m_cameraSetup.GetCameraRanges().size() &&
        m_trackingRig.GetBaseMesh().NumVertices() > 0) {
        // make sure there is a range defined for each camera if there is not yet one
        EstimateCameraRangesUsingCurrentMesh();
    }

    // clear all input data
    ResetInputData();
}

void FaceTracking::SetMeshLandmarks(const MeshLandmarks<float>& meshLandmarks)
{
    m_meshLandmarks = meshLandmarks;
    m_pcaFaceTracking.LoadFaceMeshLandmarks(m_meshLandmarks);

    SetupDefaultUserDefinedLandmarkAndCurveWeights();
    ResetDataConstraints();
}

void FaceTracking::LoadConfiguration(const std::string& filename)
{
    if (!filename.empty()) {
        const std::string configData = ReadFile(filename);
        epic::carbon::JsonElement j_config = epic::carbon::ReadJson(configData);
        LoadConfiguration(j_config);
    }
}

void FaceTracking::LoadConfiguration(const carbon::JsonElement& j_config)
{
    std::vector<std::string> unspecifiedKeys;
    std::vector<std::string> unknownKeys;
    m_fittingConfigurations.find(FaceTrackingFittingMode::RIGID)->second.FromJson(j_config["rigid"], unspecifiedKeys, unknownKeys);
    if (j_config.Contains("pca")) {
        m_fittingConfigurations.find(FaceTrackingFittingMode::PCA)->second.FromJson(j_config["pca"], unspecifiedKeys, unknownKeys);
    }
    m_fittingConfigurations.find(FaceTrackingFittingMode::RIGLOGIC)->second.FromJson(j_config["riglogic"], unspecifiedKeys, unknownKeys);
    m_fittingConfigurations.find(FaceTrackingFittingMode::FINE)->second.FromJson(j_config["pervertex"], unspecifiedKeys, unknownKeys);
    for (const std::string& key : unspecifiedKeys) {
        LOG_WARNING("config is not specifying {}", key);
    }
    for (const std::string& key : unknownKeys) {
        LOG_WARNING("config contains unknown key {}", key);
    }

    if (j_config.Contains("landmark and curve weights")) {
        auto userDefinedLandmarkAndCurveWeights = j_config["landmark and curve weights"].Get<std::map<std::string, float>>();
        userDefinedLandmarkAndCurveWeights.merge(m_userDefinedLandmarkAndCurveWeights);
        std::swap(userDefinedLandmarkAndCurveWeights, m_userDefinedLandmarkAndCurveWeights);
    } else {
        LOG_WARNING("json configuration is missing the landmark and curve weights");
    }

    if (j_config.Contains("symmetry")) {
        if (j_config["symmetry"].IsArray() && j_config["symmetry"].Size() > 0 && j_config["symmetry"][0].IsArray() && j_config["symmetry"][0].Size() == 2) {
            LOG_WARNING("specify control symmetries with a weight per symmetric pair e.g. [\"name1\", \"name2\", 1.0]");
            std::vector<std::pair<std::string, std::string>> symmetricControls = j_config["symmetry"].Get<std::vector<std::pair<std::string, std::string>>>();
            m_symmetricRigLogicGuiControls.clear();
            for (const auto& [name1, name2] : symmetricControls) {
                m_symmetricRigLogicGuiControls.push_back({name1, name2, float(1)});
            }
        } else {
            m_symmetricRigLogicGuiControls = j_config["symmetry"].Get<std::vector<std::tuple<std::string, std::string, float>>>();
        }
    } else {
        LOG_WARNING("json configuration is missing the symmetry information for the rig gui controls");
    }

    if (j_config.Contains("fixed_gui_controls")) {
        LoadControlsConfiguration(j_config["fixed_gui_controls"]);
    } else {
        LOG_WARNING("json configuration is missing which gui controls are fixed");
    }
}

void FaceTracking::SaveConfiguration(const std::string& filename) const
{
    if (!filename.empty()) {
        epic::carbon::JsonElement configuration(epic::carbon::JsonElement::JsonType::Object);
        SaveConfiguration(configuration);
        WriteFile(filename, epic::carbon::WriteJson(configuration, 1));
    }
}

void FaceTracking::SaveConfiguration(carbon::JsonElement& configuration) const
{
    configuration.Insert("rigid", GetFittingConfiguration(FaceTrackingFittingMode::RIGID).ToJson());
    configuration.Insert("pca", GetFittingConfiguration(FaceTrackingFittingMode::PCA).ToJson());
    configuration.Insert("riglogic", GetFittingConfiguration(FaceTrackingFittingMode::RIGLOGIC).ToJson());
    configuration.Insert("pervertex", GetFittingConfiguration(FaceTrackingFittingMode::FINE).ToJson());
    configuration.Insert("landmark and curve weights", epic::carbon::JsonElement(m_userDefinedLandmarkAndCurveWeights));
    configuration.Insert("symmetry", epic::carbon::JsonElement(m_symmetricRigLogicGuiControls));
    configuration.Insert("fixed_gui_controls", SaveControlsConfiguration());
}

epic::carbon::JsonElement FaceTracking::SaveControlsConfiguration() const
{
    return epic::carbon::JsonElement(m_fixedRigLogicGuiControls);
}

void FaceTracking::LoadControlsConfiguration(const epic::carbon::JsonElement& json)
{
    m_fixedRigLogicGuiControls = json.Get<std::vector<std::string>>();

    if (m_trackingRig.GetNumSolveControlSets() > 0) {
        const std::vector<std::string>& controlNames = m_trackingRig.GetRigSolveControlNames(m_trackingRig.GetNumSolveControlSets() - 1);
        std::vector<bool> optimizeControlValues(controlNames.size(), true);
        for (const std::string& guiControlName : m_fixedRigLogicGuiControls) {
            const auto& it = std::find(controlNames.begin(), controlNames.end(), guiControlName);
            if (it != controlNames.end()) {
                optimizeControlValues[std::distance(controlNames.begin(), it)] = false;
            }
        }
        SetRigSolveControlsToOptimize(m_trackingRig.GetNumSolveControlSets() - 1, optimizeControlValues);
    }
}

void FaceTracking::LoadControlsConfiguration(const std::string& filename)
{
    LoadControlsConfiguration(epic::carbon::ReadJson(ReadFile(filename)));
}

void FaceTracking::SaveControlsConfiguration(const std::string& filename) const
{
    WriteFile(filename, epic::carbon::WriteJson(SaveControlsConfiguration(), /*tabs=*/1));
}

void FaceTracking::SetUserDefinedLandmarkAndCurveWeights(const std::map<std::string, float>& userDefinedLandmarkAndCurveWeights)
{
    m_userDefinedLandmarkAndCurveWeights = userDefinedLandmarkAndCurveWeights;
    ResetDataConstraints();
}

void FaceTracking::SetupDefaultUserDefinedLandmarkAndCurveWeights()
{
    for (const auto& meshMarkers : { m_meshLandmarks,
                                     GetTemplateDescription().GetEyeLeftMeshLandmarks(),
                                     GetTemplateDescription().GetEyeRightMeshLandmarks(),
                                     GetTemplateDescription().GetTeethMeshLandmarks() }) {
        for (const auto& [name, _] : meshMarkers.LandmarksBarycentricCoordinates()) {
            if (m_userDefinedLandmarkAndCurveWeights.find(name) == m_userDefinedLandmarkAndCurveWeights.end()) {
                m_userDefinedLandmarkAndCurveWeights[name] = 1.0f;
            }
        }
        for (const auto& [name, _] : meshMarkers.MeshCurvesBarycentricCoordinates()) {
            if (m_userDefinedLandmarkAndCurveWeights.find(name) == m_userDefinedLandmarkAndCurveWeights.end()) {
                m_userDefinedLandmarkAndCurveWeights[name] = 1.0f;
            }
        }
        for (const auto& [name, _] : meshMarkers.Contours()) {
            if (m_userDefinedLandmarkAndCurveWeights.find(name) == m_userDefinedLandmarkAndCurveWeights.end()) {
                m_userDefinedLandmarkAndCurveWeights[name] = 1.0f;
            }
        }
    }
}

void FaceTracking::SetFaceTrackingState(const FaceTrackingState& state)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);
    if (state.m_guiControls.size() != m_trackingRig.GetDefaultGuiControlValues().size()) {
        CARBON_CRITICAL("invalid size for gui controls: {}, but expected {}", state.m_guiControls.size(), m_trackingRig.GetDefaultGuiControlValues().size());
    }
    if (int(state.m_rigVertices.cols()) != m_trackingRig.GetBaseMesh().NumVertices()) {
        CARBON_CRITICAL("invalid size for rig vertices");
    }
    if (int(state.m_vertexOffsets.cols()) != m_trackingRig.GetBaseMesh().NumVertices()) {
        CARBON_CRITICAL("invalid size for vertex offsets");
    }
    if (int(state.m_vertices.cols()) != m_trackingRig.GetBaseMesh().NumVertices()) {
        CARBON_CRITICAL("invalid size for vertices");
    }
    m_currentState = state;
    UpdateSolveControlSetsData();
    UpdateCurrentMesh();
    ResetDataConstraints();
}


void FaceTracking::SetLandmarks(const std::map<std::string, std::shared_ptr<const LandmarkInstance<float, 2>>>& landmarkData)
{
    if (m_currentFrameData.m_landmarkData != landmarkData) {
        // LOG_INFO("=> landmarks are new {}", landmarkData.size());
        m_currentFrameData.m_landmarkData = landmarkData;
        m_currentFrameData.m_triangulatedLandmarks.clear();
        m_landmarkConstraints.reset();
        m_constraintsDebugInfo.clear();
    }
}

void FaceTracking::SetScan(std::shared_ptr<const Mesh<float>> scan)
{
    if (scan != m_currentFrameData.m_scan) {
        // LOG_INFO("=> scan is new");
        m_currentFrameData.m_scan = scan;
        m_currentFrameData.m_triangulatedLandmarks.clear();
        m_icpConstraints.reset();
        m_constraintsDebugInfo.clear();
    }
}

std::shared_ptr<const Mesh<float> > FaceTracking::CurrentScan()
{
    return m_currentFrameData.m_scan;
}

void FaceTracking::SetDepthmaps(const std::vector<std::shared_ptr<const DepthmapData<float>>>& depthmapData)
{
    if (m_currentFrameData.m_depthmapData != depthmapData) {
        // LOG_INFO("=> depthmap is new {}", depthmapData.size());
        m_currentFrameData.m_depthmapData = depthmapData;
        m_currentFrameData.m_triangulatedLandmarks.clear();
        m_icpConstraints.reset();
        m_constraintsDebugInfo.clear();
    }
}

void FaceTracking::SetTemporalFlow(const std::map<std::string, std::shared_ptr<const MeshFlowData>>& flowData)
{
    if (flowData != m_currentFrameData.m_temporalFlow) {
        // LOG_INFO("=> temporal flow is new {}", flowData.size());
        m_currentFrameData.m_temporalFlow = flowData;
        m_temporalFlowConstraints.reset();
        m_constraintsDebugInfo.clear();
    }
}

void FaceTracking::SetModelBaseFlow(const std::map<std::string, std::shared_ptr<const MeshFlowData>>& flowData)
{
    if (flowData != m_currentFrameData.m_modelFlow) {
        // LOG_INFO("=> model flow is new {}", flowData.size());
        m_currentFrameData.m_modelFlow = flowData;
        m_modelFlowConstraints.reset();
        m_constraintsDebugInfo.clear();
    }
}

void FaceTracking::SetUVFlow(const std::map<std::string, std::shared_ptr<const MeshFlowData>>& flowData)
{
    if (flowData != m_currentFrameData.m_uvFlow) {
        // LOG_INFO("=> uv flow is new {}", flowData.size());
        m_currentFrameData.m_uvFlow = flowData;
        m_uvFlowConstraints.reset();
        m_constraintsDebugInfo.clear();
    }
}


std::map<std::string, Eigen::Vector3f> FaceTracking::EvaluateMeshLandmarks(const Eigen::Matrix<float, 3, -1>& vertices) const {
    std::map<std::string, Eigen::Vector3f> meshPositions;
    for (const auto& [landmarkName, bc] : m_meshLandmarks.LandmarksBarycentricCoordinates()) {
        meshPositions.emplace(landmarkName, bc.Evaluate<3>(vertices));
    }
    return meshPositions;
}

const std::map<std::string, Eigen::Vector3f>& FaceTracking::TriangulatedLandmarks() {

    if (m_cameraSetup.GetCameras().empty()) {
        CARBON_CRITICAL("cameras are not set up");
    }

    if (m_currentFrameData.m_triangulatedLandmarks.size() == 0) {
        if (m_currentFrameData.m_landmarkData.size() > 1) {
            std::map<std::string, LandmarkInstance<float, 2>> landmarkInstances;
            for (const auto& [cameraName, landmarkInstancePtr] : m_currentFrameData.m_landmarkData) {
                landmarkInstances[cameraName] = *landmarkInstancePtr;
            }
            m_currentFrameData.m_triangulatedLandmarks = TriangulateLandmarks(m_cameraSetup, landmarkInstances);
        } else if (m_currentFrameData.m_landmarkData.size() == 1 && CurrentScan()) {
            const auto& [cameraName, landmarkInstancePtr] = *(m_currentFrameData.m_landmarkData.begin());
            m_currentFrameData.m_triangulatedLandmarks = TriangulateLandmarksViaRayCasting(m_cameraSetup.GetCamera(cameraName), *landmarkInstancePtr, *CurrentScan());
        } else if (m_currentFrameData.m_landmarkData.size() == 1 && m_currentFrameData.m_depthmapData.size() > 0) {
            const auto& [cameraName, landmarkInstancePtr] = *(m_currentFrameData.m_landmarkData.begin());
            if (m_cameraSetup.GetCamera(cameraName).Origin() == m_currentFrameData.m_depthmapData[0]->camera.Origin()) {
                m_currentFrameData.m_triangulatedLandmarks = TriangulateLandmarksViaDepthmap(m_cameraSetup.GetCamera(cameraName), *landmarkInstancePtr, m_currentFrameData.m_depthmapData[0]->camera, m_currentFrameData.m_depthmapData[0]->depthAndNormals);
            } else {
                CARBON_CRITICAL("cannot get triangulated landmarks from the depthmap");
            }
        } else {
            CARBON_CRITICAL("no cameras with landmarks to estimate 3d landmark positions");
        }
    }

    return m_currentFrameData.m_triangulatedLandmarks;
}

void FaceTracking::EstimateCameraRangesUsingCurrentMesh(float rangeScaling)
{
    m_cameraSetup.SetCameraRanges(EstimateCameraRangesByProjection(m_cameraSetup.GetCamerasAsVector(), *CurrentMesh(FaceTrackingOutputMode::BEST), CurrentRigid(), rangeScaling));
}

void FaceTracking::EstimateRigidUsingLandmarks()
{
    const std::map<std::string, Eigen::Vector3f> reconstructedLandmarkPositions = TriangulatedLandmarks();
    const std::map<std::string, Eigen::Vector3f> meshPositions = EvaluateMeshLandmarks(CurrentHeadVertices(FaceTrackingOutputMode::BEST));
    std::vector<Eigen::Vector3f> srcPts;
    std::vector<Eigen::Vector3f> targetPts;
    for (const auto& [landmarkName, srcPt] : meshPositions) {
        auto targetIt = reconstructedLandmarkPositions.find(landmarkName);
        if (targetIt != reconstructedLandmarkPositions.end()) {
            srcPts.push_back(srcPt);
            targetPts.push_back(targetIt->second);
        }
    }
    if (srcPts.size() < 3) {
        CARBON_CRITICAL("cannot estimate the rigid alignment as there are not sufficient triangulated landmarks (only {})", srcPts.size());
    }

    Eigen::Matrix<float, 3, -1> srcPtsMap = Eigen::Map<const Eigen::Matrix3Xf>((const float*)srcPts.data(), 3, srcPts.size());
    Eigen::Matrix<float, 3, -1> targetPtsMap = Eigen::Map<const Eigen::Matrix3Xf>((const float*)targetPts.data(), 3, targetPts.size());
    m_currentState.m_mesh2scan = Procrustes<float, 3>::AlignRigid(srcPtsMap, targetPtsMap);
}

void FaceTracking::ScaleCamerasUsingLandmarks() {
    const std::map<std::string, Eigen::Vector3f> reconstructedLandmarkPositions = TriangulatedLandmarks();
    const std::map<std::string, Eigen::Vector3f> meshPositions = EvaluateMeshLandmarks(CurrentHeadVertices(FaceTrackingOutputMode::BEST));
    std::vector<Eigen::Vector3f> srcPts;
    std::vector<Eigen::Vector3f> targetPts;
    for (const auto& [landmarkName, srcPt] : reconstructedLandmarkPositions) {
        auto targetIt = meshPositions.find(landmarkName);
        if (targetIt != meshPositions.end()) {
            srcPts.push_back(srcPt);
            targetPts.push_back(targetIt->second);
        }
    }
    if (srcPts.size() < 3) {
        CARBON_CRITICAL("cannot estimate the rigid alignment as there are not sufficient triangulated landmarks (only {})", srcPts.size());
    }

    // procrustes of the triangulated 3D landmarks to the mesh landmarks
    Eigen::Matrix<float, 3, -1> srcPtsMap = Eigen::Map<const Eigen::Matrix3Xf>((const float*)srcPts.data(), 3, srcPts.size());
    Eigen::Matrix<float, 3, -1> targetPtsMap = Eigen::Map<const Eigen::Matrix3Xf>((const float*)targetPts.data(), 3, targetPts.size());
    auto [scale, _] = Procrustes<float, 3>::AlignRigidAndScale(srcPtsMap, targetPtsMap);

    ScaleCameras(scale);
}

void FaceTracking::ScaleCamerasUsingFitting()
{
    SetupDataConstraints();

    if (m_icpConstraints) {
        m_icpConstraints->SetConfiguration(GetFittingConfiguration(FaceTrackingFittingMode::RIGID)[ConfigNameIcp]);
        m_icpConstraints->ClearPreviousCorrespondences();
        // for fitting the scale we always use only the face area
        m_icpConstraints->SetCorrespondenceSearchVertexWeights(GetTemplateDescription().GetVertexWeights("icp_mask"));
    }

    DeformationModelRigidScale<float> defModelRigidScale;
    defModelRigidScale.SetVertices(CurrentHeadVertices(FaceTrackingOutputMode::BEST));
    defModelRigidScale.SetRigidTransformation(CurrentRigid(), /*preScale=*/false);
    FitData<float>(m_trackingRig.GetBaseMeshTriangulated(), &defModelRigidScale, nullptr, m_icpConstraints.get(), {}, {}, 20);

    m_currentState.m_mesh2scan = defModelRigidScale.RigidTransformation(/*preScale=*/false);
    ScaleCameras(1.0f / defModelRigidScale.Scale());
}

void FaceTracking::ScaleCameras(float scale)
{
    m_cameraSetup.ScaleCameras(scale);
    LOG_INFO("scaling cameras by {} resulting in a total scale of {}", scale, m_cameraSetup.GetScale());

    ResetInputData();
}

Eigen::Matrix<float, 3, -1> FaceTracking::HeadVertices(const FaceTrackingState& state, FaceTrackingOutputMode outputMode) const
{
    // update the output mode based on what exists
    if (outputMode == FaceTrackingOutputMode::POST && !state.m_postProcessedGuiControls) {
        outputMode = FaceTrackingOutputMode::BEST;
    }

    switch (outputMode) {
        case FaceTrackingOutputMode::NEUTRAL: return m_trackingRig.GetBaseMesh().Vertices();
        case FaceTrackingOutputMode::RIGLOGIC: return state.m_rigVertices;
        case FaceTrackingOutputMode::FINE: return  state.m_vertices;
        case FaceTrackingOutputMode::BEST: return  state.m_vertices;
        case FaceTrackingOutputMode::POST: {
            if (state.m_postProcessedGuiControls) {
                return m_trackingRig.EvaluateVertices(*state.m_postProcessedGuiControls, /*lod=*/0, 0);
            } else {
                return  state.m_vertices;
            }
        }
        default: CARBON_CRITICAL("mesh output mode missing");
    }
}

Eigen::Matrix<float, 3, -1> FaceTracking::CurrentHeadVertices(FaceTrackingOutputMode outputMode) const
{
    return HeadVertices(m_currentState, outputMode);
}

Eigen::Matrix<float, 3, -1> FaceTracking::MeshVertices(const FaceTrackingState& state, FaceTrackingOutputMode outputMode, int meshIndex) const
{
    if (meshIndex == 0) return HeadVertices(state, outputMode);

    // update the output mode based on what exists
    if (outputMode == FaceTrackingOutputMode::POST && !state.m_postProcessedGuiControls) {
        outputMode = FaceTrackingOutputMode::BEST;
    }

    switch (outputMode) {
        case FaceTrackingOutputMode::NEUTRAL: return m_trackingRig.GetRigGeometry()->GetMesh(meshIndex).Vertices();
        case FaceTrackingOutputMode::RIGLOGIC:
        case FaceTrackingOutputMode::FINE:
        case FaceTrackingOutputMode::BEST: {
            const auto it = m_meshIndexToPcaRig.find(meshIndex);
            if (state.m_pcaCoeffs.size() > 0 && it != m_meshIndexToPcaRig.end()) {
                const int k = it->second;
                const auto evaluationMode = rt::LinearVertexModel<float>::EvaluationMode::STATIC;
                switch (k) {
                    default:
                    case 0: return m_pcaFaceTracking.PCARigToMesh().Transform(m_pcaFaceTracking.GetPCARig().facePCA.EvaluateLinearized(state.m_pcaCoeffs, evaluationMode));
                    case 3: return m_pcaFaceTracking.PCARigToMesh().Transform(m_pcaFaceTracking.GetPCARig().teethPCA.EvaluateLinearized(state.m_pcaCoeffs, evaluationMode));
                    // case 1: return m_pcaFaceTracking.PCARigToMesh().Transform(m_pcaFaceTracking.GetPCARig().eyeLeftPCA.Evaluate(state.m_pcaCoeffs, evaluationMode));
                    // case 2: return m_pcaFaceTracking.PCARigToMesh().Transform(m_pcaFaceTracking.GetPCARig().eyeRightPCA.Evaluate(state.m_pcaCoeffs, evaluationMode));
                    case 1: return m_pcaFaceTracking.PCARigToMesh().Transform(m_pcaFaceTracking.GetPCARig().eyeLeftTransformPCA.EvaluateVertices(state.m_pcaCoeffs, evaluationMode));
                    case 2: return m_pcaFaceTracking.PCARigToMesh().Transform(m_pcaFaceTracking.GetPCARig().eyeRightTransformPCA.EvaluateVertices(state.m_pcaCoeffs, evaluationMode));
                }
            }
            return m_trackingRig.EvaluateVertices(state.m_guiControls, /*lod=*/0, meshIndex);
        }
        case FaceTrackingOutputMode::POST: return m_trackingRig.EvaluateVertices(*state.m_postProcessedGuiControls, /*lod=*/0, meshIndex);
        default: CARBON_CRITICAL("mesh output mode missing");
    }
}

Eigen::Matrix<float, 3, -1> FaceTracking::CurrentMeshVertices(FaceTrackingOutputMode outputMode, int meshIndex) const
{
    return MeshVertices(m_currentState, outputMode, meshIndex);
}

std::shared_ptr<const Mesh<float> > FaceTracking::CurrentMesh(FaceTrackingOutputMode outputMode) {
    auto it = m_currentMeshes.find(outputMode);
    if (it == m_currentMeshes.end()) {
        PolyAllocator<Mesh<float>> polyAllocator{ MEM_RESOURCE };
        std::shared_ptr<Mesh<float>> meshPtr = std::allocate_shared<Mesh<float>>(polyAllocator, m_trackingRig.GetBaseMeshTriangulated());
        meshPtr->SetVertices(CurrentHeadVertices(outputMode));
        meshPtr->CalculateVertexNormals();
        m_currentMeshes[outputMode] = meshPtr;
        return meshPtr;
    } else {
        return it->second;
    }
}

void FaceTracking::UpdateCurrentMesh()
{
    m_currentMeshes.clear();
}

const Affine<float, 3, 3>& FaceTracking::CurrentRigid() const {
    return m_currentState.m_mesh2scan;
}

void FaceTracking::ResetInputData()
{
    m_currentFrameData.Clear();
    m_previousStates.clear();
    ResetDataConstraints();
}

void FaceTracking::ResetDataConstraints()
{
    m_landmarkConstraints.reset();
    m_icpConstraints.reset();
    m_temporalFlowConstraints.reset();
    m_modelFlowConstraints.reset();
    m_uvFlowConstraints.reset();
    m_constraintsDebugInfo.clear();
}

void FaceTracking::SetupDataConstraints()
{
    SetupICPDataConstraints();
    SetupLandmarkDataConstraints();
    SetupTemporalFlowDataConstraints();
    SetupModelFlowDataConstraints();
    SetupUVFlowDataConstraints();
    SetupEyeballConstraints();
}

void FaceTracking::SetupICPDataConstraints()
{
    if (!m_icpConstraints) {
        PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);
        if (CurrentScan()) {
            // ICP mesh constraints
            m_icpConstraints = std::make_unique<ICPConstraints<float>>();
            m_icpConstraints->SetTargetMesh(*CurrentScan());
        } else {
            // ICP depthmap constraints
            if (m_currentFrameData.m_depthmapData.size() > 0) {
                m_icpConstraints = std::make_unique<ICPConstraints<float>>();
                for (const auto& depthmapData : m_currentFrameData.m_depthmapData) {
                    m_icpConstraints->AddTargetDepthAndNormals(depthmapData);
                }
            }
        }
    }
}

void FaceTracking::SetupLandmarkDataConstraints()
{
    if (!m_landmarkConstraints && m_currentFrameData.m_landmarkData.size() > 0) {
        PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);
        m_landmarkConstraints = std::make_unique<LandmarkConstraints2D<float>>();
        m_landmarkConstraints->SetMeshLandmarks(GetMeshLandmarks());
        m_landmarkConstraints->SetEyeMeshLandmarks(GetTemplateDescription().GetEyeLeftMeshLandmarks(),
                                                   GetTemplateDescription().GetEyeRightMeshLandmarks());
        m_landmarkConstraints->SetTeethMeshLandmarks(GetTemplateDescription().GetTeethMeshLandmarks());

        std::vector<std::pair<LandmarkInstance<float, 2>, Camera<float>> > landmarks;
        for (const auto& [cameraName, landmarkInstancePtr] : m_currentFrameData.m_landmarkData) {
            landmarks.push_back({*landmarkInstancePtr, m_cameraSetup.GetCamera(cameraName)});
        }
        m_landmarkConstraints->SetTargetLandmarks(landmarks);
        m_landmarkConstraints->SetUserDefinedLandmarkAndCurveWeights(m_userDefinedLandmarkAndCurveWeights);
    }
}

static std::unique_ptr<FlowConstraints<float>> SetupFlowConstraints(const std::map<std::string, std::shared_ptr<const MeshFlowData>>& flowData) {

    std::unique_ptr<FlowConstraints<float>> flowConstraints = std::make_unique<FlowConstraints<float>>();
    std::map<std::string, FlowConstraintsData<float>> flowConstraintsData;
    for (auto&& [cameraName, meshFlowData] : flowData) {
        if (meshFlowData != nullptr) {
            flowConstraintsData[cameraName] = FlowConstraintsData<float>{meshFlowData->m_targetCamera, meshFlowData->m_targetPixels, meshFlowData->m_confidences, meshFlowData->m_targetVertexIDs};
        } else {
            CARBON_CRITICAL("flow data should never by null");
        }
    }
    flowConstraints->SetFlowData(flowConstraintsData);
    return flowConstraints;
}

void FaceTracking::SetupTemporalFlowDataConstraints()
{
    if (!m_temporalFlowConstraints && m_currentFrameData.m_temporalFlow.size() > 0) {
        PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);
        m_temporalFlowConstraints = SetupFlowConstraints(m_currentFrameData.m_temporalFlow);
    }
}

void FaceTracking::SetupModelFlowDataConstraints()
{
    if (!m_modelFlowConstraints && m_currentFrameData.m_modelFlow.size() > 0) {
        PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);
        m_modelFlowConstraints = SetupFlowConstraints(m_currentFrameData.m_modelFlow);
    }
}

void FaceTracking::SetupUVFlowDataConstraints()
{
    if (!m_uvFlowConstraints && m_currentFrameData.m_uvFlow.size() > 0) {
        PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);
        m_uvFlowConstraints = SetupFlowConstraints(m_currentFrameData.m_uvFlow);
    }
}

void FaceTracking::SetupEyeballConstraints()
{
    if (!m_leftEyeConstraints && GetTemplateDescription().HasVertexWeights("eyeball_interface_left")) {
        PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);
        m_leftEyeConstraints = std::make_unique<EyeballConstraints<float>>();
        const int leftEyeMeshIndex = m_trackingRig.GetRigGeometry()->GetMeshIndex("eyeLeft_lod0_mesh");
        m_leftEyeConstraints->SetEyeballMesh(m_trackingRig.GetRigGeometry()->GetMesh(leftEyeMeshIndex));
        m_leftEyeConstraints->SetInterfaceVertices(GetTemplateDescription().GetVertexWeights("eyeball_interface_left"));
        m_leftEyeConstraints->SetEyeballPose(CurrentMeshVertices(FaceTrackingOutputMode::BEST, leftEyeMeshIndex));
    }
    if (!m_rightEyeConstraints && GetTemplateDescription().HasVertexWeights("eyeball_interface_right")) {
        PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);
        m_rightEyeConstraints = std::make_unique<EyeballConstraints<float>>();
        const int rightEyeMeshIndex = m_trackingRig.GetRigGeometry()->GetMeshIndex("eyeRight_lod0_mesh");
        m_rightEyeConstraints->SetEyeballMesh(m_trackingRig.GetRigGeometry()->GetMesh(rightEyeMeshIndex));
        m_rightEyeConstraints->SetInterfaceVertices(GetTemplateDescription().GetVertexWeights("eyeball_interface_right"));
        m_rightEyeConstraints->SetEyeballPose(CurrentMeshVertices(FaceTrackingOutputMode::BEST, rightEyeMeshIndex));
    }
}

std::string FaceTracking::GetICPMask(FaceTrackingFittingMode fittingMode) const
{
    if (fittingMode == FaceTrackingFittingMode::RIGID && GetTemplateDescription().HasVertexWeights("rigid_icp_mask")) {
        return "rigid_icp_mask";
    } else if (fittingMode == FaceTrackingFittingMode::FINE && GetTemplateDescription().HasVertexWeights("fine_icp_mask")) {
        return "fine_icp_mask";
    } else {
        return "icp_mask";
    }
}

std::string FaceTracking::GetFlowMask(FaceTrackingFittingMode fittingMode) const
{
    if (fittingMode == FaceTrackingFittingMode::FINE && GetTemplateDescription().HasVertexWeights("fine_flow_mask")) {
        return "fine_flow_mask";
    } else {
        return "flow_mask";
    }
}

std::string FaceTracking::GetUVFlowMask(FaceTrackingFittingMode /*fittingMode*/) const
{
    if (GetTemplateDescription().HasVertexWeights("uv_flow_mask")) {
        return "uv_flow_mask";
    } else {
        return "flow_mask";
    }
}

void FaceTracking::FitRigLogic(FaceTrackingFittingMode mode, size_t solveControlSetIndex)
{
    if (mode == FaceTrackingFittingMode::PCA) {
        FitPCA();
        return;
    }

    SetupDataConstraints();

    const Configuration& config = GetFittingConfiguration(mode);

    if (m_landmarkConstraints) {
         m_landmarkConstraints->SetConfiguration(config[ConfigNameLandmarks]);
    }
    if (m_icpConstraints) {
        m_icpConstraints->SetConfiguration(config[ConfigNameIcp]);
        const std::string icpMaskName = GetICPMask(mode);
        m_icpConstraints->SetCorrespondenceSearchVertexWeights(GetTemplateDescription().GetVertexWeights(icpMaskName));
    }
    std::vector<FlowConstraints<float>*> flowConstraints;
    const std::string flowMaskName = GetFlowMask(mode);
    if (m_temporalFlowConstraints) {
        m_temporalFlowConstraints->SetFlowWeight(config[ConfigNameFlow]["temporal flow"]);
        m_temporalFlowConstraints->SetVertexWeights(GetTemplateDescription().GetVertexWeights(flowMaskName));
        flowConstraints.push_back(m_temporalFlowConstraints.get());
    }
    if (m_modelFlowConstraints) {
        m_modelFlowConstraints->SetFlowWeight(config[ConfigNameFlow]["model flow"]);
        m_modelFlowConstraints->SetVertexWeights(GetTemplateDescription().GetVertexWeights(flowMaskName));
        flowConstraints.push_back(m_modelFlowConstraints.get());
    }
    if (m_uvFlowConstraints) {
        m_uvFlowConstraints->SetFlowWeight(config[ConfigNameFlow]["uv flow"]);
        m_uvFlowConstraints->SetVertexWeights(GetTemplateDescription().GetVertexWeights(flowMaskName));
        flowConstraints.push_back(m_uvFlowConstraints.get());
    }

    const int iterations = config[ConfigNameGeneral]["iterations"];

    switch (mode) {
        case FaceTrackingFittingMode::COARSE: {
            EstimateRigidUsingLandmarks();
        }
        break;
        case FaceTrackingFittingMode::RIGID: {
            if (m_currentFrameData.m_depthmapData.size() == 1) {
                QRigidMotion<float> rigidMotion;
                rigidMotion.q = CurrentRigid().Linear();
                rigidMotion.t = CurrentRigid().Translation();

                auto meshPtr = CurrentMesh(FaceTrackingOutputMode::BEST);
                const auto& mask = GetTemplateDescription().GetVertexWeights(GetICPMask(FaceTrackingFittingMode::RIGID)).NonzeroVertices();

                rigidMotion = RigidICPFast(meshPtr->Vertices(),
                                           meshPtr->VertexNormals(),
                                           mask,
                                           m_currentFrameData.m_depthmapData.front()->camera,
                                           m_currentFrameData.m_depthmapData.front()->depthAndNormals,
                                           rigidMotion, /*numIterations=*/20);

                m_currentState.m_mesh2scan.SetLinear(rigidMotion.q.toRotationMatrix());
                m_currentState.m_mesh2scan.SetTranslation(rigidMotion.t);
            } else {
                m_defModelRigid->SetConfiguration(config[ConfigNameRigid]);
                m_defModelRigid->SetVertices(CurrentHeadVertices(FaceTrackingOutputMode::BEST));
                m_defModelRigid->SetRigidTransformation(CurrentRigid());

                FitData<float>(m_trackingRig.GetBaseMeshTriangulated(), m_defModelRigid.get(), m_landmarkConstraints.get(), m_icpConstraints.get(), flowConstraints, {}, iterations);

                m_currentState.m_mesh2scan = m_defModelRigid->RigidTransformation();
            }
        };
        break;
        case FaceTrackingFittingMode::PCA: {
            FitPCA();
        };
        break;
        case FaceTrackingFittingMode::POSEBASEDSOLVE: {
            PerformPoseBasedSolve();
        };
        break;
        case FaceTrackingFittingMode::RIGLOGIC: {

            m_defModelRigLogic->SetConfiguration(config[ConfigNameRigLogic]);
            m_defModelRigLogic->SetSymmetricControls(m_symmetricRigLogicGuiControls);
            m_defModelRigLogic->SetRigidTransformation(CurrentRigid());
            // select the correct control set
            m_defModelRigLogic->SetRigLogicSolveControls(m_trackingRig.GetRigLogicSolveControls(solveControlSetIndex));
            m_defModelRigLogic->SetSolveControlsToOptimize(m_trackingRigState->RigSolveControlsToOptimize(solveControlSetIndex));
            m_defModelRigLogic->SetGuiControls(m_currentState.m_guiControls);

            // std::vector<DepthmapConstraints> depthmapConstraints;
            // for (const auto& depthmapData : m_currentFrameData.m_depthmapData) {
            //     depthmapConstraints.emplace_back(DepthmapConstraints(depthmapData->camera, depthmapData->depthAndNormals));
            //     const std::string icpMaskName = GetICPMask(mode);
            //     depthmapConstraints.back().SetVertexMask(GetTemplateDescription().GetVertexWeights(icpMaskName).NonzeroVertices());
            //     depthmapConstraints.back().GetOptions().geometryWeight = m_fittingConfigurations[mode][ConfigNameIcp]["geometryWeight"].template Value<float>();
            // }

            const float l1Reg = config[ConfigNameGeneral]["l1Regularization"];
            FitDataL1AndBounds(m_trackingRig.GetBaseMeshTriangulated(), m_defModelRigLogic.get(), m_landmarkConstraints.get(), m_icpConstraints.get(), flowConstraints, iterations, l1Reg);
            // FitDataL1AndBounds<float>(m_trackingRig.GetBaseMeshTriangulated(), m_defModelRigLogic.get(), m_landmarkConstraints.get(), m_icpConstraints.get(), depthmapConstraints, {}, iterations, l1Reg);

            m_currentState.m_mesh2scan = m_defModelRigLogic->RigidTransformation();
            const Eigen::VectorXf guiControls = m_defModelRigLogic->GuiControls();
            SetRigGuiControlValues(m_defModelRigLogic->GuiControls());
        };
        break;
        case FaceTrackingFittingMode::FINE: {

            m_defModelVertex->SetConfiguration(config[ConfigNamePerVertex]);
            m_defModelVertex->SetRigidTransformation(CurrentRigid());
            m_defModelVertex->SetRestVertices(m_currentState.m_rigVertices);
            m_defModelVertex->SetVertexOffsets(m_currentState.m_vertexOffsets);

            if (config[ConfigNamePerVertex]["optimizePose"].template Value<bool>()) {
                CARBON_CRITICAL("no support for optimizing pose for fine alignment");
            }

            std::vector<EyeballConstraints<float>*> vectorOfEyeballConstraints;
            const float eyeballWeight = config[ConfigNameGeneral]["eyeball"];
            if (eyeballWeight > 0) {
                if (m_leftEyeConstraints) {
                    const int leftEyeMeshIndex = m_trackingRig.GetRigGeometry()->GetMeshIndex("eyeLeft_lod0_mesh");
                    m_leftEyeConstraints->SetEyeballPose(CurrentRigid().Transform(CurrentMeshVertices(FaceTrackingOutputMode::BEST, leftEyeMeshIndex)));
                    auto eyeConfig = m_leftEyeConstraints->GetConfiguration();
                    eyeConfig["eyeball"].Set(eyeballWeight);
                    m_leftEyeConstraints->SetConfiguration(eyeConfig);
                    vectorOfEyeballConstraints.push_back(m_leftEyeConstraints.get());
                }
                if (m_rightEyeConstraints) {
                    const int rightEyeMeshIndex = m_trackingRig.GetRigGeometry()->GetMeshIndex("eyeRight_lod0_mesh");
                    m_rightEyeConstraints->SetEyeballPose(CurrentRigid().Transform(CurrentMeshVertices(FaceTrackingOutputMode::BEST, rightEyeMeshIndex)));
                    auto eyeConfig = m_rightEyeConstraints->GetConfiguration();
                    eyeConfig["eyeball"].Set(eyeballWeight);
                    m_rightEyeConstraints->SetConfiguration(eyeConfig);
                    vectorOfEyeballConstraints.push_back(m_rightEyeConstraints.get());
                }
            }

            FitData(m_trackingRig.GetBaseMeshTriangulated(), m_defModelVertex.get(), m_landmarkConstraints.get(), m_icpConstraints.get(), flowConstraints, vectorOfEyeballConstraints, iterations);

            m_currentState.m_mesh2scan = m_defModelVertex->RigidTransformation();
            m_currentState.m_vertices = m_defModelVertex->DeformedVertices();
            m_currentState.m_vertexOffsets = m_defModelVertex->VertexOffsets();
            m_currentState.m_postProcessedGuiControls.reset();

            UpdateCurrentMesh();
        };
        break;
    }

    m_constraintsDebugInfo.clear();
}

void FaceTracking::FitPCA()
{
    const auto& mode = FaceTrackingFittingMode::PCA;
    const Configuration& config = GetFittingConfiguration(mode);

    SetupDataConstraints();

    if (m_landmarkConstraints) {
        m_landmarkConstraints->SetConfiguration(config[ConfigNameLandmarks]);
    }
    if (m_icpConstraints) {
        m_icpConstraints->SetConfiguration(config[ConfigNameIcp]);
        const std::string icpMaskName = GetICPMask(mode);
        m_icpConstraints->SetCorrespondenceSearchVertexWeights(GetTemplateDescription().GetVertexWeights(icpMaskName));
    }
    std::vector<FlowConstraints<float>*> flowConstraints;
    const std::string flowMaskName = GetFlowMask(mode);
    if (m_temporalFlowConstraints) {
        m_temporalFlowConstraints->SetFlowWeight(config[ConfigNameFlow]["temporal flow"]);
        m_temporalFlowConstraints->SetVertexWeights(GetTemplateDescription().GetVertexWeights(flowMaskName));
        flowConstraints.push_back(m_temporalFlowConstraints.get());
    }
    if (m_modelFlowConstraints) {
        m_modelFlowConstraints->SetFlowWeight(config[ConfigNameFlow]["model flow"]);
        m_modelFlowConstraints->SetVertexWeights(GetTemplateDescription().GetVertexWeights(flowMaskName));
        flowConstraints.push_back(m_modelFlowConstraints.get());
    }
    if (m_uvFlowConstraints) {
        m_uvFlowConstraints->SetFlowWeight(config[ConfigNameFlow]["uv flow"]);
        m_uvFlowConstraints->SetVertexWeights(GetTemplateDescription().GetVertexWeights(flowMaskName));
        flowConstraints.push_back(m_uvFlowConstraints.get());
    }

    std::vector<DepthmapConstraints> depthmapConstraints;
    for (const auto& depthmapData : m_currentFrameData.m_depthmapData) {
        depthmapConstraints.emplace_back(DepthmapConstraints(depthmapData->camera, depthmapData->depthAndNormals));
        const std::string icpMaskName = GetICPMask(mode);
        depthmapConstraints.back().SetVertexMask(GetTemplateDescription().GetVertexWeights(icpMaskName).NonzeroVertices());
        depthmapConstraints.back().GetOptions().geometryWeight = config[ConfigNameIcp]["geometryWeight"];
        depthmapConstraints.back().ClearDynamicDistanceThreshold(10.0f);
    }

    PCAFaceTracking::Settings settings;
    settings.iterations = config[ConfigNameGeneral]["iterations"].Value<int>();
    settings.pcaRegularization = config[ConfigNameGeneral]["regularization"].Value<float>();
    settings.withRigid = config[ConfigNameGeneral]["optimizePose"].Value<bool>();
    settings.pcaVelocityRegularization = config[ConfigNameGeneral]["velocity"].Value<float>();
    settings.pcaAccelerationRegularization = config[ConfigNameGeneral]["acceleration"].Value<float>();

    Affine<float, 3, 3> rigid = m_currentState.m_mesh2scan * m_pcaFaceTracking.PCARigToMesh();
    Eigen::VectorXf pcaCoeffs = m_currentState.m_pcaCoeffs;
    if (pcaCoeffs.size() != m_pcaFaceTracking.GetPCARig().NumCoeffs()) {
        LOG_INFO("reseting pca parameters: {} vs {}", pcaCoeffs.size(), m_pcaFaceTracking.GetPCARig().NumCoeffs());
        pcaCoeffs = Eigen::VectorXf::Zero(m_pcaFaceTracking.GetPCARig().NumCoeffs());
    }

    std::vector<Eigen::VectorXf> pcaCoeffsPrevFrames;
    if (m_previousStates.size() > 0 && m_previousStates[0] && m_previousStates[0]->m_pcaCoeffs.size() == pcaCoeffs.size()) {
        pcaCoeffsPrevFrames.push_back(m_previousStates[0]->m_pcaCoeffs);
        if (m_previousStates.size() > 1 && m_previousStates[1] && m_previousStates[1]->m_pcaCoeffs.size() == pcaCoeffs.size()) {
            pcaCoeffsPrevFrames.push_back(m_previousStates[1]->m_pcaCoeffs);
        }
    }

    m_pcaFaceTracking.FitPCAData(m_trackingRig.GetBaseMeshTriangulated(),
               depthmapConstraints,
               depthmapConstraints.empty() ? m_icpConstraints.get() : nullptr,
               m_landmarkConstraints.get(),
               flowConstraints,
               rigid, pcaCoeffs, pcaCoeffsPrevFrames, settings, m_pcaFaceTrackingStates);
    m_currentState.m_mesh2scan = rigid * m_pcaFaceTracking.PCARigToMesh().Inverse();
    m_currentState.m_pcaCoeffs = pcaCoeffs;
    m_currentState.m_rigVertices = m_pcaFaceTracking.PCARigToMesh().Transform(m_pcaFaceTracking.GetPCARig().facePCA.EvaluateLinearized(pcaCoeffs, rt::LinearVertexModel<float>::EvaluationMode::STATIC));
    m_currentState.m_vertices = m_currentState.m_rigVertices;
    m_currentState.m_vertexOffsets = m_currentState.m_vertices - m_currentState.m_rigVertices;
    m_currentState.m_vertexOffsets.setZero();
    m_currentState.m_postProcessedGuiControls.reset();
    UpdateCurrentMesh();

    m_constraintsDebugInfo.clear();
}

void FaceTracking::PerformPoseBasedSolve()
{
    CARBON_ASSERT(HasPCAModel() && HasPoseBasedSolverModel(), "Must have both a PCA model and a pose-based solver model for Pose-Based Solve fitting");

    // next, run the pose-based solver(s) on the output mesh data to get final
    Eigen::VectorXf guiControls = m_defaultGuiControlValues;

    // set up the vertices for solving
    std::vector<Eigen::Matrix<float, 3, -1>> meshVertices(rigposebasedsolver::RigGenerateTrainingData::MeshNames().size());
    unsigned i = 0;
    for (const auto& name : rigposebasedsolver::RigGenerateTrainingData::MeshNames())
    {
        const int meshIndex = m_defModelRigLogic->MeshIndex(name.c_str());
        meshVertices[i] = CurrentMeshVertices(FaceTrackingOutputMode::BEST, meshIndex);
        i++;
    }

    m_globalThreadPool->AddTaskRangeAndWait(int(m_rigPoseBasedSolversData->size()), [&](int start, int end) {
        for (int i = start; i < end; ++i) {
        // for (int i = 0; i < int(m_rigPoseBasedSolversData->size()); ++i) {
            const rigposebasedsolver::RigPoseBasedSolverData& solverData = m_rigPoseBasedSolversData->at(i);
            auto uiControlResults = solverData.existingSolver.SolveFrame(meshVertices);
            const auto& uiControlNames = m_trackingRig.GetGuiControlNames();
            for (const auto& result : uiControlResults) {
                auto it = std::find(uiControlNames.begin(), uiControlNames.end(), result.first);
                guiControls(static_cast<unsigned>(it - uiControlNames.begin())) = static_cast<float>(result.second);
            }
        }
    });

    m_currentState.m_postProcessedGuiControls = std::make_shared<Eigen::VectorXf>(guiControls);
    UpdateCurrentMesh();
}



std::shared_ptr<const ConstraintDebugInfo<float>> FaceTracking::CurrentDebugConstraints(FaceTrackingOutputMode outputMode)
{
    auto it = m_constraintsDebugInfo.find(outputMode);
    if (it != m_constraintsDebugInfo.end()) {
        return it->second;
    }

    const Affine<float, 3, 3> source2target = CurrentRigid();

    // retrieve face vertices and bake rigid transformation
    Mesh<float> mesh = *CurrentMesh(outputMode);
    mesh.SetVertices(source2target.Transform(mesh.Vertices()));
    mesh.CalculateVertexNormals();
    DiffDataMatrix<float, 3, -1> diffVertices(mesh.Vertices());

    // retrieve eye and teeth vertices (evaluate on rig)
    Eigen::VectorXf controls = m_currentState.m_guiControls;
    switch (outputMode) {
        case FaceTrackingOutputMode::NEUTRAL:
            controls.setZero();
            break;
        case FaceTrackingOutputMode::RIGLOGIC:
        case FaceTrackingOutputMode::FINE:
        case FaceTrackingOutputMode::BEST:
            break;
        case FaceTrackingOutputMode::POST:
            if (m_currentState.m_postProcessedGuiControls) controls = *m_currentState.m_postProcessedGuiControls;
            break;
        default:
            CARBON_CRITICAL("mesh output mode missing");
    }
    m_defModelRigLogic->SetGuiControls(controls);
    m_defModelRigLogic->SetRigidTransformation(source2target);
    DiffDataMatrix<float, 3, -1> leftEyeVertices = DiffDataMatrix<float, 3, -1>(CurrentRigid().Transform(CurrentMeshVertices(outputMode, m_defModelRigLogic->LeftEyeMeshIndex())));
    DiffDataMatrix<float, 3, -1> rightEyeVertices = DiffDataMatrix<float, 3, -1>(CurrentRigid().Transform(CurrentMeshVertices(outputMode, m_defModelRigLogic->RightEyeMeshIndex())));
    DiffDataMatrix<float, 3, -1> teethVertices = DiffDataMatrix<float, 3, -1>(CurrentRigid().Transform(CurrentMeshVertices(outputMode, m_defModelRigLogic->TeethMeshIndex())));

    SetupDataConstraints();

    PolyAllocator<ConstraintDebugInfo<float>> polyAlloc{ MEM_RESOURCE };
    std::shared_ptr<ConstraintDebugInfo<float>> constraintsDebugInfo = std::allocate_shared<ConstraintDebugInfo<float>>(polyAlloc);

    if (m_icpConstraints) {
        m_icpConstraints->FindCorrespondences(mesh.Vertices(), mesh.VertexNormals(), constraintsDebugInfo->correspondences);
    }

    if (m_landmarkConstraints) {
        m_landmarkConstraints->EvaluateLandmarks(diffVertices, LandmarkConstraints2D<float>::MeshType::Face, &constraintsDebugInfo->landmarkConstraints);
        m_landmarkConstraints->EvaluateCurves(diffVertices, LandmarkConstraints2D<float>::MeshType::Face, &constraintsDebugInfo->curveConstraints);
        m_landmarkConstraints->EvaluateContours(diffVertices, mesh.VertexNormals(), LandmarkConstraints2D<float>::MeshType::Face, &constraintsDebugInfo->contourConstraints);
        m_landmarkConstraints->EvaluateInnerLips(diffVertices, mesh.VertexNormals(), &constraintsDebugInfo->lipConstraintsUpper, &constraintsDebugInfo->lipConstraintsLower);
        m_landmarkConstraints->EvaluateCurves(leftEyeVertices, LandmarkConstraints2D<float>::MeshType::EyeLeft, &constraintsDebugInfo->eyeLeftCurveConstraints);
        m_landmarkConstraints->EvaluateCurves(rightEyeVertices, LandmarkConstraints2D<float>::MeshType::EyeRight, &constraintsDebugInfo->eyeRightCurveConstraints);
        m_landmarkConstraints->EvaluateLandmarks(teethVertices, LandmarkConstraints2D<float>::MeshType::Teeth, &constraintsDebugInfo->teethLandmarkConstraints);
    }

    if (m_temporalFlowConstraints) {
        m_temporalFlowConstraints->Evaluate(diffVertices, &constraintsDebugInfo->temporalFlowConstraints);
    }
    if (m_modelFlowConstraints) {
        m_modelFlowConstraints->Evaluate(diffVertices, &constraintsDebugInfo->modelFlowConstraints);
    }
    if (m_uvFlowConstraints) {
        m_uvFlowConstraints->Evaluate(diffVertices, &constraintsDebugInfo->uvFlowConstraints);
    }

    m_constraintsDebugInfo[outputMode] = constraintsDebugInfo;
    return constraintsDebugInfo;
}

const Configuration& FaceTracking::GetFittingConfiguration(FaceTrackingFittingMode fittingMode) const
{
    auto it = m_fittingConfigurations.find(fittingMode);
    if (it != m_fittingConfigurations.end()) {
        return it->second;
    } else {
        CARBON_CRITICAL("no settings for fitting mode");
    }
}

void FaceTracking::SetFittingConfiguration(FaceTrackingFittingMode fittingMode, const Configuration& fittingConfiguration)
{
    m_fittingConfigurations.insert_or_assign(fittingMode, fittingConfiguration);}

void FaceTracking::SetRigSolveControlValues(size_t solveControlSetIndex, const Eigen::VectorXf& values)
{
    m_trackingRigState->SetRigSolveControlValues(solveControlSetIndex, values, m_trackingRig);
    SetRigGuiControlValues(m_trackingRigState->RigGuiControlValues());
}

void FaceTracking::SetRigGuiControlValues(const Eigen::VectorXf& values)
{
    if (m_currentState.m_guiControls.size() != values.size()) {
        CARBON_CRITICAL("rig control vector size incorrect");
    }
    m_currentState.m_guiControls = values;
    m_defModelRigLogic->SetRigLogicSolveControls(nullptr); // no solve control set for the mesh evaluation
    m_defModelRigLogic->SetGuiControls(m_currentState.m_guiControls);
    m_currentState.m_rigVertices = m_defModelRigLogic->DeformedVertices(/*meshIndex=*/0);
    m_currentState.m_vertexOffsets.setZero(); // clear the vertex offsets
    m_currentState.m_vertices = m_currentState.m_rigVertices;
    m_currentState.m_pcaCoeffs = Eigen::VectorXf(); // clear pca coefficients
    m_currentState.m_postProcessedGuiControls.reset();

    UpdateCurrentMesh();
    UpdateSolveControlSetsData();
}

void FaceTracking::UpdateSolveControlSetsData()
{
    m_trackingRigState->SetRigGuiControlValues(m_currentState.m_guiControls, m_trackingRig);
}

void FaceTracking::ResetRigSolveControlValues(size_t solveControlSetIndex, bool resetNextSets)
{
    m_trackingRigState->ResetRigSolveControlValues(solveControlSetIndex, resetNextSets, m_trackingRig);
    SetRigGuiControlValues(m_trackingRigState->RigGuiControlValues());
}

void FaceTracking::SetRigSolveControlsToOptimize(size_t solveControlSetIndex, const std::vector<bool>& controlsToOptimize)
{
    m_trackingRigState->SetRigSolveControlsToOptimize(solveControlSetIndex, controlsToOptimize, m_trackingRig);

    const std::vector<bool> guiControlsToOptimize = m_trackingRigState->RigSolveControlsToOptimize(m_trackingRigState->GetNumSolveControlSets() - 1);
    const std::vector<std::string>& guiControlNames = m_trackingRig.GetRigSolveControlNames(m_trackingRigState->GetNumSolveControlSets() - 1);
    m_fixedRigLogicGuiControls.clear();
    for (size_t i = 0; i < guiControlsToOptimize.size(); ++i) {
        if (!guiControlsToOptimize[i]) {
            m_fixedRigLogicGuiControls.emplace_back(guiControlNames[i]);
        }
    }
}

void FaceTracking::ResetVertexOffsets()
{
    m_currentState.m_vertexOffsets.setZero();
    m_currentState.m_vertices = m_currentState.m_rigVertices;
    UpdateCurrentMesh();
}

const Mesh<float>& FaceTracking::RigMesh(int meshIndex) const
{
    return m_trackingRig.GetRigGeometry()->GetMesh(meshIndex);
}

const std::string& FaceTracking::RigMeshName(int meshIndex) const
{
    return m_trackingRig.GetRigGeometry()->GetMeshName(meshIndex);
}


} // namespace epic::nls
