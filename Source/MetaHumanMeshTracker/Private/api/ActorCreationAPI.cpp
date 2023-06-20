// Copyright Epic Games, Inc. All Rights Reserved.

#include <ActorCreationAPI.h>

#include "Internals/ActorCreationUtils.h"
#include "Internals/ReferenceAligner.h"
#include "Internals/FrameInputData.h"

#include <carbon/utils/TaskThreadPool.h>
#include <Common.h>
#include <conformer/DnaDatabaseDescription.h>
#include <conformer/FaceFitting.h>
#include <conformer/IdentityModelFitting.h>
#include <nls/rig/RigLogicDNAResource.h>
#include <nrr/landmarks/LandmarkConfiguration.h>
#include <nrr/TemplateDescription.h>
#include <Internals/OpenCVCamera2MetaShapeCamera.h>
#include <pma/PolyAllocator.h>
#include <pma/utils/ManagedInstance.h>
#include <conformer/RigLogicFitting.h>
#include <conformer/RigMorphModule.h>

#include <cstring>
#include <filesystem>
#include <map>

using namespace epic::nls;
using namespace pma;


namespace titan::api {

struct ActorCreationAPI::Private {
    std::string configurationDirectory;
    std::unique_ptr<FaceFitting<float>, std::function<void(FaceFitting<float>*)> > faceFitting{};
    std::unique_ptr<IdentityModelFitting<float>, std::function<void(IdentityModelFitting<float>*)> > teethFitting{};
    std::unique_ptr<RigLogicFitting<float>, std::function<void(RigLogicFitting<float>*)> > rigLogicFitting{};
    MultiCameraSetup<float> cameras;
    std::map<FittingMaskType, VertexWeights<float> > masks;
    std::map<int, std::shared_ptr<FrameInputData> > frameData;
    std::unique_ptr<ReferenceAligner, std::function<void(ReferenceAligner*)> > referenceAligner{};
    InputDataType dataType{InputDataType::NONE};
    std::vector<epic::nls::Affine<float, 3, 3>> currentToScanTransforms;
    std::vector<float> currentToScanScales;
    bool fittingInitialized = false;

    std::shared_ptr<epic::carbon::TaskThreadPool> globalThreadPool = epic::carbon::TaskThreadPool::GlobalInstance(
        /*createIfNotAvailable=*/ true);

    bool InitFaceFitting() {
        if (!fittingInitialized) {
            const auto landmarks3d = Prepare3DLandmarks(frameData);
            faceFitting->SetTarget3DLandmarks(landmarks3d);
            if (dataType == InputDataType::DEPTHS) {
                const auto landmarks2d = Prepare2DLandmarks(frameData, cameras);
                const auto depths = PrepareDepths(frameData);
                faceFitting->SetTarget2DLandmarks(landmarks2d);
                faceFitting->SetTargetDepths(depths);
            }
            else if (dataType == InputDataType::SCAN) {
                std::map<int, float> estimatedToRigScale;
                std::map <int, MultiCameraSetup<float>> camerasPerFrame;
                // initially rigid + scale align using projected landmarks
                // scan is scaled,  and transform is added to toScanTransform for simple implementation (not adding scale variable to FaceFitting)
                int i = 0;
                for (auto& [frameNum, frame] : frameData) {
                    auto [scaleToScan, transformToScan] = EstimateRigidAndScaleUsingLandmarks(frameData[frameNum],
                                                                                              cameras,
                                                                                              faceFitting->CurrentMeshLandmarks());
                    float scaleToRig = 1.0f / scaleToScan;
                    estimatedToRigScale[frameNum] = scaleToRig;
                    camerasPerFrame[frameNum] = ScaledCameras(cameras, scaleToRig);
                    transformToScan.SetTranslation(transformToScan.Translation() * scaleToRig);
                    currentToScanTransforms[i] = transformToScan;
                    currentToScanScales[i] = scaleToScan;
                    i++;
                }
                const auto [weights, meshes] = PrepareMeshes(frameData, estimatedToRigScale);
                const auto landmarks2d = Prepare2DLandmarks(frameData, camerasPerFrame);
                faceFitting->SetTarget2DLandmarks(landmarks2d);
                faceFitting->SetTargetMeshes(meshes, weights);
            }
            else {
                return false;
            }
        }
        fittingInitialized = true;
        return true;
    }
};


ActorCreationAPI::ActorCreationAPI()
    : m(new Private()) {
}

ActorCreationAPI::~ActorCreationAPI() {
    delete m;
}

bool ActorCreationAPI::Init(const std::string& InConfigurationDirectory) {
    try {
        RESET_ERROR;
        m->configurationDirectory = InConfigurationDirectory;

        TemplateDescription templateDescription;
        DnaDatabaseDescription dnaDatabaseDescription;
        CHECK_OR_RETURN(templateDescription.Load(InConfigurationDirectory + "/template_description.json"),
                        false,
                        "failed to load template description");
        CHECK_OR_RETURN(dnaDatabaseDescription.Load(InConfigurationDirectory + "/dna_database_description.json"),
                        false,
                        "failed to load dna database description");

        Mesh<float> headMesh = templateDescription.Topology();
        headMesh.Triangulate();
        Mesh<float> teethMesh = templateDescription.GetAssetTopology("teeth");
        teethMesh.Triangulate();

        m->masks[FittingMaskType::RIGID] = templateDescription.GetVertexWeights("nonrigid_mask");
        m->masks[FittingMaskType::NONRIGID] = templateDescription.GetVertexWeights("nonrigid_mask");
        m->masks[FittingMaskType::FINE] = templateDescription.GetVertexWeights("fine_mask");
        m->masks[FittingMaskType::TEETH] = templateDescription.GetAssetVertexWeights("teeth", "nonrigid_mask");

        m->faceFitting = pma::UniqueInstance<FaceFitting<float> >::with(MEM_RESOURCE).create();
        m->faceFitting->SetSourceMesh(headMesh);
        m->faceFitting->LoadModel(dnaDatabaseDescription.PcaIdentityModelFilename());
        m->faceFitting->SetLipCollisionMasks(templateDescription.GetVertexWeights("lip_collision_upper"),
                                             templateDescription.GetVertexWeights("lip_collision_lower"));

        MeshLandmarks<float> meshLandmarks = templateDescription.GetMeshLandmarks();
        m->faceFitting->SetMeshLandmarks(meshLandmarks);

        // set default parameters for scan
        m->faceFitting->ModelRegistrationConfiguration()["minimumDistanceThreshold"].Set(10.f);
        m->faceFitting->FineRegistrationConfiguration()["minimumDistanceThreshold"].Set(10.f);
        m->faceFitting->FineRegistrationConfiguration()["vertexOffsetRegularization"].Set(0.01f);
        m->faceFitting->FineRegistrationConfiguration()["vertexLaplacian"].Set(1.0f);
        m->faceFitting->FineRegistrationConfiguration()["collisionWeight"].Set(0.1f);

        m->teethFitting = pma::UniqueInstance<IdentityModelFitting<float> >::with(MEM_RESOURCE).create();
        m->teethFitting->LoadModel(dnaDatabaseDescription.AssetIdentityModel("teeth"));
        m->teethFitting->SetSourceMesh(teethMesh);
        m->teethFitting->SetMeshLandmarks(templateDescription.GetTeethMeshLandmarks());
        m->teethFitting->ModelRegistrationConfiguration()["optimizeScale"].Set(false);

        m->rigLogicFitting = pma::UniqueInstance<RigLogicFitting<float> >::with(MEM_RESOURCE).create();
        m->rigLogicFitting->SetMeshLandmarks(templateDescription.GetTeethMeshLandmarks());

        if (templateDescription.GetMeshLandmarks().HasLandmark("pt_frankfurt_fr") &&
            templateDescription.GetMeshLandmarks().HasLandmark("pt_frankfurt_rr") &&
            templateDescription.GetMeshLandmarks().HasLandmark("pt_frankfurt_fl") &&
            templateDescription.GetMeshLandmarks().HasLandmark("pt_frankfurt_rl")) {
            const BarycentricCoordinates<float> fr =
                templateDescription.GetMeshLandmarks().LandmarksBarycentricCoordinates().find("pt_frankfurt_fr")->second;
            const BarycentricCoordinates<float> rr =
                templateDescription.GetMeshLandmarks().LandmarksBarycentricCoordinates().find("pt_frankfurt_rr")->second;
            const BarycentricCoordinates<float> fl =
                templateDescription.GetMeshLandmarks().LandmarksBarycentricCoordinates().find("pt_frankfurt_fl")->second;
            const BarycentricCoordinates<float> rl =
                templateDescription.GetMeshLandmarks().LandmarksBarycentricCoordinates().find("pt_frankfurt_rl")->second;

            m->referenceAligner = pma::UniqueInstance<ReferenceAligner>::with(MEM_RESOURCE).create(headMesh, fr, rr, fl, rl);
        }

        return true;
    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to initialize: {}", e.what());
    }
}

bool ActorCreationAPI::SetCameras(const std::map<std::string, OpenCVCamera>& InCameras) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(m->faceFitting, false, "face fitting has not been initialized");

        std::vector<MetaShapeCamera<float> > metaCameras;
        for (const auto& [cameraName, opencvCamera] : InCameras) {
            metaCameras.emplace_back(OpenCVCamera2MetaShapeCamera<float>(cameraName.c_str(), opencvCamera));
        }
        MultiCameraSetup<float> cameraSetup;
        cameraSetup.Init(metaCameras);

        m->cameras = cameraSetup;

        return true;
    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to set cameras: {}",
                         e.what());
    }
}

bool ActorCreationAPI::SetDepthInputData(int32_t frameNum,
                                         const std::map<std::string, const std::map<std::string, FaceTrackingLandmarkData> >& InLandmarksDataPerCamera,
                                         const std::map<std::string, const float*>& InDepthMaps)
{
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(m->faceFitting, false, "face fitting has not been initialized");
        if (!m->frameData.empty()) {
            CHECK_OR_RETURN(m->dataType == InputDataType::DEPTHS, false, "data buffer already contains non-depth frames");
        } else {
            m->dataType = InputDataType::DEPTHS;
        }

        const MultiCameraSetup<float>& cameraSetup = m->cameras;

        std::map<std::string, std::shared_ptr<const LandmarkInstance<float, 2> > > landmarksPerCamera;
        std::map<std::string, std::shared_ptr<const DepthmapData<float> > > depthPerCamera;

        // convert landmark input
        for (const auto& [cameraName, perCameraLandmarkData] : InLandmarksDataPerCamera) {
            CHECK_OR_RETURN(cameraSetup.HasCamera(cameraName), false, "no camera {}", cameraName);
            landmarksPerCamera[cameraName] = CreateLandmarkInstanceForCamera(perCameraLandmarkData,
                                                                             std::map<std::string, std::vector<std::string> >{},
                                                                             cameraSetup.GetCamera(
                                                                                 cameraName));
        }

        for (const auto& [cameraName, depthMaps] : InDepthMaps) {
            depthPerCamera[cameraName] = ConstructDepthmapData(cameraName, depthMaps, cameraSetup);
        }
        PolyAllocator<FrameInputData> framePolyAllocator{MEM_RESOURCE};

        m->frameData[frameNum] = std::allocate_shared<FrameInputData>(framePolyAllocator, landmarksPerCamera, depthPerCamera);
        m->currentToScanTransforms.resize(/*numOfInputFrames=*/m->frameData.size());
        m->currentToScanScales.resize(/*numOfInputFrames=*/m->frameData.size());
        std::fill(m->currentToScanTransforms.begin(), m->currentToScanTransforms.end(), epic::nls::Affine<float, 3, 3>{});
        std::fill(m->currentToScanScales.begin(), m->currentToScanScales.end(), 1.0f);
        return true;
    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to Set input data: {}",
                         e.what());
    }
}

bool ActorCreationAPI::SetScanInputData(const std::map<std::string, const FaceTrackingLandmarkData>& In3dLandmarksData,
                                        const std::map<std::string, const std::map<std::string, FaceTrackingLandmarkData>>& In2dLandmarksData,
                                        const MeshInputData& InScanData)
{
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(m->faceFitting, false, "face fitting has not been initialized");
        if (!m->frameData.empty()) {
            LOG_WARNING("data stream is not empty. Cleared to store scan data.");
            CHECK_OR_RETURN(ResetInputData(), false, "falied to reset input data.");
        }

        // convert landmark input
        auto landmarksIn3D = Create3dLandmarkInstance(In3dLandmarksData, std::map<std::string, std::vector<std::string> >{});
        auto scanMesh = ConstructMesh(InScanData);

        std::map<std::string, std::shared_ptr<const LandmarkInstance<float, 2> > > landmarksPerCamera;
        const MultiCameraSetup<float>& cameraSetup = m->cameras;

        // convert landmark input
        for (const auto& [cameraName, perCameraLandmarkData] : In2dLandmarksData) {
            CHECK_OR_RETURN(cameraSetup.HasCamera(cameraName), false, "no camera {}", cameraName);
            landmarksPerCamera[cameraName] = CreateLandmarkInstanceForCamera(perCameraLandmarkData,
                                                                             std::map<std::string, std::vector<std::string> >{},
                                                                             cameraSetup.GetCamera(cameraName));
        }

        PolyAllocator<FrameInputData> framePolyAllocator{ MEM_RESOURCE };

        m->frameData[0] = std::allocate_shared<FrameInputData>(framePolyAllocator, landmarksPerCamera, landmarksIn3D, scanMesh);
        m->dataType = InputDataType::SCAN;
        m->currentToScanTransforms.resize(/*numOfInputFrames=*/m->frameData.size());
        m->currentToScanScales.resize(/*numOfInputFrames=*/m->frameData.size());
        std::fill(m->currentToScanTransforms.begin(), m->currentToScanTransforms.end(), epic::nls::Affine<float, 3, 3>{});
        std::fill(m->currentToScanScales.begin(), m->currentToScanScales.end(), 1.0f);

        return true;
    }
    catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to Set input data: {}", e.what());
    }
}

bool ActorCreationAPI::ResetInputData() {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(m->faceFitting, false, "face fitting has not been initialized");
        m->frameData.clear();
        m->dataType = InputDataType::NONE;
        return true;
    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to reset input data: {}", e.what());
    }
}

bool ActorCreationAPI::FitRigid(float* OutVertexPositions, float* OutStackedToScanTransforms, float* OutStackedToScanScales, int32_t numIters) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(m->faceFitting, false, "face fitting has not been initialized");
        CHECK_OR_RETURN(m->InitFaceFitting(), false, "face fitting has not been initialized");
        CHECK_OR_RETURN(!m->frameData.empty(), false, "frame data is empty");
        CHECK_OR_RETURN(m->dataType == InputDataType::DEPTHS || m->dataType == InputDataType::SCAN, false,
            "no input data set");



        // fit rigid
        m->currentToScanTransforms =
            m->faceFitting->RegisterRigid(m->currentToScanTransforms, m->masks[FittingMaskType::RIGID],  /*iter=*/ numIters);
        Eigen::Matrix3Xf resultVertices = m->faceFitting->CurrentDeformedVertices();

        memcpy(OutStackedToScanTransforms,
               m->currentToScanTransforms.data(),
               int32_t(m->frameData.size()) *
               int32_t(16) *
               sizeof(float));

        memcpy(OutStackedToScanScales,
               m->currentToScanScales.data(),
               int32_t(m->frameData.size()) *
               sizeof(float));

        memcpy(OutVertexPositions,
               resultVertices.data(),
               int32_t(resultVertices.cols() * resultVertices.rows()) *
               sizeof(float));

        return true;
    }
    catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to rigid align: {}", e.what());
    }
}

bool ActorCreationAPI::FitNonRigid(float* OutVertexPositions, float* OutStackedToScanTransforms, float* OutStackedToScanScales, int32_t numIters, const bool autoMode) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(m->faceFitting, false, "face fitting has not been initialized");
        CHECK_OR_RETURN(m->InitFaceFitting(), false, "face fitting has not been initialized");
        CHECK_OR_RETURN(!m->frameData.empty(), false, "frame data is empty");
        CHECK_OR_RETURN(m->dataType == InputDataType::DEPTHS || m->dataType == InputDataType::SCAN, false,
            "no input data set");

        // fit non-rigid
        if (autoMode) {
            for (float modelRegularization : {10.0f, 1.0f, 0.1f}) {
                m->faceFitting->ModelRegistrationConfiguration()["modelRegularization"].Set(modelRegularization);
                m->currentToScanTransforms =
                    m->faceFitting->RegisterNonRigid(m->currentToScanTransforms, m->masks[FittingMaskType::NONRIGID],  /*iter=*/ 5);
            }
        }
        else {
            m->currentToScanTransforms =
                m->faceFitting->RegisterNonRigid(m->currentToScanTransforms, m->masks[FittingMaskType::NONRIGID],  /*iter=*/ numIters);
        }
        Eigen::Matrix3Xf resultVertices = m->faceFitting->CurrentDeformedVertices();

        memcpy(OutStackedToScanTransforms,
               m->currentToScanTransforms.data(),
               int32_t(m->frameData.size()) *
               int32_t(16) *
               sizeof(float));

        memcpy(OutStackedToScanScales,
               m->currentToScanScales.data(),
               int32_t(m->frameData.size()) *
               sizeof(float));

        memcpy(OutVertexPositions,
               resultVertices.data(),
               int32_t(resultVertices.cols() * resultVertices.rows()) *
               sizeof(float));

        return true;
    }
    catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to non-rigid align: {}", e.what());
    }
}

bool ActorCreationAPI::FitPerVertex(float* OutVertexPositions, float* OutStackedToScanTransforms, float* OutStackedToScanScales, int32_t numIters) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(m->faceFitting, false, "face fitting has not been initialized");
        CHECK_OR_RETURN(m->InitFaceFitting(), false, "face fitting has not been initialized");
        CHECK_OR_RETURN(!m->frameData.empty(), false, "frame data is empty");
        CHECK_OR_RETURN(m->dataType == InputDataType::DEPTHS || m->dataType == InputDataType::SCAN, false,
            "no input data set");

        // fit per-vertex
        m->currentToScanTransforms =
            m->faceFitting->RegisterFine(m->currentToScanTransforms, m->masks[FittingMaskType::FINE],  /*iter=*/ numIters);
        Eigen::Matrix3Xf resultVertices = m->faceFitting->CurrentDeformedVertices();

        memcpy(OutStackedToScanTransforms,
               m->currentToScanTransforms.data(),
               int32_t(m->frameData.size()) *
               int32_t(16) *
               sizeof(float));

        memcpy(OutStackedToScanScales,
               m->currentToScanScales.data(),
               int32_t(m->frameData.size()) *
               sizeof(float));

        memcpy(OutVertexPositions,
               resultVertices.data(),
               int32_t(resultVertices.cols() * resultVertices.rows()) *
               sizeof(float));

        return true;
    }
    catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to per-vertex align: {}", e.what());
    }
}

bool ActorCreationAPI::FitTeeth(dna::StreamReader* InDnaStream, float* OutVertexPositions) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(m->faceFitting, false, "face fitting has not been initialized");
        CHECK_OR_RETURN(!m->frameData.empty(), false, "frame data is empty");
        CHECK_OR_RETURN(InDnaStream, false, "input dna stream not valid");
        CHECK_OR_RETURN(m->cameras.GetCameras().size() >= 1, false, "at least one camera have to be set");
        CHECK_OR_RETURN(m->dataType == InputDataType::DEPTHS || m->dataType == InputDataType::SCAN, false,
                        "no input data set");
        PolyAllocator<RigMorphModule> polyAllocator{MEM_RESOURCE};

        std::shared_ptr<RigMorphModule> rigMorphing = std::allocate_shared<RigMorphModule>(polyAllocator);
        m->rigLogicFitting->LoadRig(InDnaStream);
        rigMorphing->Init(InDnaStream);

        std::vector<Affine<float, 3, 3> > toScanTransform(m->frameData.size());

        Affine<float, 3, 3> teeth2HeadTransform;

        const auto landmarks2d = Prepare2DLandmarks(m->frameData, m->cameras);
        const auto landmarks3d = Prepare3DLandmarks(m->frameData);
        m->faceFitting->SetTarget2DLandmarks(landmarks2d);
        m->rigLogicFitting->SetTarget2DLandmarks(landmarks2d);
        m->teethFitting->SetTarget2DLandmarks(landmarks2d);
        m->faceFitting->SetTarget3DLandmarks(landmarks3d);
        m->rigLogicFitting->SetTarget3DLandmarks(landmarks3d);
        m->teethFitting->SetTarget3DLandmarks(landmarks3d);
        if (m->dataType == InputDataType::DEPTHS) {
            const auto depths = PrepareDepths(m->frameData);
            m->faceFitting->SetTargetDepths(depths);
            m->rigLogicFitting->SetTargetDepths(depths);
            m->teethFitting->SetTargetDepths(depths);
        } else {
            const auto [weights, meshes] = PrepareMeshes(m->frameData);
            m->faceFitting->SetTargetMeshes(meshes, weights);
            m->rigLogicFitting->SetTargetMeshes(meshes, weights);
            m->teethFitting->SetTargetMeshes(meshes, weights);
        }

        // fit rigid
        toScanTransform =
            m->faceFitting->RegisterRigid(toScanTransform, m->masks[FittingMaskType::RIGID],  /*iter=*/ 50);

        // fit rig logic - currently just using first frame
        toScanTransform = m->rigLogicFitting->RegisterRigLogic(toScanTransform,
                                                               m->masks[FittingMaskType::RIGID],
                                                               /*iter=*/ 20);

        // fit teeth - currently just using first frame
        const auto teeth2scan = m->teethFitting->RegisterNonRigid(toScanTransform,
                                                                  std::vector<VertexWeights<float>>{m->masks[FittingMaskType::TEETH]},
                                                                  20);
        teeth2HeadTransform = toScanTransform[0].Inverse() * teeth2scan[0];
        Eigen::Matrix3Xf estimatedTeethVertices = teeth2HeadTransform.Transform(m->teethFitting->CurrentDeformedVertices());

        memcpy(OutVertexPositions,
               estimatedTeethVertices.data(),
               int32_t(estimatedTeethVertices.cols() * estimatedTeethVertices.rows()) *
               sizeof(float));

        return true;
    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to fit teeth: {}", e.what());
    }
}

// get parameters
float ActorCreationAPI::GetModelRegularization() {
    return m->faceFitting->ModelRegistrationConfiguration()["modelRegularization"].template Value<float>();
}

float ActorCreationAPI::GetPerVertexOffsetRegularization() {
    return m->faceFitting->FineRegistrationConfiguration()["vertexOffsetRegularization"].template Value<float>();
}

float ActorCreationAPI::GetPerVertexLaplacianRegularization() {
    return m->faceFitting->FineRegistrationConfiguration()["vertexLaplacian"].template Value<float>();
}

float ActorCreationAPI::GetMinimumDistanceThreshold() {
    return m->faceFitting->ModelRegistrationConfiguration()["minimumDistanceThreshold"].template Value<float>();
}

float ActorCreationAPI::GetLandmarksWeight() {
    return m->faceFitting->ModelRegistrationConfiguration()["landmarksWeight"].template Value<float>();
}

float ActorCreationAPI::GetInnerLipsLandmarksWeight() {
    return m->faceFitting->ModelRegistrationConfiguration()["innerLipWeight"].template Value<float>();
}

float ActorCreationAPI::GetInnerLipsCollisionWeight() {
    return m->faceFitting->FineRegistrationConfiguration()["collisionWeight"].template Value<float>();
}

// set parameters
void ActorCreationAPI::SetModelRegularization(float regularization) {
    m->faceFitting->ModelRegistrationConfiguration()["modelRegularization"].Set(regularization);
}

void ActorCreationAPI::SetPerVertexOffsetRegularization(float regularization) {
    m->faceFitting->FineRegistrationConfiguration()["vertexOffsetRegularization"].Set(regularization);
}

void ActorCreationAPI::SetPerVertexLaplacianRegularization(float regularization) {
    m->faceFitting->FineRegistrationConfiguration()["vertexLaplacian"].Set(regularization);
}

void ActorCreationAPI::SetMinimumDistanceThreshold(float threshold) {
    m->faceFitting->ModelRegistrationConfiguration()["minimumDistanceThreshold"].Set(threshold);
    m->faceFitting->FineRegistrationConfiguration()["minimumDistanceThreshold"].Set(threshold);
    m->faceFitting->RigidRegistrationConfiguration()["minimumDistanceThreshold"].Set(threshold);
}

void ActorCreationAPI::SetLandmarksWeight(float weight) {
    m->faceFitting->ModelRegistrationConfiguration()["landmarksWeight"].Set(weight);
    m->faceFitting->FineRegistrationConfiguration()["landmarksWeight"].Set(weight);
    m->faceFitting->RigidRegistrationConfiguration()["landmarksWeight"].Set(weight);
}

void ActorCreationAPI::SetInnerLipsLandmarksWeight(float weight) {
    m->faceFitting->ModelRegistrationConfiguration()["innerLipWeight"].Set(weight);
    m->faceFitting->FineRegistrationConfiguration()["innerLipWeight"].Set(weight);
    m->faceFitting->RigidRegistrationConfiguration()["innerLipWeight"].Set(weight);
}

void ActorCreationAPI::SetInnerLipsCollisionWeight(float weight) {
    m->faceFitting->FineRegistrationConfiguration()["collisionWeight"].Set(weight);
}

bool ActorCreationAPI::GetFittingMask(float* OutVertexWeights, FittingMaskType InMaskType) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(!m->masks.empty(), false, "frame data is empty");

        // mask
        const auto& weights = m->masks[InMaskType].Weights();

        memcpy(OutVertexWeights,
               weights.data(),
               int32_t(weights.cols() * weights.rows()) *
               sizeof(float));

        return true;
    }
    catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to get mask: {}", e.what());
    }
}

bool ActorCreationAPI::SetFittingMask(float* InVertexWeights, FittingMaskType InMaskType) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(m->faceFitting, false, "face fitting has not been initialized");
        CHECK_OR_RETURN(m->teethFitting, false, "teeth fitting has not been initialized");

        int32_t numVertices = (int32_t)m->faceFitting->CurrentDeformedVertices().cols();
        if (InMaskType == FittingMaskType::TEETH) {
            numVertices = (int32_t)m->teethFitting->CurrentDeformedVertices().cols();
        }

        Eigen::VectorXf weightsMap = Eigen::Map<const Eigen::VectorXf >(
            (const float*)InVertexWeights,
            numVertices).template cast<float>();

        m->masks[InMaskType] = VertexWeights<float>(weightsMap);

        return true;
    }
    catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to set mask: {}", e.what());
    }

}

}
