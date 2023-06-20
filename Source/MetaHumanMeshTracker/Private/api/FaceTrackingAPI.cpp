// Copyright Epic Games, Inc. All Rights Reserved.

#include "FaceTrackingAPI.h"
#include "Common.h"
#include "Internals/OpenCVCamera2MetaShapeCamera.h"
#include "Internals/StereoReconstructionHelper.h"
#include <carbon/utils/ObjectPool.h>
#include <rigposebasedsolver/RigPoseBasedSolverData.h>
#include <rigposebasedsolver/RigPoseBasedSolverTrainer.h>
#include <rigposebasedsolver/RigGenerateTrainingData.h>
#include <rigposebasedsolver/FramePoseData.h>
#include <nls/geometry/MetaShapeCamera.h>
#include <nls/rig/RigLogicDNAResource.h>
#include <nrr/landmarks/LandmarkConfiguration.h>
#include <nrr/landmarks/LandmarkInstance.h>
//TODO commented out until optical flow TPS issue is fixed
//#include <Internals/OpticalFlowTrackingSupport.h>
#include <reconstruction/MultiCameraSetup.h>
#include <tracking/FaceTracking.h>
#include <tracking/LandmarkTriangulation.h>
#include <tracking/rt/PCARigCreator.h>

#include <cstring>
#include <filesystem>

using namespace epic::nls;
using namespace epic::carbon;
using namespace pma;

namespace titan::api {
    static std::shared_ptr<const LandmarkInstance<float, 2> > CreateLandmarkInstanceFT(const std::map<std::string,
                                                                                      FaceTrackingLandmarkData>& perCameraLandmarkData,
                                                                                      const std::map<std::string, std::vector<std::string> >& curvesToMerge,
                                                                                      const MetaShapeCamera<float>& camera) {
        std::shared_ptr<LandmarkConfiguration> landmarkConfiguration = std::allocate_shared<LandmarkConfiguration>(PolyAllocator<LandmarkConfiguration>(
                                                                                                                       MEM_RESOURCE));
        for (const auto& [landmarkOrCurveName, faceTrackingLandmarkData] : perCameraLandmarkData) {
            if (faceTrackingLandmarkData.NumPoints() == 1) {
                landmarkConfiguration->AddLandmark(landmarkOrCurveName);
            } else if (faceTrackingLandmarkData.NumPoints() > 1) {
                landmarkConfiguration->AddCurve(landmarkOrCurveName, faceTrackingLandmarkData.NumPoints());
            } else {
                CARBON_CRITICAL("at least one point per landmark/curve required");
            }
        }

        Eigen::Matrix<float, 2, -1> landmarks(2, landmarkConfiguration->NumPoints());
        Eigen::Vector<float, -1> confidence(landmarkConfiguration->NumPoints());
        for (const auto& [landmarkOrCurveName, faceTrackingLandmarkData] : perCameraLandmarkData) {
            if (faceTrackingLandmarkData.PointsDimension() != 2) {
                CARBON_CRITICAL("input landmark data is not in 2D.");
            }
            if (faceTrackingLandmarkData.NumPoints() == 1) {
                int index = landmarkConfiguration->IndexForLandmark(landmarkOrCurveName);
                landmarks(0, index) = faceTrackingLandmarkData.PointsData()[0];
                landmarks(1, index) = faceTrackingLandmarkData.PointsData()[1];
                confidence[index] = faceTrackingLandmarkData.ConfidenceData()[0];
            } else {
                const std::vector<int>& indices = landmarkConfiguration->IndicesForCurve(landmarkOrCurveName);
                for (int32_t i = 0; i < faceTrackingLandmarkData.NumPoints(); ++i) {
                    int index = indices[i];
                    landmarks(0, index) = faceTrackingLandmarkData.PointsData()[2 * i + 0];
                    landmarks(1, index) = faceTrackingLandmarkData.PointsData()[2 * i + 1];
                    confidence[index] = faceTrackingLandmarkData.ConfidenceData()[i];
                }
            }
        }
        for (const auto& [mergedCurve, listOfCurves] : curvesToMerge) {
            landmarkConfiguration->MergeCurves(listOfCurves, mergedCurve, landmarks,  /*ignoreMissingCurves=*/ true);
        }

        std::shared_ptr<LandmarkInstance<float, 2> > landmarkInstance = std::allocate_shared<LandmarkInstance<float, 2> >(PolyAllocator<LandmarkInstance<float, 2> >(
                                                                                                                        MEM_RESOURCE),
                                                                                                                        landmarks,
                                                                                                                        confidence);
        landmarkInstance->SetLandmarkConfiguration(landmarkConfiguration);
        for (int i = 0; i < landmarkInstance->NumLandmarks(); ++i) {
            const Eigen::Vector2f pix = camera.Undistort(landmarkInstance->Points().col(i));
            landmarkInstance->SetLandmark(i, pix, landmarkInstance->Confidence()[i]);
        }
        return landmarkInstance;
    }

    struct FrameData {
        std::map<std::string, std::shared_ptr<const LandmarkInstance<float, 2> > > perCameraLandmarkData;
        std::vector<std::shared_ptr<const DepthmapData<float> > > stereoReconstructions;
        std::map<std::string, std::shared_ptr<const MeshFlowData> > m_temporalFlow;

        void Clear() {
            perCameraLandmarkData.clear();
            stereoReconstructions.clear();
            m_temporalFlow.clear();
        }
    };


    struct FaceTrackingAPI::Private {
        std::string configurationDirectory;
        std::unique_ptr<FaceTracking> faceTracking;
		// TODO commented out until optical flow TPS issue is fixed
        //std::unique_ptr<OpticalFlowTrackingSupport, std::function<void(OpticalFlowTrackingSupport*)> > optFlow;
        std::unique_ptr< epic::nls::rt::PCARigCreator> pcaRigCreator;
        std::string  pcaModelConfig;
        StereoReconstructionHelper stereoReconstructionHelper;
        std::vector<std::pair<std::string, std::string> > stereoReconstructionPairs;

        FrameData frameData;
        std::vector<std::shared_ptr<const FaceTrackingState> > trackingStates;
        std::map<std::string, Camera<float> > cameras;

        std::shared_ptr<epic::nls::VulkanDevice> vulkanDevice;

        int frameStart = 0;
        bool isRealtime = false;
        bool useOptFlow = false;
        bool runCoarseAlign = true;

        //! pool for depthmap data to prevent constant reallocation
        epic::carbon::ObjectPool<DepthmapData<float>> depthmapDataPool;

        std::shared_ptr<TaskThreadPool> globalThreadPool = TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/ true);
    };


    FaceTrackingAPI::FaceTrackingAPI()
        : m(new Private()) {
    }

    FaceTrackingAPI::~FaceTrackingAPI() {
        delete m;
    }

    bool FaceTrackingAPI::Init(const std::string& InConfigurationDirectory, OpticalFlowData /*InOptFlowData*/, bool bIsRealtime) {
        try {
            RESET_ERROR;

            m->configurationDirectory = InConfigurationDirectory;

            CHECK_OR_RETURN(VulkanLoader::Init(), false, "failed to initialize vulkan");

            auto vulkanInstance = VulkanInstance::CreateInstance(false, {});
            CHECK_OR_RETURN(vulkanInstance, false, "could not create vulkan instance");
            m->vulkanDevice = VulkanDevice::CreateDevice(vulkanInstance, nullptr);
            CHECK_OR_RETURN(m->vulkanDevice, false, "could not create vulkan device");

            CHECK_OR_RETURN(m->stereoReconstructionHelper.Init(m->vulkanDevice),
                            false,
                            "failed to initialize stereo reconstruction");

            TemplateDescription templateDescription;
            CHECK_OR_RETURN(templateDescription.Load(InConfigurationDirectory + "/template_description.json"),
                            false,
                            "failed to load template description");
            std::unique_ptr<FaceTracking> faceTracking = std::make_unique<FaceTracking>();
            CHECK_OR_RETURN(faceTracking->Init(templateDescription, InConfigurationDirectory + "/configuration.json", ""),
                            false,
                            "failed to initialize face tracking");
            m->pcaModelConfig = InConfigurationDirectory + "/pca_from_dna_configuration.json";

            m->faceTracking = std::move(faceTracking);
            m->isRealtime = bIsRealtime;

			// TODO commented out until optical flow TPS issue is fixed
            /*m->optFlow.reset();
            m->useOptFlow = InOptFlowData.bUseOpticalFlow;
            if (m->useOptFlow) {
                m->optFlow = pma::UniqueInstance<OpticalFlowTrackingSupport>::with(MEM_RESOURCE).create();

                const std::string flowMaskName = m->faceTracking->GetFlowMask(FaceTrackingFittingMode::FINE);
                const std::vector<int> vIDs =
                    m->faceTracking->GetTemplateDescription().GetVertexWeights(flowMaskName).NonzeroVertices();

                CHECK_OR_RETURN(m->optFlow->Init(m->vulkanDevice, InOptFlowData, m->frameStart, vIDs),
                                false,
                                "failed to initialize optical flow");
            }*/
            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to initialize: {}", e.what());
        }
    }

    bool FaceTrackingAPI::LoadDNA(const std::string& InDNAPath) {
        try {
            RESET_ERROR;

            std::shared_ptr<const RigLogicDNAResource> dnaResource = RigLogicDNAResource::LoadDNA(InDNAPath, /*retain=*/false);
            if (!dnaResource) {
                LOG_ERROR("failed to open dnafile {}", InDNAPath);
                return false;
            }

            return LoadDNA(dnaResource->Stream());
        }
        catch (const std::exception& e) {
            HANDLE_EXCEPTION("failed to open dnafile {}", e.what());
        }
    }

    bool FaceTrackingAPI::LoadDNA(dna::StreamReader* InDNAStream) {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");

            std::string solverDefinitionsFile = m->configurationDirectory + "/solver_definitions.json";
            if (!std::filesystem::exists(solverDefinitionsFile)) {
                solverDefinitionsFile = "";
            }
            if (!m->faceTracking->LoadTrackingRig(InDNAStream, solverDefinitionsFile)) {
                LOG_ERROR("could not load tracking rig");
                return false;
            }

            // now try and load the PCA rig config
            std::unique_ptr<epic::nls::rt::PCARigCreator> pcaRigCreator = std::make_unique<epic::nls::rt::PCARigCreator>(
                m->faceTracking->GetTrackingRig().GetRig());
            CHECK_OR_RETURN(pcaRigCreator->LoadConfigFile(m->pcaModelConfig), false, "failed to load PCA rig creator config file");
            m->pcaRigCreator = std::move(pcaRigCreator);

            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to set dna: {}", e.what());
        }
    }


    bool FaceTrackingAPI::LoadPCARig(dna::StreamReader* InOutPCARigStream)
    {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");

            if (!m->faceTracking->LoadPCARig(InOutPCARigStream)) {
                LOG_ERROR("could not load PCA rig");
                return false;
            }

            return true;
        }
        catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to load PCA rig: {}", e.what());
        }
    }

    bool FaceTrackingAPI::SavePCARig(dna::StreamWriter* InOutPCARigStream)
    {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");

            if (!m->faceTracking->SavePCARig(InOutPCARigStream)) {
                LOG_ERROR("could not save PCA rig");
                return false;
            }

            return true;
        }
        catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to save PCA rig: {}", e.what());
        }
    }

    bool FaceTrackingAPI::LoadPoseBasedSolvers(const std::vector<std::string>& InPoseBasedSolverFilenames)
    {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");

            m->faceTracking->LoadPoseBasedSolversData(InPoseBasedSolverFilenames);

            return true;
        }
        catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to load pose-based solvers: {}", e.what());
        }
    }

    bool FaceTrackingAPI::GetPoseBasedSolvers(std::vector<char>& OutMemoryBuffer) const
    {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");
            OutMemoryBuffer.clear();
            dlib::serialize(OutMemoryBuffer) << m->faceTracking->GetPoseBasedSolversData();
            return true;
        }
        catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to get pose-based solvers: {}", e.what());
        }
    }


    bool FaceTrackingAPI::SetPoseBasedSolvers(std::vector<char>& InMemoryBuffer)
    {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");
            std::vector<epic::rigposebasedsolver::RigPoseBasedSolverData> solverData;
            dlib::deserialize(InMemoryBuffer) >> solverData;
            m->faceTracking->SetPoseBasedSolversData(solverData);
            return true;
        }
        catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to set pose-based solvers: {}", e.what());
        }
    }

    bool FaceTrackingAPI::TrainSolverModels(std::function<void(float)> InProgressCallback, std::function<bool(void)> InIsCancelledCallback)
    {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->pcaRigCreator, false, "PCA rig creator has not been initialized");
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");
            CHECK_OR_RETURN(m->faceTracking->HasTrackingRig(), false, "no rig has been set");

            // re-train each of the solvers using the best settings from the existing solvers, for the current DNA
            // TODO we may want to add a finer grained progress indicator here ... the current one is very coarse-grained
            const auto& solversData = m->faceTracking->GetPoseBasedSolversData();
            std::vector<epic::rigposebasedsolver::RigPoseBasedSolverData> retrainedSolversData = solversData;
            InProgressCallback(0.0);
            const float maxTrainingTimeForSolvers = 0.90;
            float progress = 0;
            float progressStep = maxTrainingTimeForSolvers / (solversData.size() * 2);
            for (size_t i = 0; i < solversData.size(); i++)
            {
                // generate the training data from the current tracking rig
                epic::rigposebasedsolver::RigGenerateTrainingData generateTrainingData;
                std::vector< std::map<std::string, std::map<int, double>> > animationData;
                generateTrainingData.Init(m->faceTracking->GetTrackingRig().GetRig(), solversData[i].animationData,
                    solversData[i].existingSolver.RigMeshVertexIdData(),
                    solversData[i].poseBasedSolverTrainingDataParameters,
                    static_cast<unsigned>(solversData[i].existingSolver.ShapeAlignmentTypes().size()));
                std::vector<epic::rigposebasedsolver::FramePoseData> poseData;
                generateTrainingData.GenerateTrainingData(poseData, InProgressCallback, progress, progressStep, InIsCancelledCallback);
                progress += progressStep;
                InProgressCallback(progress);
                if (InIsCancelledCallback())
                {
                    return true;
                }

                // retrain the solver
                std::vector<cm::modular_solver_trainer<double, 3>::training_delta> trainingDeltas;
                epic::rigposebasedsolver::RigPoseBasedSolverTrainer::TrainSolverFromPreviousBestParameters(
                    solversData[i].existingSolver, poseData, trainingDeltas, retrainedSolversData[i].existingSolver);
                progress += progressStep;
                InProgressCallback(progress);
                if (InIsCancelledCallback())
                {
                    return true;
                }
            }
            m->faceTracking->SetPoseBasedSolversData(retrainedSolversData);

            // train and set the PCA model
            m->pcaRigCreator->Create();
            pma::ScopedPtr<dna::MemoryStream> stream = pma::makeScoped<trio::MemoryStream>();
            pma::AlignedMemoryResource memRes;
            pma::ScopedPtr<dna::StreamWriter> writer = pma::makeScoped<dna::StreamWriter>(stream.get(), &memRes);
            m->pcaRigCreator->GetPCARig().SaveAsDNA(writer.get());
            pma::ScopedPtr<dna::StreamReader> reader = pma::makeScoped<dna::StreamReader>(stream.get(), dna::DataLayer::All);
            reader->read();
            LoadPCARig(reader.get());

            InProgressCallback(1.0);
            return true;
        }
        catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to train the pose-based solvers: {}", e.what());
        }
    }

    bool FaceTrackingAPI::SetCameras(const std::map<std::string, OpenCVCamera>& InCameras) {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");

            std::vector<MetaShapeCamera<float> > metaCameras;
            m->cameras.clear();
            for (const auto& [cameraName, opencvCamera] : InCameras) {
                metaCameras.emplace_back(OpenCVCamera2MetaShapeCamera<float>(cameraName.c_str(), opencvCamera));

                // Deliberately slicing object!
                const Camera<float>& msCamera = metaCameras.back();

                m->cameras.insert({cameraName, msCamera});
            }
            MultiCameraSetup<float> cameraSetup;
            cameraSetup.Init(metaCameras);

            m->faceTracking->SetCameraSetup(cameraSetup);

            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to set cameras: {}", e.what());
        }
    }

    bool FaceTrackingAPI::SetCameraRanges(const std::map<std::string, std::pair<float, float> >& InCameraRanges) {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");

            MultiCameraSetup<float> cameraSetup = m->faceTracking->GetCameraSetup();
            cameraSetup.SetCameraRanges(InCameraRanges);
            m->faceTracking->SetCameraSetup(cameraSetup);

            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to set camera ranges: {}", e.what());
        }
    }

    bool FaceTrackingAPI::SetStereoCameraPairs(const std::vector<std::pair<std::string,
                                                                           std::string> >& InStereoReconstructionPairs) {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");

            for (const auto& [cameraLeft, cameraRight] : InStereoReconstructionPairs) {
                CHECK_OR_RETURN(m->faceTracking->GetCameraSetup().HasCamera(cameraLeft),
                                false,
                                "camera {} does not exist",
                                cameraLeft);
                CHECK_OR_RETURN(m->faceTracking->GetCameraSetup().HasCamera(cameraRight),
                                false,
                                "camera {} does not exist",
                                cameraRight);
            }
            m->stereoReconstructionPairs = InStereoReconstructionPairs;
            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to set camera ranges: {}", e.what());
        }
    }

    bool FaceTrackingAPI::ResetTrack(int32_t InFrameStart, int32_t InFrameEnd, OpticalFlowData /*InOptFlowData*/) {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");
            CHECK_OR_RETURN(m->faceTracking->HasTrackingRig(), false, "no rig has been set");
            CHECK_OR_RETURN(!m->isRealtime || (InFrameStart == 0 && InFrameEnd == 1),
                            false,
                            "Invalid frame numbers passed for Realtime flow");

            m->runCoarseAlign = true;
            m->trackingStates = std::vector<std::shared_ptr<const FaceTrackingState> >(InFrameEnd - InFrameStart);
            m->frameStart = InFrameStart;
            m->frameData.stereoReconstructions.clear();

			// TODO commented out until optical flow TPS issue is fixed
            /* m->optFlow.reset();
            m->useOptFlow = InOptFlowData.bUseOpticalFlow;
            if (m->useOptFlow) {
                m->optFlow = pma::UniqueInstance<OpticalFlowTrackingSupport>::with(MEM_RESOURCE).create();

                const std::string flowMaskName = m->faceTracking->GetFlowMask(FaceTrackingFittingMode::FINE);
                const std::vector<int> vIDs =
                    m->faceTracking->GetTemplateDescription().GetVertexWeights(flowMaskName).NonzeroVertices();

                CHECK_OR_RETURN(m->optFlow->Init(m->vulkanDevice, InOptFlowData, m->frameStart, vIDs),
                                false,
                                "failed to initialize optical flow");
            }*/

            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to reset track: {}",
                             e.what());
        }
    }

    bool FaceTrackingAPI::SetInputData(const std::map<std::string, const unsigned char*>& InImageDataPerCamera,
                                       const std::map<std::string, const std::map<std::string,
                                                                                  FaceTrackingLandmarkData> >& InLandmarksDataPerCamera,
                                       const std::map<std::string, const float*>& InDepthMaps,
                                       int InEndLevel) {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");
            CHECK_OR_RETURN(m->faceTracking->HasTrackingRig(), false, "no rig has been set");
            CHECK_OR_RETURN(m->stereoReconstructionHelper.IsInitialized(), false,
                            "stereo reconstruction has not been initialized");
            CHECK_OR_RETURN(InEndLevel >= 0 && InEndLevel < 4, false,
                            "invalid endLevel parametar provided. Allowable range [0,4)");

            const MultiCameraSetup<float>& cameraSetup = m->faceTracking->GetCameraSetup();

            // convert landmark input
            m->frameData.Clear();
            for (const auto& [cameraName, perCameraLandmarkData] : InLandmarksDataPerCamera) {
                CHECK_OR_RETURN(cameraSetup.HasCamera(cameraName), false, "no camera {}", cameraName);
                m->frameData.perCameraLandmarkData[cameraName] = CreateLandmarkInstanceFT(perCameraLandmarkData,
                                                                                        m->faceTracking->CurvesToMerge(),
                                                                                        cameraSetup.GetCamera(cameraName));
            }

            // perform stereo reconstruction
            m->stereoReconstructionHelper.SetCameraSetup(cameraSetup);
            if (InDepthMaps.empty()) {
                for (const auto& [cameraLeftName, cameraRightName] : m->stereoReconstructionPairs) {
                    LOG_INFO("creating stereo for {} {}", cameraLeftName, cameraRightName);
                    CHECK_OR_RETURN(cameraSetup.HasCamera(cameraLeftName), false, "no camera {}", cameraLeftName);
                    CHECK_OR_RETURN(cameraSetup.HasCamera(cameraRightName), false, "no camera {}", cameraRightName);
                    const MetaShapeCamera<float>& cameraLeft = cameraSetup.GetCamera(cameraLeftName);
                    const MetaShapeCamera<float>& cameraRight = cameraSetup.GetCamera(cameraRightName);
                    auto leftImageDataIt = InImageDataPerCamera.find(cameraLeftName);
                    auto rightImageDataIt = InImageDataPerCamera.find(cameraRightName);
                    CHECK_OR_RETURN(leftImageDataIt != InImageDataPerCamera.end(),
                                    false,
                                    "missing input data for camera {}",
                                    cameraLeftName);
                    CHECK_OR_RETURN(rightImageDataIt != InImageDataPerCamera.end(),
                                    false,
                                    "missing input data for camera {}",
                                    cameraRightName);
                    ConstImageView imgViewLeft(leftImageDataIt->second, cameraLeft.Width(), cameraLeft.Height(),
                                               PixelFormat::RGBA, PixelDepth::UINT8);
                    ConstImageView imgViewRight(rightImageDataIt->second, cameraRight.Width(), cameraRight.Height(),
                                                PixelFormat::RGBA, PixelDepth::UINT8);
                    m->frameData.stereoReconstructions.push_back(m->stereoReconstructionHelper.Reconstruct(cameraLeftName,
                                                                                                           cameraRightName,
                                                                                                           imgViewLeft,
                                                                                                           imgViewRight,
                                                                                                           InEndLevel));
                }
            } else {
                // Todo - undistortion
                PolyAllocator<DepthmapData<float> > polyAllocator{MEM_RESOURCE};

                for (const auto& [cameraName, depthMaps] : InDepthMaps) {
                    CHECK_OR_RETURN(cameraSetup.HasCamera(cameraName), false, "no camera {}", cameraName);

                    std::shared_ptr<DepthmapData<float>> depthmapData = m->depthmapDataPool.Aquire();
                    const Camera<float> camera = cameraSetup.GetCamera(cameraName);
                    depthmapData->Create(camera, depthMaps);
                    m->frameData.stereoReconstructions.push_back(depthmapData);
                }
            }
            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to Set input data: {}", e.what());
        }
    }

    bool FaceTrackingAPI::Track(int32_t InFrameNumber, const std::map<std::string, std::pair<float*,
                                                                                             float*> >& /*InFlowCameraImages*/,
                                                                                             bool bInUseFastSolver) {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");
            CHECK_OR_RETURN(m->faceTracking->HasTrackingRig() && (!bInUseFastSolver || (m->faceTracking->HasPCAModel() && m->faceTracking->HasPoseBasedSolverModel())),
                false, "for normal solving, there must be a tracking rig set, and if we are using fast solving, we must have a PCA model and pose-based solver model(s) set");
            CHECK_OR_RETURN(m->faceTracking->GetCameraSetup().GetCameras().size() >= 2,
                            false,
                            "at least two cameras have to be set");
            CHECK_OR_RETURN(m->stereoReconstructionHelper.IsInitialized(), false,
                            "stereo reconstruction has not been initialized");
            CHECK_OR_RETURN(m->trackingStates.size() > 0, false, "track needs to be reset first");
            CHECK_OR_RETURN(!m->isRealtime || InFrameNumber == 0, false, "frame number is invalid");
            CHECK_OR_RETURN(InFrameNumber >= m->frameStart, false, "frame number is invalid");
            CHECK_OR_RETURN(InFrameNumber < m->frameStart + int32_t(m->trackingStates.size()), false, "frame number is invalid");

            // track the data
            m->faceTracking->SetLandmarks(m->frameData.perCameraLandmarkData);
            m->faceTracking->SetDepthmaps(m->frameData.stereoReconstructions);

            // in offline mode, use the data from the state if it is available
            if (!m->isRealtime && m->trackingStates[InFrameNumber - m->frameStart]) {
                m->faceTracking->SetFaceTrackingState(*m->trackingStates[InFrameNumber - m->frameStart]);
            }

            if (m->runCoarseAlign) {
                // no tracking state so far, so do global rigid alignment
                m->faceTracking->EstimateRigidUsingLandmarks();
                m->runCoarseAlign = false;
            }

            if (bInUseFastSolver)
            {
                LOG_INFO("performing rigid, PCA mesh-fitting and pose-based solve");
                m->faceTracking->FitRigLogic(FaceTrackingFittingMode::RIGID, 0);
                m->faceTracking->FitRigLogic(FaceTrackingFittingMode::PCA, 0);
                m->faceTracking->PerformPoseBasedSolve();
            }
            else
            {
                // Execute opt flow
				// TODO commented out until optical flow TPS issue is fixed
				/*
                if (m->useOptFlow && m->optFlow) {
                    m->optFlow->Execute(InFrameNumber, m->trackingStates, InFlowCameraImages, *m->faceTracking);
                }
				*/

                LOG_INFO("processing rigid");
                m->faceTracking->FitRigLogic(FaceTrackingFittingMode::RIGID, 0);  // the control set index is not used for rigid
                                                                                    // alignment

                const std::vector<std::string> controlSetNames = m->faceTracking->GetTrackingRig().GetSolveControlSetNames();
                const int controlSetsToProcess = std::max<int>(1, static_cast<int>(controlSetNames.size()) - 1);  // process
                                                                                                                    // everything
                                                                                                                    // besides the
                                                                                                                    // last control
                                                                                                                    // set which is
                                                                                                                    // All controls.

                if (controlSetsToProcess > 1) {
                    // if we process multiple levels of rig logic, then we should start the process by reseting the following
                    // levels and only initialize the first level
                    m->faceTracking->ResetRigSolveControlValues(1,  /*resetNextSets=*/ true);
                }
                for (int controlSetIndex = 0; controlSetIndex < controlSetsToProcess; ++controlSetIndex) {
                    LOG_INFO("processing rig logic {}...", controlSetNames[controlSetIndex]);
                    m->faceTracking->FitRigLogic(FaceTrackingFittingMode::RIGLOGIC, controlSetIndex);
                }
            }

            PolyAllocator<FaceTrackingState> polyAlloc{MEM_RESOURCE};
            m->trackingStates[InFrameNumber - m->frameStart] = std::allocate_shared<FaceTrackingState>(polyAlloc, m->faceTracking->GetFaceTrackingState());

            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to track: {}", e.what());
        }
    }

    bool FaceTrackingAPI::Track(int32_t InFrameNumber,
                                const std::map<std::string, const unsigned char*>& InImageDataPerCamera,
                                const std::map<std::string, const std::map<std::string,
                                                                           FaceTrackingLandmarkData> >& InLandmarksDataPerCamera,
                                const std::map<std::string, const float*>& InDepthMaps,
                                const std::map<std::string, std::pair<float*, float*> >& InFlowCameraImages,
                                int InEndLevel,
                                bool bInUseFastSolver) {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(SetInputData(InImageDataPerCamera, InLandmarksDataPerCamera, InDepthMaps, InEndLevel),
                            false,
                            "could not set input data");
            CHECK_OR_RETURN(Track(InFrameNumber, InFlowCameraImages, bInUseFastSolver), false, "could not track");
            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to track: {}", e.what());
        }
    }

    bool FaceTrackingAPI::GetTrackingState(int32_t InFrameNumber, float* OutHeadPose, std::map<std::string,
                                                                                               float>& OutRawControls) {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(m->faceTracking, false, "face tracking has not been initialized");
            CHECK_OR_RETURN(m->faceTracking->HasTrackingRig(), false, "no rig has been set");
            CHECK_OR_RETURN(m->faceTracking->GetTrackingRig().GetRigLogic(), false, "no rig has been set");
            CHECK_OR_RETURN(m->trackingStates.size() > 0, false, "track needs to be reset first");
            CHECK_OR_RETURN(InFrameNumber >= m->frameStart, false, "frame number is invalid");
            CHECK_OR_RETURN(InFrameNumber < m->frameStart + int32_t(m->trackingStates.size()), false, "frame number is invalid");
            CHECK_OR_RETURN(OutHeadPose, false, "OutHeadPose is invalid");

            std::shared_ptr<const FaceTrackingState> trackingState = m->trackingStates[InFrameNumber - m->frameStart];
            CHECK_OR_RETURN(trackingState, false, "no valid tracking state");
            Eigen::Map<Eigen::Matrix4f> headPoseMapped(OutHeadPose);
            headPoseMapped = trackingState->m_mesh2scan.Matrix();
            OutRawControls.clear();
            const std::vector<std::string>& rawControlNames = m->faceTracking->GetTrackingRig().GetRawControlNames();
            const Eigen::VectorXf& guiControls = trackingState->m_postProcessedGuiControls ? *trackingState->m_postProcessedGuiControls : trackingState->m_guiControls;
            const Eigen::VectorXf rawControls = m->faceTracking->GetTrackingRig().GetRigLogic()->EvaluateRawControls(DiffData<float>(guiControls)).Value();
            CHECK_OR_RETURN(rawControlNames.size() == size_t(rawControls.size()), false, "raw control size is invalid");
            for (size_t i = 0; i < rawControlNames.size(); ++i) {
                OutRawControls[rawControlNames[i]] = rawControls[i];
            }

            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to get tracking state: {}", e.what());
        }
    }

    int32_t FaceTrackingAPI::GetNumStereoPairs() const {
        return int32_t(m->frameData.stereoReconstructions.size());
    }

    bool FaceTrackingAPI::CreateMeshForStereoReconstructionAndCallback(int32_t InStereoPairIndex,
                                                                       std::function<void(int32_t InNumberOfPoints,
                                                                                          int32_t InNumberOfTriangles,
                                                                                          const float* InVerticesData,
                                                                                          const int32_t* InTriangleIndices)> InReconstructionCallBack)
    const {
        try {
            RESET_ERROR;
            CHECK_OR_RETURN(InStereoPairIndex >= 0 && InStereoPairIndex < GetNumStereoPairs(), false, "invalid index");
            std::shared_ptr<const DepthmapData<float> > depthData = m->frameData.stereoReconstructions[InStereoPairIndex];
            CHECK_OR_RETURN(bool(depthData), false, "invalid nullptr in depth data");

            const Camera<float>& depthCamera = depthData->camera;
            const Eigen::Matrix<float, 4, -1>& depthAndNormals = depthData->depthAndNormals;
            const int w = depthCamera.Width();
            const int h = depthCamera.Height();
            const int maxNumVertices = w * h;
            const int maxNumTriangles = (w - 1) * (h - 1) * 2;

            const Eigen::Matrix3f intrinsicsInverse = depthCamera.Intrinsics().inverse();
            const Affine<float, 3, 3> extrinsicsInverse = depthCamera.Extrinsics().Inverse();
            const Eigen::Matrix3f RInverse = extrinsicsInverse.Linear();
            const Eigen::Vector3f tInverse = extrinsicsInverse.Translation();
            const Eigen::Matrix3f temp = RInverse * intrinsicsInverse;

            Eigen::Matrix<float, 3, -1> vertices(3, maxNumVertices);
            int32_t numVertices = 0;
            std::vector<int32_t> depthIndices(w* h);
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    const int pixelIndex = y * w + x;
                    const float depth = depthAndNormals(0, pixelIndex);
                    if (depth > 0) {
                        vertices.col(numVertices) = temp * Eigen::Vector3f(x + 0.5f, y + 0.5f, 1.0f) * depth + tInverse;
                        depthIndices[pixelIndex] = numVertices++;
                    } else {
                        depthIndices[pixelIndex] = -1;
                    }
                }
            }

            Eigen::Matrix<int32_t, 3, -1> triangles(3, maxNumTriangles);
            int32_t numTriangles = 0;
            for (int y = 0; y < h - 1; ++y) {
                for (int x = 0; x < w - 1; ++x) {
                    const int32_t index00 = depthIndices[y * w + x];
                    const int32_t index01 = depthIndices[y * w + x + 1];
                    const int32_t index10 = depthIndices[(y + 1) * w + x];
                    const int32_t index11 = depthIndices[(y + 1) * w + x + 1];
                    if ((index00 >= 0) && (index10 >= 0) && (index01 >= 0)) {
                        triangles.col(numTriangles++) = Eigen::Vector3<int32_t>(index00, index10, index01);
                    }
                    if ((index10 >= 0) && (index11 >= 0) && (index01 >= 0)) {
                        triangles.col(numTriangles++) = Eigen::Vector3<int32_t>(index10, index11, index01);
                    }
                }
            }

            InReconstructionCallBack(numVertices, numTriangles, (const float*)vertices.data(), (const int32_t*)triangles.data());

            return true;
        } catch (const std::exception& e) {
            HANDLE_EXCEPTION("failure to mesh stereo reconstruction: {}", e.what());
        }
    }
}  // namespace titan::api
