// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanFaceTracker.h"
#include "api/FaceTrackingAPI.h"
#include "FrameTrackingContourData.h"
#include "Math/TransformCalculus3D.h"
#include "Async/Async.h"
#include "DNAAsset.h"
#include "DNAReader.h"
#include "FReader.h"
#include "dna/StreamWriter.h"

namespace UE
{
    namespace Wrappers
    {

        static void ConvertOpenCVToUE(const FMatrix& InRotationOpenCV, const FVector& InTranslationOpenCV, FRotator& OutRotatorUE, FVector& OutTranslationUE)
        {
            // this function converts a rotation and translation from OpenCV coordinate frame, where we have
            // right-handed coordinate system, x right, y down to
            // UE coordinate system, where we have left-handed coordinate system, y right, z up

            FVector XAxisUE = FVector(InRotationOpenCV.M[2][2], InRotationOpenCV.M[2][0], -InRotationOpenCV.M[2][1]); // comes from Z axis in rotation matrix 
            FVector YAxisUE = FVector(InRotationOpenCV.M[0][2], InRotationOpenCV.M[0][0], -InRotationOpenCV.M[0][1]); // comes from X axis in rotation matrix
            FMatrix RotationUE = FRotationMatrix::MakeFromXY(XAxisUE, YAxisUE);

            // again swap the elements in the position to get the result in UE coordinate space
            OutTranslationUE = FVector(InTranslationOpenCV.Z, InTranslationOpenCV.X, -InTranslationOpenCV.Y);
            OutRotatorUE = TransformConverter<FRotator>::Convert(RotationUE);
        }


        struct FMetaHumanFaceTracker::Private
        {
            titan::api::FaceTrackingAPI API;
            uint32 State{ TrackerState::INIT_FAILED | TrackerState::LOAD_DNA_FAILED | TrackerState::SET_CAMERAS_FAILED |
                TrackerState::SET_CAMERA_RANGES_FAILED | TrackerState::SET_STEREO_CAMERA_PAIRS_FAILED };
        };

        FMetaHumanFaceTracker::FMetaHumanFaceTracker()
        {
            Impl = MakePimpl<Private>();
        }

        bool FMetaHumanFaceTracker::Init(const FString& InConfigurationDirectory, const FTrackerOpticalFlowConfiguration& InOptFlowConfig)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            titan::api::OpticalFlowData OpticalFlowConfiguration = { InOptFlowConfig.bUseOpticalFlow, InOptFlowConfig.bUseConfidence, InOptFlowConfig.bUseForwardFlow };
            bool bResult = Impl->API.Init(TCHAR_TO_ANSI(*InConfigurationDirectory), OpticalFlowConfiguration, true);
            if (bResult)
            {
                Impl->State &= ~TrackerState::INIT_FAILED;
            }
            else
            {
                Impl->State |= TrackerState::INIT_FAILED;
            }
            return bResult;
        }

        bool FMetaHumanFaceTracker::LoadDNA(const FString& InDNAFile)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            bool bResult = Impl->API.LoadDNA(TCHAR_TO_ANSI(*InDNAFile));
            if (bResult)
            {
                Impl->State &= ~TrackerState::LOAD_DNA_FAILED;
            }
            else
            {
                Impl->State |= TrackerState::LOAD_DNA_FAILED;
            }
            return bResult;
        }

		bool FMetaHumanFaceTracker::LoadDNA(UDNAAsset* InDNAAsset)
		{
			const FScopeLock ScopeLock(&AccessMutex);

			dna::FReader Reader;
			Reader.Geometry = InDNAAsset->GetGeometryReader()->Unwrap();
			Reader.Behaviour = InDNAAsset->GetBehaviorReader()->Unwrap();

			bool bResult = Impl->API.LoadDNA(&Reader);
			if (bResult)
			{
				Impl->State &= ~TrackerState::LOAD_DNA_FAILED;
			}
			else
			{
				Impl->State |= TrackerState::LOAD_DNA_FAILED;
			}
			return bResult;
		}

        bool FMetaHumanFaceTracker::LoadPCARig(dna::StreamReader* InOutPCARigStream)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            return Impl->API.LoadPCARig(InOutPCARigStream);
        }

        bool FMetaHumanFaceTracker::SavePCARig(dna::StreamWriter* InOutPCARigStream)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            return Impl->API.SavePCARig(InOutPCARigStream);
        }

		bool FMetaHumanFaceTracker::SetPCARig(const TArray<uint8>& InMemoryBuffer)
		{
			const FScopeLock ScopeLock(&AccessMutex);

			pma::ScopedPtr<dna::MemoryStream> Stream = pma::makeScoped<dna::MemoryStream>();
			Stream->write((const char*)InMemoryBuffer.GetData(), (InMemoryBuffer.Num() * sizeof(uint8)) / sizeof(char));
		
			pma::ScopedPtr<dna::StreamReader> Reader = pma::makeScoped<dna::StreamReader>(Stream.get());
			Reader->read();

			if (Impl->API.LoadPCARig(Reader.get()))
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		bool FMetaHumanFaceTracker::GetPCARig(TArray<uint8>& OutMemoryBuffer)
		{
			const FScopeLock ScopeLock(&AccessMutex);

			pma::ScopedPtr<dna::MemoryStream> Stream = pma::makeScoped<trio::MemoryStream>();
			pma::ScopedPtr<dna::StreamWriter> Writer = pma::makeScoped<dna::StreamWriter>(Stream.get());

			if (Impl->API.SavePCARig(Writer.get()))
			{
				OutMemoryBuffer.SetNumUninitialized((Stream->size() * sizeof(char)) / sizeof(uint8));
				Stream->read((char*)OutMemoryBuffer.GetData(), Stream->size());
				return true;
			}
			else
			{
				return false;
			}
		}

        bool FMetaHumanFaceTracker::LoadPoseBasedSolvers(const TArray<FString>& InPoseBasedSolverFilenames)
        {
			const FScopeLock ScopeLock(&AccessMutex);

            std::vector<std::string> SolverNames(InPoseBasedSolverFilenames.Num());
            int32 i{};
            for (const FString& Solver : InPoseBasedSolverFilenames)
            {
                SolverNames[i++] = TCHAR_TO_ANSI(*Solver);
            }

            return Impl->API.LoadPoseBasedSolvers(SolverNames);
        }

        bool FMetaHumanFaceTracker::GetPoseBasedSolvers(TArray<uint8>& OutMemoryBuffer) 
        {
			const FScopeLock ScopeLock(&AccessMutex);

            std::vector<char> OutBuffer{};
            bool bResult = Impl->API.GetPoseBasedSolvers(OutBuffer);
            if (bResult)
            {
                const int32 Size = int32(OutBuffer.size());
				OutMemoryBuffer.SetNumUninitialized((Size * sizeof(char)) / sizeof(uint8));
                memcpy(OutMemoryBuffer.GetData(), OutBuffer.data(), sizeof(char) * Size);
            }
            return bResult;
        }

        bool FMetaHumanFaceTracker::SetPoseBasedSolvers(const TArray<uint8>& InMemoryBuffer)
        {
			const FScopeLock ScopeLock(&AccessMutex);

			const int32 Size = int32(InMemoryBuffer.Num());
            std::vector<char> MemoryBuffer((Size * sizeof(uint8)) / sizeof(char));
            memcpy(MemoryBuffer.data(), InMemoryBuffer.GetData(), sizeof(uint8) * Size);

            return Impl->API.SetPoseBasedSolvers(MemoryBuffer);
        }

		bool FMetaHumanFaceTracker::TrainSolverModels(FOnTrainSolverModelsProgress InTrainSolverModelsProgressDelegate, FOnTrainSolverModelsIsCancelled InTrainSolverModelsIsCancelledDelegate)
		{
			const FScopeLock ScopeLock(&AccessMutex);

			std::function<void(float)> Progress = [&](float progress)
			{
				Async(EAsyncExecution::TaskGraphMainThread, [InTrainSolverModelsProgressDelegate, progress]() // ensure progress is reported to game thread
					{
						InTrainSolverModelsProgressDelegate.ExecuteIfBound(progress);
					});
			};

			std::function<bool(void)> IsCancelled = [&]()  // 定义匿名函数
			{
				bool Cancel = true;
				InTrainSolverModelsIsCancelledDelegate.ExecuteIfBound(Cancel);
				return Cancel;
			};

			return Impl->API.TrainSolverModels(Progress, IsCancelled);
		}

        bool FMetaHumanFaceTracker::SetCameras(const TArray<FMetaHumanCameraCalibration>& InCalibrations)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            std::map<std::string, titan::api::OpenCVCamera> Cameras;

            for (const FMetaHumanCameraCalibration& Calibration : InCalibrations)
            {
                titan::api::OpenCVCamera Camera;
                Camera.width = Calibration.ImageSizeX;
                Camera.height = Calibration.ImageSizeY;
                Camera.fx = Calibration.FX;
                Camera.fy = Calibration.FY;
                Camera.cx = Calibration.CX;
                Camera.cy = Calibration.CY;
                Camera.k1 = Calibration.K1;
                Camera.k2 = Calibration.K2;
                Camera.k3 = Calibration.K3;
                Camera.p1 = Calibration.P1;
                Camera.p2 = Calibration.P2;

                //! Transform from world coordinates to camera coordinates in column-major format.
                for (int32 Row = 0; Row < 4; Row++)
                {
                    for (int32 Col = 0; Col < 4; Col++)
                    {
                        Camera.Extrinsics[Row * 4 + Col] = Calibration.Transform.M[Row][Col];
                    }
                }

                Cameras[TCHAR_TO_ANSI(*Calibration.Name)] = Camera;
            }

            bool bResult = Impl->API.SetCameras(Cameras);
            if (bResult)
            {
                Impl->State &= ~TrackerState::SET_CAMERAS_FAILED;
            }
            else
            {
                Impl->State |= TrackerState::SET_CAMERAS_FAILED;
            }
            return bResult;
        }

        bool FMetaHumanFaceTracker::SetCameraRanges(const TMap<FString, TPair<float, float>>& InCameraRanges)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            std::map<std::string, std::pair<float, float>> CameraRanges;

            for (const TPair<FString, TPair<float, float>>& CameraRange : InCameraRanges)
            {
                CameraRanges[TCHAR_TO_ANSI(*CameraRange.Key)] = std::pair<float, float>(CameraRange.Value.Key, CameraRange.Value.Value);
            }

            bool bResult = Impl->API.SetCameraRanges(CameraRanges);
            if (bResult)
            {
                Impl->State &= ~TrackerState::SET_CAMERA_RANGES_FAILED;
            }
            else
            {
                Impl->State |= TrackerState::SET_CAMERA_RANGES_FAILED;
            }
            return bResult;
        }

        bool FMetaHumanFaceTracker::SetStereoCameraPairs(const TArray<TPair<FString, FString>>& InStereoReconstructionPairs)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            std::vector<std::pair<std::string, std::string>> CameraPairs;

            for (const TPair<FString, FString>& CameraPair : InStereoReconstructionPairs)
            {
                CameraPairs.push_back(std::pair<std::string, std::string>(TCHAR_TO_ANSI(*CameraPair.Key), TCHAR_TO_ANSI(*CameraPair.Value)));
            }

            bool bResult = Impl->API.SetStereoCameraPairs(CameraPairs);
            if (bResult)
            {
                Impl->State &= ~TrackerState::SET_STEREO_CAMERA_PAIRS_FAILED;
            }
            else
            {
                Impl->State |= TrackerState::SET_STEREO_CAMERA_PAIRS_FAILED;
            }

            return bResult;
        }

        bool FMetaHumanFaceTracker::ResetTrack(int32 InFrameStart, int32 InFrameEnd, const FTrackerOpticalFlowConfiguration& InOptFlowConfig)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            titan::api::OpticalFlowData OpticalFlowConfiguration = { InOptFlowConfig.bUseOpticalFlow, InOptFlowConfig.bUseConfidence, InOptFlowConfig.bUseForwardFlow };
            return Impl->API.ResetTrack(InFrameStart, InFrameEnd, OpticalFlowConfiguration);
        }

        bool FMetaHumanFaceTracker::SetInputData(const TMap<FString, const unsigned char*>& InImageDataPerCamera,
            const TMap<FString, const FFrameTrackingContourData*>& InLandmarksDataPerCamera, const TMap<FString, const float*>& InDepthmapDataPerCamera, int32 InLevel)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            std::map<std::string, const unsigned char*> ImageDataMap;
            std::map<std::string, const std::map<std::string, titan::api::FaceTrackingLandmarkData>> LandmarkMap;
            std::map<std::string, const float*> DepthMapDataMap;

            for (const TPair<FString, const unsigned char*>& ImageForCamera : InImageDataPerCamera)
            {
                ImageDataMap[TCHAR_TO_ANSI(*ImageForCamera.Key)] = ImageForCamera.Value;
            }

            for (const TPair<FString, const float*>& DpthForCamera : InDepthmapDataPerCamera)
            {
                DepthMapDataMap[TCHAR_TO_ANSI(*DpthForCamera.Key)] = DpthForCamera.Value;
            }

            for (const TPair<FString, const FFrameTrackingContourData*>& LandmarksForCamera : InLandmarksDataPerCamera)
            {
                std::map<std::string, titan::api::FaceTrackingLandmarkData> LandmarkMapForCamera;

                for (const TPair<FString, FTrackingContour>& Landmarks : LandmarksForCamera.Value->TrackingContours)
                {
                    std::vector<float> ComponentValues;
                    ComponentValues.reserve(Landmarks.Value.DensePoints.Num() * 2);

                    for (const FVector2D& Point : Landmarks.Value.DensePoints)
                    {
                        ComponentValues.push_back(Point.X);
                        ComponentValues.push_back(Point.Y);
                    }

                    LandmarkMapForCamera[TCHAR_TO_ANSI(*Landmarks.Key)] = titan::api::FaceTrackingLandmarkData::Create(ComponentValues.data(), nullptr, Landmarks.Value.DensePoints.Num(),2);
                }

                LandmarkMap.emplace(std::make_pair(TCHAR_TO_ANSI(*LandmarksForCamera.Key), LandmarkMapForCamera));
            }

            return Impl->API.SetInputData(ImageDataMap, LandmarkMap, DepthMapDataMap, InLevel);
        }

        bool FMetaHumanFaceTracker::Track(int32 InFrameNumber, const TMap<FString, TPair<float*, float*>>& InOpticalFlowImagesForCamera, bool bUseFastSolver)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            std::map < std::string, std::pair<float*, float*>> OpticalFlowImages;

            for (const auto& ImageForCamera : InOpticalFlowImagesForCamera)
            {
                OpticalFlowImages[TCHAR_TO_ANSI(*ImageForCamera.Key)] = { ImageForCamera.Value.Key,ImageForCamera.Value.Value };
            }

            return Impl->API.Track(InFrameNumber, OpticalFlowImages, bUseFastSolver);
        }

        bool FMetaHumanFaceTracker::GetTrackingState(int32 InFrameNumber, FTransform& OutHeadPose, TMap<FString, float>& OutRawControls)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            std::map<std::string, float> RawControls;
            float Pose[16];
            bool bIsOK = Impl->API.GetTrackingState(InFrameNumber, Pose, RawControls);

            for (const std::pair<const std::string, float>& Control : RawControls)
            {
                FString ConvertedControl = FString(Control.first.c_str()).Replace(L".", L"_", ESearchCase::IgnoreCase);

                OutRawControls.Add(ConvertedControl, Control.second);
            }


            // The original rig is in Maya, ie Y up, X right, right-handed
            // By default this gets converted on import into UE, which is Z up, Y right, left-handed, such that it is the right way up and looking along 
            // the positive y axis .
            // So the first thing we need to do is to transform the rig in UE so that it looks the same orientation as the solver code sees it ie 
            // upside down, looking along the negative x axis (in UE). 
            // We do this using an initial offset transform, below, which is applied to the rig before the pose transform
            const FTransform Offset = FTransform(FRotator(0, 90, 180));

            // get the rotation and translation in OpenCV coordinate system
            FMatrix RotationOpenCV = FRotationMatrix::MakeFromXY(FVector(Pose[0], Pose[1], Pose[2]),
                FVector(Pose[4], Pose[5], Pose[6]));
            FVector TranslationOpenCV = FVector(Pose[12], Pose[13], Pose[14]);

            // convert to UE coordinate system
            FRotator RotatorUE;
            FVector TranslationUE;
            ConvertOpenCVToUE(RotationOpenCV, TranslationOpenCV, RotatorUE, TranslationUE);

            // apply the offset transform then the transform from the solver
            OutHeadPose = Offset * FTransform(RotatorUE, TranslationUE);

            return bIsOK;
        }

        int32 FMetaHumanFaceTracker::GetNumStereoPairs()
        {
            const FScopeLock ScopeLock(&AccessMutex);
            return Impl->API.GetNumStereoPairs();
        }

#if 0
        bool FMetaHumanFaceTracker::CreateMeshForStereoReconstructionAndCallback(int32 InStereoPairIndex, FMetaHumanDepthMapData& OutDepthMapData)
        {
            const FScopeLock ScopeLock(&AccessMutex);
            return Impl->API.CreateMeshForStereoReconstructionAndCallback(InStereoPairIndex, [&OutDepthMapData](int32 InNumberOfPoints, int32 InNumberOfTriangles, const float* InVerticesData, const int32* InTriangleIndices)
            {
                FColor WhiteVertex = FColor(255, 255, 255, 255);
                OutDepthMapData.Vertices.SetNum(InNumberOfPoints);
                OutDepthMapData.VertexColors.SetNum(InNumberOfPoints);

                for (int32 Index = 0; Index < InNumberOfPoints; Index++)
                {
                    // map from Opencv to UE 
                    // X in UE comes from Z in OpenCV, Y in UE comes from X in OpenCV, Z in UE comes from -Y in OpenCV
                    OutDepthMapData.Vertices[Index] = FVector(InVerticesData[Index * 3 + 2], InVerticesData[Index * 3], -InVerticesData[Index * 3 + 1]);
                    OutDepthMapData.VertexColors[Index] = WhiteVertex;
                }

                OutDepthMapData.Triangles.SetNum(InNumberOfTriangles * 3);
                for (int32 Index = 0; Index < InNumberOfTriangles * 3; Index++)
                {
                    OutDepthMapData.Triangles[Index] = InTriangleIndices[Index];
                }

                OutDepthMapData.Normals.SetNum(InNumberOfPoints);
                for (int32 Index = 0; Index < InNumberOfTriangles; Index++)
                {
                    const FVector& A = OutDepthMapData.Vertices[OutDepthMapData.Triangles[Index * 3]];
                    const FVector& B = OutDepthMapData.Vertices[OutDepthMapData.Triangles[Index * 3 + 1]];
                    const FVector& C = OutDepthMapData.Vertices[OutDepthMapData.Triangles[Index * 3 + 2]];
                    const FVector Normal = FVector::CrossProduct(A - B, A - C).GetSafeNormal();
                    OutDepthMapData.Normals[OutDepthMapData.Triangles[Index * 3]] = Normal;
                    OutDepthMapData.Normals[OutDepthMapData.Triangles[Index * 3 + 1]] = Normal;
                    OutDepthMapData.Normals[OutDepthMapData.Triangles[Index * 3 + 2]] = Normal;
                }
            });
        }
#endif

        uint32 FMetaHumanFaceTracker::GetTrackerState()
        {
            const FScopeLock ScopeLock(&AccessMutex);
            return Impl->State;
        }
    }
}