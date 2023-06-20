// Copyright Epic Games, Inc. All Rights Reserved.

#include "Nodes/HyprsenseRealtimeNode.h"
#include "NeuralNetwork.h"

#define LOCTEXT_NAMESPACE "MetaHuman"

namespace UE::MetaHuman::Pipeline
{
	FHyprsenseRealtimeNode::FHyprsenseRealtimeNode(const FString& InName) : FHyprsenseNodeBase("HyprsenseRealtime", InName)
	{
		Pins.Add(FPin("UE Image In", EPinDirection::Input, EPinType::UE_Image));
		Pins.Add(FPin("Contours Out", EPinDirection::Output, EPinType::Contours));
	}

	bool FHyprsenseRealtimeNode::Start(const TSharedPtr<FPipelineData>& InPipelineData)
	{
		if (!bIsInitialized)
		{
			InPipelineData->SetErrorNodeCode(EErrorCode);
			InPipelineData->SetErrorNodeMessage(ErrorMessage);
			return false;
		}

		NNIModels[1] = EyebrowTracker;
		NNIModels[3] = EyeTracker;
		NNIModels[6] = LipsNasolabialTracker;

		ProcessPart = ProcessPartHere;
		TrackerPartInputSizeX = TrackerPartHereInputSizeX;
		TrackerPartInputSizeY = TrackerPartHereInputSizeY;

		InitTransformLandmark131to159();

		bIsFaceDetected = false;
		ErrorMessage = "";
		LastTransform << 0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f;

		return true;
	}
	
	bool FHyprsenseRealtimeNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseRealtimeNode::Process);

		if (!bIsInitialized)
		{
			return false;
		}

		const FUEImageDataType& Input = InPipelineData->GetData<FUEImageDataType>(Pins[0]);
		TArray<PartPoints> OutputArrayPerModelInversed = ProcessLandmarks(Input);

		if (OutputArrayPerModelInversed.IsEmpty())
		{
			return false;
		}
		else
		{
			//OutputArrayInversed contains sparse landmarsk. We will not use them
			FFrameTrackingContourData Output;

			//Brow
			AddContourToOutput(OutputArrayPerModelInversed[1].Points, CurveBrowMap, LandmarkBrowMap, Output);
			//Eye-Iris
			AddContourToOutput(OutputArrayPerModelInversed[3].Points, CurveEyeIrisMap, LandmarkEyeIrisMap, Output);

			TArray<float> LipsOutArray, NasoOutArray;
			if (!OutputArrayPerModelInversed[6].Points.IsEmpty())
			{
				LipsOutArray.SetNum(432); //The number of lips landmarks (x,y) 216 * 2 = 432
				NasoOutArray.SetNum(100); //The number of nasolabial landmarks (x,y) 50 * 2 = 100

				//Since lips & nasolabial are combined in realtime tracker, you need to separate output result
				FMemory::Memcpy(LipsOutArray.GetData(), OutputArrayPerModelInversed[6].Points.GetData(), 432 * sizeof(float));
				FMemory::Memcpy(NasoOutArray.GetData(), OutputArrayPerModelInversed[6].Points.GetData() + 432, 100 * sizeof(float));
			}

			//Lip
			AddContourToOutput(LipsOutArray, CurveLipMap, LandmarkLipMap, Output);
			//Nasolab
			AddContourToOutput(NasoOutArray, CurveNasolabMap, LandmarkNasolabMap, Output);

			InPipelineData->SetData<FFrameTrackingContourData>(Pins[1], MoveTemp(Output));
			return true;
		}
	}

	bool FHyprsenseRealtimeNode::SetTrackers(const TWeakObjectPtr<UNeuralNetwork>& InFaceTracker, const TWeakObjectPtr<UNeuralNetwork>& InFaceDetector, const TWeakObjectPtr<UNeuralNetwork>& InEyebrowTracker, const TWeakObjectPtr<UNeuralNetwork>& InEyeTracker, const TWeakObjectPtr<UNeuralNetwork>& InLipsNasolabialTracker)
	{
		FaceTracker = InFaceTracker;
		FaceDetector = InFaceDetector;
		EyebrowTracker = InEyebrowTracker;
		EyeTracker = InEyeTracker;
		LipsNasolabialTracker = InLipsNasolabialTracker;
		
		const TMap<ETrackerType, NNIModelInfo> ValidationMap = { {ETrackerType::FaceTracker, {{196608},{262,1}}},
																 {ETrackerType::FaceDetector,  {{270000},{6000,12000}}},
																 {ETrackerType::EyebrowTracker, {{ 24576 }, { 192, 0 }}},
																 {ETrackerType::EyeTracker, {{ 98304 }, { 256, 0 }}},
																 {ETrackerType::LipsNasolabialTracker, {{ 196608 }, { 532, 0 }}} };

		const TMap<TWeakObjectPtr<UNeuralNetwork>, ETrackerType> TrackerTypeMap = { {FaceTracker, ETrackerType::FaceTracker},
																					{FaceDetector, ETrackerType::FaceDetector},
																					{EyebrowTracker, ETrackerType::EyebrowTracker},
																					{EyeTracker, ETrackerType::EyeTracker},
																					{LipsNasolabialTracker, ETrackerType::LipsNasolabialTracker} };
	

		return CheckTrackers(ValidationMap, TrackerTypeMap);
	}



	FHyprsenseRealtimeManagedNode::FHyprsenseRealtimeManagedNode(const FString& InName) : FHyprsenseRealtimeNode(InName)
	{
		TArray<FSoftObjectPath> Paths = GetNNIPaths();

		UNeuralNetwork* FaceTrackerObject = LoadObject<UNeuralNetwork>(GetTransientPackage(), *Paths[0].ToString());
		UNeuralNetwork* FaceDetectorObject = LoadObject<UNeuralNetwork>(GetTransientPackage(), *Paths[1].ToString());
		UNeuralNetwork* EyebrowTrackerObject = LoadObject<UNeuralNetwork>(GetTransientPackage(), *Paths[2].ToString());
		UNeuralNetwork* EyeTrackerObject = LoadObject<UNeuralNetwork>(GetTransientPackage(), *Paths[3].ToString());
		UNeuralNetwork* LipsNasolabialTrackerObject = LoadObject<UNeuralNetwork>(GetTransientPackage(), *Paths[4].ToString());

		verify(SetTrackers(FaceTrackerObject, FaceDetectorObject, EyebrowTrackerObject, EyeTrackerObject, LipsNasolabialTrackerObject));
	}

	TArray<FSoftObjectPath> FHyprsenseRealtimeManagedNode::GetNNIPaths()
	{
		TArray<FSoftObjectPath> Paths; // Dont change order of paths without editing constructor above!

		Paths.Add(FSoftObjectPath("/MetaHuman/GenericTracker/FaceTracker.FaceTracker"));
		Paths.Add(FSoftObjectPath("/MetaHuman/GenericTracker/FaceDetector.FaceDetector"));
		Paths.Add(FSoftObjectPath("/MetaHuman/GenericTracker/LeftBrowRealtime.LeftBrowRealtime"));
		Paths.Add(FSoftObjectPath("/MetaHuman/GenericTracker/LeftEyeRealtime.LeftEyeRealtime"));
		Paths.Add(FSoftObjectPath("/MetaHuman/GenericTracker/LipsNasolabialRealtime.LipsNasolabialRealtime"));

		return Paths;
	}
}

#undef LOCTEXT_NAMESPACE
