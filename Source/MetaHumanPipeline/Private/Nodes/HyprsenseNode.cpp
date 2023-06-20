// Copyright Epic Games, Inc. All Rights Reserved.

#include "Nodes/HyprsenseNode.h"

#define LOCTEXT_NAMESPACE "MetaHuman"

namespace UE::MetaHuman::Pipeline
{
	FHyprsenseNode::FHyprsenseNode(const FString& InName) : FHyprsenseNodeBase("Hyprsense", InName)
	{
		Pins.Add(FPin("UE Image In", EPinDirection::Input, EPinType::UE_Image));
		Pins.Add(FPin("Contours Out", EPinDirection::Output, EPinType::Contours));
	}

	bool FHyprsenseNode::Start(const TSharedPtr<FPipelineData>& InPipelineData)
	{
		if (!bIsInitialized)
		{
			InPipelineData->SetErrorNodeCode(EErrorCode);
			InPipelineData->SetErrorNodeMessage(ErrorMessage);
			return false;
		}

		NNIModels[1] = EyebrowTracker;
		NNIModels[3] = EyeTracker;
		NNIModels[4] = LipsTracker;
		NNIModels[5] = NasolabialTracker;

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

	bool FHyprsenseNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
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
			//Lip
			AddContourToOutput(OutputArrayPerModelInversed[4].Points, CurveLipMap, LandmarkLipMap, Output);
			//Nasolab
			AddContourToOutput(OutputArrayPerModelInversed[5].Points, CurveNasolabMap, LandmarkNasolabMap, Output);
			InPipelineData->SetData<FFrameTrackingContourData>(Pins[1], MoveTemp(Output));
			return true;
		}
	}
	
	bool FHyprsenseNode::SetTrackers(const TWeakObjectPtr<UNeuralNetwork>& InFaceTracker, const TWeakObjectPtr<UNeuralNetwork>& InFaceDetector, const TWeakObjectPtr<UNeuralNetwork>& InEyebrowTracker, const TWeakObjectPtr<UNeuralNetwork>& InEyeTracker, const TWeakObjectPtr<UNeuralNetwork>& InLipsTracker, const TWeakObjectPtr<UNeuralNetwork>& InNasolabialTracker)
	{
		FaceTracker = InFaceTracker;
		FaceDetector = InFaceDetector;
		EyebrowTracker = InEyebrowTracker;
		EyeTracker = InEyeTracker;
		LipsTracker = InLipsTracker;
		NasolabialTracker = InNasolabialTracker;

		const TMap<ETrackerType, NNIModelInfo> ValidationMap = { {ETrackerType::FaceTracker, {{196608},{262,1}}},
																 {ETrackerType::FaceDetector,  {{270000},{6000,12000}}},
																 {ETrackerType::EyebrowTracker, {{ 393216 }, { 192, 0 }}},
																 {ETrackerType::EyeTracker, {{ 1572864 }, { 256, 0 }}},
																 {ETrackerType::LipsTracker, {{ 786432 }, { 432, 0 }}},
																 {ETrackerType::NasoLabialTracker, {{ 196608 }, { 100, 0 }} } };

		const TMap<TWeakObjectPtr<UNeuralNetwork>, ETrackerType> TrackerTypeMap = { {FaceTracker, ETrackerType::FaceTracker},
																					{FaceDetector, ETrackerType::FaceDetector},
																					{EyebrowTracker, ETrackerType::EyebrowTracker},
																					{EyeTracker, ETrackerType::EyeTracker},
																					{LipsTracker, ETrackerType::LipsTracker},
																					{NasolabialTracker, ETrackerType::NasoLabialTracker} };

		return CheckTrackers(ValidationMap, TrackerTypeMap);
	}
}
#undef LOCTEXT_NAMESPACE
