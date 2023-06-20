// Copyright Epic Games, Inc. All Rights Reserved.

#include "Nodes/RLibVNodes.h"
#include "Pipeline/PipelineData.h"

#include "RLibV-UE.h"

namespace UE::MetaHuman::Pipeline
{

FRLibVRefinementTrackerNode::FRLibVRefinementTrackerNode(const FString& InName) : FNode("RLibVRefinementTracker", InName)
{
	Pins.Add(FPin("UE Image In", EPinDirection::Input, EPinType::UE_Image));
	Pins.Add(FPin("Contours In", EPinDirection::Input, EPinType::Contours));
	Pins.Add(FPin("Contours Out", EPinDirection::Output, EPinType::Contours));
	Pins.Add(FPin("Confidence Out", EPinDirection::Output, EPinType::TrackingConfidence));
}

bool FRLibVRefinementTrackerNode::Start(const TSharedPtr<FPipelineData>& InPipelineData)
{
	Tracker = MakeShared<FRLibVRefinementTracker>();

	if (!Tracker->Initialize(ModelFile))
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
		InPipelineData->SetErrorNodeMessage(Tracker->GetErrorMessage());

		return false;
	}

	return true;
}

bool FRLibVRefinementTrackerNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	const FUEImageDataType& Image = InPipelineData->GetData<FUEImageDataType>(Pins[0]);
	const FFrameTrackingContourData& InitialContours = InPipelineData->GetData<FFrameTrackingContourData>(Pins[1]);

	FFrameTrackingContourData RefinedContours;
	FFrameTrackingConfidenceData Confidence;
	if (!Tracker->Track(Image.Width, Image.Height, Image.Data, InitialContours, RefinedContours, Confidence))
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToTrack);
		InPipelineData->SetErrorNodeMessage(FString::Printf(TEXT("Failed to track frame %i: %s"), InPipelineData->GetFrameNumber(), *Tracker->GetErrorMessage()));

		return false;
	}

	InPipelineData->SetData<FFrameTrackingContourData>(Pins[2], MoveTemp(RefinedContours));

	InPipelineData->SetData<FFrameTrackingConfidenceData>(Pins[3], MoveTemp(Confidence));

	return true;
}

bool FRLibVRefinementTrackerNode::End(const TSharedPtr<FPipelineData>& InPipelineData)
{
	Tracker = nullptr;

	return true;
}

}
