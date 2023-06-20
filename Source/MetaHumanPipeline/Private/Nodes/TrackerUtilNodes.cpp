// Copyright Epic Games, Inc. All Rights Reserved.

#include "Nodes/TrackerUtilNodes.h"
#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Pipeline/PipelineData.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"

namespace UE::MetaHuman::Pipeline
{

FJsonTrackerNode::FJsonTrackerNode(const FString& InName) : FNode("JsonTracker", InName)
{
	Pins.Add(FPin("UE Image In", EPinDirection::Input, EPinType::UE_Image)); // Does not really take an image, but this makes it a drop in replacement for other tracker nodes
	Pins.Add(FPin("Contours Out", EPinDirection::Output, EPinType::Contours));
}

bool FJsonTrackerNode::Start(const TSharedPtr<FPipelineData>& InPipelineData)
{
	bool bIsOK = false;

	Contours.Reset();

	// Robustness of parsing could be better! ie handling unexpected data in fields

	TArray<FString> Lines;
	if (FFileHelper::LoadANSITextFileToStrings(*JsonFile, nullptr, Lines))
	{
		if (Lines.Num() == 1)
		{
			FString JsonRaw;
			TSharedRef<TJsonReader<TCHAR>> JsonReader = TJsonReaderFactory<TCHAR>::Create(Lines[0]);

			TSharedPtr<FJsonObject> JsonParsed;
			if (FJsonSerializer::Deserialize(JsonReader, JsonParsed))
			{
				const TArray<TSharedPtr<FJsonValue>>* Frames = nullptr;
				if (JsonParsed->TryGetArrayField("frames", Frames))
				{
					bIsOK = true;

					for (TSharedPtr<FJsonValue> Frame : *Frames)
					{
						const TArray<TSharedPtr<FJsonValue>>* Points = nullptr;
						const TSharedPtr<FJsonObject>* Curves = nullptr;
						const TSharedPtr<FJsonObject>* Landmarks = nullptr;

						bIsOK &= (bIsOK && Frame->AsObject()->TryGetArrayField("points", Points));
						bIsOK &= (bIsOK && Frame->AsObject()->TryGetObjectField("curves", Curves));
						bIsOK &= (bIsOK && Frame->AsObject()->TryGetObjectField("landmarks", Landmarks));

						if (bIsOK)
						{
							FFrameTrackingContourData Contour;

							TArray<FVector2D> PointList;
							for (const TSharedPtr<FJsonValue>& Point : *Points)
							{
								const TArray<TSharedPtr<FJsonValue>>* XY = nullptr;
								if (Point->TryGetArray(XY) && XY->Num() == 2)
								{
									float X = (*XY)[0]->AsNumber();
									float Y = (*XY)[1]->AsNumber();
									PointList.Add(FVector2D(X, Y));
								}
							}

							bIsOK &= (PointList.Num() == Points->Num());
							if (!bIsOK)
							{
								break;
							}

							for (const auto& Curve : (*Curves)->Values)
							{
								Contour.TrackingContours.Add(Curve.Key);

								const TArray<TSharedPtr<FJsonValue>>* Indices = nullptr;
								if (Curve.Value->TryGetArray(Indices))
								{
									for (const TSharedPtr<FJsonValue>& Index : *Indices)
									{
										int32 IndexInt = Index->AsNumber();
										Contour.TrackingContours[Curve.Key].DensePoints.Add(PointList[IndexInt]);
									}
								}
							}

							for (const auto& Landmark : (*Landmarks)->Values)
							{
								Contour.TrackingContours.Add(Landmark.Key);

								const TArray<TSharedPtr<FJsonValue>>* Indices = nullptr;
								if (Landmark.Value->TryGetArray(Indices))
								{
									for (const TSharedPtr<FJsonValue>& Index : *Indices)
									{
										int32 IndexInt = Index->AsNumber();
										Contour.TrackingContours[Landmark.Key].DensePoints.Add(PointList[IndexInt]);
									}
								}
							}

							Contours.Add(MoveTemp(Contour));
						}
						else
						{
							break;
						}
					}
				}
			}
		}
	}
	else
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToLoadJsonFile);
		InPipelineData->SetErrorNodeMessage(FString::Printf(TEXT("Failed to load JSON file %s"), *JsonFile));
		return false;
	}

	if (!bIsOK)
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::InvalidData);
		InPipelineData->SetErrorNodeMessage(FString::Printf(TEXT("Invalid data in JSON file %s"), *JsonFile));
		return false;
	}

	return bIsOK;
}

bool FJsonTrackerNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 Frame = InPipelineData->GetFrameNumber();

	if (Frame < Contours.Num())
	{
		InPipelineData->SetData<FFrameTrackingContourData>(Pins[1], Contours[Frame]);
		return true;
	}
	else
	{
		return false;
	}
}

bool FJsonTrackerNode::End(const TSharedPtr<FPipelineData>& InPipelineData)
{
	Contours.Reset();

	return true;
}



FOffsetContoursNode::FOffsetContoursNode(const FString& InName) : FNode("OffsetContours", InName)
{
	Pins.Add(FPin("Contours In", EPinDirection::Input, EPinType::Contours));
	Pins.Add(FPin("Contours Out", EPinDirection::Output, EPinType::Contours));
}

bool FOffsetContoursNode::Start(const TSharedPtr<FPipelineData>& InPipelineData)
{
	RandomOffsetMinX = -RandomOffset.X / 2.0f;
	RandomOffsetMaxX = RandomOffset.X / 2.0f;
	RandomOffsetMinY = -RandomOffset.Y / 2.0f;
	RandomOffsetMaxY = RandomOffset.Y / 2.0f;

	return true;
}

bool FOffsetContoursNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	const FFrameTrackingContourData& Input = InPipelineData->GetData<FFrameTrackingContourData>(Pins[0]);

	FFrameTrackingContourData Output = Input;

	for (auto& Contour : Output.TrackingContours)
	{
		for (FVector2D& Point : Contour.Value.DensePoints)
		{
			Point += ConstantOffset;
			Point.X += FMath::RandRange(RandomOffsetMinX, RandomOffsetMaxX);
			Point.Y += FMath::RandRange(RandomOffsetMinY, RandomOffsetMaxY);
		}
	}

	InPipelineData->SetData<FFrameTrackingContourData>(Pins[1], MoveTemp(Output));

	return true;
}

}
