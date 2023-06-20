// Copyright Epic Games, Inc. All Rights Reserved.

#include "Nodes/HyprsenseTestNode.h"

#include "NeuralNetwork.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/KismetMathLibrary.h"

#define LOCTEXT_NAMESPACE "MetaHuman"

namespace UE::MetaHuman::Pipeline
{
#if WITH_DEV_AUTOMATION_TESTS
	FHyprsenseTestNode::FHyprsenseTestNode(const FString& InName) : FNode("Hyprsense", InName)
	{
		Pins.Add(FPin("UE Image In", EPinDirection::Input, EPinType::UE_Image));
		Pins.Add(FPin("Contours In", EPinDirection::Input, EPinType::Contours));
		Pins.Add(FPin("Avg Diff Out", EPinDirection::Output, EPinType::Float));

	}

	bool FHyprsenseTestNode::Start(const TSharedPtr<FPipelineData>& InPipelineData)
	{
		FString JsonFile = InJsonFilePath;
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
					const TArray<TSharedPtr<FJsonValue>>* Indices = nullptr;
					auto Frames = JsonParsed->Values["frames"];

					if (Frames->TryGetArray(Indices))
					{
						for (const TSharedPtr<FJsonValue>& Index : *Indices)
						{
							const TSharedPtr<FJsonObject>* Frame = nullptr;
							if (Index->TryGetObject(Frame))
							{
								auto Object = Frame->Get();

								TArray<FVector2D> Points;
								const TArray<TSharedPtr<FJsonValue>>* ValueArray = nullptr;
								TMap<FString, FTrackingContour> TrackingContours;

								for (const auto& Landmarks : Object->Values)
								{
									if (Landmarks.Key == "metadata")
									{
										//Do nothing
									}
									else if (Landmarks.Key == "points")
									{
										if (Landmarks.Value->TryGetArray(ValueArray))
										{
											for (const TSharedPtr<FJsonValue>& Value : *ValueArray)
											{
												const TArray<TSharedPtr<FJsonValue>>* XY = nullptr;

												if (Value->TryGetArray(XY))
												{
													float X = (*XY)[0]->AsNumber();
													float Y = (*XY)[1]->AsNumber();
													FVector2D Point = { X,Y };
													Points.Add(Point);
												}
											}
										}
									}
									else if (Landmarks.Key == "landmarks")
									{
										const TSharedPtr<FJsonObject>* LObject = nullptr;

										if (Landmarks.Value->TryGetObject(LObject))
										{
											for (const auto& Item : LObject->Get()->Values)
											{
												TSharedPtr<FJsonValue> Value = Item.Value;
												int32 Idx = int32(Value->AsNumber());
												FTrackingContour Contour;
												Contour.DensePoints.Add(Points[Idx]);
												TrackingContours.Add(Item.Key, Contour);
											}
										}
									}
									else if (Landmarks.Key == "curves")
									{
										const TSharedPtr<FJsonObject>* LObject = nullptr;

										if (Landmarks.Value->TryGetObject(LObject))
										{
											for (const auto& Item : LObject->Get()->Values)
											{
												if (Item.Value->TryGetArray(ValueArray))
												{
													FTrackingContour Contour;
													for (const TSharedPtr<FJsonValue>& Value : *ValueArray)
													{
														int32 Idx = Value->AsNumber();
														Contour.DensePoints.Add(Points[Idx]);
													}
													TrackingContours.Add(Item.Key, Contour);
												}
											}
										}
									}
								}

								FFrameTrackingContourData ContourData;
								ContourData.TrackingContours = TrackingContours;
								ContourByFrame.Add(ContourData);
							}
						}
					}
				}
			}
		}

		return true;
	}
	bool FHyprsenseTestNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
	{
		const FUEImageDataType& Image = InPipelineData->GetData<FUEImageDataType>(Pins[0]);
		const FFrameTrackingContourData& Contours = InPipelineData->GetData<FFrameTrackingContourData>(Pins[1]);

		FUEImageDataType Output = Image;
		auto InputContours = ContourByFrame[FrameCount];

		TMap<FString, float> ContourDiffAverage;
		float TotalAverage = 0.0f;
		int32 TotalContourCount = 0;

		for (const auto& Contour : Contours.TrackingContours)
		{
			float Average = 0.0f;
			auto Value = Contour.Value;
			auto InContour = InputContours.TrackingContours[Contour.Key];

			if (Value.DensePoints.Num() == InContour.DensePoints.Num())
			{
				int32 NumArray = Value.DensePoints.Num();
				for (int32 I = 0; I < NumArray; I++)
				{
					FVector2D Point1 = Value.DensePoints[I];
					FVector2D Point2 = InContour.DensePoints[I];
					float Distance = UKismetMathLibrary::Distance2D(Point1, Point2);
					Average += Distance;
				}
				TotalContourCount += NumArray;
				TotalAverage += Average;
				Average /= NumArray;
				ContourDiffAverage.Add(Contour.Key, Average);
			}

		}
		TotalAverage /= TotalContourCount;
		TotalAverageInAllFrames += TotalAverage;
		TotalLandmarkDiffAverageByFrame.Add(TotalAverage);
		ContourDiffAverageByFrame.Add(ContourDiffAverage);
		
		InPipelineData->SetData<float>(Pins[2], MoveTemp(TotalAverage));

		FrameCount++;
		return true;
	}
	bool FHyprsenseTestNode::End(const TSharedPtr<FPipelineData>& InPipelineData)
	{
		FString JsonString;
		TSharedRef<TJsonWriter<TCHAR>> JsonObject = TJsonWriterFactory<>::Create(&JsonString);
		JsonObject->WriteObjectStart();
		
		int32 FrameNum = ContourDiffAverageByFrame.Num();
		TotalAverageInAllFrames /= FrameNum;

		for (int32 I = 0; I < FrameNum; I++)
		{
			JsonObject->WriteObjectStart("Frame " + FString::FromInt(I));
			{
				auto ContourDiffAverages = ContourDiffAverageByFrame[I];
				auto LandmarkDiffAverage = TotalLandmarkDiffAverageByFrame[I];
				JsonObject->WriteObjectStart("Contours");
				for (auto ContourDiff : ContourDiffAverages)
				{
					JsonObject->WriteValue(ContourDiff.Key, ContourDiff.Value);
				}
				JsonObject->WriteObjectEnd();
				JsonObject->WriteValue("Average", LandmarkDiffAverage);
			}
			JsonObject->WriteObjectEnd();
		}

		JsonObject->WriteValue("Total Average", TotalAverageInAllFrames);

		JsonObject->WriteObjectEnd();
		JsonObject->Close();

		FFileHelper::SaveStringToFile(*JsonString, *OutJsonFilePath);
		return true;
	}
#endif
}
#undef LOCTEXT_NAMESPACE
