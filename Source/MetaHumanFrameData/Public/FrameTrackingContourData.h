// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Algo/AnyOf.h"

#include "FrameTrackingContourData.generated.h"

USTRUCT()
struct METAHUMANFRAMEDATA_API FMarkerCurveState
{
	GENERATED_BODY()

	UPROPERTY()
	bool bVisible = false;

	UPROPERTY()
	bool bActive = false;

	UPROPERTY(Transient)
	bool bSelected = false;
};

USTRUCT()
struct METAHUMANFRAMEDATA_API FTrackingContour
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "InternalPoints")
	TArray<FVector2D> DensePoints;

	UPROPERTY()
	TArray<FVector2D> ControlVertices;

	UPROPERTY()
	TArray<uint32> KeyPointIndices;

	UPROPERTY()
	FString StartPointName;

	UPROPERTY()
	FString EndPointName;

	UPROPERTY(EditAnywhere, Category = "Markers")
	FMarkerCurveState State;
};

USTRUCT()
struct METAHUMANFRAMEDATA_API FFrameTrackingContourData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, EditFixedSize, Category = "Tracking", DisplayName = "Markers")
	TMap<FString, FTrackingContour> TrackingContours;

	inline bool ContainsData() const
	{
		if (TrackingContours.Num() > 0)
		{
			for (const auto& Contour : TrackingContours)
			{
				if (Contour.Value.DensePoints.Num() > 0)
				{
					return true;
				}
			}
		}

		return false;
	}

	inline bool ContainsActiveData() const
	{
		return Algo::AnyOf(TrackingContours, [](const TPair<FString, FTrackingContour>& Contour)
		{
			return Contour.Value.State.bActive;
		});
	}
};

USTRUCT()
struct METAHUMANFRAMEDATA_API FTrackingContour3D
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "Tracking")
	TArray<FVector3d> DensePoints;

	UPROPERTY(VisibleAnywhere, Category = "Tracking")
	TArray<uint32> KeyPointIndices;

};

struct FTrackingPoint
{
	TArray<FVector2D> LinePoints;
	FVector2D PointPosition;
	TArray<FString> CurveNames;
};
