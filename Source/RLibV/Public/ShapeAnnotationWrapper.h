// Copyright Epic Games, Inc. All Rights Reserved.

// The purpose of this file it to define an interface to rlibv functionality that can
// be called by UE. Dont use dlib etc types here since that complicated the compile.

#pragma once

#include "CoreMinimal.h"
#include "FrameTrackingContourData.h"

class RLIBV_API FShapeAnnotationWrapper
{
public:
	FShapeAnnotationWrapper();
	~FShapeAnnotationWrapper();
	
	void UpdateKeyPosition(const int32 InKey, const FVector2D& InPosition) const;
	void OffsetSelectedPoints(const TSet<int32>& InSelectedPointIDs, const FVector2D& InOffset) const;
	void StoreMovedPointPosition() const;
	void AddRemoveKey(const FVector2D& InMousePosImageSpace) const;
	void ProcessUndo() const;
	void ProcessRedo() const;
	
	void ModifyKeyPointData(const FString& InFeature, const TArray<FVector2D>& InLandmarkData, const TArray<FVector2D>& InInternalPoints) const;
	void InitializeFromLandmarkData(const FFrameTrackingContourData& InLandmarkData) const;
	void OffsetCurve(const FString& InCurveName, const FVector2D InOffset) const;

	TArray<FVector2D> GetControlVerticesForCurve(const TArray<FVector2D>& InLandmarkData) const;
	TMap<FString, TArray<FVector2D>> GetSplinePointsForDraw() const;
	TMap<FString, TArray<FVector2D>> GetControlVertices() const;
	TMap<int32, FTrackingPoint> GetPointsOnSplineForDraw();

	TMap<FString, FMarkerCurveState*> VisibilityMap;

	FSimpleDelegate& OnCurveModified()
	{
		return OnCurveModifedDelegate;
	}

private:

	bool CurveIsVisible(const FString& InCurveName) const;

	FSimpleDelegate OnCurveModifedDelegate;
	
	class FImpl;
	TPimplPtr<FImpl> Impl;
};
