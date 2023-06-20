// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentityPromotedFrames.h"
#include "ShapeAnnotationWrapper.h"
#include "MetaHumanFaceContourTrackerAsset.h"

#define LOCTEXT_NAMESPACE "MetaHumanIdentityPromotedFrame"

/////////////////////////////////////////////////////
// UMetaHumanIdentityPromotedFrame

static const TCHAR* MarkerModificationTransaction = TEXT("MarkerModificationTransaction");
const FIntPoint UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize{ 2048, 2048 };

UMetaHumanIdentityPromotedFrame::UMetaHumanIdentityPromotedFrame()
	: Super{}
	, bUseToSolve{ true }
	, bIsNavigationLocked{ false }
	, bTrackOnChange{ false }
{
	ShapeAnnotation = MakeShared<FShapeAnnotationWrapper>();

	ShapeAnnotation->OnCurveModified().BindLambda([this]
	{
		const FScopedTransaction Transaction(MarkerModificationTransaction, LOCTEXT("ShapeAnnotationTransaction", "Modify Shape Annotation"), this);
		Modify();
		UpdateContourDataFromShapeAnnotation();
	});
}

void UMetaHumanIdentityPromotedFrame::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);

	// Used to initialize the ShapeAnnotationWrapper after loading the contour data previously saved
	if (Ar.IsLoading())
	{
		InitializeShapeAnnotation();
	}
}

void UMetaHumanIdentityPromotedFrame::PostEditChangeProperty(FPropertyChangedEvent& InPropertyChangedEvent)
{
	Super::PostEditChangeProperty(InPropertyChangedEvent);
}

void UMetaHumanIdentityPromotedFrame::InitializeMarkersFromParsedConfig(const FFrameTrackingContourData& InContourData)
{
	check(ShapeAnnotation);
	ContourData = InContourData;

	InitializeShapeAnnotation();
}

void UMetaHumanIdentityPromotedFrame::AddShapeAnnotationInitializationForContours(const FFrameTrackingContourData& InContourData)
{
	for (const auto& Contour : InContourData.TrackingContours)
	{
		const FString& Feature = Contour.Key;
		if (ContourData.TrackingContours.Contains(Feature))
		{
			ContourData.TrackingContours[Feature].DensePoints = Contour.Value.DensePoints;
			TArray<FVector2D> ControlVerts = ShapeAnnotation->GetControlVerticesForCurve(Contour.Value.DensePoints);
			ContourData.TrackingContours[Feature].ControlVertices = ControlVerts;
			ShapeAnnotation->ModifyKeyPointData(Feature, Contour.Value.DensePoints, ControlVerts);
		}
	}

	InitializeShapeAnnotation();
}

void UMetaHumanIdentityPromotedFrame::ModifyContourData(const FFrameTrackingContourData& InContourData)
{
	check(ShapeAnnotation);

	for (const auto& Contour : InContourData.TrackingContours)
	{
		const FString& Feature = Contour.Key;
		if (ContourData.TrackingContours.Contains(Feature) && !Contour.Value.DensePoints.IsEmpty())
		{
			ContourData.TrackingContours[Feature].DensePoints = Contour.Value.DensePoints;
			TArray<FVector2D> ControlVerts = ShapeAnnotation->GetControlVerticesForCurve(Contour.Value.DensePoints);
			ContourData.TrackingContours[Feature].ControlVertices = ControlVerts;
			ShapeAnnotation->ModifyKeyPointData(Feature, Contour.Value.DensePoints, ContourData.TrackingContours[Feature].ControlVertices);
		}
	}
}

void UMetaHumanIdentityPromotedFrame::UpdateContourDataFromShapeAnnotation()
{
	check(ShapeAnnotation);

	TMap<FString, TArray<FVector2D>> VisibleSplines = ShapeAnnotation->GetSplinePointsForDraw();
	TMap<FString, TArray<FVector2D>> ControlVerts = ShapeAnnotation->GetControlVertices();
	
	for(TPair<FString, TArray<FVector2D>>& SplineData : VisibleSplines)
	{
		const FString& CurveName = SplineData.Key;

		FVector2D StartPoint = *SplineData.Value.begin();
		FVector2D EndPoint = SplineData.Value.Last();
		SplineData.Value.Pop();
		SplineData.Value.RemoveAt(0);

		FString StartCurveName = ContourData.TrackingContours[CurveName].StartPointName;
		FString EndCurveName = ContourData.TrackingContours[CurveName].EndPointName;
		
		ContourData.TrackingContours[CurveName].DensePoints = SplineData.Value;
		ContourData.TrackingContours[CurveName].ControlVertices = ControlVerts[CurveName];

		if(!StartCurveName.IsEmpty() && !EndCurveName.IsEmpty())
		{
			ContourData.TrackingContours[StartCurveName].DensePoints = {StartPoint};
			ContourData.TrackingContours[EndCurveName].DensePoints = {EndPoint};
		}
	}
}

bool UMetaHumanIdentityPromotedFrame::FrameContoursContainActiveData() const
{
	return ContourData.ContainsActiveData();
}

TSharedPtr<FShapeAnnotationWrapper> UMetaHumanIdentityPromotedFrame::GetShapeAnnotationWrapper() const
{
	return ShapeAnnotation;
}

bool UMetaHumanIdentityPromotedFrame::CanTrack() const
{
	return ContourTracker != nullptr && ContourTracker->CanProcess();
}

bool UMetaHumanIdentityPromotedFrame::IsTrackingOnChange() const
{
	return bTrackOnChange;
}

bool UMetaHumanIdentityPromotedFrame::IsTrackingManually() const
{
	return !bTrackOnChange;
}

bool UMetaHumanIdentityPromotedFrame::IsNavigationLocked() const
{
	return bIsNavigationLocked;
}

void UMetaHumanIdentityPromotedFrame::SetNavigationLocked(bool bIsLocked)
{
	bIsNavigationLocked = bIsLocked;

	if (IsNavigationLocked())
	{
		bTrackOnChange = false;
	}
}

void UMetaHumanIdentityPromotedFrame::ToggleNavigationLocked()
{
	SetNavigationLocked(!bIsNavigationLocked);
}

void UMetaHumanIdentityPromotedFrame::InitializeShapeAnnotation()
{
	check(ShapeAnnotation);
	ShapeAnnotation->VisibilityMap.Empty();
	for(auto& Contour : ContourData.TrackingContours)
	{
		ShapeAnnotation->VisibilityMap.Add(Contour.Key, &Contour.Value.State);
	}
	
	ShapeAnnotation->InitializeFromLandmarkData(ContourData);
}

void UMetaHumanIdentityPromotedFrame::UpdateShapeAnnotationMarkers() const
{
	ShapeAnnotation->InitializeFromLandmarkData(ContourData);
}

/////////////////////////////////////////////////////
// UMetaHumanIdentityCameraFrame

void UMetaHumanIdentityCameraFrame::PostEditChangeProperty(FPropertyChangedEvent& InPropertyChangedEvent)
{
	Super::PostEditChangeProperty(InPropertyChangedEvent);

	if (FProperty* Property = InPropertyChangedEvent.MemberProperty)
	{
		const FName PropertyName = *Property->GetName();

		if (PropertyName == GET_MEMBER_NAME_CHECKED(ThisClass, ViewLocation) ||
			PropertyName == GET_MEMBER_NAME_CHECKED(ThisClass, ViewRotation) ||
			PropertyName == GET_MEMBER_NAME_CHECKED(ThisClass, LookAtLocation) ||
			PropertyName == GET_MEMBER_NAME_CHECKED(ThisClass, CameraViewFOV))
		{
			OnCameraTransformChangedDelegate.ExecuteIfBound();
		}
	}
}

FTransform UMetaHumanIdentityCameraFrame::GetCameraTransform() const
{
	return FTransform{ ViewRotation, ViewLocation };
}

#undef LOCTEXT_NAMESPACE
