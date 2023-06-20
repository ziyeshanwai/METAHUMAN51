// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "FrameTrackingContourData.h"

#include "Viewports.h"
#include "EngineDefines.h"

#include "UI/MetaHumanIdentityMarkerData.h"

#include "MetaHumanIdentityPromotedFrames.generated.h"

/////////////////////////////////////////////////////
// UMetaHumanIdentityPromotedFrame

UCLASS(Abstract, MinimalAPI)
class UMetaHumanIdentityPromotedFrame
	: public UObject
{
	GENERATED_BODY()

public:

	/** The default size to be used when capturing a promoted frame for tracking */
	static const FIntPoint DefaultTrackerImageSize;

	UMetaHumanIdentityPromotedFrame();

public:
	//~ UObject interface
	virtual void Serialize(FArchive& Ar) override;
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& InPropertyChangedEvent) override;

public:
	/** Set the contour data that was likely produced by the tracker associated with this Promoted Frame */
	void InitializeMarkersFromParsedConfig(const FFrameTrackingContourData& InContourData);
	void AddShapeAnnotationInitializationForContours(const FFrameTrackingContourData& InContourData);
	void ModifyContourData(const FFrameTrackingContourData& InContourData);
	void UpdateShapeAnnotationMarkers() const;

	/** Update the contour data using manually positioned points */
	void UpdateContourDataFromShapeAnnotation();

	/** Returns true if CountourData in ShapeAnnotation contains any curves that are active */
	bool FrameContoursContainActiveData() const;

	/** Return the ShapeAnnotation wrapper that represents the landmarks for this Promoted Frame */
	TSharedPtr<class FShapeAnnotationWrapper> GetShapeAnnotationWrapper() const;

	/** Returns true if this Promoted Frame has all the required information to track */
	UFUNCTION(BlueprintPure, Category = "Tracking")
	bool CanTrack() const;

	UFUNCTION(BlueprintCallable, Category = "Tracking")
	bool IsTrackingOnChange() const;

	UFUNCTION(BlueprintCallable, Category = "Tracking")
	bool IsTrackingManually() const;

	UFUNCTION(BlueprintPure, Category = "Navigation")
	bool IsNavigationLocked() const;

	UFUNCTION(BlueprintCallable, Category = "Navigation")
	void SetNavigationLocked(bool bIsLocked);

	UFUNCTION(BlueprintCallable, Category = "Navigation")
	void ToggleNavigationLocked();

public:

	/** The alignment of the conformal mesh associated with this promoted frame */
	UPROPERTY(VisibleAnywhere, Category = "Annotations")
	FTransform HeadAlignment;

	/** The name of frame as given by the user */
	UPROPERTY(EditAnywhere, Category = "Frame")
	FText FrameName;

	/** Whether or not the markers (landmarks) of this Promoted Frame are active */
	UPROPERTY(EditAnywhere, Category = "Frame")
	uint8 bUseToSolve : 1;

	/** Whether or not the navigation is locked for this Promoted frame */
	UPROPERTY(EditAnywhere, Category = "Frame")
	uint8 bIsNavigationLocked : 1;

	/** Whether or not track on change is enabled */
	UPROPERTY(Transient)
	uint8 bTrackOnChange : 1;

	/** The tracker that can be used to track landmarks on the data represented by this Promoted Frame */
	UPROPERTY(EditAnywhere, Category = "Trackers")
	TObjectPtr<class UMetaHumanFaceContourTrackerAsset> ContourTracker;

	/** The object that contains the low-level representation for the landmarks of this Promoted Frame */
	TSharedPtr<FShapeAnnotationWrapper> ShapeAnnotation;

	UPROPERTY(EditAnywhere, Category = "Annotations")
	FFrameTrackingContourData ContourData;

	/** The group state data for this promoted frame */
	UPROPERTY(EditAnywhere, Category = "Annotations")
	TMap<FString, FMarkerGroupState> GroupStates;

private:

	/** Initializes the shape annotation with the data stored in the ContourData */
	void InitializeShapeAnnotation();
};

/////////////////////////////////////////////////////
// UMetaHumanIdentityCameraFrame


UCLASS(MinimalAPI)
class UMetaHumanIdentityCameraFrame
	: public UMetaHumanIdentityPromotedFrame
{
	GENERATED_BODY()

public:

	//~ UObject interface
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& InPropertyChangedEvent) override;

public:

	/** Returns the viewport transform as a FTransform object */
	FTransform GetCameraTransform() const;

	FSimpleDelegate& OnCameraTransformChanged()
	{
		return OnCameraTransformChangedDelegate;
	}

public:

	/** The current camera location for this Promoted Frame */
	UPROPERTY(EditAnywhere, Category = "Camera", meta = (EditCondition = "!bIsNavigationLocked"))
	FVector ViewLocation = EditorViewportDefs::DefaultPerspectiveViewLocation;

	/** The current camera rotation for this Promoted Frame */
	UPROPERTY(EditAnywhere, Category = "Camera", meta = (EditCondition = "!bIsNavigationLocked"))
	FRotator ViewRotation = EditorViewportDefs::DefaultPerspectiveViewRotation;

	/** The current camera LookAt position for this Promoted Frame */
	UPROPERTY(EditAnywhere, Category = "Camera", meta = (EditCondition = "!bIsNavigationLocked"))
	FVector LookAtLocation = FVector::ZeroVector;

	/** The Camera FoV from when the view was promoted */
	UPROPERTY(EditAnywhere, Category = "Camera", DisplayName = "Field Of View", meta = (ClampMin = "0", ClampMax = "170", EditCondition = "!bIsNavigationLocked"))
	float CameraViewFOV = EditorViewportDefs::DefaultPerspectiveFOVAngle;

private:

	/** Delegate called when one of the camera transform parameters changes */
	FSimpleDelegate OnCameraTransformChangedDelegate;
};

/////////////////////////////////////////////////////
// UMetaHumanIdentityFootageFrame

UCLASS(MinimalAPI)
class UMetaHumanIdentityFootageFrame
	: public UMetaHumanIdentityPromotedFrame
{
	GENERATED_BODY()

public:
	UPROPERTY(VisibleAnywhere, Category = "Frame")
	int32 FrameNumber;
};