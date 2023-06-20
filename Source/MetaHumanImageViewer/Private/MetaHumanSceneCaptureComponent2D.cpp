// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanSceneCaptureComponent2D.h"
#include "EditorViewportClient.h"

UMetaHumanSceneCaptureComponent2D::UMetaHumanSceneCaptureComponent2D(const FObjectInitializer& InObjectInitializer) : Super(InObjectInitializer)
{
	CaptureMesh = nullptr;
}

void UMetaHumanSceneCaptureComponent2D::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	check(ViewportClientRef.IsValid());

	TSharedPtr<FEditorViewportClient> ViewportClient = ViewportClientRef.Pin();

	// Need to disable orbit camera before getting or setting actor position so that the viewport camera location is converted back
	ViewportClient->ToggleOrbitCamera(false);

	float CurrentFOVAngle = ViewportClient->ViewFOV;
	float CurrentCustomNearClippingPlane = ViewportClient->GetNearClipPlane();
	FRotator CurrentViewRotation = ViewportClient->GetViewRotation();
	FVector CurrentViewLocation = ViewportClient->GetViewLocation();

	if (!bUseCaching || CurrentFOVAngle != CachedFOVAngle || CurrentCustomNearClippingPlane != CachedCustomNearClippingPlane || CurrentViewRotation != CachedViewRotation || CurrentViewLocation != CachedViewLocation)
	{
		CachedFOVAngle = CurrentFOVAngle;
		CachedCustomNearClippingPlane = CurrentCustomNearClippingPlane;
		CachedViewRotation = CurrentViewRotation;
		CachedViewLocation = CurrentViewLocation;

		FOVAngle = CurrentFOVAngle;
		bOverride_CustomNearClippingPlane = true;
		CustomNearClippingPlane = CurrentCustomNearClippingPlane;

		SetWorldTransform(FTransform{ CurrentViewRotation, CurrentViewLocation });

		Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	}
}

void UMetaHumanSceneCaptureComponent2D::SetViewportClient(TWeakPtr<FEditorViewportClient> InViewportClient)
{
	ViewportClientRef = InViewportClient;
}

void UMetaHumanSceneCaptureComponent2D::SetUseCaching(bool bInUseCaching)
{
	bUseCaching = bInUseCaching;
}

void UMetaHumanSceneCaptureComponent2D::InvalidateCache()
{
	CachedFOVAngle = -1;
}
