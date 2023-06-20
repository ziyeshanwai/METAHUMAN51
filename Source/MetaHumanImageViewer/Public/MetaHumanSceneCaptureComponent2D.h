// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/SceneCaptureComponent2D.h"
#include "MetaHumanSceneCaptureComponent2D.generated.h"

class FEditorViewportClient;

UCLASS()
class METAHUMANIMAGEVIEWER_API UMetaHumanSceneCaptureComponent2D : public USceneCaptureComponent2D
{
public:
	GENERATED_BODY()

	UMetaHumanSceneCaptureComponent2D(const FObjectInitializer& InObjectInitializer);

	//~ USceneCaptureComponent2D interface
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

public:

	// Sets the viewport client that controls this component
	void SetViewportClient(TWeakPtr<class FEditorViewportClient> InPerformerViewportClient);

	void SetUseCaching(bool bInUseCaching);
	void InvalidateCache();

private:

	// A reference to the viewport client that controls this component
	TWeakPtr<FEditorViewportClient> ViewportClientRef;

	bool bUseCaching = false;
	float CachedFOVAngle = -1;
	float CachedCustomNearClippingPlane = -1;
	FRotator CachedViewRotation = FRotator(0, 0, 0);
	FVector CachedViewLocation = FVector(0, 0, 0);
};
