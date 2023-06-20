// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "ThumbnailRendering/DefaultSizedThumbnailRenderer.h"
#include "ThumbnailHelpers.h"

#include "MetaHumanIdentityThumbnailRenderer.generated.h"

/////////////////////////////////////////////////////
// FMetaHumanIdentityThumbnailScene

class FMetaHumanIdentityThumbnailScene
	: public FThumbnailPreviewScene
{
public:

	FMetaHumanIdentityThumbnailScene();

	void SetMetaHumanIdentity(class UMetaHumanIdentity* InIdentity);

protected:

	//~ FThumbnailPreviewScene interface
	virtual void GetViewMatrixParameters(const float InFOVDegrees, FVector& OutOrigin, float& OutOrbitPitch, float& OutOrbitYaw, float& OutOrbitZoom) const override;

private:

	TWeakObjectPtr<class UMetaHumanIdentity> Identity;
};

/////////////////////////////////////////////////////
// UMetaHumanIdentityThumbnailRenderer

UCLASS(MinimalAPI)
class UMetaHumanIdentityThumbnailRenderer
	: public UDefaultSizedThumbnailRenderer
{
	GENERATED_BODY()

public:
	//~UDefaultSizedThumbnailRenderer interface
	virtual bool CanVisualizeAsset(UObject* InObject) override;
	virtual void Draw(UObject* InObject, int32 InX, int32 InY, uint32 InWidth, uint32 InHeight, class FRenderTarget* InRenderTarget, class FCanvas* InCanvas, bool bInAdditionalViewFamily) override;
	virtual void BeginDestroy() override;

private:

	TUniquePtr<FMetaHumanIdentityThumbnailScene> ThumbnailScene;
};