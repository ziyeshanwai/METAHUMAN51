// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentityThumbnailRenderer.h"
#include "MetaHumanIdentity.h"
#include "MetaHumanIdentityParts.h"
#include "MetaHumanIdentityPromotedFrames.h"
#include "CaptureData.h"

#include "Components/StaticMeshComponent.h"
#include "ThumbnailRendering/SceneThumbnailInfo.h"
#include "SceneView.h"

static USceneComponent* GetNeutralPoseMeshComponentFromMetaHumanIdentity(UMetaHumanIdentity* InIdentity)
{
	if (InIdentity != nullptr)
	{
		if (UMetaHumanIdentityFace* FacePart = InIdentity->FindPartOfClass<UMetaHumanIdentityFace>())
		{
			if (UMetaHumanIdentityPose* NeutralPose = FacePart->FindPoseByType(EIdentityPoseType::Neutral))
			{
				return NeutralPose->CaptureDataSceneComponent;
			}
		}
	}

	return nullptr;
}

/////////////////////////////////////////////////////
// FMetaHumanIdentityThumbnailScene

FMetaHumanIdentityThumbnailScene::FMetaHumanIdentityThumbnailScene()
	: FThumbnailPreviewScene{}
{
	bForceAllUsedMipsResident = false;
}

void FMetaHumanIdentityThumbnailScene::SetMetaHumanIdentity(UMetaHumanIdentity* InIdentity)
{
	if (Identity.IsValid())
	{
		if (USceneComponent* PreviewComponent = GetNeutralPoseMeshComponentFromMetaHumanIdentity(Identity.Get()))
		{
			RemoveComponent(PreviewComponent);
		}
	}

	if (InIdentity != nullptr)
	{
		Identity = InIdentity;

		if (USceneComponent* PreviewComponent = GetNeutralPoseMeshComponentFromMetaHumanIdentity(InIdentity))
		{
			AddComponent(PreviewComponent, PreviewComponent->GetComponentTransform());
		}
	}
}

void FMetaHumanIdentityThumbnailScene::GetViewMatrixParameters(const float InFOVDegrees, FVector& OutOrigin, float& OutOrbitPitch, float& OutOrbitYaw, float& OutOrbitZoom) const
{
	// This should never be called if CanVisualizeAsset returns false so these checks should never fail
	check(Identity.IsValid());
	USceneComponent* NeutralPoseMeshComponent = GetNeutralPoseMeshComponentFromMetaHumanIdentity(Identity.Get());
	check(NeutralPoseMeshComponent != nullptr);

	UMetaHumanIdentityThumbnailInfo* ThumbnailInfo = Cast<UMetaHumanIdentityThumbnailInfo>(Identity->ThumbnailInfo);
	if (ThumbnailInfo == nullptr)
	{
		ThumbnailInfo = UMetaHumanIdentityThumbnailInfo::StaticClass()->GetDefaultObject<UMetaHumanIdentityThumbnailInfo>();
	}

	if (UMetaHumanIdentityFace* FacePart = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
	{
		if (UMetaHumanIdentityPose* NeutralPose = FacePart->FindPoseByType(EIdentityPoseType::Neutral))
		{
			if (!NeutralPose->PromotedFrames.IsEmpty())
			{
				const int32 PromotedFrameIndex = ThumbnailInfo->OverridePromotedFrame < NeutralPose->PromotedFrames.Num() ? ThumbnailInfo->OverridePromotedFrame : 0;
				if (UMetaHumanIdentityCameraFrame* PromotedFrame = Cast<UMetaHumanIdentityCameraFrame>(NeutralPose->PromotedFrames[PromotedFrameIndex]))
				{
					const FVector& ViewDirection = PromotedFrame->ViewLocation - PromotedFrame->LookAtLocation;

					// Calculate the Yaw and Pitch by performing a conversion from Cartesian to spherical coordinates
					// By default the thumbnail renderer will rotate the view by 90 degrees so remove that rotation here as we
					// want to preserve exactly what is set in the promoted frame
					const float OrbitYaw = -HALF_PI - FMath::Atan2(ViewDirection.Y, ViewDirection.X);
					const float OrbitPitch = FMath::Atan2(FMath::Sqrt(FMath::Square(ViewDirection.X) + FMath::Square(ViewDirection.Y)), ViewDirection.Z) - HALF_PI;

					OutOrbitYaw = FMath::RadiansToDegrees<float>(OrbitYaw);
					OutOrbitPitch = FMath::RadiansToDegrees<float>(OrbitPitch);
					OutOrigin = -PromotedFrame->LookAtLocation;

					const float BoundsMultiplier = 1.15f;
					OutOrbitZoom = ViewDirection.Length() * BoundsMultiplier;
				}
			}
		}
	}
}

/////////////////////////////////////////////////////
// UMetaHumanIdentityThumbnailRenderer

bool UMetaHumanIdentityThumbnailRenderer::CanVisualizeAsset(UObject* InObject)
{
	if (UMetaHumanIdentity* Identity = Cast<UMetaHumanIdentity>(InObject))
	{
		if (UMetaHumanIdentityFace* FacePart = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
		{
			if (UMetaHumanIdentityPose* NeutralPose = FacePart->FindPoseByType(EIdentityPoseType::Neutral))
			{
				return NeutralPose->CaptureDataSceneComponent != nullptr && !NeutralPose->PromotedFrames.IsEmpty();
			}
		}
	}

	return false;
}

void UMetaHumanIdentityThumbnailRenderer::Draw(UObject* InObject, int32 InX, int32 InY, uint32 InWidth, uint32 InHeight, FRenderTarget* InRenderTarget, class FCanvas* InCanvas, bool bInAdditionalViewFamily)
{
	if (UMetaHumanIdentity* Identity = Cast<UMetaHumanIdentity>(InObject))
	{
		if (!ThumbnailScene.IsValid())
		{
			ThumbnailScene = MakeUnique<FMetaHumanIdentityThumbnailScene>();
		}

		ThumbnailScene->SetMetaHumanIdentity(Identity);

		FSceneViewFamilyContext ViewFamily(FSceneViewFamily::ConstructionValues(InRenderTarget, ThumbnailScene->GetScene(), FEngineShowFlags(ESFIM_Game))
										   .SetTime(UThumbnailRenderer::GetTime())
										   .SetAdditionalViewFamily(bInAdditionalViewFamily));

		ViewFamily.EngineShowFlags.DisableAdvancedFeatures();
		ViewFamily.EngineShowFlags.MotionBlur = 0;
		ViewFamily.EngineShowFlags.LOD = 0;

		RenderViewFamily(InCanvas, &ViewFamily, ThumbnailScene->CreateView(&ViewFamily, InX, InY, InWidth, InHeight));
		ThumbnailScene->SetMetaHumanIdentity(nullptr);
	}
}

void UMetaHumanIdentityThumbnailRenderer::BeginDestroy()
{
	ThumbnailScene.Reset();

	Super::BeginDestroy();
}