// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Templates/SubclassOf.h"

#include "CaptureData.generated.h"

/////////////////////////////////////////////////////
// UCaptureData

// Delegate called when something changes in the capture data that others should know about
DECLARE_MULTICAST_DELEGATE(FOnCaptureDataInternalsChanged)

UCLASS(Abstract, MinimalAPI, BlueprintType)
class UCaptureData : public UObject
{
	GENERATED_BODY()

public:
	/** Creates a scene component that can represent this capture source */
	virtual class USceneComponent* CreatePreviewComponent(UObject* InOwner) const PURE_VIRTUAL(UCaptureData::CreatePreviewComponent, return nullptr;);

	/** Returns the class that can be used to instantiate a Promoted Frame for this capture data type */
	virtual TSubclassOf<class UMetaHumanIdentityPromotedFrame> GetPromotedFrameClass() const PURE_VIRTUAL(UCaptureData::GetPromotedFrameClass, return nullptr;);

	/** Returns true is the capture data is fully initialized with all required information present */
	virtual bool IsInitialized() const PURE_VIRTUAL(UCaptureData::IsInitialized, return false;);

	FOnCaptureDataInternalsChanged& OnCaptureDataInternalsChanged()
	{
		return OnCaptureDataInternalsChangedDelegate;
	}

protected:
	/** Notify that something internal to the capture data changed */
	void NotifyInternalsChanged();

private:
	FOnCaptureDataInternalsChanged OnCaptureDataInternalsChangedDelegate;
};

/////////////////////////////////////////////////////
// UMeshCaptureData

/** MetaHuman Capture Data (Mesh) */
UCLASS(MinimalAPI, BlueprintType)
class UMeshCaptureData : public UCaptureData
{
	GENERATED_BODY()

public:
	// UCaptureData interface
	virtual class USceneComponent* CreatePreviewComponent(class UObject* InOwner) const override;
	virtual TSubclassOf<class UMetaHumanIdentityPromotedFrame> GetPromotedFrameClass() const override;
	virtual bool IsInitialized() const override;

	// UObject Interface
	virtual bool IsEditorOnly() const;
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& InPropertyChangedEvent) override;
	virtual void PostTransacted(const FTransactionObjectEvent& InTransactionEvent) override;

public:
	/** Gets the data in the format expected by the face fitting API */
	void GetDataForConforming(const FTransform& InTransform, TArray<float>& OutVertices, TArray<int32>& OutTriangles) const;

	// The target mesh for conforming. This can be either a Static or SkeletalMesh
	UPROPERTY(EditAnywhere, Category = "Capture", Meta = (AllowedClasses = "/Script/Engine.StaticMesh, /Script/Engine.SkeletalMesh"))
	TObjectPtr<class UObject> TargetMesh;
};

/////////////////////////////////////////////////////
// UFootageCaptureData

UCLASS(MinimalAPI, BlueprintType)
class UFootageCaptureData : public UCaptureData
{
	GENERATED_BODY()

public:
	// UCaptureData interface
	virtual class USceneComponent* CreatePreviewComponent(class UObject* InOwner) const override;
	virtual TSubclassOf<class UMetaHumanIdentityPromotedFrame> GetPromotedFrameClass() const override;
	virtual bool IsInitialized() const override;

	// UObject Interface
	virtual bool IsEditorOnly() const;
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& InPropertyChangedEvent) override;
	virtual void PostTransacted(const FTransactionObjectEvent& InTransactionEvent) override;

public:
	UPROPERTY(EditAnywhere, Category = "Capture")
	TObjectPtr<class UImgMediaSource> ImageSequence;
};