// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "Templates/SubclassOf.h"
#include "Textures/SlateIcon.h"
#include "MetaHumanIdentityPromotedFrames.h"

#include "MetaHumanIdentityParts.generated.h"

/////////////////////////////////////////////////////
// UMetaHumanIdentityPose

UENUM()
enum class EIdentityPoseType : uint8
{
	Neutral = 0,
	Teeth,
	Custom
};

DECLARE_MULTICAST_DELEGATE(FOnCaptureDataChanged)

/**
 * A Pose describes the appearance of the Identity in some way. It could be a mesh or footage showing the
 * Identity's teeth or its neutral expression.
 */
UCLASS(MinimalAPI, HideCategories = ("Preview"))
class UMetaHumanIdentityPose
	: public UObject
{
	GENERATED_BODY()

public:
	/** Utility function to convert the EIdentityPoseType to a FString */
	static FString PoseTypeAsString(EIdentityPoseType InPoseType);

	UMetaHumanIdentityPose();

	/** Returns an icon that represents this pose */
	FSlateIcon GetPoseIcon() const;

	UFUNCTION(BlueprintCallable, Category = "Target")
	void SetCaptureData(class UCaptureData* InCaptureData);

	UFUNCTION(BlueprintCallable, Category = "Target")
	class UCaptureData* GetCaptureData() const;

	/** Returns true if the capture is initialized */
	UFUNCTION(BlueprintCallable, Category = "Target")
	bool IsCaptureDataValid() const;

	FOnCaptureDataChanged& OnCaptureDataChanged()
	{
		return OnCaptureDataChangedDelegate;
	}

	/** Returns true iff the default tracker is a valid object that is ready to track */
	bool IsDefaultTrackerValid() const;

	/** Returns all Promoted Frames that contain contour data */
	TArray<class UMetaHumanIdentityPromotedFrame*> GetAllPromotedFramesWithValidContourData() const;

	/** Returns the head alignment transform for a given promoted frame */
	const FTransform& GetHeadAlignment(int32 InFrameIndex = 0);

	/**
	 * Sets the head alignment transform for a promoted frame.
	 * If InFrameIndex is not specified, sets the same transform to all promoted frames
	 */
	void SetHeadAlignment(const FTransform& InTransform, int32 InFrameIndex = INDEX_NONE);

	/** Sets the default tracker based on the PoseType. Only changes it DefaultTracker is not currently set */
	void LoadDefaultTracker();

public:
	// UObject Interface
	virtual bool IsEditorOnly() const;
	virtual void PostLoad();
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& InPropertyChangedEvent) override;
	virtual void PostTransacted(const FTransactionObjectEvent& InTransactionEvent) override;

public:
	/** The display name of the pose. This can be edited for custom poses */
	UPROPERTY(EditAnywhere, Category = "Pose", meta = (EditCondition = "PoseType == EIdentityPoseType::Custom"))
	FText PoseName;

	/** The type this pose represents */
	UPROPERTY(VisibleAnywhere, Category = "Pose")
	EIdentityPoseType PoseType;

	/** The transform for the pose if changed in the viewport */
	UPROPERTY(EditAnywhere, Category = "Pose")
	FTransform PoseTransform;

	/** The default tracker that should be used for tracking a Promoted Frame of this pose. This can still be customized in a per-frame basis */
	// TODO: We need a common interface for trackers
	UPROPERTY(EditAnywhere, Category = "Trackers")
	TObjectPtr<class UMetaHumanFaceContourTrackerAsset> DefaultTracker;

	/** The class the defines the Promoted Frame type for this pose */
	UPROPERTY(VisibleAnywhere, Category = "Frame Promotion")
	TSubclassOf<class UMetaHumanIdentityPromotedFrame> PromotedFrameClass;

	/** The array of Promoted Frames for this pose */
	UPROPERTY(VisibleAnywhere, Instanced, Category = "Frame Promotion", meta = (EditFixedOrder))
	TArray<TObjectPtr<class UMetaHumanIdentityPromotedFrame>> PromotedFrames;

	/** The scene component that represents the capture data for this pose */
	UPROPERTY(VisibleAnywhere, Category = "Preview")
	TObjectPtr<class USceneComponent> CaptureDataSceneComponent;

private:
	void NotifyCaptureDataChanged();
	void NotifyPoseTransformChanged();

	void HandleCaptureDataChanged();
	void HandleCaptureDataSceneComponentTransformChanged(class USceneComponent* InRootComponent, enum class EUpdateTransformFlags, ETeleportType);

	void RegisterCaptureDataInternalsChangedDelegate();
	void RegisterCaptureDataSceneComponentTransformChanged();

	/** Delegated called when the capture data associated with the Pose changes */
	FOnCaptureDataChanged OnCaptureDataChangedDelegate;

	/** Source data for this pose, this could be a mesh or footage */
	UPROPERTY(EditAnywhere, Category = "Target")
	TObjectPtr<class UCaptureData> CaptureData;
};

/////////////////////////////////////////////////////
// UMetaHumanIdentityPart

/**
 * The base class for any Part that can be added to a MetaHumanIdentity
 */
UCLASS(Abstract, MinimalAPI)
class UMetaHumanIdentityPart
	: public UObject
{
	GENERATED_BODY()

public:
	/** Returns the part name */
	virtual FText GetPartName() const PURE_VIRTUAL(UMetaHumanIdentityPart::GetPartName, return {};);

	/** Returns a short description of the part */
	virtual FText GetPartDescription() const PURE_VIRTUAL(UMetaHumanIdentityPart::GetPartDescription, return {};);

	/** Returns the icon for the part. This can optionally return an icon for the given InPropertyName  */
	virtual FSlateIcon GetPartIcon(const FName& InPropertyName = NAME_None) const PURE_VIRTUAL(UMetaHumanIdentityPart::GetPartIcon, return {};);
};

////////////////////////////////////////////////////
// UMetaHumanIdentityFace

namespace UE::Wrappers
{
	class FMetaHumanConformer;
}

UCLASS(MinimalAPI, HideCategories = ("Preview"))
class UMetaHumanIdentityFace
	: public UMetaHumanIdentityPart
{
	GENERATED_BODY()

public:
	UMetaHumanIdentityFace();

	//~UMetaHumanIdentityFace Interface
	virtual FText GetPartName() const override;
	virtual FText GetPartDescription() const override;
	virtual FSlateIcon GetPartIcon(const FName& InPropertyName = NAME_None) const override;

	//~UObject Interface
	virtual void PostLoad() override;

	/** Return true if the face has all the required information to run the identity solve (conforming) */
	bool CanConform() const;

	/** Identity solve */
	void Conform();

	/** Reset the template mesh */
	void ResetTemplateMesh();

	/**
	 * Apply a DNA to the Rig
	 * Depending on the level of detail and usage (e.g.only LOD0 has blend shapes), these options can be turned off to save time/memory
	 */
	void ApplyDNAToRig(TArray<uint8>& InDNABuffer, bool bInUpdateBlendShapes = true, bool bInUpdateSkinWeights = true);

	/** Finds a Pose of given type. Returns nullptr if one is not found. */
	UMetaHumanIdentityPose* FindPoseByType(EIdentityPoseType InPoseType) const;

	/** Sets the transform for the conformal mesh and notifies subscribers */
	void SetConformalMeshTransform(const FTransform& InTransform);

	/** Return the vertices of the conformed mesh transformed to the space required by the autorigging backend */
	TArray<FVector> GetConformalVerticesForAutoRigging() const;

	/** Return world position of conformal mesh vertices */
	TArray<FVector> GetConformalVerticesWorldPos() const;

public:

	/** The conformal mesh for the face. This will be fitted to the input data. See UMetaHumanIdentityPose */
	UPROPERTY(VisibleAnywhere, Category = "Preview", DisplayName = "Template Mesh")
	TObjectPtr<class UDynamicMeshComponent> ConformalMeshComponent;

	/** An array of poses that will be used to fit the conformal mesh to the input data. See UMetaHumanIdentityPose */
	UPROPERTY(VisibleAnywhere, Category = "Poses")
	TArray<TObjectPtr<class UMetaHumanIdentityPose>> Poses;

	/** The result of the auto-rigging process. This is the conformal mesh with a proper rig able to control the face */
	UPROPERTY(VisibleAnywhere, Category = "Output", DisplayName = "Rig")
	TObjectPtr<class USkeletalMeshComponent> RigComponent;

	/** True if this face was conformed at least once */
	UPROPERTY(VisibleAnywhere, Category = "Output", AdvancedDisplay)
	bool bIsConformed;

private:
	/** Initializes the ConformalMesh dynamic mesh if the existing one is empty */
	void InitializeConformalMesh();

	/** Initializes the Rig by copying the Face Archetype provided by the plugin */
	void InitializeRig();

	void SetConformerCameraParameters(UMetaHumanIdentityPose* InPose, UE::Wrappers::FMetaHumanConformer& OutConformer) const;
	void SetConformerScanInputData(const UMetaHumanIdentityPose* InPose, UE::Wrappers::FMetaHumanConformer& OutConformer) const;
	bool RunMeshConformer(UMetaHumanIdentityPose* InPose, UE::Wrappers::FMetaHumanConformer& OutConformer);

	void WriteConformalVerticesToFile(const FString& InNameSuffix = TEXT("")) const;
	void WriteTargetMeshToFile(class UStaticMesh* InTargetMesh, const FString& InNameSuffix = TEXT("")) const;

	FString GetPluginContentDir() const;

	TSet<int32> GetObjToUEVertexMapping() const;
};

/////////////////////////////////////////////////////
// UMetaHumanIdentityBody

UCLASS(MinimalAPI)
class UMetaHumanIdentityBody
	: public UMetaHumanIdentityPart
{
	GENERATED_BODY()

public:
	UMetaHumanIdentityBody();

	//~UMetaHumanIdentityBody Interface
	virtual FText GetPartName() const override;
	virtual FText GetPartDescription() const override;
	virtual FSlateIcon GetPartIcon(const FName& InPropertyName = NAME_None) const override;

public:

	UPROPERTY()
	int32 Height;

	UPROPERTY()
	int32 BodyTypeIndex;
};

/////////////////////////////////////////////////////
// UMetaHumanIdentityHands

UCLASS(MinimalAPI)
class UMetaHumanIdentityHands
	: public UMetaHumanIdentityPart
{
	GENERATED_BODY()

public:
	UMetaHumanIdentityHands();

	//~UMetaHumanIdentityHands Interface
	virtual FText GetPartName() const override;
	virtual FText GetPartDescription() const override;
	virtual FSlateIcon GetPartIcon(const FName& InPropertyName = NAME_None) const override;
};

/////////////////////////////////////////////////////
// UMetaHumanIdentityOutfit

UCLASS(MinimalAPI)
class UMetaHumanIdentityOutfit
	: public UMetaHumanIdentityPart
{
	GENERATED_BODY()

public:
	UMetaHumanIdentityOutfit();

	//~UMetaHumanIdentityOutfit Interface
	virtual FText GetPartName() const override;
	virtual FText GetPartDescription() const override;
	virtual FSlateIcon GetPartIcon(const FName& InPropertyName = NAME_None) const override;
};

/////////////////////////////////////////////////////
// UMetaHumanIdentityProp

UCLASS(MinimalAPI)
class UMetaHumanIdentityProp
	: public UMetaHumanIdentityPart
{
	GENERATED_BODY()

public:
	UMetaHumanIdentityProp();

	//~UMetaHumanIdentityProp Interface
	virtual FText GetPartName() const override;
	virtual FText GetPartDescription() const override;
	virtual FSlateIcon GetPartIcon(const FName& InPropertyName = NAME_None) const override;
};