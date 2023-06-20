// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Templates/SubclassOf.h"
#include "EditorFramework/ThumbnailInfo.h"

#include "MetaHumanIdentity.generated.h"

/////////////////////////////////////////////////////
// UMetaHumanIdentityThumbnailInfo

UCLASS(MinimalAPI)
class UMetaHumanIdentityThumbnailInfo
	: public UThumbnailInfo
{
	GENERATED_BODY()

public:
	UMetaHumanIdentityThumbnailInfo();

	/** Override the Promoted Frame index used to generate the MetaHuman Identity thumbnail */
	UPROPERTY(EditAnywhere, Category = "Thumbnail")
	int32 OverridePromotedFrame;
};

/////////////////////////////////////////////////////
// UMetaHumanIdentity

enum class EIdentityPoseType : uint8;

/** MetaHuman Identity */
UCLASS(MinimalAPI, BlueprintType)
class UMetaHumanIdentity : public UObject
{
	GENERATED_BODY()

public:

	/** Looks for a Part of the given class in the array of parts. Returns nullptr if no Part was found */
	UFUNCTION(BlueprintCallable, Category = "MetaHuman Identity")
	class UMetaHumanIdentityPart* FindPartOfClass(TSubclassOf<class UMetaHumanIdentityPart> InPartClass) const;

	/**
	 * Searches for a Part of the given class in the array of parts.
	 * The class being searched must be a child of UMetaHumanIdentityPart.
	 */
	template<typename SearchType>
	SearchType* FindPartOfClass() const
	{
		ensure(SearchType::StaticClass()->IsChildOf(UMetaHumanIdentityPart::StaticClass()));
		return Cast<SearchType>(FindPartOfClass(SearchType::StaticClass()));
	}

	/** Returns true if the given Part class can be added to the Identity being edited */
	UFUNCTION(BlueprintCallable, Category = "MetaHuman Identity")
	bool CanAddPartOfClass(TSubclassOf<class UMetaHumanIdentityPart> InPartClass) const;

	/** Returns true if the given Pose class can be added to the Identity being edited */
	UFUNCTION(BlueprintCallable, Category = "MetaHuman Identity")
	bool CanAddPoseOfClass(TSubclassOf<class UMetaHumanIdentityPose> InPoseClass, EIdentityPoseType InPoseType) const;

public:

	// UObject interface
	virtual bool IsEditorOnly() const;

public:

	/** The list of Parts the make this Identity. See UMetaHumanIdentityPart */
	UPROPERTY(VisibleAnywhere, Category = "MetaHuman Identity")
	TArray<TObjectPtr<class UMetaHumanIdentityPart>> Parts;

	/** Information for thumbnail rendering */
	UPROPERTY(VisibleAnywhere, Instanced, AdvancedDisplay, Category = "MetaHuman Identity")
	TObjectPtr<class UThumbnailInfo> ThumbnailInfo;

public:

	/** The transaction context identifier for transactions done in the Identity being edited */
	static const TCHAR* IdentityTransactionContext;
};