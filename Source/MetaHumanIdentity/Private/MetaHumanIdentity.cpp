// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentity.h"
#include "MetaHumanIdentityParts.h"
#include "MetaHumanIdentityLog.h"

#include "Misc/ITransaction.h"
#include "Templates/SubclassOf.h"

const TCHAR* UMetaHumanIdentity::IdentityTransactionContext = TEXT("MetaHumanIdentityTransaction");

/////////////////////////////////////////////////////
// UMetaHumanIdentityThumbnailInfo

UMetaHumanIdentityThumbnailInfo::UMetaHumanIdentityThumbnailInfo()
	: Super{}
{
	OverridePromotedFrame = 0;
}

/////////////////////////////////////////////////////
// UMetaHumanIdentity

UMetaHumanIdentityPart* UMetaHumanIdentity::FindPartOfClass(TSubclassOf<UMetaHumanIdentityPart> InPartClass) const
{
	const TObjectPtr<UMetaHumanIdentityPart>* FoundPart = Parts.FindByPredicate([InPartClass](UMetaHumanIdentityPart* Part)
	{
		return Part && Part->IsA(InPartClass);
	});

	if (FoundPart != nullptr)
	{
		return *FoundPart;
	}

	return nullptr;
}

bool UMetaHumanIdentity::CanAddPartOfClass(TSubclassOf<UMetaHumanIdentityPart> InPartClass) const
{
	// Only allow distinct parts to be added
	return FindPartOfClass(InPartClass) == nullptr;
}

bool UMetaHumanIdentity::CanAddPoseOfClass(TSubclassOf<UMetaHumanIdentityPose> InPoseClass, EIdentityPoseType InPoseType) const
{
	if (UMetaHumanIdentityFace* FacePart = FindPartOfClass<UMetaHumanIdentityFace>())
	{
		if (InPoseType == EIdentityPoseType::Custom)
		{
			// Can always add a custom pose
			return true;
		}

		// We only support adding poses to the Face
		return FacePart->FindPoseByType(InPoseType) == nullptr;
	}

	return false;
}

bool UMetaHumanIdentity::IsEditorOnly() const
{
	return true;
}
