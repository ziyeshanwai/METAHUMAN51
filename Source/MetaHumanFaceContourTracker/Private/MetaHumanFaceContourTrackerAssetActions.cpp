// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanFaceContourTrackerAssetActions.h"
#include "MetaHumanFaceContourTrackerAsset.h"

/* FAssetTypeActions_Base overrides
 *****************************************************************************/


uint32 FMetaHumanFaceContourTrackerAssetActions::GetCategories()
{
	return EAssetTypeCategories::Animation;
}


FText FMetaHumanFaceContourTrackerAssetActions::GetName() const
{
	return NSLOCTEXT("MetaHuman", "MetaHumanFaceContourTrackerAssetName", "Face Contour Tracker");
}


UClass* FMetaHumanFaceContourTrackerAssetActions::GetSupportedClass() const
{
	return UMetaHumanFaceContourTrackerAsset::StaticClass();
}


FColor FMetaHumanFaceContourTrackerAssetActions::GetTypeColor() const
{
	return FColor::White;
}
