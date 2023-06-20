// Copyright Epic Games, Inc. All Rights Reserved.


#include "MetaHumanFaceContourTrackerAssetFactoryNew.h"
#include "MetaHumanFaceContourTrackerAsset.h"


/* UMetaHumanFaceContourTrackerAssetFactoryNew structors
 *****************************************************************************/

UMetaHumanFaceContourTrackerAssetFactoryNew::UMetaHumanFaceContourTrackerAssetFactoryNew()
{
	bCreateNew = true;
	bEditAfterNew = true;
	SupportedClass = UMetaHumanFaceContourTrackerAsset::StaticClass();
}


/* UFactory overrides
 *****************************************************************************/

UObject* UMetaHumanFaceContourTrackerAssetFactoryNew::FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags InFlags, UObject* InContext, FFeedbackContext* InWarn)
{
	return NewObject<UMetaHumanFaceContourTrackerAsset>(InParent, InClass, InName, InFlags);
};