// Copyright Epic Games, Inc. All Rights Reserved.

#include "CaptureDataFactoryNew.h"
#include "CaptureData.h"

//////////////////////////////////////////////////////////////////////////
// UMeshCaptureDataFactoryNew

UMeshCaptureDataFactoryNew::UMeshCaptureDataFactoryNew()
{
	bCreateNew = true;
	bEditAfterNew = true;
	SupportedClass = UMeshCaptureData::StaticClass();
}

UObject* UMeshCaptureDataFactoryNew::FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags InFlags, UObject* InContext, FFeedbackContext* InWarn)
{
	return NewObject<UMeshCaptureData>(InParent, InClass, InName, InFlags);
}

//////////////////////////////////////////////////////////////////////////
// UFootageCaptureDataFactoryNew

UFootageCaptureDataFactoryNew::UFootageCaptureDataFactoryNew()
{
	bCreateNew = true;
	bEditAfterNew = true;
	SupportedClass = UFootageCaptureData::StaticClass();
}

UObject* UFootageCaptureDataFactoryNew::FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags InFlags, UObject* InContext, FFeedbackContext* InWarn)
{
	return NewObject<UFootageCaptureData>(InParent, InClass, InName, InFlags);
}