// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentityFactoryNew.h"
#include "MetaHumanIdentity.h"

UMetaHumanIdentityFactoryNew::UMetaHumanIdentityFactoryNew()
{
	bCreateNew = true;
	bEditAfterNew = true;
	SupportedClass = UMetaHumanIdentity::StaticClass();
}

UObject* UMetaHumanIdentityFactoryNew::FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags InFlags, UObject* Context, FFeedbackContext* Warn)
{
	return NewObject<UMetaHumanIdentity>(InParent, InClass, InName, InFlags | RF_Transactional);
}