// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Factories/Factory.h"

#include "CaptureDataFactoryNew.generated.h"


//////////////////////////////////////////////////////////////////////////
// UMeshCaptureDataFactoryNew

UCLASS(hidecategories=Object)
class UMeshCaptureDataFactoryNew
	: public UFactory
{
	GENERATED_BODY()

public:
	//~ UFactory Interface
	UMeshCaptureDataFactoryNew();

	virtual UObject* FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags InFlags, UObject* InContext, FFeedbackContext* InWarn) override;
};

//////////////////////////////////////////////////////////////////////////
// UFootageCaptureDataFactoryNew

UCLASS(hideCategories=Object)
class UFootageCaptureDataFactoryNew
	: public UFactory
{
	GENERATED_BODY()

public:
	//~ UFactory Interface
	UFootageCaptureDataFactoryNew();

	virtual UObject* FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags InFlags, UObject* InContext, FFeedbackContext* InWarn) override;
};