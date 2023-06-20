// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Factories/Factory.h"

#include "MetaHumanIdentityFactoryNew.generated.h"

UCLASS(hidecategories = Object)
class UMetaHumanIdentityFactoryNew
	: public UFactory
{
	GENERATED_BODY()

public:
	//~ UFactory Interface
	UMetaHumanIdentityFactoryNew();

	virtual UObject* FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags InFlags, UObject* Context, FFeedbackContext* Warn) override;
};