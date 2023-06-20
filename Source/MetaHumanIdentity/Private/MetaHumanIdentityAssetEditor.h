// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Tools/UAssetEditor.h"

#include "MetaHumanIdentityAssetEditor.generated.h"

UCLASS(MinimalAPI)
class UMetaHumanIdentityAssetEditor
	: public UAssetEditor
{
	GENERATED_BODY()

public:
	virtual void GetObjectsToEdit(TArray<UObject*>& InObjectsToEdit) override;
	virtual TSharedPtr<FBaseAssetToolkit> CreateToolkit() override;

	void SetObjectToEdit(UObject* InObject);

protected:
	UPROPERTY()
	TObjectPtr<class UMetaHumanIdentity> ObjectToEdit;
};