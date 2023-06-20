// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "MetaHumanEditorSettings.generated.h"

DECLARE_MULTICAST_DELEGATE(FMetaHumanEditorSettingsChanged);



UCLASS(Config = EditorPerProjectUserSettings)
class METAHUMANCORE_API UMetaHumanEditorSettings : public UObject
{
	GENERATED_BODY()

public:

	UMetaHumanEditorSettings();

	// Number of samples when using A/B split window - higher value gives better quality but uses more memory
	UPROPERTY(Config, EditAnywhere, Category = "A/B split", meta = (ClampMin = "1", ClampMax = "8", UIMin = "1", UIMax = "8"))
	int32 SampleCount;

	// Maximum effective resolution of A/B split window
	UPROPERTY(Config, EditAnywhere, Category = "A/B split", meta = (ClampMin = "128", ClampMax = "65536", UIMin = "128", UIMax = "65536"))
	int32 MaximumResolution;

	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& InPropertyChangeEvent) override;

	FMetaHumanEditorSettingsChanged OnSettingsChanged;
};
