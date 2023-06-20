// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "MetaHumanEditorSettings.h"



UMetaHumanEditorSettings::UMetaHumanEditorSettings()
{
	SampleCount = 2;
	MaximumResolution = 8192;
}

void UMetaHumanEditorSettings::PostEditChangeProperty(struct FPropertyChangedEvent& InPropertyChangeEvent)
{ 
	Super::PostEditChangeProperty(InPropertyChangeEvent);

	OnSettingsChanged.Broadcast(); 
}
