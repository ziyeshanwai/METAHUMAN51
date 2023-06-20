// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"

class FMetaHumanIdentityModule
	: public IModuleInterface
{
public:

	//~ IModuleInterface interface
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	TSharedPtr<class FAssetTypeActions_MeshCaptureData> MeshCaptureDataAssetActions;
	TSharedPtr<class FAssetTypeActions_FootageCaptureData> FootageCaptureDataAssetActions;
	TSharedPtr<class FAssetTypeActions_MetaHumanIdentity> IdentityAssetActions;
};