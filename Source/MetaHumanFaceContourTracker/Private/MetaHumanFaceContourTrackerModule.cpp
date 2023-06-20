// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanFaceContourTrackerModule.h"

#include "MetaHumanFaceContourTrackerAssetFactoryNew.h"
#include "MetaHumanFaceContourTrackerAssetActions.h"

#include "AssetToolsModule.h"
#include "PropertyEditorModule.h"

void FMetaHumanFaceContourTrackerModule::StartupModule()
{
	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();

	FaceContourTrackerAssetActions = MakeShareable(new FMetaHumanFaceContourTrackerAssetActions());
	AssetTools.RegisterAssetTypeActions(FaceContourTrackerAssetActions.ToSharedRef());
}

void FMetaHumanFaceContourTrackerModule::ShutdownModule()
{
	if (FModuleManager::Get().IsModuleLoaded("AssetTools"))
	{
		IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();

		if (FaceContourTrackerAssetActions.IsValid())
		{
			AssetTools.UnregisterAssetTypeActions(FaceContourTrackerAssetActions.ToSharedRef());
		}
	}
}

IMPLEMENT_MODULE(FMetaHumanFaceContourTrackerModule, MetaHumanFaceContourTracker)