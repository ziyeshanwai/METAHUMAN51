// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentityModule.h"

#include "MetaHumanIdentity.h"
#include "MetaHumanIdentityParts.h"
#include "MetaHumanIdentityPromotedFrames.h"

#include "AssetTypeActions/AssetTypeActions_MetaHumanIdentity.h"
#include "AssetTypeActions/AssetTypeActions_CaptureData.h"

#include "Customizations/MetaHumanIdentityPoseCustomizations.h"

#include "UI/MetaHumanIdentityStyle.h"

#include "AssetToolsModule.h"
#include "PropertyEditorModule.h"
#include "ThumbnailRendering/ThumbnailManager.h"
#include "ThumbnailRendering/MetaHumanIdentityThumbnailRenderer.h"

void FMetaHumanIdentityModule::StartupModule()
{
	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
	FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

	// Register asset types
	MeshCaptureDataAssetActions = MakeShared<FAssetTypeActions_MeshCaptureData>();
	AssetTools.RegisterAssetTypeActions(MeshCaptureDataAssetActions.ToSharedRef());

	// TODO: Footage capture data type is not yet functional
	// FootageCaptureDataAssetActions = MakeShared<FAssetTypeActions_FootageCaptureData>();
	// AssetTools.RegisterAssetTypeActions(FootageCaptureDataAssetActions.ToSharedRef());

	IdentityAssetActions = MakeShared<FAssetTypeActions_MetaHumanIdentity>();
	AssetTools.RegisterAssetTypeActions(IdentityAssetActions.ToSharedRef());

	PropertyEditorModule.RegisterCustomClassLayout(UMetaHumanIdentity::StaticClass()->GetFName(), FOnGetDetailCustomizationInstance::CreateStatic(&FMetaHumanIdentityCustomization::MakeInstance));
	PropertyEditorModule.RegisterCustomClassLayout(UMetaHumanIdentityPose::StaticClass()->GetFName(), FOnGetDetailCustomizationInstance::CreateStatic(&FMetaHumanIdentityPoseCustomization::MakeInstance));
	PropertyEditorModule.RegisterCustomClassLayout(UMetaHumanIdentityBody::StaticClass()->GetFName(), FOnGetDetailCustomizationInstance::CreateStatic(&FMetaHumanIdentityBodyCustomization::MakeInstance));
	PropertyEditorModule.RegisterCustomPropertyTypeLayout(UMetaHumanIdentityPromotedFrame::StaticClass()->GetFName(), FOnGetPropertyTypeCustomizationInstance::CreateStatic(&FMetaHumanIdentityPromotedFramePropertyCustomization::MakeInstance));

	// Register the style used for the Identity editors
	FMetaHumanIdentityStyle::Register();

	// Register the thumbnail renderer
	UThumbnailManager::Get().RegisterCustomRenderer(UMetaHumanIdentity::StaticClass(), UMetaHumanIdentityThumbnailRenderer::StaticClass());
}

void FMetaHumanIdentityModule::ShutdownModule()
{
	if (FModuleManager::Get().IsModuleLoaded("AssetTools"))
	{
		IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();

		if (MeshCaptureDataAssetActions.IsValid())
		{
			AssetTools.UnregisterAssetTypeActions(MeshCaptureDataAssetActions.ToSharedRef());
		}

		if (FootageCaptureDataAssetActions.IsValid())
		{
			AssetTools.UnregisterAssetTypeActions(FootageCaptureDataAssetActions.ToSharedRef());
		}

		if (IdentityAssetActions.IsValid())
		{
			AssetTools.UnregisterAssetTypeActions(IdentityAssetActions.ToSharedRef());
		}
	}

	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

		PropertyEditorModule.UnregisterCustomClassLayout(UMetaHumanIdentity::StaticClass()->GetFName());
		PropertyEditorModule.UnregisterCustomClassLayout(UMetaHumanIdentityPose::StaticClass()->GetFName());
		PropertyEditorModule.UnregisterCustomClassLayout(UMetaHumanIdentityBody::StaticClass()->GetFName());
		PropertyEditorModule.UnregisterCustomPropertyTypeLayout(UMetaHumanIdentityPromotedFrame::StaticClass()->GetFName());
	}

	// Unregister the styles used by the Identity module
	FMetaHumanIdentityStyle::Unregister();
}

IMPLEMENT_MODULE(FMetaHumanIdentityModule, MetaHumanIdentity)