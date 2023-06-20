// Copyright Epic Games, Inc. All Rights Reserved.

#include "AssetTypeActions_CaptureData.h"
#include "CaptureData.h"
#include "MetaHumanCoreModule.h"

//////////////////////////////////////////////////////////////////////////
// FAssetTypeActions_MeshCaptureData

FText FAssetTypeActions_MeshCaptureData::GetName() const
{
	return NSLOCTEXT("MetaHuman", "MeshCaptureDataAssetName", "Capture Data (Mesh)");
}

uint32 FAssetTypeActions_MeshCaptureData::GetCategories()
{
	const IMetaHumanCoreModule& MetaHumanCoreModule = FModuleManager::GetModuleChecked<IMetaHumanCoreModule>(TEXT("MetaHumanCore"));
	return MetaHumanCoreModule.GetMetaHumanAssetCategoryBit();
}

UClass* FAssetTypeActions_MeshCaptureData::GetSupportedClass() const
{
	return UMeshCaptureData::StaticClass();
}

FColor FAssetTypeActions_MeshCaptureData::GetTypeColor() const
{
	return FColor::Red;
}

void FAssetTypeActions_MeshCaptureData::OpenAssetEditor(const TArray<UObject*>& InObjects, TSharedPtr<class IToolkitHost> InEditWithinLevelEditor /* = TSharedPtr<IToolkitHost>() */)
{
	FAssetTypeActions_Base::OpenAssetEditor(InObjects, InEditWithinLevelEditor);
}

//////////////////////////////////////////////////////////////////////////
// FAssetTypeActions_FootageCaptureData

FText FAssetTypeActions_FootageCaptureData::GetName() const
{
	return NSLOCTEXT("MetaHuman", "FootageCaptureDataAssetName", "Capture Data (Footage)");
}

uint32 FAssetTypeActions_FootageCaptureData::GetCategories()
{
	const IMetaHumanCoreModule& MetaHumanCoreModule = FModuleManager::GetModuleChecked<IMetaHumanCoreModule>(TEXT("MetaHumanCore"));
	return MetaHumanCoreModule.GetMetaHumanAssetCategoryBit();
}

UClass* FAssetTypeActions_FootageCaptureData::GetSupportedClass() const
{
	return UFootageCaptureData::StaticClass();
}

FColor FAssetTypeActions_FootageCaptureData::GetTypeColor() const
{
	return FColor::Red;
}

void FAssetTypeActions_FootageCaptureData::OpenAssetEditor(const TArray<UObject*>& InObjects, TSharedPtr<class IToolkitHost> InEditWithinLevelEditor /* = TSharedPtr<IToolkitHost>() */)
{
	FAssetTypeActions_Base::OpenAssetEditor(InObjects, InEditWithinLevelEditor);
}