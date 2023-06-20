// Copyright Epic Games, Inc. All Rights Reserved.

#include "AssetTypeActions_MetaHumanIdentity.h"
#include "MetaHumanIdentity.h"
#include "MetaHumanIdentityAssetEditor.h"
#include "MetaHumanCoreModule.h"

FText FAssetTypeActions_MetaHumanIdentity::GetName() const
{
	return NSLOCTEXT("MetaHuman", "MetaHumanIdentityAssetName", "MetaHuman Identity");
}

uint32 FAssetTypeActions_MetaHumanIdentity::GetCategories()
{
	const IMetaHumanCoreModule& MetaHumanCoreModule = FModuleManager::GetModuleChecked<IMetaHumanCoreModule>(TEXT("MetaHumanCore"));
	return MetaHumanCoreModule.GetMetaHumanAssetCategoryBit();
}

UClass* FAssetTypeActions_MetaHumanIdentity::GetSupportedClass() const
{
	return UMetaHumanIdentity::StaticClass();
}

FColor FAssetTypeActions_MetaHumanIdentity::GetTypeColor() const
{
	return FColor::Blue;
}

UThumbnailInfo* FAssetTypeActions_MetaHumanIdentity::GetThumbnailInfo(UObject* InAsset) const
{
	UMetaHumanIdentity* Identity = CastChecked<UMetaHumanIdentity>(InAsset);
	if (Identity->ThumbnailInfo == nullptr)
	{
		Identity->ThumbnailInfo = NewObject<UMetaHumanIdentityThumbnailInfo>(Identity, NAME_None, RF_Transactional);
	}
	return Identity->ThumbnailInfo;
}

void FAssetTypeActions_MetaHumanIdentity::OpenAssetEditor(const TArray<UObject*>& InObjects, TSharedPtr<class IToolkitHost> InEditWithinLevelEditor /* = TSharedPtr<IToolkitHost>() */)
{
	for (UObject* Object : InObjects)
	{
		if (UMetaHumanIdentity* Identity = Cast<UMetaHumanIdentity>(Object))
		{
			if (UAssetEditorSubsystem* AssetEditorSubsystem = GEditor->GetEditorSubsystem<UAssetEditorSubsystem>())
			{
				UMetaHumanIdentityAssetEditor* IdentityAssetEditor = NewObject<UMetaHumanIdentityAssetEditor>(AssetEditorSubsystem, NAME_None, RF_Transient);
				IdentityAssetEditor->SetObjectToEdit(Identity);
				IdentityAssetEditor->Initialize();
			}
		}
	}

}