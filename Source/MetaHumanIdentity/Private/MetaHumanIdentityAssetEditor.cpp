// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentityAssetEditor.h"
#include "MetaHumanIdentityAssetEditorToolkit.h"
#include "MetaHumanIdentity.h"

void UMetaHumanIdentityAssetEditor::GetObjectsToEdit(TArray<UObject*>& InObjectsToEdit)
{
	InObjectsToEdit.Add(ObjectToEdit);
}

TSharedPtr<FBaseAssetToolkit> UMetaHumanIdentityAssetEditor::CreateToolkit()
{
	return MakeShared<FMetaHumanIdentityAssetEditorToolkit>(this);
}

void UMetaHumanIdentityAssetEditor::SetObjectToEdit(UObject* InObject)
{
	ObjectToEdit = CastChecked<UMetaHumanIdentity>(InObject);
}