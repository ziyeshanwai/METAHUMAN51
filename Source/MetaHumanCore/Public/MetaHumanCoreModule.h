// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"
#include "AssetTypeCategories.h"

class METAHUMANCORE_API IMetaHumanCoreModule
	: public IModuleInterface
{
public:
	virtual EAssetTypeCategories::Type GetMetaHumanAssetCategoryBit() const = 0;
};