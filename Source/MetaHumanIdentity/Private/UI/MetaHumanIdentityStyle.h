// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Styling/SlateStyle.h"

class FMetaHumanIdentityStyle
	: public FSlateStyleSet
{
public:
	static FMetaHumanIdentityStyle& Get();

	static void Register();
	static void Unregister();

private:
	FMetaHumanIdentityStyle();
};