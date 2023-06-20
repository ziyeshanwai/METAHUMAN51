// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

namespace UE::MetaHuman::Pipeline
{

class FNode;

class METAHUMANPIPELINE_API FConnection
{
public:

	FConnection(const TSharedPtr<FNode>& InFrom, const TSharedPtr<FNode>& InTo, int32 InFromGroup = 0, int32 InToGroup = 0);

	TSharedPtr<FNode> From;
	TSharedPtr<FNode> To;
	int32 FromGroup;
	int32 ToGroup;

	FString ToString() const;
};

}
