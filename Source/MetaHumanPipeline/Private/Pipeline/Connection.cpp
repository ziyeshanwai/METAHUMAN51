// Copyright Epic Games, Inc. All Rights Reserved.

#include "Pipeline/Connection.h"

namespace UE::MetaHuman::Pipeline
{

FConnection::FConnection(const TSharedPtr<FNode>& InFrom, const TSharedPtr<FNode>& InTo, int32 InFromGroup, int32 InToGroup)
	: From(InFrom), To(InTo), FromGroup(InFromGroup), ToGroup(InToGroup)
{
}

}
