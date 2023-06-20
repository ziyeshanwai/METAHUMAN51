// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once
#include <Integration.h>

namespace epic
{
namespace carbon
{
	epic::core::Integration& GetIntegrationParams();
}//namespace carbon
}//namespace epic

#define LOGGER epic::carbon::GetIntegrationParams().logger
#define MEM_RESOURCE epic::carbon::GetIntegrationParams().memoryResource
