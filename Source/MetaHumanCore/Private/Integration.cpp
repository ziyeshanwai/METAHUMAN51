// Copyright Epic Games, Inc. All Rights Reserved.

#include <Integration.h>
#include <pma/resources/DefaultMemoryResource.h>

namespace epic
{
namespace core
{
	Integration& GetIntegrationParams()
	{
		static pma::DefaultMemoryResource dmr;
		static Integration params = { Logger(), &dmr };
		return params;
	}

}//namespace core
}//namespace epic

//Singletone instances
#define GLOGGER epic::core::GetIntegrationParams().logger
#define GMEM_RESOURCE epic::core::GetIntegrationParams().memoryResource
