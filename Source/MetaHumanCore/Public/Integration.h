// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <Logger.h>
#include <pma/MemoryResource.h>

namespace epic
{
namespace core
{

/**
* @Brief Integration structure wraps objects that implements core functionalities from the external client.
*/
struct Integration
{
	Logger logger{};
	pma::MemoryResource* memoryResource{};
};

/**
* @Brief Gets/Sets Integration struct to be used in the core tech lib
*
* @returns Integraton parameters - singleton instance.
*/
COREAPI Integration& GetIntegrationParams();

}//namespace core
}//namespace epic
