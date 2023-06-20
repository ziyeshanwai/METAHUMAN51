// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/common/External.h>

namespace epic
{
namespace carbon
{

epic::core::Integration& GetIntegrationParams()
{
	static epic::core::Integration params = {epic::core::Logger(), nullptr};
	return params;
}

}//namespace carbon
}//namespace epic
