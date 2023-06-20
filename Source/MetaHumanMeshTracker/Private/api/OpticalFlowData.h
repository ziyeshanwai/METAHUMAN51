// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once
#include "Defs.h"

namespace titan {
namespace api {

struct TITAN_API OpticalFlowData {
    bool bUseOpticalFlow{};
    bool bUseConfidence{};
    bool bUseForwardFlow{};
};

} // namespace api
} // namespace titan
