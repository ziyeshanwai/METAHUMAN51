// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#if defined(__clang__)
#if defined(WITH_EDITOR)
#include "HAL/Platform.h"
#define MH_DISABLE_EIGEN_WARNINGS THIRD_PARTY_INCLUDES_START // use UE warning suppression if we can
#define MH_ENABLE_WARNINGS THIRD_PARTY_INCLUDES_END
#else
#define MH_DISABLE_EIGEN_WARNINGS \
					_Pragma("clang diagnostic push") \
					_Pragma("clang diagnostic ignored \"-Wshadow\"") \
					_Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"")
#define MH_ENABLE_WARNINGS \
					_Pragma("clang diagnostic pop")
#endif
#else
#ifdef _MSC_VER

#define MH_DISABLE_EIGEN_WARNINGS \
			__pragma(warning(push)) \
			__pragma(warning(disable:4305 4127 4459 4820 4626 4996 5027 5031 4723 6001 6011 6255 6294 6313 6325 6385 6386 6387 28182))
#define MH_ENABLE_WARNINGS __pragma(warning(pop))
#else
#define MH_DISABLE_EIGEN_WARNINGS
#define MH_ENABLE_WARNINGS
#endif
#endif
