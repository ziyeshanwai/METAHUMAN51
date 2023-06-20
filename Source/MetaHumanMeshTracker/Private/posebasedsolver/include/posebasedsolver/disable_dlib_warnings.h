// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once


#if defined(__clang__) // note that _MSC_VER can also be defined for clang on Windows so do this first
	#if defined(WITH_EDITOR)
		#include "HAL/Platform.h"
		#define POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS THIRD_PARTY_INCLUDES_START // use UE warning suppression if we can
		#define POSEBASEDSOLVER_RENABLE_WARNINGS THIRD_PARTY_INCLUDES_END
		#if defined(TEXT) // undefine duplicate macro in UE
			#undef TEXT
		#endif
	#else
		#define POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS \
							_Pragma("clang diagnostic push") \
							_Pragma("clang diagnostic ignored \"-Wshadow\"") \
							_Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"")
							_Pragma("clang diagnostic ignored \"-Wunused-local-typedef\"")
							_Pragma("clang diagnostic ignored \"-Wdollar-in-identifier-extension\"")
		#define POSEBASEDSOLVER_RENABLE_WARNINGS \
							_Pragma("clang diagnostic pop")
	#endif
#else
	#ifdef _MSC_VER
	// unfortunately UE warning suppression does not do the job right now on MSVC so do our own
	#define POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS \
				__pragma(warning(push)) \
				__pragma(warning(disable:4244 4456 4458 4459 4946 4005 4127 4267 6246 6255 6326 6385 4100))
	#define POSEBASEDSOLVER_RENABLE_WARNINGS __pragma(warning(pop))
	#else
		#define POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
		#define POSEBASEDSOLVER_RENABLE_WARNINGS
	#endif
#endif




