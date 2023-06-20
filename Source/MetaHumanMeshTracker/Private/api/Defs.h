// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifdef TITAN_STATIC_API
    #define TITAN_API
#else
    #ifdef TITAN_DYNAMIC_API
        #if defined(_MSC_VER)
            #define TITAN_API __declspec(dllexport)
        #else
            #define TITAN_API __attribute__ ((visibility("default")))
        #endif
    #else
        #if defined(_MSC_VER)
            #define TITAN_API __declspec(dllimport)
        #else
            #define TITAN_API __attribute__ ((visibility("default")))
        #endif
    #endif
#endif

#ifdef __cplusplus
}
#endif
