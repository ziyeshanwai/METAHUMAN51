// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/common/Log.h>


#if defined(__clang__) // note that _MSC_VER can also be defined for clang on Windows so do this first
    #define CARBON_SUPRESS_MS_WARNING(Warning)
    #define CARBON_REENABLE_MS_WARNING

    #if defined(WITH_EDITOR)
        #include "HAL/Platform.h"
        #define CARBON_DISABLE_EIGEN_WARNINGS THIRD_PARTY_INCLUDES_START // use UE warning suppression if we can
        #define CARBON_RENABLE_WARNINGS THIRD_PARTY_INCLUDES_END
    #else
        #define CARBON_DISABLE_EIGEN_WARNINGS \
                    _Pragma("clang diagnostic push") \
                    _Pragma("clang diagnostic ignored \"-Wshadow\"") \
                    _Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"")
        #define CARBON_RENABLE_WARNINGS \
                    _Pragma("clang diagnostic pop")

    #endif
#elif defined(_MSC_VER)
    // unfortunately UE warning suppression does not do the job right now on MSVC so do our own
    #define CARBON_DISABLE_EIGEN_WARNINGS \
            __pragma(warning(push)) \
            __pragma(warning(disable:4305 4127 4459 4820 4626 4996 5027 5031 4723 6001 6011 6255 6294 6313 6326 6385 6386 6387 28182))
    #define CARBON_RENABLE_WARNINGS __pragma(warning(pop))
    #define CARBON_SUPRESS_MS_WARNING(Warning) \
            __pragma(warning(push)) \
            __pragma(warning(disable:Warning))
    #define CARBON_REENABLE_MS_WARNING CARBON_RENABLE_WARNINGS
#elif !defined(__APPLE__) && defined(__unix__)
    #define CARBON_DISABLE_EIGEN_WARNINGS \
            _Pragma("GCC diagnostic push") \
            _Pragma("GCC diagnostic ignored \"-Wunused-value\"") \
            _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
    #define CARBON_RENABLE_WARNINGS _Pragma("GCC diagnostic pop")
    #define CARBON_SUPRESS_MS_WARNING(Warning)
    #define CARBON_REENABLE_MS_WARNING
#else
    #define CARBON_DISABLE_EIGEN_WARNINGS
    #define CARBON_RENABLE_WARNINGS
    #define CARBON_SUPRESS_MS_WARNING(Warning)
    #define CARBON_REENABLE_MS_WARNING
#endif




#ifdef NDEBUG
    #define DEBUG_PRECONDITION(condition, message) (void)(condition)
#else
    #define DEBUG_PRECONDITION(condition, message) if (!(condition)) throw std::runtime_error(message)
#endif


// API symbol export import preprocessor defines.
// Currently Carbon is forced to be a static library, so no need for export/import defines
#define EPIC_CARBON_API
/*
#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)
    #define EPIC_CARBON_API __attribute__((__visibility__("default")))
#elif defined(_MSC_VER)
    #ifdef EPIC_CARBON_STATIC
        #define EPIC_CARBON_API
    #elif defined(EPIC_CARBON_EXPORT)
        #define EPIC_CARBON_API __declspec(dllexport)
    #else
        #define EPIC_CARBON_API __declspec(dllimport)
    #endif
#endif
*/

// CONSTANTS
#define CARBON_PI       3.14159265358979323846  // pi

#define CARBON_CRITICAL(...) { \
        LOG_CRITICAL(__VA_ARGS__) \
        throw std::runtime_error(epic::carbon::fmt::format("Critical error ({}, l{}): {}", \
                                                           epic::carbon::ConstexprBasename(__FILE__), __LINE__, \
                                                           epic::carbon::fmt::format(__VA_ARGS__).c_str())); \
} ((void)0)

#if !defined(NDEBUG) || (defined(CARBON_ENABLE_ASSERTS) && CARBON_ENABLE_ASSERTS)
    #define CARBON_PRECONDITION(condition, ...)   if (!(condition)) { \
            LOG_PRECONDITION(#condition, __VA_ARGS__); \
            throw std::runtime_error(epic::carbon::fmt::format("Failed precondition ({}, l{}): {}", \
                                                               epic::carbon::ConstexprBasename(__FILE__), __LINE__, \
                                                               epic::carbon::fmt::format(__VA_ARGS__).c_str())); \
    } ((void)0)
    #define CARBON_POSTCONDITION(condition, ...)  if (!(condition)) { \
            LOG_POSTCONDITION(#condition, __VA_ARGS__); \
            throw std::logic_error(epic::carbon::fmt::format("Failed postcondition ({}, l{}): {}", \
                                                             epic::carbon::ConstexprBasename(__FILE__), __LINE__, \
                                                             epic::carbon::fmt::format(__VA_ARGS__).c_str())); \
    } ((void)0)
    #define CARBON_ASSERT(condition, ...)         if (!(condition)) { \
            LOG_ASSERT(#condition, __VA_ARGS__); \
            throw std::logic_error(epic::carbon::fmt::format("Failed assert ({}, l{}): {}", \
                                                             epic::carbon::ConstexprBasename(__FILE__), __LINE__, \
                                                             epic::carbon::fmt::format(__VA_ARGS__).c_str())); \
    } ((void)0)
#else
    #define CARBON_PRECONDITION(condition, ...)   (void)(condition)
    #define CARBON_POSTCONDITION(condition, ...)  (void)(condition)
    #define CARBON_ASSERT(condition, ...)         (void)(condition)
#endif
