// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <exception>

#include <carbon/common/Format.h>

namespace epic {
namespace carbon {

// ! @returns the file basename as a constexpr
constexpr const char* ConstexprBasename(const char* path) {
    const char* file = path;
    while (*path) {
        const char c = *path++;
        if ((c == '/') || (c == '\\')) {
            file = path;
        }
    }
    return file;
}

}  // namespace carbon
}  // namespace epic

#include <carbon/common/External.h>
#include <types.h>

#define LOG_INTERNAL(LogLevel,...)  LOGGER.Log(LogLevel, "(%s, l%d): %s\n\0", epic::carbon::ConstexprBasename(__FILE__), static_cast<int>(__LINE__),\
epic::carbon::fmt::format(__VA_ARGS__).c_str())

#define LOG_CONDITION(format,condition,args)  LOGGER.Log(epic::core::LogLevel::ERR, format, condition, epic::carbon::ConstexprBasename(__FILE__), static_cast<int>(__LINE__),args)
 
#define LOG_INFO(...)  LOGGER.Log(epic::core::LogLevel::INFO, "%s\n\0", epic::carbon::fmt::format(__VA_ARGS__).c_str())

#define LOG_WARNING(...) LOG_INTERNAL(epic::core::LogLevel::WARNING, __VA_ARGS__)

#define LOG_ERROR(...) LOG_INTERNAL(epic::core::LogLevel::ERR, __VA_ARGS__)

#define LOG_CRITICAL(...) LOG_INTERNAL(epic::core::LogLevel::CRITICAL, __VA_ARGS__);

#define LOG_VERBOSE(...) LOG_INTERNAL(epic::core::LogLevel::VERBOSE, __VA_ARGS__)

#define LOG_FATAL(...) LOG_INTERNAL(epic::core::LogLevel::FATAL, __VA_ARGS__)

#define LOG_PRECONDITION(failedPrecondition, ...) LOG_CONDITION("FAILED PRECONDITION - %s  in (%s, l%d): %s\n\0", epic::carbon::fmt::to_string(failedPrecondition).c_str(), epic::carbon::fmt::format(__VA_ARGS__).c_str());

#define LOG_POSTCONDITION(failedPostcondition, ...) LOG_CONDITION("FAILED POSTCONDITION - %s  in (%s, l%d): %s\n\0", epic::carbon::fmt::to_string(failedPostcondition).c_str(), epic::carbon::fmt::format(__VA_ARGS__).c_str());

#define LOG_ASSERT(failedAssert, ...) LOG_CONDITION("FAILED ASSERT - %s  in (%s, l%d): %s\n\0", epic::carbon::fmt::to_string(failedAssert).c_str(), epic::carbon::fmt::format(__VA_ARGS__).c_str());
