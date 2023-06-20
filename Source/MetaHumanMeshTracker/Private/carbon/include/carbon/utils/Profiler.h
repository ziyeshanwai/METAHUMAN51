// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#if defined(EPIC_ENABLE_PROFILING) && EPIC_ENABLE_PROFILING

#if __APPLE__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdefaulted-function-deleted"
#endif
#include <easy/profiler.h>
#if __APPLE__
#pragma clang diagnostic pop
#endif

#define PROFILING_FUNCTION          EASY_FUNCTION
#define PROFILING_BLOCK(...)        EASY_BLOCK(__VA_ARGS__, nullptr)
#define PROFILING_END_BLOCK         EASY_END_BLOCK

#define PROFILING_COLOR_MAGENTA     profiler::colors::Magenta
#define PROFILING_COLOR_RED         profiler::colors::Red
#define PROFILING_COLOR_BLUE        profiler::colors::Blue
#define PROFILING_COLOR_GREEN       profiler::colors::Green
#define PROFILING_COLOR_NAVY        profiler::colors::Navy
#define PROFILING_COLOR_CYAN        profiler::colors::Cyan
#define PROFILING_COLOR_PINK        profiler::colors::Pink
#else
#define PROFILING_FUNCTION(...)     void()
#define PROFILING_BLOCK(...)        void()
#define PROFILING_END_BLOCK         void()

#define PROFILING_COLOR_MAGENTA
#define PROFILING_COLOR_RED
#define PROFILING_COLOR_BLUE
#define PROFILING_COLOR_GREEN
#define PROFILING_COLOR_NAVY
#define PROFILING_COLOR_CYAN
#define PROFILING_COLOR_PINK
#endif

#include <string>

namespace epic {
namespace nls {

bool HasProfiling();
void EnableProfiling();
void DisableProfiling();
void WriteProfilingToFile(const std::string& filename);

} // namespace nls
} //namespace epic
