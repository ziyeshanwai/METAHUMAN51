// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/utils/Profiler.h>

namespace epic {
namespace nls {

bool HasProfiling()
{
#if defined(EPIC_ENABLE_PROFILING) && EPIC_ENABLE_PROFILING
    return true;
#else
    return false;
#endif
}

void EnableProfiling()
{
#if defined(EPIC_ENABLE_PROFILING) && EPIC_ENABLE_PROFILING
    EASY_PROFILER_ENABLE;
#endif
}

void DisableProfiling()
{
#if defined(EPIC_ENABLE_PROFILING) && EPIC_ENABLE_PROFILING
    EASY_PROFILER_DISABLE;
#endif
}

void WriteProfilingToFile(const std::string& filename)
{
#if defined(EPIC_ENABLE_PROFILING) && EPIC_ENABLE_PROFILING
    profiler::dumpBlocksToFile(filename.c_str());
#else
    (void)filename;
#endif
}

} // namespace nls
} //namespace epic
