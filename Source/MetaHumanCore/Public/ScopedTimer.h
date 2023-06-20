// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include <types.h>
#include "Defs.h"
#include <chrono>
#include <Logger.h>

namespace epic
{
namespace core
{

/**
* @Brief ScopedTimer is rudimentary profiler that leverages std::chrono timers.
*/
class COREAPI ScopedTimer
{
public:
    /**
    * Constructor.
    *
    * @param[in] functionName_ name of the profiled function. 
    *     Function name can be obtained through __func__ macro.
    *
    * @param[in] logger logging wrapper.
    * 
    */
    ScopedTimer(const char* functionName_, const Logger& logger_) noexcept;

    /**
    * Destructor.
    */
    ~ScopedTimer() noexcept;

private:
    ScopedTimer(ScopedTimer const&) = delete;
    ScopedTimer(ScopedTimer&&) = delete;
    auto operator=(ScopedTimer const&)->ScopedTimer & = delete;
    auto operator=(ScopedTimer &&)->ScopedTimer  = delete;
    auto operator new(std::size_t)->void* = delete;
    auto operator delete(void*, std::size_t)->void = delete;

private:
    const char* m_function{};
    Logger m_logger{ };
	struct pimpl
	{
		const std::chrono::steady_clock::time_point m_start{};
	};
	pimpl m_pimpl{};
};

}//namespace core
}//namespace epic
