// Copyright Epic Games, Inc. All Rights Reserved.

#include <ScopedTimer.h>

namespace epic
{
namespace core
{

ScopedTimer::ScopedTimer(const char* function_, const Logger& logger_) noexcept:
    m_function{function_},
    m_logger{ logger_ },
	m_pimpl{std::chrono::steady_clock::now() }
{
}

ScopedTimer::~ScopedTimer() noexcept
{
    auto stop_ = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_ - m_pimpl.m_start).count();
	m_logger.Log(LogLevel::DEBUG, "[PROFILING] Function %s : %d ms.\n", m_function, static_cast<int>(duration));
}

}//namespace core
}//namespace epic
