// Copyright Epic Games, Inc. All Rights Reserved.

#include <Logger.h>
#include <stdarg.h>
#include <stdio.h>

namespace epic
{
namespace core
{
    Logger::Logger() noexcept = default;

    Logger::Logger(LogFunction logFunction) noexcept :
        m_logFunction(logFunction)
    {
    }

    Logger::~Logger() noexcept = default;
   
    void Logger::DefaultLogger(epic::core::LogLevel /*logLevel*/, const char* format, ...)
    {
        va_list Args;
        va_start(Args, format);
        vprintf(format, Args);
        va_end(Args);
    }

}//namespace core
}//namespace epic
