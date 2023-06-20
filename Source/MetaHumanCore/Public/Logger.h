// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <types.h>
#include "Defs.h"

namespace epic
{ 
namespace core
{

/**
* @Brief Logger class is simple logging functionality wrapper. Its main purpose is integration 
* of logging functionality from external system (for example UE)
* 
*/
class COREAPI Logger final
{
public:
    /**
    * Default Constructor.
    *
    */
    Logger() noexcept;

    /**
    * Constructor.
    *
    * @param[in] logFunction_ logging function provided from the external system. Initially set to DefaultLogger.
    *
    */
    Logger(LogFunction logFunction_) noexcept;

    /**
    * Destructor.
    */
    ~Logger() noexcept;

    /**
    * Send message to log system.
    *
    * @param[in] logLevel Log level of message.
    * @param[in] format format string.
    * @param[in] params multiple parameters to be embeded in the format string.
    *
    * @note
    *     C style formatting syntax needs to be applied
    *
    */
    template<typename ...ARGS>
    void Log(LogLevel logLevel, const char* format, ARGS&& ...params)
    {
        m_logFunction(logLevel, format, params...);
    }

private:
    static void DefaultLogger(epic::core::LogLevel logLevel, const char* format, ...);

    LogFunction m_logFunction{ DefaultLogger };
};

}
}
