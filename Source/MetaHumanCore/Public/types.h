// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once
namespace epic
{
namespace core
{

enum class LogLevel
{
	VERBOSE = 0,
	WARNING = 1,
	INFO = 2,
	DEBUG = 3,
	ERR = 4,
	CRITICAL = 5,
	FATAL = 6
};

using LogFunction = void(*)(LogLevel /*logLevel*/, const char* /*format*/, ...);

}//namespace core
}//namespace epic
