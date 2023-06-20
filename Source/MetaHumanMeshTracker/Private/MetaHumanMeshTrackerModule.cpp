// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanMeshTrackerModule.h"

#include "Misc/OutputDeviceRedirector.h"

#include "carbon/common/External.h"
#include "FMemoryResource.h"

DEFINE_LOG_CATEGORY_STATIC(MetaHumanMeshTracker, Log, All);


namespace
{
	//Per module in order to achieve different log category (in simple and effective way)
	using epic::core::LogLevel;
	ELogVerbosity::Type LogLevel2UELogLevel(LogLevel logLevel)
	{
		switch (logLevel)
		{
		case LogLevel::DEBUG:
		case LogLevel::INFO: return ELogVerbosity::Log;
		case LogLevel::ERR:
		case LogLevel::CRITICAL: return ELogVerbosity::Error;
		case LogLevel::WARNING: return ELogVerbosity::Warning;
		case LogLevel::VERBOSE: return ELogVerbosity::Verbose;
		case LogLevel::FATAL: return ELogVerbosity::Fatal;
		default:return ELogVerbosity::Error;
		}
	}

	void LogFunction(epic::core::LogLevel logLevel, const char* format, ...)
	{
#if !NO_LOGGING
		ELogVerbosity::Type UEVerbosity = LogLevel2UELogLevel(logLevel);
		va_list(Args);
		va_start(Args, format);
		char Buffer[1024];
#ifdef _MSC_VER
		vsprintf_s(Buffer, format, Args);
#else
		vsprintf(Buffer, format, Args);
#endif
		va_end(Args);

		FString MessageW(Buffer);

		// Broadcast the message so other modules can react accordingly
		FMetaHumanMeshTrackerModule::GetModule().OnLogError().Broadcast(UEVerbosity, MessageW);

		GLog->Serialize(*MessageW, UEVerbosity, MetaHumanMeshTracker.GetCategoryName());
#endif
	}
}

void FMetaHumanMeshTrackerModule::StartupModule()
{
	MemoryResource = MakeUnique<FMemoryResource>();
	epic::carbon::GetIntegrationParams() = { epic::core::Logger(LogFunction), MemoryResource.Get()};
}

void FMetaHumanMeshTrackerModule::ShutdownModule()
{
	MemoryResource.Reset();
}

IMPLEMENT_MODULE(FMetaHumanMeshTrackerModule, MetaHumanMeshTracker)