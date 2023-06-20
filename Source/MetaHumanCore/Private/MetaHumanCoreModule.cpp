// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanCoreModule.h"
#include "MetaHumanEditorSettings.h"

#include "Integration.h"
#include "Misc/OutputDeviceRedirector.h"
#include "FMemoryResource.h"
#include "HAL/Platform.h"
#include "Misc/Paths.h"
#include "ISettingsModule.h"
#include "AssetToolsModule.h"

#include <PerPlatformProperties.h>
#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#elif PLATFORM_LINUX || PLATFORM_MAC
#include <dlfcn.h>
#endif

#define LOCTEXT_NAMESPACE "MetaHuman"

DEFINE_LOG_CATEGORY_STATIC(MetaHumanMeshTracker, Log, All);
DEFINE_LOG_CATEGORY_STATIC(DeepFlow, Log, All);
DEFINE_LOG_CATEGORY_STATIC(RLibTrecker, Log, All);
DEFINE_LOG_CATEGORY_STATIC(CoreLib, Log, All);

namespace
{
	TMap<void*, FLogCategoryName> CategoryResolver;

	using epic::core::LogLevel;

	ELogVerbosity::Type LogLevel2UELogLevel(LogLevel InLogLevel)
	{
		switch (InLogLevel)
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

	void LogFunction(epic::core::LogLevel InLogLevel, const char* InFormat, ...)
	{
#if !NO_LOGGING
#if !IS_MONOLITHIC
		void* CallingAddress = PLATFORM_RETURN_ADDRESS();
		void* modulePtr{};
#if PLATFORM_WINDOWS
		HMODULE Module;
		GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, (LPCWSTR)CallingAddress, &Module);
		modulePtr = static_cast<void*>(Module);
#elif PLATFORM_MAC || PLATFORM_LINUX
		Dl_info DylibInfo;
		int32 Result = dladdr(static_cast<const void*>(CallingAddress), &DylibInfo);
		if (Result != 0)
		{
			modulePtr = DylibInfo.dli_fbase;
		}
#endif // PLATFORM_WINDOWS

		FLogCategoryName& CategoryName = CategoryResolver[modulePtr];;
#else
		FLogCategoryName& CategoryName = CoreLib.GetCategoryName();
#endif // !IS_MONOLITHIC

		ELogVerbosity::Type UEVerbosity = LogLevel2UELogLevel(InLogLevel);
		va_list(Args);
		va_start(Args, InFormat);
		char Buffer[1024];
#ifdef PLATFORM_MICROSOFT
		vsprintf_s(Buffer, InFormat, Args);
#else
		vsprintf(Buffer, InFormat, Args);
#endif
		va_end(Args);

		FString MessageW(Buffer);
		GLog->Serialize(*MessageW, UEVerbosity, CategoryName);
#endif
	}
}

class FMetaHumanCoreModule
	: public IMetaHumanCoreModule
{
public:

	//~ IModuleInterface interface
	virtual void StartupModule() override
	{
		void* MeshTrackerHandle = FPlatformProcess::GetDllHandle(*FModuleManager::Get().GetModuleFilename("MetaHumanMeshTracker"));

		CategoryResolver.Add(MeshTrackerHandle, MetaHumanMeshTracker.GetCategoryName());
		CategoryResolver.Add(nullptr, CoreLib.GetCategoryName());
		MemoryResource = MakeUnique<FMemoryResource>();
		epic::core::GetIntegrationParams() = { epic::core::Logger(LogFunction), MemoryResource.Get() };

		if (ISettingsModule* SettingsModule = FModuleManager::GetModulePtr<ISettingsModule>("Settings"))
		{
			SettingsModule->RegisterSettings("Editor", "Plugins", "MetaHuman_Settings",
											 LOCTEXT("SettingsName", "MetaHuman"), LOCTEXT("SettingsDescription", "Configure MetaHuman settings"),
											 GetMutableDefault<UMetaHumanEditorSettings>());
		}

		// Register the MetaHuman asset category
		IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>(TEXT("AssetTools")).Get();
		MetaHumanAssetCategoryBit = AssetTools.RegisterAdvancedAssetCategory(TEXT("MetaHuman"), LOCTEXT("MetaHumanAssetCategoryLabel", "MetaHuman"));
	}

	virtual void ShutdownModule() override
	{
		MemoryResource.Reset();
	}

	//~ IMetaHumanCoreModule interface
	virtual EAssetTypeCategories::Type GetMetaHumanAssetCategoryBit() const
	{
		return MetaHumanAssetCategoryBit;
	}

private:
	EAssetTypeCategories::Type MetaHumanAssetCategoryBit;

	TUniquePtr<class FMemoryResource> MemoryResource;
};

IMPLEMENT_MODULE(FMetaHumanCoreModule, MetaHumanCore)

#undef LOCTEXT_NAMESPACE
