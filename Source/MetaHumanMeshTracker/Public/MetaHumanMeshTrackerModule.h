// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FMetaHumanMeshTrackerModule
	: public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	static inline FMetaHumanMeshTrackerModule& GetModule()
	{
		static const FName ModuleName = TEXT("MetaHumanMeshTracker");
		return FModuleManager::LoadModuleChecked<FMetaHumanMeshTrackerModule>(ModuleName);
	}

	DECLARE_MULTICAST_DELEGATE_TwoParams(FOnLogError, ELogVerbosity::Type InLogVerbosity, const FString& InMessage)
	FOnLogError& OnLogError()
	{
		return OnLogErrorDelegate;
	}

private:
	/** Delegate called when a log message is generated from the internal mesh tracking libraries */
	FOnLogError OnLogErrorDelegate;

	TUniquePtr<class FMemoryResource> MemoryResource = nullptr;
};
