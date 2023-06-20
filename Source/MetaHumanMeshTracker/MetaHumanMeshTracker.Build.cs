// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class MetaHumanMeshTracker : ModuleRules
{
	public MetaHumanMeshTracker(ReadOnlyTargetRules Target) : base(Target)
	{
		bEnforceIWYU = false;
		bEnableUndefinedIdentifierWarnings = false;
		ShadowVariableWarningLevel = WarningLevel.Warning;
		PCHUsage = PCHUsageMode.NoPCHs;
		bUseUnity = false;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core"
		});

		PrivateDependencyModuleNames.AddRange(new string[] {
			"Vulkan",
			"Eigen",
			"RigLogicLib",
			"RigLogicModule"
		});


		PrivateDependencyModuleNames.AddRange(new string[] {
			"MetaHumanCore",
			"MetaHumanFrameData",
		});

		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true

		PrivateIncludePaths.AddRange(new string[]
		{
			Path.Combine(ModuleDirectory, "../ThirdParty/dlib/Include"),
			Path.Combine(ModuleDirectory, "Private/carbon/include"),
			Path.Combine(ModuleDirectory, "Private/nls/include"),
			Path.Combine(ModuleDirectory, "Private/nrr/include"),
			Path.Combine(ModuleDirectory, "Private/reconstruction/include"),
			Path.Combine(ModuleDirectory, "Private/tracking/include"),
			Path.Combine(ModuleDirectory, "Private/vulkantools/include"),
			// Path.Combine(ModuleDirectory, "Private/optflow/include"), TODO comment out until TPS issue is fixed
			Path.Combine(ModuleDirectory, "Private/conformer/include"),
			Path.Combine(ModuleDirectory, "Private/ThirdParty/dlib_modifications"),
			Path.Combine(ModuleDirectory, "Private/posebasedsolver/include"),
			Path.Combine(ModuleDirectory, "Private/api")
		});

		// cpp17 for tracking code
		CppStandard = CppStandardVersion.Cpp17;
		PrivateDefinitions.Add("TITAN_DYNAMIC_API");
		PrivateDefinitions.Add("LOG_INTEGRATION");

		if (Target.Platform == UnrealTargetPlatform.Win64 ||
			Target.Platform == UnrealTargetPlatform.Linux ||
			Target.Platform == UnrealTargetPlatform.Mac)
		{
			PrivateDefinitions.Add("CARBON_ENABLE_SSE=1");
		}

		// This module uses exceptions in the core tech libs, so they must be enabled here
		bEnableExceptions = true;

		PublicAdditionalLibraries.AddRange(new string[]
		{
			"$(ModuleDir)/../ThirdParty/dlib/Lib/Win64/Release/dlib19.21.99_release_64bit_msvc1927.lib"
		});
		// Need to disable unity builds for this module to avoid clang compilation issues with the TEXT macro
		//bUseUnity = false;
	}
}
