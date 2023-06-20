// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class RLibV : ModuleRules
{
	public RLibV(ReadOnlyTargetRules Target) : base(Target)
	{
		bEnforceIWYU = false;
		bEnableUndefinedIdentifierWarnings = false;
		ShadowVariableWarningLevel = WarningLevel.Warning;
		PCHUsage = PCHUsageMode.NoPCHs;
		bUseUnity = false;

		CppStandard = CppStandardVersion.Cpp17;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"MetaHumanFrameData",
			"CoreUObject",
			"GeometryCore"
		});

		PrivateIncludePaths.AddRange(new string[] {
			"ThirdParty/dlib/Include",
			"RLibV/Private/src",
			"RLibV/Private/include",
		});

		PublicAdditionalLibraries.AddRange(new string[] {
			"$(ModuleDir)/../ThirdParty/dlib/Lib/Win64/Release/dlib19.21.99_release_64bit_msvc1927.lib",
			"$(ModuleDir)/../ThirdParty/OpenBLAS/Lib/Win64/Release/openblas.lib"
		});
	}
}
