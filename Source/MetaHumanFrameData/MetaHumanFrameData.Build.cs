// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class MetaHumanFrameData : ModuleRules
{
	public MetaHumanFrameData(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"LiveLinkInterface",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"CoreUObject",
		});
	}
}
