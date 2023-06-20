// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class MetaHumanImageViewer : ModuleRules
{
	public MetaHumanImageViewer(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"EditorStyle"
		});

		PrivateDependencyModuleNames.AddRange(new string[] {
			"CoreUObject",
			"Engine",
			"Slate",
			"SlateCore",
			"InputCore",
			"RLibV",
			"UnrealEd",
			"MetaHumanFrameData",
			"MetaHumanCore",
			"EditorStyle"
		});
	}
}