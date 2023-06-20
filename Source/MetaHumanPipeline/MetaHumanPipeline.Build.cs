// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class MetaHumanPipeline : ModuleRules
{
	public MetaHumanPipeline(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"Eigen",
			"MetaHumanCore",
		});

		PrivateDependencyModuleNames.AddRange(new string[] {
			"CoreUObject",
			"ModelingOperators",
			"NeuralNetworkInference",
			"Engine",
			"Json",
			"RLibV",
			"MetaHumanMeshTracker",
			"MetaHumanFrameData",
			"GeometryCore",
			"Projects",
		});
	}
}
