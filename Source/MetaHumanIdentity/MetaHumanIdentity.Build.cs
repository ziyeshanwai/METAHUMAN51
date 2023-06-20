// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class MetaHumanIdentity : ModuleRules
{
	public MetaHumanIdentity(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PrivateDefinitions.AddRange(new string[]
		{
			// We depend on OpenCVHelpers but don't actually use functions that depend on OpenCV itself so
			// this needs to be defined as OpenCVHelper depends on this definition to compile
			"WITH_OPENCV=0"
		});

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"Json",
			"JsonUtilities"
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"CoreUObject",
			"Engine",
			"UnrealEd",
			"Projects",
			"Slate",
			"SlateCore",
			"EditorStyle",
			"RenderCore",
			"ImgMedia",
			"InputCore",
			"ToolMenus",
			"ToolWidgets",
			"GeometryFramework",
			"NeuralNetworkInference",
			"AdvancedPreviewScene",
			"GeometryCore",
			"RigLogicModule",
			"MeshConversion",
			"MeshDescription",
			"StaticMeshDescription",
			"OpenCVHelper",
			"RigLogicModule",
			"AutoRigService",
			"RLibV",
			"MetaHumanMeshTracker",
			"MetaHumanFaceContourTracker",
			"MetaHumanImageViewer",
			"MetaHumanPipeline",
			"MetaHumanFrameData"
		});
	}
}