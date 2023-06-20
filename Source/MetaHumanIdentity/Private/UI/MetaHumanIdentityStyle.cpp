// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentityStyle.h"
#include "Styling/SlateStyleMacros.h"
#include "Styling/SlateStyleRegistry.h"
#include "Interfaces/IPluginManager.h"

FMetaHumanIdentityStyle::FMetaHumanIdentityStyle()
	: FSlateStyleSet{ TEXT("MetaHumanIdentityStyle") }
{
	const FVector2D Icon16x16(16.0f, 16.0f);
	const FVector2D Thumb128x128(128.0f, 128.0f);

	SetContentRoot(IPluginManager::Get().FindPlugin(UE_PLUGIN_NAME)->GetContentDir());

	// Register all the icons used by the MetaHuman Identity asset and editors
	Set("Identity.Tab.Parts", new IMAGE_BRUSH_SVG("Icons/IdentityPartsTab_16", Icon16x16));
	Set("Identity.Root", new IMAGE_BRUSH_SVG("Icons/IdentityRoot_16", Icon16x16));
	Set("Identity.Face.Part", new IMAGE_BRUSH_SVG("Icons/IdentityFacePart_16", Icon16x16));
	Set("Identity.Face.ConformalMesh", new IMAGE_BRUSH_SVG("Icons/IdentityFaceConformalMesh_16", Icon16x16));
	Set("Identity.Face.Rig", new IMAGE_BRUSH_SVG("Icons/IdentityFaceRig_16", Icon16x16));
	Set("Identity.Body.Part", new IMAGE_BRUSH_SVG("Icons/IdentityBodyPart_16", Icon16x16));
	Set("Identity.Pose.Neutral", new IMAGE_BRUSH_SVG("Icons/IdentityPoseNeutral_16", Icon16x16));
	Set("Identity.Pose.Teeth", new IMAGE_BRUSH_SVG("Icons/IdentityPoseTeeth_16", Icon16x16));
	Set("Identity.Pose.Custom", new IMAGE_BRUSH_SVG("Icons/IdentityPoseCustom_16", Icon16x16));
	Set("Identity.Frame.FreeRoam", new IMAGE_BRUSH_SVG("Icons/IdentityFreeRoamMode_16", Icon16x16));
	Set("Identity.Tools.ComponentsFromMesh", new IMAGE_BRUSH_SVG("Icons/IdentityComponentsFromMesh_16", Icon16x16));

	// Icons for Commands. They are automatically associated with a command
	Set("MetaHuman Identity.SubmitToAutoRig", new IMAGE_BRUSH_SVG("Icons/IdentitySubmitToAutorig_16", Icon16x16));
	Set("MetaHuman Identity.IdentitySolve", new IMAGE_BRUSH_SVG("Icons/IdentitySolve_16", Icon16x16));
	Set("MetaHuman Identity.PromoteFrame", new IMAGE_BRUSH_SVG("Icons/IdentityAddPromotedFrame_16", Icon16x16));
	Set("MetaHuman Identity.DemoteFrame", new IMAGE_BRUSH_SVG("Icons/IdentityRemovePromotedFrame_16", Icon16x16));
	Set("MetaHuman Identity.TrackCurrent", new IMAGE_BRUSH_SVG("Icons/IdentityTrackCurrentFrame_16", Icon16x16));

	// Register the thumbnails used in the Identity asset's body selection UI
	// IdentityBody_xyz: x=height, y=BMI, z=gender

	//Height: 0
	Set("Identity.Body.000", new IMAGE_BRUSH("Icons/IdentityBody_000", Thumb128x128));
	Set("Identity.Body.001", new IMAGE_BRUSH("Icons/IdentityBody_001", Thumb128x128));
	Set("Identity.Body.010", new IMAGE_BRUSH("Icons/IdentityBody_010", Thumb128x128));
	Set("Identity.Body.011", new IMAGE_BRUSH("Icons/IdentityBody_011", Thumb128x128));
	Set("Identity.Body.020", new IMAGE_BRUSH("Icons/IdentityBody_020", Thumb128x128));
	Set("Identity.Body.021", new IMAGE_BRUSH("Icons/IdentityBody_021", Thumb128x128));

	//Height: 1
	Set("Identity.Body.100", new IMAGE_BRUSH("Icons/IdentityBody_100", Thumb128x128));
	Set("Identity.Body.101", new IMAGE_BRUSH("Icons/IdentityBody_101", Thumb128x128));
	Set("Identity.Body.110", new IMAGE_BRUSH("Icons/IdentityBody_110", Thumb128x128));
	Set("Identity.Body.111", new IMAGE_BRUSH("Icons/IdentityBody_111", Thumb128x128));
	Set("Identity.Body.120", new IMAGE_BRUSH("Icons/IdentityBody_120", Thumb128x128));
	Set("Identity.Body.121", new IMAGE_BRUSH("Icons/IdentityBody_121", Thumb128x128));
	
	//Height: 2
	Set("Identity.Body.200", new IMAGE_BRUSH("Icons/IdentityBody_200", Thumb128x128));
	Set("Identity.Body.201", new IMAGE_BRUSH("Icons/IdentityBody_201", Thumb128x128));
	Set("Identity.Body.210", new IMAGE_BRUSH("Icons/IdentityBody_210", Thumb128x128));
	Set("Identity.Body.211", new IMAGE_BRUSH("Icons/IdentityBody_211", Thumb128x128));
	Set("Identity.Body.220", new IMAGE_BRUSH("Icons/IdentityBody_220", Thumb128x128));
	Set("Identity.Body.221", new IMAGE_BRUSH("Icons/IdentityBody_221", Thumb128x128));

	Set("Identity.ABSplit.A.Small", new IMAGE_BRUSH_SVG("Icons/IdentityABSplit_A_Small_16", Icon16x16));
	Set("Identity.ABSplit.A.Large", new IMAGE_BRUSH_SVG("Icons/IdentityABSplit_A_Large_16", Icon16x16));
	Set("Identity.ABSplit.B.Small", new IMAGE_BRUSH_SVG("Icons/IdentityABSplit_B_Small_16", Icon16x16));
	Set("Identity.ABSplit.B.Large", new IMAGE_BRUSH_SVG("Icons/IdentityABSplit_B_Large_16", Icon16x16));
}

void FMetaHumanIdentityStyle::Register()
{
	FSlateStyleRegistry::RegisterSlateStyle(Get());
}

void FMetaHumanIdentityStyle::Unregister()
{
	FSlateStyleRegistry::UnRegisterSlateStyle(Get());
}

FMetaHumanIdentityStyle& FMetaHumanIdentityStyle::Get()
{
	static FMetaHumanIdentityStyle Inst;
	return Inst;
}