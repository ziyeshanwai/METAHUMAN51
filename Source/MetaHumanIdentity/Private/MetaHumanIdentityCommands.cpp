// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentityCommands.h"
#include "UI/MetaHumanIdentityStyle.h"

#define LOCTEXT_NAMESPACE "FMetaHumanIdentityEditorCommands"

FMetaHumanIdentityEditorCommands::FMetaHumanIdentityEditorCommands()
	: TCommands<FMetaHumanIdentityEditorCommands>(TEXT("MetaHuman Identity"),
										  NSLOCTEXT("Contexts", "FMetaHumanIdentityModule", "MetaHuman Identity Asset Editor"),
										  NAME_None,
										  FMetaHumanIdentityStyle::Get().GetStyleSetName())
{}

void FMetaHumanIdentityEditorCommands::RegisterCommands()
{
	UI_COMMAND(TrackCurrent, "Track Markers (Current Frame)", "Track the selected frame", EUserInterfaceActionType::Button, FInputChord{});

	UI_COMMAND(ActivateMarkersForCurrent, "Activate Markers (Current Frame)", "Activate the display of markers for the selected frame", EUserInterfaceActionType::Button, FInputChord{});
	UI_COMMAND(ActivateMarkersForAll, "Activate Markers (All Frames)", "Activate the display of markers for all promoted frames", EUserInterfaceActionType::Button, FInputChord{});

	UI_COMMAND(ResetTemplateMesh, "Reset Template Mesh", "Resets the Template Mesh", EUserInterfaceActionType::Button, FInputChord{});
	UI_COMMAND(IdentitySolve, "MetaHuman Identity Solve", "MetaHuman Identity Solve", EUserInterfaceActionType::Button, FInputChord{});
	UI_COMMAND(SubmitToAutorig, "Mesh to MetaHuman", "Submit the template mesh for MetaHuman conversion, will become available in MetaHuman Creator and Bridge", EUserInterfaceActionType::Button, FInputChord{});

	UI_COMMAND(RigidFitCurrent, "Rigid Fit (Current Frame)", "Rigid fit the head of the current frame", EUserInterfaceActionType::Button, FInputChord{});
	UI_COMMAND(RigidFitAll, "Rigid Fit (All Frames)", "Rigid fit the head in all frames", EUserInterfaceActionType::Button, FInputChord{});

	UI_COMMAND(ViewToggle, "<->", "Swap the contents of A and B views", EUserInterfaceActionType::Button, FInputChord{});
	UI_COMMAND(ToggleViewA, "A", "Toggle View A", EUserInterfaceActionType::ToggleButton, FInputChord{});
	UI_COMMAND(ToggleViewB, "B", "Toggle View B", EUserInterfaceActionType::ToggleButton, FInputChord{});

	UI_COMMAND(PromoteFrame, "Promote Frame", "Promote a Frame", EUserInterfaceActionType::Button, FInputChord{});
	UI_COMMAND(DemoteFrame, "Demote Frame", "Demote a Frame", EUserInterfaceActionType::Button, FInputChord{});

	UI_COMMAND(ToggleGroupVisibilityForCurrentFrame, "Toggle Group Visibility", "Toggle Visibility For Current Frame", EUserInterfaceActionType::Check, FInputChord{});
	UI_COMMAND(ToggleGroupActiveForCurrentFrame, "Toggle Group Active", "Toggle Active For Current Frame", EUserInterfaceActionType::Check, FInputChord{});
}

#undef LOCTEXT_NAMESPACE