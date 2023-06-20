// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Framework/Commands/Commands.h"

class FMetaHumanIdentityEditorCommands
	: public TCommands<FMetaHumanIdentityEditorCommands>
{
public:

	FMetaHumanIdentityEditorCommands();

	// TCommands<> interface
	virtual void RegisterCommands() override;

public:
	TSharedPtr<FUICommandInfo> TrackCurrent;
	TSharedPtr<FUICommandInfo> TrackAll;

	TSharedPtr<FUICommandInfo> ActivateMarkersForCurrent;
	TSharedPtr<FUICommandInfo> ActivateMarkersForAll;

	TSharedPtr<FUICommandInfo> ResetTemplateMesh;
	TSharedPtr<FUICommandInfo> IdentitySolve;
	TSharedPtr<FUICommandInfo> SubmitToAutorig;

	TSharedPtr<FUICommandInfo> RigidFitCurrent;
	TSharedPtr<FUICommandInfo> RigidFitAll;

	TSharedPtr<FUICommandInfo> ViewToggle;
	TSharedPtr<FUICommandInfo> ToggleViewA;
	TSharedPtr<FUICommandInfo> ToggleViewB;

	TSharedPtr<FUICommandInfo> PromoteFrame;
	TSharedPtr<FUICommandInfo> DemoteFrame;

	TSharedPtr<FUICommandInfo> ToggleGroupVisibilityForCurrentFrame;
	TSharedPtr<FUICommandInfo> ToggleGroupActiveForCurrentFrame;
};