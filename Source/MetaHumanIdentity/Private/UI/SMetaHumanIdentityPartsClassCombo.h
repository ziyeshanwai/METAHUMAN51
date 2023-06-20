// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Templates/SubclassOf.h"

enum class EIdentityPoseType : uint8;

DECLARE_DELEGATE_OneParam(FIdentityPartClassSelected, TSubclassOf<class UMetaHumanIdentityPart> InPartClass)
DECLARE_DELEGATE_TwoParams(FIdentityPoseClassSelected, TSubclassOf<class UMetaHumanIdentityPose> InPoseClass, EIdentityPoseType InPoseType)
DECLARE_DELEGATE_RetVal_OneParam(bool, FIsIdentityPartClassEnabled, TSubclassOf<class UMetaHumanIdentityPart> InPartClass)
DECLARE_DELEGATE_RetVal_TwoParams(bool, FIsIdentityPoseClassEnabled, TSubclassOf<class UMetaHumanIdentityPose> InPoseClass, EIdentityPoseType InPoseType)

/** A combo button that shows Identity Parts and Poses and emits the appropriate event when one is selected selected */
class SMetaHumanIdentityPartsClassCombo
	: public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SMetaHumanIdentityPartsClassCombo) {}
		SLATE_EVENT(FIdentityPartClassSelected, OnIdentityPartClassSelected)
		SLATE_EVENT(FIdentityPoseClassSelected, OnIdentityPoseClassSelected)
		SLATE_EVENT(FIsIdentityPartClassEnabled, OnIsIdentityPartClassEnabled)
		SLATE_EVENT(FIsIdentityPoseClassEnabled, OnIsIdentityPoseClassEnabled)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	/** Creates the widget that will be displayed as the combo menu */
	TSharedRef<SWidget> MakeAddPartMenuWidget() const;

	void MakeAddPartSubMenu(class FMenuBuilder& InMenuBuilder) const;
	void MakeAddPoseSubMenu(class FMenuBuilder& InMenuBuilder) const;
	void MakeAddPoseGroupSubMenu(class FMenuBuilder& InMenuBuilder) const;

	/** Delegate called when a Identity Part class is selected from the combo menu */
	FIdentityPartClassSelected OnIdentityPartClassSelectedDelegate;

	/** Delegate called when a Identity Pose class is selected from the combo menu */
	FIdentityPoseClassSelected OnIdentityPoseClassSelectedDelegate;

	/** Delegate called to query if a Part class should be enabled in the menu */
	FIsIdentityPartClassEnabled OnIsIdentityPartClassEnabledDelegate;

	/** Delegate called to query if a Pose class should be enabled in the menu */
	FIsIdentityPoseClassEnabled OnIsIdentityPoseClassEnabledDelegate;
};