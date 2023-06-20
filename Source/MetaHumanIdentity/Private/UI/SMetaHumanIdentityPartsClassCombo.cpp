// Copyright Epic Games, Inc. All Rights Reserved.

#include "SMetaHumanIdentityPartsClassCombo.h"

#include "MetaHumanIdentityParts.h"

#include "SPositiveActionButton.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"

#define LOCTEXT_NAMESPACE "MetaHumanIdentityComponentsClassCombo"

void SMetaHumanIdentityPartsClassCombo::Construct(const FArguments& InArgs)
{
	OnIdentityPartClassSelectedDelegate = InArgs._OnIdentityPartClassSelected;
	OnIdentityPoseClassSelectedDelegate = InArgs._OnIdentityPoseClassSelected;
	OnIsIdentityPartClassEnabledDelegate = InArgs._OnIsIdentityPartClassEnabled;
	OnIsIdentityPoseClassEnabledDelegate = InArgs._OnIsIdentityPoseClassEnabled;

	ChildSlot
	[
		SNew(SPositiveActionButton)
		.Text(LOCTEXT("AddLabel", "Add"))
		.OnGetMenuContent(this, &SMetaHumanIdentityPartsClassCombo::MakeAddPartMenuWidget)
	];
}

TSharedRef<SWidget> SMetaHumanIdentityPartsClassCombo::MakeAddPartMenuWidget() const
{
	const bool bShouldCloseAfterMenuSelection = true;
	FMenuBuilder MenuBuilder{ bShouldCloseAfterMenuSelection, MakeShared<FUICommandList>() };

	MenuBuilder.BeginSection(TEXT("AddNewPart"), LOCTEXT("AddNewPartMenuSection", "Create"));
	{
		MenuBuilder.AddSubMenu(LOCTEXT("AddPart", "Add Part"),
							   LOCTEXT("AddPartTooltip", "Add a new part to this MetaHuman Identity"),
							   FNewMenuDelegate::CreateSP(this, &SMetaHumanIdentityPartsClassCombo::MakeAddPartSubMenu));

		MenuBuilder.AddSubMenu(LOCTEXT("AddPose", "Add Pose"),
							   LOCTEXT("AddPoseTooltip", "Add a new pose for this MetaHuman Identity"),
							   FNewMenuDelegate::CreateSP(this, &SMetaHumanIdentityPartsClassCombo::MakeAddPoseSubMenu));

		// TODO: Enable this when needed
		// MenuBuilder.AddSubMenu(LOCTEXT("AddPoseGroup", "Add Pose Group"),
		// 					   LOCTEXT("AddPoseGroupTooltip", "Add a new pose group for this MetaHuman Identity"),
		// 					   FNewMenuDelegate::CreateSP(this, &SMetaHumanIdentityPartsClassCombo::MakeAddPoseGroupSubMenu));
	}
	MenuBuilder.EndSection();

	return MenuBuilder.MakeWidget();
}

void SMetaHumanIdentityPartsClassCombo::MakeAddPartSubMenu(FMenuBuilder& InMenuBuilder) const
{
	InMenuBuilder.BeginSection(TEXT("AddNewPart"), LOCTEXT("AddNewPartSubmenuSection", "Create Part"));
	{
		// Get all classes that derive from UMetaHumanIdentityPart and create a menu entry
		TArray<UClass*> IdentityPartClasses;
		GetDerivedClasses(UMetaHumanIdentityPart::StaticClass(), IdentityPartClasses);

		// TODO: Remove this when more classes are allowed to be created by the editor
		static const TArray<UClass*> AllowedPartClasses = { UMetaHumanIdentityFace::StaticClass(), UMetaHumanIdentityBody::StaticClass() };

		for (UClass* IdentityPartClass : IdentityPartClasses)
		{
			if (!AllowedPartClasses.Contains(IdentityPartClass))
			{
				continue;
			}

			// Get the CDO for the Part class so we can query the Part's display name
			const UMetaHumanIdentityPart* IdentityPartCDO = GetDefault<UMetaHumanIdentityPart>(IdentityPartClass);

			const FString EntryLabel = FString::Printf(TEXT("Add %s"), *IdentityPartCDO->GetPartName().ToString());

			InMenuBuilder.AddMenuEntry(FText::FromString(EntryLabel),
									   IdentityPartCDO->GetPartDescription(),
									   FSlateIcon(),
									   FUIAction(FExecuteAction::CreateLambda([this, IdentityPartClass]
												 {
													OnIdentityPartClassSelectedDelegate.ExecuteIfBound(IdentityPartClass);
												 }),
												 FCanExecuteAction::CreateLambda([this, IdentityPartClass]
												 {
													 if (OnIsIdentityPartClassEnabledDelegate.IsBound())
													 {
														 return OnIsIdentityPartClassEnabledDelegate.Execute(IdentityPartClass);
													 }

													 return false;
												 }))
									   );
		}
	}
	InMenuBuilder.EndSection();
}

void SMetaHumanIdentityPartsClassCombo::MakeAddPoseSubMenu(FMenuBuilder& InMenuBuilder) const
{
	InMenuBuilder.BeginSection(TEXT("AddNewPose"), LOCTEXT("CreateNewPose", "Create Pose"));
	{
		// TODO: Add other poses
		InMenuBuilder.AddMenuEntry(LOCTEXT("AddNeutralLabel", "Add Neutral"),
								   LOCTEXT("AddNeutralDescription", "Add a pose with a neutral expression of the MetaHuman Identity"),
								   FSlateIcon(),
								   FUIAction(FExecuteAction::CreateLambda([this]
											{
												OnIdentityPoseClassSelectedDelegate.ExecuteIfBound(UMetaHumanIdentityPose::StaticClass(), EIdentityPoseType::Neutral);
											}),
											FCanExecuteAction::CreateLambda([this]
											{
												if (OnIsIdentityPoseClassEnabledDelegate.IsBound())
												{
													return OnIsIdentityPoseClassEnabledDelegate.Execute(UMetaHumanIdentityPose::StaticClass(), EIdentityPoseType::Neutral);
												}

												return false;
											}))
									);
	}
	InMenuBuilder.EndSection();
}

void SMetaHumanIdentityPartsClassCombo::MakeAddPoseGroupSubMenu(FMenuBuilder& InMenuBuilder) const
{
	InMenuBuilder.BeginSection(TEXT("AddNewPoseGroup"), LOCTEXT("CreateNewPoseGroup", "CreatePoseGroup"));
	{
		// TODO: Add pose group options
	}
	InMenuBuilder.EndSection();
}

#undef LOCTEXT_NAMESPACE