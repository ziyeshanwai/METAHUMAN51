// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "SViewportToolBar.h"
#include "Layout/Visibility.h"
#include "SEditorViewportViewMenuContext.h"

#include "SMetaHumanIdentityViewportToolBar.generated.h"

class SMetaHumanIdentityViewportToolBar
	: public SViewportToolBar
{
public:
	SLATE_BEGIN_ARGS(SMetaHumanIdentityViewportToolBar) {}
		SLATE_ARGUMENT(TSharedPtr<class FUICommandList>, CommandListViewA)
		SLATE_ARGUMENT(TSharedPtr<class FUICommandList>, CommandListViewB)
		SLATE_ARGUMENT(TSharedPtr<class FConformingViewportClient>, ViewportClient)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	EVisibility GetShowAVisibility() const;
	EVisibility GetShowBVisibility() const;

private:
	TSharedRef<SWidget> CreateViewAMenuWidget();
	TSharedRef<SWidget> CreateViewSelectorMenuWidget();
	TSharedRef<SWidget> CreateViewBMenuWidget();
	TSharedRef<SWidget> FillViewSelectionMenu();
	TSharedRef<SWidget> FillViewMenu(bool bInIsViewA);
	TSharedRef<SWidget> FillPreviewOptionsMenu();

	bool CanChangeFOV() const;
	TOptional<float> GetFOVValue() const;
	void HandleFOVValueChanged(float InNewValue);

	void ToggleReferenceMesh(bool bInIsViewA) const;
	void ToggleConformedMesh(bool bInIsViewA) const;
	void ToggleConformalRig(bool bInIsViewA) const;
	void ToggleCurveVisibility(bool bInIsViewA) const;
	void TogglePointVisibility(bool bInIsViewA) const;
	void EnterSingleScreenView();
	void EnterMultiScreenView();
	void EnterSplitScreenView();

	bool ReferenceMeshForViewIsChecked(bool bInIsViewA) const;
	bool ConformedMeshForViewIsChecked(bool bInIsViewA) const;
	bool ConformalRigForViewIsChecked(bool bInIsViewA) const;
	bool DisplayCurvesForViewIsChecked(bool bInIsViewA) const;
	bool DisplayPointsForViewIsChecked(bool bInIsViewA) const;
	bool MultiScreenViewIsChecked() const;
	bool SplitScreenViewIsChecked() const;
	bool SingleViewIsChecked() const;

	static const FMargin ToolbarSlotPadding;

	TSharedPtr<class SHorizontalBox> ToolbarMenuHorizontalBox;
	TSharedPtr<class FUICommandList> CommandListViewA;
	TSharedPtr<class FUICommandList> CommandListViewB;
	TSharedPtr<class FConformingViewportClient> ViewportClient;
};

class SConformingViewportViewMenu;

UCLASS()
class UConformingViewportViewMenuContext : public UEditorViewportViewMenuContext
{
	GENERATED_BODY()
public:

	TWeakPtr<const SConformingViewportViewMenu> ConformingViewportViewMenu;
};
