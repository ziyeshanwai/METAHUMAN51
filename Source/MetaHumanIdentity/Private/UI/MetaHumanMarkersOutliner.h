// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Widgets/SCompoundWidget.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Views/STableRow.h"
#include "EditorUndoClient.h"
#include "Framework/Commands/Commands.h"
#include "MetaHumanIdentityMarkerData.h"

DECLARE_DELEGATE_TwoParams(FFrameSelectedToggle, UMetaHumanIdentityPromotedFrame* Frame, bool bForceNotify)
DECLARE_DELEGATE_TwoParams(FFrameActiveToggle, UMetaHumanIdentityPromotedFrame* Frame, bool bActive)

DECLARE_DELEGATE_TwoParams(FGroupSelectedToggle, FString GroupID, bool bSelected)
DECLARE_DELEGATE_TwoParams(FGroupVisibilityToggle, FString GroupID, bool bVisibility)
DECLARE_DELEGATE_TwoParams(FGroupActiveToggle, FString GroupID, bool bActive)

DECLARE_DELEGATE(FCurveAndGroupStateChange)
DECLARE_DELEGATE(FCurveSelectedToggle)
DECLARE_DELEGATE_TwoParams(FCurveVisibilityToggle, FString CurveID, bool bVisibility)
DECLARE_DELEGATE_TwoParams(FCurveActiveToggle, FString CurveID, bool bActive)

class FMenuBarBuilder;
class FUICommandList;
class UMetaHumanIdentityPromotedFrame;
class UMetaHumanIdentityPose;

struct FMarkerGroupState;
struct FMarkerCurveState;

namespace MarkerOutlinerFrameView
{
	/** IDs for list columns */
	static const FName ColumnID_FrameName("Group_Name");
	static const FName ColumnID_FrameActive("Frame_Active");
}

namespace MarkerOutlinerGroupsView
{
	/** IDs for list columns */
	static const FName ColumnID_GroupName("Group_Name");
	static const FName ColumnID_GroupVisible("Group_Visible");
	static const FName ColumnID_GroupActive("Group_Active");
}

namespace MarkerOutlinerCurvesView
{
	/** IDs for list columns */
	static const FName ColumnID_CurveName("Curve_Name");
	static const FName ColumnID_CurveVisible("Curve_Visible");
	static const FName ColumnID_CurveActive("Curve_Active");
}

namespace MarkerOutlinerLandmarksView
{
	/** IDs for list columns */
	static const FName ColumnID_LandmarkName("Landmark_Name");
	static const FName ColumnID_LandmarkVisible("Landmark_Visible");
}

class FMetaHumanMarkerOutlinerCommands
	: public TCommands<FMetaHumanMarkerOutlinerCommands>
{
public:

	FMetaHumanMarkerOutlinerCommands();

	// TCommands<> interface
	virtual void RegisterCommands() override;
};

class SMarkerOutlinerFrameStructListRow : public SMultiColumnTableRow<TSharedPtr<FMarkerOutlinerFrameData>>
{
public:

	SLATE_BEGIN_ARGS(SMarkerOutlinerFrameStructListRow) {}
	SLATE_ARGUMENT(TSharedPtr<FMarkerOutlinerFrameData>, Item)
		// Delegate called when frame is enabled/disabled for solve
		SLATE_EVENT(FFrameActiveToggle, OnFrameActiveToggle)
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs, const TSharedRef<STableViewBase>& InOwnerTableView)
	{
		Item = InArgs._Item;
		SMultiColumnTableRow<TSharedPtr<FMarkerOutlinerFrameData> >::Construct(FSuperRowType::FArguments(), InOwnerTableView);
		OnFrameActiveToggleDelegate = InArgs._OnFrameActiveToggle;
	}

	/**
	 * Create a sub-widget for marker group row depending on the column name.
	 **/
	virtual TSharedRef<SWidget> GenerateWidgetForColumn(const FName& ColumnName) override;

	TSharedPtr<SCheckBox> GetActiveCheckBox() { return ActiveCheckBox; }

protected:
	TSharedPtr<FMarkerOutlinerFrameData> Item;

	FFrameActiveToggle OnFrameActiveToggleDelegate;

	void OnFrameActiveStateChanged(ECheckBoxState NewState);
	TSharedPtr<SCheckBox> ActiveCheckBox;
};


class SMarkerOutlinerGroupStructListRow : public SMultiColumnTableRow<TSharedPtr<FMarkerOutlinerGroupData>>
{
public:

	SLATE_BEGIN_ARGS(SMarkerOutlinerGroupStructListRow) {}
		SLATE_ARGUMENT(TSharedPtr<FMarkerOutlinerGroupData>, Item)
		/** Delegate called when a group visibility is toggled */
		SLATE_EVENT(FGroupVisibilityToggle, OnGroupVisibleToggle)
		/** Delegate called when a group is enabled / disabled for solve */
		SLATE_EVENT(FGroupActiveToggle, OnGroupActiveToggle)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, const TSharedRef<STableViewBase>& InOwnerTableView)
	{
		Item = InArgs._Item;
		SMultiColumnTableRow<TSharedPtr<FMarkerOutlinerGroupData> >::Construct(FSuperRowType::FArguments(), InOwnerTableView);
		OnGroupVisibilityToggleDelegate = InArgs._OnGroupVisibleToggle;
		OnGroupActiveToggleDelegate = InArgs._OnGroupActiveToggle;
	}

	/**
	 * Create a sub-widget for a row depending on the column name.
	 **/
	virtual TSharedRef<SWidget> GenerateWidgetForColumn(const FName& ColumnName) override;

	TSharedPtr<SCheckBox> GetVisibilityCheckBox() { return VisibilityCheckBox; }
	TSharedPtr<SCheckBox> GetActiveCheckBox() { return ActiveCheckBox; }

protected:
	TSharedPtr<FMarkerOutlinerGroupData> Item;
	TSharedPtr<SCheckBox> VisibilityCheckBox;
	TSharedPtr<SCheckBox> ActiveCheckBox;

	FGroupVisibilityToggle OnGroupVisibilityToggleDelegate;
	FGroupActiveToggle OnGroupActiveToggleDelegate;


	void OnGroupVisibilityStateChanged(ECheckBoxState NewState);
	void OnGroupActiveStateChanged(ECheckBoxState NewState);
};

class SMarkerOutlinerCurveStructListRow : public SMultiColumnTableRow<TSharedPtr<FMarkerOutlinerCurveData>>
{
public:

	SLATE_BEGIN_ARGS(SMarkerOutlinerCurveStructListRow) {}
		SLATE_ARGUMENT(TSharedPtr<FMarkerOutlinerCurveData>, Item)
		/** Delegate called when curve visibility is toggled */
		SLATE_EVENT(FCurveVisibilityToggle, OnCurveVisibilityToggle)
		/** Delegate called when a curve is activated / deactivated */
		SLATE_EVENT(FCurveActiveToggle, OnCurveActiveToggle)
	SLATE_END_ARGS()

		void Construct(const FArguments& InArgs, const TSharedRef<STableViewBase>& InOwnerTableView)
	{
		Item = InArgs._Item;
		SMultiColumnTableRow<TSharedPtr<FMarkerOutlinerCurveData> >::Construct(FSuperRowType::FArguments(), InOwnerTableView);
		OnCurveVisibilityToggleDelegate = InArgs._OnCurveVisibilityToggle;
		OnCurveActiveToggleDelegate = InArgs._OnCurveActiveToggle;
	}

	/**
	* Create a sub-widget for marker curve row depending on the column name.
	**/
	virtual TSharedRef<SWidget> GenerateWidgetForColumn(const FName& ColumnName) override;

	TSharedPtr<SCheckBox> GetVisibilityCheckBox() { return VisibilityCheckBox; }
	TSharedPtr<SCheckBox> GetActiveCheckBox() { return ActiveCheckBox; }

protected:
	TSharedPtr<FMarkerOutlinerCurveData> Item;

	TSharedPtr<SCheckBox> VisibilityCheckBox;
	TSharedPtr<SCheckBox> ActiveCheckBox;

	FCurveVisibilityToggle OnCurveVisibilityToggleDelegate;
	FCurveActiveToggle OnCurveActiveToggleDelegate;

	void OnCurveVisibilityStateChanged(ECheckBoxState NewState);
	void OnCurveActiveStateChanged(ECheckBoxState NewState);
};


class SMetaHumanMarkersOutliner
	: public SCompoundWidget
	, public FSelfRegisteringEditorUndoClient
{
public:
	SLATE_BEGIN_ARGS(SMetaHumanMarkersOutliner) {}

	/** The Identity we are editing */
	SLATE_ARGUMENT(class UMetaHumanIdentity*, Identity)

	/** Pose we are currently editing */
	SLATE_ARGUMENT(UMetaHumanIdentityPose*, IdentityPose)

	/** Data needed to populate the list views */
	SLATE_ARGUMENT(FMarkerData*, MarkerData)

	/** A reference to the viewport client where scene components can be displayed */
	SLATE_ARGUMENT(TSharedPtr<class FEditorViewportClient>, ViewportClient)

	/** The command list with mapped actions that can be executed by this editor */
	SLATE_ARGUMENT(TSharedPtr<FUICommandList>, CommandList)
	
	/** Delegate called when frame is selected / deselected */
	SLATE_EVENT(FFrameSelectedToggle, OnFrameSelected)

	/** Delegate called when frame is enabled / disabled for solve */
	SLATE_EVENT(FFrameActiveToggle, OnFrameActiveToggle)

	/** Delegate called when either a curve or group state has changed */
	SLATE_EVENT(FCurveAndGroupStateChange, OnCurveAndGroupStateChange)

	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	~SMetaHumanMarkersOutliner();

	/** FEditorUndoClient Interface */
	virtual void PostUndo(bool bInSuccess) override;
	virtual void PostRedo(bool bInSuccess) override;

	/** SelectionChanged events for ListView(given) */
	void OnFrameSelectionChanged(TSharedPtr<FMarkerOutlinerFrameData> InFrameItem, ESelectInfo::Type SelectInfoType);
	void OnGroupSelectionChanged(TSharedPtr<FMarkerOutlinerGroupData> InGroupItem, ESelectInfo::Type SelectInfoType);
	void OnCurveSelectionChanged(TSharedPtr<FMarkerOutlinerCurveData> InCurveItem, ESelectInfo::Type SelectInfoType);

	/**  Passed from Outliner to StructListRows as arguments on their construction, so the Outliner can react to UI events handled inside them */
	void HandleFrameActiveToggle(UMetaHumanIdentityPromotedFrame* Frame, bool Active);
	void HandleGroupVisiblityToggle(FString GroupID, bool Visibility);
	void HandleGroupActiveToggle(FString GroupID, bool Active);
	void HandleCurveVisiblityToggle(FString CurveID, bool Visibility);
	void HandleCurveActiveToggle(FString CurveID, bool Active);

	TArray<FString> GetSelectedCurvesList() const;

	TSharedPtr<SListView<TSharedPtr<FMarkerOutlinerGroupData>>> GroupListView;
	TSharedPtr<SListView<TSharedPtr<FMarkerOutlinerFrameData>>> FrameListView;
	TSharedPtr<SListView<TSharedPtr<FMarkerOutlinerCurveData>>> CurveListView;

private:

	FFrameSelectedToggle OnFrameSelectedDelegate;
	FFrameActiveToggle OnFrameActiveToggleDelegate;
	FCurveAndGroupStateChange OnCurveAndGroupStateChange;

	/** Handles an undo/redo transaction */
	void HandleUndoOrRedoTransaction(const class FTransaction* InTransaction);

	/** Creates context menu for the Outliner **/
	TSharedPtr<SWidget> BuildContextMenu(TSharedPtr<FUICommandList> CommandList);

	/** Reference to the Identity object we are editing */
	TWeakObjectPtr<class UMetaHumanIdentity> IdentityPtr;

	/** The viewport client of the preview scene */
	TSharedPtr<class FEditorViewportClient> ViewportClient;

	TSharedRef<ITableRow> OnGenerateRowForGroupsList(TSharedPtr<FMarkerOutlinerGroupData> Item, const TSharedRef<STableViewBase>& OwnerTable);
	TSharedRef<ITableRow> OnGenerateRowForFramesList(TSharedPtr<FMarkerOutlinerFrameData> Item, const TSharedRef<STableViewBase>& OwnerTable);
	TSharedRef<ITableRow> OnGenerateRowForCurvesList(TSharedPtr<FMarkerOutlinerCurveData> Item, const TSharedRef<STableViewBase>& OwnerTable);
	
	TSharedPtr<class SWidget> OutlinerContextMenu;

	/** The command list with actions associated with this editor */
	TSharedPtr<FUICommandList> OutlinerCommands;

	TArray<TSharedPtr<FMarkerOutlinerGroupData>> GetGroupsByFrames(TArray<TSharedPtr<FMarkerOutlinerFrameData>> FrameItems);
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> GetCurvesByGroups(TArray<TSharedPtr<FMarkerOutlinerGroupData>> GroupItems);

	void ToggleFramesActive(TArray<TSharedPtr<FMarkerOutlinerFrameData>> FrameItems, bool bActive);

	void ToggleGroupsVisibility(TArray<TSharedPtr<FMarkerOutlinerGroupData>> GroupItems, bool bVisibility, TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames );
	void ToggleGroupsActive(TArray<TSharedPtr<FMarkerOutlinerGroupData>> GroupItems, bool bActive, TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames);

	void ToggleCurvesVisibility(TArray<TSharedPtr<FMarkerOutlinerCurveData>> CurveItems, bool bVisibility, TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames);
	void ToggleCurvesActive(TArray<TSharedPtr<FMarkerOutlinerCurveData>> CurveItems, bool bActive, TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames);

	/** Marker data filled with the frame selected in the Promoted Frames "timeline" or the Outliner */
	FMarkerData* MarkerData; //passed to this class as the argument, can be changed by SetEditedFrame method
};