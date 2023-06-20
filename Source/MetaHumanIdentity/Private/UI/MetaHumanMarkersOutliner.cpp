// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanMarkersOutliner.h"
#include "MetaHumanIdentity.h"
#include "MetaHumanIdentityParts.h"
#include "MetaHumanIdentityAssetEditorToolkit.h"

#include "Editor.h"

#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "Widgets/Layout/SScrollBox.h"
#include "ScopedTransaction.h"
#include "Editor/Transactor.h"

#define LOCTEXT_NAMESPACE "MetaHumanMarkersOutliner"

void SMetaHumanMarkersOutliner::Construct(const FArguments& InArgs)
{
	check(InArgs._Identity);

	OnFrameSelectedDelegate = InArgs._OnFrameSelected; //no "toggle" because a frame can only be selected
	OnFrameActiveToggleDelegate = InArgs._OnFrameActiveToggle;
	
	OnCurveAndGroupStateChange = InArgs._OnCurveAndGroupStateChange;

	IdentityPtr = InArgs._Identity;
	MarkerData = InArgs._MarkerData; //just an abbreviation

	ViewportClient = InArgs._ViewportClient;
	OutlinerCommands  = InArgs._CommandList;

	OutlinerContextMenu = BuildContextMenu(OutlinerCommands);

	TSharedRef< SHeaderRow > FramesHeaderRowWidget =
		SNew(SHeaderRow)

		.Visibility(EVisibility::All)

		/** Name label column */
		+ SHeaderRow::Column(MarkerOutlinerFrameView::ColumnID_FrameName)
		.DefaultLabel(LOCTEXT("FrameColumn_NameLabel", "Frames"))
		.FillWidth(0.6)

		/** Active column */
		+ SHeaderRow::Column(MarkerOutlinerFrameView::ColumnID_FrameActive)
		.DefaultLabel(LOCTEXT("FrameColumn_Active", "Active"))
		.FillWidth(0.4);
	
	TSharedRef< SHeaderRow > GroupsHeaderRowWidget =
		SNew(SHeaderRow)

		.Visibility(EVisibility::All)

		/** Name label column */
		+ SHeaderRow::Column(MarkerOutlinerGroupsView::ColumnID_GroupName)
		.DefaultLabel(LOCTEXT("GroupColumn_NameLabel", "Group"))
		.FillWidth(0.4f)

		/** Visibility column */
		+ SHeaderRow::Column(MarkerOutlinerGroupsView::ColumnID_GroupVisible)
		.DefaultLabel(LOCTEXT("GroupColumn_Visible", "Visible"))
		.FillWidth(0.2f)

		/** Active column */
		+ SHeaderRow::Column(MarkerOutlinerGroupsView::ColumnID_GroupActive)
		.DefaultLabel(LOCTEXT("GroupColumn_Active", "Active"))
		.FillWidth(0.2f);

	TSharedRef< SHeaderRow > CurvesHeaderRowWidget =
		SNew(SHeaderRow)

		.Visibility(EVisibility::All)

		+ SHeaderRow::Column(MarkerOutlinerCurvesView::ColumnID_CurveName)
		.DefaultLabel(LOCTEXT("CurveColumn_NameLabel", "Name"))
		.FillWidth(0.5)

		+ SHeaderRow::Column(MarkerOutlinerCurvesView::ColumnID_CurveVisible)
		.DefaultLabel(LOCTEXT("CurveColumn_Visible", "Visible"))
		.FillWidth(0.25f)


		+ SHeaderRow::Column(MarkerOutlinerCurvesView::ColumnID_CurveActive)
		.DefaultLabel(LOCTEXT("CurveColumn_Active", "Active"))
		.FillWidth(0.25f);


	TSharedRef< SHeaderRow > LandmarksHeaderRowWidget =
		SNew(SHeaderRow)

		.Visibility(EVisibility::All)

		+ SHeaderRow::Column(MarkerOutlinerLandmarksView::ColumnID_LandmarkName)
		.DefaultLabel(LOCTEXT("LandmarkColumn_CurveNameLabel", "Name"))
		.FillWidth(0.6f)

		+ SHeaderRow::Column(MarkerOutlinerLandmarksView::ColumnID_LandmarkVisible)
		.DefaultLabel(LOCTEXT("LandmarkColumn_Active", "Active"))
		.FillWidth(0.4f);

	ChildSlot
	[
		SNew(SSplitter)
		.Orientation(EOrientation::Orient_Vertical)
		+ SSplitter::Slot()
		[
			SAssignNew(FrameListView, SListView<TSharedPtr<FMarkerOutlinerFrameData>>)
			.SelectionMode(ESelectionMode::SingleToggle)
			.ItemHeight(24)
			.ListItemsSource(&MarkerData->MarkerFrames)
			.OnGenerateRow(this, &SMetaHumanMarkersOutliner::OnGenerateRowForFramesList)
			.ClearSelectionOnClick(false)
			.OnSelectionChanged(this, &SMetaHumanMarkersOutliner::OnFrameSelectionChanged)
			.HeaderRow(FramesHeaderRowWidget)
		]
		+ SSplitter::Slot()
		[
			SAssignNew(GroupListView, SListView<TSharedPtr<FMarkerOutlinerGroupData>>)
			.SelectionMode(ESelectionMode::Multi)
			.ItemHeight(24)
			.ListItemsSource(&MarkerData->MarkerGroups)
			.OnGenerateRow(this, &SMetaHumanMarkersOutliner::OnGenerateRowForGroupsList)
			.OnSelectionChanged(this, &SMetaHumanMarkersOutliner::OnGroupSelectionChanged)
			.HeaderRow (GroupsHeaderRowWidget)
		]
		+ SSplitter::Slot()
		[
			SAssignNew(CurveListView, SListView<TSharedPtr<FMarkerOutlinerCurveData>>)
			.SelectionMode(ESelectionMode::Multi)
			.ItemHeight(24)
			.ListItemsSource(&MarkerData->MarkerCurves)
			.OnGenerateRow(this, &SMetaHumanMarkersOutliner::OnGenerateRowForCurvesList)
			.OnSelectionChanged(this, &SMetaHumanMarkersOutliner::OnCurveSelectionChanged)
			.HeaderRow(CurvesHeaderRowWidget)
		]
	];
}

SMetaHumanMarkersOutliner::~SMetaHumanMarkersOutliner()
{
}

void SMetaHumanMarkersOutliner::PostUndo(bool bInSuccess)
{
	if (bInSuccess)
	{
		const FTransaction* Transaction = GEditor->Trans->GetTransaction(GEditor->Trans->GetQueueLength() - GEditor->Trans->GetUndoCount());
		HandleUndoOrRedoTransaction(Transaction);
	}
}

void SMetaHumanMarkersOutliner::PostRedo(bool bInSuccess)
{
	if (bInSuccess)
	{
		const FTransaction* Transaction = GEditor->Trans->GetTransaction(GEditor->Trans->GetQueueLength() - GEditor->Trans->GetUndoCount() - 1);
		HandleUndoOrRedoTransaction(Transaction);
	}
}

void SMetaHumanMarkersOutliner::HandleUndoOrRedoTransaction(const FTransaction* InTransaction)
{
	if (InTransaction && InTransaction->GetPrimaryObject() == IdentityPtr)
	{
		//TO DO
	}
}

TSharedRef<ITableRow> SMetaHumanMarkersOutliner::OnGenerateRowForGroupsList(TSharedPtr<FMarkerOutlinerGroupData> Item, const TSharedRef<STableViewBase>& OwnerTable)
{
	return
		SNew(SMarkerOutlinerGroupStructListRow, OwnerTable)
		.Item(Item)
		.OnGroupVisibleToggle(this, &SMetaHumanMarkersOutliner::HandleGroupVisiblityToggle)
		.OnGroupActiveToggle(this, &SMetaHumanMarkersOutliner::HandleGroupActiveToggle);
}

TSharedRef<ITableRow> SMetaHumanMarkersOutliner::OnGenerateRowForFramesList(TSharedPtr<FMarkerOutlinerFrameData> Item, const TSharedRef<STableViewBase>& OwnerTable)
{
	return
		SNew(SMarkerOutlinerFrameStructListRow, OwnerTable)
		.Item(Item)
		.OnFrameActiveToggle(this, &SMetaHumanMarkersOutliner::HandleFrameActiveToggle);
}

TSharedRef<ITableRow> SMetaHumanMarkersOutliner::OnGenerateRowForCurvesList(TSharedPtr<FMarkerOutlinerCurveData> Item, const TSharedRef<STableViewBase>& OwnerTable)
{
	return
		SNew(SMarkerOutlinerCurveStructListRow, OwnerTable)
		.Item(Item)
		.OnCurveVisibilityToggle(this, &SMetaHumanMarkersOutliner::HandleCurveVisiblityToggle)
		.OnCurveActiveToggle(this, &SMetaHumanMarkersOutliner::HandleCurveActiveToggle);
}

TSharedPtr<SWidget> SMetaHumanMarkersOutliner::BuildContextMenu(TSharedPtr<FUICommandList> CommandList)
{
	FMenuBuilder MenuBuilder(true, CommandList);

	//TO DO

	TSharedPtr<SWidget> MenuWidget = MenuBuilder.MakeWidget();
	return MenuWidget;

}

// the ListView event for selection change
void SMetaHumanMarkersOutliner::OnFrameSelectionChanged(TSharedPtr<FMarkerOutlinerFrameData> InFrameItem, ESelectInfo::Type SelectInfoType)
{
	if (!FrameListView->GetSelectedItems().Contains(InFrameItem))
	{
		//reset the selection in the data
		for (TSharedPtr<FMarkerOutlinerFrameData> Item : MarkerData->MarkerFrames)
		{
			Item->bSelected = Item == InFrameItem ? true : false;
		}
	}
	FrameListView->RequestListRefresh();

	//Select Groups based on selected Frames
	GroupListView->ClearSelection();
	GroupListView->SetItemSelection(GetGroupsByFrames(FrameListView->GetSelectedItems()), true);
	GroupListView->RequestListRefresh();

	//Select Curves based on selected Groups
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> SelectedCurves = GetCurvesByGroups(GroupListView->GetSelectedItems());
	CurveListView->ClearSelection();
	CurveListView->SetItemSelection(SelectedCurves, true);
	CurveListView->RequestListRefresh();

	//if this event was triggered as a side effect of clicking on Promoted Frame buttons,
	//SelectInfoType will be Direct (triggered from code); in that case, do not raise another notification,
	//as it will re-trigger equivalent event on PromotedFramesEditor again; if it was by clicking in the
	//ListView, then PromotedFramesEditor should react
	bool bNotify = SelectInfoType != ESelectInfo::Type::Direct;
	if (bNotify)
	{
		//send signal to the toolkit that the frame selection has changed, so it can refresh the MarkerData
		if (InFrameItem != nullptr)
		{
			OnFrameSelectedDelegate.ExecuteIfBound(InFrameItem->Frame, false);
		}
		else
		{
			//deselect the frame in Promoted Frames timeline too
			OnFrameSelectedDelegate.ExecuteIfBound(nullptr, false);
		}
	}
}

// the ListView event for selection change
void SMetaHumanMarkersOutliner::OnGroupSelectionChanged(TSharedPtr<FMarkerOutlinerGroupData> InGroupItem, ESelectInfo::Type SelectInfoType)
{
	//reset the selection in the data
	for (TSharedPtr<FMarkerOutlinerGroupData> GroupItem : MarkerData->MarkerGroups)
	{
		GroupItem->State.bSelected = false;
	}
	//set selection in data
	TArray<TSharedPtr<FMarkerOutlinerGroupData>> SelectedItems = GroupListView->GetSelectedItems();
	for (const TSharedPtr<FMarkerOutlinerGroupData>& SelectedItem : SelectedItems)
	{
		SelectedItem->State.bSelected = true;
	}

	GroupListView->RequestListRefresh();

	//Select Curves based on selected Groups
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> SelectedCurves = GetCurvesByGroups(GroupListView->GetSelectedItems());
	CurveListView->ClearSelection();
	CurveListView->SetItemSelection(SelectedCurves, true);
	CurveListView->RequestListRefresh();
	
	OnCurveAndGroupStateChange.ExecuteIfBound();
}

// the ListView event for selection change
void SMetaHumanMarkersOutliner::OnCurveSelectionChanged(TSharedPtr<FMarkerOutlinerCurveData> InCurveItem, ESelectInfo::Type SelectInfoType)
{
	//reset the selection in data
	for (TSharedPtr<FMarkerOutlinerCurveData> CurveItem : MarkerData->MarkerCurves)
	{
		CurveItem->State.bSelected = false;
	}
	//set selection in data
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> SelectedItems = CurveListView->GetSelectedItems();
	for (const TSharedPtr<FMarkerOutlinerCurveData>& SelectedItem : SelectedItems)
	{
		SelectedItem->State.bSelected = true;
	}
	
	OnCurveAndGroupStateChange.ExecuteIfBound();
}

TArray<TSharedPtr<FMarkerOutlinerGroupData>> SMetaHumanMarkersOutliner::GetGroupsByFrames(TArray<TSharedPtr<FMarkerOutlinerFrameData>> FrameItems)
{
	TArray<TSharedPtr<FMarkerOutlinerGroupData>> SelectedGroups;

	for (const TSharedPtr<FMarkerOutlinerFrameData>& FrameItem : FrameItems)
	{
		//go through all existing groups
		for (const TSharedPtr<FMarkerOutlinerGroupData>& GroupDataItem : MarkerData->MarkerGroups)
		{
			FString GroupName = GroupDataItem->Name;
			//fetch a state of the group in the currently iterated frame
			FMarkerGroupState& GroupState = FrameItem->Frame->GroupStates[GroupName];
			if (GroupState.bActive) //is that group active in the currently iterated frame?
			{
				SelectedGroups.Add(GroupDataItem);
			}
		}
	}

	return SelectedGroups;
}

TArray<TSharedPtr<FMarkerOutlinerCurveData>> SMetaHumanMarkersOutliner::GetCurvesByGroups(TArray<TSharedPtr<FMarkerOutlinerGroupData>> GroupItems)
{
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> SelectedCurves;

	for (const TSharedPtr<FMarkerOutlinerGroupData>& GroupItem : GroupItems)
	{
		FString GroupName = GroupItem->Name;
		for (const FMarkerCurveDef& CurveDef: MarkerData->MarkerDefs->CurveDefs)
		{
			FString CurveName = CurveDef.Name;
			if (CurveDef.GroupTagIDs.Contains(GroupName))
			{
				for (const TSharedPtr<FMarkerOutlinerCurveData>& CurveItem: MarkerData->MarkerCurves)
				{
					if (CurveItem->Name == CurveName)
					{
						SelectedCurves.Add(CurveItem);
					}
				}
			}
		}
	}

	return SelectedCurves;
}

void SMetaHumanMarkersOutliner::HandleGroupVisiblityToggle(FString GroupID, bool bVisibility)
{
	TArray<TSharedPtr<FMarkerOutlinerGroupData>> GroupsToToggle;

	bool bCheckedItemIsSelected = false;
	TArray<TSharedPtr<FMarkerOutlinerGroupData>> SelectedGroups = GroupListView->GetSelectedItems();
	for (TSharedPtr<FMarkerOutlinerGroupData>& Group : MarkerData->MarkerGroups)
	{
		if (SelectedGroups.IsEmpty() && (Group->Name == GroupID))//no selection (Active toggled on a non-selected item)?
		{
			GroupsToToggle.Add(Group);
		}
		else {
			if (Group->Name == GroupID) //this is the item checked by the user
			{
				if (SelectedGroups.Contains(Group))
				{
					bCheckedItemIsSelected = true; //this means we should toggle all selected items
				}
				GroupsToToggle.Add(Group); //but in case it is not in the selected group, add it so it will be toggled anyway
			}
		}
	}
	if (bCheckedItemIsSelected)
	{
		GroupsToToggle = SelectedGroups;
	} //else just the one toggled (out of the selection) will be toggled

	TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames;
	//for now, work on the single selected frame only, later add multi-selection to frames and pass the entire list
	ToggleGroupsVisibility(GroupsToToggle, bVisibility, AffectedFrames);
	//cascade the effect to connected objects
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> SelectedCurves = GetCurvesByGroups(GroupsToToggle);
	ToggleCurvesVisibility(SelectedCurves, bVisibility, AffectedFrames);
	
	OnCurveAndGroupStateChange.ExecuteIfBound();
}

void SMetaHumanMarkersOutliner::HandleGroupActiveToggle(FString GroupID, bool bActive)
{
	TArray<TSharedPtr<FMarkerOutlinerGroupData>> GroupsToToggle;
	bool bCheckedItemIsSelected = false;
	TArray<TSharedPtr<FMarkerOutlinerGroupData>> SelectedGroups = GroupListView->GetSelectedItems();
	for (TSharedPtr<FMarkerOutlinerGroupData>& Group : MarkerData->MarkerGroups)
	{
		if (SelectedGroups.IsEmpty() && (Group->Name == GroupID))//no selection (Active toggled on a non-selected item)?
		{
			GroupsToToggle.Add(Group);
		}
		else {
			if (Group->Name == GroupID) //this is the item checked by the user
			{
				if (SelectedGroups.Contains(Group))
				{
					bCheckedItemIsSelected = true; //this means we should toggle all selected items
				}
				GroupsToToggle.Add(Group); //but in case it is not in the selected group, add it so it will be toggled anyway
			} 
		}
	} 
	
	if (bCheckedItemIsSelected)
	{
		GroupsToToggle = SelectedGroups;
	}

	TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames;
	//for now, work on the single selected frame only, later add multi=selection to frames and pass the entire list

	ToggleGroupsActive(GroupsToToggle, bActive, AffectedFrames);
	//cascade the effect to connected objects
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> SelectedCurves = GetCurvesByGroups(GroupsToToggle);
	ToggleCurvesActive(SelectedCurves, bActive, AffectedFrames);
}

void SMetaHumanMarkersOutliner::HandleFrameActiveToggle(UMetaHumanIdentityPromotedFrame* InFrame, bool bActive)
{
	bool bCheckedItemIsSelected = false;
	for (TSharedPtr<FMarkerOutlinerFrameData>& FrameItem: FrameListView->GetSelectedItems())
	{
		if (FrameItem->Frame == InFrame) //checked item is selected?
		{
			bCheckedItemIsSelected = true;
			break;
		}
	}

	if (bCheckedItemIsSelected)
	{
		for (TSharedPtr<FMarkerOutlinerFrameData>& FrameItem : FrameListView->GetSelectedItems())
		{
			FrameItem->Frame->bUseToSolve = bActive;
		}
	}
}

void SMetaHumanMarkersOutliner::HandleCurveVisiblityToggle(FString CurveID, bool bVisibility)
{
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> CurvesToToggle;
	bool bCheckedItemIsSelected = false;
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> SelectedCurves = CurveListView->GetSelectedItems();
	for (TSharedPtr<FMarkerOutlinerCurveData>& Curve: SelectedCurves)
	{
		if (Curve->Name == CurveID) //checked item is selected?
		{
			bCheckedItemIsSelected = true;
			CurvesToToggle.Add(Curve);
			break;
		}
	}

	if (bCheckedItemIsSelected)
	{
		CurvesToToggle = SelectedCurves;
	}
	
	TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames;
	//for now, work on the single selected frame only, later add multiselection to frames and pass the entire list

	ToggleCurvesVisibility(CurvesToToggle, bVisibility, AffectedFrames);
	//cascade to corresponding points
	
	OnCurveAndGroupStateChange.ExecuteIfBound();
}

void SMetaHumanMarkersOutliner::HandleCurveActiveToggle(FString CurveID, bool bActive)
{
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> CurvesToToggle;
	bool bCheckedItemIsSelected = false;
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> SelectedCurves = CurveListView->GetSelectedItems();
	for (TSharedPtr<FMarkerOutlinerCurveData>& Curve : SelectedCurves)
	{
		if (Curve->Name == CurveID) //checked item is selected?
		{
			bCheckedItemIsSelected = true;
			CurvesToToggle.Add(Curve);
			break;
		}
	}

	if (bCheckedItemIsSelected)
	{
		CurvesToToggle = SelectedCurves;
	}

	TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames;
	ToggleCurvesActive(CurvesToToggle, bActive, AffectedFrames);
	//points don't have active flag, so nothing to do here

}

TArray<FString> SMetaHumanMarkersOutliner::GetSelectedCurvesList() const
{
	TArray<FString> Selection;
	for(const TSharedPtr<FMarkerOutlinerCurveData>& CurveData : CurveListView->GetSelectedItems())
	{
		Selection.Add(CurveData->Name);
	}

	return Selection;
}

void SMetaHumanMarkersOutliner::ToggleGroupsVisibility(TArray<TSharedPtr<FMarkerOutlinerGroupData>> GroupItems, bool bVisibility, TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames)
{
	for (TSharedPtr<FMarkerOutlinerGroupData>& Group : GroupItems)
	{
		Group->State.bVisible = bVisibility;
	}
	GroupListView->RequestListRefresh();
}

void SMetaHumanMarkersOutliner::ToggleGroupsActive(TArray<TSharedPtr<FMarkerOutlinerGroupData>> GroupItems, bool bActive, TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames)
{
	//use GroupItems from the Outliner UI (current frame) to set whether they are active in affected frames
	for (TSharedPtr<FMarkerOutlinerGroupData>& Group : GroupItems)
	{
		Group->State.bActive = bActive;
	}
	GroupListView->RequestListRefresh();
}

void SMetaHumanMarkersOutliner::ToggleFramesActive(TArray<TSharedPtr<FMarkerOutlinerFrameData>> FrameItems, bool bActive)
{
	for (TSharedPtr<FMarkerOutlinerFrameData>& FrameItem : FrameItems)
	{
		FrameItem->Frame->bUseToSolve = bActive;
	}
	FrameListView->RequestListRefresh();
}

void SMetaHumanMarkersOutliner::ToggleCurvesVisibility(TArray<TSharedPtr<FMarkerOutlinerCurveData>> CurveItems, bool bVisibility, TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames)
{
	for (TSharedPtr<FMarkerOutlinerCurveData>& Curve : CurveItems)
	{
		Curve->State.bVisible = bVisibility;
	}
	CurveListView->RequestListRefresh();
}

void SMetaHumanMarkersOutliner::ToggleCurvesActive(TArray<TSharedPtr<FMarkerOutlinerCurveData>> CurveItems, bool bActive, TArray<UMetaHumanIdentityPromotedFrame*> AffectedFrames)
{
	for (TSharedPtr<FMarkerOutlinerCurveData>& Curve : CurveItems)
	{
		Curve->State.bActive = bActive;
	}
	CurveListView->RequestListRefresh();
}

void SMarkerOutlinerGroupStructListRow::OnGroupVisibilityStateChanged(ECheckBoxState NewState)
{
	Item->State.bVisible = NewState == ECheckBoxState::Checked ? true : false;
	OnGroupVisibilityToggleDelegate.ExecuteIfBound(Item->Name, Item->State.bVisible);
}

void SMarkerOutlinerGroupStructListRow::OnGroupActiveStateChanged(ECheckBoxState NewState)
{
	Item->State.bActive = NewState == ECheckBoxState::Checked ? true : false;
	OnGroupActiveToggleDelegate.ExecuteIfBound(Item->Name, Item->State.bActive);
}

void SMarkerOutlinerFrameStructListRow::OnFrameActiveStateChanged(ECheckBoxState NewState)
{
	Item->Frame->bUseToSolve = NewState == ECheckBoxState::Checked ? true : false;
	OnFrameActiveToggleDelegate.ExecuteIfBound(Item->Frame, Item->Frame->bUseToSolve);
}

void SMarkerOutlinerCurveStructListRow::OnCurveVisibilityStateChanged(ECheckBoxState NewState)
{
	Item->State.bVisible = NewState == ECheckBoxState::Checked ? true : false;
	OnCurveVisibilityToggleDelegate.ExecuteIfBound(Item->Name, Item->State.bVisible);
}

void SMarkerOutlinerCurveStructListRow::OnCurveActiveStateChanged(ECheckBoxState NewState)
{
	Item->State.bActive = NewState == ECheckBoxState::Checked ? true : false;
	OnCurveActiveToggleDelegate.ExecuteIfBound(Item->Name, Item->State.bActive);
}

TSharedRef<SWidget> SMarkerOutlinerFrameStructListRow::GenerateWidgetForColumn(const FName& ColumnName)
{
	if (ColumnName.IsEqual(MarkerOutlinerFrameView::ColumnID_FrameName))
	{
		return SNew(SBox)
			.Padding(FMargin(4.0f, 0.0f))
			.VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Text(FText::FromString(*(Item.Get())->Name))
			];
	}
	else if (ColumnName.IsEqual(MarkerOutlinerFrameView::ColumnID_FrameActive))
	{
		return SNew(SBox)
			.Padding(FMargin(4.0f, 0.0f))
			.VAlign(VAlign_Center)
			[
				SAssignNew(ActiveCheckBox, SCheckBox)
				.IsChecked_Lambda([this]
					{
						return Item->Frame->bUseToSolve ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
					})
			.OnCheckStateChanged(this, &SMarkerOutlinerFrameStructListRow::OnFrameActiveStateChanged)
			];
	}

	// property not found
	return SNullWidget::NullWidget;
}

TSharedRef<SWidget> SMarkerOutlinerGroupStructListRow::GenerateWidgetForColumn(const FName& ColumnName)
{
	if (ColumnName.IsEqual(MarkerOutlinerGroupsView::ColumnID_GroupName))
	{
		return SNew(SBox)
			.Padding(FMargin(4.0f, 0.0f))
			.VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Text(FText::FromString(*(Item.Get())->Name))
			];
	}
	else if (ColumnName.IsEqual(MarkerOutlinerGroupsView::ColumnID_GroupVisible))
	{
		return SNew(SBox)
			.Padding(FMargin(4.0f, 0.0f))
			.VAlign(VAlign_Center)
			[
				SAssignNew(VisibilityCheckBox, SCheckBox)
				.IsChecked_Lambda([this]
					{
						return Item->State.bVisible ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
					})
			.OnCheckStateChanged(this, &SMarkerOutlinerGroupStructListRow::OnGroupVisibilityStateChanged)
			];
	}
	else if (ColumnName.IsEqual(MarkerOutlinerGroupsView::ColumnID_GroupActive))
	{
		return SNew(SBox)
			.Padding(FMargin(4.0f, 0.0f))
			.VAlign(VAlign_Center)
			[
				SAssignNew(ActiveCheckBox, SCheckBox)
				.IsChecked_Lambda([this]
					{
						return Item->State.bActive ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
					})
			.OnCheckStateChanged(this, &SMarkerOutlinerGroupStructListRow::OnGroupActiveStateChanged)
			];
	}

	// default to null widget if property cannot be found
	return SNullWidget::NullWidget;
}

TSharedRef<SWidget> SMarkerOutlinerCurveStructListRow::GenerateWidgetForColumn(const FName& ColumnName)
{
	if (ColumnName.IsEqual(MarkerOutlinerCurvesView::ColumnID_CurveName))
	{
		return SNew(SBox)
			.Padding(FMargin(4.0f, 0.0f))
			.VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Text(FText::FromString(*(Item.Get())->Name))
			];
	}
	else if (ColumnName.IsEqual(MarkerOutlinerCurvesView::ColumnID_CurveVisible))
	{
		return SNew(SBox)
			.Padding(FMargin(4.0f, 0.0f))
			.VAlign(VAlign_Center)
			[
				SAssignNew(VisibilityCheckBox, SCheckBox)
				.IsChecked_Lambda([this]
					{
						return Item->State.bVisible ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
					})
			.OnCheckStateChanged(this, &SMarkerOutlinerCurveStructListRow::OnCurveVisibilityStateChanged)
			];
	}
	else if (ColumnName.IsEqual(MarkerOutlinerCurvesView::ColumnID_CurveActive))
	{
		return SNew(SBox)
			.Padding(FMargin(4.0f, 0.0f))
			.VAlign(VAlign_Center)
			[
				SAssignNew(ActiveCheckBox, SCheckBox)
				.IsChecked_Lambda([this]
					{
						return Item->State.bActive ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
					})
			.OnCheckStateChanged(this, &SMarkerOutlinerCurveStructListRow::OnCurveActiveStateChanged)
			];
	}

	// property not found
	return SNullWidget::NullWidget;
}

#undef LOCTEXT_NAMESPACE