// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/STreeView.h"
#include "EditorUndoClient.h"


/////////////////////////////////////////////////////
// FIdentityOutlinerTreeNode

struct FIdentityOutlinerTreeNode
{
	/** The promoted frame associated with this node */
	TWeakObjectPtr<class UMetaHumanIdentityPromotedFrame> PromotedFrame;

	/** The index of the promoted frame associated with this node */
	int32 FrameIndex = INDEX_NONE;

	/** Group name as visualised by the outliner */
	FText OutlinerGroupName;

	/** Curve name as visualised by the outliner */
	FText OutlinerCurveName;

	/** Internal group name as specified in the group config. Internal names are used by everything outside the ouliner */
	FString InternalGroupName;

	/** Internal curve name as specified in the curves config. Internal names are used by everything outside the outliner */
	FString InternalCurveName;

	/** The parent of this node, nullptr if this is the root */
	TWeakPtr<FIdentityOutlinerTreeNode> Parent;

	/** The list of child nodes */
	TArray<TSharedRef<FIdentityOutlinerTreeNode>> Children;

	/** If the node is visible in the tree view */
	bool bIsNodeVisible  = true;

	bool IsFrameNode() const;

	bool IsGroupNode() const;

	bool IsCurveNode() const;

	FText GetLabel() const;

	void OnVisibleStateChanged(ECheckBoxState InNewState);

	void VisibleStateChangedRecursive(ECheckBoxState InNewState);

	bool IsKeypointVisibleForAnyCurve(const FString& InKeypointName);

	ECheckBoxState IsVisibleCheckState() const;

	void OnActiveStateChanged(ECheckBoxState InNewState);

	void ActiveStateChangedRecursive(ECheckBoxState InNewState);

	bool IsKeypointActiveForAnyCurve(const FString& InKeypointName);

	ECheckBoxState IsActiveCheckState() const;

	void SetSelected(bool bIsSelected);

	bool IsSelected(bool bInRecursive = true) const;

	FSimpleDelegate OnResetImageViewerPointsDelegate;
};

// using FIdentityOutlinerTreeNodeRef = TSharedRef<FIdentityOutlinerTreeNode>;

/////////////////////////////////////////////////////
// SMetaHumanIdentityOutliner

DECLARE_DELEGATE_OneParam(FOnOutlinerSelectionChanged, const TArray<FString>& SelectedCurves)

class SMetaHumanIdentityOutliner
	: public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SMetaHumanIdentityOutliner) {}

		SLATE_ARGUMENT(TSharedPtr<struct FMarkerDefs>, MarkerDefs)

		SLATE_ARGUMENT(UMetaHumanIdentityPromotedFrame*, PromotedFrame)

		SLATE_ARGUMENT(int32, PromotedFrameIndex)

		SLATE_EVENT(FOnOutlinerSelectionChanged, OnSelectionChanged)

		SLATE_EVENT(FSimpleDelegate, OnResetImageViewerPoints)

	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	void SetPromotedFrame(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, int32 InFrameIndex);

	/** */
	void ClearSelection();

	/** */
	void RefreshSelection();

private:

	/** Create the Header Row for the outliner tree view */
	TSharedRef<SHeaderRow> MakeHeaderRow() const;

	/** */
	TSharedRef<ITableRow> HandleGenerateOutlinerTreeRow(TSharedRef<FIdentityOutlinerTreeNode> InItem, const TSharedRef<STableViewBase>& InOnwerTable);

	/** */
	void HandleOutlinerTreeGetChildren(TSharedRef<FIdentityOutlinerTreeNode> InItem, TArray<TSharedRef<FIdentityOutlinerTreeNode>>& OutChildren);

	/** */
	void HandleOutlinerTreeSelectionChanged(TSharedPtr<FIdentityOutlinerTreeNode> InItem, ESelectInfo::Type InSelectInfo);

	/** Enable recursive expansion using Shift + click to expand a node */
	void HandleOutlinerTreeSetExpansionRecursive(TSharedRef<FIdentityOutlinerTreeNode> InItem, bool bInShouldExpand);

	/** Populate the name mapping from specified config path. Returns true if parsing was successful */
	void CreateCurveNameMappingFromFile();

	/** */
	TSharedRef<FIdentityOutlinerTreeNode> MakeOutlinerTreeNodeForPromotedFrame(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, int32 InFrameIndex);

	/** */
	void FindSelectedItemsRecursive(TSharedRef<FIdentityOutlinerTreeNode> InItem, TArray<TSharedRef<FIdentityOutlinerTreeNode>>& OutSelectedItems) const;

	/** */
	TArray<FString> FindSelectedCurveNames() const;

private:

	/** Reference to the Promoted Frame we are editing */
	TWeakObjectPtr<class UMetaHumanIdentityPromotedFrame> PromotedFrame;

	/** */
	TSharedPtr<struct FMarkerDefs> MarkerDefs;

	/** Command list for handling actions in the tree view */
	TSharedPtr<class FUICommandList> CommandList;

	/** A pointer to the Identity tree view */
	TSharedPtr<STreeView<TSharedRef<FIdentityOutlinerTreeNode>>> OutlinerTreeWidget;

	/** List of root nodes of the Outliner tree */
	TArray<TSharedRef<FIdentityOutlinerTreeNode>> RootNodes;

	/** Mapping for Outliner curve names */
	TMap<FString, FText> InternalToOutlinerNamingMap;

	/** */
	FOnOutlinerSelectionChanged OnSelectionChangedDelegate;

	FSimpleDelegate OnResetImageViewerPointsDelegate;
};