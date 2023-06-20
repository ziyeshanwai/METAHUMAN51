// Copyright Epic Games, Inc. All Rights Reserved.

#include "SMetaHumanIdentityOutliner.h"
#include "MetaHumanIdentity.h"
#include "MetaHumanIdentityPromotedFrames.h"
#include "MetaHumanIdentityMarkerData.h"
#include "ShapeAnnotationWrapper.h"

#include "Widgets/Input/SSearchBox.h"
#include "Widgets/Input/SCheckBox.h"
#include "Algo/Count.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/FileHelper.h"
#include "ScopedTransaction.h"

#define LOCTEXT_NAMESPACE "MetaHumanIdentityOutliner"

static const FName OutlinerTree_ColumnName_Frame = TEXT("Frame");
static const FName OutlinerTree_ColumnName_Visible = TEXT("Visible");
static const FName OutlinerTree_ColumnName_Active = TEXT("Active");

static const TCHAR* IdentityOutlinerTransactionContext = TEXT("IdentityOutlinerTransaction");

/////////////////////////////////////////////////////
// FIdentityOutlinerTreeNode

bool FIdentityOutlinerTreeNode::IsGroupNode() const
{
	return !InternalGroupName.IsEmpty();
}

bool FIdentityOutlinerTreeNode::IsCurveNode() const
{
	return !OutlinerCurveName.IsEmpty();
}

bool FIdentityOutlinerTreeNode::IsFrameNode() const
{
	return PromotedFrame.IsValid() && FrameIndex != INDEX_NONE;
}

FText FIdentityOutlinerTreeNode::GetLabel() const
{
	if (IsGroupNode())
	{
		return OutlinerGroupName;
	}

	if (IsCurveNode())
	{
		return OutlinerCurveName;
	}

	if (!PromotedFrame->FrameName.IsEmptyOrWhitespace())
	{
		return PromotedFrame->FrameName;
	}
	else
	{
		return FText::Format(LOCTEXT("FrameLabel", "Frame {0}"), { FrameIndex });
	}

	return LOCTEXT("InvalidNodeName", "<Invalid Node>");
}

void FIdentityOutlinerTreeNode::OnVisibleStateChanged(ECheckBoxState InNewState)
{
	VisibleStateChangedRecursive(InNewState);

	OnResetImageViewerPointsDelegate.ExecuteIfBound();
}

void FIdentityOutlinerTreeNode::VisibleStateChangedRecursive(ECheckBoxState InNewState)
{
	if (IsGroupNode())
	{
		const FScopedTransaction Transaction(IdentityOutlinerTransactionContext, LOCTEXT("GroupVisibilityChangedTransaction", "Edit Group Is Visible"), PromotedFrame.Get());
		PromotedFrame->Modify();

		for (const TSharedRef<FIdentityOutlinerTreeNode>& Child : Children)
		{
			Child->VisibleStateChangedRecursive(InNewState);
		}
	}

	if (IsCurveNode() && PromotedFrame.IsValid())
	{
		const FScopedTransaction Transaction(IdentityOutlinerTransactionContext, LOCTEXT("CurveVisibilityChangedTransaction", "Edit Curve Is Visible"), PromotedFrame.Get());
		PromotedFrame->Modify();

		const bool bIsVisible = (InNewState == ECheckBoxState::Checked);

		TMap<FString, FTrackingContour>& Contours = PromotedFrame->ContourData.TrackingContours;

		const FString& CurveStartPoint = Contours[InternalCurveName].StartPointName;
		const FString& CurveEndPointName = Contours[InternalCurveName].EndPointName;

		Contours[InternalCurveName].State.bVisible = bIsVisible;

		if (!CurveStartPoint.IsEmpty())
		{
			Contours[CurveStartPoint].State.bVisible = IsKeypointVisibleForAnyCurve(CurveStartPoint);
		}

		if (!CurveEndPointName.IsEmpty())
		{
			Contours[CurveEndPointName].State.bVisible = IsKeypointVisibleForAnyCurve(CurveEndPointName);
		}
	}

	if (IsFrameNode())
	{
		const FScopedTransaction Transaction(IdentityOutlinerTransactionContext, LOCTEXT("FrameVisibilityChangedTransaction", "Edit PromotedFrame Is Visible"), PromotedFrame.Get());
		PromotedFrame->Modify();

		for (const TSharedRef<FIdentityOutlinerTreeNode>& Child : Children)
		{
			Child->VisibleStateChangedRecursive(InNewState);
		}
	}
}

bool FIdentityOutlinerTreeNode::IsKeypointVisibleForAnyCurve(const FString& InKeypointName)
{
	TMap<FString, FTrackingContour>& Contours = PromotedFrame->ContourData.TrackingContours;
	bool bVisible = false;

	for(const TPair<FString, FTrackingContour>& Contour : Contours)
	{
		if(Contour.Value.StartPointName == InKeypointName || Contour.Value.EndPointName == InKeypointName)
		{
			bVisible |= Contour.Value.State.bVisible;
		}
	}

	return bVisible;
}

ECheckBoxState FIdentityOutlinerTreeNode::IsVisibleCheckState() const
{
	if (IsCurveNode() && PromotedFrame.IsValid())
	{
		if (PromotedFrame->ContourData.TrackingContours[InternalCurveName].State.bVisible)
		{
			return ECheckBoxState::Checked;
		}
		else
		{
			return ECheckBoxState::Unchecked;
		}
	}

	if (IsGroupNode() || IsFrameNode())
	{
		const int32 NumVisibleChildren = Algo::CountIf(Children, [](const TSharedRef<FIdentityOutlinerTreeNode>& Child)
		{
			return Child->IsVisibleCheckState() == ECheckBoxState::Checked;
		});

		if (NumVisibleChildren == Children.Num())
		{
			return ECheckBoxState::Checked;
		}

		if (NumVisibleChildren == 0)
		{
			return ECheckBoxState::Unchecked;
		}
	}

	return ECheckBoxState::Undetermined;
}

void FIdentityOutlinerTreeNode::OnActiveStateChanged(ECheckBoxState InNewState)
{
	ActiveStateChangedRecursive(InNewState);

	OnResetImageViewerPointsDelegate.ExecuteIfBound();
}

void FIdentityOutlinerTreeNode::ActiveStateChangedRecursive(ECheckBoxState InNewState)
{
	if (IsGroupNode())
	{
		const FScopedTransaction Transaction(IdentityOutlinerTransactionContext, LOCTEXT("GroupActiveChangedTransaction", "Edit Group Is Active"), PromotedFrame.Get());
		PromotedFrame->Modify();

		for (const TSharedRef<FIdentityOutlinerTreeNode>& Child : Children)
		{
			Child->ActiveStateChangedRecursive(InNewState);
		}
	}

	if (IsCurveNode() && PromotedFrame.IsValid())
	{
		const FScopedTransaction Transaction(IdentityOutlinerTransactionContext, LOCTEXT("CurveActiveChangedTransaction", "Edit Curve Is Active"), PromotedFrame.Get());
		PromotedFrame->Modify();

		TMap<FString, FTrackingContour>& Contours = PromotedFrame->ContourData.TrackingContours;
		const bool bIsActive = (InNewState == ECheckBoxState::Checked);

		const FString& CurveStartPoint = Contours[InternalCurveName].StartPointName;
		const FString& CurveEndPointName = Contours[InternalCurveName].EndPointName;

		Contours[InternalCurveName].State.bActive = bIsActive;

		if (!CurveStartPoint.IsEmpty())
		{
			Contours[CurveStartPoint].State.bActive = IsKeypointActiveForAnyCurve(CurveStartPoint);
		}

		if (!CurveEndPointName.IsEmpty())
		{
			Contours[CurveEndPointName].State.bActive = IsKeypointActiveForAnyCurve(CurveEndPointName);
		}
	}

	if (IsFrameNode())
	{
		const FScopedTransaction Transaction(IdentityOutlinerTransactionContext, LOCTEXT("FrameActiveChangedTransaction", "Edit PromotedFrame Is Active"), PromotedFrame.Get());
		PromotedFrame->Modify();

		for (const TSharedRef<FIdentityOutlinerTreeNode>& Child : Children)
		{
			Child->ActiveStateChangedRecursive(InNewState);
		}
	}
}

bool FIdentityOutlinerTreeNode::IsKeypointActiveForAnyCurve(const FString& InKeypointName)
{
	TMap<FString, FTrackingContour>& Contours = PromotedFrame->ContourData.TrackingContours;
	bool bActive = false;

	for(const TPair<FString, FTrackingContour>& Contour : Contours)
	{
		if(Contour.Value.StartPointName == InKeypointName || Contour.Value.EndPointName == InKeypointName)
		{
			bActive |= Contour.Value.State.bActive;
		}
	}

	return bActive;
}

ECheckBoxState FIdentityOutlinerTreeNode::IsActiveCheckState() const
{
	if (IsCurveNode() && PromotedFrame.IsValid())
	{
		if (PromotedFrame->ContourData.TrackingContours[InternalCurveName].State.bActive)
		{
			return ECheckBoxState::Checked;
		}
		else
		{
			return ECheckBoxState::Unchecked;
		}
	}

	if (IsGroupNode() || IsFrameNode())
	{
		const int32 NumActiveChildren = Algo::CountIf(Children, [](const TSharedRef<FIdentityOutlinerTreeNode>& Child)
		{
			return Child->IsActiveCheckState() == ECheckBoxState::Checked;
		});

		if (NumActiveChildren == Children.Num())
		{
			return ECheckBoxState::Checked;
		}

		if (NumActiveChildren == 0)
		{
			return ECheckBoxState::Unchecked;
		}
	}

	return ECheckBoxState::Undetermined;
}

void FIdentityOutlinerTreeNode::SetSelected(bool bIsSelected)
{
	if (IsCurveNode() && PromotedFrame.IsValid())
	{
		TMap<FString, FTrackingContour>& Contours = PromotedFrame->ContourData.TrackingContours;

		const FString& CurveStartPoint = Contours[InternalCurveName].StartPointName;
		const FString& CurveEndPointName = Contours[InternalCurveName].EndPointName;

		Contours[InternalCurveName].State.bSelected = bIsSelected;

		if (!CurveStartPoint.IsEmpty())
		{
			Contours[CurveStartPoint].State.bSelected = bIsSelected;
		}

		if (!CurveEndPointName.IsEmpty())
		{
			Contours[CurveEndPointName].State.bSelected = bIsSelected;
		}
	}

	if (IsGroupNode() || IsFrameNode())
	{
		for (const TSharedRef<FIdentityOutlinerTreeNode>& Child : Children)
		{
			Child->SetSelected(bIsSelected);
		}
	}
}

bool FIdentityOutlinerTreeNode::IsSelected(bool bInRecursive) const
{
	if (IsCurveNode() && PromotedFrame.IsValid())
	{
		return PromotedFrame->ContourData.TrackingContours[InternalCurveName].State.bSelected;
	}

	if (bInRecursive && (IsGroupNode() || IsFrameNode()))
	{
		const int32 NumChildrenSelected = Algo::CountIf(Children, [](const TSharedRef<FIdentityOutlinerTreeNode>& Child)
		{
			return Child->IsSelected();
		});

		return NumChildrenSelected == Children.Num();
	}

	return false;
}

/////////////////////////////////////////////////////
// SOutlinerTreeRow

class SOutlinerTreeRow
	: public SMultiColumnTableRow<TSharedRef<FIdentityOutlinerTreeNode>>
{
public:
	SLATE_BEGIN_ARGS(SOutlinerTreeRow) {}
		SLATE_ARGUMENT(TSharedPtr<FIdentityOutlinerTreeNode>, Item)
		SLATE_ARGUMENT(FSimpleDelegate, OnResetImageViewerPoints)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, const TSharedRef<STableViewBase>& InOwnerTableBase)
	{
		Item = InArgs._Item;
		Item->OnResetImageViewerPointsDelegate = InArgs._OnResetImageViewerPoints;

		SMultiColumnTableRow<TSharedRef<FIdentityOutlinerTreeNode>>::Construct(FSuperRowType::FArguments(), InOwnerTableBase);
	}

	virtual TSharedRef<SWidget> GenerateWidgetForColumn(const FName& InColumnName) override
	{
		if (InColumnName == OutlinerTree_ColumnName_Frame)
		{
			return SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
				.AutoWidth()
				[
					SNew(SExpanderArrow, SharedThis(this))
					.IndentAmount(16)
					.ShouldDrawWires(true)
				]
				+ SHorizontalBox::Slot()
				.VAlign(VAlign_Center)
				.HAlign(HAlign_Left)
				.AutoWidth()
				[
					SNew(STextBlock)
					.Margin(4)
					.Text(Item.ToSharedRef(), &FIdentityOutlinerTreeNode::GetLabel)
				];
		}

		if (InColumnName == OutlinerTree_ColumnName_Visible)
		{
			return SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
				.VAlign(VAlign_Center)
				.HAlign(HAlign_Center)
				[
					SNew(SCheckBox)
					.IsChecked(Item.ToSharedRef(), &FIdentityOutlinerTreeNode::IsVisibleCheckState)
					.OnCheckStateChanged(Item.ToSharedRef(), &FIdentityOutlinerTreeNode::OnVisibleStateChanged)
				];
		}

		if (InColumnName == OutlinerTree_ColumnName_Active)
		{
			return SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
				.VAlign(VAlign_Center)
				.HAlign(HAlign_Center)
				[
					SNew(SCheckBox)
					.IsChecked(Item.ToSharedRef(), &FIdentityOutlinerTreeNode::IsActiveCheckState)
					.OnCheckStateChanged(Item.ToSharedRef(), &FIdentityOutlinerTreeNode::OnActiveStateChanged)
				];
		}

		return SNullWidget::NullWidget;
	}

private:

	TSharedPtr<FIdentityOutlinerTreeNode> Item;
};

/////////////////////////////////////////////////////
// SMetaHumanIdentityOutliner

void SMetaHumanIdentityOutliner::Construct(const FArguments& InArgs)
{
	MarkerDefs = InArgs._MarkerDefs;
	OnSelectionChangedDelegate = InArgs._OnSelectionChanged;
	OnResetImageViewerPointsDelegate = InArgs._OnResetImageViewerPoints;

	check(MarkerDefs.IsValid());

	CreateCurveNameMappingFromFile();

	ChildSlot
	[
		SNew(SVerticalBox)
		+SVerticalBox::Slot()
		[
			SAssignNew(OutlinerTreeWidget, STreeView<TSharedRef<FIdentityOutlinerTreeNode>>)
			.SelectionMode(ESelectionMode::Multi)
			.TreeItemsSource(&RootNodes)
			.AllowInvisibleItemSelection(true)
			.HighlightParentNodesForSelection(true)
			.HeaderRow(MakeHeaderRow())
			.OnGenerateRow(this, &SMetaHumanIdentityOutliner::HandleGenerateOutlinerTreeRow)
			.OnGetChildren(this, &SMetaHumanIdentityOutliner::HandleOutlinerTreeGetChildren)
			.OnSetExpansionRecursive(this, &SMetaHumanIdentityOutliner::HandleOutlinerTreeSetExpansionRecursive)
			.OnSelectionChanged(this, &SMetaHumanIdentityOutliner::HandleOutlinerTreeSelectionChanged)
		]
	];

	SetPromotedFrame(InArgs._PromotedFrame, InArgs._PromotedFrameIndex);
}

void SMetaHumanIdentityOutliner::SetPromotedFrame(UMetaHumanIdentityPromotedFrame* InPromotedFrame, int32 InFrameIndex)
{
	if (InPromotedFrame != PromotedFrame)
	{
		PromotedFrame = InPromotedFrame;

		if (PromotedFrame.IsValid())
		{
			RootNodes = { MakeOutlinerTreeNodeForPromotedFrame(PromotedFrame.Get(), InFrameIndex) };

			OutlinerTreeWidget->SetItemExpansion(RootNodes[0], true);
		}
		else
		{
			RootNodes.Empty();
		}

		OutlinerTreeWidget->RequestTreeRefresh();
	}
}

void SMetaHumanIdentityOutliner::ClearSelection()
{
	for (const TSharedRef<FIdentityOutlinerTreeNode>& FrameNode : RootNodes)
	{
		FrameNode->SetSelected(false);
	}
}

void SMetaHumanIdentityOutliner::RefreshSelection()
{
	if (OutlinerTreeWidget.IsValid())
	{
		TArray<TSharedRef<FIdentityOutlinerTreeNode>> SelectedNodes;

		for (const TSharedRef<FIdentityOutlinerTreeNode>& FrameNode : RootNodes)
		{
			FindSelectedItemsRecursive(FrameNode, SelectedNodes);
		}

		for (const TSharedRef<FIdentityOutlinerTreeNode>& SelectedNode : SelectedNodes)
		{
			OutlinerTreeWidget->SetItemExpansion(SelectedNode, true);

			if (SelectedNode->IsCurveNode())
			{
				TWeakPtr<FIdentityOutlinerTreeNode> CurrentNode = SelectedNode->Parent;
				while (CurrentNode.IsValid())
				{
					OutlinerTreeWidget->SetItemExpansion(CurrentNode.Pin().ToSharedRef(), true);

					CurrentNode = CurrentNode.Pin()->Parent;
				}
			}
		}

		OutlinerTreeWidget->ClearSelection();
		OutlinerTreeWidget->SetItemSelection(SelectedNodes, true);
		OutlinerTreeWidget->RequestTreeRefresh();
	}
}

TSharedRef<SHeaderRow> SMetaHumanIdentityOutliner::MakeHeaderRow() const
{
	return SNew(SHeaderRow)
		.Visibility(EVisibility::All)

		+ SHeaderRow::Column(OutlinerTree_ColumnName_Frame)
		.DefaultLabel(LOCTEXT("OutlineColumnLabel_Frame", "Frame"))
		.FillWidth(0.6f)

		+SHeaderRow::Column(OutlinerTree_ColumnName_Visible)
		.DefaultLabel(LOCTEXT("OutlineColumnLabel_Visible", "Visible"))
		.FillWidth(0.2f)

		+ SHeaderRow::Column(OutlinerTree_ColumnName_Active)
		.DefaultLabel(LOCTEXT("OutlineColumnLabel_Active", "Active"))
		.FillWidth(0.2f);
}

TSharedRef<ITableRow> SMetaHumanIdentityOutliner::HandleGenerateOutlinerTreeRow(TSharedRef<FIdentityOutlinerTreeNode> InItem, const TSharedRef<STableViewBase>& InOnwerTable)
{
	return SNew(SOutlinerTreeRow, InOnwerTable)
		.Item(InItem)
		.OnResetImageViewerPoints(OnResetImageViewerPointsDelegate);
}

void SMetaHumanIdentityOutliner::HandleOutlinerTreeGetChildren(TSharedRef<FIdentityOutlinerTreeNode> InItem, TArray<TSharedRef<FIdentityOutlinerTreeNode>>& OutChildren)
{
	for (const TSharedRef<FIdentityOutlinerTreeNode>& Child : InItem->Children)
	{
		if (Child->bIsNodeVisible)
		{
			OutChildren.Add(Child);
		}
	}
}

void SMetaHumanIdentityOutliner::HandleOutlinerTreeSelectionChanged(TSharedPtr<FIdentityOutlinerTreeNode> InItem, ESelectInfo::Type InSelectInfo)
{
	for (const TSharedRef<FIdentityOutlinerTreeNode>& FrameNode : RootNodes)
	{
		FrameNode->SetSelected(false);
	}

	const TArray<TSharedRef<FIdentityOutlinerTreeNode>> SelectedItems = OutlinerTreeWidget->GetSelectedItems();
	for (const TSharedRef<FIdentityOutlinerTreeNode>& Item : SelectedItems)
	{
		Item->SetSelected(true);
	}

	if (OnSelectionChangedDelegate.IsBound())
	{
		const TArray<FString> SelectedCurveNames = FindSelectedCurveNames();
		OnSelectionChangedDelegate.Execute(SelectedCurveNames);
	}
}

void SMetaHumanIdentityOutliner::HandleOutlinerTreeSetExpansionRecursive(TSharedRef<FIdentityOutlinerTreeNode> InItem, bool bInShouldExpand)
{
	if (OutlinerTreeWidget != nullptr)
	{
		OutlinerTreeWidget->SetItemExpansion(InItem, bInShouldExpand);

		for (const TSharedRef<FIdentityOutlinerTreeNode>& Child : InItem->Children)
		{
			HandleOutlinerTreeSetExpansionRecursive(Child, bInShouldExpand);
		}
	}
}

void SMetaHumanIdentityOutliner::CreateCurveNameMappingFromFile()
{
	InternalToOutlinerNamingMap =
	{
		// Curves
		{ TEXT("crv_lip_upper_outer_l"), LOCTEXT("CrvLipOuterUpperL", "Lip Outer Upper (L)") },
		{ TEXT("crv_lip_upper_outer_r"), LOCTEXT("CrvLipOuterUpperR", "Lip Outer Upper (R)") },
		{ TEXT("crv_lip_lower_outer_l"), LOCTEXT("CrvLipOuterLowerL", "Lip Outer Lower (L)") },
		{ TEXT("crv_lip_lower_outer_r"), LOCTEXT("CrvLipOuterLowerR", "Lip Outer Lower (R)") },
		{ TEXT("crv_lip_lower_inner_l"), LOCTEXT("CrvLipInnerLowerL", "Lip Inner Lower (L)") },
		{ TEXT("crv_lip_lower_inner_r"), LOCTEXT("CrvLipInnerLowerR", "Lip Inner Lower (R)") },
		{ TEXT("crv_lip_upper_inner_l"), LOCTEXT("CrvLipInnerUpperL", "Lip Inner Upper (L)") },
		{ TEXT("crv_lip_upper_inner_r"), LOCTEXT("CrvLipInnerUpperR", "Lip Inner Upper (R)") },
		{ TEXT("crv_lip_philtrum_l"), LOCTEXT("CrvLipPhiltrumL", "Lip Philtrum (L)") },
		{ TEXT("crv_lip_philtrum_r"), LOCTEXT("CrvLipPhiltrumR", "Lip Philtrum (R)") },
		{ TEXT("crv_nasolabial_l"), LOCTEXT("CrvNasolabialL", "Nasolabial (L)") },
		{ TEXT("crv_nasolabial_r"), LOCTEXT("CrvNasolabialR", "Nasolabial (R)") },
		{ TEXT("crv_nostril_l"), LOCTEXT("CrvNostrilL", "Nostril (L)") },
		{ TEXT("crv_nostril_r"), LOCTEXT("CrvNostrilR", "Nostril (R)") },
		{ TEXT("crv_ear_outer_helix_l"), LOCTEXT("CrvEarHelixOuterL", "Ear Helix Outer (L)") },
		{ TEXT("crv_ear_outer_helix_r"), LOCTEXT("CrvEarHelixOuterR", "Ear Helix Outer (R)") },
		{ TEXT("crv_ear_inner_helix_l"), LOCTEXT("CrvEarHelixInnerL", "Ear Helix Inner (L)") },
		{ TEXT("crv_ear_inner_helix_r"), LOCTEXT("CrvEarHelixInnerR", "Ear Helix Inner (R)") },
		{ TEXT("crv_ear_central_lower_l"), LOCTEXT("EarCentralLowerL", "Ear Central Lower (L)") },
		{ TEXT("crv_ear_central_lower_r"), LOCTEXT("EarCentralLowerR", "Ear Central Lower (R)") },
		{ TEXT("crv_ear_central_upper_l"), LOCTEXT("EarCentralUpperL", "Ear Central Upper (L)") },
		{ TEXT("crv_ear_central_upper_r"), LOCTEXT("EarCentralUpperR", "Ear Central Upper (R)") },
		{ TEXT("brow_middle_line_l"), LOCTEXT("CrvBrowMiddleL", "Brow Middle (L)" ) },
		{ TEXT("brow_middle_line_r"), LOCTEXT("CrvBrowMiddleR", "Brow Middle (R)") },
		{ TEXT("crv_mentolabial_fold"), LOCTEXT("CrvMentolabialFoldC", "Mentolabial Fold (C)") },
		{ TEXT("crv_eyecrease_l"), LOCTEXT("CrvEyeCreaseL", "Eye Crease (L)") },
		{ TEXT("crv_eyecrease_r"), LOCTEXT("CrvEyeCreaseR", "Eye Crease (R)") },
		{ TEXT("crv_eyelid_lower_l"), LOCTEXT("CrvEyelidLowerL", "Eyelid Lower (L)") },
		{ TEXT("crv_eyelid_lower_r"), LOCTEXT("CrvEyelidLowerR", "Eyelid Lower (R)") },
		{ TEXT("crv_eyelid_upper_l"), LOCTEXT("CrvEyelibUpperL", "Eyelid Upper (L)") },
		{ TEXT("crv_eyelid_upper_r"), LOCTEXT("CrvEyelibUpperR", "Eyelid Upper (R)") },
		{ TEXT("eye_plica_semilunaris_l"), LOCTEXT("CrvPlicaSmilunarisL", "Plica Semilunaris (L)") },
		{ TEXT("eye_plica_semilunaris_r"), LOCTEXT("CrvPlicaSmilunarisR", "Plica Semilunaris (R)") },
		{ TEXT("crv_outer_eyelid_edge_left_lower"), LOCTEXT("CrvEyelidOuterLowerL", "Eyelid Outer Lower (L)") },
		{ TEXT("crv_outer_eyelid_edge_right_lower"), LOCTEXT("CrvEyelidOuterLowerR", "Eyelid Outer Lower (R)") },
		{ TEXT("crv_outer_eyelid_edge_left_upper"), LOCTEXT("CrvEyelidOuterUpperL", "Eyelid Outer Upper (L)") },
		{ TEXT("crv_outer_eyelid_edge_right_upper"), LOCTEXT("CrvEyelidOuterUpperR", "Eyelid Outer Upper (R)") },

		// Groups
		{ TEXT("brow_l"), LOCTEXT("GrpBrowL", "Brow (L)") },
		{ TEXT("brow_r"), LOCTEXT("GrpBrowR", "Brow (R)") },
		{ TEXT("eye_l"), LOCTEXT("GrpEyeL", "Eye (L)") },
		{ TEXT("eye_r"), LOCTEXT("GrpEyeR", "Eye (R)") },
		{ TEXT("lip_upper"), LOCTEXT("GrpLipUpper", "Lip Upper") },
		{ TEXT("lip_lower"), LOCTEXT("GrpLipLower", "Lip Lower") },
		{ TEXT("nose_l"), LOCTEXT("GrpNoseL", "Nose (L)") },
		{ TEXT("nose_r"), LOCTEXT("GrpNoseR", "Nose (R)") },
		{ TEXT("ear_l"), LOCTEXT("GrpEarL", "Ear (L)") },
		{ TEXT("ear_r"), LOCTEXT("GrpEarR", "Ear (R)") },
	};
}

TSharedRef<FIdentityOutlinerTreeNode> SMetaHumanIdentityOutliner::MakeOutlinerTreeNodeForPromotedFrame(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, int32 InFrameIndex)
{
	TSharedRef<FIdentityOutlinerTreeNode> FrameNode = MakeShared<FIdentityOutlinerTreeNode>();
	FrameNode->PromotedFrame = InPromotedFrame;
	FrameNode->FrameIndex = InFrameIndex;

	TSet<FString> CurveSet;
	for (const FMarkerCurveDef& CurveDef : MarkerDefs->CurveDefs)
	{
		CurveSet.Add(CurveDef.Name);
	}

	for (const FString& GroupName : MarkerDefs->GroupNames)
	{
		TSharedRef<FIdentityOutlinerTreeNode> GroupNode = MakeShared<FIdentityOutlinerTreeNode>();
		GroupNode->PromotedFrame = InPromotedFrame;
		GroupNode->Parent = FrameNode;
		GroupNode->InternalGroupName = GroupName;
		GroupNode->OutlinerGroupName = InternalToOutlinerNamingMap.Contains(GroupName) ? InternalToOutlinerNamingMap[GroupName] : FText::FromString(GroupName);

		for (const FMarkerCurveDef& CurveDef : MarkerDefs->CurveDefs)
		{
			if (CurveDef.GroupTagIDs.Contains(GroupName))
			{
				TSharedRef<FIdentityOutlinerTreeNode> CurveNode = MakeShared<FIdentityOutlinerTreeNode>();
				CurveNode->PromotedFrame = InPromotedFrame;
				CurveNode->Parent = GroupNode;

				FText OutlinerCurveName = InternalToOutlinerNamingMap.Contains(CurveDef.Name) ? InternalToOutlinerNamingMap[CurveDef.Name] : FText::FromString(CurveDef.Name);
				CurveNode->OutlinerCurveName = OutlinerCurveName;
				CurveNode->InternalCurveName = CurveDef.Name;

				GroupNode->Children.Add(CurveNode);

				CurveSet.Remove(CurveDef.Name);
			}
		}

		FrameNode->Children.Add(GroupNode);
	}

	if (!CurveSet.IsEmpty())
	{
		TSharedRef<FIdentityOutlinerTreeNode> OtherGroupNode = MakeShared<FIdentityOutlinerTreeNode>();
		OtherGroupNode->PromotedFrame = InPromotedFrame;
		OtherGroupNode->Parent = FrameNode;
		OtherGroupNode->InternalGroupName = TEXT("Other");
		OtherGroupNode->OutlinerGroupName = LOCTEXT("GrpOther", "Other");

		// Add the remaining curves into an a virtual "Other" group
		for (const FString& OtherCurve : CurveSet)
		{
			TSharedRef<FIdentityOutlinerTreeNode> OtherCurveNode = MakeShared<FIdentityOutlinerTreeNode>();
			OtherCurveNode->PromotedFrame = InPromotedFrame;
			OtherCurveNode->Parent = OtherGroupNode;
			FText OutlinerCurveName = InternalToOutlinerNamingMap.Contains(OtherCurve) ? InternalToOutlinerNamingMap[OtherCurve] : FText::FromString(OtherCurve);
			OtherCurveNode->OutlinerCurveName = OutlinerCurveName;
			OtherCurveNode->InternalCurveName = OtherCurve;

			OtherGroupNode->Children.Add(OtherCurveNode);
		}

		FrameNode->Children.Add(OtherGroupNode);
	}

	return FrameNode;
}

void SMetaHumanIdentityOutliner::FindSelectedItemsRecursive(TSharedRef<FIdentityOutlinerTreeNode> InItem, TArray<TSharedRef<FIdentityOutlinerTreeNode>>& OutSelectedItems) const
{
	const bool bSearchRecursive = false;
	if (InItem->IsSelected(bSearchRecursive))
	{
		OutSelectedItems.Add(InItem);
	}

	for (const TSharedRef<FIdentityOutlinerTreeNode>& Child : InItem->Children)
	{
		FindSelectedItemsRecursive(Child, OutSelectedItems);
	}
}

TArray<FString> SMetaHumanIdentityOutliner::FindSelectedCurveNames() const
{
	TArray<TSharedRef<FIdentityOutlinerTreeNode>> SelectedNodes;
	for (const TSharedRef<FIdentityOutlinerTreeNode>& FrameNode : RootNodes)
	{
		FindSelectedItemsRecursive(FrameNode, SelectedNodes);
	}

	TArray<FString> CurveNodes;
	for (const TSharedRef<FIdentityOutlinerTreeNode>& SelectedNode : SelectedNodes)
	{
		if (SelectedNode->IsCurveNode())
		{
			CurveNodes.Add(SelectedNode->InternalCurveName);
		}
	}

	return CurveNodes;
}

#undef LOCTEXT_NAMESPACE