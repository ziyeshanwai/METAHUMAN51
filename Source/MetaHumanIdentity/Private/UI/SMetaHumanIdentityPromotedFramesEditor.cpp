// Copyright Epic Games, Inc. All Rights Reserved.

#include "SMetaHumanIdentityPromotedFramesEditor.h"
#include "MetaHumanIdentityPromotedFrames.h"
#include "MetaHumanIdentityParts.h"
#include "MetaHumanIdentityLog.h"
#include "MetaHumanIdentityMarkerData.h"
#include "CaptureData.h"
#include "MetaHumanIdentityStyle.h"
#include "MetaHumanIdentityCommands.h"
#include "ConformingViewportClient.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/SInlineEditableTextBlock.h"
#include "ScopedTransaction.h"
#include "Editor/Transactor.h"
#include "Framework/Application/SlateApplication.h"
#include "Framework/Multibox/MultiBoxBuilder.h"

#define LOCTEXT_NAMESPACE "MetaHumanIdentityPromotedFrames"

/////////////////////////////////////////////////////
// SPromotedFrameButton

DECLARE_DELEGATE_TwoParams(FIdentityPromotedFrameClicked, UMetaHumanIdentityPromotedFrame* InPromotedFrame, int32 InIndex)
DECLARE_DELEGATE_RetVal(int32, FIdentityPromotedFrameGetSelectedIndex)
DECLARE_DELEGATE_RetVal_OneParam(TSharedRef<SWidget>, FIdentityPromotedFrameGetContentMenu, int32 InIndex)

/**
 * Represents a given frame/angle promotion in the Promoted Frames panel
 */
class SPromotedFrameButton :
	public SButton
{
public:
	SLATE_BEGIN_ARGS(SPromotedFrameButton) {}

		/** The Promoted Frame this button is responsible for */
		SLATE_ARGUMENT(UMetaHumanIdentityPromotedFrame*, PromotedFrame)

		/** The index of this Promoted Frame. Used to create a label for the UI */
		SLATE_ARGUMENT(int32, Index)

		/** Event used to query the current selected frame */
		SLATE_EVENT(FIdentityPromotedFrameGetSelectedIndex, OnIdentityPromotedFrameGetSelectedIndex)

		/** Delegate called when the button is clicked */
		SLATE_EVENT(FIdentityPromotedFrameClicked, OnIdentityPromotedFrameClicked)

		/** Delegate called when the context menu is requested for the button, i.e., right-clicking in it */
		SLATE_EVENT(FIdentityPromotedFrameGetContentMenu, OnIdentityPromotedFrameGetContentMenu)

	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs)
	{
		PromotedFrame = InArgs._PromotedFrame;
		PromotedFrameIndex = InArgs._Index;

		OnIdentityPromotedFrameGetSelectedIndexDelegate = InArgs._OnIdentityPromotedFrameGetSelectedIndex;
		OnIdentityPromotedFrameClickedDelegate = InArgs._OnIdentityPromotedFrameClicked;
		OnIdentityPromotedFrameGetContextMenuDelegate = InArgs._OnIdentityPromotedFrameGetContentMenu;

		check(PromotedFrame.IsValid());
		check(PromotedFrameIndex != INDEX_NONE);
		check(OnIdentityPromotedFrameGetSelectedIndexDelegate.IsBound());
		check(OnIdentityPromotedFrameClickedDelegate.IsBound());

		// Call the SButton's Construct to customize the behaviour of the Promoted Frame button
		SButton::Construct(SButton::FArguments()
			.HAlign(HAlign_Fill)
			.VAlign(VAlign_Center)
			.ButtonColorAndOpacity(this, &SPromotedFrameButton::GetButtonColor)
			.OnClicked(this, &SPromotedFrameButton::HandlePromotedFrameClicked)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
				.Padding(0.5f)
				.HAlign(HAlign_Center)
				.VAlign(VAlign_Center)
				[
					SAssignNew(PromotedFrameLabel, SInlineEditableTextBlock)
					.Text(this, &SPromotedFrameButton::GetPromotedFrameLabel)
					.OnTextCommitted(this, &SPromotedFrameButton::HandlePromotedFrameLabelComited)
				]
				+SHorizontalBox::Slot()
				.Padding(2.0f)
				.HAlign(HAlign_Right)
				.VAlign(VAlign_Center)
				.AutoWidth()
				[
					SNew(SImage)
					.Image(this, &SPromotedFrameButton::GetButtonLockIcon)
					.ColorAndOpacity(FSlateColor::UseForeground())
				]
			]
		);
	}

	virtual FReply OnMouseButtonDoubleClick(const FGeometry& InMyGeometry, const FPointerEvent& InMouseEvent) override
	{
		if (PromotedFrameLabel.IsValid())
		{
			if (InMouseEvent.GetEffectingButton() == EKeys::LeftMouseButton)
			{
				PromotedFrameLabel->EnterEditingMode();

				return FReply::Handled();
			}
		}

		return SButton::OnMouseButtonDoubleClick(InMyGeometry, InMouseEvent);
	}

	virtual FReply OnMouseButtonUp(const FGeometry& InMyGeometry, const FPointerEvent& InMouseEvent) override
	{
		if (InMouseEvent.GetEffectingButton() == EKeys::RightMouseButton)
		{
			if (OnIdentityPromotedFrameGetContextMenuDelegate.IsBound())
			{
				TSharedRef<SWidget> ContextMenuContents = OnIdentityPromotedFrameGetContextMenuDelegate.Execute(PromotedFrameIndex);
				FWidgetPath WidgetPath = InMouseEvent.GetEventPath() != nullptr ? *InMouseEvent.GetEventPath() : FWidgetPath{};
				FSlateApplication::Get().PushMenu(AsShared(), WidgetPath, ContextMenuContents, InMouseEvent.GetScreenSpacePosition(), FPopupTransitionEffect(FPopupTransitionEffect::ContextMenu));
				return FReply::Handled();
			}
		}

		return SButton::OnMouseButtonUp(InMyGeometry, InMouseEvent);
	}

	void SetPromotedFrameIndex(int32 InNewIndex)
	{
		if (PromotedFrameIndex != InNewIndex)
		{
			PromotedFrameIndex = InNewIndex;
		}
	}

private:

	bool IsSelected() const
	{
		const int32 SelectedIndex = OnIdentityPromotedFrameGetSelectedIndexDelegate.Execute();
		return SelectedIndex != INDEX_NONE && SelectedIndex == PromotedFrameIndex;
	}

	FSlateColor GetButtonColor() const
	{
		const FName SelectedColor = IsSelected() ? TEXT("SelectionColor") : TEXT("Colors.Foreground");
		return FAppStyle::Get().GetSlateColor(SelectedColor);
	}

	const FSlateBrush* GetButtonLockIcon() const
	{
		if (PromotedFrame.IsValid())
		{
			if (PromotedFrame->IsNavigationLocked())
			{
				return FAppStyle::Get().GetBrush("Icons.Lock");
			}
			else
			{
				return FAppStyle::Get().GetBrush("Icons.Unlock");
			}
		}

		return FAppStyle::GetNoBrush();
	}

	FText GetPromotedFrameLabel() const
	{
		if (PromotedFrame.IsValid())
		{
			if (!PromotedFrame->FrameName.IsEmptyOrWhitespace())
			{
				return PromotedFrame->FrameName;
			}
		}

		return FText::FromString(TEXT("Frame ") + FString::FromInt(PromotedFrameIndex));
	}

	FReply HandlePromotedFrameClicked() const
	{
		if (PromotedFrame.IsValid())
		{
			OnIdentityPromotedFrameClickedDelegate.ExecuteIfBound(PromotedFrame.Get(), PromotedFrameIndex);
		}
		return FReply::Handled();
	}

	void HandlePromotedFrameLabelComited(const FText& InNewText, ETextCommit::Type /*InCommitInfo*/)
	{
		if (PromotedFrame.IsValid())
		{
			const FScopedTransaction Transaction(LOCTEXT("EditFrameNameTransactionLabel", "Edit Frame Name"));
			PromotedFrame->Modify();
			PromotedFrame->FrameName = InNewText;
		}
	}

private:

	/** Delegate called when a Promoted Frame button is clicked */
	FIdentityPromotedFrameClicked OnIdentityPromotedFrameClickedDelegate;

	/** Delegate called to obtain the current selected index. Used to highlight this button if this is the one selected */
	FIdentityPromotedFrameGetSelectedIndex OnIdentityPromotedFrameGetSelectedIndexDelegate;

	/** Delegated called to obtain the context menu to show when right-clicking in the button */
	FIdentityPromotedFrameGetContentMenu OnIdentityPromotedFrameGetContextMenuDelegate;

	/** The index for this Promoted Frame button */
	int32 PromotedFrameIndex;

	/** A weak reference to the Promoted Frame associated with this widget */
	TWeakObjectPtr<UMetaHumanIdentityPromotedFrame> PromotedFrame;

	/** A reference to the label displayed in the button used to trigger the text edit event */
	TSharedPtr<SInlineEditableTextBlock> PromotedFrameLabel;
};

/////////////////////////////////////////////////////
// SMetaHumanIdentityPromotedFramesEditor

const TCHAR* SMetaHumanIdentityPromotedFramesEditor::PromotedFramesTransactionContext = TEXT("IdentityTransaction");

void SMetaHumanIdentityPromotedFramesEditor::Construct(const FArguments& InArgs)
{
	ViewportClient = InArgs._ViewportClient;
	OnPromotedFrameSelectionChangedDelegate = InArgs._OnPromotedFrameSelectionChanged;
	OnPromotedFrameAddedDelegate = InArgs._OnPromotedFrameAdded;
	OnPromotedFrameRemovedDelegate = InArgs._OnPromotedFrameRemoved;
	OnPromotedFrameNavigationLockedChangedDelegate = InArgs._OnPromotedFrameNavigationLockedChanged;
	OnPromotedFrameTrackingModeChangedDelegate = InArgs._OnPromotedFrameTrackingModeChanged;
	OnUpdateControlVerticesPostUndoDelegate = InArgs._OnUpdateControlVerticesPostUndo;
	CommandList = InArgs._CommandList;

	check(CommandList);

	BindCommands();

	ChildSlot
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot()
		.VAlign(VAlign_Center)
		.AutoWidth()
		[
			SNew(SButton)
			.ButtonStyle(FAppStyle::Get(), "DetailsView.NameAreaButton")
			.OnClicked_Lambda([this]
			{
				CommandList->TryExecuteAction(FMetaHumanIdentityEditorCommands::Get().PromoteFrame.ToSharedRef());
				return FReply::Handled();
			})
			.IsEnabled(this, &SMetaHumanIdentityPromotedFramesEditor::CanAddPromotedFrame)
			[
				SNew(SImage)
				.Image(FMetaHumanIdentityEditorCommands::Get().PromoteFrame->GetIcon().GetIcon())
				.ColorAndOpacity(FSlateColor::UseForeground())
			]
		]
		+ SHorizontalBox::Slot()
		.VAlign(VAlign_Center)
		.AutoWidth()
		[
			SNew(SButton)
			.ButtonStyle(FAppStyle::Get(), "DetailsView.NameAreaButton")
			.HAlign(HAlign_Center)
			.VAlign(VAlign_Center)
			.OnClicked_Lambda([this]
			{
				CommandList->TryExecuteAction(FMetaHumanIdentityEditorCommands::Get().DemoteFrame.ToSharedRef());
				return FReply::Handled();
			})
			.IsEnabled(this, &SMetaHumanIdentityPromotedFramesEditor::IsSelectionValid)
			[
				SNew(SImage)
				.Image(FMetaHumanIdentityEditorCommands::Get().DemoteFrame->GetIcon().GetIcon())
			]
		]
		+ SHorizontalBox::Slot()
		.VAlign(VAlign_Center)
		.Padding(10.0f, 0.0f)
		[
			// TODO: A simple HorizontalBox might not be the best container for a large number of Promoted Frame. Look for a better solution
			SAssignNew(PromotedFramesContainer, SHorizontalBox)
		]
		+ SHorizontalBox::Slot()
		.VAlign(VAlign_Center)
		.AutoWidth()
		[
			SNew(SButton)
			.ButtonStyle(FAppStyle::Get(), "DetailsView.NameAreaButton")
			.ButtonColorAndOpacity_Lambda([this]
			{
				const FName SelectedColor = IsSelectionValid() ? TEXT("Colors.Foreground") : TEXT("SelectionColor");
				return FAppStyle::Get().GetSlateColor(SelectedColor);
			})
			.OnClicked_Lambda([this]
			{
				ClearSelection();
				return FReply::Handled();
			})
			[
				SNew(SImage)
				.ColorAndOpacity(FSlateColor::UseForeground())
				.Image(FMetaHumanIdentityStyle::Get().GetBrush("Identity.Frame.FreeRoam"))
			]
		]
	];

	if (InArgs._IdentityPose.IsValid())
	{
		SetIdentityPose(InArgs._IdentityPose.Get());
	}

	ViewportClient->OnCameraMoved().BindSP(this, &SMetaHumanIdentityPromotedFramesEditor::HandleViewportCameraMoved);
	ViewportClient->OnCameraStopped().AddSP(this, &SMetaHumanIdentityPromotedFramesEditor::HandleViewportCameraStopped);
}

SMetaHumanIdentityPromotedFramesEditor::~SMetaHumanIdentityPromotedFramesEditor()
{
	// Empty destructor is required here as there is a TUniquePtr<class FScopedTransaction> in this class.
}

void SMetaHumanIdentityPromotedFramesEditor::SetIdentityPose(UMetaHumanIdentityPose* InPose)
{
	if (IdentityPose != InPose)
	{
		RemoveAllPromotedFrameButtons();

		if (IdentityPose != nullptr)
		{
			// Unbind the CaptureDataChanged delegate from the previous Identity pose
			IdentityPose->OnCaptureDataChanged().RemoveAll(this);
		}

		// InPose can be nullptr to indicate no pose is being edited
		IdentityPose = InPose;

		if (IdentityPose != nullptr)
		{
			IdentityPose->OnCaptureDataChanged().AddSP(this, &SMetaHumanIdentityPromotedFramesEditor::HandleIdentityPoseCaptureDataChanged);

			for (UMetaHumanIdentityPromotedFrame* PromotedFrame : IdentityPose->PromotedFrames)
			{
				RegisterPromotedFrameCameraTransformChange(PromotedFrame);
			}
		}

		AddAllPromotedFrameButtons();
	}
}

UMetaHumanIdentityPose* SMetaHumanIdentityPromotedFramesEditor::GetIdentityPose() const
{
	if (IdentityPose.IsValid())
	{
		return IdentityPose.Get();
	}

	return nullptr;
}

UMetaHumanIdentityPromotedFrame* SMetaHumanIdentityPromotedFramesEditor::GetSelectedPromotedFrame() const
{
	if (IsSelectionValid())
	{
		return IdentityPose->PromotedFrames[SelectedPromotedFrameIndex];
	}

	return nullptr;
}

void SMetaHumanIdentityPromotedFramesEditor::PostUndo(bool bInSuccess)
{
	if (bInSuccess)
	{
		// Get the selection state before recreating the promoted frame buttons
		UMetaHumanIdentityPromotedFrame* SelectedPromotedFrame = GetSelectedPromotedFrame();

		//Check to see if the number of promoted frames changed, if it did, recreate all the buttons
		bool bRecreateButtons = IdentityPose.IsValid() && PromotedFramesContainer->NumSlots() != IdentityPose->PromotedFrames.Num();
		
		if(!UndoControlVertexManipulation(SelectedPromotedFrame, false) && bRecreateButtons)
		{
			RecreatePromotedFrameButtonsForUndoRedo(SelectedPromotedFrame);
		}
	}
}

void SMetaHumanIdentityPromotedFramesEditor::PostRedo(bool bInSuccess)
{
	if (bInSuccess)
	{
		// Get the selection state before recreating the promoted frame buttons
		UMetaHumanIdentityPromotedFrame* SelectedPromotedFrame = GetSelectedPromotedFrame();

		//Check to see if the number of promoted frames changed, if it did, recreate all the buttons
		bool bRecreateButtons = IdentityPose.IsValid() && PromotedFramesContainer->NumSlots() != IdentityPose->PromotedFrames.Num();
		
		if(!UndoControlVertexManipulation(SelectedPromotedFrame, true) && bRecreateButtons)
		{
			RecreatePromotedFrameButtonsForUndoRedo(SelectedPromotedFrame);
		}
	}
}

void SMetaHumanIdentityPromotedFramesEditor::RecreatePromotedFrameButtonsForUndoRedo(const UMetaHumanIdentityPromotedFrame* InSelectedPromotedFrame)
{
	int32 PreviousFrameIndex = SelectedPromotedFrameIndex;
	
	if (IdentityPose.IsValid() && PreviousFrameIndex == IdentityPose->PromotedFrames.IndexOfByKey(InSelectedPromotedFrame))
	{
		RemoveAllPromotedFrameButtons();
		AddAllPromotedFrameButtons();
		
		SetSelection(PreviousFrameIndex);
	}
}

void SMetaHumanIdentityPromotedFramesEditor::BindCommands()
{
	const FMetaHumanIdentityEditorCommands& Commands = FMetaHumanIdentityEditorCommands::Get();

	CommandList->MapAction(Commands.PromoteFrame, FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityPromotedFramesEditor::HandleOnAddPromotedFrameClicked),
															FCanExecuteAction::CreateSP(this, &SMetaHumanIdentityPromotedFramesEditor::CanAddPromotedFrame)));

	CommandList->MapAction(Commands.DemoteFrame, FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityPromotedFramesEditor::HandleOnRemovePromotedFrameClicked),
															FCanExecuteAction::CreateSP(this, &SMetaHumanIdentityPromotedFramesEditor::IsSelectionValid)));

}

bool SMetaHumanIdentityPromotedFramesEditor::IsSelectionValid() const
{
	return (SelectedPromotedFrameIndex != INDEX_NONE) && (SelectedPromotedFrameIndex < PromotedFramesContainer->NumSlots());
}

void SMetaHumanIdentityPromotedFramesEditor::AddPromotedFrameButton(UMetaHumanIdentityPromotedFrame* InPromotedFrame, int32 InIndex)
{
	PromotedFramesContainer->InsertSlot(InIndex)
	[
		SNew(SPromotedFrameButton)
		.PromotedFrame(InPromotedFrame)
		.Index(InIndex)
		.OnIdentityPromotedFrameClicked(this, &SMetaHumanIdentityPromotedFramesEditor::HandlePromotedFrameButtonClicked)
		.OnIdentityPromotedFrameGetSelectedIndex_Lambda([this]{ return SelectedPromotedFrameIndex; })
		.OnIdentityPromotedFrameGetContentMenu(this, &SMetaHumanIdentityPromotedFramesEditor::GetPromotedFrameContextMenu)
	];
}

void SMetaHumanIdentityPromotedFramesEditor::RemovePromotedFrameButton(int32 InIndex)
{
	if ((InIndex != INDEX_NONE) && (InIndex < PromotedFramesContainer->NumSlots()))
	{
		PromotedFramesContainer->RemoveSlot(GetPromotedFrameButton(InIndex));

		// Update the index of the existing Promoted Frames in the widget
		for (int32 SlotIndex = SelectedPromotedFrameIndex; SlotIndex < PromotedFramesContainer->NumSlots(); ++SlotIndex)
		{
			// TODO: Need to adapt for the footage case. Should probably delegate this to the IdentityPose itself
			GetPromotedFrameButton(SlotIndex)->SetPromotedFrameIndex(SlotIndex);
		}
	}
}

void SMetaHumanIdentityPromotedFramesEditor::AddAllPromotedFrameButtons()
{
	if (IdentityPose.IsValid())
	{
		const TArray<UMetaHumanIdentityPromotedFrame*>& PromotedFrames = IdentityPose->PromotedFrames;
		for (int32 PromotedFrameIndex = 0; PromotedFrameIndex < PromotedFrames.Num(); ++PromotedFrameIndex)
		{
			if (UMetaHumanIdentityPromotedFrame* PromotedFrame = PromotedFrames[PromotedFrameIndex])
			{
				AddPromotedFrameButton(PromotedFrame, PromotedFrameIndex);
			}
			else
			{
				UE_LOG(LogMetaHumanIdentity, Error, TEXT("Trying to add invalid Promoted Frame of index %d for Pose '%s'"), PromotedFrameIndex, *IdentityPose->GetName());
			}
		}
	}
}

void SMetaHumanIdentityPromotedFramesEditor::SetSelection(int32 InIndex, bool bForceNotify)
{
	const bool bSelectionChanged = SelectedPromotedFrameIndex != InIndex;

	SelectedPromotedFrameIndex = InIndex;

	if (IsSelectionValid())
	{
		UMetaHumanIdentityPromotedFrame* PromotedFrame = GetSelectedPromotedFrame();
		if(!PromotedFrame->bIsNavigationLocked)
		{
			LoadCameraTransform(PromotedFrame);
		}
	}

	if (bSelectionChanged || bForceNotify)
	{
		OnPromotedFrameSelectionChangedDelegate.ExecuteIfBound(GetSelectedPromotedFrame(), bForceNotify);
	}
}

void SMetaHumanIdentityPromotedFramesEditor::RemoveAllPromotedFrameButtons()
{
	// Remove all the Promoted Frame buttons from the UI
	PromotedFramesContainer->ClearChildren();

	ClearSelection();
}

void SMetaHumanIdentityPromotedFramesEditor::ClearSelection()
{
	SetSelection(INDEX_NONE);
}

bool SMetaHumanIdentityPromotedFramesEditor::IsPoseValid() const
{
	return IdentityPose.IsValid() && (IdentityPose->PromotedFrameClass != nullptr);
}

bool SMetaHumanIdentityPromotedFramesEditor::CanAddPromotedFrame() const
{
	return IsPoseValid();
}

bool SMetaHumanIdentityPromotedFramesEditor::UndoControlVertexManipulation(const UMetaHumanIdentityPromotedFrame* InSelectedPromotedFrame, bool InIsRedo) const
{
	bool bUndoMarkerManipulation = false;
	
	if(InSelectedPromotedFrame)
	{
		int32 TransactionID = GEditor->Trans->GetQueueLength() - GEditor->Trans->GetUndoCount();
		TransactionID = InIsRedo ? TransactionID - 1 : TransactionID;
		
		const FTransaction* Transaction = GEditor->Trans->GetTransaction(TransactionID);
		const FTransactionDiff Diff = Transaction->GenerateDiff();
		
		for (const TPair<FName, TSharedPtr<FTransactionObjectEvent>>& DiffMapPair : Diff.DiffMap)
		{
			const TSharedPtr<FTransactionObjectEvent>& TransactionObjectEvent = DiffMapPair.Value;
			if (TransactionObjectEvent->HasPropertyChanges())
			{
				bUndoMarkerManipulation = TransactionObjectEvent->GetChangedProperties().Last() == GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityPromotedFrame, ContourData);
				bUndoMarkerManipulation &= TransactionObjectEvent->GetChangedProperties().Num() == 1;
			}
		}

		if(bUndoMarkerManipulation)
		{
			InSelectedPromotedFrame->UpdateShapeAnnotationMarkers();
			OnUpdateControlVerticesPostUndoDelegate.ExecuteIfBound();
		}
	}
	
	return bUndoMarkerManipulation;
}

TSharedRef<SPromotedFrameButton> SMetaHumanIdentityPromotedFramesEditor::GetPromotedFrameButton(int32 InIndex) const
{
	const int32 NumSlots = PromotedFramesContainer->NumSlots();
	check(0 <= InIndex && InIndex < NumSlots);
	const SHorizontalBox::FSlot& Slot = PromotedFramesContainer->GetSlot(InIndex);
	return StaticCastSharedRef<SPromotedFrameButton>(Slot.GetWidget());
}

TSharedRef<SWidget> SMetaHumanIdentityPromotedFramesEditor::GetPromotedFrameContextMenu(int32 InPromotedFrameIndex) const
{
	const FMetaHumanIdentityEditorCommands& Commands = FMetaHumanIdentityEditorCommands::Get();

	const bool bShouldCloseWindowAfterMenuSelection = true;
	FMenuBuilder MenuBuilder{ bShouldCloseWindowAfterMenuSelection, CommandList };

	if (InPromotedFrameIndex != INDEX_NONE && InPromotedFrameIndex < IdentityPose->PromotedFrames.Num())
	{
		UMetaHumanIdentityPromotedFrame* PromotedFrame = IdentityPose->PromotedFrames[InPromotedFrameIndex];

		MenuBuilder.BeginSection(TEXT("PromotedFrameTrackingMode"), LOCTEXT("PromotedFrameTrackingModeMenuSection", "Tracking Mode"));
		{
			MenuBuilder.AddMenuEntry(LOCTEXT("TrackOnChangeLabel", "Autotracking On"),
									 LOCTEXT("TrackOnChangeTooltip", "Run face tracker when the camera stops moving"),
									 FSlateIcon{},
									 FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityPromotedFramesEditor::HandlePromotedFrameTrackingModeChanged, PromotedFrame, true),
											   FCanExecuteAction::CreateLambda([PromotedFrame] { return !PromotedFrame->IsNavigationLocked(); }),
											   FIsActionChecked::CreateUObject(PromotedFrame, &UMetaHumanIdentityPromotedFrame::IsTrackingOnChange)),
									 NAME_None,
									 EUserInterfaceActionType::RadioButton);

			MenuBuilder.AddMenuEntry(LOCTEXT("TrackManuallyLabel", "Autotracking Off"),
									 LOCTEXT("TrackManuallyTooltip", "Run the face tracker manually"),
									 FSlateIcon{},
									 FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityPromotedFramesEditor::HandlePromotedFrameTrackingModeChanged, PromotedFrame, false),
											   FCanExecuteAction::CreateLambda([PromotedFrame] { return !PromotedFrame->IsNavigationLocked(); }),
											   FIsActionChecked::CreateUObject(PromotedFrame, &UMetaHumanIdentityPromotedFrame::IsTrackingManually)),
									 NAME_None,
									 EUserInterfaceActionType::RadioButton);


			MenuBuilder.AddMenuEntry(LOCTEXT("LockNavigationLabel", "Lock Camera"),
									 LOCTEXT("LockNavigationTooltip", "Locks the camera navigation for this frame and switches to a 2D navigation mode"),
									 FSlateIcon{},
									 FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityPromotedFramesEditor::HandlePromotedFrameToggleNavigationLocked, PromotedFrame),
											   FCanExecuteAction{},
											   FIsActionChecked::CreateUObject(PromotedFrame, &UMetaHumanIdentityPromotedFrame::IsNavigationLocked)),
									 NAME_None,
									 EUserInterfaceActionType::ToggleButton);
		}
		MenuBuilder.EndSection();
	}

	MenuBuilder.BeginSection(TEXT("PromotedFrameCommands"), LOCTEXT("PromotedFrameCommandsMenuSection", "Commands"));
	{
		MenuBuilder.AddMenuEntry(Commands.TrackCurrent);
	}
	MenuBuilder.EndSection();

	return MenuBuilder.MakeWidget();
}

void SMetaHumanIdentityPromotedFramesEditor::HandleOnAddPromotedFrameClicked()
{
	if (IsPoseValid())
	{
		if (UMetaHumanIdentityPromotedFrame* PromotedFrame = NewObject<UMetaHumanIdentityPromotedFrame>(IdentityPose.Get(), IdentityPose->PromotedFrameClass, NAME_None, RF_Transactional))
		{
			const FScopedTransaction Transaction(PromotedFramesTransactionContext, LOCTEXT("AddPromotedFrame", "Promote Frame"), IdentityPose.Get());

			IdentityPose->Modify();

			StoreCameraTransform(PromotedFrame);

			RegisterPromotedFrameCameraTransformChange(PromotedFrame);

			// Use the default tracker set in the Pose itself
			PromotedFrame->ContourTracker = IdentityPose->DefaultTracker;

			// Add the Promoted Frame to the Pose
			const int32 PromotedFrameIndex = IdentityPose->PromotedFrames.Add(PromotedFrame);
			AddPromotedFrameButton(PromotedFrame, PromotedFrameIndex);

			// Signal to the toolkit that a new frame is added, so it can initialize its Curve states
			OnPromotedFrameAddedDelegate.ExecuteIfBound(PromotedFrame);

			// Select the newly added Promoted Frame
			SetSelection(PromotedFrameIndex);
		}
		else
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error creating new Promoted Frame for Pose '%s'"), *IdentityPose->GetName());
		}
	}
}

void SMetaHumanIdentityPromotedFramesEditor::HandleOnRemovePromotedFrameClicked()
{
	if (IsSelectionValid() && IsPoseValid() && SelectedPromotedFrameIndex < IdentityPose->PromotedFrames.Num())
	{
		const FScopedTransaction Transaction(PromotedFramesTransactionContext, LOCTEXT("RemotePromotedFrame", "Remove Promoted Frame"), IdentityPose.Get());

		IdentityPose->Modify();

		int32 CurrentPromotedFrameIndex = SelectedPromotedFrameIndex;

		RemovePromotedFrameButton(CurrentPromotedFrameIndex);

		UMetaHumanIdentityPromotedFrame* PromotedFrame = IdentityPose->PromotedFrames[CurrentPromotedFrameIndex];

		IdentityPose->PromotedFrames.Remove(PromotedFrame);

		// Signal to the toolkit that a Promoted frame has been removed from the pose
		OnPromotedFrameRemovedDelegate.ExecuteIfBound(PromotedFrame);

		const int32 NumPromotedFrames = IdentityPose->PromotedFrames.Num();
		if (NumPromotedFrames >= 0)
		{
			// Select the previous Promoted Frame in the list or keep the current index if valid
			if (CurrentPromotedFrameIndex >= NumPromotedFrames)
			{
				CurrentPromotedFrameIndex = NumPromotedFrames - 1;
			}

			SetSelection(CurrentPromotedFrameIndex, true);
		}
		else
		{
			ClearSelection();
		}
	}
}

void SMetaHumanIdentityPromotedFramesEditor::HandlePromotedFrameButtonClicked(UMetaHumanIdentityPromotedFrame* InPromotedFrame, int32 InIndex)
{
	if (SelectedPromotedFrameIndex != InIndex)
	{
		SetSelection(InIndex, true);
	}
}

void SMetaHumanIdentityPromotedFramesEditor::HandleIdentityPoseCaptureDataChanged()
{
	RemoveAllPromotedFrameButtons();
	AddAllPromotedFrameButtons();
}

void SMetaHumanIdentityPromotedFramesEditor::HandleViewportCameraMoved()
{
	if (IsSelectionValid())
	{
		UMetaHumanIdentityPromotedFrame* PromotedFrame = GetSelectedPromotedFrame();

		if (!ScopedTransaction.IsValid())
		{
			ScopedTransaction = MakeUnique<FScopedTransaction>(PromotedFramesTransactionContext, LOCTEXT("CameraMovedTransactionLabel", "Camera Moved"), PromotedFrame);
		}

		PromotedFrame->Modify();

		if (!PromotedFrame->IsNavigationLocked())
		{
			StoreCameraTransform(PromotedFrame);
		}
	}
}

void SMetaHumanIdentityPromotedFramesEditor::HandleViewportCameraStopped()
{
	if (ScopedTransaction.IsValid())
	{
		ScopedTransaction.Reset();
	}
}

void SMetaHumanIdentityPromotedFramesEditor::HandlePromotedFrameTrackingModeChanged(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, bool bInTrackOnChange) const
{
	if (InPromotedFrame != nullptr && InPromotedFrame->bTrackOnChange != bInTrackOnChange)
	{
		const FScopedTransaction Transaction(PromotedFramesTransactionContext, LOCTEXT("EditTrackOnChangePromotedFrameTransasction", "Edit Track On Change"), IdentityPose.Get());

		const bool bMarkDirty = false;
		InPromotedFrame->Modify(bMarkDirty);

		InPromotedFrame->bTrackOnChange = bInTrackOnChange;

		OnPromotedFrameTrackingModeChangedDelegate.ExecuteIfBound(InPromotedFrame);
	}
}

void SMetaHumanIdentityPromotedFramesEditor::HandlePromotedFrameCameraTransformChanged(UMetaHumanIdentityPromotedFrame* InPromotedFrame) const
{
	if (InPromotedFrame != nullptr && InPromotedFrame == GetSelectedPromotedFrame())
	{
		LoadCameraTransform(InPromotedFrame);
	}
}

void SMetaHumanIdentityPromotedFramesEditor::HandlePromotedFrameToggleNavigationLocked(UMetaHumanIdentityPromotedFrame* InPromotedFrame) const
{
	if (InPromotedFrame != nullptr)
	{
		const FScopedTransaction Transaction(PromotedFramesTransactionContext, LOCTEXT("EditNavigationIsLockedTransaction", "Edit Is Navigation Locked"), IdentityPose.Get());

		InPromotedFrame->Modify();

		InPromotedFrame->ToggleNavigationLocked();

		OnPromotedFrameNavigationLockedChangedDelegate.ExecuteIfBound(InPromotedFrame);
	}
}

void SMetaHumanIdentityPromotedFramesEditor::RegisterPromotedFrameCameraTransformChange(UMetaHumanIdentityPromotedFrame* InPromotedFrame) const
{
	if (InPromotedFrame != nullptr)
	{
		if (UMetaHumanIdentityCameraFrame* CameraFrame = Cast<UMetaHumanIdentityCameraFrame>(InPromotedFrame))
		{
			CameraFrame->OnCameraTransformChanged().BindSP(this, &SMetaHumanIdentityPromotedFramesEditor::HandlePromotedFrameCameraTransformChanged, InPromotedFrame);
		}
	}
}

void SMetaHumanIdentityPromotedFramesEditor::StoreCameraTransform(class UMetaHumanIdentityPromotedFrame* InPromotedFrame) const
{
	if (UMetaHumanIdentityCameraFrame* CameraFrame = Cast<UMetaHumanIdentityCameraFrame>(InPromotedFrame))
	{
		// Need to disable orbit camera before setting actor position so that the viewport camera location is converted back
		ViewportClient->ToggleOrbitCamera(false);

		CameraFrame->ViewLocation = ViewportClient->GetViewLocation();
		CameraFrame->ViewRotation = ViewportClient->GetViewRotation();
		CameraFrame->LookAtLocation = ViewportClient->GetLookAtLocation();
		CameraFrame->CameraViewFOV = ViewportClient->ViewFOV;
	}
}

void SMetaHumanIdentityPromotedFramesEditor::LoadCameraTransform(class UMetaHumanIdentityPromotedFrame* InPromotedFrame) const
{
	if (UMetaHumanIdentityCameraFrame* CameraFrame = Cast<UMetaHumanIdentityCameraFrame>(InPromotedFrame))
	{
		ViewportClient->SetLookAtLocation(CameraFrame->LookAtLocation);
		ViewportClient->SetViewLocation(CameraFrame->ViewLocation);
		ViewportClient->SetViewRotation(CameraFrame->ViewRotation);
		ViewportClient->ViewFOV = CameraFrame->CameraViewFOV;
	}
}

#undef LOCTEXT_NAMESPACE