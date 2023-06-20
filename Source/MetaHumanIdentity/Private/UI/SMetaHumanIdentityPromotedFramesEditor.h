// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "EditorUndoClient.h"
#include "Framework/Commands/UICommandList.h"

DECLARE_DELEGATE_TwoParams(FOnPromotedFrameSelectionChanged, class UMetaHumanIdentityPromotedFrame* InPromotedFrame, bool bForceNotify)
DECLARE_DELEGATE_OneParam(FOnPromotedFrameTrackingModeChanged, class UMetaHumanIdentityPromotedFrame* InPromotedFrame)
DECLARE_DELEGATE_OneParam(FOnPromotedFrameNavigationLockedChanged, class UMetaHumanIdentityPromotedFrame* InPromotedFrame)
DECLARE_DELEGATE_OneParam(FOnPromotedFrameAdded, class UMetaHumanIdentityPromotedFrame*)
DECLARE_DELEGATE_OneParam(FOnPromotedFrameRemoved, class UMetaHumanIdentityPromotedFrame* InPromotedFrame)

/**
 * The Promoted Frames editor is responsible for creating new Promoted Frames and displaying a UI that allows the user to select it.
 * Each Promoted Frame is displayed as a custom SButton that can be selected by clicking. There are also buttons to promote
 * a new frame and remove the selected one.
 * The Promoted Frames editor also handles undo/redo events and updates the UI accordingly.
 */
class SMetaHumanIdentityPromotedFramesEditor
	: public SCompoundWidget
	, public FSelfRegisteringEditorUndoClient
{
public:
	SLATE_BEGIN_ARGS(SMetaHumanIdentityPromotedFramesEditor) {}

		// A reference to the viewport client used to read the camera transform for a given promoted frame
		SLATE_ARGUMENT(TSharedPtr<class FConformingViewportClient>, ViewportClient)

		// The initial Identity Pose to edit Promoted Frames for
		SLATE_ARGUMENT(TWeakObjectPtr<class UMetaHumanIdentityPose>, IdentityPose)

		// The command list with mapped actions that can be executed by this editor
		SLATE_ARGUMENT(TSharedPtr<FUICommandList>, CommandList)

		// Delegate called when the current selected Promoted Frame changes
		SLATE_EVENT(FOnPromotedFrameSelectionChanged, OnPromotedFrameSelectionChanged)

		// Delegate called when the navigation locked state of a Promoted Frame changes
		SLATE_EVENT(FOnPromotedFrameNavigationLockedChanged, OnPromotedFrameNavigationLockedChanged)

		// Delegated called when the tracking mode of a Promoted Frame changes
		SLATE_EVENT(FOnPromotedFrameTrackingModeChanged, OnPromotedFrameTrackingModeChanged)

		// Delegate called when adding a new Promoted Frame
		SLATE_EVENT(FOnPromotedFrameAdded, OnPromotedFrameAdded)

		// Delegate called when removing a Promoted Frame
		SLATE_EVENT(FOnPromotedFrameRemoved, OnPromotedFrameRemoved)

		// Delegate called when Post Undo is called for marker manipulation
		SLATE_EVENT(FSimpleDelegate, OnUpdateControlVerticesPostUndo)

	SLATE_END_ARGS()

	~SMetaHumanIdentityPromotedFramesEditor();

	void Construct(const FArguments& InArgs);

	/** Sets a Identity Pose to be edited by this widget */
	void SetIdentityPose(class UMetaHumanIdentityPose* InPose);

	/** Returns the current pose being edited */
	class UMetaHumanIdentityPose* GetIdentityPose() const;

	/** Returns the current selected Promoted Frame or nullptr if there isn't one selected */
	class UMetaHumanIdentityPromotedFrame* GetSelectedPromotedFrame() const;

	/** Set the current selection to the the one pointed by the InIndex
	  * called by Outliner when selecting a frame in the Outliner's Frames Panel */
	void SetSelection(int32 InIndex, bool bInForceNotify = false);

public:

	// FEditorUndoClient Interface
	virtual void PostUndo(bool bInSuccess) override;
	virtual void PostRedo(bool bInSuccess) override;
	
	void RecreatePromotedFrameButtonsForUndoRedo(const UMetaHumanIdentityPromotedFrame* InSelectedPromotedFrame);

private:
	/** Bind commands to actions that are specific for handling promoted frames */
	void BindCommands();

	/** Returns true iff the current selection is a valid index */
	bool IsSelectionValid() const;

	/** Adds a Promoted Frame button at the given index */
	void AddPromotedFrameButton(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, int32 InIndex);

	/** Removes an Promoted Frame button at the given index. Does nothing if the index is not in valid */
	void RemovePromotedFrameButton(int32 InIndex);

	/** Adds all Promoted Frame buttons to the UI */
	void AddAllPromotedFrameButtons();

	/** Remove all Promoted Frame buttons from the UI. This will also clear the current selection, if there is one */
	void RemoveAllPromotedFrameButtons();

	/** Clears the selection */
	void ClearSelection();

	/** Returns true if we are currently editing a valid Identity Pose with a valid Promoted Frame Class that can be instantiated */
	bool IsPoseValid() const;

	/** Returns true if a new Promoted Frame can be added for the Pose being edited */
	bool CanAddPromotedFrame() const;

	/** Returns true if undo operation involved reverting control vertex manipulation only */
	bool UndoControlVertexManipulation(const UMetaHumanIdentityPromotedFrame* InSelectedPromotedFrame, bool InIsRedo) const;

	/** Returns a reference to the Promoted Frame button at the given index */
	TSharedRef<class SPromotedFrameButton> GetPromotedFrameButton(int32 InIndex) const;

	/** Creates the context menu for the promoted frame of the given index */
	TSharedRef<SWidget> GetPromotedFrameContextMenu(int32 InPromotedFrameIndex) const;

	/** Called when the promote button is clicked */
	void HandleOnAddPromotedFrameClicked();

	/** Called when the unpromote button is clicked */
	void HandleOnRemovePromotedFrameClicked();

	/** Called when one Promoted Frame button is clicked in the UI */
	void HandlePromotedFrameButtonClicked(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, int32 InIndex);

	/** Called when the capture source of the pose changes externally */
	void HandleIdentityPoseCaptureDataChanged();

	/** Called when the camera in the viewport moves. Used to update the camera transform of the selected promoted frame */
	void HandleViewportCameraMoved();

	/** Called when the camera in the viewport stops moving. Used to commit the transaction holding camera transformation changes */
	void HandleViewportCameraStopped();

	/** Handles a change in the tracking mode of a Promoted Frame */
	void HandlePromotedFrameTrackingModeChanged(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, bool bInTrackOnChange) const;

	/** Updates the viewport camera when the Promoted Frame camera transform changes and this is the currently selected Promoted Frame */
	void HandlePromotedFrameCameraTransformChanged(class UMetaHumanIdentityPromotedFrame* InPromotedFrame) const;

	/** Handles changes in the navigation locked state of the given Promoted Frame */
	void HandlePromotedFrameToggleNavigationLocked(class UMetaHumanIdentityPromotedFrame* InPromotedFrame) const;

	/** Registers the HandlePromotedFrameCameraTransformChanged as a delegate called when the camera transform changes in the Promoted Frame */
	void RegisterPromotedFrameCameraTransformChange(class UMetaHumanIdentityPromotedFrame* InPromotedFrame) const;

	/** Stores the current camera transform in the given Promoted Frame */
	void StoreCameraTransform(class UMetaHumanIdentityPromotedFrame* InPromotedFrame) const;

	/** Loads the camera transform stored in the Promoted Frame and sets it in the viewport */
	void LoadCameraTransform(class UMetaHumanIdentityPromotedFrame* InPromotedFrame) const;

private:
	/** The transaction context identifier for transactions done in the Identity Pose being edited */
	static const TCHAR* PromotedFramesTransactionContext;

	// The transaction used to track changes in the camera transform for a given promoted frame
	TUniquePtr<class FScopedTransaction> ScopedTransaction;

	/** Delegate called when the selection state of a Promoted Frame changes */
	FOnPromotedFrameSelectionChanged OnPromotedFrameSelectionChangedDelegate;

	/** Delegate called when the add promoted frame button is clicked */
	FOnPromotedFrameAdded OnPromotedFrameAddedDelegate;

	/** Delegate called when a promoted frame is removed from the pose */
	FOnPromotedFrameRemoved OnPromotedFrameRemovedDelegate;

	/** Delegate called when the navigation locked state of a Promoted Frame changes */
	FOnPromotedFrameNavigationLockedChanged OnPromotedFrameNavigationLockedChangedDelegate;

	/** Delegate called when the tracking mode for a promoted frame changes */
	FOnPromotedFrameTrackingModeChanged OnPromotedFrameTrackingModeChangedDelegate;

	FSimpleDelegate OnUpdateControlVerticesPostUndoDelegate;

	/** The index of the currently selected Promoted Frame */
	int32 SelectedPromotedFrameIndex = INDEX_NONE;

	/** A reference to the pose being edited */
	TWeakObjectPtr<class UMetaHumanIdentityPose> IdentityPose;

	/** A reference to the viewport client where the pose components are being displayed */
	TSharedPtr<class FConformingViewportClient> ViewportClient;

	/** A container for adding new Promoted Frames */
	TSharedPtr<class SHorizontalBox> PromotedFramesContainer;

	/** The command list with actions associated with this editor */
	TSharedPtr<FUICommandList> CommandList;
};