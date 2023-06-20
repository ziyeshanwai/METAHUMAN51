// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "MetaHumanIdentityPromotedFrames.h"
#include "Tools/BaseAssetToolkit.h"
#include "Misc/NotifyHook.h"
#include "Widgets/Views/STreeView.h"
#include "UI/MetaHumanIdentityMarkerData.h"

#include "Pipeline/Pipeline.h"
#include "Pipeline/PipelineData.h"


class FMetaHumanIdentityAssetEditorToolkit
	: public FBaseAssetToolkit
	, public FNotifyHook
{
public:
	FMetaHumanIdentityAssetEditorToolkit(UAssetEditor* InOwningAssetEditor);
	~FMetaHumanIdentityAssetEditorToolkit();

public:
	//~ FAssetEditorToolkit interface
	virtual FName GetToolkitFName() const override;
	virtual FText GetBaseToolkitName() const override;
	virtual FText GetToolkitToolTipText() const override;
	virtual FString GetWorldCentricTabPrefix() const override;
	virtual FLinearColor GetWorldCentricTabColorScale() const override;
	virtual bool IsPrimaryEditor() const override;
	virtual void CreateWidgets() override;
	virtual void RegisterTabSpawners(const TSharedRef<class FTabManager>& InTabManager) override;
	virtual void UnregisterTabSpawners(const TSharedRef<class FTabManager>& InTabManager) override;

	//~ FNotifyHook interface
	virtual void NotifyPostChange(const FPropertyChangedEvent& InPropertyChangedEvent, FProperty* InPropertyThatChanged) override;

protected:
	//~ FBaseAssetToolkit interface
	virtual AssetEditorViewportFactoryFunction GetViewportDelegate() override;
	virtual TSharedPtr<FEditorViewportClient> CreateEditorViewportClient() const override;
	virtual void PostInitAssetEditor() override;

private:
	TSharedRef<SDockTab> SpawnPartsTab(const FSpawnTabArgs& InArgs);
	TSharedRef<SDockTab> SpawnOutlinerTab(const FSpawnTabArgs& InArgs);

	/** Create the scene capture component used to take screenshots for tracking */
	void CreateSceneCaptureComponent();

	/** Bind the Identity commands to actions performed by the toolkit */
	void BindCommands();

	/** Extend the editor's main menu with custom entries */
	void ExtendMenu();

	/** Extend the editor's toolbar with custom entries */
	void ExtendToolBar();

	/** Updates which objects is being displayed in the details panel */
	void HandleIdentityTreeSelectionChanged(UObject* InObject);

	/** Called when the tracking mode of a promoted frame changes */
	void HandlePromotedFrameTrackingModeChanged(class UMetaHumanIdentityPromotedFrame* InPromotedFrame);

	/** Called when undo/redo operation affects marker manipulation */
	void HandleUndoForMarkerManipulation();

	/** Called when the navigation mode of a prmoted frame changes */
	void HandlePromotedFrameNavigationLockedChanged(class UMetaHumanIdentityPromotedFrame* InPromotedFrame);

	/** Handles when a Promoted Frame is selected in the Promoted Frames panel */
	void HandlePromotedFrameSelectedInPromotedFramesPanel(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, bool bForceNotify) const;

	/** Handles when a Promoted Frame is selected in the Outliner */
	void HandlePromotedFrameSelectedInOutliner(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, bool bForceNotify) const;

	/** */
	void HandleCurveSelectionChanged(const TArray<FString>& InSelectedCurves) const;

	/** Refreshes the Viewport regardless of where the PromotedFrame selection change came */
	void RefreshViewportDueToFrameSelectionChange(class UMetaHumanIdentityPromotedFrame* InPromotedFrame) const;

	/** Handle to a newly created promoted frame to initialize curve and group data */
	void HandlePromotedFrameAdded(class UMetaHumanIdentityPromotedFrame* InPromotedFrame) const;

	/** Called when the camera in the viewport stops moving. Used for retracking in case track on change is enabled */
	void HandleCameraStopped();

	/** Handle curve selection coming for viewport image viewer */
	void HandleSelectionFromViewport(const TArray<FString>& InSelectedCurveNames) const;

	/** Process the array of log messages and display a dialog if there is an error message there */
	void HandleLogErrorMessage(ELogVerbosity::Type InLogVerbosity, const FString& InMessage);

	void HandleTrackCurrent();
	void HandleConform();
	void HandleResetTemplateMesh();

	void HandleSubmitToAutoRigging();
	void AutoRigProgressEnd(bool bSuccess) const;

	bool CanActivateMarkersForCurrent() const;
	bool CanActivateMarkersForAll() const;
	bool CanTrackCurrent() const;
	bool CanTrackAll() const;
	bool CanConform() const;
	bool CanResetTemplateMesh() const;
	bool CanSubmitToAutoRigging() const;
	bool ActiveCurvesAreValidForConforming() const;

	/** Track the given Promoted Frame using the image data provided */
	void TrackPromotedFrame(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, const TArray<FColor>& InImageData);

	/** Captures the scene using SceneCaptureComponent and the camera transform from the given Promoted Frame */
	void CaptureSceneForPromotedFrame(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, bool bInCaptureForTracking = false) const;

	/** Sets the size of the render target used to capture the scene */
	void SetCaptureSceneRenderTargetSize(const FIntPoint& InNewSize) const;

	/** */
	void MakeStaticMeshAssetPickerMenu(UToolMenu* InToolMenu) const;

	/**  */
	void ProcessLogMessages();

	void GetProjectedScreenCoordinates(const TArray<FVector>& InWorldPositions, TArray<FVector2d>& OutScreenPositions) const;

	/** */
	FFrameTrackingContourData ProjectPromotedFrameCurvesOnTemplateMesh(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, const TArray<FVector>& InTemplateMeshVertices) const;

private:

	static const FName PartsTabId;
	static const FName OutlinerTabId;

	/** A reference to the current selected Pose */
	TWeakObjectPtr<class UMetaHumanIdentityPose> SelectedIdentityPose;

	/** A reference to the viewport widget where we display the tracking contour data */
	TSharedPtr<class SConformingAssetEditorViewport> ViewportWidget;

	/** A reference to the Identity Parts editor Widget */
	TSharedPtr<class SMetaHumanIdentityPartsEditor> IdentityPartsEditor;

	/** The widget used to display Promoted Frames for a Identity Pose */
	TSharedPtr<class SMetaHumanIdentityPromotedFramesEditor> PromotedFramesEditorWidget;

	/** The widget used to display the Promoted Frame curves and landmarks outliner */
	TSharedPtr<class SMetaHumanIdentityOutliner> OutlinerWidget;

	/** A Reference to the Identity we are editing */
	TObjectPtr<class UMetaHumanIdentity> Identity;

	/** A component used to capture the scene in a texture for tracking purposes */
	TObjectPtr<class USceneCaptureComponent2D> SceneCaptureComponent;

	/** Pipeline for tracking Promoted Frames */
	UE::MetaHuman::Pipeline::FPipeline TrackPipeline;

	/** True if we are running the conformer */
	int8 bIsConforming : 1;

	/** True if the auto rigging service has been called */
	int8 bIsAutorigging : 1;

	/** A cache of log messages from core tech libraries */
	TArray<TPair<FString, ELogVerbosity::Type>> LogMessages;

	bool LoadCurvesAndLandmarksFromJson(const FString& FileName);
	bool LoadGroupsFromJson(const FString& FileName) const;
	void SetUpInitialOutlinerData() const;
	void UpdateCurveStateForFrameAfterTracking(const FFrameTrackingContourData& InTrackData, UMetaHumanIdentityPromotedFrame* InTrackedFrame) const;
	void UpdateOutlinerFrameData() const;

	void RefreshSceneCapture() const;

private:

	TWeakPtr<class SNotificationItem> AutoRigProgressNotification;

	/** A struct containing non-changing marker group and curve data */
	TSharedPtr<FMarkerDefs> MarkerDefs;

	/** A struct containing both static and state marker group and curve data, passed to Outliner for creating the listviews */
	TSharedPtr<FMarkerData> MarkerData;
	
	class UPostProcessComponent* PostProcessComponent;
};