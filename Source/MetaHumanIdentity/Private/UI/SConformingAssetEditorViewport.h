// Copyright Epic Games, Inc.All Rights Reserved.

#pragma once

#include "SAssetEditorViewport.h"

DECLARE_DELEGATE_OneParam(FOnViewportSizeChanged, const FIntPoint& InNewSize)
DECLARE_DELEGATE_OneParam(FOnCurvesSelected, const TArray<FString>& InCurveNames)

/**
 * The Conforming asset editor viewport is the viewport that displays the Identity scene and
 * also has an overlay that can display a snapshot of the scene plus tracking data.
 * See STrackerImageViewer for more information on the overlay widget used here.
 */
class SConformingAssetEditorViewport
	: public SAssetEditorViewport
{
public:

	SLATE_BEGIN_ARGS(SConformingAssetEditorViewport) {}

		// The viewport client that controls the viewport
		SLATE_ARGUMENT(TSharedPtr<class FConformingViewportClient>, EditorViewportClient)

		// The render target to display in the overlay image tracker widget
		SLATE_ARGUMENT(class UTextureRenderTarget2D*, RenderTarget)

		// The promoted frames editor to overlay as a bottom toolbar
		SLATE_ARGUMENT(TSharedPtr<class SMetaHumanIdentityPromotedFramesEditor>, PromotedFramesEditorWidget)

		// The command list from the toolkit with commands that are already mapped to actions
		SLATE_ARGUMENT(TSharedPtr<class FUICommandList>, ToolkitCommandList)

		// Event called when the viewport size changes
		SLATE_EVENT(FOnViewportSizeChanged, OnViewportSizeChanged)

		// Event called to request a refresh of the captures scene component when the navigation is locked
		SLATE_EVENT(FSimpleDelegate, OnCaptureSceneRequested)

		// Event called when curve in STrackerImageViewer is selected
		SLATE_EVENT(FOnCurvesSelected, OnTrackerViewCurveSelected)

	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, const FAssetEditorViewportConstructionArgs& InViewportConstructionArgs);

	/** Returns true if the navigation is locked for the selected promoted frame */
	bool IsTrackerImageViewerNavigationLocked() const;

	/** Reset the tracker overlay view, re-centering it on the viewport */
	void ResetTrackerOverlayView(bool UpdateGeometry = true) const;

	/** Sets the ShapeAnnotation with the landmarks to be displayed in the viewport */
	void SetShapeAnnotation(const TSharedPtr<class FShapeAnnotationWrapper>& InShapeAnnotation) const;

	/** Sets curve selection in viewport */
	void SetCurveSelectionInImageViewer(const TArray<FString>& InSelectedCurves) const;

	/** Called when the camera stops moving like when the user releases the mouse */
	void OnCameraStopped();

	/** Sets the size of the image brush */
	void UpdateImageBrush();

	/** Returns a reference to the promoted frames editor */
	TSharedRef<class SMetaHumanIdentityPromotedFramesEditor> GetPromotedFramesEditor() const;

	/** Returns a referenced to the AB Image Widget */
	TSharedRef<class SABImage> GetABImageWidget() const;

	//~ SWidget interface
	virtual void Tick(const FGeometry& InAllottedGeometry, const double InCurrentTime, const float InDeltaTime) override;

	//~ SEditorViewport interface
	virtual void BindCommands() override;

	virtual FReply OnKeyDown(const FGeometry& MyGeometry, const FKeyEvent& InKeyEvent) override;

protected:
	//~ SEditorViewport interface
	virtual void PopulateViewportOverlays(TSharedRef<SOverlay> InOverlay) override;
	virtual void OnFocusViewportToSelection() override;

private:

	/** Returns the visibility of the STrackerImageViewer overlay. */
	EVisibility IsTrackerImageViewerVisible() const;

private:

	/** A brush used to display the contents of the RenderTarget */
	FSlateBrush TrackerImageBrush;

	/** A reference to the tracker image viewer overlay */
	TSharedPtr<class STrackerImageViewer> TrackerImageViewer;

	/** A reference to the AB type image viewer overlay */
	TSharedPtr<class SABImage> ABImage;

	/** A reference to the promoted frames editor overlay */
	TSharedPtr<class SMetaHumanIdentityPromotedFramesEditor> PromotedFramesEditorWidget;

	/** Delegate called anytime the size of the viewport changes */
	FSimpleDelegate OnGeometryChangedDelegate;

	FOnCurvesSelected OnCurvesSelectedDelegat;

	/** Delegate called anytime the viewport size changes */
	FOnViewportSizeChanged OnViewportSizeChanged;

	/** Command list to map actions of the ViewB */
	TSharedPtr<FUICommandList> CommandListViewB;

	/** The current size of the viewport. Used to check changes in the size of the widget */
	FGeometry CurrentViewportGeometry;
};