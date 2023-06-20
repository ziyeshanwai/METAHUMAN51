// Copyright Epic Games, Inc.All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "EditorViewportClient.h"
#include "SABImage.h"

DECLARE_DELEGATE_OneParam(FOnSceneComponentClicked, const class USceneComponent*)
DECLARE_DELEGATE_TwoParams(FOnMarkerVisibilityChanged, bool, bool)
DECLARE_DELEGATE_RetVal_OneParam(class USceneComponent*, FOnGetSceneComponentOfType, enum class ESceneComponentIdentifier)

enum class ESceneComponentIdentifier
{
	None,
	ImportedMesh,
	ConformalMesh,
	ConformalRig
};

class FConformingViewportClient
	: public FEditorViewportClient
	, public TSharedFromThis<FConformingViewportClient>
{
public:
	FConformingViewportClient(const TSharedRef<FPreviewScene>& InPreviewScene, class UMetaHumanIdentity* InIdentity);
	~FConformingViewportClient();

	//~ FEditorViewportClient interface
	virtual void RequestInvalidateHitProxy(FViewport* Viewport) override;
	virtual void ProcessClick(FSceneView& InView, HHitProxy* InHitProxy, FKey InKey, EInputEvent InEvent, uint32 InHitX, uint32 InHitY) override;
	virtual UE::Widget::EWidgetMode GetWidgetMode() const override;
	virtual void SetWidgetMode(UE::Widget::EWidgetMode InWidgetMode);
	virtual FVector GetWidgetLocation() const override;
	virtual void TrackingStarted(const struct FInputEventState& InInputState, bool bInIsDraggingWidget, bool bInNudge) override;
	virtual void TrackingStopped() override;
	virtual bool InputWidgetDelta(FViewport* InViewport, EAxisList::Type InCurrentAxis, FVector& InDrag, FRotator& InRot, FVector& InScale);
	virtual void PerspectiveCameraMoved() override;
	virtual void EndCameraMovement() override;
	virtual void SetViewMode(EViewModeIndex InViewModeIndex) override;
	virtual void Tick(float DeltaSeconds) override;

	/** Returns true if the camera navigation is locked in the viewport */
	bool IsNavigationLocked() const;

	/** Returns if a view mode is enabled for view A or view B */
	bool IsViewModeIndexEnabledForImageViewMode(EViewModeIndex InViewModeIndex, EABImageViewMode InViewMode) const;

	/** Sets the view mode index for view A or view B */
	void SetViewModeIndexForImageViewMode(EViewModeIndex InViewModeIndex, EABImageViewMode InViewMode);

	/** Returns if using a fixed exposure for view A or view B */
	bool GetFixedEV100(EABImageViewMode InViewMode) const;

	/** Returns the EV100 value for view A or view B */
	float GetEV100(EABImageViewMode InViewMode) const;

	/** Sets the exposure for view A or view B */
	void SetEV100(EABImageViewMode InViewMode, float InValue);

	/** Returns the view mode index for view A or view B */
	EViewModeIndex GetViewModeIndexForViewMode(EABImageViewMode InViewMode) const;

	/**
	 * Sets the editor viewport widget this client is attached to. This is needed to support non-instant
	 * camera transitions with focusing on an object. This also configures the scene capture components
	 * used for the AB split functionality.
	 */
	void SetEditorViewportWidget(TSharedRef<class SConformingAssetEditorViewport> InEditorViewportWidget);

	/** Sets the currently selected component used to focus when pressing F in the viewport */
	void SetSelectedSceneComponents(const TArray<USceneComponent*>& InSceneComponents, const TArray<USceneComponent*>& InSceneComponentInstances);

	/** Focus the camera on the selected components */
	void FocusViewportOnSelectedComponents();

	FOnSceneComponentClicked& OnSceneComponentClicked()
	{
		return OnSceneComponentClickedDelegate;
	}

	FOnGetSceneComponentOfType& OnGetSceneComponetOfType()
	{
		return OnGetSceneComponetOfTypeDelegate;
	}

	FOnGetSceneComponentOfType& OnGetSceneComponentInstanceOfType()
	{
		return OnGetSceneComponentInstanceOfTypeDelegate;
	}

	FSimpleDelegate& OnCameraMoved()
	{
		return OnConformingViewportCameraMovedDelegate;
	}

	FSimpleMulticastDelegate& OnCameraStopped()
	{
		return OnConformingViewportCameraStoppedDelegate;
	}

	FSimpleDelegate& OnRefreshSceneCapture()
	{
		return OnRefreshSceneCaptureDelegate;
	}

	FOnMarkerVisibilityChanged& OnMarkerVisibilityChanged()
	{
		return OnMarkerVisibilityChangedDelegate;
	}

	void ToggleAB();
	void ToggleToSingleViewA();
	void ToggleToSingleViewB();
	void ToggleAB_Split();
	void ToggleAB_Side();
	void ToggleCurveVisibility(bool InIsViewA);
	void ToggleReferenceMeshForView(bool InIsViewA);
	void ToggleConformedMeshForView(bool InIsViewA);
	void ToggleConformalRigForView(bool InIsViewA);
	void ToggleDisplayCurves(bool InIsViewA);
	void ToggleDisplayPoints(bool InIsViewA);
	void SwapPerViewParameter(TMap<EABImageViewMode, bool>& InPerViewParameter) const;

	bool ShowViewA() const;
	bool ShowViewB() const;
	bool IsShowingSingleView() const;
	bool GetPerViewParameter(EABImageViewMode InMode, const TMap<EABImageViewMode, bool>& InPerViewParameter) const;
	bool IsShowingConformedMesh(bool IsViewA) const;
	bool IsShowingConformalRig(bool IsViewA) const;
	bool IsShowingReferenceMesh(bool IsViewA) const;
	bool IsShowingCurves(bool IsViewA) const;
	bool IsShowingPoints(bool IsViewA) const;
	bool GetAB_Split() const;
	bool GetAB_Side() const;
	bool GetAB_3D() const;

	TMap<EABImageViewMode, TArray<ESceneComponentIdentifier>> GetHiddenComponentsForViews() const;
	TArray<ESceneComponentIdentifier> GetVisibleComponentIdentifiers();
	EABImageViewMode GetCurrentImageViewMode() const;

	/** Updates the visibility of scene components in relation to the capture scene components of the AB split */
	void UpdateSceneComponentVisibility();

	/** Updates the visibility of points/curves in viewport */
	void UpdateMarkerVisibility();

private:

	/** Returns the bounding box for the selected components in the viewport */
	FBox GetSelectedComponentsBoundingBox() const;

	/** Returns a reference to the ConformingAssetEditorViewport */
	TSharedRef<class SConformingAssetEditorViewport> GetConformingAssetEditorViewport() const;

	/** Returns the ABImage widget we are controlling. The widget lives in SConformingAssetEditorViewport */
	TSharedRef<SABImage> GetABImageWidget() const;

	/** Returns true if all of selected components are valid, i.e. they point a valid object */
	bool AreAllSelectedComponentsValid() const;

private:
	// The preview scene to	use for rendering the viewport
	TSharedRef<FPreviewScene> PreviewScene;

	// The current gizmo widget mode
	UE::Widget::EWidgetMode WidgetMode;

	// Delegate called when the perspective camera moved
	FSimpleDelegate OnConformingViewportCameraMovedDelegate;

	// Delegate called when the perpective camera stops moving
	FSimpleMulticastDelegate OnConformingViewportCameraStoppedDelegate;

	// Delegate called when scene capture needs retaking, eg change in the the visibility of scene components, lighting change
	FSimpleDelegate OnRefreshSceneCaptureDelegate;

	// Delegate called when marker curve or point visibility is changed
	FOnMarkerVisibilityChanged OnMarkerVisibilityChangedDelegate;

	// Delegate called when the user clicks on a scene component in the viewport
	FOnSceneComponentClicked OnSceneComponentClickedDelegate;

	// Delegate called to obtain a scene component of the given type
	FOnGetSceneComponentOfType OnGetSceneComponetOfTypeDelegate;

	// Delegate called to obtain a scene component instance of the give type
	FOnGetSceneComponentOfType OnGetSceneComponentInstanceOfTypeDelegate;

	// A reference to the Identity being edited
	TWeakObjectPtr<class UMetaHumanIdentity> Identity;

	// A reference to the currently selected components
	TArray<TWeakObjectPtr<class USceneComponent>> SelectedComponents;

	// A reference to the currently selected component instances
	TArray<TWeakObjectPtr<class USceneComponent>> SelectedComponentInstances;

	// The transaction used to record modifications done using the gizmos in the viewport
	TUniquePtr<class FScopedTransaction> ScopedTransaction;

	// The initial pivot location when the user starts interacting with gizmos in the viewport
	FVector InitialPivotLocation;

	// True if we are manipulating a component through a gizmo
	uint8 bIsManipulating : 1;

	// True if showing contents of view A
	uint8 bShowViewA : 1;

	// True if showing contents of view B
	uint8 bShowViewB : 1;

	/** AB Split for data comparison*/
	TMap<EABImageViewMode, TObjectPtr<class UMetaHumanSceneCaptureComponent2D>> ABSceneCaptureComponent;
	TMap<EABImageViewMode, bool> ShowNeutralPoseMesh;
	TMap<EABImageViewMode, bool> ShowConformalMesh;
	TMap<EABImageViewMode, bool> ShowRig;
	TMap<EABImageViewMode, bool> ShowContours;
	TMap<EABImageViewMode, bool> ShowContourPoints;

	struct FABViewModeSettings
	{
		EViewModeIndex ViewModeIndex;
		FExposureSettings ExposureSettings;
	};

	TMap<EABImageViewMode, FABViewModeSettings> ViewModeSettings;
};