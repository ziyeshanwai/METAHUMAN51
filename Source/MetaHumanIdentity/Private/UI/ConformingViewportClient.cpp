// Copyright Epic Games, Inc.All Rights Reserved.

#include "ConformingViewportClient.h"
#include "MetaHumanIdentity.h"
#include "MetaHumanSceneCaptureComponent2D.h"
#include "SConformingAssetEditorViewport.h"

#include "EngineUtils.h"
#include "Kismet2/ComponentEditorUtils.h"
#include "Editor/UnrealEdEngine.h"
#include "UnrealEdGlobals.h"
#include "ScopedTransaction.h"
#include "PreviewScene.h"
#include "Algo/AllOf.h"

#define LOCTEXT_NAMESPACE "MetaHumanIdentity"

FConformingViewportClient::FConformingViewportClient(const TSharedRef<FPreviewScene>& InPreviewScene, UMetaHumanIdentity* InIdentity)
	: FEditorViewportClient{ nullptr, &InPreviewScene.Get() }
	, PreviewScene{ InPreviewScene }
	, Identity{ InIdentity }
	, bIsManipulating{ false }
	, bShowViewA{ true }
	, bShowViewB{ false }
{
	// Make sure the identity is valid
	check(Identity != nullptr);

	OverrideNearClipPlane(0.1f);
	ViewFOV = 45.0f;
	CameraSpeedSetting = 2;
	ExposureSettings.bFixed = true;

	ShowNeutralPoseMesh.Add(EABImageViewMode::A, true);
	ShowNeutralPoseMesh.Add(EABImageViewMode::B, false);
	ShowConformalMesh.Add(EABImageViewMode::B, true);
	ShowConformalMesh.Add(EABImageViewMode::A, false);
	ShowRig.Add(EABImageViewMode::A, false);
	ShowRig.Add(EABImageViewMode::B, false);
	ShowContours.Add(EABImageViewMode::A, true);
	ShowContours.Add(EABImageViewMode::B, true);
	ShowContourPoints.Add(EABImageViewMode::A, true);
	ShowContourPoints.Add(EABImageViewMode::B, true);

	FABViewModeSettings DefaultViewModeSettings;
	DefaultViewModeSettings.ViewModeIndex = VMI_Lit;
	DefaultViewModeSettings.ExposureSettings = ExposureSettings;

	ViewModeSettings.Add(EABImageViewMode::A, DefaultViewModeSettings);
	ViewModeSettings.Add(EABImageViewMode::B, DefaultViewModeSettings);

	SetViewLocation(EditorViewportDefs::DefaultPerspectiveViewLocation);
	SetViewRotation(EditorViewportDefs::DefaultPerspectiveViewRotation);
	SetRealtime(true);

	EngineShowFlags.SetSelectionOutline(GetDefault<ULevelEditorViewportSettings>()->bUseSelectionOutline);
}

FConformingViewportClient::~FConformingViewportClient()
{
	// Destructor required as the header doesn't have the definition of FScopedTransaction
}

void FConformingViewportClient::RequestInvalidateHitProxy(FViewport* InViewport)
{
	FEditorViewportClient::RequestInvalidateHitProxy(InViewport);
}

void FConformingViewportClient::ProcessClick(FSceneView& InView, HHitProxy* InHitProxy, FKey InKey, EInputEvent InEvent, uint32 InHitX, uint32 InHitY)
{
	const FViewportClick Click(&InView, this, InKey, InEvent, InHitX, InHitY);

	if (InHitProxy != nullptr)
	{
		if (HActor* ActorProxy = HitProxyCast<HActor>(InHitProxy))
		{
			if (const USceneComponent* SceneComponent = Cast<USceneComponent>(ActorProxy->PrimComponent))
			{
				OnSceneComponentClickedDelegate.ExecuteIfBound(SceneComponent);
			}
		}
	}

	FEditorViewportClient::ProcessClick(InView, InHitProxy, InKey, InEvent, InHitX, InHitY);
}

UE::Widget::EWidgetMode FConformingViewportClient::GetWidgetMode() const
{
	if (AreAllSelectedComponentsValid())
	{
		return WidgetMode;
	}

	return UE::Widget::WM_None;
}

void FConformingViewportClient::SetWidgetMode(UE::Widget::EWidgetMode InWidgetMode)
{
	WidgetMode = InWidgetMode;

	if (WidgetMode == UE::Widget::WM_Scale && AreAllSelectedComponentsValid())
	{
		InitialPivotLocation = GetSelectedComponentsBoundingBox().GetCenter();
	}
}

FVector FConformingViewportClient::GetWidgetLocation() const
{
	FVector Location = FVector::ZeroVector;

	if (WidgetMode == UE::Widget::WM_Scale)
	{
		Location = InitialPivotLocation;
	}
	else
	{
		Location = GetSelectedComponentsBoundingBox().GetCenter();
	}

	return Location;
}

void FConformingViewportClient::TrackingStarted(const FInputEventState& InInputState, bool bInIsDraggingWidget, bool bInNudge)
{
	if (!bIsManipulating && bInIsDraggingWidget)
	{
		// Prevent the editor from emitting notifications for each delta change in when manipulating components using gizmos.
		// This avoid recording intermediate steps reducing overhead of the undo system
		GEditor->DisableDeltaModification(true);

		if (!ScopedTransaction.IsValid() && Identity.IsValid() && !SelectedComponents.IsEmpty())
		{
			ScopedTransaction = MakeUnique<FScopedTransaction>(LOCTEXT("MoveMultipleIdentityComponents", "Modify Multiple"));
		}

		for (const TWeakObjectPtr<USceneComponent>& SelectedComponent : SelectedComponents)
		{
			if (SelectedComponent.IsValid())
			{
				SelectedComponent->Modify();
			}
		}

		bIsManipulating = true;
	}
}

void FConformingViewportClient::TrackingStopped()
{
	if (bIsManipulating)
	{
		// resetting the scoped transaction will call its destructor, thus, registering the transaction in the undo history
		ScopedTransaction.Reset();

		// Restore delta notifications
		GEditor->DisableDeltaModification(false);

		// Reset the initial pivot location in case we were scaling
		InitialPivotLocation = GetSelectedComponentsBoundingBox().GetCenter();

		bIsManipulating = false;
	}
}

bool FConformingViewportClient::InputWidgetDelta(FViewport* InViewport, EAxisList::Type InCurrentAxis, FVector& InDrag, FRotator& InRot, FVector& InScale)
{
	bool bHandled = false;

	if (bIsManipulating && InCurrentAxis != EAxisList::None && !SelectedComponents.IsEmpty())
	{
		// If the scale is being changed we keep the pivot in its original location
		const FVector PivotLocation = WidgetMode == UE::Widget::WM_Scale ? InitialPivotLocation : GetSelectedComponentsBoundingBox().GetCenter();

		for (int32 ComponentIndex = 0; ComponentIndex < SelectedComponents.Num(); ++ComponentIndex)
		{
			USceneComponent* SceneComponent = SelectedComponents[ComponentIndex].Get();
			USceneComponent* SceneComponentInstance = SelectedComponentInstances[ComponentIndex].Get();

			// This takes into account parent components, if any
			FComponentEditorUtils::AdjustComponentDelta(SceneComponent, InDrag, InRot);

			// Finally we change the component transform
			const bool bDelta = true;
			GUnrealEd->ApplyDeltaToComponent(SceneComponent,
											 bDelta,
											 &InDrag,
											 &InRot,
											 &InScale,
											 PivotLocation);

			SceneComponentInstance->SetWorldTransform(SceneComponent->GetComponentTransform());
			SceneComponent->TransformUpdated.Broadcast(SceneComponent, EUpdateTransformFlags::None, ETeleportType::None);
		}

		bHandled = true;
	}

	return bHandled;
}

void FConformingViewportClient::Tick(float DeltaSeconds)
{
	FEditorViewportClient::Tick(DeltaSeconds);

	if (!GIntraFrameDebuggingGameThread)
	{
		if (!GetABImageWidget()->IsSingleView())
		{
			PreviewScene->GetWorld()->Tick(LEVELTICK_ViewportsOnly, DeltaSeconds);
		}
	}

	// Store the exposure settings for the current view mode
	// Couldn't find a way to know when these settings change so use Tick to always save the value for the current view mode
	ViewModeSettings[GetABImageWidget()->GetViewMode()].ExposureSettings = ExposureSettings;
}

bool FConformingViewportClient::IsNavigationLocked() const
{
	if (EditorViewportWidget.IsValid())
	{
		if (TSharedPtr<SConformingAssetEditorViewport> ConformingViewportWidget = StaticCastSharedPtr<SConformingAssetEditorViewport>(EditorViewportWidget.Pin()))
		{
			return ConformingViewportWidget->IsTrackerImageViewerNavigationLocked();
		}
	}

	return false;
}

bool FConformingViewportClient::IsViewModeIndexEnabledForImageViewMode(EViewModeIndex InViewModeIndex, EABImageViewMode InViewMode) const
{
	return ViewModeSettings[InViewMode].ViewModeIndex == InViewModeIndex;
}

void FConformingViewportClient::SetViewModeIndexForImageViewMode(EViewModeIndex InViewModeIndex, EABImageViewMode InViewMode)
{
	if (IsShowingSingleView())
	{
		ViewModeSettings[InViewMode].ViewModeIndex = InViewModeIndex;

		if (GetCurrentImageViewMode() == InViewMode)
		{
			FEditorViewportClient::SetViewMode(InViewModeIndex);
		}

		OnRefreshSceneCaptureDelegate.ExecuteIfBound();
	}
}

EViewModeIndex FConformingViewportClient::GetViewModeIndexForViewMode(EABImageViewMode InViewMode) const
{
	return ViewModeSettings[InViewMode].ViewModeIndex;
}

bool FConformingViewportClient::GetFixedEV100(EABImageViewMode InViewMode) const
{
	return ViewModeSettings[InViewMode].ExposureSettings.bFixed;
}

float FConformingViewportClient::GetEV100(EABImageViewMode InViewMode) const
{
	return ViewModeSettings[InViewMode].ExposureSettings.FixedEV100;
}

void FConformingViewportClient::SetEV100(EABImageViewMode InViewMode, float InValue)
{
	ExposureSettings.bFixed = true;
	ExposureSettings.FixedEV100 = InValue;

	ViewModeSettings[InViewMode].ExposureSettings = ExposureSettings;

	if (GetCurrentImageViewMode() == InViewMode)
	{
		Invalidate();

		OnRefreshSceneCapture().ExecuteIfBound();
	}
}

void FConformingViewportClient::SetEditorViewportWidget(TSharedRef<SConformingAssetEditorViewport> InEditorViewportWidget)
{
	EditorViewportWidget = InEditorViewportWidget;

	TSharedRef<SABImage> ABImageWidget = GetABImageWidget();

	for (EABImageViewMode Mode : ABImageWidget->SingleViewModes())
	{
		UMetaHumanSceneCaptureComponent2D* SceneCapture = NewObject<UMetaHumanSceneCaptureComponent2D>(GetTransientPackage(), NAME_None, RF_Transient);
		SceneCapture->SetViewportClient(AsShared());
		SceneCapture->TextureTarget = ABImageWidget->GetRenderTarget(Mode);
		SceneCapture->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;

		GetPreviewScene()->AddComponent(SceneCapture, FTransform::Identity);

		ABSceneCaptureComponent.Add(Mode, SceneCapture);
	}

	UpdateSceneComponentVisibility();
}

void FConformingViewportClient::FocusViewportOnSelectedComponents()
{
	if (AreAllSelectedComponentsValid())
	{
		FocusViewportOnBox(GetSelectedComponentsBoundingBox());
	}
}

void FConformingViewportClient::SetSelectedSceneComponents(const TArray<USceneComponent*>& InSceneComponents, const TArray<USceneComponent*>& InSceneComponentInstances)
{
	check(InSceneComponents.Num() == InSceneComponentInstances.Num());

	auto ConvertToWeakPtrArrayLambda = [](const TArray<USceneComponent*>& InComponents, TArray<TWeakObjectPtr<USceneComponent>>& OutComponents)
	{
		OutComponents.Reset(InComponents.Num());
		for (USceneComponent* Component : InComponents)
		{
			OutComponents.Add(Component);
		}
	};

	ConvertToWeakPtrArrayLambda(InSceneComponents, SelectedComponents);
	ConvertToWeakPtrArrayLambda(InSceneComponentInstances, SelectedComponentInstances);

	const bool bAreAllComponentsValid = AreAllSelectedComponentsValid();
	ShowWidget(bAreAllComponentsValid);
	SetWidgetMode(WidgetMode != UE::Widget::WM_None ? WidgetMode : UE::Widget::WM_Translate);
}

void FConformingViewportClient::PerspectiveCameraMoved()
{
	bIsCameraMoving = true;
	OnConformingViewportCameraMovedDelegate.ExecuteIfBound();
}

void FConformingViewportClient::EndCameraMovement()
{
	if (bIsCameraMoving && !bIsTracking)
	{
		OnConformingViewportCameraStoppedDelegate.Broadcast();

		bIsCameraMoving = false;
	}
}

void FConformingViewportClient::SetViewMode(EViewModeIndex InViewModeIndex)
{
	//prevent reacting to modes we don't use
	if (InViewModeIndex != EViewModeIndex::VMI_Lit &&
		InViewModeIndex != EViewModeIndex::VMI_Unlit &&
		InViewModeIndex != EViewModeIndex::VMI_LightingOnly)
	{
		return;
	}

	FEditorViewportClient::SetViewMode(InViewModeIndex);

	if (IsShowingSingleView())
	{
		ViewModeSettings[GetABImageWidget()->GetViewMode()].ViewModeIndex = InViewModeIndex;
	}
}

FBox FConformingViewportClient::GetSelectedComponentsBoundingBox() const
{
	FBoxSphereBounds ComponentBounds{ ForceInit };

	if (!SelectedComponentInstances.IsEmpty())
	{
		ComponentBounds = SelectedComponents[0]->Bounds;
	}

	for (const TWeakObjectPtr<USceneComponent>& SelectedComponent : SelectedComponentInstances)
	{
		ComponentBounds = ComponentBounds + SelectedComponent->Bounds;
	}

	return ComponentBounds.GetBox();
}

TSharedRef<SConformingAssetEditorViewport> FConformingViewportClient::GetConformingAssetEditorViewport() const
{
	check(EditorViewportWidget.IsValid());
	return StaticCastSharedPtr<SConformingAssetEditorViewport>(GetEditorViewportWidget()).ToSharedRef();
}

TSharedRef<SABImage> FConformingViewportClient::GetABImageWidget() const
{
	return GetConformingAssetEditorViewport()->GetABImageWidget();
}

bool FConformingViewportClient::AreAllSelectedComponentsValid() const
{
	auto IsComponentValidLambda = [](const TWeakObjectPtr<USceneComponent>& Component)
	{
		return Component.IsValid();
	};

	const bool bSceneComponentsValid = Algo::AllOf(SelectedComponents, IsComponentValidLambda);
	const bool bSceneComponentInstancesValid = Algo::AllOf(SelectedComponentInstances, IsComponentValidLambda);

	return bSceneComponentsValid && bSceneComponentInstancesValid;
}

void FConformingViewportClient::UpdateSceneComponentVisibility()
{
	const bool bPropagateToChildren = true;

	// Make components that should be visible actually visible
	if (OnGetSceneComponetOfTypeDelegate.IsBound() && OnGetSceneComponentInstanceOfTypeDelegate.IsBound())
	{
		check(SelectedComponents.Num() == SelectedComponentInstances.Num());

		SelectedComponents.Empty();
		SelectedComponentInstances.Empty();

		for (ESceneComponentIdentifier Identifier : GetVisibleComponentIdentifiers())
		{
			if (USceneComponent* Component = OnGetSceneComponetOfTypeDelegate.Execute(Identifier))
			{
				Component->SetVisibility(true, bPropagateToChildren);

				SelectedComponents.Add(Component);
			}

			if (USceneComponent* Component = OnGetSceneComponentInstanceOfTypeDelegate.Execute(Identifier))
			{
				Component->SetVisibility(true, bPropagateToChildren);

				SelectedComponentInstances.Add(Component);
			}
		}

		TSharedRef<SABImage> ABImageWidget = GetABImageWidget();

		const EABImageViewMode CurrentViewMode = GetCurrentImageViewMode();

		// Hide the components in the respective scene capture components
		const TMap<EABImageViewMode, TArray<ESceneComponentIdentifier>> HiddenComponents = GetHiddenComponentsForViews();
		for (const EABImageViewMode Mode : ABImageWidget->SingleViewModes())
		{
			ABSceneCaptureComponent[Mode]->ClearHiddenComponents();

			if (OnGetSceneComponetOfTypeDelegate.IsBound())
			{
				for (const ESceneComponentIdentifier& Identifier : HiddenComponents[Mode])
				{
					if (UPrimitiveComponent* Component = Cast <UPrimitiveComponent>(OnGetSceneComponetOfTypeDelegate.Execute(Identifier)))
					{
						if (CurrentViewMode == Mode)
						{
							Component->SetVisibility(false, bPropagateToChildren);
						}
					}

					if (UPrimitiveComponent* Component = Cast<UPrimitiveComponent>(OnGetSceneComponentInstanceOfTypeDelegate.Execute(Identifier)))
					{
						ABSceneCaptureComponent[Mode]->HideComponent(Component);

						if (CurrentViewMode == Mode)
						{
							// Hide the component if it should be hidden for the given mode
							Component->SetVisibility(false, bPropagateToChildren);
						}
					}
				}
			}
		}

		SetViewMode(ViewModeSettings[CurrentViewMode].ViewModeIndex);
		ExposureSettings = ViewModeSettings[CurrentViewMode].ExposureSettings;
		Invalidate();

	}

	OnRefreshSceneCaptureDelegate.ExecuteIfBound();
}

void FConformingViewportClient::UpdateMarkerVisibility()
{
	auto CurrentViewmode = GetCurrentImageViewMode();
	bool bShowContours = ShowContours[CurrentViewmode];
	bool bShowPoints = ShowContourPoints[CurrentViewmode];

	OnMarkerVisibilityChangedDelegate.ExecuteIfBound(bShowContours, bShowPoints);
}

void FConformingViewportClient::ToggleAB()
{
	TSharedRef<SABImage> ABImageWidget = GetABImageWidget();

	if (IsShowingSingleView())
	{
		if (ABImageWidget->GetViewMode() == EABImageViewMode::A)
		{
			bShowViewA = false;
			bShowViewB = true;
			ABImageWidget->SetViewMode(EABImageViewMode::B);
		}
		else if (ABImageWidget->GetViewMode() == EABImageViewMode::B)
		{
			bShowViewA = true;
			bShowViewB = false;
			ABImageWidget->SetViewMode(EABImageViewMode::A);
		}
	}
	else
	{
		Swap(ShowNeutralPoseMesh[EABImageViewMode::A], ShowNeutralPoseMesh[EABImageViewMode::B]);
		Swap(ShowConformalMesh[EABImageViewMode::A], ShowConformalMesh[EABImageViewMode::B]);
		Swap(ShowRig[EABImageViewMode::A], ShowRig[EABImageViewMode::B]);
	}

	UpdateSceneComponentVisibility();
	UpdateMarkerVisibility();
}

void FConformingViewportClient::ToggleCurveVisibility(bool InIsViewA)
{
	const EABImageViewMode Viewmode = InIsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	ShowContours[Viewmode] = !ShowContours[Viewmode];
}

void FConformingViewportClient::ToggleToSingleViewA()
{
	bShowViewA = true;
	bShowViewB = false;
	GetABImageWidget()->SetViewMode(EABImageViewMode::A);

	UpdateSceneComponentVisibility();
	UpdateMarkerVisibility();
}

void FConformingViewportClient::ToggleToSingleViewB()
{
	bShowViewA = false;
	bShowViewB = true;
	GetABImageWidget()->SetViewMode(EABImageViewMode::B);

	UpdateSceneComponentVisibility();
	UpdateMarkerVisibility();
}

void FConformingViewportClient::ToggleAB_Split()
{
	bShowViewA = true;
	bShowViewB = true;
	GetABImageWidget()->SetViewMode(EABImageViewMode::ABSplit);

	UpdateSceneComponentVisibility();
	UpdateMarkerVisibility();
}

void FConformingViewportClient::ToggleAB_Side()
{
	bShowViewA = true;
	bShowViewB = true;
	GetABImageWidget()->SetViewMode(EABImageViewMode::ABSide);

	UpdateSceneComponentVisibility();
	UpdateMarkerVisibility();
}

void FConformingViewportClient::ToggleReferenceMeshForView(bool InIsViewA)
{
	const EABImageViewMode Viewmode = InIsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	ShowNeutralPoseMesh[Viewmode] = !ShowNeutralPoseMesh[Viewmode];

	UpdateSceneComponentVisibility();
}

void FConformingViewportClient::ToggleConformedMeshForView(bool InIsViewA)
{
	auto Viewmode = InIsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	ShowConformalMesh[Viewmode] = !ShowConformalMesh[Viewmode];

	UpdateSceneComponentVisibility();
}

void FConformingViewportClient::ToggleConformalRigForView(bool InIsViewA)
{
	auto Viewmode = InIsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	ShowRig[Viewmode] = !ShowRig[Viewmode];

	UpdateSceneComponentVisibility();
}

void FConformingViewportClient::ToggleDisplayCurves(bool InIsViewA)
{
	auto Viewmode = InIsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	ShowContours[Viewmode] = !ShowContours[Viewmode];

	UpdateMarkerVisibility();
}

void FConformingViewportClient::ToggleDisplayPoints(bool InIsViewA)
{
	auto Viewmode = InIsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	ShowContourPoints[Viewmode] = !ShowContourPoints[Viewmode];

	UpdateMarkerVisibility();
}

void FConformingViewportClient::SwapPerViewParameter(TMap<EABImageViewMode, bool>& InPerViewParameter) const
{
	Swap(InPerViewParameter[EABImageViewMode::A], InPerViewParameter[EABImageViewMode::B]);
}

bool FConformingViewportClient::GetAB_3D() const
{
	return true;
}

bool FConformingViewportClient::ShowViewA() const
{
	return bShowViewA;
}

bool FConformingViewportClient::ShowViewB() const
{
	return bShowViewB;
}

bool FConformingViewportClient::IsShowingSingleView() const
{
	return GetABImageWidget()->GetViewMode() == EABImageViewMode::A || GetABImageWidget()->GetViewMode() == EABImageViewMode::B;
}

bool FConformingViewportClient::IsShowingConformedMesh(bool IsViewA) const
{
	const EABImageViewMode Viewmode = IsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	return ShowConformalMesh[Viewmode];
}

bool FConformingViewportClient::IsShowingConformalRig(bool IsViewA) const
{
	const EABImageViewMode Viewmode = IsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	return ShowRig[Viewmode];
}

bool FConformingViewportClient::IsShowingReferenceMesh(bool IsViewA) const
{
	const EABImageViewMode Viewmode = IsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	return ShowNeutralPoseMesh[Viewmode];
}

bool FConformingViewportClient::IsShowingCurves(bool IsViewA) const
{
	const EABImageViewMode Viewmode = IsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	return ShowContours[Viewmode];
}

bool FConformingViewportClient::IsShowingPoints(bool IsViewA) const
{
	const EABImageViewMode Viewmode = IsViewA ? EABImageViewMode::A : EABImageViewMode::B;
	return ShowContourPoints[Viewmode];
}

bool FConformingViewportClient::GetAB_Split() const
{
	return GetABImageWidget()->GetViewMode() == EABImageViewMode::ABSplit;
}

bool FConformingViewportClient::GetAB_Side() const
{
	return GetABImageWidget()->GetViewMode() == EABImageViewMode::ABSide;
}

bool FConformingViewportClient::GetPerViewParameter(EABImageViewMode InMode, const TMap<EABImageViewMode, bool>& InPerViewParameter) const
{
	TSharedRef<SABImage> ABImage = GetABImageWidget();

	if (InMode == EABImageViewMode::Any)
	{
		for (EABImageViewMode Mode : ABImage->SingleViewModes())
		{
			if (InPerViewParameter[Mode])
			{
				return true;
			}
		}

		return false;
	}

	EABImageViewMode Mode = (InMode == EABImageViewMode::Current ? ABImage->GetViewMode() : InMode);

	if (ABImage->SingleViewModes().Contains(Mode))
	{
		return InPerViewParameter[Mode];
	}
	else
	{
		return false;
	}
}

TMap<EABImageViewMode, TArray<ESceneComponentIdentifier>> FConformingViewportClient::GetHiddenComponentsForViews() const
{
	TMap<EABImageViewMode, TArray<ESceneComponentIdentifier>> HiddenComponents;

	for (EABImageViewMode Mode : GetABImageWidget()->SingleViewModes())
	{
		TArray<ESceneComponentIdentifier> ModeHiddenIdentifiers;
		if(!ShowConformalMesh[Mode])
		{
			ModeHiddenIdentifiers.Add(ESceneComponentIdentifier::ConformalMesh);
		}
		if(!ShowNeutralPoseMesh[Mode])
		{
			ModeHiddenIdentifiers.Add(ESceneComponentIdentifier::ImportedMesh);
		}
		if(!ShowRig[Mode])
		{
			ModeHiddenIdentifiers.Add(ESceneComponentIdentifier::ConformalRig);
		}

		if(!ModeHiddenIdentifiers.IsEmpty())
		{
			HiddenComponents.Add({Mode, ModeHiddenIdentifiers});
		}
	}

	return HiddenComponents;
}

TArray<ESceneComponentIdentifier> FConformingViewportClient::GetVisibleComponentIdentifiers()
{
	TArray<ESceneComponentIdentifier> VisibleIdentifiers;

	if(GetABImageWidget()->IsSingleView())
	{
		EABImageViewMode CurrentMode = GetABImageWidget()->GetViewMode();
		if(ShowConformalMesh[CurrentMode])
		{
			VisibleIdentifiers.Add(ESceneComponentIdentifier::ConformalMesh);
		}
		if(ShowNeutralPoseMesh[CurrentMode])
		{
			VisibleIdentifiers.Add(ESceneComponentIdentifier::ImportedMesh);
		}
		if(ShowRig[CurrentMode])
		{
			VisibleIdentifiers.Add(ESceneComponentIdentifier::ConformalRig);
		}
	}
	else
	{
		if(ShowConformalMesh[EABImageViewMode::A] || ShowConformalMesh[EABImageViewMode::B])
		{
			VisibleIdentifiers.Add(ESceneComponentIdentifier::ConformalMesh);
		}
		if(ShowNeutralPoseMesh[EABImageViewMode::A] || ShowNeutralPoseMesh[EABImageViewMode::B])
		{
			VisibleIdentifiers.Add(ESceneComponentIdentifier::ImportedMesh);
		}
		if(ShowRig[EABImageViewMode::A] || ShowRig[EABImageViewMode::B])
		{
			VisibleIdentifiers.Add(ESceneComponentIdentifier::ConformalRig);
		}
	}

	return VisibleIdentifiers;
}

EABImageViewMode FConformingViewportClient::GetCurrentImageViewMode() const
{
	return GetABImageWidget()->GetViewMode();
}

#undef LOCTEXT_NAMESPACE