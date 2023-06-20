// Copyright Epic Games, Inc.All Rights Reserved.

#include "SConformingAssetEditorViewport.h"
#include "ConformingViewportClient.h"
#include "MetaHumanIdentityCommands.h"
#include "MetaHumanIdentityPromotedFrames.h"
#include "SMetaHumanIdentityPromotedFramesEditor.h"
#include "STrackerImageViewer.h"
#include "SMetaHumanIdentityViewportToolBar.h"
#include "SABImage.h"

#include "EditorViewportCommands.h"
#include "RayTracingDebugVisualizationMenuCommands.h"
#include "Engine/TextureRenderTarget2D.h"

#define LOCTEXT_NAMESPACE "ConformingAssetEditorViewport"


void SConformingAssetEditorViewport::Construct(const FArguments& InArgs, const FAssetEditorViewportConstructionArgs& InViewportConstructionArgs)
{
	OnViewportSizeChanged = InArgs._OnViewportSizeChanged;
	OnCurvesSelectedDelegat = InArgs._OnTrackerViewCurveSelected;
	PromotedFramesEditorWidget = InArgs._PromotedFramesEditorWidget;

	check(PromotedFramesEditorWidget.IsValid());

	CommandListViewB = MakeShared<FUICommandList>();

	if (UTextureRenderTarget2D* RenderTarget = InArgs._RenderTarget)
	{
		TrackerImageBrush.SetImageSize(FVector2D(RenderTarget->SizeX, RenderTarget->SizeY));
		TrackerImageBrush.SetResourceObject(RenderTarget);
	}
	
	TSharedPtr<FConformingViewportClient> ViewportClient = InArgs._EditorViewportClient;
	check(ViewportClient.IsValid());

	SAssignNew(ABImage, SABImage);

	ABImage->Setup(true);
	ABImage->OnInvalidate().AddLambda([this, ViewportClient]
	{
		if (ViewportClient.IsValid())
		{
			ViewportClient->Invalidate();
		}
	});

	ViewportClient->OnMarkerVisibilityChanged().BindLambda([&] (bool InShowCurves, bool InShowPoints)
	{
		if(TrackerImageViewer)
		{
			TrackerImageViewer->SetMarkerVisibility(InShowCurves, InShowPoints);
		}
	});

	ViewportClient->SetEditorViewportWidget(StaticCastSharedRef<SConformingAssetEditorViewport>(AsShared()));

	SAssetEditorViewport::Construct(SAssetEditorViewport::FArguments()
									.EditorViewportClient(ViewportClient),
									InViewportConstructionArgs);

	if (InArgs._ToolkitCommandList.IsValid())
	{
		CommandList->Append(InArgs._ToolkitCommandList.ToSharedRef());
	}

	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot()
		.Padding(0.0f)
		[
			ChildSlot.GetWidget()
		]
		+ SVerticalBox::Slot()
		.AutoHeight()
		[
			SNew(SBox)
			.HeightOverride(28)
			[
				PromotedFramesEditorWidget.ToSharedRef()
			]
		]
	];

	ViewportClient->UpdateSceneComponentVisibility();
}

TSharedRef<SMetaHumanIdentityPromotedFramesEditor> SConformingAssetEditorViewport::GetPromotedFramesEditor() const
{
	check(PromotedFramesEditorWidget.IsValid());
	return PromotedFramesEditorWidget.ToSharedRef();
}

TSharedRef<SABImage> SConformingAssetEditorViewport::GetABImageWidget() const
{
	check(ABImage.IsValid());
	return ABImage.ToSharedRef();
}

void SConformingAssetEditorViewport::Tick(const FGeometry& InAllottedGeometry, const double InCurrentTime, const float InDeltaTime)
{
	SAssetEditorViewport::Tick(InAllottedGeometry, InCurrentTime, InDeltaTime);

	if (ViewportWidget.IsValid())
	{
		const FGeometry& ViewportGeometry = ViewportWidget->GetCachedGeometry();
		if (ViewportGeometry != CurrentViewportGeometry)
		{
			CurrentViewportGeometry = ViewportGeometry;

			const FVector2D& ViewportSize = ViewportGeometry.GetLocalSize();
			if (ViewportSize != FVector2D::ZeroVector)
			{
				OnViewportSizeChanged.ExecuteIfBound(FIntPoint{ static_cast<int32>(ViewportSize.X), static_cast<int32>(ViewportSize.Y) });

				UpdateImageBrush();
			}
		}
	}
}

FReply SConformingAssetEditorViewport::OnKeyDown(const FGeometry& MyGeometry, const FKeyEvent& InKeyEvent)
{
	FReply Reply = FReply::Unhandled();

	TSharedRef<FConformingViewportClient> EditorViewportClient = StaticCastSharedPtr<FConformingViewportClient>(Client).ToSharedRef();
	if (EditorViewportClient->IsShowingSingleView())
	{
		TSharedRef<SABImage> ABImageWidget = GetABImageWidget();
		EABImageViewMode ImageViewMode = ABImageWidget->GetViewMode();
		if (ImageViewMode == EABImageViewMode::A)
		{
			if (CommandList->ProcessCommandBindings(InKeyEvent))
			{
				Reply = FReply::Handled();
				Client->Invalidate();
			}
		}
		else if (ImageViewMode == EABImageViewMode::B)
		{
			if (CommandListViewB->ProcessCommandBindings(InKeyEvent))
			{
				Reply = FReply::Handled();
				Client->Invalidate();
			}
			//the CommandListB doesn't contain Focus and other standard viewport commands (just those needed for B)
			//so if it doesn't process the command, we will process it with the list from the view A that does
			else if (CommandList->ProcessCommandBindings(InKeyEvent))
			{
				Reply = FReply::Handled();
				Client->Invalidate();
			}
		}
	}

	return Reply;
}

void SConformingAssetEditorViewport::BindCommands()
{
	SAssetEditorViewport::BindCommands();

	TSharedRef<FConformingViewportClient> ViewportClient = StaticCastSharedPtr<FConformingViewportClient>(Client).ToSharedRef();

	const FMetaHumanIdentityEditorCommands& Commands = FMetaHumanIdentityEditorCommands::Get();

	CommandList->MapAction(Commands.ViewToggle,
						   FExecuteAction::CreateSP(ViewportClient, &FConformingViewportClient::ToggleAB));

	CommandList->MapAction(Commands.ToggleViewA,
						   FExecuteAction::CreateSP(ViewportClient, &FConformingViewportClient::ToggleAB),
						   FCanExecuteAction{},
						   FIsActionChecked::CreateSP(ViewportClient, &FConformingViewportClient::ShowViewA));
	CommandList->MapAction(Commands.ToggleViewB,
						   FExecuteAction::CreateSP(ViewportClient, &FConformingViewportClient::ToggleAB),
						   FCanExecuteAction{},
						   FIsActionChecked::CreateSP(ViewportClient, &FConformingViewportClient::ShowViewB));

	const FEditorViewportCommands& ViewportCommands = FEditorViewportCommands::Get();

	CommandListViewB->MapAction(ViewportCommands.ToggleAutoExposure,
								FExecuteAction::CreateSP(this, &SConformingAssetEditorViewport::ChangeExposureSetting),
								FCanExecuteAction{},
								FIsActionChecked::CreateSP(this, &SConformingAssetEditorViewport::IsExposureSettingSelected));

	// The code below was adapted from SEditorViewport to map the commands to the respective actions in the viewport client

#define MAP_VIEW_A_VIEWMODE_ACTION( ViewModeCommand, ViewModeID ) \
	CommandList->MapAction( \
		ViewModeCommand, \
		FExecuteAction::CreateSP( ViewportClient, &FConformingViewportClient::SetViewModeIndexForImageViewMode, ViewModeID, EABImageViewMode::A ), \
		FCanExecuteAction(), \
		FIsActionChecked::CreateSP( ViewportClient, &FConformingViewportClient::IsViewModeIndexEnabledForImageViewMode, ViewModeID, EABImageViewMode::A ) )

#define MAP_VIEW_B_VIEWMODE_ACTION( ViewModeCommand, ViewModeID ) \
	CommandListViewB->MapAction( \
		ViewModeCommand, \
		FExecuteAction::CreateSP( ViewportClient, &FConformingViewportClient::SetViewModeIndexForImageViewMode, ViewModeID, EABImageViewMode::B ), \
		FCanExecuteAction(), \
		FIsActionChecked::CreateSP( ViewportClient, &FConformingViewportClient::IsViewModeIndexEnabledForImageViewMode, ViewModeID, EABImageViewMode::B ) )

#define MAP_VIEWMODE_ACTION( ViewModeCommand, ViewModeID )   \
	MAP_VIEW_A_VIEWMODE_ACTION(ViewModeCommand, ViewModeID); \
	MAP_VIEW_B_VIEWMODE_ACTION(ViewModeCommand, ViewModeID)

	MAP_VIEWMODE_ACTION(ViewportCommands.LitMode, VMI_Lit);
	MAP_VIEWMODE_ACTION(ViewportCommands.UnlitMode, VMI_Unlit);
	MAP_VIEWMODE_ACTION(ViewportCommands.LightingOnlyMode, VMI_LightingOnly);
}

void SConformingAssetEditorViewport::ResetTrackerOverlayView(bool bInResetGeometry) const
{
	if (TrackerImageViewer.IsValid())
	{
		TrackerImageViewer->ResetViewOverride(bInResetGeometry);
	}
}

void SConformingAssetEditorViewport::SetShapeAnnotation(const TSharedPtr<FShapeAnnotationWrapper>& InShapeAnnotation) const
{
	if (TrackerImageViewer.IsValid())
	{
		TrackerImageViewer->ReloadTrackingData(InShapeAnnotation);
	}
}

void SConformingAssetEditorViewport::SetCurveSelectionInImageViewer(const TArray<FString>& InSelectedCurves) const
{
	if (TrackerImageViewer.IsValid())
	{
		TrackerImageViewer->SetSelectedCurves(InSelectedCurves);
	}
}

EVisibility SConformingAssetEditorViewport::IsTrackerImageViewerVisible() const
{
	if (PromotedFramesEditorWidget.IsValid())
	{
		if (UMetaHumanIdentityPromotedFrame* PromotedFrame = PromotedFramesEditorWidget->GetSelectedPromotedFrame())
		{
			return EVisibility::Visible;
		}
	}

	return EVisibility::Hidden;
}

bool SConformingAssetEditorViewport::IsTrackerImageViewerNavigationLocked() const
{
	if (PromotedFramesEditorWidget.IsValid())
	{
		if (UMetaHumanIdentityPromotedFrame* PromotedFrame = PromotedFramesEditorWidget->GetSelectedPromotedFrame())
		{
			if (PromotedFrame->IsNavigationLocked())
			{
				return true;
			}
		}
	}

	return false;
}

void SConformingAssetEditorViewport::UpdateImageBrush()
{
	if (ViewportWidget.IsValid())
	{
		TrackerImageBrush.SetImageSize(ViewportWidget->GetCachedGeometry().GetLocalSize());
		ResetTrackerOverlayView();
	}
}

void SConformingAssetEditorViewport::PopulateViewportOverlays(TSharedRef<SOverlay> InOverlay)
{
	InOverlay->AddSlot()
	[
		ABImage.ToSharedRef()
	];

	InOverlay->AddSlot()
	[
		SAssignNew(TrackerImageViewer, STrackerImageViewer)
		.Image(&TrackerImageBrush)
		.Visibility(this, &SConformingAssetEditorViewport::IsTrackerImageViewerVisible)
		.IsNavigationLocked(this, &SConformingAssetEditorViewport::IsTrackerImageViewerNavigationLocked)
		.OnCurvesSelected(OnCurvesSelectedDelegat)
	];

	TrackerImageViewer->SetTrackerImageSize(UMetaHumanIdentityCameraFrame::DefaultTrackerImageSize);
	TrackerImageViewer->SetNonConstBrush(&TrackerImageBrush);
	TrackerImageViewer->OnViewChanged.AddLambda([this](FBox2D InUV)
	{
		TrackerImageBrush.SetUVRegion(InUV);
	});

	// Usually this is done in the MakeViewportToolbar override but because STrackerImageViewer is an overlay that
	// covers the whole screen we need control over the order in which the overlays are stacked in this viewport
	InOverlay->AddSlot()
	.VAlign(VAlign_Top)
	[
		SNew(SMetaHumanIdentityViewportToolBar)
		.CommandListViewA(CommandList)
		.CommandListViewB(CommandListViewB)
		.ViewportClient(StaticCastSharedPtr<FConformingViewportClient>(Client))
	];
}

void SConformingAssetEditorViewport::OnFocusViewportToSelection()
{
	if (Client.IsValid())
	{
		StaticCastSharedPtr<FConformingViewportClient>(Client)->FocusViewportOnSelectedComponents();
	}
}

#undef LOCTEXT_NAMESPACE