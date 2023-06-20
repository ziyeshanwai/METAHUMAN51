// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentityAssetEditorToolkit.h"
#include "MetaHumanIdentityAssetEditor.h"
#include "MetaHumanIdentity.h"
#include "MetaHumanIdentityLog.h"
#include "MetaHumanIdentityParts.h"
#include "MetaHumanIdentityCommands.h"
#include "MetaHumanIdentityPromotedFrames.h"
#include "MetaHumanMeshTrackerModule.h"

#include "UI/MetaHumanIdentityStyle.h"
#include "UI/SMetaHumanIdentityPartsEditor.h"
#include "UI/SMetaHumanIdentityPromotedFramesEditor.h"
#include "UI/SConformingAssetEditorViewport.h"
#include "UI/ConformingViewportClient.h"
#include "UI/SMetaHumanIdentityOutliner.h"

#include "UI/MetaHumanMarkersOutliner.h"

#include "MetaHumanFaceContourTrackerAsset.h"
#include "Nodes/ImageUtilNodes.h"
#include "Nodes/HyprsenseNode.h"
#include "AutoRigService.h"

#include "JsonObjectConverter.h"
#include "Misc/FileHelper.h"

#include "Widgets/Docking/SDockTab.h"
#include "EditorViewportTabContent.h"
#include "AdvancedPreviewScene.h"
#include "Components/PostProcessComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/DynamicMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Kismet/KismetRenderingLibrary.h"
#include "ToolMenus.h"
#include "IContentBrowserSingleton.h"
#include "ContentBrowserModule.h"
#include "AssetToolsModule.h"
#include "ShapeAnnotationWrapper.h"
#include "SWarningOrErrorBox.h"
#include "Misc/MessageDialog.h"
#include "Misc/ScopedSlowTask.h"
#include "Framework/Notifications/NotificationManager.h"
#include "Widgets/Notifications/SNotificationList.h"
#include "Interfaces/IPluginManager.h"
#include "Kismet/GameplayStatics.h"
#include "ScopedTransaction.h"


#define LOCTEXT_NAMESPACE "MetaHumanIdentityToolkit"

const FName FMetaHumanIdentityAssetEditorToolkit::PartsTabId(TEXT("FMetaHumanIdentityAssetEditorToolkit_Parts"));
const FName FMetaHumanIdentityAssetEditorToolkit::OutlinerTabId(TEXT("FMetaHumanIdentityAssetEditorToolkit_Outliner"));

FMetaHumanIdentityAssetEditorToolkit::FMetaHumanIdentityAssetEditorToolkit(UAssetEditor* InOwningAssetEditor)
	: FBaseAssetToolkit{ InOwningAssetEditor }
	, bIsConforming{ false }
	, bIsAutorigging{ false }
{
	// Get a reference to the Identity being edited
	TArray<UObject*> ObjectsToEdit;
	OwningAssetEditor->GetObjectsToEdit(ObjectsToEdit);
	check(!ObjectsToEdit.IsEmpty() && ObjectsToEdit[0] != nullptr);

	Identity = CastChecked<UMetaHumanIdentity>(ObjectsToEdit[0]);

	MarkerData = MakeShared<FMarkerData>();

	if(LoadCurvesAndLandmarksFromJson(TEXT("curves_config.json")) && LoadGroupsFromJson(TEXT("curve_groups.json")))
	{
		SetUpInitialOutlinerData();
	}

	// Register the commands that are used in this editor toolbar
	FMetaHumanIdentityEditorCommands::Register();

	const FString LayoutString = TEXT("Standalone_MetaHumanIdentityAssetEditorToolkit_Layout_v1");
	StandaloneDefaultLayout = FTabManager::NewLayout(FName{ LayoutString })
		->AddArea
		(
			// Create a vertical area and spawn the toolbar
			FTabManager::NewPrimaryArea()->SetOrientation(Orient_Vertical)
			->Split
			(
				FTabManager::NewSplitter()->SetOrientation(Orient_Horizontal)
				->Split
				(
					FTabManager::NewSplitter()->SetOrientation(Orient_Vertical)
					->SetSizeCoefficient(0.35f)
					->Split
					(
						FTabManager::NewStack()
						->SetSizeCoefficient(0.3f)
						->AddTab(PartsTabId, ETabState::OpenedTab)
						->SetHideTabWell(true)
					)
					->Split
					(
						FTabManager::NewStack()
						->SetSizeCoefficient(0.7f)
						->AddTab(DetailsTabID, ETabState::OpenedTab)
						->SetHideTabWell(true)
					)
				)
				->Split
				(
					FTabManager::NewStack()
					->SetSizeCoefficient(0.97f)
					->AddTab(ViewportTabID, ETabState::OpenedTab)
					->SetHideTabWell(true)
				)
				->Split
				(
					FTabManager::NewStack()
					->SetSizeCoefficient(0.3f)
					->AddTab(OutlinerTabId, ETabState::OpenedTab)
					->SetHideTabWell(true)
				)
			)
		);
}

FMetaHumanIdentityAssetEditorToolkit::~FMetaHumanIdentityAssetEditorToolkit()
{

}

FName FMetaHumanIdentityAssetEditorToolkit::GetToolkitFName() const
{
	return TEXT("MetaHumanIdentityAssetEditorToolkit");
}

FText FMetaHumanIdentityAssetEditorToolkit::GetBaseToolkitName() const
{
	return LOCTEXT("BaseToolkitName", "MetaHuman Identity Asset Editor Toolkit");
}

FText FMetaHumanIdentityAssetEditorToolkit::GetToolkitToolTipText() const
{
	return LOCTEXT("ToolkitToolTipText", "MetaHuman Identity Asset Editor");
}

FString FMetaHumanIdentityAssetEditorToolkit::GetWorldCentricTabPrefix() const
{
	return LOCTEXT("WorldCentricTabPrefix", "MetaHuman ").ToString();
}

FLinearColor FMetaHumanIdentityAssetEditorToolkit::GetWorldCentricTabColorScale() const
{
	return FColor::White;
}

bool FMetaHumanIdentityAssetEditorToolkit::IsPrimaryEditor() const
{
	return true;
}

void FMetaHumanIdentityAssetEditorToolkit::CreateWidgets()
{
	FBaseAssetToolkit::CreateWidgets();

	// Replace the DetailsView widget with a custom one that has a notify hook set to this class
	FPropertyEditorModule& PropertyEditorModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>(TEXT("PropertyEditor"));
	FDetailsViewArgs DetailsViewArgs;
	DetailsViewArgs.NameAreaSettings = FDetailsViewArgs::HideNameArea;
	DetailsViewArgs.bHideSelectionTip = true;
	DetailsViewArgs.NotifyHook = this;
	DetailsView = PropertyEditorModule.CreateDetailView(DetailsViewArgs);
}

void FMetaHumanIdentityAssetEditorToolkit::RegisterTabSpawners(const TSharedRef<class FTabManager>& InTabManager)
{
	WorkspaceMenuCategory = InTabManager->AddLocalWorkspaceMenuCategory(LOCTEXT("WorkspaceMenuCategory", "MetaHuman Identity"));

	FBaseAssetToolkit::RegisterTabSpawners(InTabManager);

	InTabManager->RegisterTabSpawner(PartsTabId, FOnSpawnTab::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::SpawnPartsTab))
		.SetDisplayName(LOCTEXT("PartsIdTabName", "Parts"))
		.SetGroup(WorkspaceMenuCategory.ToSharedRef())
		.SetIcon(FSlateIcon{ FMetaHumanIdentityStyle::Get().GetStyleSetName(), TEXT("Identity.Tab.Parts") });

	InTabManager->RegisterTabSpawner(OutlinerTabId, FOnSpawnTab::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::SpawnOutlinerTab))
		.SetDisplayName(LOCTEXT("OutlinerTabName", "Outliner"))
		.SetGroup(WorkspaceMenuCategory.ToSharedRef());
}

void FMetaHumanIdentityAssetEditorToolkit::UnregisterTabSpawners(const TSharedRef<class FTabManager>& InTabManager)
{
	FBaseAssetToolkit::UnregisterTabSpawners(InTabManager);
	InTabManager->UnregisterAllTabSpawners();
}

void FMetaHumanIdentityAssetEditorToolkit::NotifyPostChange(const FPropertyChangedEvent& InPropertyChangedEvent, FProperty* InPropertyThatChanged)
{
	if (IdentityPartsEditor.IsValid())
	{
		IdentityPartsEditor->NotifyPostChange(InPropertyChangedEvent, InPropertyThatChanged);
	}
}

AssetEditorViewportFactoryFunction FMetaHumanIdentityAssetEditorToolkit::GetViewportDelegate()
{
	// Set up functor for viewport tab
	AssetEditorViewportFactoryFunction TempViewportDelegate = [this](const FAssetEditorViewportConstructionArgs& InArgs)
	{
		CreateSceneCaptureComponent();

		TSharedPtr<FConformingViewportClient> ConformingViewportClient = StaticCastSharedPtr<FConformingViewportClient>(ViewportClient);

		ConformingViewportClient->OnRefreshSceneCapture().BindSP(this, &FMetaHumanIdentityAssetEditorToolkit::RefreshSceneCapture);

		SAssignNew(PromotedFramesEditorWidget, SMetaHumanIdentityPromotedFramesEditor)
			.ViewportClient(StaticCastSharedPtr<FConformingViewportClient>(ViewportClient))
			.IdentityPose(SelectedIdentityPose)
			.CommandList(GetToolkitCommands())
			.OnPromotedFrameSelectionChanged(this, &FMetaHumanIdentityAssetEditorToolkit::HandlePromotedFrameSelectedInPromotedFramesPanel)
			.OnPromotedFrameAdded(this, &FMetaHumanIdentityAssetEditorToolkit::HandlePromotedFrameAdded)
			.OnPromotedFrameNavigationLockedChanged(this, &FMetaHumanIdentityAssetEditorToolkit::HandlePromotedFrameNavigationLockedChanged)
			.OnPromotedFrameTrackingModeChanged(this, &FMetaHumanIdentityAssetEditorToolkit::HandlePromotedFrameTrackingModeChanged)
			.OnUpdateControlVerticesPostUndo(this, &FMetaHumanIdentityAssetEditorToolkit::HandleUndoForMarkerManipulation);

		return SAssignNew(ViewportWidget, SConformingAssetEditorViewport, InArgs)
			.PromotedFramesEditorWidget(PromotedFramesEditorWidget)
			.EditorViewportClient(StaticCastSharedPtr<FConformingViewportClient>(ViewportClient))
			.ToolkitCommandList(GetToolkitCommands())
			.RenderTarget(SceneCaptureComponent->TextureTarget)
			.OnViewportSizeChanged(this, &FMetaHumanIdentityAssetEditorToolkit::SetCaptureSceneRenderTargetSize)
			.OnTrackerViewCurveSelected(this, &FMetaHumanIdentityAssetEditorToolkit::HandleSelectionFromViewport);
	};

	return TempViewportDelegate;
}

TSharedPtr<FEditorViewportClient> FMetaHumanIdentityAssetEditorToolkit::CreateEditorViewportClient() const
{
	const float InitialFloorOffset = 250.0f;
	TSharedRef<FPreviewScene> PreviewScene = MakeShared<FAdvancedPreviewScene>(
		FPreviewScene::ConstructionValues(), InitialFloorOffset);

	TSharedRef<FConformingViewportClient> ConformingViewportClient = MakeShared<FConformingViewportClient>(PreviewScene, Identity);
	// The const_cast is required here since this is a const pointer in this function and AddSP requires a non-const object pointer
	ConformingViewportClient->OnCameraStopped().AddSP(const_cast<FMetaHumanIdentityAssetEditorToolkit*>(this), &FMetaHumanIdentityAssetEditorToolkit::HandleCameraStopped);
	return ConformingViewportClient;
}

void FMetaHumanIdentityAssetEditorToolkit::PostInitAssetEditor()
{
	// Add a post processing volume to turn off auto exposure/eye adaptation
	FPostProcessSettings PostProcessSettings;
	PostProcessSettings.bOverride_AutoExposureBias = 1;
	PostProcessSettings.AutoExposureBias = 0;
	PostProcessSettings.bOverride_AutoExposureMinBrightness = 1;
	PostProcessSettings.AutoExposureMinBrightness = 1;
	PostProcessSettings.bOverride_AutoExposureMaxBrightness = 1;
	PostProcessSettings.AutoExposureMaxBrightness = 1;
	PostProcessSettings.bOverride_ToneCurveAmount = 1;
	PostProcessSettings.ToneCurveAmount = 0;
	PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = 1;
	PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::None;

	PostProcessComponent = NewObject<UPostProcessComponent>(GetTransientPackage(), NAME_None, RF_Transient);
	PostProcessComponent->Settings = PostProcessSettings;

	FPreviewScene* PreviewScene = ViewportClient->GetPreviewScene();
	PreviewScene->AddComponent(PostProcessComponent, FTransform::Identity);
	PreviewScene->AddComponent(SceneCaptureComponent, FTransform::Identity);

	BindCommands();
	ExtendMenu();
	ExtendToolBar();

	// Register a delegate to be called when the Mesh Tracker module emits a log message
	FMetaHumanMeshTrackerModule::GetModule().OnLogError().AddSP(this, &FMetaHumanIdentityAssetEditorToolkit::HandleLogErrorMessage);

	// Focus the viewport on the visible components that are in the viewport
	if (ViewportClient.IsValid())
	{
		StaticCastSharedPtr<FConformingViewportClient>(ViewportClient)->FocusViewportOnSelectedComponents();
	}

	// Log us in to the AR service
	AutoRigService::Login(AutoRigService::FAutoRigServiceLoginCompleteDelegate::CreateLambda([]()
	{
		UE_LOG(LogMetaHumanIdentity, Display, TEXT("Logged in to Mesh To MetaHuman service"));
	}));
}

TSharedRef<SDockTab> FMetaHumanIdentityAssetEditorToolkit::SpawnPartsTab(const FSpawnTabArgs& InArgs)
{
	check(Identity);

	return SNew(SDockTab)
		.Label(LOCTEXT("PartsTabTitle", "Parts"))
		[
			SAssignNew(IdentityPartsEditor, SMetaHumanIdentityPartsEditor)
			.Identity(Identity)
			.ViewportClient(StaticCastSharedPtr<FConformingViewportClient>(ViewportClient))
			.OnIdentityTreeSelectionChanged(this, &FMetaHumanIdentityAssetEditorToolkit::HandleIdentityTreeSelectionChanged)
		];
}

TSharedRef<SDockTab> FMetaHumanIdentityAssetEditorToolkit::SpawnOutlinerTab(const FSpawnTabArgs& InArgs)
{
	UMetaHumanIdentityPromotedFrame* SelectedPromotedFrame = nullptr;
	int32 PromotedFrameIndex = INDEX_NONE;

	if (PromotedFramesEditorWidget.IsValid())
	{
		SelectedPromotedFrame = PromotedFramesEditorWidget->GetSelectedPromotedFrame();

		if (UMetaHumanIdentityPose* Pose = PromotedFramesEditorWidget->GetIdentityPose())
		{
			 PromotedFrameIndex = Pose->PromotedFrames.IndexOfByKey(SelectedPromotedFrame);
		}
	}

	return SNew(SDockTab)
		.Label(LOCTEXT("OutlinerTabTitle", "Outliner"))
		[
			SAssignNew(OutlinerWidget, SMetaHumanIdentityOutliner)
			.MarkerDefs(MarkerDefs)
			.PromotedFrame(SelectedPromotedFrame)
			.PromotedFrameIndex(PromotedFrameIndex)
			.OnSelectionChanged(this, &FMetaHumanIdentityAssetEditorToolkit::HandleCurveSelectionChanged)
			.OnResetImageViewerPoints_Lambda([this]
			{
				if(ViewportWidget)
				{
					ViewportWidget->ResetTrackerOverlayView(false);
				}
			})
		];
}

void FMetaHumanIdentityAssetEditorToolkit::BindCommands()
{
	const FMetaHumanIdentityEditorCommands& Commands = FMetaHumanIdentityEditorCommands::Get();

	ToolkitCommands->MapAction(Commands.RigidFitCurrent,
							   FExecuteAction{},
							   FCanExecuteAction::CreateLambda([] { return false; }));

	ToolkitCommands->MapAction(Commands.RigidFitAll,
							   FExecuteAction{},
							   FCanExecuteAction::CreateLambda([] { return false; }));

	ToolkitCommands->MapAction(Commands.TrackCurrent,
							   FExecuteAction::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::HandleTrackCurrent),
							   FCanExecuteAction::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::CanTrackCurrent));

	ToolkitCommands->MapAction(Commands.IdentitySolve,
							   FExecuteAction::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::HandleConform),
							   FCanExecuteAction::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::CanConform));

	ToolkitCommands->MapAction(Commands.SubmitToAutorig,
							   FExecuteAction::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::HandleSubmitToAutoRigging),
							   FCanExecuteAction::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::CanSubmitToAutoRigging));

	ToolkitCommands->MapAction(Commands.ResetTemplateMesh,
							   FExecuteAction::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::HandleResetTemplateMesh),
							   FCanExecuteAction::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::CanResetTemplateMesh));
}

void FMetaHumanIdentityAssetEditorToolkit::ExtendMenu()
{
	const FMetaHumanIdentityEditorCommands& Commands = FMetaHumanIdentityEditorCommands::Get();

	const FName FitMenuName = UToolMenus::JoinMenuPaths(GetToolMenuAppName(), TEXT("Fit"));
	const FName TrackMenuName = UToolMenus::JoinMenuPaths(GetToolMenuAppName(), TEXT("Track"));
	const FName SolveMenuName = UToolMenus::JoinMenuPaths(GetToolMenuAppName(), TEXT("Solve"));

	UToolMenus* ToolMenus = UToolMenus::Get();

	if (!ToolMenus->IsMenuRegistered(TrackMenuName))
	{
		UToolMenu* TrackMenu = ToolMenus->RegisterMenu(TrackMenuName);

		FToolMenuSection& TrackingSection = TrackMenu->AddSection(TEXT("TrackMenuTrackers"), LOCTEXT("TrackMenuTrackersSection", "Trackers"));
		{
			TrackingSection.AddMenuEntry(Commands.TrackCurrent);
		}
	}

	if (!ToolMenus->IsMenuRegistered(SolveMenuName))
	{
		UToolMenu* SolveMenu = ToolMenus->RegisterMenu(SolveMenuName);

		FToolMenuSection& SolveSection = SolveMenu->AddSection(TEXT("SolveMenuLocalSolve"), LOCTEXT("SolveMenuLocalSolveSection", "Local Solve"));
		{
			SolveSection.AddMenuEntry(Commands.IdentitySolve);
		}
	}

	const FName TrackMainMenuName = UToolMenus::JoinMenuPaths(GetToolMenuName(), TEXT("Track"));
	const FName SolveMainMenuName = UToolMenus::JoinMenuPaths(GetToolMenuName(), TEXT("Solve"));

	if (!ToolMenus->IsMenuRegistered(TrackMainMenuName))
	{
		ToolMenus->RegisterMenu(TrackMainMenuName, TrackMenuName);
	}

	if (!ToolMenus->IsMenuRegistered(SolveMainMenuName))
	{
		ToolMenus->RegisterMenu(SolveMainMenuName, SolveMenuName);
	}

	if (UToolMenu* MainMenu = ToolMenus->ExtendMenu(GetToolMenuName()))
	{
		const FToolMenuInsert MenuInsert{ TEXT("Tools"), EToolMenuInsertType::After };

		FToolMenuSection& Section = MainMenu->FindOrAddSection(NAME_None);

		FToolMenuEntry& TrackEntry = Section.AddSubMenu(TEXT("Track"),
														LOCTEXT("IdentityEditorTrackMenuLabel", "Track"),
														LOCTEXT("IdentityEditorTrackMenuTooltip", "Tools for Tracking"),
														FNewToolMenuChoice{});
		TrackEntry.InsertPosition = MenuInsert;

		FToolMenuEntry& SolveEntry = Section.AddSubMenu(TEXT("Solve"),
														LOCTEXT("IdentityEditorSolveMenuLabel", "Solve"),
														LOCTEXT("IdentityEditorSolveMenuTooltip", "Tools for MetaHuman Identity Solve"),
														FNewToolMenuChoice{});
		SolveEntry.InsertPosition = MenuInsert;
	}

	const FName AssetMainMenuName = UToolMenus::JoinMenuPaths(GetToolMenuName(), TEXT("Asset"));
	if (UToolMenu* AssetMenu = ToolMenus->ExtendMenu(AssetMainMenuName))
	{
		FToolMenuSection& Section = AssetMenu->AddSection(TEXT("MetaHumanIdentityAssetActions"), LOCTEXT("MetaHumanIdentityAssetActionsSection", "MetaHuman Identity"));

		Section.AddMenuEntry(Commands.ResetTemplateMesh);
	}
}

void FMetaHumanIdentityAssetEditorToolkit::ExtendToolBar()
{
	const FMetaHumanIdentityEditorCommands& Commands = FMetaHumanIdentityEditorCommands::Get();

	UToolMenus* ToolMenus = UToolMenus::Get();

	if (UToolMenu* ToolBarMenu = ToolMenus->ExtendMenu(GetToolMenuToolbarName()))
	{
		FToolMenuSection& IdentityToolsSection = ToolBarMenu->AddSection(TEXT("MetaHumanIdentityTools"));
		{

			const FUIAction AddFaceFromMeshAction(FExecuteAction{},
												  FCanExecuteAction::CreateLambda([this]
												  {
													return Identity->FindPartOfClass<UMetaHumanIdentityFace>() == nullptr;
												  }));
			const bool bSimpleComboBox = false;
			IdentityToolsSection.AddEntry(FToolMenuEntry::InitComboButton(TEXT("AddFaceFromMeshToolButton"),
																		  FToolUIActionChoice{ AddFaceFromMeshAction },
																		  FNewToolMenuDelegate::CreateSP(this, &FMetaHumanIdentityAssetEditorToolkit::MakeStaticMeshAssetPickerMenu),
																		  LOCTEXT("AddFaceFromMeshToolButtonLabel", "Components from Mesh"),
																		  LOCTEXT("AddFaceFromMeshToolButtonTooltip", "Create a Capture Data (Mesh) and a Face part from a Static Mesh with a Neutral Pose"),
																		  FSlateIcon{ FMetaHumanIdentityStyle::Get().GetStyleSetName(), TEXT("Identity.Tools.ComponentsFromMesh") },
																		  bSimpleComboBox));

			IdentityToolsSection.AddEntry(FToolMenuEntry::InitToolBarButton(Commands.PromoteFrame));
			IdentityToolsSection.AddEntry(FToolMenuEntry::InitToolBarButton(Commands.TrackCurrent, LOCTEXT("TrackCurrentFrameLabelOverride", "Track Active Frame")));
			IdentityToolsSection.AddEntry(FToolMenuEntry::InitToolBarButton(Commands.IdentitySolve));
		}

		FToolMenuSection& AutoRiggingSection = ToolBarMenu->AddSection(TEXT("AutoRigging"));
		{
			AutoRiggingSection.AddEntry(FToolMenuEntry::InitToolBarButton(Commands.SubmitToAutorig));
		}
	}
}

void FMetaHumanIdentityAssetEditorToolkit::CreateSceneCaptureComponent()
{
	// Create the SceneCaptureComponent used to read the scene as a texture for tracking
	UTextureRenderTarget2D* RenderTarget = NewObject<UTextureRenderTarget2D>(GetTransientPackage(), NAME_None, RF_Transient);
	RenderTarget->InitAutoFormat(UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.X, UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.Y);
	RenderTarget->UpdateResourceImmediate(false);

	SceneCaptureComponent = NewObject<USceneCaptureComponent2D>(GetTransientPackage(), NAME_None, RF_Transient);
	SceneCaptureComponent->TextureTarget = RenderTarget;
	SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
	SceneCaptureComponent->bCaptureEveryFrame = false;
	SceneCaptureComponent->bCaptureOnMovement = false;
	SceneCaptureComponent->bAlwaysPersistRenderingState = true;
}

void FMetaHumanIdentityAssetEditorToolkit::HandleIdentityTreeSelectionChanged(UObject* InObject)
{
	DetailsView->SetObject(InObject);

	SelectedIdentityPose = Cast<UMetaHumanIdentityPose>(InObject);

	if (PromotedFramesEditorWidget.IsValid())
	{
		PromotedFramesEditorWidget->SetIdentityPose(SelectedIdentityPose.IsValid() ? SelectedIdentityPose.Get() : nullptr);
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandlePromotedFrameTrackingModeChanged(class UMetaHumanIdentityPromotedFrame* InPromotedFrame)
{
	if (PromotedFramesEditorWidget.IsValid()
		&& InPromotedFrame == PromotedFramesEditorWidget->GetSelectedPromotedFrame()
		&& InPromotedFrame->IsTrackingOnChange())
	{
		HandleCameraStopped();
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandleUndoForMarkerManipulation()
{
	if(ViewportWidget.IsValid())
	{
		ViewportWidget->ResetTrackerOverlayView(false);
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandlePromotedFrameNavigationLockedChanged(class UMetaHumanIdentityPromotedFrame* InPromotedFrame)
{
	if (InPromotedFrame != nullptr && InPromotedFrame->IsNavigationLocked() &&
		ViewportWidget.IsValid() &&
		PromotedFramesEditorWidget.IsValid() && PromotedFramesEditorWidget->GetSelectedPromotedFrame() == InPromotedFrame)
	{
		// Update the image brush first so we have a render target of the correct size
		ViewportWidget->UpdateImageBrush();

		// Now capture the scene for the promoted frame
		CaptureSceneForPromotedFrame(InPromotedFrame);
	}

	ViewportWidget->ResetTrackerOverlayView();
}

void FMetaHumanIdentityAssetEditorToolkit::HandlePromotedFrameSelectedInPromotedFramesPanel(UMetaHumanIdentityPromotedFrame* InPromotedFrame, bool bForceNotify) const
{
	for (TSharedPtr<FMarkerOutlinerFrameData> FrameItem : MarkerData->MarkerFrames)
	{
		FrameItem->bSelected = false;
	}

	if (OutlinerWidget.IsValid())
	{
		if (InPromotedFrame != nullptr)
		{
			if (UMetaHumanIdentityFace* Face = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
			{
				if (UMetaHumanIdentityPose* NeutralPose = Face->FindPoseByType(EIdentityPoseType::Neutral))
				{
					const int32 FrameIndex = NeutralPose->PromotedFrames.IndexOfByKey(InPromotedFrame);
					OutlinerWidget->SetPromotedFrame(InPromotedFrame, FrameIndex);
				}
			}
		}
		else
		{
			// Clear the outliner tree view
			OutlinerWidget->SetPromotedFrame(nullptr, INDEX_NONE);
		}
	}

	if (InPromotedFrame != nullptr)
	{		
		RefreshViewportDueToFrameSelectionChange(InPromotedFrame);
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandlePromotedFrameSelectedInOutliner(UMetaHumanIdentityPromotedFrame* InPromotedFrame, bool bForceNotify) const
{
	if (InPromotedFrame != nullptr)
	{
		RefreshViewportDueToFrameSelectionChange(InPromotedFrame);

		for (TSharedPtr<FMarkerOutlinerFrameData> FrameItem : MarkerData->MarkerFrames)
		{
			if (FrameItem->Frame == InPromotedFrame)
			{
				PromotedFramesEditorWidget->SetSelection(FrameItem->FrameNumber, bForceNotify);
				break;
			}
		}
	}
	else
	{
		PromotedFramesEditorWidget->SetSelection(INDEX_NONE, bForceNotify);
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandleCurveSelectionChanged(const TArray<FString>& InSelectedCurves) const
{
	if (ViewportWidget.IsValid())
	{
		ViewportWidget->SetCurveSelectionInImageViewer(InSelectedCurves);
		ViewportWidget->ResetTrackerOverlayView(false);
	}
}

void FMetaHumanIdentityAssetEditorToolkit::RefreshViewportDueToFrameSelectionChange(class UMetaHumanIdentityPromotedFrame* InPromotedFrame) const
{
	ViewportWidget->SetShapeAnnotation(InPromotedFrame->GetShapeAnnotationWrapper());
	
	CaptureSceneForPromotedFrame(InPromotedFrame);
	ViewportWidget->ResetTrackerOverlayView();
}

void FMetaHumanIdentityAssetEditorToolkit::HandlePromotedFrameAdded(UMetaHumanIdentityPromotedFrame* InPromotedFrame) const
{
	if (UMetaHumanIdentityFace* Face = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
	{
		CaptureSceneForPromotedFrame(InPromotedFrame);

		const TArray<FVector> Vertices = Face->GetConformalVerticesWorldPos();
		const FFrameTrackingContourData Contours = ProjectPromotedFrameCurvesOnTemplateMesh(InPromotedFrame, Vertices);

		InPromotedFrame->InitializeMarkersFromParsedConfig(Contours);
		ViewportWidget->SetShapeAnnotation(InPromotedFrame->GetShapeAnnotationWrapper());
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandleSelectionFromViewport(const TArray<FString>& InCurveNames) const
{
	if (PromotedFramesEditorWidget.IsValid() && OutlinerWidget.IsValid())
	{
		if (UMetaHumanIdentityPromotedFrame* PromotedFrame = PromotedFramesEditorWidget->GetSelectedPromotedFrame())
		{
			OutlinerWidget->ClearSelection();

			TMap<FString, FTrackingContour>& CountourData = PromotedFrame->ContourData.TrackingContours;
			for (TPair<FString, FTrackingContour>& CurveData : CountourData)
			{
				const FString& CurveName = CurveData.Key;
				FTrackingContour& Contour = CurveData.Value;

				Contour.State.bSelected = InCurveNames.Contains(CurveName);
			}

			OutlinerWidget->RefreshSelection();
		}
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandleCameraStopped()
{
	if (PromotedFramesEditorWidget.IsValid())
	{
		if(UMetaHumanIdentityFace* Face = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
		{
			const TArray<FVector> TemplateMeshVertices = Face->GetConformalVerticesWorldPos();
			if (UMetaHumanIdentityPromotedFrame* PromotedFrame = PromotedFramesEditorWidget->GetSelectedPromotedFrame())
			{
				PromotedFrame->ModifyContourData(ProjectPromotedFrameCurvesOnTemplateMesh(PromotedFrame, TemplateMeshVertices));
				ViewportWidget->SetShapeAnnotation(PromotedFrame->GetShapeAnnotationWrapper());

				if (PromotedFrame->IsTrackingOnChange())
				{
					HandleTrackCurrent();
				}
			}
		}
	}
}

void FMetaHumanIdentityAssetEditorToolkit::GetProjectedScreenCoordinates(const TArray<FVector>& InWorldPositions, TArray<FVector2d>& OutScreenPositions) const
{
	TArray<bool> ValidFlags;
	FMinimalViewInfo ViewInfo;
	TOptional<FMatrix> CustomProjectionMatrix;

	SceneCaptureComponent->GetCameraView(0.f, ViewInfo);
	ViewInfo.AspectRatio = UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.X / UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.Y;
	const FIntRect ViewRect = FIntRect(0, 0, UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.X, UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.Y);
	FMatrix CaptureViewMatrix, CaptureProjectionMatrix, CaptureViewProjectionMatrix;
	UGameplayStatics::CalculateViewProjectionMatricesFromMinimalView(ViewInfo, CustomProjectionMatrix,
		CaptureViewMatrix, CaptureProjectionMatrix, CaptureViewProjectionMatrix);

	FVector2d OutScreenPosition;
	for(const FVector& WorldPosition : InWorldPositions)
	{
		if(FSceneView::ProjectWorldToScreen(WorldPosition, ViewRect, CaptureViewProjectionMatrix, OutScreenPosition))
		{
			OutScreenPositions.Add(OutScreenPosition);
		}
	}
}

FFrameTrackingContourData FMetaHumanIdentityAssetEditorToolkit::ProjectPromotedFrameCurvesOnTemplateMesh(class UMetaHumanIdentityPromotedFrame* InPromotedFrame, const TArray<FVector>& InTemplateMeshVertices) const
{
	FFrameTrackingContourData Contours;

	if (UMetaHumanIdentityCameraFrame* CameraFrame = Cast<UMetaHumanIdentityCameraFrame>(InPromotedFrame))
	{
		// Set this because GetProjectedScreenCoordinates calculates the projection based in the screen capture component view transform
		SceneCaptureComponent->FOVAngle = CameraFrame->CameraViewFOV;
		SceneCaptureComponent->SetWorldTransform(CameraFrame->GetCameraTransform());

		// Populate Countours with "curves" parsed in config
		for (const FMarkerCurveDef& Curve : MarkerDefs->CurveDefs)
		{
			TArray<FVector> FeatureVerts;
			for (const int32 ID : Curve.VertexIDs)
			{
				FeatureVerts.Add(InTemplateMeshVertices[ID]);
			}

			TArray<FVector2d> ScreenCoordinates;
			GetProjectedScreenCoordinates(FeatureVerts, ScreenCoordinates);

			FTrackingContour TrackingContour;
			TrackingContour.DensePoints = ScreenCoordinates;
			TrackingContour.StartPointName = Curve.StartPointName;
			TrackingContour.EndPointName = Curve.EndPointName;
			Contours.TrackingContours.Add(Curve.Name, TrackingContour);
		}

		for (const TPair<FString, int32>& Landmark : MarkerDefs->Landmarks)
		{
			TArray<FVector2d> ScreenCoordinates;
			GetProjectedScreenCoordinates({ InTemplateMeshVertices[Landmark.Value] }, ScreenCoordinates);
			FTrackingContour TrackingContour;
			TrackingContour.DensePoints = ScreenCoordinates;
			Contours.TrackingContours.Add(Landmark.Key, TrackingContour);
		}
	}

	return Contours;
}

void FMetaHumanIdentityAssetEditorToolkit::HandleLogErrorMessage(ELogVerbosity::Type InLogVerbosity, const FString& InMessage)
{
	if (bIsConforming)
	{
		LogMessages.Emplace(InMessage.TrimStartAndEnd(), InLogVerbosity);
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandleTrackCurrent()
{
	if (UMetaHumanIdentityPromotedFrame* PromotedFrame = PromotedFramesEditorWidget->GetSelectedPromotedFrame())
	{
		// Only create a transaction if we are tracking manually
		const bool bShouldTransact = PromotedFrame->IsTrackingManually();
		const FScopedTransaction Transaction{ UMetaHumanIdentity::IdentityTransactionContext, LOCTEXT("TrackCurrentTrasactionLabel", "Track Promoted Frame"), PromotedFrame, bShouldTransact };

		PromotedFrame->Modify();

		const FIntPoint CurrentRenderTargetSize{ SceneCaptureComponent->TextureTarget->SizeX, SceneCaptureComponent->TextureTarget->SizeY };
		SetCaptureSceneRenderTargetSize(UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize);

		const bool bCaptureForTracking = true;
		CaptureSceneForPromotedFrame(PromotedFrame, bCaptureForTracking);

		TArray<FColor> LocalSamples;
		if (UKismetRenderingLibrary::ReadRenderTarget(SceneCaptureComponent->TextureTarget, SceneCaptureComponent->TextureTarget, LocalSamples))
		{
			TrackPromotedFrame(PromotedFrame, LocalSamples);
		}
		else
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("Failed to read image for tracking from Promoted Frame '%s'"), *PromotedFrame->GetName());
		}

		// If tracking manually lock the navigation after tracking
		if (PromotedFrame->IsTrackingManually())
		{
			PromotedFrame->SetNavigationLocked(true);
		}

		SetCaptureSceneRenderTargetSize(CurrentRenderTargetSize);
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandleConform()
{
	if (UMetaHumanIdentityFace* Face = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
	{

		if (!ActiveCurvesAreValidForConforming())
		{
			const FText MessageText = LOCTEXT("UnableToConformMeshMessage", "Some active curves are placed outside the promoted frame area.");
			const FText TitleText = LOCTEXT("CurvesInvalid", "Unable to solve");
			FMessageDialog::Open(EAppMsgType::Ok, MessageText, &TitleText);
			return;
		}
		
		FGuardValue_Bitfield(bIsConforming, true);

		const FScopedTransaction Transaction{ UMetaHumanIdentity::IdentityTransactionContext, LOCTEXT("IdentitySolveTransaction", "MetaHuman Identity Solve"), Identity };
		Face->Modify();
		Face->Conform();

		ProcessLogMessages();

		// Reproject the the points for each Promoted Frame so they are shown on top of the template mesh
		if (Face->bIsConformed)
		{
			if (IdentityPartsEditor.IsValid())
			{
				// Call NotifyMeshChangedUpdate in the Template Mesh that is being displayed so the new conformed mesh is reflected in the viewport
				const bool bInstance = true;
				if (UDynamicMeshComponent* TemplateMeshComponentInstance = Cast<UDynamicMeshComponent>(IdentityPartsEditor->GetSceneComponentOfType(ESceneComponentIdentifier::ConformalMesh, bInstance)))
				{
					TemplateMeshComponentInstance->NotifyMeshUpdated();
				}
			}

			if (UMetaHumanIdentityPose* NeutralPose = Face->FindPoseByType(EIdentityPoseType::Neutral))
			{
				const TArray<FVector> TemplateMeshVertices = Face->GetConformalVerticesWorldPos();

				for (UMetaHumanIdentityPromotedFrame* PromotedFrame : NeutralPose->PromotedFrames)
				{
					FFrameTrackingContourData ReprojectedContours = ProjectPromotedFrameCurvesOnTemplateMesh(PromotedFrame, TemplateMeshVertices);

					for(const TPair<FString, FTrackingContour>& Contours : PromotedFrame->ContourData.TrackingContours)
					{
						if(Contours.Value.State.bActive)
						{
							ReprojectedContours.TrackingContours.Remove(Contours.Key);
						}
					}

					if (ReprojectedContours.ContainsData())
					{
						PromotedFrame->AddShapeAnnotationInitializationForContours(ReprojectedContours);
					}
				}
			}
		}
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandleResetTemplateMesh()
{
	if (UMetaHumanIdentityFace* Face = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
	{
		const FScopedTransaction Transaction{ LOCTEXT("ResetTemplateMeshTransaction", "Reset Template Mesh" ) };
		Face->Modify();
		Face->ConformalMeshComponent->Modify();
		Face->ResetTemplateMesh();
		Face->bIsConformed = false;

		if (IdentityPartsEditor.IsValid())
		{
			// Notify the template mesh component that the mesh was updated
			const bool bInstance = true;
			if (UDynamicMeshComponent* TemplateMeshComponentInstance = Cast<UDynamicMeshComponent>(IdentityPartsEditor->GetSceneComponentOfType(ESceneComponentIdentifier::ConformalMesh, bInstance)))
			{
				TemplateMeshComponentInstance->NotifyMeshUpdated();
			}
		}
	}
}

void FMetaHumanIdentityAssetEditorToolkit::HandleSubmitToAutoRigging()
{
	if (bIsAutorigging)
	{
		UE_LOG(LogMetaHumanIdentity, Error, TEXT("Autorigging service is already running for this MetaHuman Identity"));
		return;
	}

	if (UMetaHumanIdentityFace* Face = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
	{
		if (Face->bIsConformed)
		{
			TArray<FVector> ConformalVertices = Face->GetConformalVerticesForAutoRigging();
			if (!ConformalVertices.IsEmpty())
			{
				AutoRigService::TargetSolveParameters Params;
				Params.IdentityName = Identity->GetName();

				if (const UMetaHumanIdentityBody* Body = Identity->FindPartOfClass<UMetaHumanIdentityBody>())
				{
					if (Body->BodyTypeIndex == -1)
					{
						const FText Title = LOCTEXT("ARSErrorTitle", "Mesh to MetaHuman Error");
						FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("ARNoBodyType", "No Body Type Preset is selected in the Body Part. Please select a Body Type Preset to continue."), &Title);
						return;
					}

					Params.Height = Body->Height;
					Params.BodyTypeIndex = Body->BodyTypeIndex;
				}
				else
				{
					const FText Title = LOCTEXT("ARSErrorTitle", "Mesh to MetaHuman Error");
					FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("ARNoBody", "Mesh to MetaHuman requires the addition of a Body Part, and a Body Type Preset selection."), &Title);
					return;
				}

				bIsAutorigging = true;

				FNotificationInfo Info(LOCTEXT("AutoRigProgressText", "Waiting for MetaHuman backend..."));
				Info.bFireAndForget = false;
				AutoRigProgressNotification = FSlateNotificationManager::Get().AddNotification(Info);
				if (AutoRigProgressNotification.IsValid())
				{
					AutoRigProgressNotification.Pin()->SetCompletionState(SNotificationItem::CS_Pending);
				}

				SolveForTarget(MoveTemp(ConformalVertices),
					Params,
					AutoRigService::FAutoRigServiceSolveCompleteDelegate::CreateLambda(
						[this](const TArray<uint8>& Dna)
						{
							UE_LOG(LogMetaHumanIdentity, Display, TEXT("Autorigging service executed, we got %d bytes of DNA back"), Dna.Num());
							if (UMetaHumanIdentityFace* Face = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
							{
								Face->ApplyDNAToRig(const_cast<TArray<uint8>&>(Dna));
							}

							const FText Title = LOCTEXT("ARSSuccessTitle", "Mesh to MetaHuman");
							FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("ARSSuccess", "Your MetaHuman is now available in Creator and Bridge, in the My MetaHumans section"), &Title);

							bIsAutorigging = false;

							AutoRigProgressEnd(true);
						}),
					AutoRigService::FAutoRigServiceSolveFailureDelegate::CreateLambda(
						[this](AutoRigService::ESolveRequestResult ResultCode)
						{
							FText ErrorText;
							switch (ResultCode)
							{
							case AutoRigService::ESolveRequestResult::Busy:
								ErrorText = LOCTEXT("ARSBusy", "Mesh to MetaHuman service is busy, try again later");
								break;
							case AutoRigService::ESolveRequestResult::Unauthorized:
								ErrorText = LOCTEXT("ARSUnauthorized", "You are not authorized to use the Mesh to MetaHuman service");
								break;
							case AutoRigService::ESolveRequestResult::EulaNotAccepted:
								ErrorText = LOCTEXT("ARSEulaNotAccepted", "Mesh to MetaHuman EULA was not accepted");
								break;
							case AutoRigService::ESolveRequestResult::InvalidArguments:
								ErrorText = LOCTEXT("ARSInvalidArguments", "Mesh to MetaHuman service invoked with invalid arguments");
								break;
							case AutoRigService::ESolveRequestResult::ServerError:
								ErrorText = LOCTEXT("ARSServerError", "Error while executing the Mesh to MetaHuman service");
								break;
							case AutoRigService::ESolveRequestResult::LoginFailed:
								ErrorText = LOCTEXT("ARSServerError", "Unable to log in successfully");
								break;
							default:
								ErrorText = LOCTEXT("ARSUnknownErorr", "Unknown error while executing the Mesh to MetaHuman service");
								break;
							}

							const FText Title = LOCTEXT("ARSErrorTitle", "Mesh to MetaHuman Error");
							FMessageDialog::Open(EAppMsgType::Ok, ErrorText, &Title);

							UE_LOG(LogMetaHumanIdentity, Error, TEXT("Autorigging service returned an error: '%s'"), *ErrorText.ToString());

							bIsAutorigging = false;

							AutoRigProgressEnd(false);
						}));
			}
			else
			{
				UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error submitting to autorig. ConformalMesh has no vertices to submit in the MetaHuman Identity %s"), *Identity->GetName());
			}
		}
		else
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error submitting to autorig. Face mesh was not conformed in the MetaHuman Identity %s"), *Identity->GetName());
		}
	}
	else
	{
		UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error submitting to autorig. Face Part not found in the MetaHuman Identity %s"), *Identity->GetName());
	}
}

void FMetaHumanIdentityAssetEditorToolkit::AutoRigProgressEnd(bool bSuccess) const
{
	if (AutoRigProgressNotification.IsValid())
	{
		if (bSuccess)
		{
			AutoRigProgressNotification.Pin()->SetText(LOCTEXT("AutoRigProgressComplete", "Mesh to MetaHuman complete!"));
		}
		else
		{
			AutoRigProgressNotification.Pin()->SetText(LOCTEXT("AutoRigProgressFailed", "Mesh to MetaHuman failed!"));
		}
		AutoRigProgressNotification.Pin()->SetCompletionState(SNotificationItem::CS_None);
		AutoRigProgressNotification.Pin()->ExpireAndFadeout();
	}
}

bool FMetaHumanIdentityAssetEditorToolkit::CanActivateMarkersForCurrent() const
{
	if (PromotedFramesEditorWidget.IsValid())
	{
		if (UMetaHumanIdentityPromotedFrame* SelectedPromotedFrame = PromotedFramesEditorWidget->GetSelectedPromotedFrame())
		{
			return SelectedPromotedFrame->FrameContoursContainActiveData() && !SelectedPromotedFrame->bUseToSolve;
		}
	}

	return false;
}

bool FMetaHumanIdentityAssetEditorToolkit::CanActivateMarkersForAll() const
{
	if (PromotedFramesEditorWidget.IsValid())
	{
		if (UMetaHumanIdentityPose* Pose = PromotedFramesEditorWidget->GetIdentityPose())
		{
			for (UMetaHumanIdentityPromotedFrame* PromotedFrame : Pose->PromotedFrames)
			{
				if (PromotedFrame != nullptr)
				{
					if (PromotedFrame->FrameContoursContainActiveData() && !PromotedFrame->bUseToSolve)
					{
						return true;
					}
				}
			}
		}
	}

	return false;
}

bool FMetaHumanIdentityAssetEditorToolkit::CanTrackCurrent() const
{
	if (TrackPipeline.IsRunning())
	{
		return false;
	}

	if (PromotedFramesEditorWidget.IsValid())
	{
		if (UMetaHumanIdentityPromotedFrame* PromotedFrame = PromotedFramesEditorWidget->GetSelectedPromotedFrame())
		{
			return PromotedFrame->CanTrack();
		}
	}

	return false;
}

bool FMetaHumanIdentityAssetEditorToolkit::CanTrackAll() const
{
	if (TrackPipeline.IsRunning())
	{
		return false;
	}

	if (PromotedFramesEditorWidget.IsValid())
	{
		if (UMetaHumanIdentityPose* Pose = PromotedFramesEditorWidget->GetIdentityPose())
		{
			for (UMetaHumanIdentityPromotedFrame* PromotedFrame : Pose->PromotedFrames)
			{
				if (PromotedFrame->CanTrack())
				{
					// If at least one Promoted Frame can be tracked we enable the Track All button
					return true;
				}
			}
		}
	}

	return false;
}

bool FMetaHumanIdentityAssetEditorToolkit::CanConform() const
{
	if (UMetaHumanIdentityFace* Face = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
	{
		return Face->CanConform();
	}

	return false;
}

bool FMetaHumanIdentityAssetEditorToolkit::CanResetTemplateMesh() const
{
	// If we can conform we can reset
	return CanConform();
}

bool FMetaHumanIdentityAssetEditorToolkit::CanSubmitToAutoRigging() const
{
	if (bIsAutorigging)
	{
		// Don't allow multiple submissions to the autorigging service
		return false;
	}

	if (UMetaHumanIdentityFace* Face = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
	{
		// Only enables AutoRigging if the Face was conformed successfully
		return Face->bIsConformed;
	}

	// NOTE: if the user is not logged in or has not accepted the EULA the solve request will trigger that flow itself.  
	return false;
}

bool FMetaHumanIdentityAssetEditorToolkit::ActiveCurvesAreValidForConforming() const
{
	FBox2D TexCanvas = FBox2D(FVector2D(0,0), FVector2D(UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.X, UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.Y));

	if (UMetaHumanIdentityPose* Pose = PromotedFramesEditorWidget->GetIdentityPose())
	{
		for (const UMetaHumanIdentityPromotedFrame* PromotedFrame : Pose->PromotedFrames)
		{
			for (const TPair<FString, FTrackingContour> &Contour : PromotedFrame->ContourData.TrackingContours)
			{
				if (Contour.Value.State.bActive)
				{
					for (const FVector2D& Point : Contour.Value.DensePoints)
					{
						if (!TexCanvas.IsInside(Point))
						{
							return false;
						}
					}
					
				}
			}
		}
	}

	return true;
}

void FMetaHumanIdentityAssetEditorToolkit::TrackPromotedFrame(UMetaHumanIdentityPromotedFrame* InPromotedFrame, const TArray<FColor>& InImageData)
{
	if (TrackPipeline.IsRunning())
	{
		return;
	}

	const TSharedPtr<UE::MetaHuman::Pipeline::FFColorToUEImageNode> UEImage = TrackPipeline.MakeNode<UE::MetaHuman::Pipeline::FFColorToUEImageNode>("RenderTarget");
	const TSharedPtr<UE::MetaHuman::Pipeline::FHyprsenseNode> GenericTracker = TrackPipeline.MakeNode<UE::MetaHuman::Pipeline::FHyprsenseNode>("GenericTracker");

	UEImage->Samples = InImageData;
	UEImage->Width = UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.X;
	UEImage->Height = UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.Y;

	UMetaHumanFaceContourTrackerAsset* FaceContourTracker = InPromotedFrame->ContourTracker;

	const bool bShowProgress = true;

	FaceContourTracker->LoadTrackers(bShowProgress, [=](bool bTrackersLoaded)
	{
		check(bTrackersLoaded);

		verify(GenericTracker->SetTrackers(FaceContourTracker->FullFaceTracker.Get(),
										   FaceContourTracker->FaceDetector.Get(),
										   FaceContourTracker->BrowsDenseTracker.Get(),
										   FaceContourTracker->EyesDenseTracker.Get(),
										   FaceContourTracker->MouthDenseTracker.Get(),
										   FaceContourTracker->NasioLabialsDenseTracker.Get()));

		const FString TrackingResultsPinName = GenericTracker->Name + ".Contours Out";

		TrackPipeline.MakeConnection(UEImage, GenericTracker);

		UE::MetaHuman::Pipeline::FFrameComplete OnFrameComplete;
		UE::MetaHuman::Pipeline::FProcessComplete OnProcessComplete;

		OnFrameComplete.AddLambda([this, InPromotedFrame, TrackingResultsPinName](const TSharedPtr<UE::MetaHuman::Pipeline::FPipelineData> InPipelineData)
		{
			const FFrameTrackingContourData& TrackedData = InPipelineData->GetData<FFrameTrackingContourData>(TrackingResultsPinName);
			UpdateCurveStateForFrameAfterTracking(TrackedData, InPromotedFrame);
			InPromotedFrame->AddShapeAnnotationInitializationForContours(TrackedData);
			ViewportWidget->SetShapeAnnotation(InPromotedFrame->GetShapeAnnotationWrapper());
		});

		OnProcessComplete.AddLambda([this](TSharedPtr<UE::MetaHuman::Pipeline::FPipelineData> InPipelineData)
		{
			TrackPipeline.Reset();
		});

		UE::MetaHuman::Pipeline::FPipelineRunParameters PipelineRunParameters;
		PipelineRunParameters.SetStartFrame(0);
		PipelineRunParameters.SetEndFrame(1);
		PipelineRunParameters.SetOnFrameComplete(OnFrameComplete);
		PipelineRunParameters.SetOnProcessComplete(OnProcessComplete);

		TrackPipeline.Run(PipelineRunParameters);
	});
}

void FMetaHumanIdentityAssetEditorToolkit::CaptureSceneForPromotedFrame(UMetaHumanIdentityPromotedFrame* InPromotedFrame, bool bInCaptureForTracking) const
{
	if (InPromotedFrame != nullptr && SceneCaptureComponent != nullptr && IdentityPartsEditor.IsValid())
	{
		if (UMetaHumanIdentityCameraFrame* CameraFrame = Cast<UMetaHumanIdentityCameraFrame>(InPromotedFrame))
		{
			IdentityPartsEditor->UpdateViewportSelectionOutlines(false);

			bool bIsNeutralMeshVisibilityState = true;
			const bool bPropagateToChildren = true;
			const bool bIsInstance = true;

			USceneComponent* NeutralMeshComponent = IdentityPartsEditor->GetSceneComponentOfType(ESceneComponentIdentifier::ImportedMesh, bIsInstance);

			SceneCaptureComponent->ClearHiddenComponents();

			if (bInCaptureForTracking)
			{
				if (NeutralMeshComponent)
				{
					// Store the visibility state of the neutral mesh
					bIsNeutralMeshVisibilityState = NeutralMeshComponent->IsVisible();

					// Set it to be visible so we can capture a screenshot with the neutral mesh in the scene
					NeutralMeshComponent->SetVisibility(true, bPropagateToChildren);
				}

				// Hide components that shouldn't appear in the captured screenshot
				if (UPrimitiveComponent* ConformalMeshSceneComponent = Cast<UPrimitiveComponent>(IdentityPartsEditor->GetSceneComponentOfType(ESceneComponentIdentifier::ConformalMesh, bIsInstance)))
				{
					SceneCaptureComponent->HideComponent(ConformalMeshSceneComponent);
				}

				if (UPrimitiveComponent* RigComponent = Cast<UPrimitiveComponent>(IdentityPartsEditor->GetSceneComponentOfType(ESceneComponentIdentifier::ConformalRig, bIsInstance)))
				{
					SceneCaptureComponent->HideComponent(RigComponent);
				}
			}

			SceneCaptureComponent->FOVAngle = CameraFrame->CameraViewFOV;
			SceneCaptureComponent->SetWorldTransform(CameraFrame->GetCameraTransform());

			// Apply the ViewMode from the ViewportClient to make sure the capture is consistent with what is in the view
			const EViewModeIndex ViewMode = ViewportClient->GetViewMode();
			ensure(ViewMode == EViewModeIndex::VMI_Lit || ViewMode == EViewModeIndex::VMI_Unlit || ViewMode == EViewModeIndex::VMI_LightingOnly); // Scene capture component does not support other modes
			FEngineShowFlags SceneCaptureComponentShowFlagsCopy = SceneCaptureComponent->ShowFlags;
			const bool bCanDisableToneMapping = false;
			EngineShowFlagOverride(ESFIM_Editor, ViewMode, SceneCaptureComponent->ShowFlags, bCanDisableToneMapping);

			if (ViewportClient->ExposureSettings.bFixed)
			{
				PostProcessComponent->Settings.AutoExposureMinBrightness = ViewportClient->ExposureSettings.FixedEV100;
				PostProcessComponent->Settings.AutoExposureMaxBrightness = ViewportClient->ExposureSettings.FixedEV100;
			}

			SceneCaptureComponent->CaptureScene();

			SceneCaptureComponent->ShowFlags = SceneCaptureComponentShowFlagsCopy;
			PostProcessComponent->Settings.AutoExposureMinBrightness = 1.0;
			PostProcessComponent->Settings.AutoExposureMaxBrightness = 1.0;

			if (bInCaptureForTracking && NeutralMeshComponent)
			{
				// Restore the visibility state of the neutral mesh component
				NeutralMeshComponent->SetVisibility(bIsNeutralMeshVisibilityState, bPropagateToChildren);
			}

			IdentityPartsEditor->UpdateViewportSelectionOutlines(true);
		}
	}
}

void FMetaHumanIdentityAssetEditorToolkit::SetCaptureSceneRenderTargetSize(const FIntPoint& InNewSize) const
{
	if (SceneCaptureComponent != nullptr && SceneCaptureComponent->TextureTarget != nullptr)
	{
		SceneCaptureComponent->TextureTarget->ResizeTarget(FMath::Max(1, InNewSize.X), FMath::Max(1, InNewSize.Y));
		RefreshSceneCapture();
	}
}

void FMetaHumanIdentityAssetEditorToolkit::MakeStaticMeshAssetPickerMenu(UToolMenu* InToolMenu) const
{
	if (Identity->FindPartOfClass<UMetaHumanIdentityFace>() != nullptr)
	{
		TSharedRef<SWidget> WarningMessageBox = SNew(SBox)
			.Padding(FMargin{ 0.0f, 4.0f })
			[
				SNew(SWarningOrErrorBox)
				.MessageStyle(EMessageStyle::Warning)
				.Message(LOCTEXT("CantSelectMeshMessage", "This MetaHuman Identity already has a Face part. Remove it first to use this functionality"))
			];

		// If we have a Face already display a message to the user
		InToolMenu->AddMenuEntry(TEXT("CantSelectStaticMesh"), FToolMenuEntry::InitMenuEntry(TEXT("CantSelectStaticMesh"), FToolUIActionChoice{}, WarningMessageBox));
	}
	else
	{
		IContentBrowserSingleton& ContentBrowser = FModuleManager::LoadModuleChecked<FContentBrowserModule>(TEXT("ContentBrowser")).Get();

		FAssetPickerConfig AssetPickerConfig;
		AssetPickerConfig.SelectionMode = ESelectionMode::Single;
		AssetPickerConfig.Filter.ClassPaths.Add(UStaticMesh::StaticClass()->GetClassPathName());
		AssetPickerConfig.Filter.ClassPaths.Add(USkeletalMesh::StaticClass()->GetClassPathName());
		AssetPickerConfig.Filter.bRecursiveClasses = true;
		AssetPickerConfig.bAllowNullSelection = false;
		AssetPickerConfig.InitialAssetViewType = EAssetViewType::List;

		auto HandleAssetSelected = [this](const FAssetData& InAssetData)
		{
			IdentityPartsEditor->AddFaceFromMesh(InAssetData.GetAsset());

			FSlateApplication::Get().DismissAllMenus();
		};

		AssetPickerConfig.OnAssetSelected = FOnAssetSelected::CreateLambda(HandleAssetSelected);

		AssetPickerConfig.OnAssetEnterPressed = FOnAssetEnterPressed::CreateLambda([HandleAssetSelected](const TArray<FAssetData>& InAssetDataList)
		{
			if (!InAssetDataList.IsEmpty())
			{
				HandleAssetSelected(InAssetDataList[0].GetAsset());
			}
		});

		TSharedRef<SWidget> AssetPicker = SNew(SBox)
			.WidthOverride(300.0f)
			.HeightOverride(400.0f)
			[
				ContentBrowser.CreateAssetPicker(AssetPickerConfig)
			];

		InToolMenu->AddMenuEntry(TEXT("SelectMeshMenu"), FToolMenuEntry::InitMenuEntry(TEXT("MeshAssetPicker"), FToolUIActionChoice{}, AssetPicker));
	}
}

void FMetaHumanIdentityAssetEditorToolkit::ProcessLogMessages()
{
	if (!LogMessages.IsEmpty())
	{
		bool bContainsError = false;
		FString AssembledLogMessage;
		for (const TPair<FString, ELogVerbosity::Type>& LogMessagePair : LogMessages)
		{
			const FString& Message = LogMessagePair.Key;
			ELogVerbosity::Type Verbosity = LogMessagePair.Value;

			if (Verbosity == ELogVerbosity::Error)
			{
				bContainsError = true;
			}

			AssembledLogMessage += Message + TEXT("\n");
		}

		if (!AssembledLogMessage.IsEmpty() && bContainsError)
		{
			const FText Title = LOCTEXT("IdentitySolveError", "MetaHuman Identity Solve Error");
			FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(AssembledLogMessage), &Title);
		}

		LogMessages.Empty();
	}
}

bool FMetaHumanIdentityAssetEditorToolkit::LoadCurvesAndLandmarksFromJson(const FString& FileName)
{
	const FString JsonFilePath = IPluginManager::Get().FindPlugin(TEXT(UE_PLUGIN_NAME))->GetContentDir() + "/MeshFitting/Template/" + FileName;

	FString JsonString;

	FFileHelper::LoadFileToString(JsonString, *JsonFilePath);

	TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject());
	TSharedRef<TJsonReader<>> JsonReader = TJsonReaderFactory<>::Create(JsonString);

	if (FJsonSerializer::Deserialize(JsonReader, JsonObject) && JsonObject.IsValid())
	{
		TMap<FString, TSharedPtr<FJsonValue>> Values = JsonObject->Values;
		MarkerData->MarkerCurves.Empty();

		MarkerDefs = MakeShared<FMarkerDefs>();
		for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : Values)
		{
			FString Name = Pair.Key;
			TSharedPtr<FJsonValue> JsonValue = Pair.Value;
			TSharedPtr<FJsonObject> SubObject = JsonValue->AsObject();
			FString SubObjectType = SubObject->GetStringField(TEXT("type"));
			if (SubObjectType == TEXT("curve"))
			{
				FMarkerCurveDef MarkerCurveDef;
				MarkerCurveDef.Name = Name;

				//"start", "end" and "vIDs" have to be specified in the config
				SubObject->TryGetStringField(TEXT("start"), MarkerCurveDef.StartPointName);
				SubObject->TryGetStringField(TEXT("end"), MarkerCurveDef.EndPointName);

				const TArray<TSharedPtr<FJsonValue>>* VertexIDs;
				SubObject->TryGetArrayField(TEXT("vIDs"), VertexIDs);
				for(TSharedPtr<FJsonValue> ID : *VertexIDs)
				{
					MarkerCurveDef.VertexIDs.Add(ID->AsNumber());
				}

				MarkerDefs->CurveDefs.Add(MarkerCurveDef);
			}
			else if(SubObjectType == TEXT("landmark"))
			{
				FString VIDString = SubObject->GetStringField("vID");
				int32 VertexID = FCString::Atoi(*VIDString);
				if(SubObject->HasField("solo"))
				{
					FMarkerCurveDef MarkerCurveDef;
					MarkerCurveDef.Name = Name;
					MarkerCurveDef.VertexIDs.Add(VertexID);
					MarkerDefs->CurveDefs.Add(MarkerCurveDef);
				}
				else
				{
					MarkerDefs->Landmarks.Add(Name, VertexID);
				}
			}
		}

		MarkerData->MarkerDefs = MarkerDefs.Get();
	}
	else
	{
		UE_LOG(LogMetaHumanIdentity, Error, TEXT("%s"), *JsonReader->GetErrorMessage());
		return false;
	}

	return true;
}

bool FMetaHumanIdentityAssetEditorToolkit::LoadGroupsFromJson(const FString& FileName) const
{
	const FString JsonFilePath = IPluginManager::Get().FindPlugin(UE_PLUGIN_NAME)->GetContentDir() + "/MeshFitting/Template/" + FileName;

	FString JsonString;

	FFileHelper::LoadFileToString(JsonString, *JsonFilePath);

	TSharedPtr<FJsonObject> RootJsonObject = MakeShareable(new FJsonObject());
	TSharedRef<TJsonReader<>> JsonReader = TJsonReaderFactory<>::Create(JsonString);

	if (FJsonSerializer::Deserialize(JsonReader, RootJsonObject) && RootJsonObject.IsValid())
	{
		TMap<FString, TSharedPtr<FJsonValue>> Values = RootJsonObject->Values;
		MarkerDefs->GroupNames.Empty();
		for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : Values)
		{
			FString GroupName = Pair.Key;
			MarkerDefs->GroupNames.Add(GroupName);

			const TSharedPtr<FJsonValue> JsonValue = Pair.Value;
			const TArray<TSharedPtr<FJsonValue>> CurvesArray = JsonValue->AsArray();

			for (int Index = 0; Index < CurvesArray.Num(); Index++)
			{
				FString CurveName = CurvesArray[Index]->AsString();

				int32 FoundIndex = MarkerDefs->CurveDefs.IndexOfByPredicate
				(
					[&CurveName](const FMarkerCurveDef& Curve)
					{
						return CurveName == Curve.Name;
					}
				);

				if (FoundIndex >= 0)
				{
					MarkerDefs->CurveDefs[FoundIndex].GroupTagIDs.Add(GroupName);
				}
			}
		}
	}
	else
	{
		UE_LOG(LogMetaHumanIdentity, Error, TEXT("%s"), *JsonReader->GetErrorMessage());
		return false;
	}
	return true;
}

void FMetaHumanIdentityAssetEditorToolkit::SetUpInitialOutlinerData() const
{
	for (const FString& GroupName : MarkerDefs->GroupNames)
	{
		TSharedPtr<FMarkerOutlinerGroupData> GroupData = MakeShared<FMarkerOutlinerGroupData>();
		GroupData->Name = GroupName;
		MarkerData->MarkerGroups.Add(GroupData);
	}

	for (const FMarkerCurveDef& CurveDef : MarkerDefs->CurveDefs)
	{
		TSharedPtr<FMarkerOutlinerCurveData> CurveData = MakeShared<FMarkerOutlinerCurveData>();
		const FString CurveName = CurveDef.Name;
		CurveData->Name = CurveName;
		MarkerData->MarkerCurves.Add(CurveData);
	}
}

void FMetaHumanIdentityAssetEditorToolkit::UpdateCurveStateForFrameAfterTracking(const FFrameTrackingContourData& InTrackData, UMetaHumanIdentityPromotedFrame* InTrackedFrame) const
{
	TMap<FString, FTrackingContour>& FrameContours = InTrackedFrame->ContourData.TrackingContours;
	for(const TPair<FString, FTrackingContour>& TrackedContour : InTrackData.TrackingContours)
	{
		if(FrameContours.Contains(TrackedContour.Key))
		{
			FrameContours[TrackedContour.Key].State.bVisible = true;
			FrameContours[TrackedContour.Key].State.bActive = true;
		}
	}
}

void FMetaHumanIdentityAssetEditorToolkit::UpdateOutlinerFrameData() const
{
	MarkerData->MarkerFrames.Empty();

	if (UMetaHumanIdentityPose* Pose = PromotedFramesEditorWidget->GetIdentityPose())
	{
		int32 FrameNumber = 0;
		UMetaHumanIdentityPromotedFrame *SelectedFrame = PromotedFramesEditorWidget->GetSelectedPromotedFrame();
		for (UMetaHumanIdentityPromotedFrame* PromotedFrame : Pose->PromotedFrames)
		{
			TSharedPtr<FMarkerOutlinerFrameData> MarkerDataPtr(new FMarkerOutlinerFrameData());
			MarkerDataPtr->Name = PromotedFrame->FrameName.ToString();
			if (MarkerDataPtr->Name.IsEmpty())
			{
				MarkerDataPtr->Name = "[Frame " + FString::FromInt(FrameNumber) + "]";
			}

			MarkerDataPtr->bSelected = PromotedFrame == SelectedFrame;
			MarkerDataPtr->Frame = PromotedFrame;
			MarkerDataPtr->FrameNumber = FrameNumber;
			MarkerData->MarkerFrames.Add(MarkerDataPtr);

			++FrameNumber;
		}
	}
}

void FMetaHumanIdentityAssetEditorToolkit::RefreshSceneCapture() const
{
	if (PromotedFramesEditorWidget.IsValid())
	{
		if (UMetaHumanIdentityPromotedFrame* PromotedFrame = PromotedFramesEditorWidget->GetSelectedPromotedFrame())
		{
			if (PromotedFrame->IsNavigationLocked())
			{
				CaptureSceneForPromotedFrame(PromotedFrame);
			}
		}
	}
}

#undef LOCTEXT_NAMESPACE