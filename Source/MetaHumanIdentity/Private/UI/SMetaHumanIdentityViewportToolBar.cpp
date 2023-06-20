// Copyright Epic Games, Inc. All Rights Reserved.

#include "SMetaHumanIdentityViewportToolBar.h"

#include "MetaHumanIdentityCommands.h"
#include "SConformingAssetEditorViewport.h"
#include "ConformingViewportClient.h"
#include "MetaHumanIdentityStyle.h"
#include "SABImage.h"

#include "EditorViewportCommands.h"
#include "SEditorViewportToolBarMenu.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/SBoxPanel.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "SEditorViewportViewMenu.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "SEditorViewportViewMenuContext.h"
#include "ToolMenus.h"

#define LOCTEXT_NAMESPACE "MetaHumanIdentityViewportToolbar"

/**
 * Customized version of an SEditorViewportViewMenu that overrides the behaviour of the button so its not tied to
 * the viewport client directly. This is necessary as we have to keep the state for two of these in the Identity asset editor toolbar
 */
class SConformingViewportViewMenu
	: public SEditorViewportViewMenu
{
public:
	SLATE_BEGIN_ARGS(SConformingViewportViewMenu) {}
		SLATE_ARGUMENT(EABImageViewMode, ViewMode)
		SLATE_ARGUMENT(TSharedPtr<FExtender>, MenuExtenders)
		SLATE_ARGUMENT(TSharedPtr<FUICommandList>, CommandList)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, TSharedRef<SEditorViewport> InViewport, TSharedRef<SViewportToolBar> InParentToolBar)
	{
		Viewport = InViewport;
		MenuName = BaseMenuName;
		MenuExtenders = InArgs._MenuExtenders;
		CommandList = InArgs._CommandList;
		ViewMode = InArgs._ViewMode;

		SEditorViewportToolbarMenu::Construct(SEditorViewportToolbarMenu::FArguments()
											  .ParentToolBar(InParentToolBar)
											  .Cursor(EMouseCursor::Default)
											  .Label(this, &SConformingViewportViewMenu::GetViewMenuLabelOverride)
											  .LabelIcon(this, &SConformingViewportViewMenu::GetViewMenuLabelIconOverride)
											  .OnGetMenuContent(this, &SConformingViewportViewMenu::GenerateViewMenuContent));
	}

protected:

	/** SEditorViewportViewMenu::GetViewMenuLabel is private in the base but needs to be overriden to customize the label it returns */
	FText GetViewMenuLabelOverride() const
	{
		FText Label = LOCTEXT("ViewMenuTitle_Default", "View");

		if (TSharedPtr<SEditorViewport> PinnedViewport = Viewport.Pin())
		{
			if (const TSharedPtr<FConformingViewportClient> ViewportClient = StaticCastSharedPtr<FConformingViewportClient>(PinnedViewport->GetViewportClient()))
			{
				const EViewModeIndex ViewModeIndex = ViewportClient->GetViewModeIndexForViewMode(ViewMode);
				// If VMI_VisualizeBuffer, return its subcategory name
				if (ViewModeIndex == VMI_VisualizeBuffer)
				{
					Label = ViewportClient->GetCurrentBufferVisualizationModeDisplayName();
				}
				else if (ViewModeIndex == VMI_VisualizeNanite)
				{
					Label = ViewportClient->GetCurrentNaniteVisualizationModeDisplayName();
				}
				else if (ViewModeIndex == VMI_VisualizeLumen)
				{
					Label = ViewportClient->GetCurrentLumenVisualizationModeDisplayName();
				}
				else if (ViewModeIndex == VMI_VisualizeVirtualShadowMap)
				{
					Label = ViewportClient->GetCurrentVirtualShadowMapVisualizationModeDisplayName();
				}
				// For any other category, return its own name
				else
				{
					Label = UViewModeUtils::GetViewModeDisplayName(ViewModeIndex);
				}
			}
		}

		return Label;
	}

	/** SEditorViewportViewMenu::GetViewMenuLabelIcon is private in the base class but needs to be overriden to customize the icon shown */
	const FSlateBrush* GetViewMenuLabelIconOverride() const
	{
		if (TSharedPtr<SEditorViewport> PinnedViewport = Viewport.Pin())
		{
			if (const TSharedPtr<FConformingViewportClient> ViewportClient = StaticCastSharedPtr<FConformingViewportClient>(PinnedViewport->GetViewportClient()))
			{
				const EViewModeIndex ViewModeIndex = ViewportClient->GetViewModeIndexForViewMode(ViewMode);
				return UViewModeUtils::GetViewModeDisplayIcon(ViewModeIndex);
			}
		}

		return FStyleDefaults::GetNoBrush();
	}

	bool IsFixedEV100Enabled() const
	{
		if (TSharedPtr<SEditorViewport> PinnedViewport = Viewport.Pin())
		{
			if (const TSharedPtr<FConformingViewportClient> ViewportClient = StaticCastSharedPtr<FConformingViewportClient>(PinnedViewport->GetViewportClient()))
			{
				return ViewportClient->GetFixedEV100(ViewMode);
			}
		}

		return false;
	}

	float OnGetFixedEV100Value() const
	{
		if (TSharedPtr<SEditorViewport> PinnedViewport = Viewport.Pin())
		{
			if (const TSharedPtr<FConformingViewportClient> ViewportClient = StaticCastSharedPtr<FConformingViewportClient>(PinnedViewport->GetViewportClient()))
			{
				return ViewportClient->GetEV100(ViewMode);
			}
		}

		return 0;
	}
	
	void OnFixedEV100ValueChanged(float InNewValue) const
	{
		if (TSharedPtr<SEditorViewport> PinnedViewport = Viewport.Pin())
		{
			if (const TSharedPtr<FConformingViewportClient> ViewportClient = StaticCastSharedPtr<FConformingViewportClient>(PinnedViewport->GetViewportClient()))
			{
				ViewportClient->SetEV100(ViewMode, InNewValue);
			}
		}
	}

	TSharedRef<SWidget> BuildFixedEV100Menu() const // Copied from SEditorViewport::BuildFixedEV100Menu
	{
		const float EV100Min = -10.f;
		const float EV100Max = 20.f;

		return
			SNew( SBox )
			.HAlign( HAlign_Right )
			[
				SNew( SBox )
				.Padding( FMargin(0.0f, 0.0f, 0.0f, 0.0f) )
				.WidthOverride( 100.0f )
				[
					SNew ( SBorder )
					.BorderImage(FAppStyle::Get().GetBrush("Menu.WidgetBorder"))
					.Padding(FMargin(1.0f))
					[
						SNew(SSpinBox<float>)
						.Style(&FAppStyle::Get(), "Menu.SpinBox")
						.Font( FAppStyle::GetFontStyle( TEXT( "MenuItem.Font" ) ) )
						.MinValue(EV100Min)
						.MaxValue(EV100Max)
						.Value(this, &SConformingViewportViewMenu::OnGetFixedEV100Value )
						.OnValueChanged(this, &SConformingViewportViewMenu::OnFixedEV100ValueChanged )
						.ToolTipText(LOCTEXT( "EV100ToolTip", "Sets the exposure value of the camera using the specified EV100. Exposure = 1 / (1.2 * 2^EV100)"))
						.IsEnabled(this, &SConformingViewportViewMenu::IsFixedEV100Enabled )
					]
				]
			];
	};

	void FillViewMenu(UToolMenu* InMenu) const
	{
		const FEditorViewportCommands& BaseViewportActions = FEditorViewportCommands::Get();

		{
			// View modes
			{
				FToolMenuSection& Section = InMenu->AddSection("ViewMode", LOCTEXT("ViewModeHeader", "View Mode"));
				{
					Section.AddMenuEntry(BaseViewportActions.LitMode, UViewModeUtils::GetViewModeDisplayName(VMI_Lit));
					Section.AddMenuEntry(BaseViewportActions.UnlitMode, UViewModeUtils::GetViewModeDisplayName(VMI_Unlit));
					Section.AddMenuEntry(BaseViewportActions.LightingOnlyMode, UViewModeUtils::GetViewModeDisplayName(VMI_LightingOnly));
				}
			}

			// Auto Exposure
			{
				const FEditorViewportCommands& BaseViewportCommands = FEditorViewportCommands::Get();

				TSharedRef<SWidget> FixedEV100Menu = BuildFixedEV100Menu();

				FToolMenuSection& Section = InMenu->AddSection("Exposure", LOCTEXT("ExposureHeader", "Exposure"));
				Section.AddEntry(FToolMenuEntry::InitWidget("FixedEV100", FixedEV100Menu, LOCTEXT("FixedEV100", "EV100")));
			}
		}
	}

	virtual TSharedRef<SWidget> GenerateViewMenuContent() const override
	{
		RegisterMenus();

		UConformingViewportViewMenuContext* ContextObject = NewObject<UConformingViewportViewMenuContext>();
		ContextObject->EditorViewportViewMenu = SharedThis(this);
		ContextObject->ConformingViewportViewMenu = SharedThis(this);

		FToolMenuContext MenuContext(CommandList, MenuExtenders, ContextObject);
		return UToolMenus::Get()->GenerateWidget(MenuName, MenuContext);
	}

	virtual void RegisterMenus() const override
	{
		if (!UToolMenus::Get()->IsMenuRegistered(BaseMenuName))
		{
			UToolMenu* Menu = UToolMenus::Get()->RegisterMenu(BaseMenuName);
			Menu->AddDynamicSection("BaseSection", FNewToolMenuDelegate::CreateLambda([](UToolMenu* InMenu)
				{
					if (UConformingViewportViewMenuContext* Context = InMenu->FindContext<UConformingViewportViewMenuContext>())
					{
						Context->ConformingViewportViewMenu.Pin()->FillViewMenu(InMenu);
					}
				}));
		}
	}

private:
	/** The command list to generate the menu from */
	TSharedPtr<FUICommandList> CommandList;

	/** The view mode associated with this toolbar menu */
	EABImageViewMode ViewMode;

	static const FName BaseMenuName;
};

const FName SConformingViewportViewMenu::BaseMenuName("UnrealEd.ViewportToolbar.View.ConformingCustomization");

const FMargin SMetaHumanIdentityViewportToolBar::ToolbarSlotPadding{ 4.0f, 4.0f };

void SMetaHumanIdentityViewportToolBar::Construct(const FArguments& InArgs)
{
	CommandListViewA = InArgs._CommandListViewA;
	CommandListViewB = InArgs._CommandListViewB;
	ViewportClient = InArgs._ViewportClient;

	ChildSlot
	[
		SNew(SBorder)
		.BorderImage(FAppStyle::Get().GetBrush("EditorViewportToolBar.Background"))
		.Cursor(EMouseCursor::Default)
		[
			SAssignNew(ToolbarMenuHorizontalBox, SHorizontalBox)
			+ SHorizontalBox::Slot()
			.Padding(ToolbarSlotPadding)
			.HAlign(HAlign_Left)
			[
				CreateViewAMenuWidget()
			]
			+ SHorizontalBox::Slot()
			.Padding(ToolbarSlotPadding)
			.HAlign(HAlign_Center)
			[
				CreateViewSelectorMenuWidget()
			]
			+ SHorizontalBox::Slot()
			.Padding(ToolbarSlotPadding)
			.HAlign(HAlign_Right)
			[
				CreateViewBMenuWidget()
			]
		]
	];

	SViewportToolBar::Construct(SViewportToolBar::FArguments());
}

TSharedRef<SWidget> SMetaHumanIdentityViewportToolBar::CreateViewAMenuWidget()
{
	FSlimHorizontalToolBarBuilder ToolbarBuilder(CommandListViewA, FMultiBoxCustomization::None);
	ToolbarBuilder.SetStyle(&FAppStyle::Get(), TEXT("EditorViewportToolBar"));
	ToolbarBuilder.SetLabelVisibility(EVisibility::Visible);

	ToolbarBuilder.BeginSection(TEXT("ViewMenuA"));
	{
		TSharedRef<SEditorViewportToolbarMenu> ViewOptionsDropdownMenu = SNew(SEditorViewportToolbarMenu)
			.ParentToolBar(SharedThis(this))
			.ToolTipText(LOCTEXT("ViewMenuAToolTip", "View Menu A"))
			.Cursor(EMouseCursor::Default)
			.Image("EditorViewportToolBar.OptionsDropdown")
			.OnGetMenuContent(this, &SMetaHumanIdentityViewportToolBar::FillPreviewOptionsMenu);

		TSharedRef<SConformingViewportViewMenu> ViewAModeDropdownMenu =
			SNew(SConformingViewportViewMenu, ViewportClient->GetEditorViewportWidget().ToSharedRef(), SharedThis(this))
			.ViewMode(EABImageViewMode::A)
			.CommandList(CommandListViewA);

		TSharedRef<SEditorViewportToolbarMenu> MixOptionsViewADropdownMenu = SNew(SEditorViewportToolbarMenu)
			.ParentToolBar(SharedThis(this))
			.ToolTipText(LOCTEXT("ViewAMenuToolTip", "View A Menu"))
			.Label(LOCTEXT("MixOptionsViewA", "A"))
			.OnGetMenuContent(this, &SMetaHumanIdentityViewportToolBar::FillViewMenu, true);

		ToolbarBuilder.AddWidget(ViewOptionsDropdownMenu);
		ToolbarBuilder.AddSeparator();
		ToolbarBuilder.AddWidget(ViewAModeDropdownMenu);
		ToolbarBuilder.AddSeparator();
		ToolbarBuilder.AddWidget(MixOptionsViewADropdownMenu);
	}
	ToolbarBuilder.EndSection();

	return ToolbarBuilder.MakeWidget();
}

TSharedRef<SWidget> SMetaHumanIdentityViewportToolBar::CreateViewSelectorMenuWidget()
{
	FSlimHorizontalToolBarBuilder ToolbarBuilder(CommandListViewA, FMultiBoxCustomization::None);
	ToolbarBuilder.SetStyle(&FAppStyle::Get(), TEXT("EditorViewportToolBar"));
	ToolbarBuilder.SetLabelVisibility(EVisibility::Collapsed);
	ToolbarBuilder.SetIsFocusable(false);

	ToolbarBuilder.BeginSection(TEXT("ViewTypeSelection"));
	{
		ToolbarBuilder.BeginBlockGroup();
		{
			TAttribute<FSlateIcon> AIcon = TAttribute<FSlateIcon>::CreateLambda([this]
			{
				if (ViewportClient.IsValid())
				{
					if (ViewportClient->ShowViewA())
					{
						return FSlateIcon{ FMetaHumanIdentityStyle::Get().GetStyleSetName(), TEXT("Identity.ABSplit.A.Large") };
					}
					else
					{
						return FSlateIcon{ FMetaHumanIdentityStyle::Get().GetStyleSetName(), TEXT("Identity.ABSplit.A.Small") };
					}
				}

				return FSlateIcon{};
			});

			TAttribute<FSlateIcon> BIcon = TAttribute<FSlateIcon>::CreateLambda([this]
			{
				if (ViewportClient.IsValid())
				{
					if (ViewportClient->ShowViewB())
					{
						return FSlateIcon{ FMetaHumanIdentityStyle::Get().GetStyleSetName(), TEXT("Identity.ABSplit.B.Large") };
					}
					else
					{
						return FSlateIcon{ FMetaHumanIdentityStyle::Get().GetStyleSetName(), TEXT("Identity.ABSplit.B.Small") };
					}
				}

				return FSlateIcon{};
			});

			ToolbarBuilder.AddToolBarButton(FMetaHumanIdentityEditorCommands::Get().ToggleViewA,
											NAME_None,
											TAttribute<FText>{},
											TAttribute<FText>{},
											AIcon);

			ToolbarBuilder.AddToolBarButton(FMetaHumanIdentityEditorCommands::Get().ToggleViewB,
											NAME_None,
											TAttribute<FText>{},
											TAttribute<FText>{},
											BIcon);
		}
		ToolbarBuilder.EndBlockGroup();
	}
	ToolbarBuilder.EndSection();

	return ToolbarBuilder.MakeWidget();
}

TSharedRef<SWidget> SMetaHumanIdentityViewportToolBar::CreateViewBMenuWidget()
{
	FSlimHorizontalToolBarBuilder ToolbarBuilder(CommandListViewB, FMultiBoxCustomization::None);

	ToolbarBuilder.SetStyle(&FAppStyle::Get(), TEXT("EditorViewportToolBar"));
	ToolbarBuilder.SetLabelVisibility(EVisibility::Visible);

	ToolbarBuilder.BeginSection("ViewMenu");
	{
		TSharedRef<SEditorViewportToolbarMenu> ViewOptionsDropdownMenu = SNew(SEditorViewportToolbarMenu)
			.ParentToolBar(SharedThis(this))
			.ToolTipText(LOCTEXT("ViewMenuBToolTip", "View B Menu"))
			.Cursor(EMouseCursor::Default)
			.Image("EditorViewportToolBar.OptionsDropdown")
			.OnGetMenuContent(this, &SMetaHumanIdentityViewportToolBar::FillPreviewOptionsMenu);

		const FText LabelText = LOCTEXT("ViewMenuB", "View B");

		TSharedRef<SConformingViewportViewMenu> ViewBModeDropdownMenu =
			SNew(SConformingViewportViewMenu, ViewportClient->GetEditorViewportWidget().ToSharedRef(), SharedThis(this))
			.ViewMode(EABImageViewMode::B)
			.CommandList(CommandListViewB);

		TSharedRef<SEditorViewportToolbarMenu> MixOptionsViewBDropdownMenu = SNew(SEditorViewportToolbarMenu)
			.ParentToolBar(SharedThis(this))
			.ToolTipText(LOCTEXT("ViewBMenuToolTip", "View B Menu"))
			.Label(LOCTEXT("MixOptionsViewB", "B"))
			.OnGetMenuContent(this, &SMetaHumanIdentityViewportToolBar::FillViewMenu, false);

		const FName ViewModeMenuOptionsMenuName = TEXT("ViewModeMenuOptionsMenu");
		const FName ViewMenuOptionsMenuName = TEXT("ViewMenuOptionsMenu");
		const FName MixOptionsMenuName = TEXT("MixOptionsMenuName");
		const bool bIsSearchable = false;

		ToolbarBuilder.AddWidget(MixOptionsViewBDropdownMenu);
		ToolbarBuilder.AddSeparator();
		ToolbarBuilder.AddWidget(ViewBModeDropdownMenu);
		ToolbarBuilder.AddSeparator();
		ToolbarBuilder.AddWidget(ViewOptionsDropdownMenu);
	}
	ToolbarBuilder.EndSection();

	return ToolbarBuilder.MakeWidget();
}

TSharedRef<SWidget> SMetaHumanIdentityViewportToolBar::FillViewSelectionMenu()
{
	const bool bShouldCloseWindowAfterMenuSelection = true;
	FMenuBuilder LocationGridMenuBuilder(bShouldCloseWindowAfterMenuSelection, CommandListViewA);

	LocationGridMenuBuilder.BeginSection(TEXT("ViewMixExtensionHook"), LOCTEXT("ViewMixSectionLabel", "View Mix"));
	{
		LocationGridMenuBuilder.AddMenuEntry(LOCTEXT("SingleScreenLabel", "Single"),
											 LOCTEXT("SingleScreenTooltip", "Display Single View"),
											 FSlateIcon(),
											 FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityViewportToolBar::EnterSingleScreenView),
													   FCanExecuteAction{},
													   FIsActionChecked::CreateSP(this, &SMetaHumanIdentityViewportToolBar::SingleViewIsChecked)),
											 NAME_None,
											 EUserInterfaceActionType::RadioButton);

		LocationGridMenuBuilder.AddMenuEntry(LOCTEXT("MultiScreenLabel", "Dual"),
											 LOCTEXT("MultiScreenTooltip", "Display View A and B in dual mode"),
											 FSlateIcon(),
											 FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityViewportToolBar::EnterMultiScreenView),
													   FCanExecuteAction{},
													   FIsActionChecked::CreateSP(this, &SMetaHumanIdentityViewportToolBar::MultiScreenViewIsChecked)),
											 NAME_None,
											 EUserInterfaceActionType::RadioButton);

		LocationGridMenuBuilder.AddMenuEntry(LOCTEXT("SplitScreenLabel", "Wipe"),
											 LOCTEXT("SplitScreenTooltip", "Display view A and B with a wiper"),
											 FSlateIcon(),
											 FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityViewportToolBar::EnterSplitScreenView),
													   FCanExecuteAction{},
													   FIsActionChecked::CreateSP(this, &SMetaHumanIdentityViewportToolBar::SplitScreenViewIsChecked)),
											 NAME_None,
											 EUserInterfaceActionType::RadioButton);
	}
	LocationGridMenuBuilder.EndSection();

	return LocationGridMenuBuilder.MakeWidget();
}

TSharedRef<SWidget> SMetaHumanIdentityViewportToolBar::FillViewMenu(bool bInIsViewA)
{
	const bool bShouldCloseWindowAfterMenuSelection = true;
	FMenuBuilder LocationGridMenuBuilder(bShouldCloseWindowAfterMenuSelection, CommandListViewA);

	LocationGridMenuBuilder.BeginSection(TEXT("ViewMenuExtensionHook"), LOCTEXT("GeometrySectionLabel", "Geometry"));
	{
		LocationGridMenuBuilder.AddMenuEntry(LOCTEXT("NeutralPoseMeshLabel", "Neutral Pose"),
											 LOCTEXT("NeutralPoseMeshTooltip", "Toggle the visibility of the Neutral Pose Mesh"),
											 FSlateIcon(),
											 FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityViewportToolBar::ToggleReferenceMesh, bInIsViewA),
													   FCanExecuteAction{},
													   FIsActionChecked::CreateSP(this, &SMetaHumanIdentityViewportToolBar::ReferenceMeshForViewIsChecked, bInIsViewA)),
											 NAME_None,
											 EUserInterfaceActionType::ToggleButton);

		LocationGridMenuBuilder.AddMenuEntry(LOCTEXT("TemplateMeshLabel", "Template Mesh"),
											 LOCTEXT("TemplateMeshTooltip", "Toggle the Template Mesh Visibility"),
											 FSlateIcon(),
											 FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityViewportToolBar::ToggleConformedMesh, bInIsViewA),
													   FCanExecuteAction{},
													   FIsActionChecked::CreateSP(this, &SMetaHumanIdentityViewportToolBar::ConformedMeshForViewIsChecked, bInIsViewA)),
											 NAME_None,
											 EUserInterfaceActionType::ToggleButton);

		// TODO: Re-enable when we have the Rig in the viewport
		// LocationGridMenuBuilder.AddMenuEntry(LOCTEXT("ConformalRig", "Conformal Rig"),
		// 									 LOCTEXT("ConformalRigTooltip", "Toggle Conformal Rig Visibility"),
		// 									 FSlateIcon(),
		// 									 FUIAction(FExecuteAction::CreateSP(this, &SIdentityViewportToolBar::ToggleConformalRig, IsViewA),
		// 											   FCanExecuteAction::CreateLambda([this] {return true; }),
		// 											   FIsActionChecked::CreateSP(this, &SIdentityViewportToolBar::ConformalRigForViewIsChecked, IsViewA)),
		// 									 NAME_None,
		// 									 EUserInterfaceActionType::ToggleButton);
	}
	LocationGridMenuBuilder.EndSection();

	LocationGridMenuBuilder.BeginSection(TEXT("ViewMenuExtensionHook"), LOCTEXT("MarkersSectionLabel", "Markers"));
	{
		LocationGridMenuBuilder.AddMenuEntry(LOCTEXT("CurvesVisibilityLabel", "Curves"),
											 LOCTEXT("CurvesVisibilityTooltip", "Toggle the curve visibility"),
											 FSlateIcon(),
											 FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityViewportToolBar::ToggleCurveVisibility, bInIsViewA),
													   FCanExecuteAction{},
													   FIsActionChecked::CreateSP(this, &SMetaHumanIdentityViewportToolBar::DisplayCurvesForViewIsChecked, bInIsViewA)),
											 NAME_None,
											 EUserInterfaceActionType::ToggleButton);

		LocationGridMenuBuilder.AddMenuEntry(LOCTEXT("PointVisibilityLabel", "Control Vertices"),
											 LOCTEXT("PointVisibilityTooltip", "Toggle the visibility of control vertices"),
											 FSlateIcon(),
											 FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityViewportToolBar::TogglePointVisibility, bInIsViewA),
													   FCanExecuteAction{},
													   FIsActionChecked::CreateSP(this, &SMetaHumanIdentityViewportToolBar::DisplayPointsForViewIsChecked, bInIsViewA)),
											 NAME_None,
											 EUserInterfaceActionType::ToggleButton);
	}
	LocationGridMenuBuilder.EndSection();

	return LocationGridMenuBuilder.MakeWidget();
}

TSharedRef<SWidget> SMetaHumanIdentityViewportToolBar::FillPreviewOptionsMenu()
{
	const float FOVMin = 5.f;
	const float FOVMax = 170.f;

	TSharedRef<SWidget> ViewFOVWidget = SNew(SBox)
		.HAlign(HAlign_Right)
		[
			SNew(SBox)
			.Padding(FMargin(4.0f, 0.0f, 0.0f, 0.0f))
			.WidthOverride(100.0f)
			[
				SNew(SNumericEntryBox<float>)
				.Font(FAppStyle::GetFontStyle(TEXT("MenuItem.Font")))
				.AllowSpin(true)
				.MinValue(FOVMin)
				.MaxValue(FOVMax)
				.MinSliderValue(FOVMin)
				.MaxSliderValue(FOVMax)
				.IsEnabled(this, &SMetaHumanIdentityViewportToolBar::CanChangeFOV)
				.Value(this, &SMetaHumanIdentityViewportToolBar::GetFOVValue)
				.OnValueChanged(this, &SMetaHumanIdentityViewportToolBar::HandleFOVValueChanged)
			]
		];

	const bool bShouldCloseWindowAfterMenuSelection = true;
	FMenuBuilder MenuBuilder(bShouldCloseWindowAfterMenuSelection, CommandListViewA);

	MenuBuilder.BeginSection(TEXT("ViewMenuExtensionHook"), LOCTEXT("ViewMenuCameraSectionLabel", "Camera"));
	{
		MenuBuilder.AddMenuEntry(FEditorViewportCommands::Get().FocusViewportToSelection);
		MenuBuilder.AddWidget(ViewFOVWidget, LOCTEXT("CameraOptionsFOVLabel", "Field Of View"));
	}
	MenuBuilder.EndSection();

	return MenuBuilder.MakeWidget();
}

bool SMetaHumanIdentityViewportToolBar::CanChangeFOV() const
{
	if (ViewportClient.IsValid())
	{
		return !ViewportClient->IsNavigationLocked();
	}

	return false;
}

TOptional<float> SMetaHumanIdentityViewportToolBar::GetFOVValue() const
{
	if (ViewportClient.IsValid())
	{
		return ViewportClient->ViewFOV;
	}

	return 0.0f;
}

void SMetaHumanIdentityViewportToolBar::HandleFOVValueChanged(float InNewValue)
{
	if (ViewportClient.IsValid())
	{
		ViewportClient->ViewFOV = InNewValue;

		// Tell the viewport client that the camera moved so promoted frames can track the new FoV value
		ViewportClient->PerspectiveCameraMoved();
		ViewportClient->EndCameraMovement();
		ViewportClient->Invalidate();
	}
}

EVisibility SMetaHumanIdentityViewportToolBar::GetShowAVisibility() const
{
	return ViewportClient->ShowViewA() ? EVisibility::Visible : EVisibility::Hidden;
}

EVisibility SMetaHumanIdentityViewportToolBar::GetShowBVisibility() const
{
	return ViewportClient->ShowViewB() ? EVisibility::Visible : EVisibility::Hidden;
}

void SMetaHumanIdentityViewportToolBar::ToggleReferenceMesh(bool bInIsViewA) const
{
	ViewportClient->ToggleReferenceMeshForView(bInIsViewA);
}

void SMetaHumanIdentityViewportToolBar::ToggleConformedMesh(bool bInIsViewA) const
{
	ViewportClient->ToggleConformedMeshForView(bInIsViewA);
}

void SMetaHumanIdentityViewportToolBar::ToggleConformalRig(bool bInIsViewA) const
{
	ViewportClient->ToggleConformalRigForView(bInIsViewA);
}

void SMetaHumanIdentityViewportToolBar::ToggleCurveVisibility(bool bInIsViewA) const
{
	ViewportClient->ToggleDisplayCurves(bInIsViewA);
}

void SMetaHumanIdentityViewportToolBar::TogglePointVisibility(bool bInIsViewA) const
{
	ViewportClient->ToggleDisplayPoints(bInIsViewA);
}

void SMetaHumanIdentityViewportToolBar::EnterSingleScreenView()
{
	ViewportClient->ToggleToSingleViewA();
}

void SMetaHumanIdentityViewportToolBar::EnterMultiScreenView()
{
	ViewportClient->ToggleAB_Side();
}

void SMetaHumanIdentityViewportToolBar::EnterSplitScreenView()
{
	ViewportClient->ToggleAB_Split();
}

bool SMetaHumanIdentityViewportToolBar::ReferenceMeshForViewIsChecked(bool bInIsViewA) const
{
	return ViewportClient->IsShowingReferenceMesh(bInIsViewA);
}

bool SMetaHumanIdentityViewportToolBar::ConformedMeshForViewIsChecked(bool bInIsViewA) const
{
	return ViewportClient->IsShowingConformedMesh(bInIsViewA);
}

bool SMetaHumanIdentityViewportToolBar::ConformalRigForViewIsChecked(bool bInIsViewA) const
{
	return ViewportClient->IsShowingConformalRig(bInIsViewA);
}

bool SMetaHumanIdentityViewportToolBar::DisplayCurvesForViewIsChecked(bool bInIsViewA) const
{
	return ViewportClient->IsShowingCurves(bInIsViewA);
}

bool SMetaHumanIdentityViewportToolBar::DisplayPointsForViewIsChecked(bool bInIsViewA) const
{
	return ViewportClient->IsShowingPoints(bInIsViewA);
}

bool SMetaHumanIdentityViewportToolBar::MultiScreenViewIsChecked() const
{
	return ViewportClient->GetAB_Side();
}

bool SMetaHumanIdentityViewportToolBar::SplitScreenViewIsChecked() const
{
	return ViewportClient->GetAB_Split();
}

bool SMetaHumanIdentityViewportToolBar::SingleViewIsChecked() const
{
	return ViewportClient->IsShowingSingleView();
}

#undef LOCTEXT_NAMESPACE