// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentityPoseCustomizations.h"
#include "MetaHumanIdentityPromotedFrames.h"
#include "MetaHumanIdentityParts.h"
#include "MetaHumanIdentity.h"
#include "UI/MetaHumanIdentityStyle.h"

#include "DetailLayoutBuilder.h"
#include "DetailCategoryBuilder.h"
#include "IDetailGroup.h"
#include "IDetailChildrenBuilder.h"
#include "PropertyCustomizationHelpers.h"
#include "IDetailCustomNodeBuilder.h"

#include "Components/Widget.h"
#include "Widgets/Input/SVectorInputBox.h"
#include "Widgets/Input/SRotatorInputBox.h"
#include "Widgets/Input/SSlider.h"
#include "Widgets/Views/STileView.h"
#include "Widgets/Images/SImage.h"
#include "Styling/StyleColors.h"
#include "ScopedTransaction.h"

#define LOCTEXT_NAMESPACE "MetaHumanIdentityPoseCustomization"

const int32 FMetaHumanIdentityBodyCustomization::BodyTypeSubRangeSize = 6; //3 body max indices x 2 genders

/**
 * Thumbnail tile for a body type in the body detail customization.
 */
class SMetaHumanIdentityBodyTile : public STableRow<TSharedPtr<FMetaHumanIdentityBodyType>>
{
public:
	SLATE_BEGIN_ARGS(SMetaHumanIdentityBodyTile) {}
		SLATE_ARGUMENT(TSharedPtr<FMetaHumanIdentityBodyType>, Item)
	SLATE_END_ARGS()

	static TSharedRef<ITableRow> BuildTile(TSharedPtr<FMetaHumanIdentityBodyType> Item, const TSharedRef<STableViewBase>& OwnerTable)
	{
		if (!ensure(Item.IsValid()))
		{
			return SNew(STableRow<TSharedPtr<FMetaHumanIdentityBodyType>>, OwnerTable);
		}

		return SNew(SMetaHumanIdentityBodyTile, OwnerTable).Item(Item);
	}

	void Construct(const FArguments& InArgs, const TSharedRef<STableViewBase>& OwnerTable)
	{
		check(InArgs._Item.IsValid());

		Item = InArgs._Item;

		STableRow::Construct(
			STableRow::FArguments()
			.Style(FAppStyle::Get(), "ProjectBrowser.TableRow")
			.Padding(2.0f)
			.Content()
			[
				SNew(SBorder)
				.Padding(FMargin(0.0f, 0.0f, 5.0f, 5.0f))
				.BorderImage(FAppStyle::Get().GetBrush("ProjectBrowser.ProjectTile.DropShadow"))
				[
					SNew(SOverlay)
					+ SOverlay::Slot()
					[
						SNew(SBox)
						.WidthOverride(128)
						.HeightOverride(128)
						[
							SNew(SBorder)
							.Padding(FMargin(0))
							.BorderImage(FAppStyle::Get().GetBrush("ProjectBrowser.ProjectTile.ThumbnailAreaBackground"))
							.HAlign(HAlign_Fill)
							.VAlign(VAlign_Fill)
							[
								SNew(SImage)
								.Image_Lambda([this]()
								{
									return Item->ThumbnailBrush;
								})
							]
						]
					]
					+ SOverlay::Slot()
					[
						SNew(SImage)
						.Visibility(EVisibility::HitTestInvisible)
						.Image(this, &SMetaHumanIdentityBodyTile::GetSelectionOutlineBrush)
					]
				]
			],
			OwnerTable
		);
	}

	TSharedPtr <FMetaHumanIdentityBodyType> Item;

private:
	const FSlateBrush* GetSelectionOutlineBrush() const
	{
		const bool bIsSelected = IsSelected();
		const bool bIsTileHovered = IsHovered();

		if (bIsSelected && bIsTileHovered)
		{
			static const FName SelectedHover("ProjectBrowser.ProjectTile.SelectedHoverBorder");
			return FAppStyle::Get().GetBrush(SelectedHover);
		}
		else if (bIsSelected)
		{
			static const FName Selected("ProjectBrowser.ProjectTile.SelectedBorder");
			return FAppStyle::Get().GetBrush(Selected);
		}
		else if (bIsTileHovered)
		{
			static const FName Hovered("ProjectBrowser.ProjectTile.HoverBorder");
			return FAppStyle::Get().GetBrush(Hovered);
		}

		return FStyleDefaults::GetNoBrush();
	}
};


/////////////////////////////////////////////////////
// FTrackingContourLayoutBuilder

/**
 * Custom detail panel builder for FFrameTrackingContourData
 */
class FTrackingContourLayoutBuilder
	: public IDetailCustomNodeBuilder
	, public TSharedFromThis<FTrackingContourLayoutBuilder>
{
public:
	FTrackingContourLayoutBuilder(TSharedRef<IPropertyHandle> InTrackingContourProperty, int32 InFrameIndex)
		: TrackingContourProperty{ InTrackingContourProperty }
		, TrackingContourMapProperty{ InTrackingContourProperty->AsMap().ToSharedRef() }
	{
	}

	virtual void GenerateHeaderRowContent(FDetailWidgetRow& InNodeRow) override
	{
		FSimpleDelegate OnNumElementsChanged = FSimpleDelegate::CreateSP(this, &FTrackingContourLayoutBuilder::OnNumMarkersChanged);
		TrackingContourMapProperty->SetOnNumElementsChanged(OnNumElementsChanged);

		uint32 NumMarkers = 0;
		if (TrackingContourMapProperty->GetNumElements(NumMarkers) == FPropertyAccess::Success)
		{
			InNodeRow.NameContent()
			[
				SNew(STextBlock)
				.Font(IDetailLayoutBuilder::GetDetailFont())
				.Text(LOCTEXT("MarkersLabel", "Markers"))
			]
			.ValueContent()
			[
				SNew(STextBlock)
				.Font(IDetailLayoutBuilder::GetDetailFont())
				.Text(this, &FTrackingContourLayoutBuilder::GetHeaderRowText)
			];
		}
		else
		{
			InNodeRow.WholeRowContent()
			.VAlign(VAlign_Center)
			.HAlign(HAlign_Center)
			[
				SNew(STextBlock)
				.Font(IDetailLayoutBuilder::GetDetailFontItalic())
				.Text(LOCTEXT("InvalidMarkersLabel", "Invalid Markers Property"))
			];
		}
	}

	virtual void GenerateChildContent(IDetailChildrenBuilder& InChildrenBuilder) override
	{
		uint32 NumMarkers = 0;
		if (TrackingContourMapProperty->GetNumElements(NumMarkers) == FPropertyAccess::Success)
		{
			for (uint32 MarkerIndex = 0; MarkerIndex < NumMarkers; ++MarkerIndex)
			{
				// This refers to an entry in the TrackingContours TMap of the FFrameTrackingContourData struct
				TSharedPtr<IPropertyHandle> MarkerPropertyHandle = TrackingContourProperty->GetChildHandle(MarkerIndex);

				// Therefore we can get the key handle of the entry, which will be the name of the Marker
				TSharedPtr<IPropertyHandle> MarkerNameHandle = MarkerPropertyHandle->GetKeyHandle();

				// Lets get the actual name of the Marker
				FText MarkerName;
				if (MarkerNameHandle->GetValueAsDisplayText(MarkerName) == FPropertyAccess::Success)
				{
					IDetailGroup& Group = InChildrenBuilder.AddGroup(*MarkerName.ToString(), MarkerName);
					{
						// We then add any child property of FTrackingContour to the panel
						uint32 NumChildren = 0;
						MarkerPropertyHandle->GetNumChildren(NumChildren);

						for (uint32 ChildIndex = 0; ChildIndex < NumChildren; ++ChildIndex)
						{
							Group.AddPropertyRow(MarkerPropertyHandle->GetChildHandle(ChildIndex).ToSharedRef());
						}
					}
				}
			}
		}
	}

	virtual FName GetName() const
	{
		return *FString::Format(TEXT("FTrackingContourLayoutBuilder_{0}"), { FrameIndex });
	}

	virtual void SetOnRebuildChildren(FSimpleDelegate InOnRegenerateChildren)
	{
		OnRebuildChildrenDelegate = InOnRegenerateChildren;
	}

private:

	/** Function called when the number of elements in the TrackingContourMapProperty is changed. This basically request a refresh in the UI */
	void OnNumMarkersChanged() const
	{
		OnRebuildChildrenDelegate.ExecuteIfBound();
	}

	/** Returns the text used for the Header row of the markers array */
	FText GetHeaderRowText() const
	{
		uint32 NumMarkers = 0;
		if (TrackingContourMapProperty->GetNumElements(NumMarkers) == FPropertyAccess::Success)
		{
			return FText::Format(LOCTEXT("NumMarkersLabel", "{0} Markers"), { NumMarkers });
		}
		else
		{
			return LOCTEXT("FailToReadNumMarkersLabel", "Error reading number of markers");
		}
	}

private:
	TSharedRef<IPropertyHandle> TrackingContourProperty;
	TSharedRef<IPropertyHandleMap> TrackingContourMapProperty;
	FSimpleDelegate OnRebuildChildrenDelegate;
	int32 FrameIndex;
};

/////////////////////////////////////////////////////
// FMetaHumanIdentityPromotedFramePropertyCustomization

TSharedRef<IPropertyTypeCustomization> FMetaHumanIdentityPromotedFramePropertyCustomization::MakeInstance()
{
	return MakeShareable(new FMetaHumanIdentityPromotedFramePropertyCustomization{});
}

void FMetaHumanIdentityPromotedFramePropertyCustomization::CustomizeHeader(TSharedRef<IPropertyHandle> InPropertyHandle, FDetailWidgetRow& InHeaderRow, IPropertyTypeCustomizationUtils& InCustomizationUtils)
{
	InHeaderRow.WholeRowContent()
	.VAlign(VAlign_Center)
	[
		SNew(STextBlock)
		.Text(FText::Format(LOCTEXT("PromotedFrameIndex", "Frame {0}"), FText::AsNumber(InPropertyHandle->GetIndexInArray())))
		.Font(IDetailLayoutBuilder::GetDetailFont())
	];
}

void FMetaHumanIdentityPromotedFramePropertyCustomization::CustomizeChildren(TSharedRef<IPropertyHandle> InPropertyHandle, IDetailChildrenBuilder& InChildBuilder, IPropertyTypeCustomizationUtils& InCustomizationUtils)
{
	TSharedRef<IPropertyHandle> FrameNameProperty = InPropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityPromotedFrame, FrameName)).ToSharedRef();
	TSharedRef<IPropertyHandle> UseToSolveProperty = InPropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityPromotedFrame, bUseToSolve)).ToSharedRef();
	TSharedRef<IPropertyHandle> NavigationLockedProperty = InPropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityPromotedFrame, bIsNavigationLocked)).ToSharedRef();
	TSharedRef<IPropertyHandle> CountourTrackerProperty = InPropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityPromotedFrame, ContourTracker)).ToSharedRef();
	TSharedRef<IPropertyHandle> HeadAlignmentProperty = InPropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityPromotedFrame, HeadAlignment)).ToSharedRef();

	InChildBuilder.AddProperty(FrameNameProperty);
	InChildBuilder.AddProperty(UseToSolveProperty);
	InChildBuilder.AddProperty(NavigationLockedProperty);
	InChildBuilder.AddProperty(CountourTrackerProperty);

	UObject* PromotedFrameObject = nullptr;
	if (InPropertyHandle->GetValue(PromotedFrameObject) == FPropertyAccess::Success)
	{
		if (PromotedFrameObject->IsA<UMetaHumanIdentityCameraFrame>())
		{
			TSharedRef<IPropertyHandle> ViewLocationProperty = InPropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityCameraFrame, ViewLocation)).ToSharedRef();
			TSharedRef<IPropertyHandle> ViewRotationProperty = InPropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityCameraFrame, ViewRotation)).ToSharedRef();
			TSharedRef<IPropertyHandle> ViewFOVProperty = InPropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityCameraFrame, CameraViewFOV)).ToSharedRef();
			TSharedRef<IPropertyHandle> ViewLookAtProperty = InPropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityCameraFrame, LookAtLocation)).ToSharedRef();

			IDetailGroup& CameraGroup = InChildBuilder.AddGroup(TEXT("CameraGroupName"), LOCTEXT("CameraGroupLabel", "Camera Transform"));
			{
				const float ValueContentWidth = 125.0f * 3.0f;

				CameraGroup.AddWidgetRow()
				.NameContent()
				[
					SNew(STextBlock)
					.Text(LOCTEXT("CameraLocationLabel", "Location"))
					.Font(IDetailLayoutBuilder::GetDetailFont())
				]
				.ValueContent()
				.MinDesiredWidth(ValueContentWidth)
				.MaxDesiredWidth(ValueContentWidth)
				[
					MakeNumericVectorInputBoxWidget(ViewLocationProperty, NavigationLockedProperty, ETransformField::Location)
				];

				CameraGroup.AddWidgetRow()
				.NameContent()
				[
					SNew(STextBlock)
					.Text(LOCTEXT("CameraRotationLabel", "Rotation"))
					.Font(IDetailLayoutBuilder::GetDetailFont())
				]
				.ValueContent()
				.MinDesiredWidth(ValueContentWidth)
				.MaxDesiredWidth(ValueContentWidth)
				[
					MakeNumericVectorInputBoxWidget(ViewRotationProperty, NavigationLockedProperty, ETransformField::Rotation)
				];

				CameraGroup.AddWidgetRow()
				.NameContent()
				[
					SNew(STextBlock)
					.Text(LOCTEXT("CameraOrbitPivotLabel", "Orbit Pivot"))
					.Font(IDetailLayoutBuilder::GetDetailFont())
				]
				.ValueContent()
				.MinDesiredWidth(ValueContentWidth)
				.MaxDesiredWidth(ValueContentWidth)
				[
					MakeNumericVectorInputBoxWidget(ViewLookAtProperty, NavigationLockedProperty, ETransformField::Location)
				];
			}

			CameraGroup.AddPropertyRow(ViewFOVProperty);
		}
		else if (UMetaHumanIdentityFootageFrame* FootageFrame = Cast<UMetaHumanIdentityFootageFrame>(PromotedFrameObject))
		{
			InChildBuilder.AddProperty(HeadAlignmentProperty);
		}
	}
}

TOptional<FVector::FReal> FMetaHumanIdentityPromotedFramePropertyCustomization::GetAxisValue(TSharedRef<IPropertyHandle> InPropertyHandle, ETransformField InField, EAxis::Type InAxis) const
{
	TOptional<FVector::FReal> Result;

	if (InPropertyHandle->IsValidHandle())
	{
		switch (InField)
		{
		case ETransformField::Location:
		{
			FVector Location = FVector::ZeroVector;
			if (InPropertyHandle->GetValue(Location) == FPropertyAccess::Success)
			{
				Result = Location.GetComponentForAxis(InAxis);
			}
			break;
		}

		case ETransformField::Rotation:
		{
			FRotator Rotator = FRotator::ZeroRotator;
			if (InPropertyHandle->GetValue(Rotator) == FPropertyAccess::Success)
			{
				Result = Rotator.GetComponentForAxis(InAxis);
			}
			break;
		}

		default:
			break;
		}
	}

	return Result;
}

void FMetaHumanIdentityPromotedFramePropertyCustomization::SetAxisValue(FVector::FReal InNewValue, TSharedRef<IPropertyHandle> InPropertyHandle, ETransformField InField, EAxis::Type InAxis) const
{
	if (InPropertyHandle->IsValidHandle())
	{
		switch (InField)
		{

		case ETransformField::Location:
		{
			FVector Vector = FVector::ZeroVector;
			if (InPropertyHandle->GetValue(Vector) == FPropertyAccess::Success)
			{
				Vector.SetComponentForAxis(InAxis, InNewValue);
				InPropertyHandle->SetValue(Vector);
			}
			break;
		}

		case ETransformField::Rotation:
		{
			FRotator Rotator = FRotator::ZeroRotator;
			if (InPropertyHandle->GetValue(Rotator) == FPropertyAccess::Success)
			{
				Rotator.SetComponentForAxis(InAxis, InNewValue);
				InPropertyHandle->SetValue(Rotator);
			}
			break;
		}

		default:
			break;
		}
	}
}

bool FMetaHumanIdentityPromotedFramePropertyCustomization::CanEditCameraTransform(TSharedRef<IPropertyHandle> InNavigationLockedHandle) const
{
	bool bIsLocked = false;
	InNavigationLockedHandle->GetValue(bIsLocked);
	return !bIsLocked;
}

TSharedRef<SWidget> FMetaHumanIdentityPromotedFramePropertyCustomization::MakeNumericVectorInputBoxWidget(TSharedRef<IPropertyHandle> InPropertyHandle, TSharedRef<IPropertyHandle> InIsEnabledProeprty, ETransformField InField) const
{
	switch (InField)
	{
	case ETransformField::Location:
		return SNew(SNumericVectorInputBox<FVector::FReal>)
			.X(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::GetAxisValue, InPropertyHandle, ETransformField::Location, EAxis::X)
			.Y(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::GetAxisValue, InPropertyHandle, ETransformField::Location, EAxis::Y)
			.Z(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::GetAxisValue, InPropertyHandle, ETransformField::Location, EAxis::Z)
			.OnXChanged(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::SetAxisValue, InPropertyHandle, ETransformField::Location, EAxis::X)
			.OnYChanged(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::SetAxisValue, InPropertyHandle, ETransformField::Location, EAxis::Y)
			.OnZChanged(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::SetAxisValue, InPropertyHandle, ETransformField::Location, EAxis::Z)
			.IsEnabled(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::CanEditCameraTransform, InIsEnabledProeprty)
			.Font(IDetailLayoutBuilder::GetDetailFont())
			.bColorAxisLabels(true)
			.AllowSpin(true)
			.SpinDelta(1.0f);

	case ETransformField::Rotation:
		return SNew(SNumericRotatorInputBox<FRotator::FReal>)
			.Roll(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::GetAxisValue, InPropertyHandle, ETransformField::Rotation, EAxis::X)
			.Pitch(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::GetAxisValue, InPropertyHandle, ETransformField::Rotation, EAxis::Y)
			.Yaw(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::GetAxisValue, InPropertyHandle, ETransformField::Rotation, EAxis::Z)
			.OnRollChanged(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::SetAxisValue, InPropertyHandle, ETransformField::Rotation, EAxis::X)
			.OnPitchChanged(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::SetAxisValue, InPropertyHandle, ETransformField::Rotation, EAxis::Y)
			.OnYawChanged(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::SetAxisValue, InPropertyHandle, ETransformField::Rotation, EAxis::Z)
			.IsEnabled(this, &FMetaHumanIdentityPromotedFramePropertyCustomization::CanEditCameraTransform, InIsEnabledProeprty)
			.Font(IDetailLayoutBuilder::GetDetailFont())
			.bColorAxisLabels(true)
			.AllowSpin(true);

	default:
		break;
	}

	return SNew(STextBlock)
		.Text(LOCTEXT("InvalidFieldText", "Invalid value for ETransformField"));
}

/////////////////////////////////////////////////////
// FMetaHumanIdentityPoseCustomization

TSharedRef<IDetailCustomization> FMetaHumanIdentityPoseCustomization::MakeInstance()
{
	return MakeShareable(new FMetaHumanIdentityPoseCustomization{});
}

void FMetaHumanIdentityPoseCustomization::CustomizeDetails(IDetailLayoutBuilder& InDetailBuilder)
{
	IDetailCategoryBuilder& PoseCategory = InDetailBuilder.EditCategory(TEXT("Pose"));
	IDetailCategoryBuilder& TargetCategory = InDetailBuilder.EditCategory(TEXT("Target"));
	IDetailCategoryBuilder& TrackersCategory = InDetailBuilder.EditCategory(TEXT("Trackers"));
	IDetailCategoryBuilder& FramePromotionCategory = InDetailBuilder.EditCategory(TEXT("Frame Promotion"));

	PoseCategory.SetSortOrder(1000);
	TrackersCategory.SetSortOrder(1001);
	TargetCategory.SetSortOrder(1002);
	FramePromotionCategory.SetSortOrder(1003);
}

/////////////////////////////////////////////////////
// FMetaHumanIdentityBodyCustomization

FMetaHumanIdentityBodyCustomization::FMetaHumanIdentityBodyCustomization()
{

	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(0, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.000"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(1, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.001"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(2, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.010"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(3, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.011"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(4, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.020"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(5, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.021"))));

	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(0, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.100"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(1, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.101"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(2, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.110"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(3, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.111"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(4, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.120"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(5, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.121"))));

	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(0, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.200"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(1, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.201"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(2, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.210"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(3, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.211"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(4, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.220"))));
	BodyTypes.Emplace(MakeShareable(new FMetaHumanIdentityBodyType(5, FMetaHumanIdentityStyle::Get().GetBrush("Identity.Body.221"))));
}

TSharedRef<IDetailCustomization> FMetaHumanIdentityBodyCustomization::MakeInstance()
{
	return MakeShareable(new FMetaHumanIdentityBodyCustomization{});
}

void FMetaHumanIdentityBodyCustomization::CustomizeDetails(IDetailLayoutBuilder& InDetailBuilder)
{
	UMetaHumanIdentityBody* Body = nullptr;

	// Get the body object that we're building the details panel for.
	if (!InDetailBuilder.GetSelectedObjects().IsEmpty())
	{
		Body = Cast<UMetaHumanIdentityBody>(InDetailBuilder.GetSelectedObjects()[0].Get());
	}

	// Add a row with a slider for height.
	InDetailBuilder.EditCategory("Body")
	.AddCustomRow(LOCTEXT("Detail_Height", "Height"))
		.NameContent()
		[
			SNew(STextBlock)
			.Font(IDetailLayoutBuilder::GetDetailFont())
			.Text(LOCTEXT("Detail_Height", "Height"))
		]
		.ValueContent()
		[
			SNew(SSlider)
			.MinValue(0)
			.MaxValue(2)
			.StepSize(1)
			.MouseUsesStep(true)
			.Style(&FAppStyle::Get().GetWidgetStyle<FSliderStyle>("AnimBlueprint.AssetPlayerSlider"))
			.Value_Lambda([Body]
			{
				return Body != nullptr ? Body->Height : 1;
			})
			.OnValueChanged_Lambda([this, Body](float NewHeight)
			{
				if (Body)
				{
					const FScopedTransaction Transaction{ UMetaHumanIdentity::IdentityTransactionContext, LOCTEXT("BodyHeightTransaction", "Change Body Height"), Body };
					Body->Modify();
					Body->Height = NewHeight;
					this->SwapThumbnails(Body, NewHeight);
				}
			})
		];

	BodyTypeSubRangeByHeight = GetBodyTypeSubRangeByHeight(BodyTypes, Body->Height);

	// Build out the body selection tile UI.
	InDetailBuilder.EditCategory("Body")
		.AddCustomRow(LOCTEXT("Detail_BodyType", "Body Type"))
		.WholeRowContent()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot()
			.Padding(FMargin(0, 8, 0, 0))
			.MaxWidth(300)
			[
				SAssignNew(TileWidget, STileView<TSharedPtr<FMetaHumanIdentityBodyType>>)
				.ListItemsSource(&BodyTypeSubRangeByHeight)
				.SelectionMode(ESelectionMode::Single)
				.ClearSelectionOnClick(false)
				.ItemAlignment(EListItemAlignment::LeftAligned)
				.OnGenerateTile_Static(&SMetaHumanIdentityBodyTile::BuildTile)
				.ItemHeight(137)
				.ItemWidth(137)
				.OnMouseButtonClick_Lambda([Body](TSharedPtr<FMetaHumanIdentityBodyType> BodyType)
				{
					if (Body != nullptr)
					{
						const FScopedTransaction Transaction{ UMetaHumanIdentity::IdentityTransactionContext, LOCTEXT("BodyTypeTransaction", "Change Body Type"), Body };
						Body->Modify();
						Body->BodyTypeIndex = BodyType->Index;
					}
				})
			]
		];

	// Initially select the tile corresponding to the chosen body type.
	if (Body && BodyTypeSubRangeByHeight.Num() > Body->BodyTypeIndex && Body->BodyTypeIndex != INDEX_NONE)
	{
		TileWidget->SetSelection(BodyTypeSubRangeByHeight[Body->BodyTypeIndex]);
	}
}

/////////////////////////////////////////////////////
// FMetaHumanIdentityCustomization
TSharedRef<IDetailCustomization> FMetaHumanIdentityCustomization::MakeInstance()
{
	return MakeShareable(new FMetaHumanIdentityCustomization{});
}

void FMetaHumanIdentityCustomization::CustomizeDetails(IDetailLayoutBuilder& InDetailBuilder)
{
	InDetailBuilder.EditCategory(TEXT("MetaHuman Identity"), LOCTEXT("MetaHumanIdentity", "MetaHuman Identity")); // prevents it appearing as "Meta SPACE Human"
}

void FMetaHumanIdentityBodyCustomization::SwapThumbnails(UMetaHumanIdentityBody* Body, int32 NewHeight)
{
	BodyTypeSubRangeByHeight = GetBodyTypeSubRangeByHeight(BodyTypes, NewHeight);
	FChildren* Children = TileWidget->GetChildren();

	for (int32 BodyTypeIndex = 0; BodyTypeIndex < BodyTypeSubRangeSize; BodyTypeIndex++)
	{
		BodyTypeSubRangeByHeight[BodyTypeIndex]->ThumbnailBrush = BodyTypes[NewHeight*BodyTypeSubRangeSize + BodyTypeIndex]->ThumbnailBrush;
	}

	// Select the thumbnail in the new subrange based on the previously selected index, if any
	if (Body != nullptr)
	{
		if (Body && Body->BodyTypeIndex != INDEX_NONE && Body->BodyTypeIndex < BodyTypeSubRangeByHeight.Num())
		{
			TileWidget->SetSelection(BodyTypeSubRangeByHeight[Body->BodyTypeIndex]);
		}
	}
	TileWidget->RequestListRefresh();
}

TArray<TSharedPtr<FMetaHumanIdentityBodyType>> FMetaHumanIdentityBodyCustomization::GetBodyTypeSubRangeByHeight(TArray<TSharedPtr<FMetaHumanIdentityBodyType>>& BodyTypesFullRange, int32 Height)
{
	//the height of the characters in the thumbnails should change on slider move
	//prepare a subset of 6 of 18 in the thumbnails array to generate the UI for those chosen by the height slider
	TArray<TSharedPtr<FMetaHumanIdentityBodyType>> BodyTypeSubRange;

	int32 BodyTypeSubRangeStart = Height * BodyTypeSubRangeSize;
	int32 BodyTypeSubRangeEnd = BodyTypeSubRangeSize * (Height + 1);

	for (int BodyTypeIndex = BodyTypeSubRangeStart; BodyTypeIndex < BodyTypeSubRangeEnd; BodyTypeIndex++)
	{
		BodyTypeSubRange.Add(BodyTypesFullRange[BodyTypeIndex]);
	}
	return BodyTypeSubRange;
}

#undef LOCTEXT_NAMESPACE