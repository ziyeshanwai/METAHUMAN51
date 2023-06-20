// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "UObject/ObjectPtr.h"
#include "IDetailCustomization.h"
#include "IPropertyTypeCustomization.h"
#include "Widgets/Views/STileView.h"

class UMetaHumanIdentityBody;

/////////////////////////////////////////////////////
// FMetaHumanIdentityPromotedFramePropertyCustomization

/**
 * Detail customization for a UMetaHumanIdentityPromotedFrame
 */
class FMetaHumanIdentityPromotedFramePropertyCustomization
	: public IPropertyTypeCustomization
{
public:
	static TSharedRef<IPropertyTypeCustomization> MakeInstance();

	//~ IPropertyTypeCustomization interface
	virtual void CustomizeHeader(TSharedRef<IPropertyHandle> InPropertyHandle, FDetailWidgetRow& InHeaderRow, IPropertyTypeCustomizationUtils& InCustomizationUtils) override;
	virtual void CustomizeChildren(TSharedRef<IPropertyHandle> InPropertyHandle, IDetailChildrenBuilder& InChildBuilder, IPropertyTypeCustomizationUtils& InCustomizationUtils) override;

private:
	enum class ETransformField
	{
		Location,
		Rotation,
	};

	/** Get the axis value of the given PropertyHandle */
	TOptional<FVector::FReal> GetAxisValue(TSharedRef<IPropertyHandle> InPropertyHandle, ETransformField InField, EAxis::Type InAxis) const;

	/** Sets the axis value of given PropertyHandle */
	void SetAxisValue(FVector::FReal InNewValue, TSharedRef<IPropertyHandle> InPropertyHandle, ETransformField InField, EAxis::Type InAxis) const;

	/** Whether or not the camera transform can be edited based on the NavigationLocked property */
	bool CanEditCameraTransform(TSharedRef<IPropertyHandle> InNavigationLockedHandle) const;

	/**
	 * Create a SNumericVectorInputBox widget for the given property handle
	 * @param InPropertyHandle The property handle to read and write values to
	 * @param InIsEnabledProperty The property handle used to enable/disable the widget
	 * @param InField If the field represents a location or rotation value
	 */
	TSharedRef<SWidget> MakeNumericVectorInputBoxWidget(TSharedRef<IPropertyHandle> InPropertyHandle, TSharedRef<IPropertyHandle> InIsEnabledProperty, ETransformField InField) const;
};

/////////////////////////////////////////////////////
// FMetaHumanIdentityPoseCustomization

/**
 * Detail customization for a UMetaHumanIdentityPose
 */
class FMetaHumanIdentityPoseCustomization
	: public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();

	//~ IDetailCustomization interface
	virtual void CustomizeDetails(IDetailLayoutBuilder& InDetailBuilder) override;
};

/////////////////////////////////////////////////////
// FMetaHumanIdentityBodyCustomization

struct FMetaHumanIdentityBodyType
{
	int32 Index;
	const FSlateBrush* ThumbnailBrush;

	FMetaHumanIdentityBodyType(int32 InIndex, const FSlateBrush* InThumbnailBrush)
		: Index(InIndex)
		, ThumbnailBrush(InThumbnailBrush)
	{}
};

/**
 * Detail customization for a UMetaHumanIdentityBody
 */
class FMetaHumanIdentityBodyCustomization
	: public IDetailCustomization
{
public:

	FMetaHumanIdentityBodyCustomization();

	static TSharedRef<IDetailCustomization> MakeInstance();

	//~ IDetailCustomization interface
	virtual void CustomizeDetails(IDetailLayoutBuilder& InDetailBuilder) override;

private:

	void SwapThumbnails(UMetaHumanIdentityBody* Body, int32 NewHeight);

	TArray<TSharedPtr<FMetaHumanIdentityBodyType>> GetBodyTypeSubRangeByHeight(TArray<TSharedPtr<FMetaHumanIdentityBodyType>>& BodyTypesFullRange, int32 Height);

	/** A list of all available body types */
	TArray<TSharedPtr<FMetaHumanIdentityBodyType>> BodyTypes;
	/** A subset (based on the selected height) of body types shown as thumbnails that the user can select */
	TArray<TSharedPtr<FMetaHumanIdentityBodyType>> BodyTypeSubRangeByHeight;
	/** TileView container, allowing for swapping thumbnails on height slider move after the widgets has beein constructed **/
	TSharedPtr<STileView<TSharedPtr<FMetaHumanIdentityBodyType>>> TileWidget;
	static const int32 BodyTypeSubRangeSize;
};

/**
 * Detail customization for a UMetaHumanIdentity
 */
class FMetaHumanIdentityCustomization
	: public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();

	//~ IDetailCustomization interface
	virtual void CustomizeDetails(IDetailLayoutBuilder& InDetailBuilder) override;
};