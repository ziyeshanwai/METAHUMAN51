// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/Images/SImage.h"

DECLARE_MULTICAST_DELEGATE_OneParam(FOnViewChanged, FBox2D);
DECLARE_MULTICAST_DELEGATE(FOnGeometeryChanged);

class METAHUMANIMAGEVIEWER_API SMetaHumanImageViewer : public SImage
{
public:

	SLATE_BEGIN_ARGS(SMetaHumanImageViewer) {}

		/** Image resource */
		SLATE_ATTRIBUTE(const FSlateBrush*, Image)

	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	FReply HandleMouseButtonDown(const FGeometry& InGeometry, const FVector2D& InLocalMouse, const FKey& InEffectingButton);
	FReply HandleMouseButtonUp(const FGeometry& InGeometry, const FVector2D& InLocalMouse, const FKey& InEffectingButton);
	FReply HandleMouseMove(const FGeometry& InGeometry, const FVector2D& InLocalMouse);
	FReply HandleMouseWheel(const FGeometry& InGeometry, const FVector2D& InLocalMouse, float InWheelDelta);

	virtual FReply OnMouseButtonDown(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;
	virtual FReply OnMouseButtonUp(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;
	virtual FReply OnMouseMove(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;
	virtual FReply OnMouseWheel(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;

	virtual int32 OnPaint(const FPaintArgs& InArgs, const FGeometry& InAllottedGeometry,
		const FSlateRect& InWidgetClippingRect, FSlateWindowElementList& OutDrawElements,
		int32 InLayerId, const FWidgetStyle& InWidgetStyle, bool InParentEnabled) const override;

	FOnViewChanged OnViewChanged;

	void SetNonConstBrush(FSlateBrush* InBrush);
	virtual void ResetView();

	void SetDrawBlanking(bool bInDrawBlanking);

protected:

	FBox2D UVOrig;
	FVector2D MouseOrig;
	bool bIsPanning = false;
	mutable FGeometry Geometry;

	FOnGeometeryChanged OnGeometryChanged;
	FSlateBrush* NonConstBrush = nullptr;
	void GeometryChanged();

	bool bDrawBlanking = true;
};
