// Copyright Epic Games, Inc. All Rights Reserved.

#include "SMetaHumanImageViewer.h"
#include "Brushes/SlateColorBrush.h"

void SMetaHumanImageViewer::Construct(const FArguments& InArgs)
{
	SImage::Construct(SImage::FArguments()
					  .Image(InArgs._Image));
}

FReply SMetaHumanImageViewer::HandleMouseButtonDown(const FGeometry& InGeometry, const FVector2D& InLocalMouse, const FKey& InEffectingButton)
{
	if (InEffectingButton == EKeys::RightMouseButton)
	{
		MouseOrig.X = InLocalMouse.X / InGeometry.GetLocalSize().X;
		MouseOrig.Y = InLocalMouse.Y / InGeometry.GetLocalSize().Y;

		UVOrig = GetImageAttribute().Get()->GetUVRegion();

		bIsPanning = true;
	}

	return FReply::Handled();
}

FReply SMetaHumanImageViewer::HandleMouseButtonUp(const FGeometry& InGeometry, const FVector2D& InLocalMouse, const FKey& InEffectingButton)
{
	if (InEffectingButton == EKeys::RightMouseButton)
	{
		bIsPanning = false;
	}

	return FReply::Handled();
}

FReply SMetaHumanImageViewer::HandleMouseMove(const FGeometry& InGeometry, const FVector2D& InLocalMouse)
{
	if (bIsPanning)
	{
		FVector2D Mouse;
		Mouse.X = InLocalMouse.X / InGeometry.GetLocalSize().X;
		Mouse.Y = InLocalMouse.Y / InGeometry.GetLocalSize().Y;

		FVector2D MouseDelta = MouseOrig - Mouse;

		FBox2D UV = UVOrig;
		UV = UV.ShiftBy(FVector2D(MouseDelta.X * (UV.Max.X - UV.Min.X), MouseDelta.Y * (UV.Max.Y - UV.Min.Y)));

		OnViewChanged.Broadcast(UV);
	}

	return FReply::Handled();
}

FReply SMetaHumanImageViewer::HandleMouseWheel(const FGeometry& InGeometry, const FVector2D& InLocalMouse, float InWheelDelta)
{
	float X = InLocalMouse.X / InGeometry.GetLocalSize().X;
	float Y = InLocalMouse.Y / InGeometry.GetLocalSize().Y;

	FBox2D UV = GetImageAttribute().Get()->GetUVRegion();

	X = UV.Min.X + X * (UV.Max.X - UV.Min.X);
	Y = UV.Min.Y + Y * (UV.Max.Y - UV.Min.Y);

	float Delta = InWheelDelta < 0 ? 1.1 : 1.0 / 1.1;

	UV.Min.X = X - (X - UV.Min.X) * Delta;
	UV.Max.X = X + (UV.Max.X - X) * Delta;
	UV.Min.Y = Y - (Y - UV.Min.Y) * Delta;
	UV.Max.Y = Y + (UV.Max.Y - Y) * Delta;

	OnViewChanged.Broadcast(UV);

	return FReply::Handled();
}

FReply SMetaHumanImageViewer::OnMouseButtonDown(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	return HandleMouseButtonDown(InGeometry, InGeometry.AbsoluteToLocal(InMouseEvent.GetScreenSpacePosition()), InMouseEvent.GetEffectingButton());
}

FReply SMetaHumanImageViewer::OnMouseButtonUp(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	return HandleMouseButtonUp(InGeometry, InGeometry.AbsoluteToLocal(InMouseEvent.GetScreenSpacePosition()), InMouseEvent.GetEffectingButton());
}

FReply SMetaHumanImageViewer::OnMouseMove(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	return HandleMouseMove(InGeometry, InGeometry.AbsoluteToLocal(InMouseEvent.GetScreenSpacePosition()));
}

FReply SMetaHumanImageViewer::OnMouseWheel(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	return HandleMouseWheel(InGeometry, InGeometry.AbsoluteToLocal(InMouseEvent.GetScreenSpacePosition()), InMouseEvent.GetWheelDelta());
}

int32 SMetaHumanImageViewer::OnPaint(const FPaintArgs& InArgs, const FGeometry& InAllottedGeometry,
	const FSlateRect& InWidgetClippingRect, FSlateWindowElementList& OutDrawElements,
	int32 InLayerId, const FWidgetStyle& InWidgetStyle, bool InParentEnabled) const
{
	if (InAllottedGeometry != Geometry)
	{
		Geometry = InAllottedGeometry;
		OnGeometryChanged.Broadcast();
	}

	FSlateColorBrush Brush = FSlateColorBrush(FLinearColor::White);
	FLinearColor Colour = FLinearColor(0, 0, 0);

	if (!GetImageAttribute().Get()->GetResourceObject()) // fill window with black if nothing to display
	{
		FSlateDrawElement::MakeBox(OutDrawElements, InLayerId, InAllottedGeometry.ToPaintGeometry(), &Brush, ESlateDrawEffect::None, Colour);

		return InLayerId;
	}

	SImage::OnPaint(InArgs, InAllottedGeometry, InWidgetClippingRect, OutDrawElements, InLayerId, InWidgetStyle, InParentEnabled);

	if (bDrawBlanking)
	{
		FBox2D UV = GetImageAttribute().Get()->GetUVRegion();
		FVector2D Size = InAllottedGeometry.GetLocalSize();

		if (UV.Min.X < 0)
		{
			float d = (UV.Max.X - UV.Min.X);
			FGeometry Box = InAllottedGeometry.MakeChild(FVector2D(0, 0), FVector2D(-UV.Min.X / d * Size.X, Size.Y), 1);

			FSlateDrawElement::MakeBox(OutDrawElements, InLayerId, Box.ToPaintGeometry(), &Brush, ESlateDrawEffect::None, Colour);
		}

		if (UV.Min.Y < 0)
		{
			float d = (UV.Max.Y - UV.Min.Y);
			FGeometry Box = InAllottedGeometry.MakeChild(FVector2D(0, 0), FVector2D(Size.X, -UV.Min.Y / d * Size.Y), 1);

			FSlateDrawElement::MakeBox(OutDrawElements, InLayerId, Box.ToPaintGeometry(), &Brush, ESlateDrawEffect::None, Colour);
		}

		if (UV.Max.X > 1)
		{
			float d = (UV.Max.X - UV.Min.X);
			FGeometry Box = InAllottedGeometry.MakeChild(FVector2D((-UV.Min.X + 1) / d * Size.X, 0), FVector2D(Size.X - (-UV.Min.X + 1) / d * Size.X, Size.Y), 1);

			FSlateDrawElement::MakeBox(OutDrawElements, InLayerId, Box.ToPaintGeometry(), &Brush, ESlateDrawEffect::None, Colour);
		}

		if (UV.Max.Y > 1)
		{
			float d = (UV.Max.Y - UV.Min.Y);
			FGeometry box = InAllottedGeometry.MakeChild(FVector2D(0, (-UV.Min.Y + 1) / d * Size.Y), FVector2D(Size.X, Size.Y - (-UV.Min.Y + 1) / d * Size.Y), 1);

			FSlateDrawElement::MakeBox(OutDrawElements, InLayerId, box.ToPaintGeometry(), &Brush, ESlateDrawEffect::None, Colour);
		}
	}

	return InLayerId;
}

void SMetaHumanImageViewer::SetNonConstBrush(FSlateBrush* InBrush)
{ 
	NonConstBrush = InBrush;

	OnGeometryChanged.AddRaw(this, &SMetaHumanImageViewer::GeometryChanged);
}

void SMetaHumanImageViewer::ResetView()
{
	if (NonConstBrush)
	{
		GeometryChanged();
	}
}

void SMetaHumanImageViewer::GeometryChanged()
{
	FVector2D WidgetSize = GetPaintSpaceGeometry().GetLocalSize();
	FVector2D ImageSize = NonConstBrush->GetImageSize();

	if (WidgetSize.X < 1 || WidgetSize.Y < 1 || ImageSize.X < 1 || ImageSize.Y < 1)
	{
		return;
	}

	float WidgetAspect = float(WidgetSize.X) / WidgetSize.Y;
	float ImageAspect = float(ImageSize.X) / ImageSize.Y;

	float XRange, YRange;

	if (ImageAspect > WidgetAspect) // fit to width
	{
		XRange = 1.0;
		YRange = ImageAspect / WidgetAspect;
	}
	else // fit to height
	{
		XRange = WidgetAspect / ImageAspect;
		YRange = 1.0;
	}

	NonConstBrush->SetUVRegion(FBox2D(FVector2D(0.5 - XRange / 2, 0.5 - YRange / 2), FVector2D(0.5 + XRange / 2, 0.5 + YRange / 2)));
}

void SMetaHumanImageViewer::SetDrawBlanking(bool bInDrawBlanking)
{
	bDrawBlanking = bInDrawBlanking;
}
