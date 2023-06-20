// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Math/Color.h"
#include "SMetaHumanImageViewer.h"

class UTexture;
class UTextureRenderTarget2D;
class UMaterialInstanceDynamic;

enum class EABImageViewMode
{
	// Ensure any change is reflected in the material shader

	A = 0, // Single view modes
	B,

	ABSplit, // Multi view modes
	ABSide,

	Current, // Special modes for quering per-view parameter state
	Any
};

enum class EABImageNavigationMode
{
	ThreeD = 0,
	TwoD
};

enum class EABImageMouseSide
{
	NotApplicable = 0,
	A,
	B
};

class METAHUMANIMAGEVIEWER_API SABImage : public SMetaHumanImageViewer, public FGCObject
{
public:

#if WITH_EDITOR
	DECLARE_MULTICAST_DELEGATE(FOnInvalidate);

	FOnInvalidate& OnInvalidate() { return OnInvalidateDelegate; }
#endif

	void Setup(bool bInManageTextures);

	virtual void AddReferencedObjects(FReferenceCollector&) override;
	virtual FString GetReferencerName() const override;

	virtual FReply OnMouseButtonDown(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;
	virtual FReply OnMouseButtonUp(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;
	virtual FReply OnMouseMove(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;
	virtual FReply OnMouseWheel(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;

	virtual void ResetView() override;

	virtual int32 OnPaint(const FPaintArgs& InArgs, const FGeometry& InAllottedGeometry,
		const FSlateRect& InWidgetClippingRect, FSlateWindowElementList& OutDrawElements,
		int32 InLayerId, const FWidgetStyle& InWidgetStyle, bool InParentEnabled) const override;

	void SetTextures(UTexture* InTextureA, UTexture* InTextureB);
	UTextureRenderTarget2D* GetRenderTarget(EABImageViewMode InMode) const;

	void SetViewMode(EABImageViewMode InViewMode);
	EABImageViewMode GetViewMode() const;

	TArray<EABImageViewMode> SingleViewModes() const { return { EABImageViewMode::A, EABImageViewMode::B }; }
	bool IsSingleView() const { return SingleViewModes().Contains(ViewMode); }
	bool IsMultiView() const { return !IsSingleView(); }
	bool IsTextureView() const { return (IsMultiView() || NavigationMode == EABImageNavigationMode::TwoD); }

	void SetNavigationMode(EABImageNavigationMode InNavigationMode);
	EABImageNavigationMode GetNavigationMode() const;

private:

#if WITH_EDITOR
	FOnInvalidate OnInvalidateDelegate;
#endif

	TMap<EABImageViewMode, TObjectPtr<UTextureRenderTarget2D>> RenderTarget;

	EABImageViewMode ViewMode = EABImageViewMode::A;
	EABImageNavigationMode NavigationMode = EABImageNavigationMode::ThreeD;

	FVector2D Origin;
	float Angle;
	float Alpha;

	const float OriginSize = 20;
	const float LineTickness = 1;
	const float PickSensitivity = 5;
	const float LabelOffset = 30;
	const float AlphaLineLength = 200;

	const FLinearColor NormalColour = FLinearColor(1, 0, 0);
	const FLinearColor HighlightedColour = FLinearColor(1, 1, 0);

	bool bOriginHightlighted = false;
	bool bOriginMove = false;
	bool bAngleHightlighted = false;
	bool bAngleMove = false;
	bool bAlphaHightlighted = false;
	bool bAlphaMove = false;

	FVector2D OriginOffset;
	float AngleOffset;

	void GetLines(const FGeometry& InGeometry, TArray<FVector2D>& OutOrigin, TArray<FVector2D>& OutAngle, TArray<FVector2D>& OutAlpha) const;
	bool HitLines(const FVector2D& InPoint, const TArray<FVector2D>& InLines) const;

	FOnGeometeryChanged OnGeometryChanged;

	mutable FGeometry Geometry;

	void GeometryChanged();

	TObjectPtr<UMaterialInstanceDynamic> MaterialInstance = nullptr;

	FSlateBrush Brush;

	FVector2D Get2DLocalMouse(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent, EABImageMouseSide &OutMouseSide);
	EABImageMouseSide MouseSideOrig = EABImageMouseSide::NotApplicable;
};
