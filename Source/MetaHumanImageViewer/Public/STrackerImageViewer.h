// Copyright Epic Games, Inc.All Rights Reserved.

#pragma once

#include "SMetaHumanImageViewer.h"
#include "FrameTrackingContourData.h"

class FShapeAnnotationWrapper;

DECLARE_DELEGATE_OneParam(FOnCurvesSelected, const TArray<FString>& InCurveNames)

class METAHUMANIMAGEVIEWER_API STrackerImageViewer : public SMetaHumanImageViewer
{
public:

	SLATE_BEGIN_ARGS(STrackerImageViewer)
		: _IsNavigationLocked(true)
	{}
		/** Image resource */
		SLATE_ATTRIBUTE(const FSlateBrush*, Image)

		/** If the navigation is locked display the 2D image */
		SLATE_ATTRIBUTE(bool, IsNavigationLocked)

		SLATE_EVENT(FOnCurvesSelected, OnCurvesSelected)

	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	virtual FReply OnMouseButtonDown(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;
	virtual FReply OnMouseButtonUp(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;
	virtual FReply OnMouseMove(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;
	virtual FReply OnMouseWheel(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent) override;
	virtual FReply OnKeyDown(const FGeometry& MyGeometry, const FKeyEvent& InKeyEvent) override;

	virtual bool SupportsKeyboardFocus() const override { return true; }

	virtual int32 OnPaint(const FPaintArgs& InArgs, const FGeometry& InAllottedGeometry,
		const FSlateRect& InWidgetClippingRect, FSlateWindowElementList& OutDrawElements,
		int32 InLayerId, const FWidgetStyle& InWidgetStyle, bool InParentEnabled) const override;

	/** Sets the size of the underlying tracker image. Used to calculate the correct placement of curves and points in the image being displayed */
	void SetTrackerImageSize(const FIntPoint& InTrackerImageSize);

	void SetSelectedCurves(const TArray<FString>& InSelectedCurves);
	void ReloadTrackingData(TSharedPtr<FShapeAnnotationWrapper> InShapeAnnotation);
	void ResetViewOverride(bool bInResetImageGeometry);
	void SetMarkerVisibility(bool InShowCurves, bool InShowPoints);

private:

	FVector2D GetPointPositionOnScreen(const FVector2D& InOriginalPosition, const FBox2D& InUV, const FVector2D& InWidgetSize, const FVector2D& InImageSize) const;
	FVector2D GetPointPositionOnImage(const FVector2D& InScreenPosition) const;
	FLinearColor GetPointColor(const int32 InPointID) const;
	FLinearColor GetCurveColor(const FString& InCurveName) const;
	TArray<FString> GetCurveNamesForPointID(const int32 InPointID) const;

	void DrawTrackingPoints(FSlateWindowElementList& OutDrawElements, int32 InLayerId, const FGeometry& AllottedGeometry) const;
	void DrawTrackingCurves(FSlateWindowElementList& OutDrawElements, int32 InLayerId, const FGeometry& AllottedGeometry) const;
	void DrawSelectionBoxes(FSlateWindowElementList& OutDrawElements, int32 InLayerId, const FGeometry& AllottedGeometry) const;
	
	void ResolveHighlightingForMouseMove(const FVector2D& InMousePosition);
	void ResolveSelectionFromBox();
	void PopulateSelectionListForMouseClick();
	void PopulateCurveSelectionListFromSelectedPoints();
	void UpdatePointPositionImageSpace();
	void MovePointPosition(const FVector2D& InMousePos);
	void AddRemoveKey(const FVector2D& InMousePos);

	TArray<FVector2D> GetPointAtPosition(const FVector2D& InScreenPosition) const;
	TArray<int32> GetPointIDsForCurve(const FString& InCurveName) const;
	bool SetHighlightingFromPoint(const FVector2D& InMousePos);
	bool SetHighlightingFromCurve(const FVector2D& InMousePos);
	bool ResolveSelectionForMouseClick(const FPointerEvent& InMouseEvent, const FVector2D& InMousePos);
	bool SetManipulationStateForMouseClick(const FVector2D& InMousePos);

	TMap<int32, FTrackingPoint> PointsOnSpline;
	TMap<FString, TArray<FVector2D>> SplinePoints;
	TSet<int32> SelectedPointIDs;
	TArray<FString> SelectedCurveNames;

	FString HighlightedCurveName;
	FVector2D TrackerImageSize;
	FVector2D SelectionMoveStart;
	FVector2D DrawSelectionBoxStart;
	FVector2D DrawSelectionBoxEnd;

	const int32 LinesPerCircle = 33;
	int32 HighlightedPointID = 0;
	int32 PointSize;
	
	bool bMovingSinglePoint = false;
	bool bMovingSelection = false;
	bool bDrawSelection = false;
	bool bDrawPointsEnabled = true;
	bool bDrawCurvesEnabled = true;

	TAttribute<bool> bIsNavigationLocked;
	
	FOnCurvesSelected OnCurvesSelectedDelegate;

	TSharedPtr<FShapeAnnotationWrapper> ShapeAnnotation;

	FLinearColor DefaultColor;
	FLinearColor HighlightedColor;
	FLinearColor SelectedColor;
	FLinearColor DeactivatedColor;

	double MouseMoveLastTime = 0;
	double MouseMoveElapsed = 0;
};
