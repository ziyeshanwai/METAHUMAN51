// Copyright Epic Games, Inc.All Rights Reserved.

#include "STrackerImageViewer.h"

#include "Polygon2.h"
#include "ShapeAnnotationWrapper.h"
#include "EditorStyleSet.h"

void STrackerImageViewer::Construct(const FArguments& InArgs)
{
	PointSize = 5;
	bIsNavigationLocked = InArgs._IsNavigationLocked;
	OnCurvesSelectedDelegate = InArgs._OnCurvesSelected;

	OnGeometryChanged.AddSP(this, &STrackerImageViewer::UpdatePointPositionImageSpace);

	DefaultColor = FLinearColor::Green;
	HighlightedColor = FLinearColor::White;
	SelectedColor = FLinearColor::Yellow;
	DeactivatedColor = FLinearColor::Gray;

	SMetaHumanImageViewer::Construct(SMetaHumanImageViewer::FArguments().Image(InArgs._Image));
}

FReply STrackerImageViewer::OnMouseButtonDown(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	if (!bIsNavigationLocked.Get())
	{
		return FReply::Unhandled();
	}
	
	FVector2D MousPosition = InGeometry.AbsoluteToLocal(InMouseEvent.GetScreenSpacePosition());

	bool bMakingSelection = ResolveSelectionForMouseClick(InMouseEvent, MousPosition);
	bool bManipulationInitiated = SetManipulationStateForMouseClick(MousPosition);

	if(!bMakingSelection && !bManipulationInitiated)
	{
		SelectedPointIDs.Empty();
		SelectedCurveNames.Empty();
	}
	
	OnCurvesSelectedDelegate.ExecuteIfBound(SelectedCurveNames);

	return SMetaHumanImageViewer::OnMouseButtonDown(InGeometry, InMouseEvent);
}

FReply STrackerImageViewer::OnMouseButtonUp(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	if (!bIsNavigationLocked.Get())
	{
		return FReply::Unhandled();
	}

	if(bMovingSinglePoint || bMovingSelection)
	{
		ShapeAnnotation->StoreMovedPointPosition();
	}

	if(bDrawSelection)
	{
		DrawSelectionBoxEnd = InGeometry.AbsoluteToLocal(InMouseEvent.GetScreenSpacePosition());
		if(FVector2D::Distance(DrawSelectionBoxStart, DrawSelectionBoxEnd) > 10)
		{
			ResolveSelectionFromBox();
		}
	}
	else if(InMouseEvent.IsControlDown())
	{
		FVector2D LocalMouse = InGeometry.AbsoluteToLocal(InMouseEvent.GetScreenSpacePosition());
		AddRemoveKey(LocalMouse);
	}

	bDrawSelection = false;
	bMovingSinglePoint = false;
	bMovingSelection = false;

	return SMetaHumanImageViewer::OnMouseButtonUp(InGeometry, InMouseEvent);
}

FReply STrackerImageViewer::OnMouseMove(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	if (!bIsNavigationLocked.Get())
	{
		return FReply::Unhandled();
	}

	// Ignore mouse events coming in too fast.
	// If an event takes 0.1 seconds to process, have an effective mouse update rate of 5Hz (twice 0.1)
	// Without this the widget is constantly busy and does not refresh on screen.

	double MouseMoveStartTime = FPlatformTime::Seconds();

	if ((MouseMoveStartTime - MouseMoveLastTime) < MouseMoveElapsed * 2)
	{
		return FReply::Handled();
	}

	FVector2D LocalMouse = InGeometry.AbsoluteToLocal(InMouseEvent.GetScreenSpacePosition());
	FReply RetVal = SMetaHumanImageViewer::OnMouseMove(InGeometry, InMouseEvent);

	if(bIsPanning)
	{
		UpdatePointPositionImageSpace();
	}
	if(bDrawSelection)
	{
		DrawSelectionBoxEnd = LocalMouse;
	}

	if(bMovingSinglePoint)
	{
		FTrackingPoint NewPoint {GetPointAtPosition(LocalMouse), LocalMouse};
		PointsOnSpline[HighlightedPointID] = NewPoint;
		MovePointPosition(LocalMouse);
	}
	else if(bMovingSelection)
	{
		FVector2D MoveDelta = GetPointPositionOnImage(SelectionMoveStart) - GetPointPositionOnImage(LocalMouse);
		ShapeAnnotation->OffsetSelectedPoints(SelectedPointIDs, MoveDelta);
		SelectionMoveStart = LocalMouse;
		UpdatePointPositionImageSpace();
	}
	else
	{
		ResolveHighlightingForMouseMove(LocalMouse);
	}

	double Now = FPlatformTime::Seconds();
	MouseMoveLastTime = Now;
	MouseMoveElapsed = Now - MouseMoveStartTime;

	return  RetVal;
}

FReply STrackerImageViewer::OnMouseWheel(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	if (!bIsNavigationLocked.Get())
	{
		return FReply::Unhandled();
	}

	FReply RetVal = SMetaHumanImageViewer::OnMouseWheel(InGeometry, InMouseEvent);
	UpdatePointPositionImageSpace();

	return RetVal;
}

FReply STrackerImageViewer::OnKeyDown(const FGeometry& MyGeometry, const FKeyEvent& InKeyEvent)
{
	if (!bIsNavigationLocked.Get())
	{
		return FReply::Unhandled();
	}

	if(InKeyEvent.GetKey() == EKeys::Escape)
	{
		SelectedPointIDs.Empty();
		return FReply::Handled();
	}

	FReply KeyProcessed = SMetaHumanImageViewer::OnKeyDown(MyGeometry, InKeyEvent);

	if(InKeyEvent.IsControlDown() && (InKeyEvent.GetKey() == EKeys::Z || InKeyEvent.GetKey() == EKeys::Y))
	{
		UpdatePointPositionImageSpace();
	}

	return KeyProcessed;
}

int32 STrackerImageViewer::OnPaint(const FPaintArgs& InArgs, const FGeometry& InAllottedGeometry,
                                   const FSlateRect& InWidgetClippingRect, FSlateWindowElementList& OutDrawElements, int32 InLayerId,
                                   const FWidgetStyle& InWidgetStyle, bool bInParentEnabled) const
{
	if (InAllottedGeometry != Geometry)
	{
		Geometry = InAllottedGeometry;
		OnGeometryChanged.Broadcast();
	}
	
	int32 LayerId = InLayerId;
	if (bIsNavigationLocked.Get())
	{
		LayerId = SMetaHumanImageViewer::OnPaint(InArgs, InAllottedGeometry, InWidgetClippingRect, OutDrawElements, InLayerId, InWidgetStyle, bInParentEnabled);
	}

	DrawTrackingPoints(OutDrawElements, LayerId + 1, InAllottedGeometry);
	DrawTrackingCurves(OutDrawElements, LayerId + 1, InAllottedGeometry);
	DrawSelectionBoxes(OutDrawElements, LayerId + 1, InAllottedGeometry);

	return LayerId;
}

void STrackerImageViewer::SetTrackerImageSize(const FIntPoint& InTrackerImageSize)
{
	TrackerImageSize = FVector2D(InTrackerImageSize.X, InTrackerImageSize.Y);
}

void STrackerImageViewer::SetSelectedCurves(const TArray<FString>& InSelectedCurves)
{
	SelectedCurveNames = InSelectedCurves;
	SelectedPointIDs.Empty();

	for (const FString& Name : SelectedCurveNames)
	{
		for (const int32 ID : GetPointIDsForCurve((Name)))
		{
			SelectedPointIDs.Add(ID);
		}
	}

	UpdatePointPositionImageSpace();
}

void STrackerImageViewer::ReloadTrackingData(const TSharedPtr<FShapeAnnotationWrapper> InShapeAnnotation)
{
	ShapeAnnotation = InShapeAnnotation;
	UpdatePointPositionImageSpace();
}

void STrackerImageViewer::ResetViewOverride(bool bInResetImageGeometry)
{
	if(bInResetImageGeometry)
	{
		SMetaHumanImageViewer::ResetView();
	}

	UpdatePointPositionImageSpace();
}

void STrackerImageViewer::UpdatePointPositionImageSpace()
{
	if(ShapeAnnotation.IsValid())
	{
		FBox2D UV = GetImageAttribute().Get()->GetUVRegion();
		FVector2D WidgetSize = GetPaintSpaceGeometry().GetLocalSize();

		SplinePoints.Empty();
		TMap<FString, TArray<FVector2D>> DenseSplinePoints = ShapeAnnotation->GetSplinePointsForDraw();
		for(const auto& DensePoints : DenseSplinePoints)
		{
			TArray<FVector2D> WidgetSpacePoints;
			for(const auto& Point : DensePoints.Value)
			{
				WidgetSpacePoints.Add(GetPointPositionOnScreen(Point, UV, WidgetSize, TrackerImageSize));
			}
			SplinePoints.Add(DensePoints.Key, WidgetSpacePoints);
		}
		
		PointsOnSpline = ShapeAnnotation->GetPointsOnSplineForDraw();
		for(TPair<int32, FTrackingPoint> &Point : PointsOnSpline)
		{
			Point.Value.PointPosition = GetPointPositionOnScreen(Point.Value.PointPosition, UV, WidgetSize, TrackerImageSize);
			Point.Value.LinePoints = GetPointAtPosition(Point.Value.PointPosition);
		}
	}
}

void STrackerImageViewer::MovePointPosition(const FVector2D& InMousePos)
{
	FVector2D PosImageSpace = GetPointPositionOnImage(InMousePos);
	ShapeAnnotation->UpdateKeyPosition(HighlightedPointID, PosImageSpace);
	UpdatePointPositionImageSpace();
}

void STrackerImageViewer::AddRemoveKey(const FVector2D& InMousePos)
{
	FVector2D MousePosImageSpace = GetPointPositionOnImage(InMousePos);
	ShapeAnnotation->AddRemoveKey(MousePosImageSpace);
	UpdatePointPositionImageSpace();
}

void STrackerImageViewer::SetMarkerVisibility(bool InShowCurves, bool InShowPoints)
{
	bDrawCurvesEnabled = InShowCurves;
	bDrawPointsEnabled = InShowPoints;
}

FVector2D STrackerImageViewer::GetPointPositionOnScreen(const FVector2D& InImagePosition, const FBox2D& InUV, const FVector2D& InWidgetSize, const FVector2D& InImageSize) const
{
	FVector2D OffsetPos = InImagePosition;

	// 0..1 Normalized Coords
	OffsetPos = OffsetPos / InImageSize;

	// -1..1 projection space
	OffsetPos.X = (OffsetPos.X - 0.5) * 2.0;
	OffsetPos.Y = ((1.0 - OffsetPos.Y) - 0.5) * 2.0;

	// The horizontal field of view is fixed so we need to apply the widget aspect ratio to the Y coordinate
	// to get the correct placement
	OffsetPos.Y *= InWidgetSize.X / InWidgetSize.Y;

	// 0..1 Normalized Coords
	OffsetPos.X = (OffsetPos.X / 2.0) + 0.5;
	OffsetPos.Y = 1.0 - (OffsetPos.Y / 2.0) - 0.5;

	// Back to Widget space
	OffsetPos -= InUV.Min;
	OffsetPos *= InWidgetSize / (InUV.Max - InUV.Min);

	return OffsetPos;
}

FVector2D STrackerImageViewer::GetPointPositionOnImage(const FVector2D& InScreenPosition) const
{
	FBox2D UV = GetImageAttribute().Get()->GetUVRegion();
	FVector2D WidgetSize = GetPaintSpaceGeometry().GetLocalSize();

	FVector2D OffsetPos = InScreenPosition;

	// 0..1 Normalized Coords
	OffsetPos = OffsetPos * (UV.Max - UV.Min) / WidgetSize;
	OffsetPos += UV.Min;

	// -1..1 Projection space
	OffsetPos.X = (OffsetPos.X - 0.5) * 2.0;
	OffsetPos.Y = ((1.0 - OffsetPos.Y) - 0.5) * 2.0;

	// The horizontal field of view is fixed so we need to apply the widget aspect ratio to the Y coordinate
	// to get the correct placement
	OffsetPos.Y /= WidgetSize.X / WidgetSize.Y;

	// 0..1 Normalized Coords
	OffsetPos.X = (OffsetPos.X / 2.0) + 0.5;
	OffsetPos.Y = 1.0 - (OffsetPos.Y / 2.0) - 0.5;

	// Back to Image space
	OffsetPos *= TrackerImageSize;

	return OffsetPos;
}

FLinearColor STrackerImageViewer::GetPointColor(const int32 InPointID) const
{
	// Highlighted point should override any previously set colors
	if(InPointID == HighlightedPointID)
	{
		return HighlightedColor;
	}
	
	FLinearColor Color = DefaultColor;
	TArray<FString> CurveNames = GetCurveNamesForPointID(InPointID);
	bool bActive = false;
	bool bSelected = false;

	for(const FString& CurveName : CurveNames)
	{
		const FMarkerCurveState* VisibilityState = ShapeAnnotation->VisibilityMap[CurveName];
		bActive |= VisibilityState->bActive;
		bSelected |= VisibilityState->bSelected;
	}
	
	if(bSelected || SelectedPointIDs.Contains(InPointID))
	{
		Color = SelectedColor;
	}
	else if(!bActive)
	{
		Color = DeactivatedColor;
	}
	
	return Color;
}

FLinearColor STrackerImageViewer::GetCurveColor(const FString& InCurveName) const
{
	FLinearColor Color = DefaultColor;
	const FMarkerCurveState* VisibilityState = ShapeAnnotation->VisibilityMap[InCurveName];
	
	if(InCurveName == HighlightedCurveName)
	{
		Color = HighlightedColor;
	}
	else if(VisibilityState->bSelected)
	{
		Color = SelectedColor;
	}
	else if(!VisibilityState->bActive)
	{
		Color = DeactivatedColor;
	}

	return Color;
}

TArray<FString> STrackerImageViewer::GetCurveNamesForPointID(const int32 InPointID) const
{
	if(PointsOnSpline.Contains(InPointID))
	{
		return PointsOnSpline[InPointID].CurveNames;
	}

	return {};
}

void STrackerImageViewer::DrawTrackingPoints(FSlateWindowElementList& OutDrawElements, int32 InLayerId, const FGeometry& AllottedGeometry) const
{
	if(bDrawPointsEnabled)
	{
		FPaintGeometry MyGeometry = AllottedGeometry.ToPaintGeometry();
		FSlateFontInfo SummaryFont = FCoreStyle::GetDefaultFontStyle("Regular", 8);

		for(const auto& Point : PointsOnSpline)
		{
			FLinearColor Color = GetPointColor(Point.Key);
			FSlateDrawElement::MakeLines(OutDrawElements, InLayerId, MyGeometry,	Point.Value.LinePoints,	ESlateDrawEffect::None,	Color);
		}
	}
}

void STrackerImageViewer::DrawTrackingCurves(FSlateWindowElementList& OutDrawElements, int32 InLayerId, const FGeometry& AllottedGeometry) const
{
	if(bDrawCurvesEnabled)
	{
		FPaintGeometry MyGeometry = AllottedGeometry.ToPaintGeometry();

		for(const auto& Points : SplinePoints)
		{
			if(Points.Value.Num() > 1)
			{
				FLinearColor Color = GetCurveColor(Points.Key);
				FSlateDrawElement::MakeLines(OutDrawElements, InLayerId, MyGeometry,	Points.Value, ESlateDrawEffect::None, Color);
			}
		}
	}
}

void STrackerImageViewer::DrawSelectionBoxes(FSlateWindowElementList& OutDrawElements, int32 InLayerId,	const FGeometry& AllottedGeometry) const
{
	FPaintGeometry MyGeometry = AllottedGeometry.ToPaintGeometry();
	static const FName BorderName("AnimTimeline.Outliner.DefaultBorder");

	const float InverseScale = Inverse(AllottedGeometry.Scale);

	if(bDrawSelection)
	{
		double DrawBoxWidth = fabs(DrawSelectionBoxStart.X - DrawSelectionBoxEnd.X);
		double DrawBoxHeight = fabs(DrawSelectionBoxStart.Y - DrawSelectionBoxEnd.Y);
		double StartX = FMath::Min(DrawSelectionBoxStart.X, DrawSelectionBoxEnd.X);
		double StartY = FMath::Min(DrawSelectionBoxStart.Y, DrawSelectionBoxEnd.Y);
		FVector2D StartPos = {StartX, StartY};

		auto BoxGeometry = AllottedGeometry.ToPaintGeometry(TransformVector(InverseScale, FVector2D(DrawBoxWidth, DrawBoxHeight)), FSlateLayoutTransform(TransformPoint(InverseScale, StartPos)));
		FSlateDrawElement::MakeBox(OutDrawElements, InLayerId, BoxGeometry, FAppStyle::GetBrush(BorderName), ESlateDrawEffect::None, FLinearColor(1.0f, 1.0f, 1.0f, 0.2f));
	}
}

void STrackerImageViewer::ResolveHighlightingForMouseMove(const FVector2D& InMousePosition)
{
	HighlightedCurveName.Empty();
	HighlightedPointID = 0;

	bool bCheckCurveHighlight = bDrawCurvesEnabled;
	if(bDrawPointsEnabled)
	{
		bCheckCurveHighlight &= !SetHighlightingFromPoint(InMousePosition);
	}
	
	if(bCheckCurveHighlight)
	{
		SetHighlightingFromCurve(InMousePosition);
	}
}

void STrackerImageViewer::ResolveSelectionFromBox()
{
	FBox2D SelectionBox = FBox2D(DrawSelectionBoxStart, DrawSelectionBoxEnd);
	SelectedPointIDs.Empty();
	SelectedCurveNames.Empty();

	if(bDrawCurvesEnabled || bDrawPointsEnabled)
	{
		for(const TPair<int32, FTrackingPoint>& Point : PointsOnSpline)
		{
			if(SelectionBox.IsInside(Point.Value.PointPosition))
			{
				SelectedPointIDs.Add(Point.Key);
			}
		}

		PopulateCurveSelectionListFromSelectedPoints();
	}
}

void STrackerImageViewer::PopulateSelectionListForMouseClick()
{
	bool bSelectionHandledForCurves = false;
	if(SelectedCurveNames.Contains(HighlightedCurveName))
	{
		SelectedCurveNames.Remove(HighlightedCurveName);
		for(const int32 ID : GetPointIDsForCurve((HighlightedCurveName)))
		{
			SelectedPointIDs.Remove(ID);
		}
		bSelectionHandledForCurves = true;
	}
	else if(!HighlightedCurveName.IsEmpty())
	{
		SelectedCurveNames.Add(HighlightedCurveName);
		for(const int32 ID : GetPointIDsForCurve((HighlightedCurveName)))
		{
			SelectedPointIDs.Add(ID);
		}
		bSelectionHandledForCurves = true;
	}

	if(!bSelectionHandledForCurves)
	{
		if(SelectedPointIDs.Contains(HighlightedPointID))
		{
			SelectedPointIDs.Remove(HighlightedPointID);
			for(const FString& CurveName : GetCurveNamesForPointID(HighlightedPointID))
			{
				SelectedCurveNames.Remove(CurveName);
			}
		}
		else if(HighlightedPointID > 0)
		{
			SelectedPointIDs.Add(HighlightedPointID);
			PopulateCurveSelectionListFromSelectedPoints();
		}
	}	
}

// TODO: Come back with a fresh brain and make this entire selection logic more simple
void STrackerImageViewer::PopulateCurveSelectionListFromSelectedPoints()
{
	TMap<FString, TArray<int32>> SelectedPointsPerCurve;
	for(const TPair<int32, FTrackingPoint>& Point : PointsOnSpline)
	{
		for(const FString& CurveName : Point.Value.CurveNames)
		{
			if(SelectedPointsPerCurve.Contains(CurveName))
			{
				SelectedPointsPerCurve[CurveName].Add(Point.Key);
			}
			else
			{
				SelectedPointsPerCurve.Add(CurveName, {Point.Key});
			}
		}
	}

	for(const TPair<FString, TArray<int32>>& PointsOnCurve : SelectedPointsPerCurve)
	{
		bool bAllPointsSelected = false;
		for(int32 PointID : PointsOnCurve.Value)
		{
			if(!SelectedPointIDs.Contains(PointID))
			{
				bAllPointsSelected = false;
				break;
			}

			bAllPointsSelected = true;
		}

		if(bAllPointsSelected)
		{
			SelectedCurveNames.Add(PointsOnCurve.Key);
		}
	}

	if(!SelectedCurveNames.IsEmpty())
	{
		OnCurvesSelectedDelegate.ExecuteIfBound(SelectedCurveNames);
	}
}

TArray<FVector2D> STrackerImageViewer::GetPointAtPosition(const FVector2D& InScreenPosition) const
{
	const UE::Geometry::TPolygon2<float> Poly = UE::Geometry::FPolygon2f::MakeCircle(PointSize, LinesPerCircle);
	TArray<FVector2D> Point;
    Point.Reserve(Poly.GetVertices().Num());
	for(const UE::Math::TVector2<float>& Vert : Poly.GetVertices())
	{
		Point.Add(FVector2D(Vert.X, Vert.Y) + InScreenPosition);
	}

	return Point;
}

TArray<int32> STrackerImageViewer::GetPointIDsForCurve(const FString& InCurveName) const
{
	TArray<int32> PointIDs;
	for(const TPair<int32, FTrackingPoint>& PointData : PointsOnSpline)
	{
		if(PointData.Value.CurveNames.Contains(InCurveName))
		{
			PointIDs.Add(PointData.Key);
		}
	}
	
	return PointIDs;
}

bool STrackerImageViewer::SetHighlightingFromPoint(const FVector2D& InMousePos)
{
	for(const TPair<int32, FTrackingPoint>& Point : PointsOnSpline)
	{
		if(FVector2D::Distance(InMousePos, Point.Value.PointPosition) < PointSize + 1)
		{
			HighlightedPointID = Point.Key;
			return true;
		}
	}

	return false;
}

// This is brute force, need to check if rlibv has anything more clever than this
bool STrackerImageViewer::SetHighlightingFromCurve(const FVector2D& InMousePos)
{
	// TODO: This value seems to only work if you're zoomed out. Need to make the threshold more dynamic
	const double Threshold = 5;

	for(const TPair<FString, TArray<FVector2D>>& DenseCurveData : SplinePoints)
	{
		for(const FVector2D& Point : DenseCurveData.Value)
		{
			if(FVector2D::Distance(InMousePos, Point) < Threshold)
			{
				HighlightedCurveName = DenseCurveData.Key;
				return true;
			}
		}
	}
	return false;
}

bool STrackerImageViewer::ResolveSelectionForMouseClick(const FPointerEvent& InMouseEvent, const FVector2D& InMousePos)
{
	bool bIsSelecting = false;

	if(InMouseEvent.IsShiftDown())
	{
		PopulateSelectionListForMouseClick();
		
		DrawSelectionBoxStart = InMousePos;
		bDrawSelection = true;
		bIsSelecting = true;
	}
	else if(!HighlightedCurveName.IsEmpty() && !SelectedCurveNames.Contains(HighlightedCurveName))
	{
		SelectedCurveNames.Empty();
		SelectedPointIDs.Empty();
		
		SelectedCurveNames.Add(HighlightedCurveName);
		for(const int32 ID : GetPointIDsForCurve((HighlightedCurveName)))
		{
			SelectedPointIDs.Add(ID);
		}
		bIsSelecting = true;
	}
	else if(HighlightedPointID > 0 && !SelectedPointIDs.Contains(HighlightedPointID))
	{
		SelectedCurveNames.Empty();
		SelectedPointIDs.Empty();
		SelectedPointIDs.Add(HighlightedPointID);
		bIsSelecting = true;
	}
	
	return bIsSelecting;
}

bool STrackerImageViewer::SetManipulationStateForMouseClick(const FVector2D& InMousePos)
{
	bMovingSelection = false;

	if(!SelectedCurveNames.IsEmpty() || SelectedPointIDs.Num() > 1)
	{
		bool bClickedOnSelectedCurve = SelectedCurveNames.Contains(HighlightedCurveName);
		bool bClickedOnSelectedPoint = SelectedPointIDs.Contains(HighlightedPointID);
		bMovingSelection = bClickedOnSelectedCurve || bClickedOnSelectedPoint;
		
		if(bMovingSelection)
		{
			SelectionMoveStart = InMousePos;
		}
	}

	bMovingSinglePoint = !bMovingSelection && HighlightedPointID > 0;
	if(bMovingSinglePoint || bMovingSelection)
	{
		bDrawSelection = false;
	}
	
	return bMovingSelection || bMovingSinglePoint;
}
