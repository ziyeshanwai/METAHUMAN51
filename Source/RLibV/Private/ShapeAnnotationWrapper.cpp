// Copyright Epic Games, Inc. All Rights Reserved.

#include "ShapeAnnotationWrapper.h"
#include <Math/NumericLimits.h>
#include "Polygon2.h"

#include <rlibv/disable_dlib_warnings.h>
RLIBV_DISABLE_DLIB_WARNINGS
#include <rlibv/shape_annotation.h>
#include <rlibv/linear_pdm.h>
#include <rlibv/shape_refiner.h>
#include <rlibv/keypoint_curve.h>
#include <rlibv/keypoint.h>
#include <dlib/image_processing.h>
#include <dlib/image_io.h>
RLIBV_RENABLE_WARNINGS


class FShapeAnnotationWrapper::FImpl
{
public:
	
	FVector2D GetFVectorFromPoint2d(const rlibv::point2d<double>& InPoint) const;
	TArray<FVector2D> TArrFromVec(const std::vector<rlibv::point2d<double>>& InPoints) const;
	FString FStringFromStd(const std::string& InStdString) const;
	FString GetCurveNameForKeyPoint(const std::string& InKeyPointName);

	std::vector<rlibv::point2d<double>> VecFromTArray(const TArray<FVector2D>& InTArray) const;
	std::string StdFromFString(const FString& InFString) const;
	bool FindCurveInsertionPoint(const FVector2D& InMouse, float& OutDist, rlibv::point2d<double>& OutPointPos, std::string& OutName, int32& OutIndex);
	bool FindNearestControlPoint(const FVector2D& InMouse, float& OutDist, std::string& OutName, int32& OutIndex);
	
	int32 GetReducedPointNumber(const TArray<FVector2D>& InTrackingPoints) const;
	
	TMap<int32,rlibv::point2d<double>*> IdToPointReference;	

	float AddRemoveCoefficient = 0.25f;
	float SelectionCaptureRange = 40.0f;
	
	std::map<std::string, int> n_dense_internals_;
	std::map<std::string, int> n_keypoint_internals_;
	
	rlibv::shape_annotation ShapeAnnotation;

	std::stack<rlibv::shape_annotation> undo_stack_;
	std::stack<rlibv::shape_annotation> redo_stack_;
};

FVector2D FShapeAnnotationWrapper::FImpl::GetFVectorFromPoint2d(const rlibv::point2d<double>& InPoint) const
{
	return FVector2D(InPoint.x(), InPoint.y());
}

TArray<FVector2D> FShapeAnnotationWrapper::FImpl::TArrFromVec(const std::vector<rlibv::point2d<double>>& InPoints) const
{
	TArray<FVector2D> RetVal;
	for(const rlibv::point2d<double>& Point : InPoints)
	{
		RetVal.Add(GetFVectorFromPoint2d(Point));
	}

	return RetVal;
}

std::vector<rlibv::point2d<double>> FShapeAnnotationWrapper::FImpl::VecFromTArray(const TArray<FVector2D>& InTArray) const
{
	rlibv::shape2d<double> Points;
	Points.reserve(InTArray.Num());
	for(const FVector2D& TrackingPoint : InTArray)
	{
		rlibv::point2d<double> PointPos{TrackingPoint.X, TrackingPoint.Y};
		Points.push_back(PointPos);
	}
	return Points;
}

std::string FShapeAnnotationWrapper::FImpl::StdFromFString(const FString& InFString) const
{
	return std::string(TCHAR_TO_UTF8(ToCStr((InFString))));
}

bool FShapeAnnotationWrapper::FImpl::FindCurveInsertionPoint(const FVector2D& InMouse, float& OutDist,
	rlibv::point2d<double>& OutPointPos, std::string& OutName, int32& OutIndex)
{
	float BestDistance = TNumericLimits<float>::Max();
	int32 SelectedSplineIndex = -1;
	std::vector<rlibv::point2d<double>> AllSplinePoints;
	auto Splines = ShapeAnnotation.get_drawing_splines(n_dense_internals_);
	
	for (const auto& SplineItem : Splines)
	{
		const auto& TheseSplinePts = SplineItem.second;
		for (int i = 0; i < TheseSplinePts.size(); ++i)
		{
			FVector2D PointPos = GetFVectorFromPoint2d(TheseSplinePts[i]);
			float Distance = FVector2D::Distance(PointPos, InMouse);
			if (Distance < BestDistance && Distance < SelectionCaptureRange)
			{
				BestDistance = Distance;
				OutName = SplineItem.first;
				SelectedSplineIndex = i;
				AllSplinePoints = TheseSplinePts;
			}
		}
	}

	if(OutName.empty())
	{
		return false;
	}

	//Where are the internal curve points in the spline?
	const auto& InternalPts = ShapeAnnotation.keypoint_curves().at(OutName).internal_points;
	auto NInternal = static_cast<int>(InternalPts.size());
	std::vector<int> InternalCPIndsInSpline(NInternal);
	for (int IP = 0; IP < NInternal; ++IP)
	{
		float BestDistanceInternal = TNumericLimits<float>::Max();
		int32 BestSplineIndex = -1;
		for (int32 SplineIndex = 0; SplineIndex < AllSplinePoints.size(); ++SplineIndex)
		{
			double Distance = static_cast<double>((InternalPts[IP] - AllSplinePoints[SplineIndex]).length());
			if (Distance < BestDistanceInternal)
			{
				BestDistanceInternal = Distance;
				BestSplineIndex = SplineIndex;
			}
		}
		InternalCPIndsInSpline[IP] = BestSplineIndex;
	}

	OutIndex = NInternal;
	for (int IP = 0; IP < NInternal; ++IP)
	{
		if (SelectedSplineIndex < InternalCPIndsInSpline[IP])
		{
			OutIndex = IP;
			break;
		}
	}

	OutPointPos = AllSplinePoints[SelectedSplineIndex];
	OutDist = BestDistance;
	
	return true;
}

bool FShapeAnnotationWrapper::FImpl::FindNearestControlPoint(const FVector2D& InMouse, float& OutDist, std::string& OutName, int32& OutIndex)
{
	float BestDistance = TNumericLimits<float>::Max();
	// Check for internal curve points
	for (const auto& KeypointCurve : ShapeAnnotation.keypoint_curves())
	{
		int32 InternalPointCtr = 0;
		for (const auto& InternalPoint : KeypointCurve.second.internal_points)
		{
			FVector2D PointPos = GetFVectorFromPoint2d(InternalPoint);
			float Distance = FVector2D::Distance(PointPos, InMouse);
			
			if(Distance < BestDistance && Distance < SelectionCaptureRange)
			{
				BestDistance = Distance;
				OutName = KeypointCurve.first;
				OutIndex = InternalPointCtr;
			}
			++InternalPointCtr;
		}
	}
	OutDist = BestDistance;
	
	return true;
}

int32 FShapeAnnotationWrapper::FImpl::GetReducedPointNumber(const TArray<FVector2D>& InTrackingPoints) const
{
	const double Tolerance = 0.00001;
	const double Distance = 1;
	
	UE::Geometry::FPolygon2d Polygon;
	Polygon.AppendVertices(InTrackingPoints);
	Polygon.Simplify(Tolerance, Distance);

	int32 ReducedNumber = Polygon.VertexCount() < 4 ? 4 : Polygon.VertexCount();

	return ReducedNumber;
}

FString FShapeAnnotationWrapper::FImpl::FStringFromStd(const std::string& InStdString) const
{
	return FString(InStdString.c_str());
}

FString FShapeAnnotationWrapper::FImpl::GetCurveNameForKeyPoint(const std::string& InKeyPointName)
{
	FString CurveName;
	for(const auto& Curve: ShapeAnnotation.keypoint_curves())
	{
		if(Curve.second.end_keypoint_name == InKeyPointName || Curve.second.start_keypoint_name == InKeyPointName)
		{
			CurveName = FStringFromStd(Curve.first);
			break;
		}
	}

	return CurveName;
}

FShapeAnnotationWrapper::FShapeAnnotationWrapper()
{
	try
	{
		Impl = MakePimpl<FImpl>();
	}
	catch (...)
	{
		checkf(false, TEXT("Exception"));
	}
}

FShapeAnnotationWrapper::~FShapeAnnotationWrapper()
{
	try
	{
		Impl = nullptr;
	}
	catch (...)
	{
		checkf(false, TEXT("Exception"));
	}
}

TMap<FString, TArray<FVector2D>> FShapeAnnotationWrapper::GetSplinePointsForDraw() const
{
	TMap<FString,TArray<FVector2D>> SplinePoints4Draw;
	std::map<std::string, std::vector<rlibv::point2d<double>>> SplineData;
	try
	{
		SplineData = Impl->ShapeAnnotation.get_drawing_splines(Impl->n_dense_internals_);
	}
	catch (...)
	{
	}

	for(const auto& Spline : SplineData)
	{
		FString FeatureName = FString(Spline.first.c_str());
		if(CurveIsVisible(FeatureName))
		{
			TArray<FVector2D> Points = Impl->TArrFromVec(Spline.second);
			SplinePoints4Draw.Add(FeatureName, Points);
		}
	}

	return SplinePoints4Draw;
}

TMap<FString, TArray<FVector2D>> FShapeAnnotationWrapper::GetControlVertices() const
{
	TMap<FString, TArray<FVector2D>> ControlVerts;
	auto InitializedCurves = Impl->ShapeAnnotation.keypoint_curves();
	
	for(const auto& Curve : InitializedCurves)
	{
		FString Name = Impl->FStringFromStd(Curve.first);
		TArray<FVector2D> InternalPoints = Impl->TArrFromVec(Curve.second.internal_points);
		ControlVerts.Add(Name, InternalPoints);
	}
	
	return ControlVerts;
}

bool FShapeAnnotationWrapper::CurveIsVisible(const FString& InCurveName) const
{
	if(VisibilityMap.Contains(InCurveName))
	{
		return VisibilityMap[InCurveName]->bVisible;
	}

	return false;
}

void FShapeAnnotationWrapper::InitializeFromLandmarkData(const FFrameTrackingContourData& InLandmarkData) const
{
    std::map<std::string, rlibv::keypoint_curve> KeypointCurves;
    std::map<std::string, rlibv::keypoint> KeyPoints;
	
	for(const TPair<FString, FTrackingContour>& Contour : InLandmarkData.TrackingContours)
	{
		std::string FeatureName = std::string(TCHAR_TO_UTF8(ToCStr((Contour.Key))));
		int32 Size = Contour.Value.DensePoints.Num();
		if (Size > 1)
		{
			Impl->n_dense_internals_.insert({ FeatureName, Size });

			rlibv::shape2d<double> SplinePoints;
			SplinePoints.reserve(Size);
			for(const FVector2D& TrackingPoint : Contour.Value.DensePoints)
			{
				rlibv::point2d<double> PointPos{TrackingPoint.X, TrackingPoint.Y};
				SplinePoints.push_back(PointPos);
			}

			rlibv::keypoint_curve Curve;
			Curve.start_keypoint_name = std::string(TCHAR_TO_UTF8(*Contour.Value.StartPointName));
			Curve.end_keypoint_name = std::string(TCHAR_TO_UTF8(*Contour.Value.EndPointName));

			rlibv::shape2d<double> ReducedInternalPoints;

			if(Contour.Value.ControlVertices.IsEmpty())
			{
				TArray<FVector2D> ControlVerts = GetControlVerticesForCurve(Contour.Value.DensePoints);
				ReducedInternalPoints = Impl->VecFromTArray(ControlVerts);
			}
			else
			{
				ReducedInternalPoints = Impl->VecFromTArray(Contour.Value.ControlVertices);
			}
			
			Curve.internal_points = ReducedInternalPoints;
			KeypointCurves.insert({FeatureName, Curve});
		}
		else if(Size == 1)
		{
			rlibv::keypoint Keypoint;
			FVector2D LandmarkPoint = Contour.Value.DensePoints.Last();
			Keypoint.pos = {LandmarkPoint.X, LandmarkPoint.Y};
			KeyPoints.insert({FeatureName, Keypoint});
		}
	}

	Impl->ShapeAnnotation.initialize("", KeyPoints, KeypointCurves);
}

void FShapeAnnotationWrapper::ModifyKeyPointData(const FString& InFeature, const TArray<FVector2D>& InLandmarkData, const TArray<FVector2D>& InInternalPoints) const
{
	auto& InitializedCurves = Impl->ShapeAnnotation.keypoint_curves();
	auto& InitializedKeypoints = Impl->ShapeAnnotation.keypoints();
	
	std::string FeatureName = std::string(TCHAR_TO_UTF8(ToCStr((InFeature))));
	
	if(InitializedCurves.find(FeatureName) != InitializedCurves.end())
	{
		rlibv::shape2d<double> ControlVertices;
		ControlVertices.reserve(InInternalPoints.Num());
		for(const FVector2D& InternalPoint : InInternalPoints)
		{
			rlibv::point2d<double> PointPos{InternalPoint.X, InternalPoint.Y};
			ControlVertices.push_back(PointPos);
		}

		InitializedCurves[FeatureName].internal_points = ControlVertices;
	}

	if(InitializedKeypoints.find(FeatureName) != InitializedKeypoints.end())
	{
		const rlibv::point2d<double> PointPos{InLandmarkData.Last().X, InLandmarkData.Last().Y};
		InitializedKeypoints[FeatureName].pos = PointPos;
	}
}

TArray<FVector2D> FShapeAnnotationWrapper::GetControlVerticesForCurve(const TArray<FVector2D>& InLandmarkData) const
{
	TArray<FVector2D> ControlVerts;
	if(!InLandmarkData.IsEmpty())
	{
		rlibv::shape2d<double> SplinePoints;
		SplinePoints.reserve(InLandmarkData.Num());
		for(const FVector2D& TrackingPoint : InLandmarkData)
		{
			rlibv::point2d<double> PointPos{TrackingPoint.X, TrackingPoint.Y};
			SplinePoints.push_back(PointPos);
		}

		const int32 NumberOfReducedKeys = Impl->GetReducedPointNumber(InLandmarkData);

		rlibv::shape2d<double> ReducedInternalPoints = SplinePoints;
		try
		{
			ReducedInternalPoints = rlibv::approximate_evenly_spaced_curve(SplinePoints, NumberOfReducedKeys, false);;
		}
		catch (...)
		{
		}

		ReducedInternalPoints.erase(ReducedInternalPoints.begin());
		ReducedInternalPoints.pop_back();

		ControlVerts = Impl->TArrFromVec(ReducedInternalPoints);
	}

	return ControlVerts;
}

TMap<int32, FTrackingPoint> FShapeAnnotationWrapper::GetPointsOnSplineForDraw()
{
	TMap<int, FTrackingPoint> Points;

	Impl->IdToPointReference.Empty();

	std::map<std::string, rlibv::keypoint_curve>& KeypointCurves = Impl->ShapeAnnotation.keypoint_curves();
	std::map<std::string, rlibv::keypoint>& KeyPoints = Impl->ShapeAnnotation.keypoints();

	int32 PointId = 0;
	std::map<std::string, int32> KeyPointNameToIDMap;

	for(auto& KeyPoint : KeyPoints)
	{
		if(CurveIsVisible(Impl->FStringFromStd(KeyPoint.first)))
		{
			FTrackingPoint PointToAdd;
			PointToAdd.PointPosition = Impl->GetFVectorFromPoint2d(KeyPoint.second.pos);
			Points.Add(++PointId, PointToAdd);
			KeyPointNameToIDMap.insert({KeyPoint.first, PointId});

			Impl->IdToPointReference.Add(PointId, &KeyPoint.second.pos);
		}
	}
	
	for(auto& Curve : KeypointCurves)
	{
		if(CurveIsVisible(Impl->FStringFromStd(Curve.first)))
		{
			int32 CurveStartPointID = KeyPointNameToIDMap[Curve.second.start_keypoint_name];
			int32 CurveEndPointID = KeyPointNameToIDMap[Curve.second.end_keypoint_name];

			Points[CurveStartPointID].CurveNames.Add(Impl->FStringFromStd(Curve.first));
			Points[CurveEndPointID].CurveNames.Add(Impl->FStringFromStd(Curve.first));
			
			for(auto& Point : Curve.second.internal_points)
			{
				FTrackingPoint PointToAdd;
				PointToAdd.CurveNames.Add(Impl->FStringFromStd(Curve.first));
				PointToAdd.PointPosition = Impl->GetFVectorFromPoint2d(Point);
				Points.Add(++PointId, PointToAdd);
				Impl->IdToPointReference.Add(PointId, &(Point));
			}
		}
	}
	
	return Points;
}

void FShapeAnnotationWrapper::UpdateKeyPosition(const int32 InKey, const FVector2D& InPosition) const
{
	auto PointToMov = Impl->IdToPointReference[InKey];
	PointToMov->x() = InPosition.X;
	PointToMov->y() = InPosition.Y;
}

void FShapeAnnotationWrapper::OffsetSelectedPoints(const TSet<int32>& InSelectedPointIDs, const FVector2D& InOffset) const
{
	const rlibv::point2d<double> Offset = rlibv::point2d<double>(InOffset.X, InOffset.Y);
	
	for(const int32 ID : InSelectedPointIDs)
	{
		auto& PointToMove = Impl->IdToPointReference[ID];
		PointToMove->x() -= Offset.x();
		PointToMove->y() -= Offset.y();
	}
}

void FShapeAnnotationWrapper::OffsetCurve(const FString& InCurveName, const FVector2D InOffset) const
{
	std::map<std::string, rlibv::keypoint_curve>& KeypointCurves = Impl->ShapeAnnotation.keypoint_curves();
	std::map<std::string, rlibv::keypoint>& KeyPoints = Impl->ShapeAnnotation.keypoints();

	std::string CurveString = Impl->StdFromFString(InCurveName);
	const rlibv::point2d<double> Offset = rlibv::point2d<double>(InOffset.X, InOffset.Y);
	
	if(KeypointCurves.count(CurveString))
	{	
		for(auto& Point : KeypointCurves[CurveString].internal_points)
		{
			Point -= Offset;
		}

		KeyPoints[KeypointCurves[CurveString].start_keypoint_name].pos -= Offset;
		KeyPoints[KeypointCurves[CurveString].end_keypoint_name].pos -= Offset;
	}
	else if(KeyPoints.count(CurveString))
	{
		KeyPoints[CurveString].pos -= Offset;
	}
}

void FShapeAnnotationWrapper::StoreMovedPointPosition() const
{
	OnCurveModifedDelegate.ExecuteIfBound();
}

void FShapeAnnotationWrapper::AddRemoveKey(const FVector2D& InMousePosImageSpace) const
{
	std::string InsertName, ControlPointName;
	float InsertDistance = 0.0f, RemoveDistance = 0.0f;
	int32 InsertIndex = -1, ControlPointIndex = -1;
	rlibv::point2d<double> OutPointPos;

	bool bCurveInsertionFound = Impl->FindCurveInsertionPoint(InMousePosImageSpace, InsertDistance, OutPointPos, InsertName, InsertIndex);
	Impl->FindNearestControlPoint(InMousePosImageSpace, RemoveDistance, ControlPointName, ControlPointIndex);

	if(bCurveInsertionFound && InsertDistance < Impl->AddRemoveCoefficient * RemoveDistance)
	{
		Impl->ShapeAnnotation.insert_internal_point(InsertName, InsertIndex, OutPointPos);
		StoreMovedPointPosition();
	}
	else if(Impl->AddRemoveCoefficient * RemoveDistance <= InsertDistance)
	{
		Impl->ShapeAnnotation.remove_internal_point(ControlPointName, ControlPointIndex);
		StoreMovedPointPosition();
	}
}

void FShapeAnnotationWrapper::ProcessUndo() const
{
	if (!Impl->undo_stack_.empty())
	{
		Impl->redo_stack_.push(Impl->ShapeAnnotation);
		Impl->ShapeAnnotation = Impl->undo_stack_.top();
		Impl->undo_stack_.pop();
	}
}

void FShapeAnnotationWrapper::ProcessRedo() const
{
	if (!Impl->redo_stack_.empty())
	{
		Impl->undo_stack_.push(Impl->ShapeAnnotation);
		Impl->ShapeAnnotation = Impl->redo_stack_.top();
		Impl->redo_stack_.pop();
		
	}
}
