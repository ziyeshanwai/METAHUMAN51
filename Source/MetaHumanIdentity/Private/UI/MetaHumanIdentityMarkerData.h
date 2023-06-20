// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "FrameTrackingContourData.h"
#include "MetaHumanIdentityMarkerData.generated.h"

// WIP:
//#define SHOW_OUTLINER 

class UMetaHumanIdentityPromotedFrame;
struct FMarkerCurveState;

USTRUCT()
struct FMarkerGroupState
{
	GENERATED_BODY()

	FMarkerGroupState()
		: bVisible(false)
		, bActive(false)
		, bSelected(false)
	{}

	UPROPERTY()
	uint8 bVisible : 1;

	UPROPERTY()
	uint8 bActive : 1;

	UPROPERTY(Transient)
	uint8 bSelected : 1;
};

/** persistent data for curves, loaded from json */
struct FMarkerCurveDef
{
	FString Name;
	FString StartPointName;
	FString EndPointName;
	TArray<int32> VertexIDs;

	TArray<FString> GroupTagIDs;
};

struct FMarkerDefs
{
	TArray<FString> GroupNames;
	TArray<FMarkerCurveDef> CurveDefs;
	TMap<FString, int32> Landmarks;
};

/** UI data for frames, needed to create the ListView */
struct FMarkerOutlinerFrameData
{
	FString Name;
	uint32 FrameNumber;
	UMetaHumanIdentityPromotedFrame* Frame;
	bool bSelected; //there is just one frame list, so selection state can go directly in it
};

/** UI data for groups (per frame) */
struct FMarkerOutlinerGroupData
{
	FString Name;
	uint32 NumFrames; //number of frames this group participates in
	FMarkerGroupState State;
};

/* UI data for curves (per frame) */
struct FMarkerOutlinerCurveData
{
	FString Name;
	FString StartPointName;
	FString EndPointName;

	FMarkerCurveState State;
};

/* A structure holding data used by UI to populate ListView
   ListView expects an array of pointers to Items structs (not USTRUCTs!) to create rows, so that's a given */
struct FMarkerData
{
	FMarkerDefs* MarkerDefs;

	TArray<TSharedPtr<FMarkerOutlinerFrameData>> MarkerFrames;
	TArray<TSharedPtr<FMarkerOutlinerGroupData>> MarkerGroups;
	TArray<TSharedPtr<FMarkerOutlinerCurveData>> MarkerCurves;
};