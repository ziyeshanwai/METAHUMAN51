// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Pipeline/Node.h"
#include "Pipeline/PipelineData.h"
#include "UObject/WeakObjectPtr.h"
#include "UObject/GCObject.h"

#include "SupressWarnings.h"

MH_DISABLE_EIGEN_WARNINGS
#include "Eigen/Dense"
MH_ENABLE_WARNINGS

class UNeuralNetwork;

namespace UE::MetaHuman::Pipeline
{
	class METAHUMANPIPELINE_API FHyprsenseNodeBase : public FNode, public FGCObject
	{
	public:
		enum ErrorCode
		{
			InvalidTracker = 0,
			ModelNotLoaded,
			InvalidIOConfig
		};

		enum class ETrackerType : uint8
		{
			FaceTracker = 0,
			FaceDetector,
			EyebrowTracker,
			EyeTracker,
			LipsTracker,
			NasoLabialTracker,
			LipsNasolabialTracker
		};

		struct Interval
		{
			//Defines continious interval of indices [Start,End]
			int32 Start{};
			int32 End{};

			//Any additional indices added to continious interval above
			TArray<int32> AdditionalIndices{};
		};

		struct NNIModelInfo
		{
			TArray<int32> Inputs{};
			TArray<int32> Outputs{};
		};

		FHyprsenseNodeBase(const FString& InTypeName, const FString& InName);

		virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) = 0;
		virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) = 0;

		FString GetErrorMessage() const;
		ErrorCode GetErrorCode() const;

		virtual void AddReferencedObjects(FReferenceCollector& InCollector) override;
		virtual FString GetReferencerName() const override;

	protected:
		using Matrix23f = Eigen::Matrix<float, 2, 3>;
		using Matrix32f = Eigen::Matrix<float, 3, 2>;
		using Matrix33f = Eigen::Matrix<float, 3, 3>;

		struct Bbox
		{
			float X1, Y1;
			float X2, Y2;
			float Score;
			float Area;
		};

		enum FacePart
		{
			RightEyeBrow,
			LeftEyeBrow,
			RightEye,
			LeftEye,
			Lips,
			Nasolabial,
			LipsNasolabial,
			Num = 7
		};

		struct PartPoints
		{
			TArray<float> Points;
		};

		TArray<Bbox> HardNMS(const TArray<float>& InScores, const TArray<float>& InBoxes, float InIouThreshold, float InProbThreshold, int32 InTotalSize, int32 InTopK = -1);
		float IOU(const Bbox& InBox1, const Bbox& InBox2);

		Matrix23f GetTransformFromBbox(const Bbox& InBbox, int32 InImageWidth, int32 InImageHeight, int32 InCropBoxSize, float InRatio, float InRotation, bool bFlip);
		Matrix23f GetTransformFromLandmarkPart(int32 InImageWidth, int32 InImageHeight, int32 InCropBoxSize, float InRatio, const TArray<float>& InLandmarks, float InRotation, bool bFlip);
		Matrix23f GetTransformFromLandmarkFacePart(int32 InImageWidth, int32 InImageHeight, int32 InCropBoxSize, float InRatio, const TArray<float>& InLandmarks, FacePart InPartName, float InRotation, bool bFlip);

		TArray<float> SelectLandmarksToCrop(const TArray<float>& InLandmarks, const TArray<int32>& InLandmarkIndices, const TArray<int32>& LandmarkRangeIdxNormal, const TArray<int32>& InLandmarkIdxRangeExtra, const TArray<int32>& InLandmarkIdxCenter, const TArray<int32>& InLandmarkIdxCenterExtra);

		void AddContourToOutput(const TArray<float>& InPoints, const TMap<FString, Interval>& InCurveMap, const TMap<FString, int32>& InLandmarkMap, FFrameTrackingContourData& OutResult);

		TArray<float> PrepareInput(const TArray<FColor>& InData, int32 InInputSizeX, int32 InInputSizeY);
		float GetRotationToUpright(const TArray<float>& InLandmarks);

		Bbox GetInversedBbox(Bbox& InBbox, const Matrix23f& InTransform);
		TArray<float> GetInversedPoints(float* InLandmarks, int32 InNum, const Matrix23f& InTransform);
		TArray<FColor> WarpAffineBilinear(const FColor* InSrcImage, int32 InSrcWidth, int32 InSrcHeight, const Matrix23f& InTransform, int32 InTargetWidth, int32 InTargetHeight);

		void InitTransformLandmark131to159();
		TArray<float> GetLandmark131to159(const TArray<float>& InLandmarks131);
		TArray<int32> Index131to159;

		TArray<PartPoints> ProcessLandmarks(const FUEImageDataType& Input);

		bool CheckValidity(const TWeakObjectPtr<UNeuralNetwork>& InTracker, NNIModelInfo& ModelInfo);
		bool CheckTrackers(const TMap<ETrackerType, NNIModelInfo>& ValidationMap, const TMap<TWeakObjectPtr<UNeuralNetwork>, ETrackerType>& TrackerTypeMap);

		const int32 DetectorInputSizeX = 300;
		const int32 DetectorInputSizeY = 300;
		const int32 TrackerInputSizeX = 256;
		const int32 TrackerInputSizeY = 256;

		TArray<int32> TrackerPartInputSizeX;
		TArray<int32> TrackerPartInputSizeY;

		TWeakObjectPtr<UNeuralNetwork> FaceTracker{};
		TWeakObjectPtr<UNeuralNetwork> FaceDetector{};

		TWeakObjectPtr<UNeuralNetwork> EyebrowTracker{};
		TWeakObjectPtr<UNeuralNetwork> EyeTracker{};
		TWeakObjectPtr<UNeuralNetwork> LipsTracker{};
		TWeakObjectPtr<UNeuralNetwork> NasolabialTracker{};
		TWeakObjectPtr<UNeuralNetwork> LipsNasolabialTracker{};

		FString ErrorMessage{ "Not initialized." };
		ErrorCode EErrorCode{};

		bool bIsInitialized{};
		bool bIsFaceDetected{};

		TArray<TWeakObjectPtr<UNeuralNetwork>>NNIModels = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

		// To make right part of eye/eyebrow to "left-looking" to make an input for partwise tracker.
		const bool ImageFlipPart[FacePart::Num] = { true, false, true , false, false, false };

		//for separating and merging left/right part
		const bool CombineDataPart[FacePart::Num] = { false, true, false, true, false, false };

		TArray<bool> ProcessPart;

		const float ExtendingRatio = 256.0f / 192.0f;
		const float FaceScoreThreshold = 0.5f;
		const int InvalidMarker = -1;
		bool bIsTrackerSetToGPU = false;
		Matrix23f LastTransform;

		const TMap<FString, Interval> CurveLipMap = { {FString("crv_lip_upper_outer_r"), {24,0}},  {FString("crv_lip_philtrum_r"), {31,24}},
									  {FString("crv_lip_philtrum_l"), {31,38}},    {FString("crv_lip_upper_outer_l"), {38,62}},
									  {FString("crv_lip_lower_outer_l"), {90,62}}, {FString("crv_lip_lower_outer_r"), {90,117,{0}}},//{0} is additional discrete indice
									  {FString("crv_lip_upper_inner_r"), { 142,118,{0} }}, { FString("crv_lip_upper_inner_l"), {142,166,{62}}},
									  {FString("crv_lip_lower_inner_l"), { 191,167,{62}}}, { FString("crv_lip_lower_inner_r"), {191,215,{0}}} };

		const TMap<FString, int32> LandmarkLipMap = { {FString("pt_lip_lower_inner_m"), 191},  {FString("pt_lip_lower_outer_m"), 90},
													  {FString("pt_lip_philtrum_r"), 24},    {FString("pt_lip_philtrum_l"), 38},
													  {FString("pt_lip_upper_inner_m"), 142}, {FString("pt_lip_upper_outer_m"), 31},
													  {FString("pt_mouth_corner_r"), 0}, { FString("pt_mouth_corner_l"), 62} };

		const TMap<FString, Interval> CurveNasolabMap = { { FString("crv_nasolabial_r"), {0,24} }, { FString("crv_nasolabial_l"), {25,49} } };

		const TMap<FString, int32> LandmarkNasolabMap = { {FString("pt_naso_upper_r"), 0},  {FString("pt_naso_lower_r"), 24},
														  {FString("pt_naso_upper_l"), 25},    {FString("pt_naso_lower_l"), 49} };


		const TMap<FString, Interval> CurveEyeIrisMap = { {FString("crv_eyelid_upper_r"), {19,0}},  {FString("crv_eyelid_lower_r"), {19,37, {0} }},
														  {FString("crv_iris_r"), {63,38}}, {FString("crv_eyelid_upper_l"), {83,64}},
														  {FString("crv_eyelid_lower_l"), { 83, 101, {64} }}, { FString("crv_iris_l"), {127,102}} };

		const TMap<FString, int32> LandmarkEyeIrisMap = { {FString("pt_eye_corner_inner_r"), 19},  {FString("pt_eye_corner_inner_l"), 83},
														  {FString("pt_eye_corner_outer_r"), 0}, {FString("pt_eye_corner_outer_l"), 64},
														  {FString("pt_iris_top_r"), 38}, { FString("pt_iris_top_l"), 102} };

		const TMap<FString, Interval> CurveBrowMap = { {FString("crv_brow_upper_r"), {24,0}},  {FString("crv_brow_lower_r"), {24,47,{0}}},
													   {FString("crv_brow_upper_l"), {72,48}}, {FString("crv_brow_lower_l"), {72,95,{48}}} };

		const TMap<FString, int32> LandmarkBrowMap = { {FString("pt_brow_inner_r"), 24}, {FString("pt_brow_inner_l"), 72},
													   {FString("pt_brow_outer_r"), 0},  {FString("pt_brow_outer_l"), 48} };

	};

}
