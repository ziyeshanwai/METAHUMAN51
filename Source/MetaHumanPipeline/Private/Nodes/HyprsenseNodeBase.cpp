// Copyright Epic Games, Inc. All Rights Reserved.

#include "Nodes/HyprsenseNodeBase.h"

#include "NeuralNetwork.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/KismetMathLibrary.h"

#define LOCTEXT_NAMESPACE "MetaHuman"

namespace UE::MetaHuman::Pipeline
{
	FHyprsenseNodeBase::FHyprsenseNodeBase(const FString& InTypeName, const FString& InName) : FNode(InTypeName, InName)
	{
	}
	
	void FHyprsenseNodeBase::AddContourToOutput(const TArray<float>& InPoints, const TMap<FString, Interval>& InCurveMap, const TMap<FString, int32>& InLandmarkMap, FFrameTrackingContourData& OutResult)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::AddContourToOutput);

		if (!InPoints.IsEmpty())
		{
			for (const TPair<FString, Interval>& Curve : InCurveMap)
			{
				int32 Size = FMath::Abs(Curve.Value.End - Curve.Value.Start) + 1 + Curve.Value.AdditionalIndices.Num();
				FTrackingContour Contour;
				Contour.DensePoints.SetNum(Size);
				int32 Step = Curve.Value.End > Curve.Value.Start ? 1 : -1;

				int32 K{};
				for (int32 I = Curve.Value.Start; Step > 0 ? I <= Curve.Value.End : I >= Curve.Value.End; I += Step, ++K)
				{
					Contour.DensePoints[K].X = InPoints[2 * I];
					Contour.DensePoints[K].Y = InPoints[2 * I + 1];
				}
				for (int32 I = 0; I < Curve.Value.AdditionalIndices.Num(); ++I, ++K)
				{
					const int32 Index = Curve.Value.AdditionalIndices[I];
					Contour.DensePoints[K].X = InPoints[2 * Index];
					Contour.DensePoints[K].Y = InPoints[2 * Index + 1];
				}
				OutResult.TrackingContours.Add(Curve.Key, Contour);
			}

			for (const TPair<FString, int32>& Landmark : InLandmarkMap)
			{
				FTrackingContour Contour;
				Contour.DensePoints.SetNum(1);

				Contour.DensePoints[0].X = InPoints[2 * Landmark.Value];
				Contour.DensePoints[0].Y = InPoints[2 * Landmark.Value + 1];

				OutResult.TrackingContours.Add(Landmark.Key, Contour);
			}
		}
	}

	bool FHyprsenseNodeBase::CheckValidity(const TWeakObjectPtr<UNeuralNetwork>& InTracker, NNIModelInfo& ModelInfo)
	{
		if (InTracker->GetInputTensorNumber() != ModelInfo.Inputs.Num())
		{
			return false;
		}

		if (InTracker->GetOutputTensorNumber() != ModelInfo.Outputs.Num())
		{
			return false;
		}

		for (int32 I = 0; I < ModelInfo.Inputs.Num(); ++I)
		{
			if (InTracker->GetInputTensor(I).Num() != ModelInfo.Inputs[I])
			{
				return false;
			}
		}

		for (int32 I = 0; I < ModelInfo.Outputs.Num(); ++I)
		{
			if (InTracker->GetOutputTensor(I).Num() != ModelInfo.Outputs[I])
			{
				return false;
			}
		}
		return true;
	}

	bool FHyprsenseNodeBase::CheckTrackers(const TMap<ETrackerType, NNIModelInfo>& ValidationMap, const TMap<TWeakObjectPtr<UNeuralNetwork>, ETrackerType>& TrackerTypeMap)
	{
		bIsInitialized = false;
		for (auto tuple : TrackerTypeMap)
		{
			TWeakObjectPtr<UNeuralNetwork>& Model = tuple.Key;
			ETrackerType TrackerType = tuple.Value;
			FString ModelName = Model->GetFName().ToString();
			NNIModelInfo ModelInfo = ValidationMap[TrackerType];

			if (!Model.IsValid())
			{
				EErrorCode = ErrorCode::InvalidTracker;
				ErrorMessage = FString::Printf(TEXT("%s Tracker is null."), *ModelName);
				return false;
			}

			if (!Model->IsLoaded())
			{
				EErrorCode = ErrorCode::ModelNotLoaded;
				ErrorMessage = FString::Printf(TEXT("%s Model not loaded."), *ModelName);
				return false;
			}

			if (!CheckValidity(Model, ModelInfo))
			{
				return false;
			}

			if (Model.IsValid())
			{
				// Asks the model first if GPU is supposed or SetDeviceType will show a dialog for each model
				// warning the user that GPU is not supported
				if (Model->IsGPUSupported())
				{
					Model->SetDeviceType(ENeuralDeviceType::GPU);
				}
			}
		}
		bIsInitialized = true;
		return true;
	}

	TArray<FHyprsenseNodeBase::PartPoints> FHyprsenseNodeBase::ProcessLandmarks(const FUEImageDataType& Input)
	{
		bool bIsFaceTracked = true;
		TArray<FColor> ResizedData;
		TArray<float> DetectorInputArray;

		FColor* OrigImage = static_cast<FColor*>((void*)Input.Data.GetData());

		// if face is not detected in previous frame, we need to run face detector to find the face box 
		if (!bIsFaceDetected)
		{
			Bbox FullBox = { 0, 0, 1.f, 1.f };
			Matrix23f  iTransform = GetTransformFromBbox(FullBox, Input.Width, Input.Height, DetectorInputSizeX, 1.f, 0.0f, false);

			// resize image for detector input size
			ResizedData = WarpAffineBilinear(OrigImage, Input.Width, Input.Height, iTransform, DetectorInputSizeX, DetectorInputSizeY);

			const float ImageMean = 127.0f;
			const float ImageStd = 128.0f;
			DetectorInputArray.Init(0.f, DetectorInputSizeX * DetectorInputSizeY * 3);

			for (int32 Y = 0; Y < DetectorInputSizeY; ++Y)
			{
				for (int32 X = 0; X < DetectorInputSizeX; ++X)
				{
					int32 Idx = Y * DetectorInputSizeY + X;
					FColor Pixel = ResizedData[Idx];

					DetectorInputArray[Idx] = float(Pixel.R - ImageMean) / ImageStd;
					DetectorInputArray[DetectorInputSizeX * DetectorInputSizeY + Idx] = float(Pixel.G - ImageMean) / ImageStd;
					DetectorInputArray[DetectorInputSizeX * DetectorInputSizeY * 2 + Idx] = float(Pixel.B - ImageMean) / ImageStd;
				}
			}

			// Output
			TArray<float> Scores{}, Boxes{};

			if (FaceDetector.IsValid())
			{
				FaceDetector->SetInputFromArrayCopy(DetectorInputArray);
				FaceDetector->Run();
				Scores = FaceDetector->GetOutputTensor(0).GetArrayCopy<float>();
				Boxes = FaceDetector->GetOutputTensor(1).GetArrayCopy<float>();
			}
			else
			{
				return TArray<PartPoints>();
			}

			const int32 OutSize = 3000;
			const float IouThreshold = 0.45f;
			const float ProbThreshold = 0.275f;
			const int32 TopK = 20;

			// calculate the most accurate face by score
			TArray<Bbox> ResultBoxes = HardNMS(Scores, Boxes, IouThreshold, ProbThreshold, OutSize, TopK);
			if (ResultBoxes.IsEmpty())
			{
				bIsFaceTracked = false;
				bIsFaceDetected = false;
			}
			else
			{
				// face detected
				LastTransform = GetTransformFromBbox(ResultBoxes[0], Input.Width, Input.Height, TrackerInputSizeX, ExtendingRatio, 0.0f, false);
				bIsFaceDetected = true;
			}
		}

		TArray<float> OutputArrayInversed;
		TArray<PartPoints> OutputArrayPartInversed;
		TArray<PartPoints> OutputArrayPerModelInversed;
		OutputArrayInversed.SetNum(0);
		OutputArrayPartInversed.SetNum(FacePart::Num);
		OutputArrayPerModelInversed.SetNum(FacePart::Num);

		// detector found a face already or face is still tracked from last frame
		if (bIsFaceTracked)
		{
			// resize original image to face tracker input size
			ResizedData = WarpAffineBilinear(OrigImage, Input.Width, Input.Height, LastTransform, TrackerInputSizeX, TrackerInputSizeY);

			// prepare input for NNI input format
			TArray<float> ResizedNNInput = PrepareInput(ResizedData, TrackerInputSizeX, TrackerInputSizeY);
			TArray<float> OutputArrayCropped{}, Score{};

			if (FaceTracker.IsValid())
			{
				TRACE_CPUPROFILER_EVENT_SCOPE(TEXT("FHyprsenseNodeBase::FaceTracker_Run"));

				FaceTracker->SetInputFromArrayCopy(ResizedNNInput);
				// Run
				FaceTracker->Run();

				OutputArrayCropped = FaceTracker->GetOutputTensor(0).GetArrayCopy<float>();
				Score = FaceTracker->GetOutputTensor(1).GetArrayCopy<float>();
			}
			else
			{
				return TArray<PartPoints>();
			}

			// if there's no output from face tracker or face tracking score is too low, it means face is lost 
			if (OutputArrayCropped.IsEmpty() || Score[0] < FaceScoreThreshold)
			{
				bIsFaceDetected = false;
			}
			else
			{
				// make 159 number of landmarks from 131 number of landmarks with interpolation
				OutputArrayCropped = GetLandmark131to159(OutputArrayCropped);

				// make inverse transform from cropped image(for face tracker input) coordinate to full/original image coordinate
				OutputArrayInversed = GetInversedPoints(OutputArrayCropped.GetData(), OutputArrayCropped.Num(), LastTransform);

				float Rotation = GetRotationToUpright(OutputArrayInversed);
				// save transform (to crop the face for next frame) based on current landmarks 
				LastTransform = GetTransformFromLandmarkPart(Input.Width, Input.Height, TrackerInputSizeX, ExtendingRatio, OutputArrayInversed, Rotation, false);

				//TArray<float> LandmarkPart[2];
				TArray<float> OutputArrayPartTemp;
				TArray<float> NNInputTemp;
				NNInputTemp.SetNum(0);

				Matrix23f PrevTransform;

				for (int32 I = 0; I < FacePart::Num; ++I)
				{
					TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseRealtimeNode::FacePart_Run);
					if (ProcessPart[I])
					{
						const int32 InputX = TrackerPartInputSizeX[I];
						const int32 InputY = TrackerPartInputSizeY[I];

						float ratio = 1.0f;

						// get bounding box from landmarks and get transform out of the bounding box with flip(for right eye+iris, eyebrow to make it "left-looking"
						Matrix23f TransformPart = GetTransformFromLandmarkFacePart(Input.Width, Input.Height, InputX, ratio, OutputArrayInversed, static_cast<FacePart>(I), Rotation, ImageFlipPart[I]);

						// crop image and prepare input to inject to each partwise tracker
						ResizedData = WarpAffineBilinear(OrigImage, Input.Width, Input.Height, TransformPart, InputX, InputY);
						ResizedNNInput = PrepareInput(ResizedData, InputX, InputY);

						// attach each side of the eye+iris or eyebrow (if it's lips and Nasolabial part, it doesn't attach) 
						NNInputTemp += ResizedNNInput;

						if (NNIModels[I] != nullptr)
						{
							if (NNIModels[I].IsValid())
							{
								TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseRealtimeNode::NNIModel_Run);

								NNIModels[I]->SetInputFromArrayCopy(NNInputTemp);
								NNIModels[I]->Run();
								OutputArrayPartTemp = NNIModels[I]->GetOutputTensor(0).GetArrayCopy<float>();
							}
							else
							{
								return TArray<PartPoints>();
							}

							// size and offset will be differnt depends on the part (eye+iris, eyebrow != lips, Nasolabial)
							int32 Size = OutputArrayPartTemp.Num();
							Size = (CombineDataPart[I]) ? Size / 2 : Size;
							int32 Offset = (CombineDataPart[I]) ? Size : 0;

							// for eye+iris or eyebrow part, output will be attached to hard-wire for refinement tracker (Right -> Left order)
							if (CombineDataPart[I])
							{
								OutputArrayPartInversed[I - 1].Points = GetInversedPoints(OutputArrayPartTemp.GetData(), Size, PrevTransform);
								OutputArrayPerModelInversed[I].Points = OutputArrayPartInversed[I - 1].Points;
							}

							OutputArrayPartInversed[I - 0].Points = GetInversedPoints(OutputArrayPartTemp.GetData() + Offset, Size, TransformPart);
							OutputArrayPerModelInversed[I].Points += OutputArrayPartInversed[I - 0].Points;

							// to reset partwise tracker input
							NNInputTemp.SetNum(0);
						}

						// to keep previous transform to merge left/right of eye+iris and eyebrow part
						PrevTransform = TransformPart;
					}
				}

			}
		}
		return OutputArrayPerModelInversed;
	}
	
	FString FHyprsenseNodeBase::GetErrorMessage() const
	{
		return ErrorMessage;
	}

	FHyprsenseNodeBase::ErrorCode FHyprsenseNodeBase::GetErrorCode() const
	{
		return EErrorCode;
	}

	void FHyprsenseNodeBase::InitTransformLandmark131to159()
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::InitTransformLandmark131to159);

		Index131to159.SetNum(159);

		const TArray<int32> Target = { 18, 20, 22, 24, 27, 29, 31, 33, 35, 37, 39, 41, 44, 46, 48, 50, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93 , -1 };
		int32 TargetIdx = 0;
		int32 LandmarkIdx = 0;
		int32 Idx = 0;

		while (Idx < 159)
		{
			if (Target[TargetIdx] == Idx)
			{
				Index131to159[Idx++] = InvalidMarker;
				Index131to159[Idx++] = LandmarkIdx;
				TargetIdx++;
			}
			else
			{
				Index131to159[Idx++] = LandmarkIdx;
			}
			LandmarkIdx++;
		}
	}

	TArray<float> FHyprsenseNodeBase::GetLandmark131to159(const TArray<float>& InLandmarks131)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::GetLandmark131to159);

		const TArray<int32> Source = { 17, 18,  18, 19,  19, 20,  20, 21,  22, 23,  23, 24,  24, 25,  25, 17,
										26, 27,  27, 28,  28, 29,  29, 30,  31, 32,  32, 33,  33, 34,  34, 26,
										54, 55,  55, 56,  56, 57,  57, 58,  58, 59,  59, 54,
										60, 61,  61, 62,  62, 63,  63, 64,  64, 65,  65, 60 };
		int32 SourceIdx = 0;

		TArray<float> Landmarks159;
		Landmarks159.SetNum(159 * 2);

		for (int32 I = 0; I < 159; ++I)
		{
			if (Index131to159[I] == InvalidMarker)
			{
				Landmarks159[I * 2 + 0] = (InLandmarks131[Source[SourceIdx * 2] * 2 + 0] + InLandmarks131[Source[SourceIdx * 2 + 1] * 2 + 0]) / 2.0f;
				Landmarks159[I * 2 + 1] = (InLandmarks131[Source[SourceIdx * 2] * 2 + 1] + InLandmarks131[Source[SourceIdx * 2 + 1] * 2 + 1]) / 2.0f;
				SourceIdx++;
			}
			else
			{
				Landmarks159[I * 2 + 0] = InLandmarks131[Index131to159[I] * 2 + 0];
				Landmarks159[I * 2 + 1] = InLandmarks131[Index131to159[I] * 2 + 1];
			}
		}
		return Landmarks159;
	}

	TArray<float> FHyprsenseNodeBase::SelectLandmarksToCrop(const TArray<float>& InLandmarks, const TArray<int32>& InLandmarkIndices, const TArray<int32>& LandmarkRangeIdxNormal, const TArray<int32>& InLandmarkIdxRangeExtra, const TArray<int32>& InLandmarkIdxCenter, const TArray<int32>& InLandmarkIdxCenterExtra)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::SelectLandmarksToCrop);

		TArray<float> LandmarkSelections;

		for (int32 I = 0; I < InLandmarkIndices.Num(); ++I)
		{
			LandmarkSelections.Add(InLandmarks[InLandmarkIndices[I] * 2 + 0]);
			LandmarkSelections.Add(InLandmarks[InLandmarkIndices[I] * 2 + 1]);
		}

		if (LandmarkRangeIdxNormal.Num() == 2)
		{
			for (int32 I = LandmarkRangeIdxNormal[0]; I < LandmarkRangeIdxNormal[1]; ++I)
			{
				LandmarkSelections.Add(InLandmarks[I * 2 + 0]);
				LandmarkSelections.Add(InLandmarks[I * 2 + 1]);
			}
		}

		if (InLandmarkIdxRangeExtra.Num() == 3)
		{
			for (int32 I = InLandmarkIdxRangeExtra[0]; I < InLandmarkIdxRangeExtra[1]; ++I)
			{
				LandmarkSelections.Add(3 * InLandmarks[I * 2 + 0] - 2 * InLandmarks[(InLandmarkIdxRangeExtra[2] - I) * 2 + 0]);
				LandmarkSelections.Add(3 * InLandmarks[I * 2 + 1] - 2 * InLandmarks[(InLandmarkIdxRangeExtra[2] - I) * 2 + 1]);
			}
		}

		if (InLandmarkIdxCenter.Num() == 4)
		{
			LandmarkSelections.Add(0.5f * (InLandmarks[InLandmarkIdxCenter[0] * 2 + 0] + InLandmarks[InLandmarkIdxCenter[1] * 2 + 0]));
			LandmarkSelections.Add(0.5f * (InLandmarks[InLandmarkIdxCenter[0] * 2 + 1] + InLandmarks[InLandmarkIdxCenter[1] * 2 + 1]));
			LandmarkSelections.Add(2.0f * (InLandmarks[InLandmarkIdxCenter[2] * 2 + 0]) - InLandmarks[InLandmarkIdxCenter[3] * 2 + 0]);
			LandmarkSelections.Add(2.0f * (InLandmarks[InLandmarkIdxCenter[2] * 2 + 1]) - InLandmarks[InLandmarkIdxCenter[3] * 2 + 1]);
		}

		if (InLandmarkIdxCenterExtra.Num() > 0)
		{
			for (int32 I = 0; I < InLandmarkIdxCenterExtra.Num(); I += 2)
			{
				LandmarkSelections.Add(2 * InLandmarks[InLandmarkIdxCenterExtra[I] * 2 + 0] - InLandmarks[InLandmarkIdxCenterExtra[I + 1] * 2 + 0]);
				LandmarkSelections.Add(2 * InLandmarks[InLandmarkIdxCenterExtra[I] * 2 + 1] - InLandmarks[InLandmarkIdxCenterExtra[I + 1] * 2 + 1]);
			}
		}
		return LandmarkSelections;
	}

	FHyprsenseNodeBase::Matrix23f FHyprsenseNodeBase::GetTransformFromLandmarkFacePart(int32 InImageWidth, int32 InImageHeight, int32 InCropBoxSize, float InRatio, const TArray<float>& InLandmarks, FacePart InPartName, float InRotation, bool bFlip)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::GetTransformFromLandmarkFacePart);

		TArray<int32> LandmarkIndices = {};
		TArray<int32> LandmarkIdxRangeNormal = {};
		TArray<int32> LandmarkIdxRangeExtra = {};
		TArray<int32> LandmarkIdxCenter = {};
		TArray<int32> LandmarkIdxCenterExtra = {};

		TArray<float> LandmarkSelections = {};

		if (InPartName == FacePart::LipsNasolabial)
		{
			LandmarkIndices = { 12, 53 };
			LandmarkIdxRangeNormal = { 4, 13 };
			LandmarkIdxCenterExtra = { 154, 94, 158, 106 };
		}
		else if(InPartName == FacePart::Lips)
		{
			LandmarkIndices = { 154, 62, 158, 8 };
		}
		else if (InPartName == FacePart::Nasolabial)
		{
			LandmarkIndices = { 4, 8, 12, 53, 57, 67 };
		}
		else if (InPartName == FacePart::LeftEye)
		{
			LandmarkIndices = { 51, 52 };
			LandmarkIdxRangeNormal = { 34, 43 };
		}
		else if (InPartName == FacePart::LeftEyeBrow)
		{
			LandmarkIndices = { 51, 52 };
			LandmarkIdxRangeExtra = { 35, 43, 85 };
			LandmarkIdxCenter = { 34, 16, 34, 82 };
		}
		else if (InPartName == FacePart::RightEye)
		{
			LandmarkIndices = { 51, 52 };
			LandmarkIdxRangeNormal = { 17, 26 };
		}
		else if (InPartName == FacePart::RightEyeBrow)
		{
			LandmarkIndices = { 51, 52 };
			LandmarkIdxRangeExtra = { 18, 26, 51 };
			LandmarkIdxCenter = { 17, 0, 17, 70 };
		}

		LandmarkSelections = SelectLandmarksToCrop(InLandmarks, LandmarkIndices, LandmarkIdxRangeNormal, LandmarkIdxRangeExtra, LandmarkIdxCenter, LandmarkIdxCenterExtra);

		return GetTransformFromLandmarkPart(InImageWidth, InImageHeight, InCropBoxSize, InRatio, LandmarkSelections, InRotation, bFlip);
	}

	FHyprsenseNodeBase::Matrix23f FHyprsenseNodeBase::GetTransformFromBbox(const Bbox& InBbox, int32 InImageWidth, int32 InImageHeight, int32 InCropBoxSize, float InRatio, float InRotation, bool bFlip)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::GetTransformFromBbox);

		const int32 X = InBbox.X1 * InImageWidth;
		const int32 Y = InBbox.Y1 * InImageHeight;
		const int32 W = (InBbox.X2 - InBbox.X1) * InImageWidth;
		const int32 H = (InBbox.Y2 - InBbox.Y1) * InImageHeight;

		const float Cx = X + 0.5 * W;
		const float Cy = Y + 0.5 * H;
		const float Size = InRatio * (W > H ? W : H);

		// affine a face box on an image to an input frame
		Matrix33f TransformFlip;
		{
			if (bFlip)
			{
				TransformFlip << -1, 0, InCropBoxSize,
					0, 1, 0,
					0, 0, 1;
			}
			else
			{
				TransformFlip << 1, 0, 0,
					0, 1, 0,
					0, 0, 1;
			}
		}

		// affine a face box on an image to an input frame
		Matrix23f TransformSrcToDst;

		// warning: do not cast center positions into INT type. It will cause shaking input_image_data and jitters in the tracking.

		TransformSrcToDst << std::cos(InRotation), -std::sin(InRotation), Cx, std::sin(InRotation),
			std::cos(InRotation), Cy;

		Matrix23f SrcFrame;
		{
			Eigen::Matrix3Xf SrcFrameOrig(3, 3);

			const float CroppedHalfWidth = (0.5) * Size;
			SrcFrameOrig << -CroppedHalfWidth, CroppedHalfWidth, CroppedHalfWidth, -CroppedHalfWidth,
				-CroppedHalfWidth, CroppedHalfWidth, 1.f, 1.f, 1.f;

			SrcFrame = TransformSrcToDst * SrcFrameOrig;
		}

		Matrix23f DstFrame;
		DstFrame << 0, InCropBoxSize, InCropBoxSize, 0, 0, InCropBoxSize;

		Eigen::Matrix3Xf Src3(3, DstFrame.cols());
		Src3.topRows(2) = DstFrame;
		Src3.row(2).setOnes();

		const Eigen::MatrixX3f Src3T = Src3.transpose(); // x3f
		const Matrix23f Transform = SrcFrame * Src3T * (Src3 * Src3T).inverse() * TransformFlip;

		return Transform;
	}

	float FHyprsenseNodeBase::GetRotationToUpright(const TArray<float>& InLandmarks)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::GetRotationToUpright);

		const int32 REyesIdx = 70;
		const int32 LEyesIdx = 82;

		const int32 X1 = InLandmarks[REyesIdx * 2];
		const int32 Y1 = InLandmarks[REyesIdx * 2 + 1];
		const int32 X2 = InLandmarks[LEyesIdx * 2];
		const int32 Y2 = InLandmarks[LEyesIdx * 2 + 1];

		return UKismetMathLibrary::Atan2(Y2 - Y1, X2 - X1);
	}

	FHyprsenseNodeBase::Matrix23f FHyprsenseNodeBase::GetTransformFromLandmarkPart(int32 InImageWidth, int32 InImageHeight, int32 InCropBoxSize, float InRatio, const TArray<float>& InLandmarks, float InRotation, bool bFlip)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::GetTransformFromLandmarkPart);

		float MinX = InImageWidth, MinY = InImageHeight;
		float MaxX = 0, MaxY = 0;

		const int32 LandmarkSize = InLandmarks.Num();
		for (int32 I = 0; I < LandmarkSize; I += 2)
		{
			MinX = (MinX > InLandmarks[I + 0]) ? InLandmarks[I + 0] : MinX;
			MaxX = (MaxX < InLandmarks[I + 0]) ? InLandmarks[I + 0] : MaxX;
			MinY = (MinY > InLandmarks[I + 1]) ? InLandmarks[I + 1] : MinY;
			MaxY = (MaxY < InLandmarks[I + 1]) ? InLandmarks[I + 1] : MaxY;
		}

		Bbox LandmarkBox;
		LandmarkBox.X1 = MinX / InImageWidth;
		LandmarkBox.X2 = MaxX / InImageWidth;
		LandmarkBox.Y1 = MinY / InImageHeight;
		LandmarkBox.Y2 = MaxY / InImageHeight;

		return GetTransformFromBbox(LandmarkBox, InImageWidth, InImageHeight, InCropBoxSize, InRatio, InRotation, bFlip);
	}

	float FHyprsenseNodeBase::IOU(const Bbox& InBox1, const Bbox& InBox2)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::IOU);

		const float MaxX = (InBox1.X1 > InBox2.X1) ? InBox1.X1 : InBox2.X1;
		const float MaxY = (InBox1.Y1 > InBox2.Y1) ? InBox1.Y1 : InBox2.Y1;
		const float MinX = (InBox1.X2 < InBox2.X2) ? InBox1.X2 : InBox2.X2;
		const float MinY = (InBox1.Y2 < InBox2.Y2) ? InBox1.Y2 : InBox2.Y2;

		const float Width = ((MinX - MaxX + 0.01f) > 0) ? (MinX - MaxX + 0.01f) : 0.0f;
		const float Height = ((MinY - MaxY + 0.01f) > 0) ? (MinY - MaxY + 0.01f) : 0.0f;
		const float Overlap = Width * Height;

		return Overlap / (InBox1.Area + InBox2.Area - Overlap);
	}

	TArray<FHyprsenseNodeBase::Bbox> FHyprsenseNodeBase::HardNMS(const TArray<float>& InScores, const TArray<float>& InBoxes, float InIouThreshold, float InProbThreshold, int32 InTotalSize, int32 InTopK)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::HardNMS);

		TArray<Bbox> FilteredBoxes;

		for (int32 I = 0; I < InTotalSize; ++I)
		{
			int32 ScoreIdx = I * 2 + 1;
			int32 BoxIdx = I * 4;
			float Score = InScores[ScoreIdx];
			if (Score > InProbThreshold)
			{
				Bbox Box;
				Box.Score = Score;
				Box.X1 = InBoxes[BoxIdx];
				Box.Y1 = InBoxes[BoxIdx + 1];
				Box.X2 = InBoxes[BoxIdx + 2];
				Box.Y2 = InBoxes[BoxIdx + 3];
				Box.Area = (Box.X2 - Box.X1) * (Box.Y2 - Box.Y1);
				FilteredBoxes.Add(Box);
			}
		}
		FilteredBoxes.Sort([](const Bbox& a, const Bbox& b) { return a.Score > b.Score; });

		for (auto Iter(FilteredBoxes.CreateIterator()); Iter; ++Iter)
		{
			for (auto Iter2 = Iter + 1; Iter2; ++Iter2)
			{
				float Iou = IOU(*Iter, *Iter2);
				if (Iou > InIouThreshold)
				{
					Iter2.RemoveCurrent();
				}
			}
		}

		return FilteredBoxes;
	}

	FHyprsenseNodeBase::Bbox FHyprsenseNodeBase::GetInversedBbox(Bbox& InBbox, const Matrix23f& InTransform)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::GetInversedBbox);

		Eigen::Affine2f Affine(InTransform);

		Eigen::Vector2f CroppedBoxPt1 = Eigen::Vector2f(InBbox.X1, InBbox.Y1);
		Eigen::Vector2f TransformedBoxPt1 = Affine * CroppedBoxPt1;
		InBbox.X1 = TransformedBoxPt1.x();
		InBbox.Y1 = TransformedBoxPt1.y();

		Eigen::Vector2f  CroppedBoxPt2 = Eigen::Vector2f(InBbox.X2, InBbox.Y2);
		Eigen::Vector2f TransformedBoxPt2 = Affine * CroppedBoxPt2;
		InBbox.X2 = TransformedBoxPt2.x();
		InBbox.Y2 = TransformedBoxPt2.y();

		return InBbox;
	}

	TArray<float> FHyprsenseNodeBase::GetInversedPoints(float* InLandmarks, int32 InNum, const Matrix23f& InTransform)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::GetInversedPoints);

		TArray<float> InversedLandmarks;
		Eigen::Affine2f Affine(InTransform);

		const int32 NumPoints = InNum / 2;
		for (int32 I = 0; I < NumPoints; ++I) {
			const float X = InLandmarks[I * 2 + 0];
			const float Y = InLandmarks[I * 2 + 1];
			Eigen::Vector2f CroppedImgPt = Eigen::Vector2f(X, Y);
			Eigen::Vector2f OrigImgPt = Affine * CroppedImgPt;

			InversedLandmarks.Add(OrigImgPt.x());
			InversedLandmarks.Add(OrigImgPt.y());
		}

		return InversedLandmarks;
	}
	
	TArray<FColor> FHyprsenseNodeBase::WarpAffineBilinear(const FColor* InSrcImage, int32 InSrcWidth, int32 InSrcHeight, const Matrix23f& InTransform, int32 InTargetWidth, int32 InTargetHeight)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::WarpAffineBilinear);

		const uint8* const SourceData = (uint8*)InSrcImage;

		const int32 PixelSize = 4;
		const int32 NumPixels = InTargetWidth * InTargetHeight;
		
		TArray<FColor> Result;
		Result.SetNumZeroed(NumPixels);
		
		for (int32 R = 0; R < InTargetHeight; ++R)
		{
			for (int32 C = 0; C < InTargetWidth; ++C)
			{
				// we assume that pixel color is at the center of the pixel square.
				const Eigen::Vector3f TargetPixel(C + 0.5f, R + 0.5f, 1.0f);
				const Eigen::Vector2f SourcePixel = InTransform * TargetPixel;

				const Eigen::Vector2i SourcePixelFloor = SourcePixel.array().floor().cast<int>();
				const Eigen::Vector2i SourcePixelCeil = SourcePixel.array().ceil().cast<int>();

				const float X = SourcePixel[0];
				const float Y = SourcePixel[1];

				const int32 X1 = SourcePixelFloor[0];
				const int32 X2 = SourcePixelCeil[0];

				const int32 Y1 = SourcePixelFloor[1];
				const int32 Y2 = SourcePixelCeil[1];

				if (0 <= X1 && X2 < InSrcWidth && 0 <= Y1 && Y2 < InSrcHeight)
				{
					const float XWeight1 = X2 - X;
					const float XWeight2 = 1.f - XWeight1;
					const float YWeight1 = Y2 - Y;
					const float YWeight2 = 1.f - YWeight1;
				
					const float Weight11 = XWeight1 * YWeight1;
					const float Weight12 = XWeight2 * YWeight1;
					const float Weight21 = XWeight1 * YWeight2;
					const float Weight22 = XWeight2 * YWeight2;
					
					const uint8* const SrcCursor11 = &SourceData[PixelSize * (X1 + Y1 * InSrcWidth)];
					const uint8* const SrcCursor21 = &SourceData[PixelSize * (X1 + Y2 * InSrcWidth)];
					const uint8* const SrcCursor12 = &SourceData[PixelSize * (X2 + Y1 * InSrcWidth)];
					const uint8* const SrcCursor22 = &SourceData[PixelSize * (X2 + Y2 * InSrcWidth)];
					
					const int32 OutputArrayLinearPosition = R * InTargetWidth + C;
					const uint8 Blue = static_cast<uint8>(Weight11 * static_cast<float>(SrcCursor11[0]) + Weight12 * static_cast<float>(SrcCursor12[0]) + Weight21 * static_cast<float>(SrcCursor21[0]) + Weight22 * static_cast<float>(SrcCursor22[0]));
					Result[OutputArrayLinearPosition].B = Blue;
					const uint8 Green = static_cast<uint8>(Weight11 * static_cast<float>(SrcCursor11[1]) + Weight12 * static_cast<float>(SrcCursor12[1]) + Weight21 * static_cast<float>(SrcCursor21[1]) + Weight22 * static_cast<float>(SrcCursor22[1]));
					Result[OutputArrayLinearPosition].G = Green;
					const uint8 Red = static_cast<uint8>(Weight11 * static_cast<float>(SrcCursor11[2]) + Weight12 * static_cast<float>(SrcCursor12[2]) + Weight21 * static_cast<float>(SrcCursor21[2]) + Weight22 * static_cast<float>(SrcCursor22[2]));
					Result[OutputArrayLinearPosition].R = Red;
					const int8 Alpha = static_cast<uint8>(Weight11 * static_cast<float>(SrcCursor11[3]) + Weight12 * static_cast<float>(SrcCursor12[3]) + Weight21 * static_cast<float>(SrcCursor21[3]) + Weight22 * static_cast<float>(SrcCursor22[3]));
					Result[OutputArrayLinearPosition].A = Alpha;
				}
			}
		}

		return Result;
	}

	TArray<float> FHyprsenseNodeBase::PrepareInput(const TArray<FColor>& InData, int32 InInputSizeX, int32 InInputSizeY)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(FHyprsenseNodeBase::PrepareInput);

		TArray<float> ResizedNNInput;
		ResizedNNInput.Init(0.f, InInputSizeX * InInputSizeY * 3);
		const float Sqrt2 = FMath::Sqrt(2.f);

		for (int32 Y = 0; Y < InInputSizeY; ++Y)
		{
			for (int32 X = 0; X < InInputSizeX; ++X)
			{
				int32 Idx = Y * InInputSizeX + X;
				FColor PixelColor = InData[Idx];

				ResizedNNInput[Idx] = (((PixelColor.R / 255.f) - 0.5) * Sqrt2);
				ResizedNNInput[InInputSizeX * InInputSizeY + Idx] = (((PixelColor.G / 255.f) - 0.5) * Sqrt2);
				ResizedNNInput[InInputSizeX * InInputSizeX * 2 + Idx] = (((PixelColor.B / 255.f) - 0.5) * Sqrt2);
			}
		}
		return ResizedNNInput;
	}

	void FHyprsenseNodeBase::AddReferencedObjects(FReferenceCollector& InCollector)
	{
		for (TWeakObjectPtr<UNeuralNetwork> NeuralNetwork : { FaceTracker, FaceDetector, EyebrowTracker, EyeTracker, 
			LipsTracker, NasolabialTracker, LipsNasolabialTracker })
		{
			if (NeuralNetwork.IsValid())
			{
				UNeuralNetwork* NeuralNetworkPtr = NeuralNetwork.Get();
				InCollector.AddReferencedObject(NeuralNetworkPtr);
			}
		}
	}

	FString FHyprsenseNodeBase::GetReferencerName() const
	{ 
		return "HyprsenseNode"; 
	}
}

#undef LOCTEXT_NAMESPACE
