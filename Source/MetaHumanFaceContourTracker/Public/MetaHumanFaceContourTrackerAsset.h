// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NeuralNetwork.h"

#include "MetaHumanFaceContourTrackerAsset.generated.h"

UCLASS(BlueprintType)
class METAHUMANFACECONTOURTRACKER_API UMetaHumanFaceContourTrackerAsset : public UObject
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere, Category = TrackerModels)
	TSoftObjectPtr<UNeuralNetwork> FaceDetector;

	UPROPERTY(EditAnywhere, Category = TrackerModels)
	TSoftObjectPtr<UNeuralNetwork> FullFaceTracker;

	UPROPERTY(EditAnywhere, Category = TrackerModels)
	TSoftObjectPtr<UNeuralNetwork> BrowsDenseTracker;

	UPROPERTY(EditAnywhere, Category = TrackerModels)
	TSoftObjectPtr<UNeuralNetwork> EyesDenseTracker;

	UPROPERTY(EditAnywhere, Category = TrackerModels)
	TSoftObjectPtr<UNeuralNetwork> NasioLabialsDenseTracker;

	UPROPERTY(EditAnywhere, Category = TrackerModels)
	TSoftObjectPtr<UNeuralNetwork> MouthDenseTracker;

public:

	bool CanProcess() const;

	void LoadTrackers(bool bInShowProgressNotification, TFunction<void(bool)>&& Callback);

	bool LoadTrackersSynchronous();

private:

	UPROPERTY()
	TArray<TObjectPtr<UNeuralNetwork>> LoadedTrackers;
};
