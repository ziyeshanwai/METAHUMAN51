// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanFaceContourTrackerAsset.h"

#include "Engine/AssetManager.h"
#include "Engine/StreamableManager.h"

#include "Framework/Notifications/NotificationManager.h"
#include "Widgets/Notifications/SNotificationList.h"

#define LOCTEXT_NAMESPACE "FaceContourTracker"

bool UMetaHumanFaceContourTrackerAsset::CanProcess() const
{
	// TODO we want to add more validation here that the NNI models have the right number of outputs if possible
	// but needs extra functionality adding to the Pipeline HyprSense node
	return !FaceDetector.IsNull()
		&& !FullFaceTracker.IsNull()
		&& !BrowsDenseTracker.IsNull()
		&& !EyesDenseTracker.IsNull()
		&& !MouthDenseTracker.IsNull()
		&& !NasioLabialsDenseTracker.IsNull();
}

void UMetaHumanFaceContourTrackerAsset::LoadTrackers(bool bInShowProgressNotification, TFunction<void(bool)>&& Callback)
{
	TArray<FSoftObjectPath> TrackersToLoad;
	FStreamableManager& StreamableManager = UAssetManager::GetStreamableManager();
	TWeakPtr<SNotificationItem> LoadNotification;

	TrackersToLoad.Add(FaceDetector.ToSoftObjectPath());
	TrackersToLoad.Add(FullFaceTracker.ToSoftObjectPath());
	TrackersToLoad.Add(BrowsDenseTracker.ToSoftObjectPath());
	TrackersToLoad.Add(EyesDenseTracker.ToSoftObjectPath());
	TrackersToLoad.Add(MouthDenseTracker.ToSoftObjectPath());
	TrackersToLoad.Add(NasioLabialsDenseTracker.ToSoftObjectPath());

	// Show a progress indicator if requested.
	if (bInShowProgressNotification)
	{
		// Only show if the trackers aren't loaded already (and assume if FaceDetector is loaded they all are).
		if (!FaceDetector.IsValid())
		{
			FNotificationInfo Info(LOCTEXT("LoadTrackersNotification", "Loading trackers..."));
			Info.bFireAndForget = false;
			LoadNotification = FSlateNotificationManager::Get().AddNotification(Info);
			if (LoadNotification.IsValid())
			{
				LoadNotification.Pin()->SetCompletionState(SNotificationItem::CS_Pending);
			}
		}
	}

	StreamableManager.RequestAsyncLoad(TrackersToLoad, [this, LoadNotification, Callback]()
	{
		bool bLoadSucceeded = true;

		for (const TSoftObjectPtr<UNeuralNetwork>& Model : { FaceDetector, FullFaceTracker, BrowsDenseTracker, EyesDenseTracker, MouthDenseTracker, NasioLabialsDenseTracker })
		{
			if (Model.IsValid())
			{
				LoadedTrackers.Add(Model.Get());
			}
			else
			{
				bLoadSucceeded = false;
			}
		}

		if (LoadNotification.IsValid())
		{
			LoadNotification.Pin()->SetCompletionState(SNotificationItem::CS_None);
			LoadNotification.Pin()->ExpireAndFadeout();
		}

		Callback(bLoadSucceeded);
	});
}


bool UMetaHumanFaceContourTrackerAsset::LoadTrackersSynchronous()
{
	TArray<FSoftObjectPath> TrackersToLoad;
	FStreamableManager& StreamableManager = UAssetManager::GetStreamableManager();

	TrackersToLoad.Add(FaceDetector.ToSoftObjectPath());
	TrackersToLoad.Add(FullFaceTracker.ToSoftObjectPath());
	TrackersToLoad.Add(BrowsDenseTracker.ToSoftObjectPath());
	TrackersToLoad.Add(EyesDenseTracker.ToSoftObjectPath());
	TrackersToLoad.Add(MouthDenseTracker.ToSoftObjectPath());
	TrackersToLoad.Add(NasioLabialsDenseTracker.ToSoftObjectPath());

	StreamableManager.RequestSyncLoad(TrackersToLoad);
	bool bLoadSucceeded = true;

	for (const TSoftObjectPtr<UNeuralNetwork>& Model : { FaceDetector, FullFaceTracker, BrowsDenseTracker, EyesDenseTracker, MouthDenseTracker, NasioLabialsDenseTracker })
	{
		if (Model.IsValid())
		{
			LoadedTrackers.Add(Model.Get());
		}
		else
		{
			bLoadSucceeded = false;
		}
	}

	return bLoadSucceeded;
}

#undef LOCTEXT_NAMESPACE
