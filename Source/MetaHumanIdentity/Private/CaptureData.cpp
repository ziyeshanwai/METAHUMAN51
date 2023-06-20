// Copyright Epic Games, Inc. All Rights Reserved.

#include "CaptureData.h"
#include "MetaHumanIdentityPromotedFrames.h"
#include "MetaHumanIdentityLog.h"

#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Rendering/SkeletalMeshModel.h"
#include "Rendering/SkeletalMeshLODModel.h"
#include "StaticMeshAttributes.h"
#include "MeshAttributes.h"
#include "Engine/StaticMesh.h"
#include "Misc/TransactionObjectEvent.h"

/////////////////////////////////////////////////////
// UCaptureData

void UCaptureData::NotifyInternalsChanged()
{
	OnCaptureDataInternalsChangedDelegate.Broadcast();
}

/////////////////////////////////////////////////////
// UMeshCaptureData

USceneComponent* UMeshCaptureData::CreatePreviewComponent(UObject* InOwner) const
{
	USceneComponent* PreviewComponent = nullptr;

	if (TargetMesh != nullptr)
	{
		if (UStaticMesh* StaticMesh = Cast<UStaticMesh>(TargetMesh))
		{
			UStaticMeshComponent* StaticMeshComponent = NewObject<UStaticMeshComponent>(InOwner, NAME_None, RF_Transactional);
			StaticMeshComponent->SetStaticMesh(StaticMesh);

			PreviewComponent = StaticMeshComponent;
		}

		if (USkeletalMesh* SkeletalMesh = Cast<USkeletalMesh>(TargetMesh))
		{
			USkeletalMeshComponent* SkeletalMeshComponent = NewObject<USkeletalMeshComponent>(InOwner, NAME_None, RF_Transactional);
			SkeletalMeshComponent->SetSkeletalMesh(SkeletalMesh);

			PreviewComponent = SkeletalMeshComponent;
		}

		if (PreviewComponent != nullptr)
		{
			PreviewComponent->SetMobility(EComponentMobility::Movable);
		}
	}

	return PreviewComponent;
}

TSubclassOf<UMetaHumanIdentityPromotedFrame> UMeshCaptureData::GetPromotedFrameClass() const
{
	return UMetaHumanIdentityCameraFrame::StaticClass();
}

bool UMeshCaptureData::IsInitialized() const
{
	return TargetMesh != nullptr && (TargetMesh->IsA<UStaticMesh>() || TargetMesh->IsA<USkeletalMesh>());
}

bool UMeshCaptureData::IsEditorOnly() const
{
	return true;
}

void UMeshCaptureData::PostEditChangeProperty(FPropertyChangedEvent& InPropertyChangedEvent)
{
	Super::PostEditChangeProperty(InPropertyChangedEvent);

	if (FProperty* Property = InPropertyChangedEvent.Property)
	{
		const FName PropertyName = *Property->GetName();

		if (PropertyName == GET_MEMBER_NAME_CHECKED(ThisClass, TargetMesh))
		{
			NotifyInternalsChanged();
		}
	}
}

void UMeshCaptureData::PostTransacted(const FTransactionObjectEvent& InTransactionEvent)
{
	Super::PostTransacted(InTransactionEvent);

	if (InTransactionEvent.GetEventType() == ETransactionObjectEventType::UndoRedo)
	{
		if (InTransactionEvent.GetChangedProperties().Contains(GET_MEMBER_NAME_CHECKED(ThisClass, TargetMesh)))
		{
			NotifyInternalsChanged();
		}
	}
}

void UMeshCaptureData::GetDataForConforming(const FTransform& InTransform, TArray<float>& OutVertices, TArray<int32>& OutTriangles) const
{
	if (USkeletalMesh* TargetSkeletalMesh = Cast<USkeletalMesh>(TargetMesh))
	{
		FSkeletalMeshModel* ImportedModel = TargetSkeletalMesh->GetImportedModel();
		FSkeletalMeshLODModel& LODModel = ImportedModel->LODModels[0];

		OutVertices.Reset(LODModel.NumVertices * 3);

		for (FSkelMeshSection& Section : LODModel.Sections)
		{
			const int32 NumVertices = Section.GetNumVertices();
			for (int32 VertexIndex = 0; VertexIndex < NumVertices; ++VertexIndex)
			{
				FSoftSkinVertex& OriginalVertex = Section.SoftVertices[VertexIndex];
				FVector TransformedVertex = InTransform.TransformPosition(FVector{ OriginalVertex.Position });
				OutVertices.Add(TransformedVertex.Y);
				OutVertices.Add(-TransformedVertex.Z);
				OutVertices.Add(TransformedVertex.X);
			}
		}

		OutTriangles.Reset(LODModel.IndexBuffer.Num());
		for (uint32 Index : LODModel.IndexBuffer)
		{
			OutTriangles.Add(Index);
		}
	}
	else if (UStaticMesh* TargetStaticMesh = Cast<UStaticMesh>(TargetMesh))
	{
		FMeshDescription* MeshDescription = TargetStaticMesh->GetMeshDescription(0);
		check(MeshDescription);
		FStaticMeshAttributes Attributes(*MeshDescription);

		TVertexAttributesRef<FVector3f> OriginalMeshVerts = Attributes.GetVertexPositions();
		TTriangleAttributesRef<TArrayView<FVertexID>>  OriginalMeshIndices = Attributes.GetTriangleVertexIndices();

		OutVertices.Reset(OriginalMeshVerts.GetNumElements());

		for (int32 RenderCtr = 0; RenderCtr < OriginalMeshVerts.GetNumElements(); ++RenderCtr)
		{
			// map the mesh vertices (in UE coordinate system) to OpenCV coordinate system
			FVector3f OriginalVertex = OriginalMeshVerts.Get(RenderCtr);
			FVector TransformedVertex = InTransform.TransformPosition(FVector{ OriginalVertex });
			OutVertices.Add(TransformedVertex.Y);
			OutVertices.Add(-TransformedVertex.Z);
			OutVertices.Add(TransformedVertex.X);
		}

		OutTriangles.Reset(OriginalMeshIndices.GetNumElements());
		const TArrayView<FVertexID>& RawIndArray = OriginalMeshIndices.GetRawArray();
		for (const auto& Index : RawIndArray)
		{
			OutTriangles.Add(Index.GetValue());
		}
	}
	else
	{
		// This is an error state so log it accordingly
		if (TargetMesh != nullptr)
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("Failed to get data for conforming as TargetMesh is a '%s' but should be a UStaticMesh or USkeletalMesh"), *TargetMesh->GetClass()->GetName());
		}
		else
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("Failed to get data for conforming as TargetMesh is invalid"));
		}
	}
}

/////////////////////////////////////////////////////
// UFootageCaptureData

USceneComponent* UFootageCaptureData::CreatePreviewComponent(UObject*) const
{
	return nullptr;
}

TSubclassOf<UMetaHumanIdentityPromotedFrame> UFootageCaptureData::GetPromotedFrameClass() const
{
	return UMetaHumanIdentityFootageFrame::StaticClass();
}

bool UFootageCaptureData::IsInitialized() const
{
	// TODO: Likely to need more checks to tell if the capture data is valid
	return ImageSequence != nullptr;
}

bool UFootageCaptureData::IsEditorOnly() const
{
	return true;
}

void UFootageCaptureData::PostEditChangeProperty(FPropertyChangedEvent& InPropertyChangedEvent)
{
	Super::PostEditChangeProperty(InPropertyChangedEvent);

	if (FProperty* Property = InPropertyChangedEvent.Property)
	{
		const FName PropertyName = *Property->GetName();

		if (PropertyName == GET_MEMBER_NAME_CHECKED(ThisClass, ImageSequence))
		{
			NotifyInternalsChanged();
		}
	}
}

void UFootageCaptureData::PostTransacted(const FTransactionObjectEvent& InTransactionEvent)
{
	Super::PostTransacted(InTransactionEvent);

	if (InTransactionEvent.GetEventType() == ETransactionObjectEventType::UndoRedo)
	{
		if (InTransactionEvent.GetChangedProperties().Contains(GET_MEMBER_NAME_CHECKED(ThisClass, ImageSequence)))
		{
			NotifyInternalsChanged();
		}
	}
}
