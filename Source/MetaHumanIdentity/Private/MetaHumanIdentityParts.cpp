// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanIdentityParts.h"
#include "MetaHumanIdentityLog.h"
#include "MetaHumanIdentityPromotedFrames.h"
#include "CaptureData.h"

#include "UI/MetaHumanIdentityStyle.h"

#include "MetaHumanFaceContourTrackerAsset.h"
#include "MetaHumanConformer.h"

#include "DNAReader.h"
#include "DNAAsset.h"
#include "DNAUtils.h"
#include "SkelMeshDNAUtils.h"
#include "DNAToSkelMeshMap.h"

#include "Engine/StaticMesh.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/DynamicMeshComponent.h"
#include "MeshDescriptionToDynamicMesh.h"
#include "UDynamicMesh.h"
#include "DynamicMesh/MeshNormals.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "OpenCVHelper.h"
#include "AssetToolsModule.h"
#include "ProfilingDebugging/ScopedTimers.h"
#include "ScopedTransaction.h"
#include "Editor/Transactor.h"
#include "Misc/ScopedSlowTask.h"
#include "Interfaces/IPluginManager.h"
#include "Factories/FbxStaticMeshImportData.h"
#include "UObject/GCObjectScopeGuard.h"
#include "AssetExportTask.h"
#include "Exporters/StaticMeshExporterOBJ.h"
#include "ObjectTools.h"
#include "Misc/NamePermissionList.h"
#include "Materials/Material.h"

#define LOCTEXT_NAMESPACE "MetaHumanIdentityParts"

namespace
{
	TAutoConsoleVariable<bool> CVarEnableExportMeshes {
		TEXT("mh.Identity.ExportMeshes"),
		false,
		TEXT("Enables exporting MetaHuman Identity meshes as OBJs"),
		ECVF_Default
	};
}

//////////////////////////////////////////////////////////////////////////
// UMetaHumanIdentityPose

FString UMetaHumanIdentityPose::PoseTypeAsString(EIdentityPoseType InPoseType)
{
	return StaticEnum<EIdentityPoseType>()->GetDisplayNameTextByValue(static_cast<int64>(InPoseType)).ToString();
}

UMetaHumanIdentityPose::UMetaHumanIdentityPose()
	: Super{}
	, PoseTransform{ FTransform::Identity }
{
}

FSlateIcon UMetaHumanIdentityPose::GetPoseIcon() const
{
	const FString PoseTypeName = PoseTypeAsString(PoseType);
	const ANSICHAR* PoseTypeNamePtr = StringCast<ANSICHAR>(*PoseTypeName).Get();

	// Compute the name of the Pose icon based on the PoseType enum
	const FName PoseIconName = FAppStyle::Join("Identity.Pose.", PoseTypeNamePtr);

	FMetaHumanIdentityStyle& Style = FMetaHumanIdentityStyle::Get();
	const FName& StyleSetName = Style.GetStyleSetName();

	return FSlateIcon{ StyleSetName, PoseIconName };
}

void UMetaHumanIdentityPose::SetCaptureData(UCaptureData* InCaptureData)
{
	if (CaptureData != InCaptureData)
	{
		CaptureData = InCaptureData;

		RegisterCaptureDataInternalsChangedDelegate();
		HandleCaptureDataChanged();
		NotifyCaptureDataChanged();
	}
}

UCaptureData* UMetaHumanIdentityPose::GetCaptureData() const
{
	return CaptureData;
}

bool UMetaHumanIdentityPose::IsCaptureDataValid() const
{
	return (CaptureData != nullptr) && CaptureData->IsInitialized();
}

bool UMetaHumanIdentityPose::IsDefaultTrackerValid() const
{
	return (DefaultTracker != nullptr) && DefaultTracker->CanProcess();
}

TArray<UMetaHumanIdentityPromotedFrame*> UMetaHumanIdentityPose::GetAllPromotedFramesWithValidContourData() const
{
	TArray<UMetaHumanIdentityPromotedFrame*> ValidPromotedFrames;
	for (UMetaHumanIdentityPromotedFrame* PromotedFrame : PromotedFrames)
	{
		if (PromotedFrame->bUseToSolve && PromotedFrame->FrameContoursContainActiveData())
		{
			ValidPromotedFrames.Add(PromotedFrame);
		}
	}

	return ValidPromotedFrames;
}

const FTransform& UMetaHumanIdentityPose::GetHeadAlignment(int32 InFrameIndex)
{
	check(InFrameIndex < PromotedFrames.Num());
	return PromotedFrames[InFrameIndex]->HeadAlignment;
}

void UMetaHumanIdentityPose::SetHeadAlignment(const FTransform& InTransform, int32 InFrameIndex)
{
	if (InFrameIndex == INDEX_NONE)
	{
		for (UMetaHumanIdentityPromotedFrame* PromotedFrame : PromotedFrames)
		{
			PromotedFrame->HeadAlignment = InTransform;
		}
	}
	else
	{
		if (InFrameIndex < PromotedFrames.Num())
		{
			PromotedFrames[InFrameIndex]->HeadAlignment = InTransform;
		}
	}
}

void UMetaHumanIdentityPose::LoadDefaultTracker()
{
	if (DefaultTracker == nullptr && PoseType == EIdentityPoseType::Neutral)
	{
		static constexpr const TCHAR* GenericTrackerPath = TEXT("MetaHumanFaceContourTrackerAsset'/MetaHuman/GenericTracker/GenericFaceContourTracker.GenericFaceContourTracker'");
		if (UMetaHumanFaceContourTrackerAsset* Tracker = Cast<UMetaHumanFaceContourTrackerAsset>(StaticLoadObject(UMetaHumanFaceContourTrackerAsset::StaticClass(), nullptr, GenericTrackerPath)))
		{
			if (Tracker->CanProcess())
			{
				DefaultTracker = Tracker;
			}
		}
	}
}

bool UMetaHumanIdentityPose::IsEditorOnly() const
{
	return true;
}

void UMetaHumanIdentityPose::PostLoad()
{
	Super::PostLoad();

	RegisterCaptureDataInternalsChangedDelegate();
	RegisterCaptureDataSceneComponentTransformChanged();

	if (CaptureDataSceneComponent != nullptr)
	{
		CaptureDataSceneComponent->SetWorldTransform(PoseTransform);
		CaptureDataSceneComponent->UpdateComponentToWorld();
	}
}

void UMetaHumanIdentityPose::PostEditChangeProperty(struct FPropertyChangedEvent& InPropertyChangedEvent)
{
	Super::PostEditChangeProperty(InPropertyChangedEvent);

	if (FProperty* Property = InPropertyChangedEvent.Property)
	{
		const FName PropertyName = *Property->GetName();

		if (PropertyName == GET_MEMBER_NAME_CHECKED(ThisClass, CaptureData))
		{
			RegisterCaptureDataInternalsChangedDelegate();
			HandleCaptureDataChanged();
			NotifyCaptureDataChanged();
		}
		else if (PropertyName == GET_MEMBER_NAME_CHECKED(ThisClass, PoseTransform))
		{
			NotifyPoseTransformChanged();
		}
	}
}

void UMetaHumanIdentityPose::PostTransacted(const FTransactionObjectEvent& InTransactionEvent)
{
	Super::PostTransacted(InTransactionEvent);

	if (InTransactionEvent.GetEventType() == ETransactionObjectEventType::UndoRedo)
	{
		const FPermissionListOwners& ChangedProperties = InTransactionEvent.GetChangedProperties();

		if (ChangedProperties.Contains(GET_MEMBER_NAME_CHECKED(ThisClass, CaptureData)))
		{
			RegisterCaptureDataInternalsChangedDelegate();
			NotifyCaptureDataChanged();
		}
		else if (ChangedProperties.Contains(GET_MEMBER_NAME_CHECKED(ThisClass, PoseTransform)))
		{
			NotifyPoseTransformChanged();
		}
	}
}

void UMetaHumanIdentityPose::NotifyCaptureDataChanged()
{
	bool bInvalidatePromotedFrames = true;

	if (CaptureData != nullptr)
	{
		TSubclassOf<UMetaHumanIdentityPromotedFrame> NewPromotedFrameClass = CaptureData->GetPromotedFrameClass();
		if (PromotedFrameClass != NewPromotedFrameClass)
		{
			PromotedFrameClass = NewPromotedFrameClass;
		}
		else
		{
			bInvalidatePromotedFrames = false;
		}
	}
	else
	{
		// Clear the Promoted Frame class type to prevent new Promoted Frames from being created without a valid CaptureData present
		PromotedFrameClass = nullptr;
	}

	if (bInvalidatePromotedFrames)
	{
		PromotedFrames.Empty();
	}

	OnCaptureDataChangedDelegate.Broadcast();
}

void UMetaHumanIdentityPose::NotifyPoseTransformChanged()
{
	CaptureDataSceneComponent->SetWorldTransform(PoseTransform);
	CaptureDataSceneComponent->TransformUpdated.Broadcast(CaptureDataSceneComponent, EUpdateTransformFlags::None, ETeleportType::None);
}

void UMetaHumanIdentityPose::HandleCaptureDataChanged()
{
	if (CaptureData != nullptr)
	{
		CaptureDataSceneComponent = CaptureData->CreatePreviewComponent(this);

		RegisterCaptureDataSceneComponentTransformChanged();
	}
}

void UMetaHumanIdentityPose::HandleCaptureDataSceneComponentTransformChanged(USceneComponent* InRootComponent, EUpdateTransformFlags, ETeleportType)
{
	PoseTransform = InRootComponent->GetComponentTransform();
}

void UMetaHumanIdentityPose::RegisterCaptureDataInternalsChangedDelegate()
{
	if (CaptureData != nullptr)
	{
		CaptureData->OnCaptureDataInternalsChanged().AddUObject(this, &UMetaHumanIdentityPose::HandleCaptureDataChanged);
	}
}

void UMetaHumanIdentityPose::RegisterCaptureDataSceneComponentTransformChanged()
{
	if (CaptureDataSceneComponent != nullptr)
	{
		CaptureDataSceneComponent->TransformUpdated.AddUObject(this, &UMetaHumanIdentityPose::HandleCaptureDataSceneComponentTransformChanged);
	}
}


//////////////////////////////////////////////////////////////////////////
// UMetaHumanIdentityFace

UMetaHumanIdentityFace::UMetaHumanIdentityFace()
	: Super{}
	, bIsConformed{ false }
{
	ConformalMeshComponent = CreateDefaultSubobject<UDynamicMeshComponent>(TEXT("Template Mesh"));

	InitializeConformalMesh();
}

FText UMetaHumanIdentityFace::GetPartName() const
{
	return LOCTEXT("IdentityFacePartName", "Face");
}

FText UMetaHumanIdentityFace::GetPartDescription() const
{
	return LOCTEXT("IdentityFacePartDescription", "The Face of the MetaHuman Identity. This creates a new static mesh asset representing the Template Mesh");
}

FSlateIcon UMetaHumanIdentityFace::GetPartIcon(const FName& InPropertyName) const
{
	FMetaHumanIdentityStyle& Style = FMetaHumanIdentityStyle::Get();
	const FName& StyleSetName = Style.GetStyleSetName();

	if (!InPropertyName.IsNone())
	{
		if (InPropertyName == GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityFace, ConformalMeshComponent))
		{
			return FSlateIcon{ StyleSetName, "Identity.Face.ConformalMesh" };
		}
		else if (InPropertyName == GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityFace, RigComponent))
		{
			return FSlateIcon{ StyleSetName, "Identity.Face.Rig" };
		}
	}

	return FSlateIcon{ StyleSetName, "Identity.Face.Part" };
}

void UMetaHumanIdentityFace::PostLoad()
{
	Super::PostLoad();

	if (ConformalMeshComponent != nullptr)
	{
		// Need this to make sure the component's members are initialized to the correct
		// values as they are derived from the actual properties that are serialized
		ConformalMeshComponent->UpdateComponentToWorld();
	}
}

bool UMetaHumanIdentityFace::CanConform() const
{
	if (ConformalMeshComponent != nullptr && ConformalMeshComponent->GetDynamicMesh() != nullptr)
	{
		if (UMetaHumanIdentityPose* NeutralPose = FindPoseByType(EIdentityPoseType::Neutral))
		{
			if (NeutralPose->IsCaptureDataValid())
			{
				const TArray<UMetaHumanIdentityPromotedFrame*> ValidPromotedFrames = NeutralPose->GetAllPromotedFramesWithValidContourData();
				return !ValidPromotedFrames.IsEmpty();
			}
		}
	}

	return false;
}

void UMetaHumanIdentityFace::ResetTemplateMesh()
{
	InitializeConformalMesh();
}

void UMetaHumanIdentityFace::Conform()
{
	const FString FitConfigPath = GetPluginContentDir() / TEXT("MeshFitting/Template");
	UE::Wrappers::FMetaHumanConformer Conformer;
	const int32 NumTasksForProgressBar = 3;

	FScopedSlowTask ConformTask(NumTasksForProgressBar, LOCTEXT("ConformProgressText", "Running MetaHuman Identity Solve..."));
	ConformTask.MakeDialog();

	ConformTask.EnterProgressFrame();

	if (!Conformer.Init(FitConfigPath))
	{
		UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error initializing FMetaHumanConformer"));
		return;
	}

	if (UMetaHumanIdentityPose* NeutralPose = FindPoseByType(EIdentityPoseType::Neutral))
	{
		const TArray<UMetaHumanIdentityPromotedFrame*> ValidPromotedFrames = NeutralPose->GetAllPromotedFramesWithValidContourData();
		for(UMetaHumanIdentityPromotedFrame* PromotedFrame : ValidPromotedFrames)
		{
			PromotedFrame->UpdateContourDataFromShapeAnnotation();
		}

		if (!NeutralPose->IsCaptureDataValid())
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("CaptureData for Pose '%s' is not valid"), *NeutralPose->GetName());
			return;
		}

		UCaptureData* CaptureData = NeutralPose->GetCaptureData();

		double ConformDuration = 0.0f;

		if (CaptureData->IsA<UMeshCaptureData>())
		{
			const FScopedDurationTimer Timer{ ConformDuration };

			// Run the Mesh Fitting Conformer

			ConformTask.EnterProgressFrame();

			SetConformerCameraParameters(NeutralPose, Conformer);
			SetConformerScanInputData(NeutralPose, Conformer);

			ConformTask.EnterProgressFrame();

			bIsConformed = RunMeshConformer(NeutralPose, Conformer);
		}
		else if (CaptureData->IsA<UFootageCaptureData>())
		{
			// TODO: Run the Footage Conformer
		}

		if (bIsConformed)
		{
			UE_LOG(LogMetaHumanIdentity, Display, TEXT("Conforming took %lf seconds"), ConformDuration);
		}
	}
	else
	{
		UE_LOG(LogMetaHumanIdentity, Error, TEXT("Neutral Pose was not found for MetaHuman Identity Face '%s'"), *GetName());
		return;
	}

	if (UMetaHumanIdentityPose* TeethPose = FindPoseByType(EIdentityPoseType::Teeth))
	{
		// TODO: Conformer.FitTeeth()
	}
}

void UMetaHumanIdentityFace::ApplyDNAToRig(TArray<uint8>& InDNABuffer, bool bInUpdateBlendShapes, bool bInUpdateSkinWeights)
{
	if (CVarEnableExportMeshes.GetValueOnAnyThread())
	{
		const FString PathToDNAFile = FPaths::ProjectSavedDir() / FString::Format(TEXT("{0}_DNA.dna"), { GetOuter()->GetName() });
		FFileHelper::SaveArrayToFile(InDNABuffer, *PathToDNAFile);
	}

	if (RigComponent != nullptr && RigComponent->GetSkeletalMeshAsset()->GetSkeleton() != nullptr)
	{
		double ApplyDNADuration = 0.0;
		FDurationTimer Timer{ ApplyDNADuration };

		const bool bSkipLoadingBlendshapes = false; // we can skip loading blend shapes in editor to reduce memory consumption, if needed
		const  EDNADataLayer DataLayer = bSkipLoadingBlendshapes ? EDNADataLayer::All : EDNADataLayer::AllWithoutBlendShapes;
		const TSharedPtr<IDNAReader> DNAReader = ReadDNAFromBuffer(&InDNABuffer, DataLayer);

		const FScopedTransaction Transaction(LOCTEXT("ApplyDNATransaction", "Apply DNA to Rig"));

		RigComponent->Modify();

		// Map the structures in SkeletalMesh so we can update them; this needs to be done just once at the beginning (not at every update)
		FDNAToSkelMeshMap* DNAToSkelMeshMap = USkelMeshDNAUtils::CreateMapForUpdatingNeutralMesh(DNAReader.Get(), RigComponent->GetSkeletalMeshAsset());

		// TO DO:
		// if there is a window with a skeletal mesh open, then the Behavior in the instance needs to be updated
		// in this test, we don't need this as we are dealing just with assets, not instances
		// for the final version, pass the instance of the skeletal mesh component into this pointer:
		// TObjectPtr<USkeletalMeshComponent> MeshComponent;
		DNAToSkelMeshMap->MapJoints(DNAReader.Get());
		DNAToSkelMeshMap->MapMorphTargets(DNAReader.Get());

		// Set the Behavior part of DNA in skeletal mesh AssetUserData
		UDNAAsset* DNAAsset = NewObject<UDNAAsset>();
		DNAAsset->SetBehaviorReader(DNAReader);
		RigComponent->GetSkeletalMeshAsset()->AddAssetUserData(DNAAsset);

		USkelMeshDNAUtils::UpdateJoints(RigComponent->GetSkeletalMeshAsset(), DNAReader.Get(), DNAToSkelMeshMap);
		USkelMeshDNAUtils::UpdateBaseMesh(RigComponent->GetSkeletalMeshAsset(), DNAReader.Get(), DNAToSkelMeshMap, ELodUpdateOption::LOD0Only);

		if (!bInUpdateBlendShapes)
		{
			USkelMeshDNAUtils::RebuildRenderData_VertexPosition(RigComponent->GetSkeletalMeshAsset());
		}

		if (bInUpdateSkinWeights)
		{
			USkelMeshDNAUtils::UpdateSkinWeights(RigComponent->GetSkeletalMeshAsset(), DNAReader.Get(), DNAToSkelMeshMap, ELodUpdateOption::LOD0Only);
		}

		if (bInUpdateBlendShapes)
		{
			// we know that blend shapes exist only for LOD 0, so here we ignore the Options.LODsToInclude
			USkelMeshDNAUtils::UpdateMorphTargets(RigComponent->GetSkeletalMeshAsset(), DNAReader.Get(), DNAToSkelMeshMap, ELodUpdateOption::LOD0Only);
			USkelMeshDNAUtils::RebuildRenderData(RigComponent->GetSkeletalMeshAsset());
		}

		// Source data must be updated if cooking
		USkelMeshDNAUtils::UpdateSourceData(RigComponent->GetSkeletalMeshAsset());

		Timer.Stop();
		UE_LOG(LogMetaHumanIdentity, Display, TEXT("Apply DNA To Rig took %lf seconds"), ApplyDNADuration);
	}
}

void UMetaHumanIdentityFace::InitializeConformalMesh()
{
	SetConformalMeshTransform(FTransform{ FRotator{ 0.0f, 0.0f, 90.0f } });

	static constexpr const TCHAR* TemplateConformalMeshPath = TEXT("StaticMesh'/MetaHuman/IdentityTemplate/SM_MetaHumanIdentity_Archetype.SM_MetaHumanIdentity_Archetype'");
	if (UStaticMesh* TemplateConformalMesh = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), nullptr, TemplateConformalMeshPath)))
	{
		if (!TemplateConformalMesh->GetSourceModels().IsEmpty())
		{
			ConformalMeshComponent->GetDynamicMesh()->Reset();

			FMeshDescriptionToDynamicMesh DynamicMeshConverter;
			DynamicMeshConverter.Convert(TemplateConformalMesh->GetMeshDescription(0), *ConformalMeshComponent->GetMesh());
			if (ConformalMeshComponent->GetDynamicMesh()->IsEmpty())
			{
				UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error initializing the Identity Template Mesh. Failed to copy mesh from '%s'"), *TemplateConformalMesh->GetFullName());
			}

			ConformalMeshComponent->NotifyMeshUpdated();
		}
		else
		{
			UE_LOG(LogMetaHumanIdentity, Warning, TEXT("Error initializing the Identity Template Mesh. Template mesh '%s' has no source models."), *TemplateConformalMesh->GetFullName());
		}
	}
	else
	{
		UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error initializing the Identity Template Mesh. Failed to load '%s'"), TemplateConformalMeshPath);
	}

	static constexpr const TCHAR* ConformalMeshMaterialPath = TEXT("Material'/MetaHuman/IdentityTemplate/Default_Archetype_Material.Default_Archetype_Material'");
	if (UMaterial* TemplateConformalMaterial = Cast<UMaterial>(StaticLoadObject(UMaterial::StaticClass(), nullptr, ConformalMeshMaterialPath)))
	{
		ConformalMeshComponent->SetOverrideRenderMaterial(TemplateConformalMaterial);
	}

}

void UMetaHumanIdentityFace::InitializeRig()
{
	if (RigComponent != nullptr && RigComponent->GetSkeletalMeshAsset() != nullptr && RigComponent->GetSkeletalMeshAsset()->GetSkeleton() == nullptr)
	{
		IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();

		static constexpr const TCHAR* TemplateRigPath = TEXT("SkeletalMesh'/MetaHuman/MetaHumans/Common/Face/Face_Archetype.Face_Archetype'");
		if (USkeletalMesh* TemplateRig = Cast<USkeletalMesh>(StaticLoadObject(USkeletalMesh::StaticClass(), nullptr, TemplateRigPath)))
		{
			FString NewRigAssetName;
			FString NewRigPath;
			AssetTools.CreateUniqueAssetName(GetOutermost()->GetName(), TEXT("_Rig"), NewRigPath, NewRigAssetName);
			NewRigPath = FPackageName::GetLongPackagePath(NewRigPath);

			RigComponent->SetSkeletalMesh(Cast<USkeletalMesh>(AssetTools.DuplicateAsset(NewRigAssetName, NewRigPath, TemplateRig)));

			if (RigComponent->GetSkeletalMeshAsset() == nullptr || RigComponent->GetSkeletalMeshAsset()->GetSkeleton() == nullptr)
			{
				UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error initializing the MetaHuman Identity Rig. Failed to duplicate the template skeletal mesh from '%s'"), *TemplateRig->GetFullName());
			}
		}
		else
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error initializing the MetaHuman Identity Rig. Failed to load template rig from '%s'"), TemplateRigPath);
		}
	}
}

UMetaHumanIdentityPose* UMetaHumanIdentityFace::FindPoseByType(EIdentityPoseType InPoseType) const
{
	const TObjectPtr<UMetaHumanIdentityPose>* PoseFound = Poses.FindByPredicate([InPoseType](UMetaHumanIdentityPose* Pose)
	{
		return Pose && Pose->PoseType == InPoseType;
	});

	if (PoseFound != nullptr)
	{
		return *PoseFound;
	}

	return nullptr;
}

void UMetaHumanIdentityFace::SetConformalMeshTransform(const FTransform& InTransform)
{
	ConformalMeshComponent->SetWorldTransform(InTransform);
	ConformalMeshComponent->TransformUpdated.Broadcast(ConformalMeshComponent, EUpdateTransformFlags::None, ETeleportType::None);
}

TArray<FVector> UMetaHumanIdentityFace::GetConformalVerticesForAutoRigging() const
{
	const TSet<int32> VertMap = GetObjToUEVertexMapping();

	TArray<FVector> ConformalVertices;

	if (ConformalMeshComponent->GetDynamicMesh() != nullptr && !ConformalMeshComponent->GetDynamicMesh()->IsEmpty())
	{
		UE::Geometry::FDynamicMesh3& Mesh = *ConformalMeshComponent->GetMesh();

		ConformalVertices.SetNumUninitialized(Mesh.VertexCount());
		FTransform Transform = ConformalMeshComponent->GetComponentTransform();

		int32 VertexId = 0;
		for (const int32& Index : VertMap)
		{
			FVector Vertex = Mesh.GetVertex(VertexId++);
			// Transform the vertex to scan space
			Vertex = Transform.TransformPosition(Vertex);
			ConformalVertices[Index] = FVector{ Vertex.Y, -Vertex.Z, Vertex.X };
		}
	}

	return ConformalVertices;
}

TArray<FVector> UMetaHumanIdentityFace::GetConformalVerticesWorldPos() const
{
	const TSet<int32> VertMap = GetObjToUEVertexMapping();

	TArray<FVector> ConformalVertices;

	if (ConformalMeshComponent != nullptr && ConformalMeshComponent->GetDynamicMesh() != nullptr && !ConformalMeshComponent->GetDynamicMesh()->IsEmpty())
	{
		UE::Geometry::FDynamicMesh3& Mesh = *ConformalMeshComponent->GetMesh();
		FTransform Transform = ConformalMeshComponent->GetComponentTransform();

		ConformalVertices.SetNumUninitialized(Mesh.VertexCount());

		int32 VertexId = 0;
		for (const int32& Index : VertMap)
		{
			FVector Vertex = Mesh.GetVertex(VertexId++);
			Vertex = Transform.TransformPosition(Vertex);
			ConformalVertices[Index] = FVector{ Vertex.X, Vertex.Y, Vertex.Z };
		}
	}

	return ConformalVertices;
}

void UMetaHumanIdentityFace::SetConformerCameraParameters(UMetaHumanIdentityPose* InPose, UE::Wrappers::FMetaHumanConformer& OutConformer) const
{
	TArray<FMetaHumanCameraCalibration> CalibrationList;

	for (UMetaHumanIdentityPromotedFrame* PromotedFrame : InPose->GetAllPromotedFramesWithValidContourData())
	{
		if (UMetaHumanIdentityCameraFrame* CameraFrame = Cast<UMetaHumanIdentityCameraFrame>(PromotedFrame))
		{
			const int32 SyntheticWidth = UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.X;
			const int32 SyntheticHeight = UMetaHumanIdentityPromotedFrame::DefaultTrackerImageSize.Y;

			FMetaHumanCameraCalibration Calibration;
			Calibration.Name = CameraFrame->GetName();
			Calibration.ImageSizeX = SyntheticWidth;
			Calibration.ImageSizeY = SyntheticHeight;
			Calibration.CX = SyntheticWidth * 0.5;
			Calibration.CY = SyntheticHeight * 0.5;

			// convert FOV angle to focal length in pixels using:
			//  FOV angle = 2 x arctan (sensor size / 2 f )

			const double ViewFOV = CameraFrame->CameraViewFOV;
			Calibration.FX = SyntheticWidth * 0.5 / FMath::Tan(CameraFrame->CameraViewFOV * TMathUtilConstants<float>::Pi / 360.0);
			Calibration.FY = SyntheticHeight * 0.5 / FMath::Tan(CameraFrame->CameraViewFOV * TMathUtilConstants<float>::Pi / 360.0);

			FTransform CameraTransform = CameraFrame->GetCameraTransform();
			FOpenCVHelper::ConvertUnrealToOpenCV(CameraTransform);

			// camera model matrix is the inverse of the camera position and orientation
			Calibration.Transform = CameraTransform.Inverse().ToMatrixWithScale();

			CalibrationList.Add(Calibration);
		}
	}

	OutConformer.SetCameras(CalibrationList);
}

void UMetaHumanIdentityFace::SetConformerScanInputData(const UMetaHumanIdentityPose* InPose, UE::Wrappers::FMetaHumanConformer& OutConformer) const
{
	TMap<FString, const FFrameTrackingContourData*> ActiveFrameWithData;
	for (UMetaHumanIdentityPromotedFrame* PromotedFrame : InPose->GetAllPromotedFramesWithValidContourData())
	{
		ActiveFrameWithData.Add(PromotedFrame->GetName(), &PromotedFrame->ContourData);
	}

	if (UMeshCaptureData* CaptureData = Cast<UMeshCaptureData>(InPose->GetCaptureData()))
	{
		if (CVarEnableExportMeshes.GetValueOnAnyThread())
		{
			if (UStaticMesh* TargetStaticMesh = Cast<UStaticMesh>(CaptureData->TargetMesh))
			{
				WriteTargetMeshToFile(TargetStaticMesh);
			}
		}

		TArray<float> Vertices;
		TArray<int32> Triangles;
		CaptureData->GetDataForConforming(InPose->PoseTransform, Vertices, Triangles);

		OutConformer.SetScanInputData(ActiveFrameWithData, {}, Triangles, Vertices);
	}
}

bool UMetaHumanIdentityFace::RunMeshConformer(UMetaHumanIdentityPose* InPose, UE::Wrappers::FMetaHumanConformer& OutConformer)
{
	TArray<float> ConformalVerts;
	TArray<float> StackedToScanTransforms;
	TArray<float> StackedScales;
	if (OutConformer.FitIdentity(ConformalVerts, StackedToScanTransforms, StackedScales))
	{
		ConformalMeshComponent->GetDynamicMesh()->EditMesh([&](FDynamicMesh3& InMesh3D)
		{
			const TSet<int32> VertMap = GetObjToUEVertexMapping();
			const TArrayView<FVector3f> ConformalVertsView((FVector3f*) ConformalVerts.GetData(), ConformalVerts.Num() / 3);

			int32 Ctr = 0;
			for (const int32& Index : VertMap)
			{
				// Convert the coordinates back to UE space
				const FVector3f& Vertex = ConformalVertsView[Index];
				InMesh3D.SetVertex(Ctr++, FVector(Vertex.Z, Vertex.X, -Vertex.Y));
			}

			// Need to recompute the overlay normals so it renders the mesh correctly
			UE::Geometry::FMeshNormals::QuickRecomputeOverlayNormals(InMesh3D);

		}, EDynamicMeshChangeType::MeshVertexChange, EDynamicMeshAttributeChangeFlags::VertexPositions | EDynamicMeshAttributeChangeFlags::NormalsTangents);

		if (InPose->GetCaptureData()->IsA<UMeshCaptureData>())
		{
			// Set the head transform
			FMatrix44f TransformMatrix;
			FMemory::Memcpy(TransformMatrix.M, StackedToScanTransforms.GetData(), sizeof(FMatrix44f));
			FTransform HeadTransform{ FMatrix{ TransformMatrix } };
			FOpenCVHelper::ConvertOpenCVToUnreal(HeadTransform);
			HeadTransform.SetScale3D(FVector(StackedScales[0]));
			HeadTransform.SetTranslation(HeadTransform.GetTranslation() * HeadTransform.GetScale3D());

			SetConformalMeshTransform(HeadTransform);

			if (CVarEnableExportMeshes.GetValueOnAnyThread())
			{
				WriteConformalVerticesToFile();
			}

			// For scan data, set the same transform for all promoted frames
			InPose->SetHeadAlignment(HeadTransform);
		}
		else if (InPose->GetCaptureData()->IsA<UFootageCaptureData>())
		{
			// TODO: For the footage case we get one transform for each depth frame so need to place the transforms in the promoted frames using
			// InPose->SetHeadAlignmentForPromotedFrame()
		}

		return true;
	}
	else
	{
		UE_LOG(LogMetaHumanIdentity, Error, TEXT("Unable to fit the mesh"));
		return false;
	}
}

void UMetaHumanIdentityFace::WriteConformalVerticesToFile(const FString& InNameSuffix) const
{
	TArray<FString> Data;

	Data.Add("# This file uses centimeters as units for non-parametric coordinates.");
	Data.Add("");
	Data.Add("mtllib mean.mtl");
	Data.Add("g default");

	for (FVector Vertex : GetConformalVerticesForAutoRigging())
	{
		// Transform the vertex back to UE space
		Vertex = FVector(Vertex.Z, Vertex.X, -Vertex.Y);

		// Finally, transform it to Obj space (flip Z and Y) so it is exported in the correct orientation
		Data.Add(FString::Format(TEXT("v {0} {1} {2}"), { Vertex.X, Vertex.Z, Vertex.Y }));
	}

	const FString PathToMeanObj = GetPluginContentDir() / TEXT("MeshFitting/Template/mean.obj");

	TArray<FString> Faces;
	FFileHelper::LoadFileToStringArrayWithPredicate(Faces, *PathToMeanObj, [](const FString& Line)
	{
		return Line.StartsWith("f ") || Line.StartsWith("vt ") || Line.StartsWith("vn ");
	});

	for (const FString& FaceString : Faces)
	{
		Data.Add(FaceString);
	}

	const FString PathToConformalObject = FPaths::ProjectSavedDir() / FString::Format(TEXT("{0}_ConformalMesh{1}.obj"), { GetOuter()->GetName(), InNameSuffix });
	FFileHelper::SaveStringArrayToFile(Data, *PathToConformalObject);
}

void UMetaHumanIdentityFace::WriteTargetMeshToFile(UStaticMesh* InTargetMesh, const FString& InNameSuffix) const
{
	if (InTargetMesh != nullptr)
	{
		TArray<UExporter*> Exporters;
		ObjectTools::AssembleListOfExporters(Exporters);

		UExporter* ObjExporter = nullptr;
		for (int32 ExporterIndex = Exporters.Num() - 1; ExporterIndex >= 0; --ExporterIndex)
		{
			UExporter* Exporter = Exporters[ExporterIndex];
			if (Exporter->SupportedClass == UStaticMesh::StaticClass() &&
				Exporter->FormatExtension.Contains(TEXT("OBJ")))
			{
				ObjExporter = Exporter;
				break;
			}
		}

		if (ObjExporter != nullptr)
		{
			UAssetExportTask* ExportTask = NewObject<UAssetExportTask>();
			FGCObjectScopeGuard ExportTaskGuard(ExportTask);
			ExportTask->Object = InTargetMesh;
			ExportTask->Exporter = ObjExporter;
			ExportTask->Filename = FPaths::ProjectSavedDir() / FString::Format(TEXT("{0}_ScannedMesh{1}.obj"), { GetOuter()->GetName(), InNameSuffix });
			ExportTask->bSelected = false;
			ExportTask->bReplaceIdentical = true;
			ExportTask->bPrompt = false;
			ExportTask->bUseFileArchive = true;
			ExportTask->bWriteEmptyFiles = false;
			UExporter::RunAssetExportTask(ExportTask);
		}
	}
}

FString UMetaHumanIdentityFace::GetPluginContentDir() const
{
	return IPluginManager::Get().FindPlugin(UE_PLUGIN_NAME)->GetContentDir();
}

TSet<int32> UMetaHumanIdentityFace::GetObjToUEVertexMapping() const
{
	const FString ObjFilePath = GetPluginContentDir() / TEXT("MeshFitting/Template/mean.obj");

	TSet<int32> Indices;
	FFileHelper ObjParser;
	ObjParser.LoadFileToStringWithLineVisitor(*ObjFilePath, [&Indices](FStringView InView)
	{
		if (InView.Left(2).Equals("f "))
		{
			InView.RightChopInline(2);
			int32 IndexDash = INDEX_NONE;
			int32 IndexWhitespace = INDEX_NONE;
			int32 Itr = 0;

			while (Itr < 4)
			{
				InView.FindChar(TCHAR('/'), IndexDash);
				InView.FindChar(TCHAR(' '), IndexWhitespace);

				auto NumStrin = FString(InView.Left(IndexDash));
				int32 Ind = FCString::Atoi(*NumStrin);
				Indices.Add(Ind - 1);

				InView.RightChopInline(IndexWhitespace + 1);
				++Itr;
			}
		}
	});

	return Indices;
}

//////////////////////////////////////////////////////////////////////////
// UMetaHumanIdentityBody

UMetaHumanIdentityBody::UMetaHumanIdentityBody()
	: Super{}
	, Height(1)
	, BodyTypeIndex(INDEX_NONE)
{
}

FText UMetaHumanIdentityBody::GetPartName() const
{
	return LOCTEXT("IdentityBodyComponentName", "Body");
}

FText UMetaHumanIdentityBody::GetPartDescription() const
{
	return LOCTEXT("IdentityBodyComponentDescription", "The Body of the MetaHuman Identity");
}

FSlateIcon UMetaHumanIdentityBody::GetPartIcon(const FName& InPropertyName) const
{
	return FSlateIcon{ FMetaHumanIdentityStyle::Get().GetStyleSetName(), TEXT("Identity.Body.Part") };
}


//////////////////////////////////////////////////////////////////////////
// UMetaHumanIdentityHands

UMetaHumanIdentityHands::UMetaHumanIdentityHands()
	: Super{}
{
}

FText UMetaHumanIdentityHands::GetPartName() const
{
	return LOCTEXT("IdentityHandComponentName", "Hands");
}

FText UMetaHumanIdentityHands::GetPartDescription() const
{
	return LOCTEXT("IdentityHandComponentDescription", "The Hands of the MetaHuman Identity");
}

FSlateIcon UMetaHumanIdentityHands::GetPartIcon(const FName& InPropertyName) const
{
	return FSlateIcon{};
}


//////////////////////////////////////////////////////////////////////////
// UMetaHumanIdentityOutfit

UMetaHumanIdentityOutfit::UMetaHumanIdentityOutfit()
	: Super{}
{
}

FText UMetaHumanIdentityOutfit::GetPartName() const
{
	return LOCTEXT("IdentityOutfitComponentName", "Outfit");
}

FText UMetaHumanIdentityOutfit::GetPartDescription() const
{
	return LOCTEXT("IdentityOutfitComponentDescription", "The Outfit of the MetaHuman Identity");
}

FSlateIcon UMetaHumanIdentityOutfit::GetPartIcon(const FName& InPropertyName) const
{
	return FSlateIcon{};
}

//////////////////////////////////////////////////////////////////////////
// UMetaHumanIdentityProp

UMetaHumanIdentityProp::UMetaHumanIdentityProp()
	: Super{}
{
}

FText UMetaHumanIdentityProp::GetPartName() const
{
	return LOCTEXT("IdentityPropComponentName", "Prop");
}

FText UMetaHumanIdentityProp::GetPartDescription() const
{
	return LOCTEXT("IdentityPropComponentDescription", "A Prop for the MetaHuman Identity");
}

FSlateIcon UMetaHumanIdentityProp::GetPartIcon(const FName& InPropertyName) const
{
	return FSlateIcon{};
}

#undef LOCTEXT_NAMESPACE