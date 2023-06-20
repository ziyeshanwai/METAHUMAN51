// Copyright Epic Games, Inc. All Rights Reserved.

#include "SMetaHumanIdentityPartsEditor.h"

#include "MetaHumanIdentity.h"
#include "MetaHumanIdentityParts.h"
#include "MetaHumanIdentityStyle.h"
#include "MetaHumanIdentityLog.h"
#include "CaptureData.h"
#include "ConformingViewportClient.h"
#include "SMetaHumanIdentityPartsClassCombo.h"

#include "Widgets/Input/SSearchBox.h"
#include "Widgets/Images/SImage.h"
#include "Framework/Commands/GenericCommands.h"
#include "EditorViewportCommands.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "Framework/Commands/UICommandList.h"
#include "Components/DynamicMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "ScopedTransaction.h"
#include "Editor/Transactor.h"

#include "Editor.h"
#include "PreviewScene.h"
#include "AssetToolsModule.h"
#include "Algo/AllOf.h"


#define LOCTEXT_NAMESPACE "MetaHumanIdentityPartsEditor"

/////////////////////////////////////////////////////
// FIdentityTreeNode

FIdentityTreeNode::FIdentityTreeNode(class UMetaHumanIdentityPart* InIdentityPart, AActor* InIdentityActor, FName InPropertyName, USceneComponent* InPreviewComponent, ESceneComponentIdentifier InComponentIdentifier)
	: IdentityPart{ InIdentityPart }
	, PreviewSceneComponent{ InPreviewComponent }
	, SceneComponentIdentifier{ InComponentIdentifier }
{
	if (!InPropertyName.IsNone())
	{
		// This is a node that points to a member of InIdentityPart
		IdentityPartPropertyName = InPropertyName;

		SetupPreviewSceneComponentInstance(InIdentityActor);
	}
	else
	{
		// This is a node that directly represents a Part
		if (UMetaHumanIdentityFace* Face = Cast<UMetaHumanIdentityFace>(InIdentityPart))
		{
			// The face will have one node for each pose plus one for the template mesh and one for the resulting rig
			Children.Reserve(2 + Face->Poses.Num());

			if (Face->ConformalMeshComponent != nullptr && Face->ConformalMeshComponent->GetDynamicMesh() != nullptr)
			{
				TSharedRef<FIdentityTreeNode> TemplateMeshNode = MakeShareable(new FIdentityTreeNode{ Face, InIdentityActor, GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityFace, ConformalMeshComponent), Face->ConformalMeshComponent, ESceneComponentIdentifier::ConformalMesh });

				if (UDynamicMeshComponent* TemplateMeshComponentInstance = Cast<UDynamicMeshComponent>(TemplateMeshNode->PreviewSceneComponentInstance))
				{
					TemplateMeshComponentInstance->SetDynamicMesh(Face->ConformalMeshComponent->GetDynamicMesh());
				}

				Children.Add(TemplateMeshNode);
			}

			if (Face->RigComponent != nullptr && Face->RigComponent->GetSkeletalMeshAsset() != nullptr && Face->RigComponent->GetSkeletalMeshAsset()->GetSkeleton() != nullptr)
			{
				Children.Add(MakeShareable(new FIdentityTreeNode{ Face, InIdentityActor, GET_MEMBER_NAME_CHECKED(UMetaHumanIdentityFace, RigComponent), Face->RigComponent, ESceneComponentIdentifier::ConformalRig }));
			}

			// Create one node for each pose already stored in the Face Part
			for (UMetaHumanIdentityPose* Pose : Face->Poses)
			{
				ESceneComponentIdentifier PoseSceneComponentIdentifier = Pose->PoseType == EIdentityPoseType::Neutral ? ESceneComponentIdentifier::ImportedMesh : ESceneComponentIdentifier::None;
				Children.Add(MakeShareable(new FIdentityTreeNode{ Pose, InIdentityActor, PoseSceneComponentIdentifier }));
			}
		}
		else if (UMetaHumanIdentityBody* Body = Cast<UMetaHumanIdentityBody>(IdentityPart))
		{
			// TODO: Handle this case
		}
		else if (UMetaHumanIdentityHands* Hands = Cast<UMetaHumanIdentityHands>(IdentityPart))
		{
			// TODO: Handle this case
		}
		else if (UMetaHumanIdentityOutfit* Outfit = Cast<UMetaHumanIdentityOutfit>(IdentityPart))
		{
			// TODO: Handle this case
		}
		else if (UMetaHumanIdentityProp* Prop = Cast<UMetaHumanIdentityProp>(IdentityPart))
		{
			// TODO: Handle this case
		}
	}
}


FIdentityTreeNode::FIdentityTreeNode(UMetaHumanIdentityPose* InIdentityPose, AActor* InIdentityActor, ESceneComponentIdentifier InIdentifier)
	: IdentityPose{ InIdentityPose }
	, PreviewSceneComponent{ InIdentityPose->CaptureDataSceneComponent }
	, SceneComponentIdentifier{ InIdentifier }
{
	SetupPreviewSceneComponentInstance(InIdentityActor);
}

FIdentityTreeNode::FIdentityTreeNode(UMetaHumanIdentity* InIdentity, AActor* InIdentityActor)
	: Identity{ InIdentity }
	, SceneComponentIdentifier(ESceneComponentIdentifier::None)
{
	Children.Reserve(InIdentity->Parts.Num());

	// Add one child node for each part already in the Identity
	for (UMetaHumanIdentityPart* Part : InIdentity->Parts)
	{
		Children.Add(MakeShareable(new FIdentityTreeNode{ Part, InIdentityActor }));
	}
}

void FIdentityTreeNode::SetupPreviewSceneComponentInstance(AActor* InIdentityActor)
{
	if (PreviewSceneComponent.IsValid())
	{
		// The PreviewSceneComponentInstance is what is actually is displayed in the viewport. Duplicate object will duplicate the Scene component that
		// was serialized last time so the viewport is kept up-to-date and will display any changes the user has saved
		PreviewSceneComponentInstance = DuplicateObject<USceneComponent>(PreviewSceneComponent.Get(), InIdentityActor, *PreviewSceneComponent->GetName().Append(TEXT("_Instance")));
		check(PreviewSceneComponentInstance.IsValid());
		PreviewSceneComponentInstance->SetFlags(RF_Transient);
		InIdentityActor->AddOwnedComponent(PreviewSceneComponentInstance.Get());

		// The PreviewSceneComponent is what the user is editing in the details panel. This will make sure any changes to the transform
		// component of will get copied to the instance that is displayed on the screen
		PreviewSceneComponent->TransformUpdated.AddLambda([this](USceneComponent* InRootComponent, EUpdateTransformFlags, ETeleportType)
		{
			if (PreviewSceneComponent.IsValid() && PreviewSceneComponentInstance.IsValid() && PreviewSceneComponent == InRootComponent)
			{
				PreviewSceneComponentInstance->SetWorldTransform(PreviewSceneComponent->GetComponentTransform());
			}
		});
	}
}

void FIdentityTreeNode::UpdateSceneComponentInstanceProperty(FProperty* InProperty)
{
	if (InProperty != nullptr && PreviewSceneComponent.IsValid() && PreviewSceneComponentInstance.IsValid())
	{
		if (PreviewSceneComponentInstance->GetClass()->HasProperty(InProperty) && PreviewSceneComponent->GetClass()->HasProperty(InProperty))
		{
			InProperty->CopyCompleteValue_InContainer(PreviewSceneComponentInstance.Get(), PreviewSceneComponent.Get());

			// When the transform property changes in the details panel the actual transform, the ComponentToWorld member,
			// doesn't change immediately, so this needs to be called. This happen because the component is registered
			// to a world but calling UpdateComponentToWorld solves it
			PreviewSceneComponent->UpdateComponentToWorld();
			PreviewSceneComponentInstance->UpdateComponentToWorld();

			// Makes sure any change to rendering properties from the CopyCompleteValue_InContainer above are updated in the viewport
			PreviewSceneComponentInstance->MarkRenderStateDirty();
		}
		else
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("UpdateSceneComponentInstanceProperty called with a property named '%s' that doesn't exist in class %s"), *InProperty->GetFName().ToString(), *PreviewSceneComponent->GetClass()->GetName());
		}
	}
}

bool FIdentityTreeNode::CanDelete() const
{
	// Don't allow the root node to be deleted
	if (Identity.IsValid())
	{
		return false;
	}

	if (IdentityPart.IsValid())
	{
		// If this is a part that doesn't point to a property it can be deleted
		if (IdentityPartPropertyName.IsNone())
		{
			return true;
		}
	}

	// A pose can also be deleted
	if (IdentityPose.IsValid())
	{
		return true;
	}

	return false;
}

FText FIdentityTreeNode::GetDisplayText() const
{
	if (Identity.IsValid())
	{
		return FText::FromString(Identity->GetName());
	}

	if (IdentityPart.IsValid())
	{
		if (!IdentityPartPropertyName.IsNone())
		{
			return GetObjectProperty()->GetDisplayNameText();
		}
		else
		{
			return IdentityPart->GetPartName();
		}
	}

	if (IdentityPose.IsValid())
	{
		return IdentityPose->PoseName;
	}

	return LOCTEXT("InvalidNodeName", "<Invalid Node>");
}

const FSlateBrush* FIdentityTreeNode::GetDisplayIconBrush() const
{
	FMetaHumanIdentityStyle& Style = FMetaHumanIdentityStyle::Get();

	if (Identity.IsValid())
	{
		return Style.GetBrush("Identity.Root");
	}

	if (IdentityPart.IsValid())
	{
		return IdentityPart->GetPartIcon(IdentityPartPropertyName).GetIcon();
	}

	if (IdentityPose.IsValid())
	{
		return IdentityPose->GetPoseIcon().GetIcon();
	}

	return nullptr;
}

UObject* FIdentityTreeNode::GetObject() const
{
	if (Identity.IsValid())
	{
		return Identity.Get();
	}

	if (IdentityPart.IsValid())
	{
		if (!IdentityPartPropertyName.IsNone())
		{
			if (PreviewSceneComponent.IsValid())
			{
				return PreviewSceneComponent.Get();
			}
			else
			{
				// Gets the value of the property named IdentityPartPropertyName in the Part object
				return GetObjectProperty()->GetObjectPropertyValue_InContainer(IdentityPart.Get());
			}
		}
		else
		{
			return IdentityPart.Get();
		}
	}

	if (IdentityPose.IsValid())
	{
		return IdentityPose.Get();
	}

	return nullptr;
}

FObjectProperty* FIdentityTreeNode::GetObjectProperty() const
{
	if (IdentityPart.IsValid() && !IdentityPartPropertyName.IsNone())
	{
		return FindFProperty<FObjectProperty>(IdentityPart->GetClass(), *IdentityPartPropertyName.ToString());
	}

	return nullptr;
}

/////////////////////////////////////////////////////
// SMetaHumanIdentityPartsEditor

void SMetaHumanIdentityPartsEditor::Construct(const FArguments& InArgs)
{
	check(InArgs._Identity);
	check(InArgs._ViewportClient);

	OnIdentityPartAddedDelegate = InArgs._OnIdentityPartAdded;
	OnIdentityPartRemovedDelegate = InArgs._OnIdentityPartRemoved;
	OnIdentityPoseAddedDelegate = InArgs._OnIdentityPoseAdded;
	OnIdentityPoseRemovedDelegate = InArgs._OnIdentityPoseRemoved;
	OnIdentityTreeSelectionChangedDelegate = InArgs._OnIdentityTreeSelectionChanged;

	IdentityPtr = InArgs._Identity;
	ViewportClient = InArgs._ViewportClient;
	ViewportClient->OnSceneComponentClicked().BindSP(this, &SMetaHumanIdentityPartsEditor::HandleSceneComponentClicked);
	ViewportClient->OnGetSceneComponetOfType().BindSP(this, &SMetaHumanIdentityPartsEditor::GetSceneComponentOfType, false);
	ViewportClient->OnGetSceneComponentInstanceOfType().BindSP(this, &SMetaHumanIdentityPartsEditor::GetSceneComponentOfType, true);

	BindCommands();

	ChildSlot
	[
		SNew(SVerticalBox)
			+ SVerticalBox::Slot()
			.AutoHeight()
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
				.VAlign(VAlign_Center)
				.Padding(4.0f, 0.0f, 4.0f, 0.0f)
				.AutoWidth()
				[
					SNew(SMetaHumanIdentityPartsClassCombo)
					.OnIdentityPartClassSelected(this, &SMetaHumanIdentityPartsEditor::HandleAddIdentityPartOfClass)
					.OnIdentityPoseClassSelected(this, &SMetaHumanIdentityPartsEditor::HandleAddIdentityPoseOfClass)
					.OnIsIdentityPartClassEnabled(this, &SMetaHumanIdentityPartsEditor::CanAddIdentityPartOfClass)
					.OnIsIdentityPoseClassEnabled(this, &SMetaHumanIdentityPartsEditor::CanAddIdentityPoseOfClass)
				]
				+ SHorizontalBox::Slot()
				.Padding(6.0f)
				[
					SNew(SSearchBox)
					.OnTextChanged(this, &SMetaHumanIdentityPartsEditor::HandleIdentityFilterTextChanged)
				]
			]
			+ SVerticalBox::Slot()
			[
				SAssignNew(IdentityTreeWidget, STreeView<TSharedRef<FIdentityTreeNode>>)
				.SelectionMode(ESelectionMode::Single)
				.TreeItemsSource(&RootNodes)
				.AllowInvisibleItemSelection(false)
				.OnGenerateRow(this, &SMetaHumanIdentityPartsEditor::HandleIdentityTreeGenerateRow)
				.OnGetChildren(this, &SMetaHumanIdentityPartsEditor::HandleIdentityTreeGetChildren)
				.OnSelectionChanged(this, &SMetaHumanIdentityPartsEditor::HandleIdentityTreeSelectionChanged)
				.OnSetExpansionRecursive(this, &SMetaHumanIdentityPartsEditor::HandleIdentityTreeExpanstionRecursive)
				.OnContextMenuOpening(this, &SMetaHumanIdentityPartsEditor::HandleIdentityTreeContextMenu)
				.HighlightParentNodesForSelection(true)
			]
	];

	// Create the preview actor to be displayed in the viewport
	SpawnIdentityPreviewActor();

	// Builds the Identity hierarchy with the Parts/Poses it already has
	RefreshIdentityTree();

	// Add all preview scene components from the Identity in the viewport
	AddAllPreviewSceneComponentInstances(GetIdentityRootNode());
}

SMetaHumanIdentityPartsEditor::~SMetaHumanIdentityPartsEditor()
{
	RemoveAllPreviewSceneComponents(GetIdentityRootNode());
}

void SMetaHumanIdentityPartsEditor::PostUndo(bool bInSuccess)
{
	if (bInSuccess)
	{
		const FTransaction* Transaction = GEditor->Trans->GetTransaction(GEditor->Trans->GetQueueLength() - GEditor->Trans->GetUndoCount());
		HandleUndoOrRedoTransaction(Transaction);
	}
}

void SMetaHumanIdentityPartsEditor::PostRedo(bool bInSuccess)
{
	if (bInSuccess)
	{
		const FTransaction* Transaction = GEditor->Trans->GetTransaction(GEditor->Trans->GetQueueLength() - GEditor->Trans->GetUndoCount() - 1);
		HandleUndoOrRedoTransaction(Transaction);
	}
}

void SMetaHumanIdentityPartsEditor::AddReferencedObjects(FReferenceCollector& InCollector)
{
	if (IdentityPreviewActorInstance != nullptr)
	{
		InCollector.AddReferencedObject(IdentityPreviewActorInstance);
	}
}

FString SMetaHumanIdentityPartsEditor::GetReferencerName() const
{
	return TEXT("SMetaHumanIdentityPartsEditor");
}

void SMetaHumanIdentityPartsEditor::AddFaceFromMesh(UObject* InMesh)
{
	if (InMesh != nullptr)
	{
		if (InMesh->IsA<UStaticMesh>() || InMesh->IsA<USkeletalMesh>())
		{
			const FScopedTransaction Transaction(LOCTEXT("AddFaceFromStaticMesh", "Add MetaHuman Identity Face from Static Mesh"));

			IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>(TEXT("AssetTools")).Get();

			FString NewCaptureDataAssetName;
			FString NewCaptureDataPath;
			AssetTools.CreateUniqueAssetName(InMesh->GetOutermost()->GetName(), TEXT("_CaptureData"), NewCaptureDataPath, NewCaptureDataAssetName);
			NewCaptureDataPath = FPackageName::GetLongPackagePath(GetIdentity()->GetOutermost()->GetName());

			if (UMeshCaptureData* CaptureData = Cast<UMeshCaptureData>(AssetTools.CreateAsset(NewCaptureDataAssetName, NewCaptureDataPath, UMeshCaptureData::StaticClass(), nullptr)))
			{
				CaptureData->Modify();
				CaptureData->TargetMesh = InMesh;

				HandleAddIdentityPartOfClass(UMetaHumanIdentityFace::StaticClass());
				UMetaHumanIdentityFace* FacePart = GetIdentity()->FindPartOfClass<UMetaHumanIdentityFace>();

				HandleAddIdentityPoseOfClass(UMetaHumanIdentityPose::StaticClass(), EIdentityPoseType::Neutral);
				UMetaHumanIdentityPose* NeutralPose = FacePart->FindPoseByType(EIdentityPoseType::Neutral);
				NeutralPose->SetCaptureData(CaptureData);

				if (GetIdentity()->CanAddPartOfClass(UMetaHumanIdentityBody::StaticClass()))
				{
					HandleAddIdentityPartOfClass(UMetaHumanIdentityBody::StaticClass());
				}
			}

			UpdateSceneComponentVisiblity();

			if (ViewportClient.IsValid())
			{
				SelectAndExpandIdentityTreeNode(GetIdentityRootNode());
				ViewportClient->FocusViewportOnSelectedComponents();
			}
		}
		else
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("Unable to create a Face from mesh of type '%s'. It should be either a Static or Skeletal mesh"), *InMesh->GetClass()->GetName());
		}
	}
	else
	{
		UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error creating Face from mesh. Mesh object is not valid"));
	}
}

USceneComponent* SMetaHumanIdentityPartsEditor::GetSceneComponentOfType(ESceneComponentIdentifier InComponentIdentifier, bool bInInstance) const
{
	if (TSharedPtr<FIdentityTreeNode> FoundNode = FindIdentityNodeByComponentId(InComponentIdentifier, GetIdentityRootNode()))
	{
		return bInInstance ? FoundNode->PreviewSceneComponentInstance.Get() : FoundNode->PreviewSceneComponent.Get();
	}

	return nullptr;
}

void SMetaHumanIdentityPartsEditor::UpdateViewportSelectionOutlines(bool bShowSelectionOutlines)
{
	TArray<USceneComponent*> SelectedComponentInstances;
	bool bDeselectAll = true;

	const TArray<TSharedRef<FIdentityTreeNode>> SelectedTreeItems = IdentityTreeWidget->GetSelectedItems();

	if (!SelectedTreeItems.IsEmpty())
	{
		const TSharedRef<FIdentityTreeNode> SelectedNode = SelectedTreeItems[0];

		const bool bInstances = true;
		const bool bOnlyVisible = true;
		FindAllPreviewSceneComponents(SelectedNode, SelectedComponentInstances, bInstances, bOnlyVisible);

		bDeselectAll = SelectedComponentInstances.IsEmpty() || !bShowSelectionOutlines;
	}

	if (ViewportClient.IsValid())
	{
		ViewportClient->Invalidate();
	}

	if (IdentityPreviewActorInstance != nullptr)
	{
		TArray<UActorComponent*> AllComponents;
		IdentityPreviewActorInstance->GetComponents(UPrimitiveComponent::StaticClass(), AllComponents);

		for (UActorComponent* PreviewComponent : AllComponents)
		{
			if (UPrimitiveComponent* PrimitivePreviewComponent = Cast<UPrimitiveComponent>(PreviewComponent))
			{
				const bool bSelect = !bDeselectAll && SelectedComponentInstances.Contains(PrimitivePreviewComponent);
				PrimitivePreviewComponent->SelectionOverrideDelegate.BindLambda([bSelect](const UPrimitiveComponent*) { return bSelect; });
				PrimitivePreviewComponent->PushSelectionToProxy();
			}
		}
	}
}

void SMetaHumanIdentityPartsEditor::NotifyPostChange(const FPropertyChangedEvent& InPropertyChangedEvent, FProperty* InPropertyThatChanged)
{
	if (InPropertyThatChanged != nullptr)
	{
		const TArray<TSharedRef<FIdentityTreeNode>> SelectedTreeItems = IdentityTreeWidget->GetSelectedItems();
		if (SelectedTreeItems.Num() == 1)
		{
			TSharedRef<FIdentityTreeNode> SelectedItem = SelectedTreeItems[0];
			if (SelectedItem->PreviewSceneComponent.IsValid() && SelectedItem->PreviewSceneComponentInstance.IsValid())
			{
				if (InPropertyThatChanged->GetOwnerClass()->IsChildOf<USceneComponent>())
				{
					SelectedItem->UpdateSceneComponentInstanceProperty(InPropertyThatChanged);
				}
			}
		}
	}
}

FReply SMetaHumanIdentityPartsEditor::OnKeyDown(const FGeometry& InGeometry, const FKeyEvent& InKeyEvent)
{
	// Function required to process keyboard events in the tree view
	if (CommandList->ProcessCommandBindings(InKeyEvent))
	{
		return FReply::Handled();
	}

	return FReply::Unhandled();
}

void SMetaHumanIdentityPartsEditor::BindCommands()
{
	CommandList = MakeShared<FUICommandList>();

	CommandList->MapAction(FGenericCommands::Get().Delete,
						   FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityPartsEditor::HandleIdentityTreeDeleteSelectedNode),
									 FCanExecuteAction::CreateSP(this, &SMetaHumanIdentityPartsEditor::CanDeleteSelectedIdentityTreeNode)));

	CommandList->MapAction(FEditorViewportCommands::Get().FocusViewportToSelection,
						   FUIAction(FExecuteAction::CreateSP(this, &SMetaHumanIdentityPartsEditor::HandleFocusToSelection),
									 FCanExecuteAction::CreateSP(this, &SMetaHumanIdentityPartsEditor::CanFocusToSelection)));
}

void SMetaHumanIdentityPartsEditor::SpawnIdentityPreviewActor()
{
	FActorSpawnParameters SpawnInfo;
	SpawnInfo.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	SpawnInfo.bNoFail = true;
	SpawnInfo.ObjectFlags = RF_Transient;
	IdentityPreviewActorInstance = GetPreviewScene()->GetWorld()->SpawnActor<AActor>(SpawnInfo);

	// Create a root scene component for the preview actor
	// Automatic attachment means this will be the new root component
	const bool bManualAttachment = false;
	const bool bDeferredFinish = false;
	IdentityPreviewActorInstance->AddComponentByClass(USceneComponent::StaticClass(), bManualAttachment, FTransform{}, bDeferredFinish);

	check(IdentityPreviewActorInstance);

	// Make sure the preview actor instance is visible in the viewport
	if (IdentityPreviewActorInstance->IsHidden())
	{
		IdentityPreviewActorInstance->SetHidden(false);
		IdentityPreviewActorInstance->MarkComponentsRenderStateDirty();
	}
}

void SMetaHumanIdentityPartsEditor::FindAllPreviewSceneComponents(const TSharedRef<FIdentityTreeNode>& InNode, TArray<USceneComponent*>& OutPreviewComponents, bool bInInstances, bool bInOnlyVisible) const
{
	if ((InNode->PreviewSceneComponent.IsValid() && !bInInstances) ||
		(InNode->PreviewSceneComponentInstance.IsValid() && bInInstances))
	{
		USceneComponent* Component = bInInstances ? InNode->PreviewSceneComponentInstance.Get() : InNode->PreviewSceneComponent.Get();

		if (bInOnlyVisible && Component->IsVisible())
		{
			OutPreviewComponents.Add(Component);
		}
		else
		{
			OutPreviewComponents.Add(Component);
		}
	}

	for (const TSharedRef<FIdentityTreeNode>& ChildNode : InNode->Children)
	{
		FindAllPreviewSceneComponents(ChildNode, OutPreviewComponents, bInInstances, bInOnlyVisible);
	}
}

void SMetaHumanIdentityPartsEditor::AddAllPreviewSceneComponentInstances(const TSharedRef<FIdentityTreeNode>& InNode)
{
	const bool bInstances = true;
	const bool bOnlyVisible = false;
	TArray<USceneComponent*> SceneComponents;
	FindAllPreviewSceneComponents(InNode, SceneComponents, bInstances, bOnlyVisible);

	for (USceneComponent* SceneComponent : SceneComponents)
	{
		GetPreviewScene()->AddComponent(SceneComponent, SceneComponent->GetRelativeTransform());
	}
}

void SMetaHumanIdentityPartsEditor::RemoveAllPreviewSceneComponents(const TSharedRef<FIdentityTreeNode>& InNode)
{
	const bool bInstances = true;
	const bool bOnlyVisible = false;
	TArray<USceneComponent*> SceneComponents;
	FindAllPreviewSceneComponents(InNode, SceneComponents, bInstances, bOnlyVisible);

	for (USceneComponent* SceneComponent : SceneComponents)
	{
		GetPreviewScene()->RemoveComponent(SceneComponent);
	}
}

void SMetaHumanIdentityPartsEditor::RefreshWidget()
{
	if (IsIdentityTreeValid())
	{
		RemoveAllPreviewSceneComponents(GetIdentityRootNode());
	}

	// Builds the Identity hierarchy with the Parts/Poses it already has
	RefreshIdentityTree();

	// Add all preview scene components from the Identity in the viewport
	AddAllPreviewSceneComponentInstances(GetIdentityRootNode());

	if (IdentityPreviewActorInstance != nullptr)
	{
		IdentityPreviewActorInstance->MarkComponentsRenderStateDirty();
	}
}

void SMetaHumanIdentityPartsEditor::RefreshIdentityTree()
{
	UMetaHumanIdentity* Identity = GetIdentity();

	// Register a delegate to handle changes in the capture source of a pose
	if (UMetaHumanIdentityFace* FacePart = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
	{
		for (UMetaHumanIdentityPose* Pose : FacePart->Poses)
		{
			Pose->OnCaptureDataChanged().AddSP(this, &SMetaHumanIdentityPartsEditor::HandleIdentityPoseCaptureDataChanged, Pose);
		}
	}

	// Rebuild the Identity hierarchy
	RootNodes = { MakeShareable(new FIdentityTreeNode{ Identity, IdentityPreviewActorInstance }) };

	// Expand all the nodes
	HandleIdentityTreeExpanstionRecursive(GetIdentityRootNode(), true);

	IdentityTreeWidget->SetSelection(GetIdentityRootNode());
	IdentityTreeWidget->RequestTreeRefresh();
}

void SMetaHumanIdentityPartsEditor::HandleAddIdentityPartOfClass(TSubclassOf<UMetaHumanIdentityPart> InIdentityPartClass)
{
	UMetaHumanIdentity* Identity = GetIdentity();

	if (Identity->CanAddPartOfClass(InIdentityPartClass))
	{
		if (UMetaHumanIdentityPart* NewIdentityPart = NewObject<UMetaHumanIdentityPart>(Identity, InIdentityPartClass, NAME_None, RF_Transactional))
		{
			const FScopedTransaction Transaction(UMetaHumanIdentity::IdentityTransactionContext, LOCTEXT("AddIdentityPart", "Add Part to the MetaHuman Identity"), Identity);

			Identity->Modify();

			// Add the part to the Identity
			Identity->Parts.Add(NewIdentityPart);

			// Add the new Part to the tree view, this will create its preview scene component
			TSharedRef<FIdentityTreeNode> NewPartNode = MakeShareable(new FIdentityTreeNode{ NewIdentityPart, IdentityPreviewActorInstance });

			GetIdentityRootNode()->Children.Add(NewPartNode);

			AddAllPreviewSceneComponentInstances(NewPartNode);

			SelectAndExpandIdentityTreeNode(NewPartNode);

			// Update the visibility of the preview scene components
			UpdateSceneComponentVisiblity();

			// Notify that a new Part was added
			OnIdentityPartAddedDelegate.ExecuteIfBound(NewIdentityPart);
		}
		else
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("Error creating MetaHuman Identity Part '%s'"), *InIdentityPartClass->GetName());
		}
	}
	else
	{
		UE_LOG(LogMetaHumanIdentity, Error, TEXT("Trying to add a Part that the MetaHuman Identity already has: '%s'"), *InIdentityPartClass->GetName());
	}
}

void SMetaHumanIdentityPartsEditor::HandleAddIdentityPoseOfClass(TSubclassOf<UMetaHumanIdentityPose> InIdentityPose, EIdentityPoseType InPoseType)
{
	UMetaHumanIdentity* Identity = GetIdentity();

	if (Identity->CanAddPoseOfClass(InIdentityPose, InPoseType))
	{
		if (UMetaHumanIdentityFace* FacePart = Identity->FindPartOfClass<UMetaHumanIdentityFace>())
		{
			// At the moment all poses are related to the face, so add the pose directly there
			// TODO: Handle cases where poses can be added to other Parts, might need to check which Identity Part is selected in the tree view when
			// this gets called

			if (TSharedPtr<FIdentityTreeNode> FacePartNode = FindIdentityTreeNode(FacePart, GetIdentityRootNode()))
			{
				const FScopedTransaction Transaction(UMetaHumanIdentity::IdentityTransactionContext, LOCTEXT("AddIdentityPose", "Add Pose to the MetaHuman Identity"), Identity);

				FacePart->Modify();

				UMetaHumanIdentityPose* NewIdentityPose = NewObject<UMetaHumanIdentityPose>(FacePart, InIdentityPose, NAME_None, RF_Transactional);
				NewIdentityPose->PoseName = FText::FromString(FString::Format(TEXT("{0} Pose"), { UMetaHumanIdentityPose::PoseTypeAsString(InPoseType) }));
				NewIdentityPose->PoseType = InPoseType;
				NewIdentityPose->LoadDefaultTracker();

				NewIdentityPose->OnCaptureDataChanged().AddSP(this, &SMetaHumanIdentityPartsEditor::HandleIdentityPoseCaptureDataChanged, NewIdentityPose);

				FacePart->Poses.Add(NewIdentityPose);

				// Add the new pose to the face part in the tree view
				TSharedRef<FIdentityTreeNode> NewPoseNode = MakeShareable(new FIdentityTreeNode{ NewIdentityPose, IdentityPreviewActorInstance });

				FacePartNode->Children.Add(NewPoseNode);

				AddAllPreviewSceneComponentInstances(NewPoseNode);

				// Update the visibility of the preview scene component
				UpdateSceneComponentVisiblity();

				// Automatically select the newly created node in the tree view
				SelectAndExpandIdentityTreeNode(NewPoseNode);

				// Notify that a new pose was added to the given part
				OnIdentityPoseAddedDelegate.ExecuteIfBound(NewIdentityPose, FacePartNode->IdentityPart.Get());
			}
			else
			{
				UE_LOG(LogMetaHumanIdentity, Error, TEXT("Failed to find the Face node to add the new Pose to."));
			}
		}
		else
		{
			UE_LOG(LogMetaHumanIdentity, Error, TEXT("Trying to add a Pose that the MetaHuman Identity already has: '%s' of type '%s'"), *InIdentityPose->GetName(), *UMetaHumanIdentityPose::PoseTypeAsString(InPoseType));
		}
	}
}

TSharedRef<ITableRow> SMetaHumanIdentityPartsEditor::HandleIdentityTreeGenerateRow(TSharedRef<FIdentityTreeNode> InNode, const TSharedRef<STableViewBase>& InOwnerTable)
{
	return SNew(STableRow<TSharedRef<FIdentityTreeNode>>, InOwnerTable)
		.Content()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot()
			.VAlign(VAlign_Center)
			.AutoWidth()
			[
				SNew(SImage)
				.ColorAndOpacity(FSlateColor::UseForeground())
				.Image(InNode, &FIdentityTreeNode::GetDisplayIconBrush)
			]
			+ SHorizontalBox::Slot()
			[
				SNew(STextBlock)
				.Margin(4)
				.Text(InNode, &FIdentityTreeNode::GetDisplayText)
			]
		];
}

void SMetaHumanIdentityPartsEditor::HandleIdentityTreeGetChildren(TSharedRef<FIdentityTreeNode> InItem, TArray<TSharedRef<FIdentityTreeNode>>& OutChildren)
{
	for (const TSharedRef<FIdentityTreeNode>& Child : InItem->Children)
	{
		if (Child->bVisible)
		{
			OutChildren.Add(Child);
		}
	}
}

void SMetaHumanIdentityPartsEditor::HandleIdentityTreeSelectionChanged(TSharedPtr<FIdentityTreeNode> InItem, ESelectInfo::Type InSelectInfo)
{
	OnIdentityTreeSelectionChangedDelegate.ExecuteIfBound(InItem.IsValid() ? InItem->GetObject() : nullptr);

	TArray<USceneComponent*> VisibleComponents, VisibleComponentInstances;

	if (InItem.IsValid())
	{
		const bool bOnlyVisible = true;
		bool bInstances = false;
		FindAllPreviewSceneComponents(InItem.ToSharedRef(), VisibleComponents, bInstances, bOnlyVisible);

		bInstances = true;
		FindAllPreviewSceneComponents(InItem.ToSharedRef(), VisibleComponentInstances, bInstances, bOnlyVisible);
	}

	if (ViewportClient.IsValid())
	{
		ViewportClient->SetSelectedSceneComponents(VisibleComponents, VisibleComponentInstances);
	}

	UpdateViewportSelectionOutlines();
}

void SMetaHumanIdentityPartsEditor::HandleIdentityTreeExpanstionRecursive(TSharedRef<FIdentityTreeNode> InItem, bool bInShouldExpand)
{
	if (IdentityTreeWidget != nullptr)
	{
		IdentityTreeWidget->SetItemExpansion(InItem, bInShouldExpand);

		for (const TSharedRef<FIdentityTreeNode>& Child : InItem->Children)
		{
			HandleIdentityTreeExpanstionRecursive(Child, bInShouldExpand);
		}
	}
}

void SMetaHumanIdentityPartsEditor::HandleIdentityTreeDeleteSelectedNode()
{
	const TArray<TSharedRef<FIdentityTreeNode>> SelectedItems = IdentityTreeWidget->GetSelectedItems();

	if (SelectedItems.Num() == 1)
	{
		UMetaHumanIdentity* Identity = GetIdentity();
		TSharedRef<FIdentityTreeNode> IdentityNode = GetIdentityRootNode();

		const TSharedRef<FIdentityTreeNode>& Node = SelectedItems[0];
		if (Node->IdentityPart.IsValid())
		{
			UMetaHumanIdentityPart* IdentityPart = Node->IdentityPart.Get();

			const FScopedTransaction Transaction(UMetaHumanIdentity::IdentityTransactionContext, LOCTEXT("RemoveIdentityPart", "Remove Part from MetaHuman Identity"), Identity);
			Identity->Modify();

			// Remove the Part from the Identity and the tree view
			if (Identity->Parts.Remove(IdentityPart))
			{
				IdentityNode->Children.Remove(Node);

				// Remove all preview scene components from the node that was just removed
				RemoveAllPreviewSceneComponents(Node);

				OnIdentityPartRemovedDelegate.ExecuteIfBound(IdentityPart);
			}
			else
			{
				UE_LOG(LogMetaHumanIdentity, Error, TEXT("Failed to remove MetaHuman Identity Part '%s'"), *IdentityPart->GetPartName().ToString())
			}
		}
		else if (Node->IdentityPose.IsValid())
		{
			if (TSharedPtr<FIdentityTreeNode> FaceNode = FindIdentityPartNodeByClass(UMetaHumanIdentityFace::StaticClass(), GetIdentityRootNode()))
			{
				if (UMetaHumanIdentityFace* FacePart = Cast<UMetaHumanIdentityFace>(FaceNode->IdentityPart.Get()))
				{
					const FScopedTransaction Transaction(UMetaHumanIdentity::IdentityTransactionContext, LOCTEXT("RemoveIdentityPose", "Remove Pose from MetaHuman Identity"), Identity);
					FacePart->Modify();

					UMetaHumanIdentityPose* Pose = Node->IdentityPose.Get();

					// Remove the Pose from the Identity and the tree view
					if (FacePart->Poses.Remove(Pose))
					{
						FaceNode->Children.Remove(Node);

						// Remove all preview scene components from the node that was just removed
						RemoveAllPreviewSceneComponents(Node);

						OnIdentityPoseRemovedDelegate.ExecuteIfBound(Pose, FacePart);
					}
					else
					{
						UE_LOG(LogMetaHumanIdentity, Error, TEXT("Failed to remove MetaHuman Identity Pose '%s'"), *Pose->PoseName.ToString())
					}
				}
			}
		}

		IdentityTreeWidget->RequestTreeRefresh();
	}
}

TSharedPtr<SWidget> SMetaHumanIdentityPartsEditor::HandleIdentityTreeContextMenu()
{
	const TArray<TSharedRef<FIdentityTreeNode>> SelectedItems = IdentityTreeWidget->GetSelectedItems();

	if (SelectedItems.Num() == 1)
	{
		const bool bShouldCloseAfterMenuSelection = true;
		FMenuBuilder MenuBuilder{ bShouldCloseAfterMenuSelection, CommandList };

		MenuBuilder.BeginSection(TEXT("PartCommandsParts"), LOCTEXT("PartsCommandPartsSectionLabel", "Part Options"));
		{
			MenuBuilder.AddMenuEntry(FGenericCommands::Get().Delete);
		}
		MenuBuilder.EndSection();

		MenuBuilder.BeginSection(TEXT("PartCommandsView"), LOCTEXT("PartCommandsViewSectionLabel", "View Options"));
		{
			MenuBuilder.AddMenuEntry(FEditorViewportCommands::Get().FocusViewportToSelection);
		}
		MenuBuilder.EndSection();

		return MenuBuilder.MakeWidget();
	}

	return TSharedPtr<SWidget>{};
}

void SMetaHumanIdentityPartsEditor::HandleUndoOrRedoTransaction(const FTransaction* InTransaction)
{
	if (InTransaction != nullptr)
	{
		if (InTransaction->GetPrimaryObject() == IdentityPtr)
		{
			// Something happened to the Identity so react by rebuilding the tree hierarchy and the viewport components
			RefreshWidget();
		}
		else
		{
			// Something happened to the objects we are editing so iterate over the changes recorded in
			// the transaction to make sure the instances being displayed in the viewport are in sync
			// with what's changed

			TArray<UObject*> AffectedObjects;
			InTransaction->GetTransactionObjects(AffectedObjects);

			const FTransactionDiff Diff = InTransaction->GenerateDiff();
			for (const TPair<FName, TSharedPtr<FTransactionObjectEvent>>& DiffMapPair : Diff.DiffMap)
			{
				const FName& ObjectName = DiffMapPair.Key;
				const TSharedPtr<FTransactionObjectEvent>& TransactionObjectEvent = DiffMapPair.Value;

				if (TransactionObjectEvent->HasPropertyChanges())
				{
					const int32 ObjectIndex = AffectedObjects.IndexOfByPredicate([ObjectName](const UObject* InObject)
					{
						return InObject != nullptr && InObject->GetPathName() == ObjectName.ToString();
					});

					if (ObjectIndex != INDEX_NONE)
					{
						UObject* AffectedObject = AffectedObjects[ObjectIndex];

						// Find the node in the tree view that holds the object that was affected
						if (TSharedPtr<FIdentityTreeNode> Node = FindIdentityTreeNode(AffectedObject, GetIdentityRootNode()))
						{
							// Checks if the affected object is the preview scene component
							if (Node->PreviewSceneComponent.IsValid() && Node->PreviewSceneComponentInstance.IsValid() &&
								Node->PreviewSceneComponent == AffectedObject)
							{
								// Finally iterate over all the properties that changed and update the value in the
								// scene component instance
								for (const FName& PropertyNameThatChanged : TransactionObjectEvent->GetChangedProperties())
								{
									FProperty* PropertyThatChanged = FindFProperty<FProperty>(AffectedObject->GetClass(), PropertyNameThatChanged);
									Node->UpdateSceneComponentInstanceProperty(PropertyThatChanged);
								}
							}
						}
					}
				}
			}
		}
	}
}

void SMetaHumanIdentityPartsEditor::HandleSceneComponentClicked(const USceneComponent* InSceneComponent)
{
	if (InSceneComponent != nullptr)
	{
		if (TSharedPtr<FIdentityTreeNode> Node = FindIdentityTreeNode(InSceneComponent, GetIdentityRootNode()))
		{
			SelectAndExpandIdentityTreeNode(Node.ToSharedRef());
		}
	}
}

void SMetaHumanIdentityPartsEditor::ClearIdentityTreeFilter(TSharedRef<FIdentityTreeNode> InNode)
{
	InNode->bVisible = true;

	for (TSharedRef<FIdentityTreeNode> Child : InNode->Children)
	{
		ClearIdentityTreeFilter(Child);
	}
}

bool SMetaHumanIdentityPartsEditor::FilterIdentityTree(TSharedRef<FIdentityTreeNode> InNode, const FString& InFilterString)
{
	// Set the state of the current node based on the filter string
	if (InNode->GetDisplayText().ToString().Contains(InFilterString))
	{
		InNode->bVisible = true;
		SelectAndExpandIdentityTreeNode(InNode);
	}
	else
	{
		InNode->bVisible = false;
	}

	// If any child of this node is visible, set this node to be visible as well
	for (TSharedRef<FIdentityTreeNode> Child : InNode->Children)
	{
		InNode->bVisible |= FilterIdentityTree(Child, InFilterString);
	}

	return InNode->bVisible;
}

void SMetaHumanIdentityPartsEditor::HandleIdentityFilterTextChanged(const FText& InFilterText)
{
	TSharedRef<FIdentityTreeNode> RootNode = GetIdentityRootNode();
	ClearIdentityTreeFilter(RootNode);

	if (!InFilterText.IsEmpty())
	{
		const FString FilterString = FText::TrimPrecedingAndTrailing(InFilterText).ToString();

		for (TSharedRef<FIdentityTreeNode> Child : RootNode->Children)
		{
			FilterIdentityTree(Child, FilterString);
		}
	}

	IdentityTreeWidget->RequestTreeRefresh();
}

void SMetaHumanIdentityPartsEditor::SelectAndExpandIdentityTreeNode(TSharedRef<FIdentityTreeNode> InNode)
{
	// Expand the root and the new node so they are visible in the tree view
	IdentityTreeWidget->SetItemExpansion(GetIdentityRootNode(), true);
	IdentityTreeWidget->SetItemExpansion(InNode, true);

	// Finally select the new node automatically
	IdentityTreeWidget->SetSelection(InNode);
}

bool SMetaHumanIdentityPartsEditor::CanAddIdentityPartOfClass(TSubclassOf<class UMetaHumanIdentityPart> InIdentityPartClass) const
{
	return GetIdentity()->CanAddPartOfClass(InIdentityPartClass);
}

bool SMetaHumanIdentityPartsEditor::CanAddIdentityPoseOfClass(TSubclassOf<class UMetaHumanIdentityPose> InIdentityPoseClass, EIdentityPoseType InPoseType) const
{
	return GetIdentity()->CanAddPoseOfClass(InIdentityPoseClass, InPoseType);
}

bool SMetaHumanIdentityPartsEditor::CanDeleteSelectedIdentityTreeNode() const
{
	const TArray<TSharedRef<FIdentityTreeNode>> SelectedItems = IdentityTreeWidget->GetSelectedItems();

	if (SelectedItems.Num() == 1)
	{
		const TSharedRef<FIdentityTreeNode>& Node = SelectedItems[0];
		return Node->CanDelete();
	}

	return false;
}

bool SMetaHumanIdentityPartsEditor::CanFocusToSelection() const
{
	const TArray<TSharedRef<FIdentityTreeNode>> SelectedItems = IdentityTreeWidget->GetSelectedItems();

	if (SelectedItems.Num() == 1 && ViewportClient.IsValid())
	{
		const bool bInstances = true;
		const bool bOnlyVisible = true;
		TArray<USceneComponent*> VisibleComponents;
		FindAllPreviewSceneComponents(SelectedItems[0], VisibleComponents, bInstances, bOnlyVisible);

		if (!VisibleComponents.IsEmpty())
		{
			const bool bIsWorldValidForAllVisibleComponents = Algo::AllOf(VisibleComponents, [](USceneComponent* Component)
			{
				return Component->GetWorld() != nullptr;
			});

			return bIsWorldValidForAllVisibleComponents;
		}
	}

	return false;
}

bool SMetaHumanIdentityPartsEditor::IsIdentityTreeValid() const
{
	return !RootNodes.IsEmpty();
}

void SMetaHumanIdentityPartsEditor::HandleIdentityPoseCaptureDataChanged(UMetaHumanIdentityPose* InIdentityPose)
{
	if (TSharedPtr<FIdentityTreeNode> PoseNode = FindIdentityTreeNode(InIdentityPose, GetIdentityRootNode()))
	{
		FPreviewScene* PreviewScene = GetPreviewScene();

		if (PoseNode->PreviewSceneComponentInstance.IsValid())
		{
			// Remove it from the scene
			PreviewScene->RemoveComponent(PoseNode->PreviewSceneComponentInstance.Get());

			PoseNode->PreviewSceneComponentInstance = nullptr;
			PoseNode->PreviewSceneComponent = nullptr;
		}

		if (USceneComponent* NewPreviewComponent = InIdentityPose->CaptureDataSceneComponent)
		{
			// Update the Pose node with information from the new capture data
			PoseNode->SceneComponentIdentifier = PoseNode->IdentityPose->PoseType == EIdentityPoseType::Neutral ? ESceneComponentIdentifier::ImportedMesh : ESceneComponentIdentifier::None;
			PoseNode->PreviewSceneComponent = NewPreviewComponent;
			PoseNode->SetupPreviewSceneComponentInstance(IdentityPreviewActorInstance);

			check(PoseNode->PreviewSceneComponentInstance.IsValid());

			IdentityPreviewActorInstance->AddOwnedComponent(PoseNode->PreviewSceneComponentInstance.Get());

			// Add the new one to the scene and store a reference to it in the tree node
			PreviewScene->AddComponent(PoseNode->PreviewSceneComponentInstance.Get(), PoseNode->PreviewSceneComponentInstance->GetComponentTransform());

			// Update the preview scene components visibility to make sure the new capture data visibility state is reflected in the viewport
			UpdateSceneComponentVisiblity();

			if (IdentityTreeWidget->IsItemSelected(PoseNode.ToSharedRef()) && ViewportClient.IsValid())
			{
				ViewportClient->SetSelectedSceneComponents({ PoseNode->PreviewSceneComponent.Get() }, { PoseNode->PreviewSceneComponentInstance.Get() });
				ViewportClient->FocusViewportOnSelectedComponents();
			}
		}
	}
}

void SMetaHumanIdentityPartsEditor::HandleFocusToSelection()
{
	const TArray<TSharedRef<FIdentityTreeNode>> SelectedItems = IdentityTreeWidget->GetSelectedItems();

	if (SelectedItems.Num() == 1 && ViewportClient.IsValid())
	{
		ViewportClient->FocusViewportOnSelectedComponents();
	}
}

TSharedRef<FIdentityTreeNode> SMetaHumanIdentityPartsEditor::GetIdentityRootNode() const
{
	check(IsIdentityTreeValid());
	return RootNodes[0];
}

UMetaHumanIdentity* SMetaHumanIdentityPartsEditor::GetIdentity() const
{
	check(IdentityPtr.IsValid());
	return IdentityPtr.Get();
}

FPreviewScene* SMetaHumanIdentityPartsEditor::GetPreviewScene() const
{
	check(ViewportClient.IsValid());
	return ViewportClient->GetPreviewScene();
}

TSharedPtr<FIdentityTreeNode> SMetaHumanIdentityPartsEditor::FindIdentityPartNodeByClass(TSubclassOf<UMetaHumanIdentityPart> InIdentityPart, const TSharedRef<FIdentityTreeNode>& InNode) const
{
	if (InNode->IdentityPart.IsValid() && InNode->IdentityPart->IsA(InIdentityPart))
	{
		return InNode;
	}
	else
	{
		// Look on all the children
		for (const TSharedRef<FIdentityTreeNode>& ChildNode : InNode->Children)
		{
			if (TSharedPtr<FIdentityTreeNode> FoundNode = FindIdentityPartNodeByClass(InIdentityPart, ChildNode))
			{
				return FoundNode;
			}
		}
	}

	return nullptr;
}

TSharedPtr<FIdentityTreeNode> SMetaHumanIdentityPartsEditor::FindIdentityTreeNode(const UObject* InObject, const TSharedRef<FIdentityTreeNode>& InNode) const
{
	if (InNode->GetObject() == InObject || InNode->PreviewSceneComponentInstance == InObject || InNode->PreviewSceneComponent == InObject)
	{
		return InNode;
	}
	else
	{
		// Look on all the children
		for (const TSharedRef<FIdentityTreeNode>& ChildNode : InNode->Children)
		{
			if (TSharedPtr<FIdentityTreeNode> FoundNode = FindIdentityTreeNode(InObject, ChildNode))
			{
				return FoundNode;
			}
		}
	}

	return nullptr;
}

TSharedPtr<FIdentityTreeNode> SMetaHumanIdentityPartsEditor::FindIdentityNodeByComponentId(ESceneComponentIdentifier InComponenetIdentifier, const TSharedRef<FIdentityTreeNode>& InNode) const
{
	if (InNode->PreviewSceneComponent.IsValid() && InNode->SceneComponentIdentifier == InComponenetIdentifier)
	{
		return InNode;
	}
	else
	{
		for (const TSharedRef<FIdentityTreeNode>& ChildNode : InNode->Children)
		{
			if (TSharedPtr<FIdentityTreeNode> FoundNode = FindIdentityNodeByComponentId(InComponenetIdentifier, ChildNode))
			{
				return FoundNode;
			}
		}
	}

	return nullptr;
}

void SMetaHumanIdentityPartsEditor::UpdateSceneComponentVisiblity()
{
	if (ViewportClient.IsValid())
	{
		ViewportClient->UpdateSceneComponentVisibility();
	}
}



#undef LOCTEXT_NAMESPACE