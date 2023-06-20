// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/STreeView.h"
#include "EditorUndoClient.h"

#include "UI/ConformingViewportClient.h"

/////////////////////////////////////////////////////
// FIdentityTreeNode

/** A node in the Identity Parts tree */
struct FIdentityTreeNode
{
	/** Builds a node from a Part */
	FIdentityTreeNode(class UMetaHumanIdentityPart* InIdentityPart, class AActor* InIdentityActor, FName InPropertyName = NAME_None, class USceneComponent* InPreviewComponent = nullptr, ESceneComponentIdentifier InIdentifier = ESceneComponentIdentifier::None);

	/** Builds a node from a Pose */
	FIdentityTreeNode(class UMetaHumanIdentityPose* InIdentityPose, class AActor* InIdentityActor, ESceneComponentIdentifier InIdentifier = ESceneComponentIdentifier::None);

	/** Builds the node hierarchy for a given Identity */
	FIdentityTreeNode(class UMetaHumanIdentity* InIdentity, class AActor* InIdentityActor);

	/** Creates the PreviewSceneComponentInstance which will be displayed in the viewport */
	void SetupPreviewSceneComponentInstance(class AActor* InIdentityActor);

	/** Updates the value of the given property in the PreviewSceneComponentInstance */
	void UpdateSceneComponentInstanceProperty(class FProperty* InProperty);

	/** Returns true if this node can be deleted from the tree */
	bool CanDelete() const;

	/** Returns the display text for this node based on the data it is holding */
	FText GetDisplayText() const;

	/** Returns the display icon for this node based on the data it is holding */
	const FSlateBrush* GetDisplayIconBrush() const;

	/** Returns a pointer to the valid object associated with this node. It can return nullptr if not directly associated with one */
	UObject* GetObject() const;

	/** Returns the FObjectProperty associated with this node. Only valid if this node points to a property of a Part, i.e, IdentityPartPropertyName is not none */
	class FObjectProperty* GetObjectProperty() const;

	/** A weak pointer to the Identity object associated with this node. This is valid iff this is the root node of the hierarchy */
	TWeakObjectPtr<class UMetaHumanIdentity> Identity;

	/** A weak pointer to the Identity Part associated with this node, if there is one */
	TWeakObjectPtr<class UMetaHumanIdentityPart> IdentityPart;

	/** The name of the Part property associated with this node, if there is one */
	FName IdentityPartPropertyName = NAME_None;

	/** A weak pointer to the Identity pose associated with this node, if there is one */
	TWeakObjectPtr<class UMetaHumanIdentityPose> IdentityPose;

	/** The children nodes */
	TArray<TSharedRef<FIdentityTreeNode>> Children;

	/** The preview component associated with this node, if there is one */
	TWeakObjectPtr<USceneComponent> PreviewSceneComponent;

	/** The instance of the preview scene component that is actually displayed in the viewport */
	TWeakObjectPtr<USceneComponent> PreviewSceneComponentInstance;

	/** An identifier for scene components for relevant tree nodes */
	ESceneComponentIdentifier SceneComponentIdentifier;

	/** If this node is visible in the tree. Note that an invisible node hides all its children even if they are tagged as visible */
	bool bVisible = true;
};

/////////////////////////////////////////////////////
// SMetaHumanIdentityPartsEditor

enum class EIdentityPoseType : uint8;

DECLARE_DELEGATE_OneParam(FIdentityPartAdded, class UMetaHumanIdentityPart* InIdentityPart)
DECLARE_DELEGATE_OneParam(FIdentityPartRemoved, class UMetaHumanIdentityPart* InIdentityPart)
DECLARE_DELEGATE_TwoParams(FIdentityPoseAdded, class UMetaHumanIdentityPose* InIdentityPose, class UMetaHumanIdentityPart* InIdentityPart)
DECLARE_DELEGATE_TwoParams(FIdentityPoseRemoved, class UMetaHumanIdentityPose* InIdentityPose, class UMetaHumanIdentityPart* InIdentityPart)
DECLARE_DELEGATE_OneParam(FIdentityTreeSelectionChanged, class UObject*)

/**
 * A widget that allows editing the Parts of a Identity.
 * It displays a button to add new Parts and a tree view with the hierarchy of Parts and Poses that form a Identity
 */
class SMetaHumanIdentityPartsEditor
	: public SCompoundWidget
	, public FSelfRegisteringEditorUndoClient
	, public FGCObject
{
public:
	SLATE_BEGIN_ARGS(SMetaHumanIdentityPartsEditor) {}

		// The Identity we are editing
		SLATE_ARGUMENT(class UMetaHumanIdentity*, Identity)

		// A reference to the viewport client where scene components can be displayed
		SLATE_ARGUMENT(TSharedPtr<class FConformingViewportClient>, ViewportClient)

		// Delegate called when a new Identity Part is added
		SLATE_EVENT(FIdentityPartAdded, OnIdentityPartAdded)

		// Delegated called when a Identity Part is removed
		SLATE_EVENT(FIdentityPartRemoved, OnIdentityPartRemoved)

		// Delegate called when a new Identity Pose is added
		SLATE_EVENT(FIdentityPoseAdded, OnIdentityPoseAdded)

		// Delegate called when a Identity Pose is removed
		SLATE_EVENT(FIdentityPoseRemoved, OnIdentityPoseRemoved)

		// Delegate called when a new Pose is added in the Identity
		SLATE_EVENT(FIdentityTreeSelectionChanged, OnIdentityTreeSelectionChanged)

	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	~SMetaHumanIdentityPartsEditor();

	// FEditorUndoClient Interface
	virtual void PostUndo(bool bInSuccess) override;
	virtual void PostRedo(bool bInSuccess) override;

	//~ FGCObject interface
	virtual void AddReferencedObjects(FReferenceCollector& InCollector) override;
	virtual FString GetReferencerName() const override;

	/** Creates a CaptureData a Face and a NeutralPose directly from the given Mesh, can be either a Static or SkeletalMesh */
	void AddFaceFromMesh(class UObject* InMesh);

	/** Returns the scene component of the given identifier or nullptr if there isn't one */
	class USceneComponent* GetSceneComponentOfType(ESceneComponentIdentifier InComponentIdentifier, bool bInInstance) const;

	/** Update the selection highlights in the viewport. */
	void UpdateViewportSelectionOutlines(bool bShowSelectionOutlines = true);

	/** Called whenever a property is edited in the details panel */
	void NotifyPostChange(const FPropertyChangedEvent& InPropertyChangedEvent, FProperty* InPropertyThatChanged);

protected:
	// SWidget Interface
	virtual FReply OnKeyDown(const FGeometry& InGeometry, const FKeyEvent& InKeyEvent) override;

private:
	/** Creates and binds the list of commands used in this widget */
	void BindCommands();

	/** Spawn the Identity preview actor that will be displayed in the viewport */
	void SpawnIdentityPreviewActor();

	/**
	 * Recursively gets all preview scene components currently store in a node and its descendants with the option to only returns the ones that are visible
	 *
	 * @param InNode The node to start the search from
	 * @param OutPreviewComponents The list of components matching the search criteria
	 * @param bInInstances Return the instances of the the scene components that are actually displayed in the viewport
	 * @param bInOnlyVisible Return only the components that are currently set to be visible
	 */
	void FindAllPreviewSceneComponents(const TSharedRef<FIdentityTreeNode>& InNode, TArray<USceneComponent*>& OutPreviewComponents, bool bInInstances, bool bInOnlyVisible) const;

	/** Adds all preview scene components from the given node and all its children */
	void AddAllPreviewSceneComponentInstances(const TSharedRef<FIdentityTreeNode>& InNode);

	/** Removes all preview scene components from the viewport */
	void RemoveAllPreviewSceneComponents(const TSharedRef<FIdentityTreeNode>& InNode);

	/** Rebuilds the tree view and updates the viewport accordingly */
	void RefreshWidget();

	/** Recreates the Identity hierarchy tree, useful when handling undo operations */
	void RefreshIdentityTree();

	/** Handles the addition of a new Identity Part of the specified class */
	void HandleAddIdentityPartOfClass(TSubclassOf<class UMetaHumanIdentityPart> InIdentityPartClass);

	/** Handles the addition of a new Identity Pose of the specified class */
	void HandleAddIdentityPoseOfClass(TSubclassOf<class UMetaHumanIdentityPose> InIdentityPoseClass, EIdentityPoseType InPoseType);

	/** Handles the generation of a row in the tree view */
	TSharedRef<class ITableRow> HandleIdentityTreeGenerateRow(TSharedRef<FIdentityTreeNode> InNode, const TSharedRef<class STableViewBase>& InOwnerTable);

	/** Returns the children of a given node in the three */
	void HandleIdentityTreeGetChildren(TSharedRef<FIdentityTreeNode> InItem, TArray<TSharedRef<FIdentityTreeNode>>& OutChildren);

	/** Handles selection events in the tree view */
	void HandleIdentityTreeSelectionChanged(TSharedPtr<FIdentityTreeNode> InItem, ESelectInfo::Type InSelectInfo);

	/** Handles recursive expansion on the tree widget when Shift + Click in a node */
	void HandleIdentityTreeExpanstionRecursive(TSharedRef<FIdentityTreeNode> InItem, bool bInShouldExpand);

	/** Handles a change in the text used to filter the tree view */
	void HandleIdentityFilterTextChanged(const FText& InFilterText);

	/** Handles deleting the selected node in the Identity tree view */
	void HandleIdentityTreeDeleteSelectedNode();

	/** Handles a change in the capture data used for the given pose */
	void HandleIdentityPoseCaptureDataChanged(class UMetaHumanIdentityPose* InIdentityPose);

	/** Handles focusing on the current selected item in the tree view */
	void HandleFocusToSelection();

	/** Creates the context menu for a node in the Identity tree view */
	TSharedPtr<SWidget> HandleIdentityTreeContextMenu();

	/** Handles an undo/redo transaction */
	void HandleUndoOrRedoTransaction(const class FTransaction* InTransaction);

	/** Handles a click in a scene component in the viewport */
	void HandleSceneComponentClicked(const class USceneComponent* InSceneComponent);

	/** Recursively set all nodes to be visible in the tree view. */
	void ClearIdentityTreeFilter(TSharedRef<FIdentityTreeNode> InNode);

	/** Recursively filter nodes from the tree view based on the given filter string */
	bool FilterIdentityTree(TSharedRef<FIdentityTreeNode> InNode, const FString& InFilterString);

	/** Select and expand to the given node in the tree view */
	void SelectAndExpandIdentityTreeNode(TSharedRef<FIdentityTreeNode> InNode);

	/** Returns true if a part of the given class can be added to the Identity being edited */
	bool CanAddIdentityPartOfClass(TSubclassOf<class UMetaHumanIdentityPart> InIdentityPartClass) const;

	/** Returns true if a pose of the given class and type can be added to the Identity being edited */
	bool CanAddIdentityPoseOfClass(TSubclassOf<class UMetaHumanIdentityPose> InIdentityPoseClass, EIdentityPoseType InPoseType) const;

	/** Returns true if the selected node in the Identity tree view can be deleted */
	bool CanDeleteSelectedIdentityTreeNode() const;

	/** Returns true if the current selected node in the tree view can focus on the viewport, i.e., if it has a scene component */
	bool CanFocusToSelection() const;

	/** Returns true if the Identity tree has its root node created */
	bool IsIdentityTreeValid() const;

	/** Returns the root node of in the tree view */
	TSharedRef<FIdentityTreeNode> GetIdentityRootNode() const;

	/** Returns a pointer to the Identity we are editing which is stored in the root node of the tree */
	class UMetaHumanIdentity* GetIdentity() const;

	/** Returns a pointer to the preview scene where scene components are being displayed */
	class FPreviewScene* GetPreviewScene() const;

	/** Recursively looks for a Identity Part of the given class */
	TSharedPtr<FIdentityTreeNode> FindIdentityPartNodeByClass(TSubclassOf<class UMetaHumanIdentityPart> InIdentityPart, const TSharedRef<FIdentityTreeNode>& InNode) const;

	/** Recursively looks for a node that holds the given UObject */
	TSharedPtr<FIdentityTreeNode> FindIdentityTreeNode(const UObject* InObject, const TSharedRef<FIdentityTreeNode>& InNode) const;

	/** Recursively looks for a node holding the given component identifier */
	TSharedPtr<FIdentityTreeNode> FindIdentityNodeByComponentId(ESceneComponentIdentifier InComponenetIdentifier, const TSharedRef<FIdentityTreeNode>& InNode) const;

	/** Update the scene component visibility for identity. Triggers an update on the viewport client */
	void UpdateSceneComponentVisiblity();

private:
	/** The actor representing the identity in the viewport. Used to attach components for rendering Identity Parts and Poses */
	TObjectPtr<class AActor> IdentityPreviewActorInstance;

	/** Reference to the Identity object we are editing */
	TWeakObjectPtr<class UMetaHumanIdentity> IdentityPtr;

	/** The viewport client of the preview scene */
	TSharedPtr<class FConformingViewportClient> ViewportClient;

	/** Command list for handling actions in the tree view */
	TSharedPtr<class FUICommandList> CommandList;

	/** A pointer to the Identity tree view */
	TSharedPtr<STreeView<TSharedRef<FIdentityTreeNode>>> IdentityTreeWidget;

	/** List of root nodes of the Identity tree. This will have a single element but an array is required for the tree view */
	TArray<TSharedRef<FIdentityTreeNode>> RootNodes;

	/** Delegate called when a new Identity Part was added to the Identity */
	FIdentityPartAdded OnIdentityPartAddedDelegate;

	/** Delegate called when a Identity Part was removed from the Identity in a given Part */
	FIdentityPartRemoved OnIdentityPartRemovedDelegate;

	/** Delegate called when a new Identity Pose was added to the Identity */
	FIdentityPoseAdded OnIdentityPoseAddedDelegate;

	/** Delegate called when a Identity pose was removed from the Identity in a given Part */
	FIdentityPoseRemoved OnIdentityPoseRemovedDelegate;

	/** Delegated called when a new node in the tree view is selected */
	FIdentityTreeSelectionChanged OnIdentityTreeSelectionChangedDelegate;
};