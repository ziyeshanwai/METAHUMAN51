// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Widgets/SCompoundWidget.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/SWindow.h"
#include "CoreMinimal.h"

class SScrollBox;
class SRichTextBlock;
class SCheckBox;
class SButton;

class AUTORIGSERVICE_API SMetaHumanEulaDialog : public SCompoundWidget
{
public:

	SLATE_BEGIN_ARGS(SMetaHumanEulaDialog) :
		_ParentWindow(),
		_EulaText()
	{}

		/** A pointer to the parent window */
		SLATE_ATTRIBUTE(TSharedPtr<SWindow>, ParentWindow)
		/** Text to display in the EULA box */
		SLATE_ATTRIBUTE(FText, EulaText)

	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	virtual void Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime) override;

	bool WasEulaAccepted() const;

private:

	void OnAcceptCheckboxToggled(ECheckBoxState CheckState);
	FReply OnAcceptButtonClicked();

private:

	/** Pointer to the parent window, so we know to destroy it when done */
	TWeakPtr<SWindow> ParentWindowPtr;

	TSharedPtr<SScrollBox> ScrollBox;
	TSharedPtr<SRichTextBlock> RichTextBox;
	TSharedPtr<SCheckBox> AcceptCheckbox;
	TSharedPtr<SButton> AcceptButton;
	FText EulaText;

	bool bUserClickedAccept;
	bool bFirstTick;
};