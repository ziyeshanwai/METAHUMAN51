// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanEulaDialog.h"

#include "Misc/Paths.h"
#include "HAL/PlatformProcess.h"
#include "Misc/FileHelper.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Text/SRichTextBlock.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SHyperlink.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "EditorStyleSet.h"
#include "Framework/Text/SlateTextRun.h"
#include "Framework/Text/SlateHyperlinkRun.h"
#include "Styling/CoreStyle.h"

#define LOCTEXT_NAMESPACE "MetaHuman"


class FEulaHeaderDecorator : public ITextDecorator
{
public:

	static TSharedRef<FEulaHeaderDecorator> Create()
	{
		return MakeShareable(new FEulaHeaderDecorator());
	}

	bool Supports(const FTextRunParseResults& RunInfo, const FString& Text) const override
	{
		return RunInfo.Name == DecoratorName;
	}

	TSharedRef<ISlateRun> Create(const TSharedRef<class FTextLayout>& TextLayout, const FTextRunParseResults& RunParseResult, const FString& OriginalText, const TSharedRef<FString>& ModelText, const ISlateStyle* Style) override
	{
		FRunInfo RunInfo(RunParseResult.Name);
		for (const TPair<FString, FTextRange>& Pair : RunParseResult.MetaData)
		{
			RunInfo.MetaData.Add(Pair.Key, OriginalText.Mid(Pair.Value.BeginIndex, Pair.Value.EndIndex - Pair.Value.BeginIndex));
		}

		ModelText->Append(OriginalText.Mid(RunParseResult.ContentRange.BeginIndex, RunParseResult.ContentRange.Len()));

		return FSlateTextRun::Create(RunInfo, ModelText, TextStyle);
	}

private:

	FEulaHeaderDecorator()
		: DecoratorName(FString("EULA.Header"))
		, TextStyle(FAppStyle::Get().GetWidgetStyle<FTextBlockStyle>("Log.Normal"))
	{
		TextStyle.SetFont(FCoreStyle::GetDefaultFontStyle("Bold", 10));
	}

private:

	/** Name of this decorator */
	FString DecoratorName;

	/** Style of this decorator */
	FTextBlockStyle TextStyle;

};

void SMetaHumanEulaDialog::Construct(const FArguments& InArgs)
{
	bUserClickedAccept = false;
	bFirstTick = true;
	ParentWindowPtr = InArgs._ParentWindow.Get();

	// Load any EULA text that was supplied directly.
	EulaText = InArgs._EulaText.Get();

	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot()
		.Padding(FMargin(8))
		[
			SAssignNew(ScrollBox, SScrollBox)
			.Style(FAppStyle::Get(), "ScrollBox")

			+ SScrollBox::Slot()
			[
				SNew(SVerticalBox)

				+ SVerticalBox::Slot()
				.FillHeight(1.0f)
				[
					SAssignNew(RichTextBox, SRichTextBlock)
					.Text(EulaText)
					.AutoWrapText(true)
					.Justification(ETextJustify::Left)
					+ SRichTextBlock::Decorator(FEulaHeaderDecorator::Create())
					+ SRichTextBlock::HyperlinkDecorator(FString(), FSlateHyperlinkRun::FOnClick::CreateStatic([](const FSlateHyperlinkRun::FMetadata& Metadata) {
						const FString* UrlPtr = Metadata.Find(TEXT("href"));
						if (UrlPtr)
						{
							FPlatformProcess::LaunchURL(**UrlPtr, nullptr, nullptr);
						}
					}))
				]
			]
		]

		+ SVerticalBox::Slot()
		.VAlign(VAlign_Bottom)
		.HAlign(HAlign_Center)
		.Padding(0, 0, 0, 8)
		.AutoHeight()
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot()
			.Padding(20, 5, 20, 5)
			.AutoHeight()
			[
				SAssignNew(AcceptCheckbox, SCheckBox)
				.IsEnabled(false)
				.OnCheckStateChanged(this, &SMetaHumanEulaDialog::OnAcceptCheckboxToggled)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("MHEula_AcceptCheckbox", "I have read and agree to the terms and conditions"))
					.ColorAndOpacity(FSlateColor::UseForeground())
				]
			]

			+ SVerticalBox::Slot()
			.Padding(20, 5, 20, 5)
			.AutoHeight()
			.HAlign(HAlign_Center)
			[
				SAssignNew(AcceptButton, SButton)
				.IsEnabled(false)
				.OnClicked(this, &SMetaHumanEulaDialog::OnAcceptButtonClicked)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("MHEula_Accept", "Accept"))
					.ColorAndOpacity(FSlateColor::UseForeground())
				]
			]
		]
	];
}

void SMetaHumanEulaDialog::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime)
{
	const float EulaWindowEndThreshold = 32.f;

	// Prevent enabling checkbox early when the EULA text hasn't been initialized yet during the first tick.
	if (bFirstTick)
	{
		bFirstTick = false;
	}
	else
	{
		// Only enable the Accept checkbox once the user has scrolled to the bottom.
		if (ScrollBox.IsValid() && AcceptCheckbox.IsValid())
		{
			const float DistanceFromEnd = ScrollBox.Get()->GetScrollOffsetOfEnd() - ScrollBox.Get()->GetScrollOffset();

			if (DistanceFromEnd < EulaWindowEndThreshold)
			{
				AcceptCheckbox.Get()->SetEnabled(true);
			}
		}
	}
}

bool SMetaHumanEulaDialog::WasEulaAccepted() const
{
	return bUserClickedAccept;
}

void SMetaHumanEulaDialog::OnAcceptCheckboxToggled(ECheckBoxState CheckState)
{
	if (AcceptButton.IsValid())
	{
		AcceptButton.Get()->SetEnabled(CheckState == ECheckBoxState::Checked);
	}
}

FReply SMetaHumanEulaDialog::OnAcceptButtonClicked()
{
	bUserClickedAccept = true;

	ParentWindowPtr.Pin()->RequestDestroyWindow();
	return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE