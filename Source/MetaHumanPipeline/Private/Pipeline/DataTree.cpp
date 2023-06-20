// Copyright Epic Games, Inc. All Rights Reserved.

#include "Pipeline/DataTree.h"

namespace UE::MetaHuman::Pipeline
{

TSharedPtr<FDataTree> FDataTree::GetChild(const FString& InName) const
{
	TSharedPtr<FDataTree> Child = nullptr;

	FString Left, Right;
	if (InName.Split(".", &Left, &Right))
	{
		if (Children.Contains(Left))
		{
			Child = Children[Left]->GetChild(Right);
		}
	}
	else
	{
		if (Children.Contains(InName))
		{
			Child = Children[InName];
		}
	}

	return Child;
}

TSharedPtr<FDataTree> FDataTree::GetChildOrCreate(const FString& InName)
{
	TSharedPtr<FDataTree> Child = nullptr;

	FString Left, Right;
	if (InName.Split(".", &Left, &Right))
	{
		if (Children.Contains(Left))
		{
			Child = Children[Left]->GetChildOrCreate(Right);
		}
		else
		{
			Children.Add(Left, MakeShared<FDataTree>());
			Child = Children[Left]->GetChildOrCreate(Right);
		}
	}
	else
	{
		if (Children.Contains(InName))
		{
			Child = Children[InName];
		}
		else
		{
			Child = MakeShared<FDataTree>();
			Children.Add(InName, Child);
		}
	}

	return Child;
}

void FDataTree::MoveTo(TSharedPtr<FDataTree> InOther)
{
	FScopeLock ScopeLock(&Mutex);

	InOther->Data = MoveTemp(Data);
	InOther->Children = MoveTemp(Children);
}

FString FDataTree::ToString(int32 InDepth) const
{
	FScopeLock ScopeLock(&Mutex);

	FString Message;

	if (InDepth == 0)
	{
		Message += LINE_TERMINATOR;
		Message += LINE_TERMINATOR;
		Message += "--------------------";
		Message += LINE_TERMINATOR;
	}

	for (const auto &Child : Children)
	{
		for (int32 Index = 0; Index < InDepth; ++Index)
		{
			Message += " ";
		}

		Message += Child.Key + LINE_TERMINATOR;;

		Message += Child.Value->ToString(InDepth + 1);
	}

	if (InDepth == 0)
	{
		Message += "--------------------";
		Message += LINE_TERMINATOR;
		Message += LINE_TERMINATOR;
	}

	return Message;
}

}
