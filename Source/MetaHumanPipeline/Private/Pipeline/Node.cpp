// Copyright Epic Games, Inc. All Rights Reserved.

#include "Pipeline/Node.h"

namespace UE::MetaHuman::Pipeline
{

FNode::FNode(const FString& InTypeName, const FString& InName) : TypeName(InTypeName), Name(InName)
{
	static int32 NodeCount = 0;
	ID = FString::Printf(TEXT("Node%i"), NodeCount++);
}

FNode::~FNode()
{
}

FString FNode::ToString() const
{
	FString Message;

	Message += FString::Printf(TEXT(" ID [%s] Name [%s] Type [%s] Pins:"), *ID, *Name, *TypeName);
	Message += LINE_TERMINATOR;

	for (const FPin& Pin : Pins)
	{
		Message += Pin.ToString();
		Message += LINE_TERMINATOR;
	}

	return Message;
}

}
