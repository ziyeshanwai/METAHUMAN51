// Copyright Epic Games, Inc.All Rights Reserved.

#include "AutoRigServiceBinding.h"

UAutorigServiceBrowserBinding::UAutorigServiceBrowserBinding(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

void UAutorigServiceBrowserBinding::LoginSessionId(const FString& id)
{
	SessionId = id;
	(void)OnAutoRigServiceBindSessionDelegate.ExecuteIfBound();
}

void UAutorigServiceBrowserBinding::SetBindCallback(
	FOnAutoRigServiceBindSessionDelegate&& OnAutoRigServiceBindSessionDelegate_)
{
	OnAutoRigServiceBindSessionDelegate = MoveTemp(OnAutoRigServiceBindSessionDelegate_);
}
