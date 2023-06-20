// Copyright Epic Games, Inc.All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "ObjectTools.h"
#include "AutoRigServiceBinding.generated.h"

UCLASS()
class UAutorigServiceBrowserBinding : public UObject
{
	GENERATED_UCLASS_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "Miscellaneous")
	void LoginSessionId(const FString& id);

	const FString& GetSessionId() const 
	{
		return SessionId;
	}

	// delegate to invoke when the binding is complete
	// at the point when this delegate is invoked we know that we have a valid logged-in session 
	DECLARE_DELEGATE(FOnAutoRigServiceBindSessionDelegate);
	void SetBindCallback(FOnAutoRigServiceBindSessionDelegate&&  OnAutoRigServiceBindSessionDelegate);

private:
	FOnAutoRigServiceBindSessionDelegate OnAutoRigServiceBindSessionDelegate;
	FString SessionId;
};
