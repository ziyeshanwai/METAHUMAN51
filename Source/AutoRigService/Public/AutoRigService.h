// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

namespace AutoRigService
{
	enum class ESolveRequestResult
	{
		Ok,
		Busy,
		Unauthorized,
		EulaNotAccepted,
		InvalidArguments,
		ServerError,
		LoginFailed
	};
	// Invoked on a successful solve with DNA file contents
	// NOTE: if something has failed this could be empty and the delegate must handle it gracefully
	DECLARE_DELEGATE_OneParam(FAutoRigServiceSolveCompleteDelegate, const TArray<uint8>&);
	// Invoked if the request to solve fails at any point
	// @param ResponseCode HTTP response code from server
	DECLARE_DELEGATE_OneParam(FAutoRigServiceSolveFailureDelegate, ESolveRequestResult Result);
	/**
	 * @brief Parameter structure to for autorig target solve 
	 */
	struct TargetSolveParameters
	{
		FString IdentityName;
		// default
		int32 Height = 0;
		// default
		int32 BodyTypeIndex = 2;
	};
	/**
	* @brief issues a solve request to the AR backend service for the XYZ data  format data passed in.
	* NOTE: THIS IS NOT THREAD SAFE.
	* @param InVertices XYZ input target mesh
	* @param InSolveParameters Parameters for the solve
	* @param OnAutoRigServiceSolveCompleteDelegate callback on success
	* @param OnAutoRigServiceSolveFailureDelegate callback on failure
	*/
	AUTORIGSERVICE_API void SolveForTarget(TArray<FVector3d>&& InVertices,
		const TargetSolveParameters & InSolveParameters,
		FAutoRigServiceSolveCompleteDelegate&& OnAutoRigServiceSolveCompleteDelegate,
		FAutoRigServiceSolveFailureDelegate&& OnAutoRigServiceSolveFailureDelegate);
	/**
	 * @brief check if a user is logged in 
	 * @return true if logged in
	 */
	AUTORIGSERVICE_API bool IsLoggedIn();
	DECLARE_DELEGATE(FAutoRigServiceLoginCompleteDelegate);
	/**
	 * @brief log the user in to the AutoRigService
	 * @param OnAutoRigServiceLoginCompleteDelegate callback on successful log-in
	 */
	AUTORIGSERVICE_API void Login(FAutoRigServiceLoginCompleteDelegate&& OnAutoRigServiceLoginCompleteDelegate);
}
