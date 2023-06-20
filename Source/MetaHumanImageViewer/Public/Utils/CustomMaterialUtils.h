// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NameTypes.h"
#include "Misc/EngineVersionComparison.h"

#include "Materials/Material.h"
#include "Materials/MaterialExpressionCustom.h"
#include "Materials/MaterialInstanceDynamic.h"

class METAHUMANIMAGEVIEWER_API CustomMaterialUtils
{
public:

	static void SetupExpression(class UMaterialExpressionTextureObjectParameter* InExpression, const FName& InName, bool bInUseExternalSampler);
	static void SetupExpression(class UMaterialExpressionScalarParameter* InExpression, const FName& InName, bool bInUseExternalSampler);
	static void SetupExpression(class UMaterialExpression* InExpression, const FName& InName, bool bInUseExternalSampler);

	template<typename T>
	static void AddInput(const FName& InName, UMaterial* InMaterial, UMaterialExpressionCustom* InCustomNode, bool bInUseExternalSampler = false)
	{
		T* Expression = NewObject<T>(InMaterial);

		SetupExpression(Expression, InName, bInUseExternalSampler);

#if UE_VERSION_NEWER_THAN(5, 0, ENGINE_PATCH_VERSION)
		InMaterial->GetExpressionCollection().AddExpression(Expression);
#else
		InMaterial->Expressions.Add(Expression);
#endif

		FCustomInput CustomInput;
		CustomInput.InputName = InName;
		CustomInput.Input.Expression = Expression;
		InCustomNode->Inputs.Add(CustomInput);
	}
	
	// A material that can show the raw footage, a contour overlay, and depth data. Material parameters are:
	//		"Movie"			Texture		RGBA or depth texture
	//		"Contours"		Texture		RGBA texture which is overlaid on above
	//		"IsDepthData"	Scalar		If >0.5 movie represents depth
	//		"ShowDarken"		Scalar		If >0.5 image is dimmed down
	//		"ShowContours"	Scalar		If >0.5 overlay is applied
	//		"DepthNear"		Scalar		Mininum visible depth value
	//		"DepthFar"		Scalar		Maximum visible depth value

	static UMaterialInstanceDynamic* CreateMovieContourDepthMaterial(const FName& InName, bool bInUseExternalSampler, int32 InDepthComponent);
};
