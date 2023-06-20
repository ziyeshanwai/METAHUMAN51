// Copyright Epic Games, Inc. All Rights Reserved.

#include "CustomMaterialUtils.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Factories/MaterialFactoryNew.h"
#include "ComponentReregisterContext.h"

#include "Materials/MaterialExpressionTextureObjectParameter.h"
#include "Materials/MaterialExpressionTextureCoordinate.h"
#include "Materials/MaterialExpressionScalarParameter.h"

#include "Misc/EngineVersionComparison.h"


void CustomMaterialUtils::SetupExpression(UMaterialExpressionTextureObjectParameter* InExpression, const FName& InName, bool bInUseExternalSampler)
{
	InExpression->ParameterName = InName;
	InExpression->SetDefaultTexture();
	if (bInUseExternalSampler)
	{
		InExpression->SamplerType = EMaterialSamplerType::SAMPLERTYPE_External;
	}
}

void CustomMaterialUtils::SetupExpression(UMaterialExpressionScalarParameter* InExpression, const FName& InName, bool bInUseExternalSampler)
{
	InExpression->ParameterName = InName;
	InExpression->DefaultValue = 0.0;
}

void CustomMaterialUtils::SetupExpression(UMaterialExpression* InExpression, const FName& InName, bool bInUseExternalSampler)
{
}

UMaterialInstanceDynamic* CustomMaterialUtils::CreateMovieContourDepthMaterial(const FName &InName, bool bInUseExternalSampler, int32 InDepthComponent)
{
	UMaterialFactoryNew* MaterialFactory = NewObject<UMaterialFactoryNew>();

	// Material for clip
	UMaterial* ClipMaterial = Cast<UMaterial>(MaterialFactory->FactoryCreateNew(UMaterial::StaticClass(), GetTransientPackage(), InName, RF_Standalone | RF_Public, NULL, GWarn));
	FAssetRegistryModule::AssetCreated(ClipMaterial);

	UMaterialExpressionCustom* ClipCustomNode = NewObject<UMaterialExpressionCustom>(ClipMaterial);

	CustomMaterialUtils::AddInput<UMaterialExpressionTextureObjectParameter>("Movie", ClipMaterial, ClipCustomNode, bInUseExternalSampler);
	CustomMaterialUtils::AddInput<UMaterialExpressionTextureObjectParameter>("Contours", ClipMaterial, ClipCustomNode);
	CustomMaterialUtils::AddInput<UMaterialExpressionTextureCoordinate>("TexCoord", ClipMaterial, ClipCustomNode);
	CustomMaterialUtils::AddInput<UMaterialExpressionScalarParameter>("IsDepthData", ClipMaterial, ClipCustomNode);
	CustomMaterialUtils::AddInput<UMaterialExpressionScalarParameter>("ShowDarken", ClipMaterial, ClipCustomNode);
	CustomMaterialUtils::AddInput<UMaterialExpressionScalarParameter>("ShowContours", ClipMaterial, ClipCustomNode);
	CustomMaterialUtils::AddInput<UMaterialExpressionScalarParameter>("DepthNear", ClipMaterial, ClipCustomNode);
	CustomMaterialUtils::AddInput<UMaterialExpressionScalarParameter>("DepthFar", ClipMaterial, ClipCustomNode);
	CustomMaterialUtils::AddInput<UMaterialExpressionScalarParameter>("DepthComponent", ClipMaterial, ClipCustomNode);

	const FString ClipCode = R"|(

// UV coords for nearest neightbour sampling - reduces artifacts
float2 Resolution;
Movie.GetDimensions(Resolution.x, Resolution.y);

float SampleX = int(TexCoord.x * Resolution.x) + 0.5;
float SampleY = int(TexCoord.y * Resolution.y) + 0.5;

float2 UV;
UV.x = SampleX / Resolution.x;
UV.y = SampleY / Resolution.y;

// Sample movie
float4 MovieSample = Movie.SampleLevel(MovieSampler, UV, 0);

if (IsDepthData > 0.5)
{
	// Convert floating point depth value between DepthNear and DepthFar into range 1-0
	float Value = 0.0;

	if (MovieSample[DepthComponent] > DepthNear && MovieSample[DepthComponent] < DepthFar)
	{
		Value = 1.0 - ((MovieSample[DepthComponent] - DepthNear) / (DepthFar - DepthNear));
	}

	// Use depth as greyscale value making linear changes in depth perceptually the same
	MovieSample[0] = pow(Value, 2.2);
	MovieSample[1] = MovieSample[0];
	MovieSample[2] = MovieSample[0];
}

if (ShowDarken > 0.5)
{
	MovieSample *= 0.1;
}

if (ShowContours > 0.5)
{
	float4 ContoursSample = Contours.SampleLevel(ContoursSampler, UV, 0);
	if (ContoursSample[3] > 0.5)
	{
		MovieSample[0] = ContoursSample[0];
		MovieSample[1] = ContoursSample[1];
		MovieSample[2] = ContoursSample[2];
	}
}

return MovieSample;

	)|";

	ClipCustomNode->Code = ClipCode;
	ClipMaterial->SetShadingModel(EMaterialShadingModel::MSM_Unlit);

#if UE_VERSION_NEWER_THAN(5, 0, ENGINE_PATCH_VERSION)
	UMaterialEditorOnlyData* ClipMaterialEditorOnly = ClipMaterial->GetEditorOnlyData();
	ClipMaterial->GetExpressionCollection().AddExpression(ClipCustomNode);
	ClipMaterialEditorOnly->EmissiveColor.Expression = ClipCustomNode;
#else
	ClipMaterial->Expressions.Add(ClipCustomNode);
	ClipMaterial->EmissiveColor.Expression = ClipCustomNode;
#endif

	ClipMaterial->MaterialDomain = EMaterialDomain::MD_UI;

	ClipMaterial->PreEditChange(nullptr);
	ClipMaterial->PostEditChange();

	UMaterialInstanceDynamic* ClipMaterialInstance = UMaterialInstanceDynamic::Create(ClipMaterial, nullptr);

	ClipMaterialInstance->SetScalarParameterValue(FName("DepthComponent"), InDepthComponent);

	FGlobalComponentReregisterContext RecreateComponents;

	return ClipMaterialInstance;
}