// Copyright Epic Games, Inc. All Rights Reserved.

#include "CoreUtils.h"

namespace epic
{
namespace core
{
	namespace
	{
		void CalcAlphaComplement(int32 InX, int32 InY, const FBox& InPointBox, float& OutAlpha, float& OutAlphaComplement)
		{
			const FBox PixelBox = FBox(FVector(InX, InY, 0), FVector(InX + 1, InY + 1, 1));
			const FBox Overlap = InPointBox.Overlap(PixelBox);
			OutAlpha = Overlap.GetSize().X * Overlap.GetSize().Y;
			OutAlphaComplement = 1.0f - OutAlpha;
		}

		void CalcExtentsAndPointBox(const FVector2D& InPoint, float InHalfSize, int32& OutXMin, int32& OutXMax, int32& OutYMin, int32& OutYMax, FBox& OutPointBox)
		{
			OutXMin = int(InPoint.X - InHalfSize);
			OutXMax = int(InPoint.X + InHalfSize);
			OutYMin = int(InPoint.Y - InHalfSize);
			OutYMax = int(InPoint.Y + InHalfSize);

			OutPointBox = FBox(FVector(InPoint.X - InHalfSize, InPoint.Y - InHalfSize, 0), 
				FVector(InPoint.X + InHalfSize, InPoint.Y + InHalfSize, 1));
		}
	}

void BurnPointsIntoImage(const TArray<FVector2D>& InPoints, int32 InWidth, int32 InHeight, FColor* OutImgData, uint8 InRed, uint8 InGreen, uint8 InBlue, int32 InSize, bool bInUseAntiAliasing)
{
	const float R = InRed / 255.0f;
	const float G = InGreen / 255.0f;
	const float B = InBlue / 255.0f;

	const float HalfSize = InSize / 2.0f;

	for (const FVector2D& Point : InPoints)
	{
		FBox PointBox;
		int32 XMin, XMax, YMin, YMax;
		CalcExtentsAndPointBox(Point, HalfSize, XMin, XMax, YMin, YMax, PointBox);

		float AlphaComplement, Alpha;

		for (int32 Y = YMin; Y <= YMax && Y >= 0 && Y < InHeight; ++Y)
		{
			int32 Index = (Y * InWidth + XMin);

			if (bInUseAntiAliasing)
			{
				for (int32 X = XMin; X <= XMax && X >= 0 && X < InWidth; ++X)
				{
					CalcAlphaComplement(X, Y, PointBox, Alpha, AlphaComplement);

					OutImgData[Index] = FColor(((OutImgData[Index].R / 255.0f * AlphaComplement) + (R * Alpha)) * 255.0f,
						((OutImgData[Index].G / 255.0f * AlphaComplement) + (G * Alpha)) * 255.0f,
						((OutImgData[Index].B / 255.0f * AlphaComplement) + (B * Alpha)) * 255.0f);

					++Index;
				}
			}
			else
			{
				for (int32 X = XMin; X <= XMax && X >= 0 && X < InWidth; ++X)
				{
					OutImgData[Index] = FColor(InRed, InGreen, InBlue);
					++Index;
				}
			}
		}
	}
}


void BurnPointsIntoImage(const TArray<FVector2D>& InPoints, int32 InWidth, int32 InHeight, TArray<uint8>& InImgData, uint8 InRed, uint8 InGreen, uint8 InBlue, int32 InSize, bool bInUseAntiAliasing)
{
	const float R = InRed / 255.0f;
	const float G = InGreen / 255.0f;
	const float B = InBlue / 255.0f;

	const float HalfSize = InSize / 2.0f;

	for (const FVector2D& Point : InPoints)
	{
		FBox PointBox;
		int32 XMin, XMax, YMin, YMax;
		CalcExtentsAndPointBox(Point, HalfSize, XMin, XMax, YMin, YMax, PointBox);


		for (int32 Y = YMin; Y <= YMax && Y >= 0 && Y < InHeight; ++Y)
		{
			int32 Index = (Y * InWidth + XMin) * 4;

			if (bInUseAntiAliasing)
			{
				for (int32 X = XMin; X <= XMax && X >= 0 && X < InWidth; ++X)
				{
					float AlphaComplement, Alpha;
					CalcAlphaComplement(X, Y, PointBox, Alpha, AlphaComplement);

					InImgData[Index] = ((InImgData[Index] / 255.0f * AlphaComplement) + (B * Alpha)) * 255.0f;
					++Index;

					InImgData[Index] = ((InImgData[Index] / 255.0f * AlphaComplement) + (G * Alpha)) * 255.0f;
					++Index;

					InImgData[Index] = ((InImgData[Index] / 255.0f * AlphaComplement) + (R * Alpha)) * 255.0f;
					++Index;

					++Index;
				}
			}
			else
			{
				for (int32 X = XMin; X <= XMax && X >= 0 && X < InWidth; ++X)
				{
					InImgData[Index] = InBlue;
					++Index;
					InImgData[Index] = InGreen;
					++Index;
					InImgData[Index] = InRed;
					++Index;
					++Index;
				}
			}
		}
	}
}

}//namespace core
}//namespace epic
