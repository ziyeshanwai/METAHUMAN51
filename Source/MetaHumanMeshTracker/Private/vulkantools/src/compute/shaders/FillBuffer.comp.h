// Copyright Epic Games, Inc. All Rights Reserved.

struct FillBufferPushConstants
{
	int width;
	int height;
	float value;
};

static const int FillBufferThreadSizeX = 8;
static const int FillBufferThreadSizeY = 8;
