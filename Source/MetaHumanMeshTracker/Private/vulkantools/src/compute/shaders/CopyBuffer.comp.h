// Copyright Epic Games, Inc. All Rights Reserved.

struct CopyBufferPushConstants
{
	int inputWidth;
	int inputHeight;
	int outputWidth;
};

static const int CopyBufferThreadSizeX = 8;
static const int CopyBufferThreadSizeY = 8;
