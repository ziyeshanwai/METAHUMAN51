// Copyright Epic Games, Inc. All Rights Reserved.

struct SORPushConstants
{
    int isRedPass;
    int evenLen;
    int oddLen;
    int width;
    int height;
    float omega;
};

static const int SORThreadSizeX = 16;
static const int SORThreadSizeY = 16;
