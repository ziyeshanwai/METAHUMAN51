// Copyright Epic Games, Inc. All Rights Reserved.

struct WeightedAveragePushConstants
{
    int width;
    int height;
    float weightA;
    float weightB;
    float delta;
};

static const int WeightedAverageComputeThreadSizeX = 32;
static const int WeightedAverageComputeThreadSizeY = 32;
