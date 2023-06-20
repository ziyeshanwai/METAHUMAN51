// Copyright Epic Games, Inc. All Rights Reserved.

struct FlowConfidencePushConstants
{
    int width;
    int height;
    int interpolationMode;
};

static const int FlowConfidenceThreadSizeX = 32;
static const int FlowConfidenceThreadSizeY = 32;
