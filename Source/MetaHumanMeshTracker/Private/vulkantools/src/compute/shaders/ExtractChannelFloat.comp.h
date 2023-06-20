// Copyright Epic Games, Inc. All Rights Reserved.

struct ExtractChannelFloatPushConstants {
    int width;
    int height;
    int numChannels;
    int channel;
    int isSRGB;
    float scale;
};

static const int ExtractChannelFloatComputeThreadSizeX = 32;
static const int ExtractChannelFloatComputeThreadSizeY = 32;
