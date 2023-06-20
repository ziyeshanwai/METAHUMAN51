// Copyright Epic Games, Inc. All Rights Reserved.

struct ExtractChannelUint8PushConstants {
    int width;
    int height;
    int numChannels;
    int channel;
    int isSRGB;
    float scale;
};

static const int ExtractChannelUint8ComputeThreadSizeX = 32;
static const int ExtractChannelUint8ComputeThreadSizeY = 32;
