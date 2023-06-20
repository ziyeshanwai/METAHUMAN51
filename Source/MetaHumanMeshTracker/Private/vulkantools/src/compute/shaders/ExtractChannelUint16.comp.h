// Copyright Epic Games, Inc. All Rights Reserved.

struct ExtractChannelUint16PushConstants {
    int width;
    int height;
    int numChannels;
    int channel;
    int isSRGB;
    float scale;
};

static const int ExtractChannelUint16ComputeThreadSizeX = 32;
static const int ExtractChannelUint16ComputeThreadSizeY = 32;
