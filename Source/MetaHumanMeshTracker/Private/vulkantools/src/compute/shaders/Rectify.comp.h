// Copyright Epic Games, Inc. All Rights Reserved.

struct RectifyPushConstants {
    float H[9];
    int width;
    int height;
    int maxx;
    int maxy;
    int interpolationMethod;
};

static const int RectifyComputeThreadSizeX = 32;
static const int RectifyComputeThreadSizeY = 32;
