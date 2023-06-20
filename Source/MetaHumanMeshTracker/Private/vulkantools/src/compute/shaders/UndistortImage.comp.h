// Copyright Epic Games, Inc. All Rights Reserved.

struct UndistortImagePushConstants {
    float H[9];
    float fx;
    float cx;
    float fy;
    float cy;
    float K[4];
    float P[4];
    float B[2];
    int width;
    int height;
    int maxx;
    int maxy;
    int interpolationMethod;
};

static const int UndistortImageComputeThreadSizeX = 32;
static const int UndistortImageComputeThreadSizeY = 32;
