// Copyright Epic Games, Inc. All Rights Reserved.

#define UPDATE_BORDERS_TOP_BOTTOM 0
#define UPDATE_BORDERS_LEFT_RIGHT 1

struct UpdateRepeatedBordersPushConstants
{
    int width;
    int height;
    int numRedEven;
    int numRedOdd;
    int numBlackEven;
    int numBlackOdd;
    int borders;
};

static const int UpdateRepeatedBordersThreadSizeX = 32;
