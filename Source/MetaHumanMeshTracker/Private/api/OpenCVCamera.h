// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <stdint.h>

namespace titan {
namespace api {

struct OpenCVCamera {
    int32_t width;
    int32_t height;
    float fx;
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;
    float k3;
    float p1;
    float p2;
    //! Transform from world coordinates to camera coordinates in column-major format.
    float Extrinsics[16];
};

} // namespace api
} // namespace titan
