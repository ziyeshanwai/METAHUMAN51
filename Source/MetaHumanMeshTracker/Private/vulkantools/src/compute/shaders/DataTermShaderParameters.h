// Copyright Epic Games, Inc. All Rights Reserved.

// This file is shared between HLSL and C++ so we need to include guard it to avoid multiple definitions
// of this struct. #pragma once is not supported to all shader compilers.
#ifndef DATATERMSHADERPARAMS_H
#define DATATERMSHADERPARAMS_H

struct DataTermShaderParams
{
    float zetaSquared;
    float epsilonSquared;
    float gamma2;
    float delta2;
    float alpha2;
    float omega;
    float regularizationSquared;
};

#endif
