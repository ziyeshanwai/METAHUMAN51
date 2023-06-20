// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

namespace epic::nls
{
    struct DataTermParams
    {
        float zetaSquared;
        float epsilonSquared;
        float gamma2;
        float delta2;
        float alpha2;
        float omega;
        float regularizationSquared;
    };
}
