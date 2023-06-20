// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

namespace epic::nls
{
    enum class InterpolationMode
    {
        Nearest = 0,
        Linear = 1,
        Cubic = 2
    };

    enum class Direction
    {
        Horizontal = 0,
        Vertical = 1
    };

    enum class Borders
    {
        LeftRight = 0,
        TopBottom = 1
    };
}
