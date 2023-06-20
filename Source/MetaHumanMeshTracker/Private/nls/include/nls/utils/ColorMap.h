// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic::nls {

template<class T>
class ColorSegments {

public:
    Eigen::Vector4<T> ValueToColor(const T value) {\
        Eigen::Vector4<T> color;
        color[0] = ValueToColor(value, pointsAndValuesRed);
        color[1] = ValueToColor(value, pointsAndValuesGreen);
        color[2] = ValueToColor(value, pointsAndValuesBlue);
        color[3] = T(1);
        return color;
    }

    static ColorSegments JetColorSegments()
    {
        ColorSegments colorSegments;
        colorSegments.pointsAndValuesRed = {{T(0), T(0)}, {T(0.35), T(0)}, {T(0.66), T(1)}, {T(0.89), T(1)}, {T(1), T(0.5)}};
        colorSegments.pointsAndValuesGreen = {{T(0), T(0)}, {T(0.125), T(0)}, {T(0.375), T(1)}, {T(0.64), T(1)}, {T(0.91), T(0)}, {T(1), T(0)}};
        colorSegments.pointsAndValuesBlue = {{T(0), T(0.5)}, {T(0.11), T(1)}, {T(0.34), T(1)}, {T(0.65), T(0)}, {T(1), T(0)}};
        return colorSegments;
    }

    static ColorSegments GrayColorSegments()
    {
        ColorSegments colorSegments;
        colorSegments.pointsAndValuesRed = {{T(0), T(0)}, {T(1), T(1.0)}};
        colorSegments.pointsAndValuesGreen = {{T(0), T(0)}, {T(1), T(1.0)}};
        colorSegments.pointsAndValuesBlue = {{T(0), T(0)}, {T(1), T(1.0)}};
        return colorSegments;
    }

    static ColorSegments RedBlueColorSegments()
    {
        ColorSegments colorSegments;
        colorSegments.pointsAndValuesRed = {{T(0), T(1)}, {T(0.5), T(1.0)}, {T(1), T(0.0)}};
        colorSegments.pointsAndValuesGreen = {{T(0), T(0)}, {T(0.5), T(1.0)}, {T(1), T(0.0)}};
        colorSegments.pointsAndValuesBlue = {{T(0), T(0)}, {T(0.5), T(1.0)}, {T(1), T(1.0)}};
        return colorSegments;
    }

private:
    T ValueToColor(const T value, const std::vector<std::pair<T,T>>& pointsAndValues) {
        if (value <= pointsAndValues.front().first) return pointsAndValues.front().second;
        for (size_t i = 1; i < pointsAndValues.size(); i++) {
            if (value < pointsAndValues[i].first) {
                const T start = pointsAndValues[i - 1].first;
                const T end = pointsAndValues[i].first;
                const T v0 = pointsAndValues[i - 1].second;
                const T v1 = pointsAndValues[i].second;
                const T w = (value - start) / (end - start);
                return (T(1) - w) * v0 + w * v1;
            }
        }
        return pointsAndValues.back().second;
    }

    std::vector<std::pair<T,T>> pointsAndValuesRed;
    std::vector<std::pair<T,T>> pointsAndValuesGreen;
    std::vector<std::pair<T,T>> pointsAndValuesBlue;

};

template <class T>
class ColorMap {
public:
    enum class Type {
        GRAY,
        REDBLUE,
        JET
    };

    /**
     * Maps input values to colors using the specified color map
     */
    static Eigen::Matrix<T, 4, -1> Evaluate(Eigen::Ref<const Eigen::VectorX<T>> values, const T minValue = T(0), const T maxValue = T(1), Type type = Type::JET)
    {
        Eigen::Matrix<T, 4, -1> colors(4, values.size());
        ColorSegments<T> colorSegments;
        switch (type) {
            case Type::GRAY: {
                colorSegments = ColorSegments<T>::GrayColorSegments();
                break;
            };
            case Type::REDBLUE: {
                colorSegments = ColorSegments<T>::RedBlueColorSegments();
                break;
            };
            case Type::JET: {
                colorSegments = ColorSegments<T>::JetColorSegments();
                break;
            }
        }

        for (int i = 0; i < values.size(); i++) {
            colors.col(i) = colorSegments.ValueToColor((values[i] - minValue) / (maxValue - minValue));
        }

        return colors;
    }
};

}