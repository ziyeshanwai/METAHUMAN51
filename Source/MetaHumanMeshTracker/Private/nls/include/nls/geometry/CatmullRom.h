// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/Polyline.h>

#include <vector>

namespace epic::nls {

/**
 */
template <class T, int D>
class CatmullRom
{
public:
    CatmullRom() = default;
    CatmullRom(const Eigen::Matrix<T, D, -1>& controlPoints, int pointsPerSegment, bool closed)
    {
        Set(controlPoints, pointsPerSegment, closed);
    }

    //! Sets the control points of the catmull rom curve.
    void Set(const Eigen::Matrix<T, D, -1>& controlPoints, int pointsPerSegment, bool closed);

    const Polyline<T, D>& ControlPoints() const { return m_controlPoints; }
    const Polyline<T, D>& SampledPoints() const { return m_sampledPoints; }

private:
    Polyline<T, D> m_controlPoints;
    int m_pointsPerSegment = 1;
    Polyline<T, D> m_sampledPoints;
};

} // namespace epic::nls
