// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

namespace epic::nls {

/**
 * Ray triangle intersection using Moeller–Trumbore intersection algorithm.
 * Code based on https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm.
 */
template <class T>
bool RayTriangleIntersection(const Eigen::Vector3<T>& rayOrigin,
                             const Eigen::Vector3<T>& rayVector,
                             const Eigen::Vector3<T>& vertex0,
                             const Eigen::Vector3<T>& vertex1,
                             const Eigen::Vector3<T>& vertex2,
                             T* outT,
                             Eigen::Vector3<T>* outIntersection)
{
    const T EPSILON = T(0.0000001);
    const Eigen::Vector3<T> edge1 = vertex1 - vertex0;
    const Eigen::Vector3<T> edge2 = vertex2 - vertex0;
    const Eigen::Vector3<T> h = rayVector.cross(edge2);
    const T a = edge1.dot(h);
    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.
    const T f = T(1.0)/a;
    const Eigen::Vector3<T> s = rayOrigin - vertex0;
    const T u = f * s.dot(h);
    if (u < 0.0 || u > 1.0)
        return false;
    const Eigen::Vector3<T> q = s.cross(edge1);
    const T v = f * rayVector.dot(q);
    if (v < T(0.0) || u + v > T(1.0))
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    const T t = f * edge2.dot(q);
    if (outT) {
        *outT = t;
    }
    if (t > EPSILON) // ray intersection
    {
        if (outIntersection) {
            *outIntersection = rayOrigin + rayVector * t;
        }
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}

} // namespace epic::nls
