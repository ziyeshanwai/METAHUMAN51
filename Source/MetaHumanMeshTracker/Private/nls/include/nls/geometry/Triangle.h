// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

#include <algorithm>
#include <numeric>

namespace epic {
namespace nls {

/**
 * Calculates the triangle quality as circumradius over minimal edge length. The lower the better, the optimal/lowest quality is 2/sqrt(3) for
 * an equilateral triangle.
 */
template <class T>
T TriangleQuality(const Eigen::Vector3<T>& v1, const Eigen::Vector3<T>& v2, const Eigen::Vector3<T>& v3)
{
    const Eigen::Vector3<T>& e12 = v2 - v1;
    const Eigen::Vector3<T>& e23 = v3 - v2;
    const Eigen::Vector3<T>& e31 = v1 - v3;
    const T l12 = e12.norm();
    const T l23 = e23.norm();
    const T l31 = e31.norm();
    const T area2 = e12.cross(e23).norm();
    if (area2 > 0) {
        const T minL = std::min<T>(l12, std::min<T>(l23, l31));
        const T diameter = (l12 * l23 * l31) / area2; // area2 = l12 * l23 * sin(theta) => diameter = l12 / (sin(theta))
        return diameter / minL;
    } else {
    return std::numeric_limits<T>::max();
    }
}


/**
 * Closest point calculation as described in "Real-Time Collision Detection" by Christer Ericson
 * @returns the barycentric coordinates
 */
template <class T>
inline Eigen::Vector3<T> ClosestPtPointTriangle(Eigen::Vector3<T> p, Eigen::Vector3<T> a, Eigen::Vector3<T> b, Eigen::Vector3<T> c)
{
    Eigen::Vector3<T> ab = b - a;
    Eigen::Vector3<T> ac = c - a;
    Eigen::Vector3<T> bc = c - b;
    // Compute parametric position s for projection P’ of P on AB,
    // P’ = A + s*AB, s = snom/(snom+sdenom)
    const T snom = (p - a).dot(ab);
    const T sdenom = - (p - b).dot(ab);
    // Compute parametric position t for projection P’ of P on AC,
    // P’ = A + t*AC, s = tnom/(tnom+tdenom)
    const T tnom = (p - a).dot(ac);
    const T tdenom = - (p - c).dot(ac);
    if (snom <= 0 && tnom <= 0) return Eigen::Vector3<T>(1, 0, 0); // Vertex region early out
    // Compute parametric position u for projection P’ of P on BC,
    // P’ = B + u*BC, u = unom/(unom+udenom)
    const T unom = (p - b).dot(bc);
    const T udenom = - (p - c).dot(bc);
    if (sdenom <= 0 && unom <= 0) return Eigen::Vector3<T>(0, 1, 0); // Vertex region early out
    if (tdenom <= 0 && udenom <= 0) return Eigen::Vector3<T>(0, 0, 1); // Vertex region early out
    // P is outside (or on) AB if the triple scalar product [N PA PB] <= 0
    const Eigen::Vector3<T> n = ab.cross(ac);
    const T vc = n.dot((a - p).cross(b - p));
    // If P outside AB and within feature region of AB,
    // return projection of P onto AB
    if (vc <= 0 && snom >= 0 && sdenom >= 0) {
        const T v = snom / (snom + sdenom);
        return Eigen::Vector3<T>(T(1) - v, v, 0);
    }
    // P is outside (or on) BC if the triple scalar product [N PB PC] <= 0
    const T va = n.dot((b - p).cross(c - p));
    // If P outside BC and within feature region of BC,
    // return projection of P onto BC
    if (va <= 0 && unom >= 0 && udenom >= 0) {
        const T w = unom / (unom + udenom);
        return Eigen::Vector3<T>(0, T(1) - w, w);
    }
    // P is outside (or on) CA if the triple scalar product [N PC PA] <= 0
    const T vb = n.dot((c - p).cross(a - p));
    // If P outside CA and within feature region of CA,
    // return projection of P onto CA
    if (vb <= 0 && tnom >= 0 && tdenom >= 0) {
        const T w = tnom / (tnom + tdenom);
        return Eigen::Vector3<T>(T(1) - w, 0, w);
    }
    // P must project inside face region. Compute Q using barycentric coordinates
    const T u = va / (va + vb + vc);
    const T v = vb / (va + vb + vc);
    const T w = 1.0f - u - v; // = vc / (va + vb + vc)
    return Eigen::Vector3<T>(u, v, w);
}

} // namespace nls
} //namespace epic
