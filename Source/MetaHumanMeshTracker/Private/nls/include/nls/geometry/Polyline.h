// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>

#include <limits>

namespace epic::nls {

/**
 * Polyline represents a curve as a set of points. When setting the control points, Polyine automtically
 * removes all invalid i.e. zero-length line segments.
 */
template <class T, int D>
class Polyline
{
public:
    Polyline() = default;
    Polyline(const Eigen::Matrix<T, D, -1>& controlPoints)
    {
        Set(controlPoints);
    }

    //! Sets the control points of the polyline. Removes all empty segments.
    void Set(const Eigen::Matrix<T, D, -1>& controlPoints)
    {
        m_controlPoints = controlPoints;
        int lastIndex = 0;
        // make sure we only keep line segments that do not have zero length
        for (int k = 1; k < int(controlPoints.cols()); k++) {
            if ((m_controlPoints.col(k) - m_controlPoints.col(lastIndex)).squaredNorm() > 0) {
                lastIndex++;
            }
        }
        m_controlPoints.conservativeResize(D, lastIndex + 1);
    }

    const Eigen::Matrix<T, D, -1>& ControlPoints() const { return m_controlPoints; }

    int NumControlPoints() const { return int(m_controlPoints.cols()); }

    //! a polyline is only valid if it has at least two control points.
    bool Valid() const { return NumControlPoints() > 1; }

    /**
     * Finds the closest point by going through all line segments and finding the closest point.
     * @param [in]  query    The query position.
     * @param [out] segment  The segment on which the closest point lies.
     * @param [out] lambda   The lambda value between [0, 1] on the segment.
     * @return the distance to the line
     *
     */
    T ClosestPoint(const Eigen::Vector<T, D>& query, int& closestSegment, T& closestLambda) const
    {
        closestSegment = 0;
        closestLambda = 0;
        if (Valid()) {
            T bestDist = std::numeric_limits<T>::max();
            const int numControlPoints = int(m_controlPoints.cols());
            for (int i = 0; i < numControlPoints - 1; ++i) {
                const Eigen::Vector<T, D> P = m_controlPoints.col(i);
                const Eigen::Vector<T, D> Q = m_controlPoints.col(i + 1);
                const T lengthSquared = (Q - P).dot(Q - P);
                T lambda = (lengthSquared > 0) ? (query - P).dot(Q - P) / lengthSquared : 0;
                Eigen::Vector<T, D> V;
                if (lambda < 0) {
                    V = P;
                    lambda = 0;
                } else if (lambda > 1) {
                    V = Q;
                    lambda = 1;
                } else {
                    V = P + lambda * (Q - P);
                }
                T dist = (query - V).squaredNorm();
                if (dist < bestDist) {
                    bestDist = dist;
                    closestSegment = i;
                    closestLambda = lambda;
                }
            }
            return sqrt(bestDist);
        } else {
            return std::numeric_limits<T>::max();
        }
    }

    /**
     * Finds the closest point by going through all line segments and finding the closest point.
     * @param [in]  query      The query position.
     * @param [out] closest    The closest point.
     * @param [out] direction  The direction of the polyline at the closest point (defined by the line segment).
     * @return the distance to the line
     *
     */
    T ClosestPointAndDirection(const Eigen::Vector<T, D>& query, Eigen::Vector<T, D>& closest, Eigen::Vector<T, D>& direction, T& confidence) const
    {
        confidence = 0;
        if (Valid()) {
            int segment = 0;
            T lambda = 0;
            const T dist = ClosestPoint(query, segment, lambda);
            const int numControlPoints = int(m_controlPoints.cols());
            const Eigen::Vector<T, D> dir = m_controlPoints.col(segment+1) - m_controlPoints.col(segment);
            closest = m_controlPoints.col(segment) + lambda * dir;
            direction = dir.normalized();
            const bool correspondenceIsAtCurveEndPoints = (lambda <= 0 && segment == 0) || (lambda >= 1 && segment == (numControlPoints - 2));
            confidence = correspondenceIsAtCurveEndPoints ? T(0) : T(1);
            return dist;
        } else {
            closest = Eigen::Vector<T, D>::Zero();
            direction = Eigen::Vector<T, D>::Zero();
            return std::numeric_limits<T>::max();
        }
    }

    /**
     * Finds the closest point by going through all line segments and finding the closest point.
     * @param [in]  query    The query position.
     * @param [out] closest  The closest point.
     * @param [out] normal   The normal of the polyline at the closest point (defined by the line segment).
     * @return the distance to the line
     *
     */
    template <typename U = T>
    T ClosestPointAndNormal(const Eigen::Vector<T, 2>& query, Eigen::Vector<T, 2>& closest, Eigen::Vector<T, 2>& normal, T& confidence, typename std::enable_if<std::is_same<U,T>::value && D == 2>::type* = 0) const
    {
        Eigen::Vector<T, 2> direction;
        T dist = ClosestPointAndDirection(query, closest, direction, confidence);
        normal = Eigen::Vector<T, 2>(-direction[1], direction[0]);
        return dist;
    }

    /**
     * Finds the closest point by going through all line segments and finding the closest point.
     * @param [in]  query     The query position.
     * @param [out] closest   The closest point.
     * @param [out] normal    The random normal of the polyline.
     * @param [out] direction The direction of the line segment.
     * @return the distance to the line
     *
     */
    template <typename U = T>
    T ClosestPointAndNormal(const Eigen::Vector<T, 3>& query, Eigen::Vector<T, 3>& closest, Eigen::Vector<T, 3>& normal, Eigen::Vector<T, 3>& direction, T & confidence, typename std::enable_if<std::is_same<U, T>::value && D == 3>::type* = 0) const
    {
        T dist = ClosestPointAndDirection(query, closest, direction, confidence);
        Eigen::Vector<T, 3> rnd = Eigen::Vector<T, 3>(T(0), T(0), T(0));
        if (std::abs(direction[0]) < std::abs(direction[1]))
        {
            rnd[0] = T(1);
        }
        else 
        {
            rnd[1] = T(1);
        }
        normal = rnd.cross(direction).normalized();
        return dist;
    }

    /**
     * Finds all intersection points of the line [A, B] with the polyline.
     * @returns the intersection points as alpha values i.e. an intersection point is (1 - alpha) A + alpha B
     */
    template <typename U = T>
    std::vector<T> FindIntersections(const Eigen::Vector2<T>& A, const Eigen::Vector2<T>& B, typename std::enable_if<std::is_same<U,T>::value && D == 2>::type* = 0) const
    {
        std::vector<T> intersections;
        const int numControlPoints = int(m_controlPoints.cols());
        for (int i = 0; i < numControlPoints - 1; ++i) {
            const Eigen::Vector2<T> P = m_controlPoints.col(i);
            const Eigen::Vector2<T> Q = m_controlPoints.col(i + 1);
            const T v1 = (A[0] - P[0]) * (P[1] - Q[1]) - (A[1] - P[1]) * (P[0] - Q[0]);
            const T v2 = (A[0] - B[0]) * (P[1] - Q[1]) - (A[1] - B[1]) * (P[0] - Q[0]);
            const T v3 = - (A[0] - B[0]) * (A[1] - P[1]) + (A[1] - B[1]) * (A[0] - P[0]);
            if (fabs(v2) > T(1e-9)) {
                const T alpha = v1 / v2;
                const T beta = v3 / v2;
                if (alpha >= 0 && alpha <= 1 && beta >= 0 && (beta < 1 || (i == numControlPoints - 2 && beta <= 1))) {
                    intersections.push_back(alpha);
                }
            }

        }
        return intersections;
    }

private:
    Eigen::Matrix<T, D, -1> m_controlPoints;
};

} // namespace epic::nls
