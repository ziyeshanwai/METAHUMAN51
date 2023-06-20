// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/CatmullRom.h>

namespace epic::nls {

template<typename T, int D>
Eigen::Vector<T, D> CatmullRomPointOnCurve(const Eigen::Vector<T, D>& a, const Eigen::Vector<T, D>& b, const Eigen::Vector<T, D>& c, const Eigen::Vector<T, D>& d, T t)
{
    static_assert( std::is_same<T, float>::value || std::is_same<T, double>::value, "T must be a float or double type");

    return a * ((-t + T(2)) * t - T(1)) * t * T(0.5) +
        b * (((T(3) * t - T(5)) * t) * t + T(2)) * T(0.5) +
        c * ((-T(3) * t + T(4)) * t + T(1)) * t * T(0.5) +
        d * ((t - T(1)) * t * t) * T(0.5);
}

template<typename T, int D>
Eigen::Matrix<T, D, -1> GenerateCatmullRomSpline(const Eigen::Matrix<T, D, -1>& points,
        const std::vector<int>& n_points_per_section, bool closed)
{
    static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value, "T must be a float or double type");

    int N_points = static_cast<int>(points.cols());
    CARBON_ASSERT(N_points >= 2, "catmull rom curve needs at least two points");
    CARBON_ASSERT(((static_cast<int>(n_points_per_section.size()) == (N_points - 1)) && (closed == false)) ||
            ((static_cast<int>(n_points_per_section.size()) == N_points) && (closed == true)), "vector of number of points of sections does not match the requested number of control points");

    std::vector<Eigen::Vector<T, D>> spline_points;
    Eigen::Vector<T, D> a, b, c, d;

    if (points.cols() == 2)   // Straight line
    {
        Eigen::Vector<T, D> delta = points.col(1) - points.col(0);
        Eigen::Vector<T, D> step = delta / T(n_points_per_section[0] - 1);
        for (int i = 0; i < n_points_per_section[0]; ++i)
        {
            spline_points.emplace_back(points.col(0) + T(i) * step);
        }
    }
    else
    {
        if (!closed)
        {
            for (int i_point = 0; i_point < static_cast<int>(points.cols()) - 1; i_point++)
            {
                if (0 == i_point)
                {
                    a = points.col(i_point);//points.col(i_point) - (points.col(i_point + 1) - points.col(i_point));
                    b = points.col(i_point);
                    c = points.col(i_point + 1);
                    d = points.col(i_point + 2);
                    spline_points.emplace_back(points.col(i_point)); //Add the first point if this is the first section
                    for (int j = 1; j <= n_points_per_section[i_point]; j++)
                    {
                        T t = (T(1) / T(n_points_per_section[i_point])) * j;
                        spline_points.emplace_back(CatmullRomPointOnCurve<T>(a, b, c, d, t));
                    }
                }
                else if (i_point == (N_points - 2))
                {
                    a = points.col(i_point - 1);
                    b = points.col(i_point);
                    c = points.col(i_point + 1);
                    d = points.col(i_point + 1);//c - (b - c);
                    for (int j = 1; j <= n_points_per_section[i_point]; j++)
                    {
                        T t = (T(1) / T(n_points_per_section[i_point])) * j;
                        spline_points.emplace_back(CatmullRomPointOnCurve<T, D>(a, b, c, d, t));
                    }
                }
                else
                {
                    a = points.col(i_point - 1);
                    b = points.col(i_point);
                    c = points.col(i_point + 1);
                    d = points.col(i_point + 2);
                    for (int j = 1; j <= n_points_per_section[i_point]; j++)
                    {
                        T t = (T(1) / n_points_per_section[i_point]) * j;
                        spline_points.emplace_back(CatmullRomPointOnCurve<T, D>(a, b, c, d, t));
                    }
                }
            }
        }
        else // closed
        {
            for (int i_point = 0; i_point < N_points; i_point++)
            {
                if (0 == i_point)
                {
                    a = points.col(N_points - 1);
                    b = points.col(i_point);
                    c = points.col(i_point + 1);
                    d = points.col(i_point + 2);
                    spline_points.emplace_back(points.col(i_point)); //Add the first point if this is the first section
                    for (int j = 1; j <= n_points_per_section[i_point]; j++)
                    {
                        T t = (T(1) / n_points_per_section[i_point]) * j;
                        spline_points.emplace_back(CatmullRomPointOnCurve<T, D>(a, b, c, d, t));
                    }
                }
                else if (i_point == (N_points - 2))
                {
                    a = points.col(i_point - 1);
                    b = points.col(i_point);
                    c = points.col(i_point + 1);
                    d = points.col(0);
                    for (int j = 1; j <= n_points_per_section[i_point]; j++)
                    {
                        T t = (T(1) / n_points_per_section[i_point]) * j;
                        spline_points.emplace_back(CatmullRomPointOnCurve<T, D>(a, b, c, d, t));
                    }
                }
                else if (i_point == (N_points - 1))
                {
                    a = points.col(i_point - 1);
                    b = points.col(i_point);
                    c = points.col(0);
                    d = points.col(1);
                    for (int j = 1; j <= n_points_per_section[i_point] - 1; j++)
                    {
                        T t = (T(1) / n_points_per_section[i_point]) * j;
                        spline_points.emplace_back(CatmullRomPointOnCurve<T, D>(a, b, c, d, t));
                    }
                }
                else
                {
                    a = points.col(i_point - 1);
                    b = points.col(i_point);
                    c = points.col(i_point + 1);
                    d = points.col(i_point + 2);
                    for (int j = 1; j <= n_points_per_section[i_point]; j++)
                    {
                        T t = (T(1) / n_points_per_section[i_point]) * j;
                        spline_points.emplace_back(CatmullRomPointOnCurve<T, D>(a, b, c, d, t));
                    }
                }
            }
        }
    }
    return Eigen::Map<const Eigen::Matrix<T, D, -1>>((const T*)spline_points.data(), D, spline_points.size());
}

template<typename T, int D>
Eigen::Matrix<T, D, -1> GenerateCatmullRomSpline(const Eigen::Matrix<T, D, -1>& points, int n_points_per_section, bool closed)
{
    static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value, "T must be a float or double type");
    CARBON_ASSERT(points.cols() >= 2, "catmull rom curve needs at least two points");
    return closed
            ? GenerateCatmullRomSpline(points, std::vector<int>(points.cols(), n_points_per_section), true)
            : GenerateCatmullRomSpline(points, std::vector<int>(points.cols() - 1, n_points_per_section), false);
}

template <class T, int D>
void CatmullRom<T, D>::Set(const Eigen::Matrix<T, D, -1>& controlPoints, int pointsPerSegment, bool closed)
{
    m_controlPoints.Set(controlPoints);
    m_sampledPoints.Set(GenerateCatmullRomSpline(controlPoints, pointsPerSegment, closed));
    m_pointsPerSegment = pointsPerSegment;
}

// explicitly instantiate the catmullrom classes
template class CatmullRom<float, 2>;
template class CatmullRom<double, 2>;
template class CatmullRom<float, 3>;
template class CatmullRom<double, 3>;

} // namespace epic::nls
