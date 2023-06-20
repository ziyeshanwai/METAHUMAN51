// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "data_types.h"
#include "data_utils.h"
#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/geometry.h>
POSEBASEDSOLVER_RENABLE_WARNINGS
#include <numeric>
#include <random>

namespace cm
{
    using namespace dlib;


    /*!
        Casts all points in a vector of points to have double type

        - REQUIREMENTS ON T
            - Must be either float, double, or long double, ie:
              is_float_type<T>::value == true

         - Ensures
             - Returns a vector of points matching the input vector but cast to double type
    */
    template<typename T>
    std::vector<point2<double>> cast_points_to_double(const std::vector<point2<T>>& pts)
    {
        std::vector<dlib::vector<double, 2>> dbl_pts(pts.size());
        for (size_t i = 0; i < pts.size(); ++i)
        {
            dbl_pts[i] = point2<double>(pts[i].x(), pts[i].y());
        }
        return dbl_pts;
    }


    /*!
        Compute the mean point of a shape.

        - REQUIREMENTS ON T
               - Must be either float, double, or long double, ie:
                 is_float_type<T>::value == true

        - Requires
            -  shape.size() > 0

        - Ensures
            - Returns the mean point of the shape.
    */
    template<typename T>
    point2<T> mean_point(const point2_vector<T>& shape)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(shape.size() > 0,
            "\t mean_point(const point2_vector<T>& shape)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t shape.size(): " << shape.size());

        point2<T> mean_pt{ 0,0 };
        for (auto pt : shape)
        {
            mean_pt += pt;
        }
        mean_pt /= static_cast<T>(shape.size());
        return mean_pt;
    }

    /*!
        Compute the mean point of a shape.

        - REQUIREMENTS ON T
               - Must be either float, double, or long double, ie:
                 is_float_type<T>::value == true

        - Requires
            -  shape.size() > 0

        - Ensures
            - Returns the mean point of the shape.
    */
    template<typename T>
    point3<T> mean_point(const point3_vector<T>& shape)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(shape.size() > 0,
            "\t mean_point(const point3_vector<T>& shape)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t shape.size(): " << shape.size());

        point3<T> mean_pt{ 0,0,0 };
        for (const auto& pt : shape)
        {
            mean_pt += pt;
        }
        mean_pt /= static_cast<T>(shape.size());
        return mean_pt;
    }

}
