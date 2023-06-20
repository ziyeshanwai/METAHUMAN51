// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "basic_types.h"
#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/geometry.h>
RLIBV_RENABLE_WARNINGS

namespace rlibv {
	/**
	* \defgroup Transforms Transforms
	* @{
	*/

    /**
    * @brief Apply a projective transform to every point in the shape vector
    * @tparam T Must be either float or double
    * @param points
    * @param xform Projective transformation
    * @pre points.size() >= 1
    * @returns New shape vector where every point has been transformed by the given projective transform
    */
    template <typename T>
    shape2d<T> transform_shape(const shape2d<T>& points, const dlib::point_transform_projective& xform);

    /**
    * @brief Apply a projective transform to every rectangle in the vector
    * @tparam T Must be either float or double
    * @param rects
    * @param xform Projective transformation
    * @pre points.size() >= 1
    * @returns New  vector of rects where each one has been transformed by the given projective transform
    */
    inline std::vector<dlib::drectangle> transform_rects(const std::vector<dlib::drectangle>& rects, const dlib::point_transform_projective& xform);

	/**
	* @brief Compute a weighted mean of the points
	* @tparam T Must be either float or double
	* @param shape
	* @param weights
	* @pre shape.size() == weights.size()
	* @returns New point which is the mean of all points. 
	* @remark If the weights sum to zero this function returns (0,0). Weights are always considered positive
	*		  even if you supply negative values - you shouldn't!
	*/
	template<typename T>
	point2d<T> weighted_mean_point(const shape2d<T>& shape, const std::vector<T>& weights);

 

    /**
    * Find the projective transform which best aligns two sets of 2d points.
    * @tparam T Must be either float or double
    * @param from_points
    * @param to_points
    * @pre from_points.size() == to_points.size()
    * @pre from_points.size() >= 4
    * @returns A point_transform_projective object P
    * @details For all valid i: length(P(from_points[i]) - to_points[i])
    *          is minimized as often as possible.  That is, this function finds the
    *          transform that maps points in from_points to points in to_points.  If no
    *          transform exists which performs this mapping exactly then the one
    *          which minimizes the mean squared error is selected.
    */
    template <typename T>
    dlib::point_transform_projective find_projective_transform_as_projective(
        const shape2d<T>& from_points, const shape2d<T>& to_points);

    /**
    * @brief Find the similarity transform which best aligns two sets of 2d points
    * @tparam T Must be either float or double
    * @param from_points
    * @param to_points
    * @pre from_points.size() == to_points.size()
    * @pre from_points.size() >= 2
    * @returns A point_transform_projective object P
    * @details For all valid i: length(P(from_points[i]) - to_points[i])
    *          is minimized as often as possible using a similarity transform.
    *          That is, this function finds the transform that maps points in from_points
    *          to points in to_points.  If no transform exists which performs this mapping
    *          exactly then the one which minimizes the mean squared error is selected.
    */
    template <typename T>
    dlib::point_transform_projective find_similarity_transform_as_projective(
        const shape2d<T>& from_points, const shape2d<T>& to_points);

    /**
    * @brief Find the rigid transform which best aligns two sets of 2d points
    * @tparam T Must be either float or double
    * @param from_points
    * @param to_points
    * @pre from_points.size() == to_points.size()
    * @pre from_points.size() >= 2
    * @returns A point_transform_projective object P
    * @details For all valid i: length(P(from_points[i]) - to_points[i])
    *          is minimized as often as possible using a rigid transform.
    *          That is, this function finds the transform that maps points in from_points
    *          to points in to_points.  If no transform exists which performs this mapping
    *          exactly then the one which minimizes the mean squared error is selected.
    */
    template <typename T>
    dlib::point_transform_projective find_rigid_transform_as_projective(
        const std::vector<point2d<T> >& from_points, const std::vector<point2d<T> >& to_points);

    /**
    * @brief Find the translation transform which best aligns two sets of 2d points
    * @tparam T Must be either float or double
    * @param from_points
    * @param to_points
    * @pre from_points.size() == to_points.size()
    * @pre from_points.size() >= 1
    * @returns A point_transform_projective object P
    * @details For all valid i: length(P(from_points[i]) - to_points[i])
    *          is minimized as often as possible using a translation transform.
    *          That is, this function finds the transform that maps points in from_points
    *          to points in to_points.  If no transform exists which performs this mapping
    *          exactly then the one which minimizes the mean squared error is selected.
    */
    template <typename T>
    dlib::point_transform_projective find_translate_transform_as_projective(
        const std::vector<point2d<T> >& from_points, const std::vector<point2d<T> >& to_points);

    /**
    * @brief Find the scale_translate transform which best aligns two sets of 2d points
    * @tparam T Must be either float or double
    * @param from_points
    * @param to_points
    * @pre from_points.size() == to_points.size()
    * @pre from_points.size() >= 2
    * @returns A point_transform_projective object P
    * @details For all valid i: length(P(from_points[i]) - to_points[i])
    *          is minimized as often as possible using a scale_translate transform.
    *          That is, this function finds the transform that maps points in from_points
    *          to points in to_points.  If no transform exists which performs this mapping
    *          exactly then the one which minimizes the mean squared error is selected.
    */
    template <typename T>
    dlib::point_transform_projective find_scale_translate_transform_as_projective(
        const std::vector<point2d<T> >& from_points, const std::vector<point2d<T> >& to_points);

    /**
   * @brief Find the transform which best aligns two sets of 2d points using a given alignment method.
   * @tparam T Must be either float or double
   * @param from_points
   * @param to_points
   * @param align_method Alignment type
   * @pre from_points.size() == to_points.size()
   * @pre from_points.size() >= 0,1,2 or 3 depending on the alignment type
   * @remark Alignment type can be: none, translation, rigid, similarity, projection or scale_translate.
   *         - none simply returns the null transform and requires no points
   *         - translation needs 1 or more points
   *         - rigid, similarity, and scale_translate need 2 or more points
   *         - projective needs 4 or more points
   * @returns A point_transform_projective object P
   * @details For all valid i: length(P(from_points[i]) - to_points[i])
   *          is minimized as often as possible.  That is, this function finds the
   *          transform that maps points in from_points to points in to_points.  If no
   *          transform exists which performs this mapping exactly then the one
   *          which minimizes the mean squared error is selected.
   * @remark The result is returned as a general projective transform, even though
   *         all cases except projective are special cases of the projective transform.
   */
    template <typename T>
    dlib::point_transform_projective find_transform_as_projective(
        const std::vector<point2d<T>>& from_points, const std::vector<point2d<T>>& to_points, const alignment_type align_method);


    /*!
    * @brief Convert the given transform to an approximation of the matching similarity transform
    * @param xform
    * @returns The approximate transform as an array [scale, angle, dx, dy]
    * @remark The approximation is exact for all transforms except projective and affine.
    */
    template <typename T>
    std::array<T, 4> approx_similarity_transform(const dlib::point_transform_projective& xform);

    /*!
    * @brief Convert the given similarity transform parameters to a projective transform matrix
    * @param params The approximate transform as an array [scale, angle, dx, dy]
    * @returns A point_transform_projective object 
    */
    template <typename T>
    dlib::point_transform_projective similarity_params_as_projective(const std::array<T, 4>& params);


    /**@}*/
}

#include "impl/transforms.hpp"