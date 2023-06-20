// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "basic_types.h"

#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/geometry.h>
RLIBV_RENABLE_WARNINGS

namespace rlibv {
	/**
	* \defgroup Geometry Geometry
	* @{
	*/

	/**
	 * @brief Is a point inside a 2D polygon
	 * @tparam T Must be either float or double
	 * @param polygon Vector of vertices
	 * @param pt The test point
	 * @return true if point is inside the polygon otherwise false
	 */
	template<typename T>
	bool in_polygon(const std::vector<point2d<T>>& polygon, const point2d<T>& pt);

	/**
	 * @brief Cast the points in a shape from one type to another
	 * @tparam T Must be either float or double
	 * @tparam K Must be int, long, long long, float or double
	 * @param shape The shape to convert
	 * @return The points as type T
	 */
	template<typename T, typename K> 
	std::vector<point2d<T>> cast_shape(const std::vector<point2d<K>>& shape);


	/**
	 * @brief Computes the distance between a point and line segment
	 * @tparam T Must be either float or double
	 * @param a one end of the line
	 * @param b the other end of the line
	 * @param p the point to test
	 * @return Distance of the point to the line SEGMENT defined by the ends
	 *         a and b. This is NOT the same as the distance to the infinitely
	 *         long line passing through a and b. Note therefore, this still works
	 *         even if a==b
	 */
	template<typename T>
	T distance_to_line_segment(const point2d<T>& a, const point2d<T>& b, const point2d<T>& p);

	/**
	 * @brief Computes the minimum distance between a point and vector of line segments
s	 * @tparam T Must be either float or double
	 * @param line_pts contiguous points defining the line
	 * @param p the point to test
	 * @return Distance of the point to closest point on any of the the line SEGMENTS
	 * @pre line_pts.size() > 1
	 */
	template<typename T>
	T distance_to_polyline(const std::vector<point2d<T>>& line_pts, const point2d<T>& p);

	/**
	 * @brief Computes a distance cross-table for a set of points
	 * @tparam T Must be either float or double
	 * @param points
	 * @return Cross-table of the distance between each pair of points
	 */
	template<typename T>
	std::vector<std::vector<T>> point_cross_distances(const std::vector<point2d<T>>& points);

	/**
	* @brief Computes the positive area enclosed by a quadrilateral
	* @tparam T Must be either float or double
	* @param vertices
	* @return The area enclosed by the quadrilateral formed by the four vertices
	*/
	template<typename T>
	T quad_area(const std::array<point2d<T>, 4>& vertices);

	/**
	* @brief Computes the angles of the tangents along a curve defined by points
	* @tparam T Must be either float or double
	* @param curve
	* @param closed
	* @pre curve.size()>0
	* @return Angle of the tangent at each point on the curve.
	* @remark The tangents at the first and last points assume the line continues
	*         in the same direction unless 'closed' is true in which case the
	*         first point is considered connected to the last.
	*/
	template<typename T>
	std::vector<T> tangent_angles(const std::vector<point2d<T>>& curve, bool closed);

	/**
	* @brief Computes the angles of the normals along a curve defined by points
	* @tparam T Must be either float or double
	* @param curve
	* @param closed
	* @pre curve.size()>0
	* @return Angle of the normal at each point on the curve.
	* @remark The tangents at the first and last points assume the line continues
	*         in the same direction unless 'closed' is true in which case the
	*         first point is considered connected to the last.
	*/
	template<typename T>
	std::vector<T> normal_angles(const std::vector<point2d<T>>& curve, bool closed);

	/**
	* @brief Computes the signed angle between two vectors
	* @tparam T Must be either float or double
	* @param a Start point of first line
	* @param b End point of first line
	* @param c Start point of second line
	* @param d End point of second line
	* @pre (b-a).length() > 0
	* @pre (d-c).length() > 0
	* @returns Signed angle between the vectors b-a, and d-c
	*/
	template<typename T>
	T signed_angle(point2d<T> a, point2d<T> b, point2d<T> c, point2d<T> d);

	
	/**
	* @brief Find the range of all the points in a map of points
	* @tparam T Must be either float or double
	* @param points
	* @pre points.size() > 0
	* @returns Single column matrix with 4 elements giving the ranges covered by the point vector
	*         as min x, max x, min y, max y
	*/
	template<typename T>
	dlib::matrix<T, 4, 1> minx_maxx_miny_maxy(const shape2d<T>& points);

	/**
	* @brief Find the range of the points across a set of point maps
	* @tparam T Must be either float or double
	* @tparam K must be an int or string type
	* @param points
	* @pre points.size() > 0
	* @pre points[0].size() > 0
	* @returns Single column matrix with 4 elements giving the ranges covered by all the point maps
	*          as min x, max x, min y, max y
	*/
	template<typename T, typename K>
	dlib::matrix<T, 4, 1> minx_maxx_miny_maxy(const std::vector<std::map<K, point2d<T>>>& points);

	/**
	* @brief Find the range of the points across a set of point vectors
	* @tparam T Must be either float or double
	* @param points
	* @pre points.size() > 0
	* @pre points[0].size() > 0
	* @returns Single column matrix with 4 elements giving the ranges covered by all the point vectors
	*          as min x, max x, min y, max y
	*/
	template<typename T>
	dlib::matrix<T, 4, 1> minx_maxx_miny_maxy(const std::vector<shape2d<T>>& points);

	/**
	* @brief Computes the relative length of the "shadow" of one vector projected on another
	* @tparam T Must be either float or double
	* @param a
	* @param b
	* @pre b.length() > 0
	* @returns Relative length of the "shadow" when a is projected onto b
	* @note if(b.length_squared() < std::numeric_limits<T>::min()) returns std::numeric_limits<T>::max()
	*/
	template<typename T>
	T fractional_scalar_projection(const point2d<T>& a, const point2d<T>& b);

	/**
	* @brief Computes the relative length of the "shadow" of one difference vector projected on another
	* @tparam T Must be either float or double
	* @param a
	* @param b
	* @param c
	* @param d
	* @pre (d-c).length() > 0
	* @returns Relative length of the "shadow" when (b-a) is projected onto (d-c)
	*/
	template<typename T>
	T fractional_scalar_projection(const point2d<T>& a, const point2d<T>& b, const point2d<T>& c, const point2d<T>& d);

	/**
	 * @brief Spread the points evenly across some line segments.
	 * @tparam T Must be either float or double
	 * @param input_points ordered set of points
	 * @param n_output_points the number of points (including the end points) in the output
	 * @pre input_points.size()>=2
	 * @pre n_ouput_points >=2
	 * @returns the evenly spaced points
	 */
	template<typename T>
	std::vector<point2d<T>> spread_points_evenly(const std::vector<point2d<T>>& input_points, int n_output_points);


	/**
	* @brief Generate a point on a 2d Catmull-Rom Spline
	* @tparam T Must be either float or double
	* @param a
	* @param b
	* @param c
	* @param d
	* @param t
	* @param alpha
	* @returns 2d point on the Catmull-Rom spline with parameter value t and coefficient of alpha
	*/
	template<typename T>
	point2d<T> catmullrom_point_on_curve(point2d<T> a, point2d<T> b, point2d<T> c, point2d<T> d, T t, T alpha=0.5f);

	/**
	* @brief Create a dense vector of points representing a 2D Catmull-Rom spline
	* @tparam T Must be either float or double
	* @param points
	* @param n_points_per_section The number of points in each section of the spline.
	* @param closed
	* @pre n_points_per_section >= 0
	* @pre points.size() >= 2
	* @returns Dense vector of points representing a Catmull-Rom spline made from key 2D points
	*/
	template<typename T>
	shape2d<T> generate_catmullrom_spline(const shape2d<T>& points, int n_points_per_section, bool closed);

	/**
	* @brief Reinterpret points along an open spline, taking into account fixed anchors
	* @tparam T Must be either float or double
	* @param points
	* @param anchor_indices Indices of the anchor points in the input curve.
	* @param n_out_points number of points in the final curve
	* @param out_fixed_indices the fixed indices in the output curve
	* @param resolution approximate number of points in the generated spline
	* @pre n_out_points >= points.size()
	* @pre resolution >= points.size()-1
	* @pre points.size() >= 2
	* @pre anchor_indices.size() >= 0
	* @pre anchor_indices.size() == out_fixed_indices.size()
	* @pre every element of anchor_indices >= 0 and < points.size()
	* @pre every element of out_fixed_indices >= 0 and < n_out_points
	* @returns Vector of points which reinterpret the spline with fixed anchors
	*/
	template<typename T>
	shape2d<T> reinterpret_open_catmullrom_spline(const shape2d<T>& points, const std::vector<int>& anchor_indices, int n_out_points, const std::vector<int>& out_fixed_indices, int resolution);

	/**
	 * @brief Reduce the density of a curve to a specific number of points roughly evenly spaced
	 * @tparam T Must be either float or double
	 * @param points A vector of points 
	 * @param n_output_points The number of point in the resulting spline, including the end point(s)
	 * @param is_closed True if the input curve (and therefore the output curve too) is a closed curve
	 * @pre points.size() >= 4
	 * @pre n_output_points >=2
	 * @returns vector of n_out_points points, selected from the original curve.
	 */
	template<typename T>
	shape2d<T> approximate_evenly_spaced_curve(const shape2d<T>& points, unsigned int n_output_points, bool is_closed);

	/**
	 * @brief Fit a Catmull-Rom spline with a fixed number of points
	 * @tparam T Must be either float or double
	 * @param extended_points A vector of points extended by one point at each end
	 * @param n_out_points The number of point in the resulting spline, including the end points
	 * @param resolution (default=100) The number of extra points per input point to generate for the approximation
	 * @pre extend_points.size() >= 4
	 * @pre n_out_points >=2
	 * @returns vector of n_out_points spline interpolated points from extended_points[1] to extended_points[extended_points.size()-2]
	 */
	template<typename T>
	shape2d<T> approximate_open_catmullrom_spline(const shape2d<T>& extended_points, int n_out_points, int resolution = 30);

	/**
	* @brief Reinterpret points along a closed spline, taking into account fixed anchors
	* @tparam T Must be either float or double
	* @param points
	* @param anchor_indices Indices of the anchor points in the input curve.
	* @param n_out_points number of points in the final curve
	* @param out_fixed_indices the fixed indices in the output curve
	* @param resolution approximate number of points in the generated spline
	* @pre n_out_points >= points.size()
	* @pre resolution >= points.size()-1
	* @pre points.size() >= 2
	* @pre anchor_indices.size() >= 0
	* @pre anchor_indices.size() == out_fixed_indices.size()
	* @pre every element of anchor_indices >= 0 and < points.size()
	* @pre every element of out_fixed_indices >= 0 and < n_out_points
	* @returns Vector of points which reinterpret the spline with fixed anchors
	*/
	template<typename T>
	shape2d<T> reinterpret_closed_catmullrom_spline(const shape2d<T>& points, const std::vector<int>& anchor_indices, int n_out_points, const std::vector<int>& out_fixed_indices, int resolution);

	/**
	* @brief Approximates a dense (open) Catmull-Rom spline with a sparser one
	* @param dense_points Vector of points describing the source curve
	* @param dense_anchor_inds Vector of indices of the anchor points in the dense curve
	* @param max_total_points The number of points in the sparse curve will not exceed this
	* @param tolerance Fitting will stop if error falls below this value
	* @param closed Set to true for closed curves
	* @param[out] sparse_points the resulting curve
	* @param[out] sparse_anchor_inds indices of the anchor points in the resulting curve
	* @pre every element of anchor_indices >= 0 and < points.size()
	* @pre dense_points.size()>=2
	*/
	template<typename T>
	void fit_sparse_catmullrom_spline(
		const shape2d<T>& dense_points,
		const std::vector<int>& dense_anchor_inds,
		int max_total_points,
		T tolerance,
		bool closed,
		shape2d<T>& sparse_points,
		std::vector<int>& sparse_anchor_inds);


	/**
	* @brief Create a dense vector of points representing a 2D Catmull-Rom spline
	* @tparam T Must be either float or double
	* @param points
	* @param n_points_per_section The number of points in each section of the spline
	* @param closed
	* @pre (n_points_per_section.size() == points.size()-1) && (closed == false) ||
	*	   (n_points_per_section.size() == points.size()) && (closed == true)
	* @pre points.size() >= 2
	* @returns Dense vector of points representing a Catmull-Rom spline made from key 2D points
	*/
	template<typename T>
	shape2d<T> generate_catmullrom_spline(const shape2d<T>& points, const std::vector<int>& n_points_per_section,
			bool closed);

	/**
	* @brief Create a dense vector of points representing a 2D Catmull-Rom spline
	* @tparam T Must be either float or double
	* @param points
	* @param n_points_per_section The number of points in each section of the spline.
	* @param closed
	* @pre n_points_per_section >= 0
	* @pre points.size() >= 2
	* @param[out] spline_points contains a dense vector of points representing a Catmull-Rom spline made from key 2D points
	* @param[out] sections contains a vector of ints indicating which section of the spline each point belongs to
	*/
	template<typename T>
	void generate_catmullrom_spline(const shape2d<T>& points, const std::vector<int>& n_points_per_section,
			bool closed, shape2d<T>& spline_points, std::vector<int>& sections);

	/**
	* @brief Generates intermediate points between given points, following a Catmull-Rom spline (open)
	* @tparam T Must be either float or double
	* @param points
	* @param n_intermediates_per_section The number of points to put in each intermediate section
	* @pre n_intermediates_per_section >= 0
	* @pre points.size() >= 2
	* @returns Intermediate points between given points, following a Catmull-Rom spline (open)
	*/
	template<typename T>
	shape2d<T> find_catmullrom_inbetween_points(const shape2d<T>& points, int n_intermediates_per_section);



	/**
	* @brief Generates a curve which sits between the two input curves
	* @tparam T Must be either float or double
	* @param curve_a
	* @param curve_b
	* @param n_points Integer
	* @param weight See 'details' for the meaning of the weight parameter
	* @param sample_accuracy Integer
	* @pre n_points >= 2
	* @pre curve_a.size() >= 2
	* @pre curve_b.size() >= 2
	* @returns Generates a curve which sits between curve_a and curve_b according to 'weight'.
	* @details A weight of 0.5 would put the new curve half way between, and weight of
	*		   0.0 would simply mimic curve_a, 1.0 would mimic curve_b.
	*		   The curve is created by first generating a Catmull Rom spline of 'sample_accuracy'
	*		   number of points for each of the two input curves, then interpolating between
	*		   corresponding positions along the two curves. Higher 'sample_accuracy' will produce
	*		   more accurate interpolation.
	*/
	template<typename T>
	shape2d<T> generate_intermediate_curve(const shape2d<T>& curve_a, const shape2d<T>& curve_b, int n_points, T weight, int sample_accuracy);

	/**
	* @brief Centres a shape around the origin
	* @tparam T Must be either float or double
	* @param points
	* @pre points.size() >= 1
	* @returns New points vector which is simply the input points vector such that the mean of the points is (0,0).
	*/
	template<typename T>
	shape2d<T> centre_shape_at_origin(const shape2d<T>& points);

	/**
	* @brief Compute the norm of a shape
	* @tparam T Must be either float or double
	* @param points
	* @pre points.size() >= 1
	* @returns Compute the norm of the input points vector, that is: sqrt(sum(x*x-y*y)) over all the points.
	*/
	template<typename T>
	T shape_norm(const shape2d<T>& points);

	/**
	* @brief Transform a shape to have unit norm
	* @tparam T Must be either float or double
	* @param points
	* @pre points.size() >= 1
	* @returns New points vector which is simply the input points vector adjusted to have a norm of 1.0.
	*/
	template<typename T>
	shape2d<T> scale_shape_to_unit_norm(const shape2d<T>& points);

	/**
	* @brief Returns the distance norm between two shapes
	* @tparam T Must be either float or double
	* @param points_a
	* @param points_b
	* @pre points_a.size() >= 1
	* @pre points_a.size() == points_b.size()
	* @returns Distance norm between two shapes
	*/
	template<typename T>
	T norm_between_shapes(const shape2d<T>& points_a, const shape2d<T>& points_b);

	/**
	* @brief Apply a random similarity distortion to a shape.
	* @tparam T Must be either float or double
	* @param points
	* @param scale_coeff
	* @param angle_coeff
	* @param dx_coeff
	* @param dy_coeff
	* @param rng Random number generator
	* @pre points.size() >= 1
	* @returns New shape; the original subject to random similarity distortion.
	*/
	template<typename T>
	shape2d<T> apply_random_similarity_distortion(const shape2d<T>& points, T scale_coeff, T angle_coeff, T dx_coeff, T dy_coeff, dlib::rand& rng);

	/**
	* @brief Generates intermediate points between given 3d points, following linear interpolation.
	* @tparam T Must be either float or double
	* @param a 3d point
	* @param b 3d point
	* @param n_intermediate_points
	* @pre n_intermediates_points >= 0
	* @returns Intermediate points between given points.
	*/
	template<typename T>
	shape3d<T> find_lerp_inbetween_points3d(const point3d<T>& a, const point3d<T>& b, int n_intermediate_points);

	/**
	* @brief Generates intermediate points between 2 given vectors, following linear interpolation.
	* @tparam T Must be either float or double
	* @param v0 
	* @param v1
	* @param n_intermediate_points
	* @pre v0.size() == v1.size()
	* @pre n_intermediates_points >= 0
	* @returns Intermediate points between given points.
	* @remark This can be thought of as an interpolation between 2 n-dimensional points. 
	*/
	template<typename T>
	std::vector<std::vector<T>> find_lerp_inbetween_vectors(const std::vector<T>& v0, const std::vector<T>& v1, int n_intermediate_points);

	/**
	* @brief Computes the sum of distances between corresponding points in a shape.
	* @tparam T Must be either float or double
	* @param points_a
	* @param points_b
	* @pre points_a.size() == points_b.size()
	* @returns Sum of distances between corresponding points in a shape.
	*/
	template<typename T>
	T sum_of_point_distances(const shape2d<T>& points_a, const shape2d<T>& points_b);

	/**
	* @tparam T Must be either float or double
	* @param shapes
	* @pre shapes.size() > 0
	* @pre every item in shapes is the same size.
	* @returns Mean shape of all the shapes.
	*/
	template<typename T>
	shape2d<T> mean_shape(const std::vector<shape2d<T>>& shapes);

	/**
	* @brief Compute the mean point of a shape.
	* @tparam T Must be either float or double
	* @param shape
	* @pre shape.size() > 0
	* @returns Mean point of the shape.
	*/
	template<typename T>
	point2d<T> mean_point(const shape2d<T>& shape);

	/**
	* @brief Casts all points in a vector of points to have double type
	* @tparam T Must be either float or double
	* @param points
	* @returns Vector of points matching the input vector but cast to double type
	*/
	template<typename T>
	shape2d<double> cast_points_to_double(const shape2d<T>& points);

	/**
	 * @brief Find the "mean shape" of a set using the Procrustes alignment process.
	 * @tparam T Must be either float or double
	 * @param points
	 * @param align_method Alignment type
	 * @pre vectors_all_same_size_with_at_least_n_elements(points_data,2) == true
	 * @returns Mean shape
     * @details The 'mean shape' (in the appropriate sense) of the Procrustes process, for
	 *			the points specified by alignment_ids using the align_method specified.
	 */
	template<typename T>
	shape2d<T> compute_procrustes_base_shape(const std::vector<shape2d<T>>& points, const alignment_type& align_method);

	/**@}*/
}


#include "impl/geometry.hpp"