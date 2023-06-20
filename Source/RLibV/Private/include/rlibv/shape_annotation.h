// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "keypoint.h"
#include "keypoint_curve.h"
#include "data_utils.h"
#include "disable_dlib_warnings.h"

#include "string_functions.h"

RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/global_optimization.h>
RLIBV_RENABLE_WARNINGS
#include <vector>
#include <iostream>

namespace rlibv
{
	enum class annotation_state
	{
		ok,
		smoothing_keypoint_has_more_than_two_curves,
		unknown_error
	};

	/**
   	* \defgroup Data Image Annotation
   	* @{
   	*/

	using dlib::serialize;
	using dlib::deserialize;
	
	struct simple_spline 
	{
		std::vector<rlibv::point2d<double>> pts;
		std::vector<int> internal_control_point_inds;
	};

	enum class curve_connection_direction
	{
		to_start_of_other_curve,
		to_end_of_other_curve
	};

	struct curve_connection
	{
		std::string other_curve_name;
		curve_connection_direction direction;
	};

	/**
	 * @brief A class representing a shape annotated with named curves and keypoints
	 */
	class shape_annotation
	{
	public:
		shape_annotation();

		/**
		 * @brief Initializing constructor
		 * @param image_filename The image to which the shape annotation applies.
		 * @param keypoints Map of named keypoints, each of which can be a smooth or sharp point
		 * @param keypoint_curves Map of curves connected to keypoints
		 * @pre validate_annotation_data(keypoints,keypoint_curves) == annotation_state::ok
		 */
		shape_annotation(
			const std::string& image_filename,
			const std::map<std::string, keypoint>& keypoints,
			const std::map<std::string, keypoint_curve>& keypoint_curves);

		/**
		 * @brief Initialize manually
		 * @param image_filename The image to which the shape annotation applies.
		 * @param keypoints Map of named keypoints, each of which can be a smooth or sharp point
		 * @param keypoint_curves Map of curves connected to keypoints
		 * @pre validate_annotation_data(keypoints,keypoint_curves) == annotation_state::ok
		 */
		void initialize(
			const std::string& image_filename,
			const std::map<std::string, keypoint>& keypoints,
			const std::map<std::string, keypoint_curve>& keypoint_curves);

		/**
		 * @brief Get a const reference to the keypoints
		 * @returns Const reference to the keypoints
		 */
		const std::map<std::string, keypoint>& keypoints() const;

		/**
		 * @brief Get a const reference to the keypoint curves
		 * @returns Const reference to the keypoint curves
		 */
		const std::map<std::string, keypoint_curve>& keypoint_curves() const;

		/**
		 * @brief Get a mutable reference to the keypoints
		 * @returns Const reference to the keypoints
		 */
		 std::map<std::string, keypoint>& keypoints();

		/**
		 * @brief Get a const reference to the keypoint curves
		 * @returns Const reference to the keypoint curves
		 */
		std::map<std::string, keypoint_curve>& keypoint_curves();

		/**
		 * @brief Insert an internal control point
		 * @param name The name of of the curve
		 * @param insert_before The index ahead of which this point should be inserted. If greater than number
		 *                      of existing control points this will become the last control point.
		 * @param pos The location of the new point
		 * @pre name exists in keypoint_curves()
		 * @pre insert_before <= keypoint_curves().at(name).internal_points.size()+1
		 */
		void insert_internal_point(const std::string& name, int insert_before, const rlibv::point2d<double> pos);

		/**
		 * @brief Clear the internal points of a curve
		 * @param name The name of the curve to clear
		 * @pre name exists in keypoint_curves()
		 */
		void clear_internal_points(const std::string& name);

		/**
		 * @brief Remove an internal point
		 * @param name The name of the curve
		 * @param index The index of the internal point to remove
		 * @pre name exists in keypoint_curves()
		 * @pre index <= keypoint_curves().at(name).internal_points.size()
		 */
		void remove_internal_point(const std::string& name, int index);

		/**
		 * @brief Generate drawable splines
		 * @param points_per_spline The internal number of points to generate in each spline
		 * @returns Map (by curve name) of the splines
		 */
		std::map<std::string, std::vector<point2d<double>>> get_drawing_splines(const std::map<std::string, int>&  points_per_spline) const;

		/**
		 * @brief Get the drawing colours for each spline
		 * @returns Map (by curve name) of the colours
		 */
		std::map<std::string, std::array<unsigned char,3>> get_drawing_colours() const;

		/**
		 * @brief Set the drawing colours for each spline
		 * @param Map (by curve name) of the colours
		 */
		void set_drawing_colours(std::map<std::string, std::array<unsigned char, 3>>& colours);

		/**
		 * @brief Get a const reference to the image filename to which this annotation applies.
		 * @returns Const reference to the filename
		 */
		const std::string& image_filename() const;

		/**
		 * @brief Change the image filename
		 * @param filename The new filename
		 */
		void set_image_filename(const std::string& filename);

		/**
		 * @brief Verifies that some map has the same keys as the curves stored in the annotation
		 * @details This should be used to verify the preconditions of various functions
		 * @return true if all correct
		 */
		template<typename U>
		bool all_curve_names_correct(const std::map<std::string, U>& input_map) const;

		/**
		 * @brief Extract a dense representation of the shape annotation
		 * @param image_width The dense representation is in real image coordinates so the width and height need to be known
		 * @param image_height See image_width
		 * @param[out] inbound_links Index or indices of the inbound connections for each dense point 
		 * @param[out] outbound_links Index or indices of the inbound connections for each dense point 
		 * @param[out] curve_lookup A map of giving the indices corresponding to each curve
		 * @param internal_densities The internal densities for the exported curves
		 * @pre the keys internal densities match one-to-one with the curve names in the annotation, i.e.
		 *      all_curve_names_present(internal_densities)
		 * @returns The dense shape
		 */
		std::vector<point2d<double>> get_dense_points(const int image_width,
			const int image_height,
			std::vector<std::vector<int>>& inbound_links,
			std::vector<std::vector<int>>& outbound_links,
			std::map<std::string, std::vector<int>>& curve_lookup,
			std::map<std::string, int>& keypoint_lookup,
			const std::map<std::string, int>& internal_densities) const;

		/**
		 * @brief Extract a dense representation of the shape annotation with no additional info
		 * @param image_width The dense representation is in real image coordinates so the width and height need to be known
		 * @param image_height See image_width
		 * @param internal_densities The internal densities for the exported curves
		 * @pre the keys internal densities match one-to-one with the curve names in the annotation, i.e.
		 *      all_curve_names_present(internal_densities)
		 * @returns The dense shape
		 */
		std::vector<point2d<double>> get_dense_points(const int image_width,
			const int image_height,
			const std::map<std::string, int>& internal_densities) const;

		/**
		 * @brief Convert a dense representation into an annotation
		 * @param dense_shape The vector of points in normalized coordinates
		 * @param curve_lookup The lookup table for each curve in the dense data. 
		 *                     The first index of each vector is the curve start point, the last is the end point.
		 *                     The others are the intermediate points.
		 * @pre the keys in curve_lookup match one-to-one with the curve names in the annotation, i.e.
		 *      all_curve_names_present(curve_lookup)
		 */
		void set_from_dense_points(const std::vector<point2d<double>>& dense_shape, const std::map<std::string, std::vector<int>>& curve_lookup);

		/**
		 * @brief Extract a sparse representation of the dense shape annotation using catmull-rom splines.
		 * @param internal_densities The internal density(i.e. without endpoints) required for each curve.
		 * @param image_width The dense representation is in real image coordinates so the width and height need to be known
		 * @param image_height See image_width
		 * @param[out] inbound_links Index or indices of the inbound connections for each dense point
		 * @param[out] outbound_links Index or indices of the inbound connections for each dense point
		 * @pre the keys internal densities match one-to-one with the curve names in the annotation, i.e.
		 *      all_curve_names_present(internal_densities)
		 * @returns The internal points to use for the catmull-rom spline
		 */
		void get_sparse_points(const std::vector<rlibv::point2d<double>> dense_shape,
			const int image_width,
			const int image_height,
			const std::map<std::string, std::vector<int>>& curve_lookup,
			std::map<std::string, int>& internal_densities,
			const int max_internal_points = 2,
			int max_n_func_calls = 100,
			const float point_tolerance = 0.05,
			const double error_func_epsilon = 1e-3);

		/**
		 * @brief Thin the curves by removing some internal points
		 * @param factor The approximate reduction in the density (e.g. 0.5f would produce curves half as dense)
		 * @param internal_densities The internal densities for the exported curves
		 */
		void sparsify_simple(double factor, const std::map<std::string, int>& internal_densities);

		/**
		 * @brief Return a pointer to the position of a point in this annotation
		 * @param name The name of EITHER the key point, or the curve
		 * @param index The index of the intermediate point if the request is for one
		 *              If index == -1 this means you want an intermediate point on a keypoint curve, 
		 *              rather than a keypoint
		 * @returns A pointer to the point's position, or nullptr if the request point isn't found
		 */
		point2d<double>* get_position_pointer(const std::string& name, int index);

		/**
		* @brief Check if the scheme of this shape annotation matches the scheme of another
		* @details That is, this scheme and the other must contain the same keypoints and keypoint curves
		*          but (obviously) they don't need to be in the same positions, just same connectivity etc
		* @param other The other shape_annotation
		* @param[out] first_error The function populates this string with a description of the first problem found.
		* @return true iff the scheme matches this
		*/
		bool matches_scheme(const shape_annotation& other, std::string& first_error) const;

		/**
		 * @brief Verify that the given set of keypoints and curves makes for valid annotation
		 * @returns annotation_state enum : 'ok' indicates the data is good.
		 */
		static annotation_state validate_annotation_data(
			const std::map<std::string, keypoint>& keypoints,
			const std::map<std::string, keypoint_curve>& keypoint_curves);

		/**
		 * @brief Serialization
		 * @param item The shape_annotation
		 * @param out The output stream
		 */
		friend void serialize(const shape_annotation& item, std::ostream& out);

		/**
		 * @brief Deserialization
		 * @param item The resulting shape_annotation
		 * @param in The input stream
		 */
		friend void deserialize(shape_annotation& item, std::istream& in);

		friend void alt_deserialize(shape_annotation& item, std::istream& in);

	private:
		std::string image_filename_ = "none";
		std::map<std::string, keypoint> keypoints_;
		std::map<std::string, keypoint_curve> keypoint_curves_;
		

		curve_connection incoming_connection(const std::string& curve_name) const;
		curve_connection outgoing_connection(const std::string& curve_name) const;

		point2d<double> first_point_before_end(const std::string& curve_name) const;
		point2d<double> first_point_after_start(const std::string& curve_name) const;

		point2d<double> dummy_first_point(const std::string& curve_name) const;
		point2d<double> dummy_last_point(const std::string& curve_name) const;

	};

	template<typename U>
	bool rlibv::shape_annotation::all_curve_names_correct(const std::map<std::string, U>& input_map) const
	{
		return rlibv::maps_have_same_keys(input_map, keypoint_curves_);
	}

	/** Custom comparator so vectors of shape annotations can be sorted */
	struct shape_annotation_less_than
	{
		bool operator ()(const shape_annotation& sa1, const shape_annotation& sa2);
	};

	/** Custom comparator for testing in annotation refer to the same image */
	bool refer_to_same_image(const shape_annotation& sa1, const shape_annotation& sa2);


	/**@}*/
}


