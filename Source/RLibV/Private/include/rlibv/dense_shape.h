// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "keypoint.h"
#include "keypoint_curve.h"
#include "shape_annotation.h"
#include "data_utils.h"
#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
RLIBV_RENABLE_WARNINGS
#include <vector>
#include <iostream>

namespace rlibv
{

	/**
   	* \defgroup Data Image Annotation
   	* @{
   	*/


	using dlib::serialize;
	using dlib::deserialize;

	/**
	 * @brief A class representing a dense shape formed by connected points
	 */
	class dense_shape
	{
	public:
		dense_shape() {};


		/**
		 * @brief Initializing constructor
		 * @param annotation Catmull-Rom style annotation
		 * @param image_width The dense representation is in real image coordinates so the width and height need to be known
		 * @param image_height See image_width
		 * @param n_dense_internals The number of internal points to generate for the dense curves 
		 */
		dense_shape(const shape_annotation& annotation, const int image_width, const int image_height, const std::map<std::string, int>& n_dense_internals);

		/**
		 * @brief Get the number of dense points
		 * @return The number of points in the dense shape
		 */
		const int n_points() const;

		/**
		 * @brief Get a  reference to the dense points
		 * @return  reference to the dense points
		 */
		std::vector<point2d<double>>& points() ;

		/**
		 * @brief Get a const reference to the dense points
		 * @return  reference to the dense points
		 */
		const std::vector<point2d<double>>& const_points() const;

		/**
		 * @brief Set the internal points if we know them
		 */
		void set_points(const std::vector<point2d<double>>& pts);

		/**
		 * @brief Get the inbound links to a given point
		 * @pre index >= 0 && index < points.size();
		 * @return Const reference to the inbound links
		 */
		const std::vector<int>& inbound_links(int index) const;

		/**
		 * @brief Get the outbound links to a given point
		 * @pre index >= 0 && index < points.size();
		 * @return Const reference to the outbound links
		 */
		const std::vector<int>& outbound_links(int index) const;

		/**
		 * @brief Initialize manually
		 * @param annotation Catmull-Rom style annotation
		 * @param image_width The dense representation is in real image coordinates so the width and height need to be known
		 * @param image_height See image_width
		 * @param n_dense_internals The number of internal points to generate for the dense curves 
		 */
		void initialize_from_shape_annotation(
			const shape_annotation& annotation,
			const int image_width,
			const int image_height,
			const std::map<std::string, int>& n_dense_internals);

		/**
		 * @brief Calculate the tangent lines at each point, scaled to local point spacing.
		 * @return The lines, expressed as a pair of endpoints.
		 */
		std::vector<std::pair<point2d<double>,point2d<double>>> curve_oriented_tangent_lines() const;


		/**
		 * @brief Calculates a square patch scaled to local point spacing.
		 * @return The patches, expressed as an array of corners (clockwise from top-left)
		 */
		std::vector<std::array<point2d<double>,4>> curve_oriented_square_patches(double scale) const;

		/**
		 * @brief Calculates an unrotated square patch scaled to local point spacing.
		 * @return The patches, expressed as an array of corners (clockwise from top-left)
		 */
		std::vector<std::array<point2d<double>, 4>> unaligned_square_patches(double scale) const;

		/**
		 * @brief Calculate the normal lines at each point, scaled to local point spacing.
		 * @return The lines, expressed as a pair of endpoints.
		 */
		std::vector<std::pair<point2d<double>, point2d<double>>> curve_orientied_normal_lines() const;

		/**
		 * @brief Calculate the normal vector at each point, scaled to local point spacing.
		 * @return The normal vectors
		 */
		std::vector<point2d<double>> curve_oriented_normal_vectors() const;

		/**
		 * @brief Calculate the tangent vector at each point, scaled to local point spacing.
		 * @return The normal vectors
		 */
		std::vector<point2d<double>> curve_oriented_tangent_vectors() const;

		/**
		 * @brief Serialization
		 * @param item The dense_shape
		 * @param out The output stream
		 */
		friend void serialize(const dense_shape& item, std::ostream& out);

		/**
		 * @brief Deserialization
		 * @param item The resulting dense_shape
		 * @param in The input stream
		 */
		friend void deserialize(dense_shape& item, std::istream& in);

	private:
		std::vector<point2d<double>> points_;
		std::vector<std::vector<int>> inbound_links_;
		std::vector<std::vector<int>> outbound_links_;
	};

	void serialize(const dense_shape& item, std::ostream& out);

	void deserialize(dense_shape& item, std::istream& in);

	/**@}*/
}


