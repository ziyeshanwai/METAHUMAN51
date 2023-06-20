// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "basic_types.h"

namespace rlibv
{
	using dlib::serialize;
	using dlib::deserialize;

	/**
   	* \defgroup Tracking Shape Tracking
   	* @{
   	*/

	/**
	 * @brief A class representing a curve with a number of anchor points
	 * @details The curve is represented by the all the control points, but some of the control
	 *          points can also be marked as 'anchor' points. This allows users to reinterpret
	 *          curves in the future with the understanding that certain points have a fixed and
	 *          specific meaning - for example, the curve might represent the outer edge of a
	 *          lip, with an extra anchor point specifically defining the mid point.
	 * @tparam T T must be either float or double
	 */
	template<typename T>
	class anchored_curve
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
	public:
		anchored_curve() = default;

		/**
		 * @brief Initializing Constructor
		 * @param control_points
		 * @param extra_anchor_point_indices The indices indicating the indices of any extra anchor points. Note
		 *        that the first and last points are automatically considered to be anchor points.
		 * @param closed if true then the two end points are considered connected
		 * @pre every item in extra_anchor_point_indices must be >1 and < control_points.size()-1
		 */
		anchored_curve(const rlibv::shape2d<T>& control_points, const std::vector<int>& extra_anchor_point_indices, bool closed);

		/**
		* @brief Add an anchor point to the end of the curve
		* @param point The location of the new anchor points
		*/
		void append_anchor_point(const rlibv::point2d<T>& point);

		/**
		 * @brief Get the control points by constant reference
		 * @return reference to control points
		 */
		const rlibv::shape2d<T>& control_points_const_ref() const;

		/**
		 * @brief Get the control points by mutable reference
		 * @return reference to control points
		 */
		rlibv::shape2d<T>& control_points_ref();

		/**
		 * @brief Get the anchor point indices by constant reference
		 * @return constant reference to anchor point indices
		 */
		const std::vector<int>& anchor_point_indices_const_ref() const;

		/**
		 * @brief Get the anchor point indices by mutable reference
		 * @return reference to anchor point indices
		 */
		std::vector<int>& anchor_point_indices_ref();

		/**
		 * @brief Get reference to whether the curve is closed
		 * @return reference to closure state of the curve
		 */
		bool closed() const;

		/**
		 * @brief Closes the curve, removing the last control point if it's the same as the first.
		 */
		void make_closed();
	
		/**
		 * @brief Get preferred_display_colour
		 * @return RGBA colour
		 */
		std::array<unsigned char,4> get_preferred_display_colour_uchar() const;

		/**
		 * @brief Get preferred_display_colour as a pointer to the float array with 4 elements
		 * @return Pointer to a 4-element float array
		 */
		float* preferred_display_colour();

		/**
		 * @brief Clear all the contents of an anchored curve
		 * @return Pointer to a 4-element float array
		 */
		void clear();

		/**
		* @brief Indicates if this curve is visible (vs hidden) on the specific image
		*        it sits on.
		* @return True if visible
		*/
		bool visible() const;

		/**
		 * @brief Set a curve's visibility (i.e. can it be actually seen in the image)
		 */
		void set_visible(bool is_visible);

		/**
		 * @brief Serialization
		 * @tparam U must be either float or double
		 * @param item
		 * @param out
		 */
		template<typename U>
		friend void serialize(const anchored_curve<U>& item, std::ostream& out);

		/**
		 * @brief Deserialization
		 * @tparam U must be either float or double
		 * @param item
		 * @param in
		 */
		template<typename U>
		friend void deserialize(anchored_curve<U>& item, std::istream& in);

	private:
		rlibv::shape2d<T> control_points_;
		std::vector<int> anchor_point_indices_;
		float preferred_display_colour_[4] = {0,1,0,1};
		bool closed_ = false;
		bool visible_ = true;
	};



	/**@}*/
}


#include "impl/anchored_curve.hpp"