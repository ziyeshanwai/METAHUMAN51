// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

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
	using dlib::serialize;
	using dlib::deserialize;

	/**
	 * @brief A class representing a set of annotated data
	 */
	class shape_annotation_set
	{
	public:
		shape_annotation_set() {};

		/**
		 * @brief Initializing constructor
		 * @param prototype The prototype image annotation scheme
		 * @param data Actual annotation data
		 * @param n_dense_internals The number of internal points to generate for the dense curves 
		 */
		shape_annotation_set(const shape_annotation& prototype, const std::vector<shape_annotation>& data, const std::map<std::string, int>& n_dense_internals) :
			is_prototype_set_(true), prototype_(prototype), data_(data), n_internals_for_dense_curves_(n_dense_internals){ }

		/**
		* @brief Get a const pointer to a given annotation
		* @returns the pointer which will be nullptr if index is outside the size of the data
		*/
		shape_annotation* const item(size_t index);

		/**
		* @brief Get a const pointer to a given rough annotation
		* @returns the pointer which will be nullptr if index is outside the size of the data
		*/
		shape_annotation* const rough_item(size_t index);

		/**
		* @brief Get a const pointer to the prototype annotation
		* @returns the const pointer (i.e. pointer cannot be later set to nullptr or modified in some way,
		*                             but the prototype data itself can be - e.g. in a markup app)
		*							 Value will be nullptr if the prototype hasn't been set.
		*/
		shape_annotation* const prototype();

		/**
		* @brief Set the  data for this set
		* @param data Vector of annotated images
		*/
		void set_all_data(const std::vector<shape_annotation>& data);

		/**
		 * @brief Remove an example
		 * @param index The index of the example to remove
		 */
		void remove_example_at_index(int index);

		/**
		 * @brief Remove all examples
		 */
		void remove_all_examples();

		/**
		* @brief Set the  data for this set
		* @param data Vector of annotated images
		*/
		void add_example(const shape_annotation& data);

		/**
		* @brief Add more  data to this set
		* @details This merges the data in 'other' with the existing data, sorting the result and removing duplicates
		* @param data An additional shape annotation set
		*/
		void merge_data(const shape_annotation_set& other);

		/**
		 * @brief Set the prototype annotation
		 * @param exemplar the prototype (usually has an image filename indicating a dummy image)
		 */
		void set_prototype(const shape_annotation& exemplar);


		/**
		 * @brief Initialize manually
		 * @param prototype The prototype image annotation scheme
		 * @param data Actual annotation data
		 * @param n_dense_internals The number of internal points to generate for the dense curves
		 */
		void initialize(const shape_annotation& prototype, const std::vector<shape_annotation>& data,
			const std::map<std::string, int>& n_dense_internals);


		/**
		 * @brief Check if the prototype has been set for the refiner
		 * @returns true if set_prototype has previously been called
		 */
		bool is_prototype_set() const;

		/*
		* @brief Check that all the examples have the same keypoint and curve structure and match the prototype
		* @param[out] first_error Description of the first inconsistency detected
		* @returns true if a prototype exists and all examples are consistent with it and each other
		*/
		bool is_consistent(std::string& first_error) const;

		/**
		 * @brief Get the number of  examples
		 * @returns the number of examples
		 */
		int n_examples() const;

		/**
		 * @brief Sort the items in the  set according to the image filename
		 * @details This is generally an alphanumeric sort on the filename, but with a couple
		 *          of tweaks: slashes are removed first so they don't affect things, and trailing
		 *          numbers (not including the file extension) are parsed into real numbers and
		 *          sorted accordingly so that 2 comes before 10 (for example). File extensions are ignored.
		 */
		void sort_data();

		/**
		 * @brief Removes data corresponding the same image from the  set
		 */
		void remove_duplicates();

		/**
		 * @brief Removes data where any of the points are nan
		 */
		void remove_bad_data();

		/**
		 * @brief Removes the listed curve names from the set along with any 'hanging'
		 *        keypoints that are left without a curve
		 */
		void remove_curves(const std::vector<std::string>& curve_names);

		/**
		 * @brief Removes all free (not part of any curve) points from the markup data
		 */
		void remove_free_points();

		/**
		 * @brief Get a const reference to all the markup data
		 * @return The const reference
		 */
		const std::vector<shape_annotation>& data() const;
		
		/**
		 * @brief Get a const reference to the number of internal points to generate for the dense curves
		 * @return The const reference
		 */
		const std::map<std::string, int>& n_internals_for_dense_curves() const;

		/**
		 * @brief Set the number of internal points to generate for the dense curves
		 * @param n_internals The number of internal points to generate for the dense curves
		 */
		void set_n_internals_for_dense_curves(const std::map<std::string, int>& n_internals);


		/**
		 * @brief Serialization
		 * @param item The shape_annotation_set
		 * @param out The output stream
		 */
		friend void serialize(const shape_annotation_set& item, std::ostream& out);

		/**
		 * @brief Deserialization
		 * @param item The resulting shape_annotation_set
		 * @param in The input stream
		 */
		friend void deserialize(shape_annotation_set& item, std::istream& in);

	private:
		bool is_prototype_set_ = false;
		shape_annotation prototype_;
		std::vector<shape_annotation> data_;
		std::vector<shape_annotation> rough_data_;
		std::map<std::string, int> n_internals_for_dense_curves_;

	};

}


