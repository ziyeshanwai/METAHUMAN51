// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "basic_types.h"
#include "image_annotation.h"

namespace rlibv
{
	/**
	* \defgroup Utils Utilities
	* @{
	*/

	/**
	 * @brief Checks if two maps shared the same keys
	 * @param a the first map
	 * @param b the second map
	 * @returns True if the keys in a match the keys in b
	 */
	template<typename T, typename U, typename V>
	bool maps_have_same_keys(const std::map<T, U>& a, const std::map<T, V>& b);

	/**
	 * @brief Checks if map has a key
	 * @param m the map
	 * @param k the key
	 * @returns True iff map contains key
	 */
	template<typename T, typename U>
	bool map_contains_key(const std::map<T, U>& m, const T& k);

	/**
	 * @brief Checks if every item in a vector is unique
	 * @tparam T Must be a object with a .size() method
	 * @param x
	 * @returns True if all the items in the vector are unique, else false
	 */
	template<typename T>
	bool is_unique(std::vector<T> x);

	/**
	 * @brief Checks if every item in a vector is less-than a given value.
	 * @tparam T Must support less-than operator (<) e.g. int, float, double, etc.
	 * @param data_set
	 * @param limit
	 * @returns True if all the items in data are less than the limit
	 */
	template<typename T>
	bool all_items_less_than(const std::vector<T>& data_set, const T& limit);

	/**
	 * @brief Check if all the column vectors have the same size
	 * @tparam T Must be either float or double
	 * @param data_set
	 * @return True if each col_vector in the std::vector has the same size, else false.
	 */
	template<typename T>
	bool all_col_vectors_are_same_size(const std::vector<col_vector<T>>& data_set);

	/**
	 * @brief Check if all the 2d shapes in a vector have the same number of points
	 * @tparam T Must be either float or double
	 * @param data_set
	 * @return True iff each shape in the vector has the same number of points.
	 */
	template<typename T>
	bool all_shapes_are_same_size(const std::vector<shape2d<T>>& data_set);

	/**
	 * @brief Check if all shapes in a vector of vectors have the same size, and are bigger than a minimum size
	 * @tparam T Must be either float or double
	 * @param data_set
	 * @param n Minimum size
	 * @returns True iff input has at least one entry, each entry has the same size,
	 *		    and each entry has at least n elements. False if data_set.empty().
	 */
	template<typename T>
	bool shapes_all_same_size_with_at_least_n_elements(const std::vector<shape2d<T>>& data_set, const size_t n);

	/**
	 * @brief  Turns a std::vector of dlib column vectors into a dlib matrix
	 * @tparam T Must be either float or double
	 * @param data_as_col_vectors
	 * @pre data_as_col_vectors.size() > 0;
	 * @pre	all_col_vectors_are_same_size(data_as_col_vectors);
	 * @pre	data_as_col_vectors[0].size() > 0;
	 * @return A dlib matrix, each column of which is the corresponding element of the std::vector
	 */
	template<typename T>
	dlib::matrix<T> col_vectors_to_single_mat(const std::vector<col_vector<T>>& data_as_col_vectors);

	/**
	 * @brief Converts a dlib column matrix to a shape2d
	 * @tparam T Must be either float or double
	 * @param data
	 * @pre data.nr() % 2 == 0
	 * @returns A shape2d of size data.nr() / 2;
	 */
	template<typename T>
	shape2d<T> col_matrix_to_shape2d(const dlib::matrix<T, 0, 1>& data);

	/**
	 * @brief Converts a shape2dtor to a dlib column matrix
	 * @tparam T Must be either float or double
	 * @param shape
	 * @pre shape.size() > 0
	 * @returns A dlib column matrix 
	 * @details Matrix is of the form:
	 *		    - [shape[0].x(), shape[0].y(), shape[1].x(), shape[1].y() ... shape[n].x(), shape[n].y()]
	 *		    - where n is the size of 'shape' (the number of points)
	 */
	template<typename T>
	dlib::matrix<T, 0, 1> shape2d_to_col_matrix(const shape2d<T>& shape);

	/**
	 * @brief Find minimum and maximum values seen in each row of items in a data set
	 * @tparam T Must be either float or double
	 * @param data_set
	 * @param[out] min_result
	 * @param[out] max_result
	 * @pre data_set.size() > 0
	 * @pre all_col_vectors_are_same_size(data_set)
	 */
	template<typename T>
	void element_wise_min_max(const std::vector<col_vector<T>>& data_set, col_vector<T>& min_result,
			col_vector<T>& max_result);

	/**
	 * @brief Returns a subvector consisting of the items in the original vector at the listed indices
	 * @tparam T Must be either float or double
	 * @param input
	 * @param indices
	 * @pre Every element in 'indices' is less than input.size()
	 * @returns A subvector consisting of the items in 'input' at the positions listed in 'indices'
     * @remark Indices needn't contain unique elements so technically the "subvector" can be longer than the input vector.
	 */
	template<typename T>
	std::vector<T> subvector(const std::vector<T>& input, const std::vector<int>& indices);

	/**
	 * @brief Returns an array listing the indices of the elements according to a sort
	 * @tparam T 
	 * @param v Input array.
	 * @param comp A custom comparer (default is std::less<>).
	 * @returns An int array giving the index of the elements according to a sort.
	 */
	template <typename T, int N, typename Comparer = std::less<>>
	std::array<int, N> sort_indices(const std::array<T, N>& v, Comparer comp = Comparer());

	/**
	 * @brief Returns a vector listing the indices of the elements according to a sort
	 * @tparam T
	 * @param v Input vector.
	 * @param comp A custom comparer (default is std::less<>).
	 * @returns An int array giving the index of the elements according to a sort.
	 */
	template <typename T, typename Comparer = std::less<>>
	std::vector<int> sort_indices(const std::vector<T>& v, Comparer comp = Comparer());


	/**@}*/
}

#include "impl/data_utils.hpp"