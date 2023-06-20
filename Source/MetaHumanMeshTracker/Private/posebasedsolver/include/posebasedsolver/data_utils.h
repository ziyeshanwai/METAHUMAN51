// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include "data_types.h"
#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
POSEBASEDSOLVER_RENABLE_WARNINGS

namespace cm
{
    using namespace dlib;

    template <template<typename...> class R = std::vector,
        typename Top,
        typename Sub = typename Top::value_type>
        R<typename Sub::value_type> flatten(Top const& all)
    {
        using std::begin;
        using std::end;

        R<typename Sub::value_type> accum;

        for (const auto &sub : all)
            accum.insert(end(accum), begin(sub), end(sub));

        return accum;
    }

	/*!
	   Amends the input vector by appending the item to it n times

	   - Requires:
		   -
	   - Ensures:
		   - vec.size() becomes [input] vec.size()+n where the last n elements are copies of item.
   */
	template<typename T>
	void append_item_to_vector_multiple_times(std::vector<T>& vec, const T& item, int n)
	{
		std::vector<T> vec_to_append(n,item);
		vec.insert(vec.end(),vec_to_append.begin(),vec_to_append.end());
	}

	/*!
	   Returns vector of vectors where each inner vector contains one item from each of the input vectors

	   - Requires:
		   -
	   - Ensures:
		   - the result is vector of vectors where each inner vector contains one item from each of the input vectors
		   - e.g.  if the input is {{1,2,3},{4,5}} the output is {{1,4},{2,4},{3,4},{1,5},{2,5},{3,5}}
   */
	template<typename T>
	std::vector<std::vector<T>> all_combinations_of_one_item_from_each_vector(const std::vector<std::vector<T>>& a)
	{
		std::vector<std::vector<T>> r(1);
		for (const auto& x : a)
		{
			std::vector<std::vector<T>> t;
			for (const T& y : x)
			{
				for (const std::vector<T>& i : r)
				{
					std::vector<T> ia(i);
					ia.emplace_back(y);
					t.emplace_back(ia);
				}
			}
			r = std::vector<std::vector<T>>(t);
		}
		return r;
	}

    /*!
        Returns a subvector consisting of the items in the original vector at the listed indices

        - Requires:
            - every item in 'indices' is less than input.size()
        - Ensures:
            - Returns a subvector consisting of the items in 'input' at the positions listed
              in 'indices'
            - Note that indices needn't contain unique elements so technically the "subvector"
              can be longer than the input vector.
    */
    template<typename T>
    std::vector<T> subvector(const std::vector<T>& input, const std::vector<int>& indices)
    {
        DLIB_ASSERT(std::all_of(indices.begin(), indices.end(), [input](size_t i) {return i < input.size(); }),
            "\t subvector(...)"
            << "\n\t Perhaps some of the items in 'indices' where larger than the size of 'input'");

        std::vector<T> output(indices.size());
        auto count = 0;
        for(const auto &ind : indices)
        {
            output[count++] = input[ind];
        }
        return output;
    }


	 /*
	    Splits a string into a vector of strings according to a delimiter string.

	    - Ensures:
	        - Returns vector of strings - input split according to a delimiter string.
	        - If no instances of the delimiter are found, the vector will contain just the original string
	        - The returned strings do not contain the delimiter itself.
	*/
	inline std::vector<std::string> split_by_string(const std::string &input, const std::string &delimiter)
	{
		std::vector<std::string> result;
		result.reserve(4); // little boost for most common input such as 'frame_number,point_id,x,y'

		size_t begin_pos = 0, end_pos = 0, delimiter_len = delimiter.length();

		while ((end_pos = input.find(delimiter, begin_pos)) != std::string::npos)
		{
			result.push_back(input.substr(begin_pos, end_pos - begin_pos));

			begin_pos = end_pos + delimiter_len;
		}

		if (begin_pos < input.size())
		{
			result.push_back(input.substr(begin_pos));
		}

		return result;
	}

    /*
        It collapses a vector of vectors to a single vector. The order of the elements is not changed.

        - Ensures:
            - Returns a vector of strings.
    */
    template <typename T>
    std::vector<T> collapse_vectors(std::vector<std::vector<T>> input_vec)
    {
        std::vector<T> output_vec;
        std::for_each(input_vec.begin(), input_vec.end(), [&](std::vector<T> i) {output_vec.insert(output_vec.end(), i.begin(), i.end());});

        return output_vec;
    }

    /*!
         Calculates Hamming distance between two unsigned integers.
    */
    inline int calc_hamming_dist(unsigned patch1, unsigned patch2)
    {
        unsigned xor_res = patch1 ^ patch2;
        int hamming_dist = 0;
        while (xor_res > 0)
        {
            hamming_dist += xor_res & 1;
            xor_res >>= 1;
        }

        return  hamming_dist;
    }

    /*!
          Calculates an rms point to point error between 2 sets of points.

          - Requires:
              - points_1.size() == points_2.size()
          - Ensures:
              - Returns the rms errors per point for current frame.
     */
    inline std::vector<double> calc_rms_point_error(const point2_vector<double>& points_1, const point2_vector<double>& points_2)
    {
        DLIB_ASSERT(points_1.size() == points_2.size(),
            "\n\t Invalid inputs were given to calculate_error_per_point function.");

        std::vector<double> error_per_point;
        for (std::size_t idx = 0; idx < points_1.size(); idx++)
        {
            double tmp = sqrt((points_1[idx].x() - points_2[idx].x()) * (points_1[idx].x() - points_2[idx].x()) + (points_1[idx].y() - points_2[idx].y()) * (points_1[idx].y() - points_2[idx].y()));
            error_per_point.emplace_back(tmp);
        }
        return error_per_point;
    }

    /*!
        Returns the indices of the elements according to a sort

        - Requirements:
            - Comparer must be a functor implementing the (const T&, const T&) -> bool operator
              which compares two elements T belonging to b, deciding in which order to sort the indices.
        - Ensures:
            - Returns a vector of indices giving the sort order of the elements in v
            - For example sort_indices({3,4,1}) would return {2,0,1}
            - For example sort_indices({3,4,1}, std::greater<>()) would return {1,0,2}
    */
    template <typename T, typename Comparer = std::less<>>
    std::vector<int> sort_indices(const std::vector<T> &v, Comparer comp = Comparer())
    {
        // initialize original index locations
        std::vector<int> idx(v.size());
        for (unsigned i = 0; i != idx.size(); ++i) idx[i] = i;

        // sort indexes based on comparing values in v
        std::sort(idx.begin(), idx.end(),
            [&v, &comp](int i1, int i2) {return comp(v[i1], v[i2]); });

        return idx;
    }

    /*!
      Returns the indices of the elements according to a partial sort for the top number_to_sort values

      - Requirements:
          - Comparer must be a functor implementing the (const T&, const T&) -> bool operator
            which compares two elements T belonging to b, deciding in which order to sort the indices.
      - Ensures:
          - Returns a vector of indices giving the sort order of the elements in v
          - For example sort_indices({3,4,1}) would return {2,0,1}
          - For example sort_indices({3,4,1}, std::greater<>()) would return {1,0,2}
    */
    template <typename T, typename Comparer = std::less<>>
    std::vector<int> partial_sort_indices(const std::vector<T> &v, unsigned number_to_sort, Comparer comp = Comparer())
    {
        // initialize original index locations
        std::vector<int> idx(v.size());
        for (unsigned i = 0; i != idx.size(); ++i) idx[i] = i;

        // sort indexes based on comparing values in v
        std::partial_sort(idx.begin(), idx.begin() + number_to_sort, idx.end(),
            [&v, &comp](int i1, int i2) {return comp(v[i1], v[i2]); });

        return idx;
    }

	/*!
	   Returns the indices of the elements according to a sort

	   - Requirements:
		   - Comparer must be a functor implementing the (const T&, const T&) -> bool operator
			 which compares two elements T belonging to b, deciding in which order to sort the indices.
	   - Ensures:
		   - Returns a vector of indices giving the sort order of the elements in v
		   - For example sort_indices({3,4,1}) would return {2,0,1}
		   - For example sort_indices({3,4,1}, std::greater<>()) would return {1,0,2}
   */
	template <typename T, typename Comparer = std::less<>>
	std::vector<int> sort_indices(const col_vector<T> &v, Comparer comp = Comparer())
	{
		// initialize original index locations
		std::vector<int> idx(v.size());
		for (unsigned i = 0; i != idx.size(); ++i) idx[i] = i;

		// sort indexes based on comparing values in v
		std::sort(idx.begin(), idx.end(),
			[&v, &comp](int i1, int i2) {return comp(v(i1), v(i2)); });

		return idx;
	}

    /*!
        Converts a point2_vector to a dlib column matrix

        - Ensures:
            - Returns a dlib column matrix of the form:
            - [shape[0].x(), shape[0].y(), shape[1].x(), shape[1].y() ... shape[n].x(), shape[n].y()]
            - where n is the size of 'shape' (the number of points)
    */
    template<typename T>
    dlib::matrix<T, 0, 1> point2_vector_to_col_matrix(const point2_vector<T>& shape)
    {
        matrix<T, 0, 1> pts_data(shape.size() * 2);
        for (auto p = 0; p < static_cast<int>(shape.size()); ++p)
        {
            pts_data(p * 2) = shape[p].x();
            pts_data(p * 2 + 1) = shape[p].y();
        }
        return pts_data;
    }

    /*!
        Converts a dlib column matrix to a point2_vector
        - Requires
            - data.nr() % 2 == 0
        - Ensures:
            - Returns a point2_vector of size data.nr()/2;
    */
    template<typename T>
    point2_vector<T> col_matrix_to_point2_vector(const dlib::matrix<T, 0, 1>& data)
    {
        DLIB_ASSERT(data.nr() % 2 == 0,
            "\t col_matrix_to_point2_vector(const dlib::matrix<T, 0, 1>& data)"
            << "\n\t Invalid column matrix was passed to this function."
            << "\n\t Data should have even number of elements."
            << "\n\t data.nr(): " << data.nr());

        point2_vector<T> shape(data.size() / 2);
        for (auto p = 0; p < static_cast<int>(data.size() / 2); ++p)
        {
            shape[p] = point2<T>(data(p * 2), data(p * 2 + 1));
        }
        return shape;
    }

    /*!
        Converts a dlib row matrix to a point2_vector
        - Requires
            - data.nc() % 2 == 0
        - Ensures:
            - Returns a point2_vector of size data.nr()/2;
    */
    template<typename T>
    point2_vector<T> row_matrix_to_point2_vector(const dlib::matrix<T, 1, 0>& data)
    {
        DLIB_ASSERT(data.nc() % 2 == 0,
            "\t row_matrix_to_point2_vector(const dlib::matrix<T, 1, 0>& data)"
            << "\n\t Invalid column matrix was passed to this function."
            << "\n\t Data should have even number of elements."
            << "\n\t data.nc(): " << data.nc());

        point2_vector<T> shape(data.size() / 2);
        for (auto p = 0; p < static_cast<int>(data.size() / 2); ++p)
        {
            shape[p] = point2<T>(data(p * 2), data(p * 2 + 1));
        }
        return shape;
    }


	/*!
		Converts a point3_vector to a dlib column matrix.
        See also col_matrix_to_point3_vector (the inverse operation)

		- Ensures:
			- Returns a dlib column matrix of the form:
			- [shape[0].x(), shape[0].y(), shape[0].z(), shape[1].x(), shape[1].y(), shape[1].z() ... shape[n-1].x(), shape[n-1].y(), shape[n-1].z()]
			- where n is the size of 'shape' (the number of points)
	*/
	template<typename T>
	dlib::matrix<T, 0, 1> point3_vector_to_col_matrix(const point3_vector<T>& shape)
	{
		matrix<T, 0, 1> pts_data(shape.size() * 3);
		for (unsigned p = 0; p < shape.size(); ++p)
		{
			pts_data(p * 3) = shape[p].x();
			pts_data(p * 3 + 1) = shape[p].y();
			pts_data(p * 3 + 2) = shape[p].z();
		}
		return pts_data;
	}

	/*!
	 Converts a dlib column matrix to a point3_vector.
     See also point3_vector_to_col_matrix (the inverse operation)

	 - Requires
		 - data.nr() % 3 == 0
	 - Ensures:
		 - Returns a point3_vector of size data.nr()/3;
	*/
	template<typename T>
	point3_vector<T> col_matrix_to_point3_vector(const dlib::matrix<T, 0, 1>& data)
	{
		DLIB_ASSERT(data.nr() % 3 == 0,
			"\t col_matrix_to_point3_vector(const dlib::matrix<T, 0, 1>& data)"
			<< "\n\t Invalid column matrix was passed to this function."
			<< "\n\t Data should have number of elements divisible by 3."
			<< "\n\t data.nr(): " << data.nr());

		point3_vector<T> shape(data.size() / 3);
		for (int p = 0; p < data.size() / 3; ++p)
		{
			shape[p] = point3<T>(data(p * 3), data(p * 3 + 1), data(p * 3 + 2));
		}
		return shape;
	}

    /*!
        Apply a piecewise linear mapping to a value

        - REQUIREMENTS ON T
		    - Must be either float, double, or long double, ie:
			  is_float_type<T>::value == true
        - Requires:
            - joints.size() > 2
            - joints.rbegin()->first > joints.begin()->first (i.e. the keys are not identical)
        - Ensures:
            - Returns the original value x mapped through the pwl transform defined in 'joints'.
            - The mapping will extrapolate outside the range of the joints.
            - Each key in 'joints' is a sample input value point, each value is the desired output
              at that given input.
    */
    template<typename T>
    double piecewise_linear_transform(const std::map<T, T>& joints, T x)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);
        DLIB_ASSERT(joints.size() > 1
            && joints.rbegin()->first > joints.begin()->first,
            "\t double piecewise_linear_transform(const std::map<T, T>& joints, T& input_val)"
            << "\n\t Invalid joints were passed to this function.");

        auto it = joints.lower_bound(x);
        if (it == joints.end())
        {
            it = joints.begin();
        }
        else if (it != joints.begin())
        {
            --it;
        }
        const auto low_in = it->first;
        const auto low_out = it->second;

        ++it;
        auto high_in = it->first;
        auto high_out = it->second;


        const auto m = (high_out - low_out) /
            (high_in - low_in);
        const auto c = low_out - m * low_in;

        return m * x + c;
    }

    /*!
    *   Checks if every item in a vector is less-than a given value.
    *
    *    - REQUIREMENTS ON T
    *	      - Must support less-than operator (<)
    *    - Ensures
    *       - Returns true if all the items in data are less than the limit
    */
    template <class T>
    bool all_items_less_than(const std::vector<T>& data, const T& limit)
    {
        for (const auto& item : data)
        {
            if (item >= limit)
            {
                return false;
            }
        }
        return true;
    }

    /*!
    *   Finds any duplicates in a string vector.
    *
    *    - Ensures
    *       - Returns a map with the duplicate point ids and their indices in the input vector.
    */
    inline std::map<std::string, std::vector<unsigned>> find_duplicate_ids(const std::vector<std::string>& point_ids)
    {
        std::map<std::string, std::vector<unsigned>> duplicate_ids;

        for (unsigned idx1 = 0; idx1 < point_ids.size() - 1; idx1++)
        {
            for (unsigned idx2 = idx1 + 1; idx2 < point_ids.size(); idx2++)
            {
                if (point_ids[idx1] == point_ids[idx2])
                {
                    //The point id is not already found in the map.
                    if (duplicate_ids.find(point_ids[idx1]) == duplicate_ids.end())
                    {
                        duplicate_ids.insert({ point_ids[idx1], {idx1, idx2} });
                    }
                    else
                    {
                        duplicate_ids.at(point_ids[idx1]).emplace_back(idx2);
                    }
                    break;
                }
            }
        }
        return duplicate_ids;
    }

    /*!
        Checks if every item in a vector is unique

        - Ensures
            - Returns true if all the items in the vector are unique, else false
    */
    template <class T>
    bool is_unique(std::vector<T> x)
    {
        std::set<T> y(x.begin(), x.end());
        return x.size() == y.size();
    }

    /*!
     Checks if all keys in one map are present in another map

     - Ensures
         - Returns true if all the keys in key_map are keys of input_map
           else returns false
    */
    template<typename T1, typename T2, typename K>
    bool all_keys_present(const std::map<K, T1>& input_map, const std::map<K, T2> & key_map)
    {
        for (const auto& item : key_map)
        {
            if (input_map.find(item.first) == input_map.end())
            {
                return false;
            }
        }
        return true;
    }


    /*!
        Checks if every item in a list of keys exists as keys of a map

        - Ensures
            - Returns true if all the items in keys are keys of input_map
              else returns false
    */
    template<typename T, typename K>
    bool all_keys_present(const std::map<K, T>& input_map, const std::vector<K>& keys)
    {
        for(const auto& key : keys)
        {
            if(input_map.find(key)==input_map.end())
            {
                return false;
            }
        }
        return true;
    }

    /*!
        Check if any extra keys present in the map which aren't in the list of keys

        - Ensures
            - Returns true if there are extra keys present in the map which aren't in the list of keys, false otherwise
    */
    template<typename T, typename K>
    bool extra_keys_present(const std::map<K, T>& input_map, const std::vector<K>& keys)
    {
        std::map < K, bool > extra;
        for (const auto& item : input_map)
        {
            extra[item.first] = true;
        }

        for (const auto& key : keys)
        {
            extra[key] = false;
        }

        for (const auto & item : extra)
        {
            if (item.second)
            {
                return true;
            }
        }
        return false;
    }

    /*!
        Find any keys that are not keys of the given map

        - Ensures
            - Returns a vector of items in keys which are missing from input map.
    */
    template<typename T, typename K>
    std::vector<K> find_missing_keys(const std::map<K, T>& input_map, const std::vector<K>& keys)
    {
        std::vector<K> missing;
        for (const auto& key : keys)
        {
            if (input_map.find(key) == input_map.end())
            {
                missing.push_back(key);
            }
        }
        return missing;
    }


    /*!
       Check if all keys are same in two maps

       - Ensures
           - Returns true if all keys are the same, false otherwise

    */
    template<typename T, typename K>
    bool map_keys_identical(const std::map<K, T>& map1, const std::map<K, T>& map2)
    {
        if (map1.size() != map2.size())
        {
            return false;
        }

        // check the keys are the same
        auto it1 = map1.begin();
        auto it2 = map2.begin();
        // bool finished = false;
        while (it1 != map1.end())
        {
            if (it1->first != it2->first)
            {
                return false;
            }
            it1++;
            it2++;
        }

        return true;
    }

    /*!
        Creates a vector of the items in a std::map at given keys.

        - Requires
            Every element in keys is a key in input_map,
            i.e. all_keys_present(input_map, keys) == true

        - Ensures
            - Returns a vector of the items in a std::vector at given indices.
            - Return items can be duplicated.
    */
    template<typename T, typename K>
    std::vector<T> get_map_items_at(const std::map<K,T>& input_map, const std::vector<K>& keys)
    {
        DLIB_ASSERT(all_keys_present(input_map, keys),
            "\t std::vector<T> get_map_items_at(const std::map<K,T>& input_map, std::vector<K>& keys)"
            << "\n\t Invalid inputs were given to this function, "
            << "\n\t It would appear that not all the given keys exist in the map. ");

        std::vector<T> result(keys.size());
        auto count = 0;
        for (const auto& key : keys)
        {
            result[count++] = input_map.at(key);
        }
        return result;
    }

    /*!
        Check if all the column vectors have the same size

        - REQUIREMENTS ON T
		       - Must be either float, double, or long double, ie:
			     is_float_type<T>::value == true

	    - Ensures
	        - returns true if all the matrices in the vector have the same shape
	*/
    template<typename T>
    bool all_col_vectors_are_same_size(const std::vector<col_vector<T>>& data_set)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);

        auto first_size = data_set[0].size();
        for (size_t s = 1; s < data_set.size(); s++)
        {
			const auto& item = data_set[s];

            if(item.size() != first_size)
            {
                return false;
            }
        }
        return true;
    }

    /*!
        Check if all the elements in a vector have the same size

        - REQUIREMENTS ON T
    	    - Must be a object with a .size() method

        - Ensures
            - Returns true if all elements in the vector have the same size, else false.
    */
    template<typename T>
    bool all_vectors_are_same_size(const std::vector<T>& data_set)
    {
        auto first_size = data_set[0].size();
		for (size_t s = 1; s < data_set.size(); s++)
		{
			const auto& item = data_set[s];

            if(item.size() != first_size)
            {
                return false;
            }
        }
        return true;
    }


    /*!
        Check if two image pyramids have the same size.

        - Ensures
            - Returns true if pyramids have the same number of pyramid levels and contain arrays of the same size in each level. Else false.
    */
    inline bool image_pyramids_same_size(const std::vector<std::unique_ptr<array2d<unsigned char>>>& img1_pyramid, const std::vector<std::unique_ptr<array2d<unsigned char>>>& img2_pyramid)
    {
        if (img1_pyramid.size() != img2_pyramid.size())
            return false;

        for (size_t level = 0; level < img1_pyramid.size(); level++)
        {
            if (img1_pyramid[level]->size() != img2_pyramid[level]->size())
                return false;
        }
        return true;
    }

    /*!
       Check if the size of chips is equal for each pyramid level.

       - Ensures
           - Returns true if the two inputs have the same number of levels and contain chips of the same size in each level. Else false.
   */
    inline bool all_chips_same_size(const std::vector<array2d<unsigned char>>& reference_chips, const std::vector<dlib::array<array2d<unsigned char>>>& stereo_chips)
    {
        if (reference_chips.size() != stereo_chips.size())
            return false;

        for (std::size_t level = 0; level < stereo_chips.size(); level++)
        {
            for (const auto& chip : stereo_chips[level])
            {
                if (chip.nc() != reference_chips[level].nc() || chip.nr() != reference_chips[level].nr())
                {
                    return false;
                }
            }
        }
        return true;
    }

    /*!
        Find minimum values seen in each row of items in a data set

        - REQUIREMENTS ON T
		     - Must be either float, double, or long double, ie:
			   is_float_type<T>::value == true

        - Requires
             - data_set.size() > 0
             - all_col_vectors_are_same_size(data_set)

        - Ensures
	         - Returns a column vector containing the minimum values
               seen in each corresponding row of data_set.
	*/
	template<typename T>
    col_vector<T> element_wise_min(const std::vector<col_vector<T>>& data_set)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);
        DLIB_ASSERT(data_set.size() > 0 &&
            all_col_vectors_are_same_size(data_set),
			"\t element_wise_min(const std::vector<matrix<T, 0, 1>>& data_set)"
			<< "\n\t Invalid inputs were given to this function "
			<< "\n\t data_set.size(): " << data_set.size()
            << "\n\t all_col_vectors_are_same_size(data_set): " << all_col_vectors_are_same_size(data_set));

		matrix<T, 0, 1> result = data_set.front();

		for (size_t s = 1; s < data_set.size(); s++)
		{
			const col_vector<T>& item = data_set[s];

			for (long i = 0; i < item.nr(); i++)
			{
				result(i) = std::min(result(i), item(i));
			}
		}

		return result;
	}

    /*!
        Find maximum values seen in each row of items in a data set

        - REQUIREMENTS ON T
		     - Must be either float, double, or long double, ie:
			   is_float_type<T>::value == true

        - Requires
             - data_set.size() > 0
             - all_col_vectors_are_same_size(data_set)

        - Ensures
	         - Returns a column vector containing the maximum values
               seen in each corresponding row of data_set.
	*/
	template<typename T>
    col_vector<T> element_wise_max(const std::vector<col_vector<T>>& data_set)
	{
        COMPILE_TIME_ASSERT(is_float_type<T>::value);
        DLIB_ASSERT(data_set.size() > 0 &&
            all_col_vectors_are_same_size(data_set),
            "\t element_wise_max(const std::vector<matrix<T, 0, 1>>& data_set)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t data_set.size(): " << data_set.size()
            << "\n\t all_col_vectors_are_same_size(data_set): " << all_col_vectors_are_same_size(data_set));

	    matrix<T, 0, 1> result = data_set.front();

		for (size_t s = 1; s < data_set.size(); s++)
		{
			const col_vector<T>& item = data_set[s];

			for (long i = 0; i < item.nr(); i++)
			{
				result(i) = std::max(result(i), item(i));
			}
		}

		return result;
	}

    /*!
        Find minimum and maximum values seen in each row of items in a data set

        - REQUIREMENTS ON T
		     - Must be either float, double, or long double, ie:
			   is_float_type<T>::value == true

        - Requires
             - data_set.size() > 0
             - all_col_vectors_are_same_size(data_set)

        - Ensures
	         - Output is two column vectors containing the minimum and maximum values
               seen in each corresponding row of data_set.
	*/
	template<typename T>
    void element_wise_min_max(const std::vector<col_vector<T>>& data_set, col_vector<T>& min_result, col_vector<T>& max_result)
	{
        COMPILE_TIME_ASSERT(is_float_type<T>::value);
        DLIB_ASSERT(data_set.size() > 0 &&
            all_col_vectors_are_same_size(data_set),
            "\t element_wise_max(const std::vector<matrix<T, 0, 1>>& data_set)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t data_set.size(): " << data_set.size()
            << "\n\t all_col_vectors_are_same_size(data_set): " << all_col_vectors_are_same_size(data_set));

		min_result = max_result = data_set.front();

		for (size_t s = 1; s < data_set.size(); s++)
		{
			const col_vector<T>& item = data_set[s];

			for (long i = 0; i < item.nr(); i++)
			{
				min_result(i) = std::min(min_result(i), item(i));
				max_result(i) = std::max(max_result(i), item(i));
			}
		}
	}

     /*!
         Turns a std::vector of dlib column vectors into a dlib matrix

         - REQUIREMENTS ON T
	 	     - Must be either float, double, or long double, ie:
	 		   is_float_type<T>::value == true

         - Requires
             - data_as_col_vectors.size() > 0
             - Every element of data_as_col_vectors must be same size and have nr() > 0

	     - Ensures
	         - Turns the std::vector of dlib column vectors into a dlib matrix, each column of which
	           is the corresponding element of the std::vector
	 */
	template <typename T>
	matrix<T> col_vectors_to_single_mat(const std::vector<col_vector<T>>& data_as_col_vectors)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);
        DLIB_ASSERT(data_as_col_vectors.size() > 0 &&
            all_col_vectors_are_same_size(data_as_col_vectors) &&
            data_as_col_vectors[0].size() > 0
            ,
			"\t col_vectors_to_single_mat(const std::vector<col_vector<T>>& data_as_col_vectors)"
			<< "\n\t Invalid inputs were given to this function "
			<< "\n\t (data_as_col_vectors.size(): " << data_as_col_vectors.size()
            << "\n\t (all_col_vectors_are_same_size(data_as_col_vectors): " << all_col_vectors_are_same_size(data_as_col_vectors)
            << "\n\t (data_as_col_vectors[0].size(): " << data_as_col_vectors[0].size());

		matrix<T> mat;
		auto nr = data_as_col_vectors[0].nr();
		auto nc = static_cast<int>(data_as_col_vectors.size());
		mat.set_size(nr, nc );
		auto count = 0;
		for (const auto& vec : data_as_col_vectors)
		{
			set_colm(mat, count++) = vec;
		}
		return mat;
	}

    /*!
        Converts a vector to a map according the given keys.

        - Requires
            - keys.size()==values.size()
            - is_unique(keys)

	    - Ensures
	        - Returns a map in which the corresponding items in 'keys' index the ordered items in 'values'.
	 */
	template<class T, class U>
	std::map<T, U> vector_to_map(const std::vector<U>& values, const std::vector<T>& keys)
	{
		DLIB_ASSERT(keys.size()==values.size(),
			"\t vector_to_map(const std::vector<U>& values, const std::vector<T>& keys)"
			<< "\n\t Invalid inputs were given to this function "
			<< "\n\t keys.size(): " << keys.size()
            << "\n\t values.size(): " << values.size()
            << "\n\t is_unique(keys): " << is_unique(keys));

		std::map<T, U> output;
		for (size_t i = 0; i < keys.size(); i++)
		{
			output[keys[i]] = values[i];
		}
		return output;
	}

    /*!
        Converts a map to a vector corresponding to the given keys, in order

        - Requires
            - All items in selected_keys must be in the map, i.e.
              all_items_are_keys(const std::map<T, U>& input, const std::vector<T>& keys) == true

	    - Ensures
	        - Returns the items from the map, selected by selected_keys, as a vector.
            - The returned vector has the same size as 'keys'.
	*/
	template<class T, class U>
	std::vector<U> map_to_vector(const std::map<T, U>& input, const std::vector<T>& keys)
	{
		DLIB_ASSERT(all_keys_present(input, keys) == true,
			"\t map_to_vector(const std::map<T, U>& input, const std::vector<T>& keys)"
			<< "\n\t Invalid inputs were given to this function "
			<< "\n\t all_keys_present(const std::map<T, U>& input, const std::vector<T>& keys): " << all_keys_present(input, keys));

		std::vector<U> output;
		output.reserve(keys.size());

		for (const auto& key : keys)
		{
			output.push_back(input.at(key));
		}

		return output;
	}

	/*!
        Lists all keys in a given map

	    - Ensures
	        - Returns a vector of all keys in a given map
	*/
	template<typename T, typename U>
	std::vector<T> list_all_keys(const std::map<T, U>& input)
	{
		std::vector<T> output(input.size());
        auto count = 0;
		for (const auto &imap : input)
		{
			output[count++] = imap.first;
		}
		return output;
	}

	/*!
	    Checks existence of an element in a vector

	    - Ensures
	        - Returns true if the vector contains the specified element.
	*/
	template <typename T>
	bool does_vector_contain_element(const std::vector<T> & vec, const T& element)
	{
		return std::find(vec.begin(), vec.end(), element) != vec.end();
	}

    /*!
        Checks if elements of one vector are contained within another

        - Ensures
            - Returns true if the specified vector is a subset.
    */
    template <typename T>
    bool does_vector_contain_vector(std::vector<T> superset, std::vector<T> subset)
    {
        if (superset.size() < subset.size())
            return false;

        std::sort(superset.begin(), superset.end());
        std::sort(subset.begin(), subset.end());

        return std::includes(superset.begin(), superset.end(), subset.begin(), subset.end());
    }

    /*!
        Checks if two vectors are equal.

        - Ensures
            - Returns true if the vectors contain the same elements.
    */
    template <typename T>
    bool vectors_contain_same_elements(std::vector<T> vec1, std::vector<T> vec2)
    {
        if (vec1.size() != vec2.size())
            return false;

        std::sort(vec1.begin(), vec1.end());
        std::sort(vec2.begin(), vec2.end());

        return (vec1 == vec2 ? true : false);
    }


	/*!
	    Finds the index of the element in the vector

	    - Ensures
	        - Returns the zero-based index of the specified element in the vector,
              or -1 if the element does not exist
	*/
	template <typename T>
	int index_of_item_in_vector(const std::vector<T> & vec, const T& value)
	{
		int index = static_cast<int>(vec.size() - 1);
		for (; index >= 0 && vec[index] != value; index--);
		return index;
	}

	/*!
	    Extends a vector with the elements from another vector

        - Ensures
            - Returns a vector containing the elements of the first vector
              followed by the elements of the second vector.
	*/
	template <typename T>
	void extend_vector(std::vector<T>& vec_to_extend, const std::vector<T> & items_to_add)
	{
		vec_to_extend.reserve(vec_to_extend.size() + items_to_add.size());
		std::copy(items_to_add.begin(), items_to_add.end(), back_inserter(vec_to_extend));
	}

	/*!
	    Given a vector of std::pairs a new vector is created containing just the first part of each pair

	    - Ensures
	        - Returns a vector of the first parts of the pairs
	*/
	template<typename T, typename U>
	std::vector<T> extract_first_element_in_vector_of_pairs(const std::vector<std::pair<T, U>> & vec)
	{
        std::vector<T> result;
		result.reserve(vec.size());

		for (const auto& this_pair : vec)
		{
			result.push_back(this_pair.first);
		}
		return result;
	}

    /*!
        Given a vector of std::pairs a new vector is created containing just the second part of each pair

        - Ensures
            - Returns a vector of the first parts of the pairs
    */
	template<typename T, typename U>
	std::vector<U> extract_second_element_in_vector_of_pairs(const std::vector<std::pair<T, U>> & vec)
	{
		std::vector<U> result;
		result.reserve(vec.size());

		for (const auto & this_pair : vec)
		{
			result.push_back(this_pair.second);
		}

		return result;
	}

    /*!
        Find next position in the vector where target is found.

	    - Ensures
	        - Returns the next position in the vector where target is found,
	          starting at start_pos (not including start_pos)
	        - note, the position is returned as the absolute position in the vector,
	          NOT the positions relative to start_pos
	        - if target is not found, returns -1
	        - if start pos is outside the vector, returns -1
	*/
	template<typename T>
	int find_next_case(const std::vector<T>& vec, const T& target, const int start_pos)
	{
		if (start_pos >= 0 && start_pos < vec.size() - 1)
		{
			for (auto i = start_pos + 1; i < vec.size(); ++i)
			{
				if (target == vec[i])
				{
					return i;
				}
			}
		}
		return -1;
	}

	/*!
        Find previous position in the vector where target is found.

	    - Ensures
	        - Returns the previous position in the vector where target is found,
	          starting at start_pos (not including start_pos)
	        - note, the position is returned as the absolute position in the vector,
	          NOT the positions relative to start_pos
	        - if target is not found, returns -1
	        - if start pos is outside the vector, returns -1
	*/
	template<typename T>
	int find_prev_case(const std::vector<T>& vec, const T& target, const int start_pos)
	{
		if (start_pos > 0 && start_pos < vec.size())
		{
			for (auto i = start_pos - 1; i >= 0; --i)
			{
				if (target == vec[i])
				{
					return i;
				}
			}
		}
		return -1;
	}

    /*!
        Check if all vectors in a vector of vectors have the same size, and are bigger than a minimum size

	    - Ensures
	        - Returns true iff input has at least one entry, each entry has the same size,
	          and each entry has at least n elements
            - Returns false if input.empty()
	*/
	template<typename T>
	bool vectors_all_same_size_with_at_least_n_elements(const std::vector<std::vector<T>>& input, const size_t n)
	{
		if (input.empty())
		{
			return false;
		}
		const size_t first_size = input[0].size();
		if (first_size < n)
		{
			return false;
		}

		return all_vectors_are_same_size(input);
	}

    /*!
        Extracts the unique elements in a vector

	    - Ensures
	        - Returns a new vector containing only the unique elements in input
	*/
	template<typename T>
	std::vector<T> unique_elements(const std::vector<T>& input)
	{
		std::vector<T> output(input);
		std::sort(output.begin(), output.end());
		output.erase(std::unique(output.begin(), output.end()), output.end());
		return output;
	}

    /*!
        Checks that all maps in a vector of maps contain the same keys.

	    - Ensures
	        - Returns true if all the maps in the vector have the same keys.
	*/
    template <typename K, typename T>
    bool all_maps_have_same_keys(const std::vector<std::map<K, T>>& maps)
    {
        if (maps.empty())
			return false;

        const auto& maps_0 = maps[0];

        for (size_t s = 1; s < maps.size(); s++)
        {
			const auto& maps_s = maps[s];

			if (maps_s.size() != maps_0.size())
				return false;

			for (auto it_0 = maps_0.begin(), it_s = maps_s.begin(); it_0 != maps_0.end(); ++it_0, ++it_s)
			{
				if (it_0->first != it_s->first)
					return false;
			}
        }

        return true;
    }

    /*!
        Converts a vector of maps to a map of vectors
        - Requires
            - every item in input must contain the same keys
	    - Ensures
	        - returns a map of vectors by transposing the vector of maps
	*/
    template<typename K, typename T>
    std::map<K,std::vector<T>> vector_of_maps_to_map_of_vectors(const std::vector<std::map<K, T>>& input)
    {
        DLIB_ASSERT(all_maps_have_same_keys(input) == true,
            "\t vector_of_maps_to_map_of_vectors(std::vector<std::map<K, T>>& input)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t all_maps_have_same_keys(input): " << all_maps_have_same_keys(input));

        std::map<K, std::vector<T>> output;
        if (input.empty())
            return output;

        auto keys = list_all_keys(input[0]);
        for(const auto& key : keys)
        {
            output[key] = std::vector<T>(input.size());
            auto count = 0;
            for(const auto& item : input)
            {
                output[key][count++] = item.at(key);
            }
        }
        return output;
    }

    /*!
    ensures
    - returns a std::vector of int ranging from 0 to n-1
    !*/

    inline std::vector<int> zero_based_int_range(unsigned int n)
    {
        std::vector<int> result(n);
        for (unsigned int i = 0; i < n; ++i)
        {
            result[i] = i;
        }
        return result;
    }

    /*
   Returns the median of an input vector.
   */
    template<typename T>
    T calc_median(std::vector<T> input_vec)
    {
        std::size_t middle_idx = input_vec.size() / 2;
        std::nth_element(input_vec.begin(), input_vec.begin() + middle_idx, input_vec.end());

        if (input_vec.size() % 2 != 0)
            return input_vec[middle_idx];
        else
        {
            T tmp = input_vec[middle_idx];
            middle_idx = middle_idx - 1;
            std::nth_element(input_vec.begin(), input_vec.begin() + middle_idx, input_vec.end());
            return (tmp + input_vec[middle_idx]) / 2.0;
        }
    }

    /*!
          Calculates the median filter for all points across all frames.

          - Requires:
              - all frames contain the same points.
          - Ensures:
              - Returns the smoothed points over time by taking their median values at a user defined window size.
     */
    template<typename T>
    std::vector<std::map<std::string, point2<T>>> calc_median_filter(std::vector<std::map<std::string, point2<T>>> input_points, unsigned window_size)
    {
        DLIB_ASSERT(all_maps_have_same_keys(input_points),
            "\n\t Input vector should contain maps with the same point ids.");

        std::vector<std::map<std::string, point2<T>>> smoothed_input_points(input_points.size());

        //Loop for all points.
        for (const auto& pt : input_points[0])
        {
            //Loop for all frames.
            for (unsigned fr = 0; fr < input_points.size(); fr++)
            {
                std::vector<T> temp_vec_x, temp_vec_y;

                if (fr < std::floor(window_size / 2))
                {
                    for (unsigned i = 0; i < window_size; i++)
                    {
                        temp_vec_x.emplace_back(input_points[i].at(pt.first).x());
                        temp_vec_y.emplace_back(input_points[i].at(pt.first).y());
                    }
                }
                else if (fr > input_points.size() - std::floor(window_size / 2) - 1)
                {
                    for (unsigned i = input_points.size() - window_size; i < input_points.size(); i++)
                    {
                        temp_vec_x.emplace_back(input_points[i].at(pt.first).x());
                        temp_vec_y.emplace_back(input_points[i].at(pt.first).y());
                    }
                }
                else
                {
                    if (window_size % 2 == 0)
                    {
                        for (unsigned i = fr - std::floor(window_size / 2); i <= fr + std::floor(window_size / 2); i++)
                        {
                            temp_vec_x.emplace_back(input_points[i].at(pt.first).x());
                            temp_vec_y.emplace_back(input_points[i].at(pt.first).y());
                        }
                    }
                    else
                    {
                        for (unsigned i = fr - std::floor(window_size / 2); i < fr + std::floor(window_size / 2); i++)
                        {
                            temp_vec_x.emplace_back(input_points[i].at(pt.first).x());
                            temp_vec_y.emplace_back(input_points[i].at(pt.first).y());
                        }
                    }
                }

                T median_val_x = calc_median(temp_vec_x);
                T median_val_y = calc_median(temp_vec_y);

                smoothed_input_points[fr].emplace(pt.first, point2<T>(median_val_x, median_val_y));
            }
        }

        return smoothed_input_points;
    }

	/*
		A constraint on vector components of arbitrary dimensionality.
		Particularly suited for constrained PCA encoding and reconstruction.
	*/
	template <typename T=double>
	class subvector_constraint
	{
	public:

        /*
            Overload for point2 vectors
        */
        inline subvector_constraint(size_t index, const point2<T>& pt2):index_(index)
        {
            constraint_vector_ = col_vector<T>({ pt2.x(), pt2.y() });
        }


        /*
            Overload for col_vector<T>
        */
        inline subvector_constraint(size_t index, const col_vector<T>& vec) :index_(index)
        {
            constraint_vector_ = vec;
        }

		/*
			Returns the index of the constraint in the big data vector
		*/
		inline size_t index() const { return index_; }

        /*
            Dimension of the constraint vector
        */
        inline size_t dimension() const { return constraint_vector_.nr(); }

		/*
			Get the constraint vector
		*/
        inline col_vector<T> constraint_vector()const { return constraint_vector_; }

		/*
			Returns the dimensionality of a vector of constraints
		*/
		template <typename P>
		inline static size_t dimensionality(const std::vector<subvector_constraint<P>>& constraints)
		{
			if (constraints.empty()) return 0;

			size_t dim = 0;
			for (size_t i = 0; i < constraints.size(); i++)
			{
				dim += constraints[i].dimension();
			}
			return dim;
		}

	private:

		// Index of the constraint in the vector. To compute the 0-based position in the vector:
		// index in vector = dimension_ * index (see position_in_vector() )
		size_t index_;

		// The constraint (sub)vector
		col_vector<T> constraint_vector_;
	};


}
