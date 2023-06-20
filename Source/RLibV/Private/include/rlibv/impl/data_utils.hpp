// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "basic_types.h"

namespace rlibv
{
	template<typename T, typename U, typename V>
	bool maps_have_same_keys(const std::map<T, U>& a, const std::map<T, V>& b)
	{
		if (a.size() != b.size())
		{
			return false;
		}
		auto a_it = a.begin();
		auto b_it = b.begin();
		while (a_it != a.end())
		{
			if (a_it->first != b_it->first)
			{
				return false;
			}
			++a_it;
			++b_it;
		}
		return true;
	}

	template<typename T, typename U>
	bool map_contains_key(const std::map<T, U>& m, const T& k)
	{
		return m.find(k) != m.end();
	}
	
	template<typename T>
	bool is_unique(std::vector<T> x)
	{
		std::set<T> y(x.begin(), x.end());
		return x.size() == y.size();
	}

	template<typename T>
	bool all_items_less_than(const std::vector<T>& data_set, const T& limit)
	{
		for (const auto& item : data_set)
		{
			if (item >= limit)
			{
				return false;
			}
		}
		return true;
	}

	template<typename T>
	bool all_col_vectors_are_same_size(const std::vector<col_vector<T>>& data_set)
	{
		static_assert(dlib::is_float_type<T>::value, "template parameter T must be a float or double type");

		auto first_size = data_set[0].size();
		for (size_t s = 1; s < data_set.size(); s++)
		{
			const auto& item = data_set[s];

			if (item.size() != first_size)
			{
				return false;
			}
		}
		return true;
	}


	template<typename T>
	bool all_shapes_are_same_size(const std::vector<shape2d<T>>& data_set)
	{
		auto first_size = data_set[0].size();
		for (size_t s = 1; s < data_set.size(); s++)
		{
			const auto& item = data_set[s];

			if (item.size() != first_size)
			{
				return false;
			}
		}
		return true;
	}

	template<typename T>
	bool shapes_all_same_size_with_at_least_n_elements(const std::vector<shape2d<T>>& data_set, const size_t n)
	{
		if (data_set.empty())
		{
			return false;
		}
		const size_t first_size = data_set[0].size();
		if (first_size < n)
		{
			return false;
		}

		return all_shapes_are_same_size(data_set);
	}
	
	template<typename T>
	dlib::matrix<T> col_vectors_to_single_mat(const std::vector<col_vector<T>>& data_as_col_vectors)
	{
		static_assert(dlib::is_float_type<T>::value, "template parameter T must be a float or double type");
		DLIB_ASSERT(data_as_col_vectors.size() > 0);
		DLIB_ASSERT(all_col_vectors_are_same_size(data_as_col_vectors));
		DLIB_ASSERT(data_as_col_vectors[0].size() > 0);

		dlib::matrix<T> mat;
		auto nr = data_as_col_vectors[0].nr();
		auto nc = static_cast<int>(data_as_col_vectors.size());
		mat.set_size(nr, nc);
		auto count = 0;
		for (const auto& vec : data_as_col_vectors)
		{
			set_colm(mat, count++) = vec;
		}
		return mat;
	}


	template<typename T>
	shape2d<T> col_matrix_to_shape2d(const dlib::matrix<T, 0, 1>& data)
	{
		static_assert(dlib::is_float_type<T>::value, "template parameter T must be a float or double type");
		DLIB_ASSERT(data.nr() % 2 == 0);

		shape2d<T> shape(data.size() / 2);
		for (auto p = 0; p < static_cast<int>(data.size() / 2); ++p)
		{
			shape[p] = point2d<T>(data(p * 2), data(p * 2 + 1));
		}
		return shape;
	}

	template<typename T>
	dlib::matrix<T, 0, 1> shape2d_to_col_matrix(const shape2d<T>& shape)
	{
		static_assert(dlib::is_float_type<T>::value, "template parameter T must be a float or double type");
		DLIB_ASSERT(shape.size() > 0);

		dlib::matrix<T, 0, 1> pts_data(static_cast<int>(shape.size()) * 2);
		for (auto p = 0; p < static_cast<int>(shape.size()); ++p)
		{
			pts_data(p * 2) = shape[p].x();
			pts_data(p * 2 + 1) = shape[p].y();
		}
		return pts_data;
	}


	template<typename T>
	void element_wise_min_max(const std::vector<col_vector<T>>& data_set, col_vector<T>& min_result,
			col_vector<T>& max_result)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(data_set.size() > 0);
		DLIB_ASSERT(all_col_vectors_are_same_size(data_set));

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

	template<typename T>
	std::vector<T> subvector(const std::vector<T>& input, const std::vector<int>& indices)
	{
		DLIB_ASSERT(std::all_of(indices.begin(), indices.end(), [input](size_t i) {return i < input.size(); }));

		std::vector<T> output;
		output.reserve(indices.size());
		std::transform(indices.begin(), indices.end(), std::back_inserter(output),[&input](int idx) {return input.at(idx);});
		return output;
	}

	template <typename T, int N, typename Comparer>
	std::array<int, N> sort_indices(const std::array<T, N>& v, Comparer comp)
	{
		// initialize original index locations
		std::array<int, N> idx;
		for (unsigned i = 0; i != idx.size(); ++i) idx[i] = i;

		// sort indexes based on comparing values in v
		std::sort(idx.begin(), idx.end(),
			[&v, &comp](int i1, int i2) {return comp(v[i1], v[i2]); });

		return idx;
	}
	
	template <typename T, typename Comparer>
	std::vector<int> sort_indices(const std::vector<T>& v, Comparer comp)
	{
		// initialize original index locations
		std::vector<int> idx(v.size());
		for (unsigned int i = 0; i != static_cast<unsigned int>(idx.size()); ++i) idx[i] = i;

		// sort indexes based on comparing values in v
		std::sort(idx.begin(), idx.end(),
			[&v, &comp](int i1, int i2) {return comp(v[i1], v[i2]); });

		return idx;
	}

}