// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "transforms.h"
#include "data_types.h"
#include "data_utils.h"
#include "simple_geometry.h"
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



	/**
   *   Computes a distance cross-table for a set of points
   *
   *    - REQUIREMENTS ON T
   *	     - Must be either float, double, or long double, ie:
   *		    is_float_type<T>::value == true
   *    - Requires
   *
   *    - Ensures
   *       - Returns a vector of vectors where each inner vector gives the distance to each point from the point indexed
   *         by the row of the outer vector
   */
	template<typename T>
	std::vector<std::vector<double>> point_cross_distances(const std::vector<point2<T>>& points)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
		
		auto n_points = points.size();
		std::vector<std::vector<double>> distances(n_points);
		for (size_t p = 0; p < points.size(); ++p)
		{
			distances[p].resize(n_points);
			for (size_t q = 0; q < points.size(); ++q)
			{
				distances[p][q] = (points[p] - points[q]).length();
			}
		}
		return distances;
	}


    /**
    *   Computes the signed angle between two vectors
    *
    *    - REQUIREMENTS ON T
    *	     - Must be either float, double, or long double, ie:
    *		    is_float_type<T>::value == true
    *    - Requires
    *        - (b-a).length() > 0
    *        - (d-c).length() > 0
    *    - Ensures
    *       - Returns the signed angle between the vectors b-a, and d-c
    */
    template<typename T>
    T signed_angle(point2<T> a, point2<T>b, point2<T> c, point2<T>d)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT((b - a).length() > 0 && (d - c).length() >0,
            "\t T signed_angle(point2<T> a, point2<T>b, point2<T> c, point2<T>d)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t a: " << a
            << "\n\t b: " << b
            << "\n\t c: " << c
            << "\n\t d: " << d);

        auto v1 = b - a;
        auto v2 = d - c;
        auto dot = v1.x()*v2.x() + v1.y()*v2.y();
        auto det = v1.x()*v2.y() - v1.y()*v2.x();
        return atan2(det, dot);
    }

    /*!
        Find the range of all the points in a map of points.
    
        - REQUIREMENTS ON T
		    - Must be either float, double, or long double, ie:
			  is_float_type<T>::value == true
    
        - Requires
            -  points.size() > 0
    
	    - Ensures
	        - Returns a single column matrix with 4 elements giving the ranges covered by the point map.
	*/
	template<typename T, typename K>
	matrix<T, 4, 1> minx_maxx_miny_maxy(const std::map<K,point2<T>>& points) 
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(points.size()>0,
            "\t minx_maxx_miny_maxy(const point2_map<T>& points)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t points.size(): " << points.size());

		matrix<T, 4, 1> result;
		T min_x = 0;
		T max_x = 0;
		T min_y = 0;
		T max_y = 0;
		auto is_first = true;
		for (auto item : points)
		{
			if (is_first)
			{
				min_x = item.second.x();
				max_x = item.second.x();
				min_y = item.second.y();
				max_y = item.second.y();
				is_first = false;
			}
			else
			{
				min_x = std::min(min_x, item.second.x());
				max_x = std::max(max_x, item.second.x());
				min_y = std::min(min_y, item.second.y());
				max_y = std::max(max_y, item.second.y());
			}
		}

		result(0) = min_x;
		result(1) = max_x;
		result(2) = min_y;
		result(3) = max_y;

		return result;
	}

    /*
    Find the range covered by the points in a vector of points

    - REQUIREMENTS ON T
    - Must be either float, double, or long double, ie:
    is_float_type<T>::value == true

    - Requires
    - points.size() > 0

    - Ensures
    - Returns a single column matrix with 4 elements giving the ranges covered by the point vector.
    */
    template<typename T>
    matrix<T, 4, 1> minx_maxx_miny_maxy(const point2_vector<T>& points)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(points.size()>0,
            "\t minx_maxx_miny_maxy(const point2_vector<T>& points)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t points.size(): " << points.size());
        matrix<T, 4, 1> result;
        T min_x = 0;
        T max_x = 0;
        T min_y = 0;
        T max_y = 0;
        auto is_first = true;
        for (auto item : points)
        {
            if (is_first)
            {
                min_x = item.x();
                max_x = item.x();
                min_y = item.y();
                max_y = item.y();
                is_first = false;
            }
            else
            {
                min_x = std::min(min_x, item.x());
                max_x = std::max(max_x, item.x());
                min_y = std::min(min_y, item.y());
                max_y = std::max(max_y, item.y());
            }
        }

        result(0) = min_x;
        result(1) = max_x;
        result(2) = min_y;
        result(3) = max_y;

        return result;
    }


    /*!
        Centres a shape around the origin
        
         - REQUIREMENTS ON T
		     - Must be either float, double, or long double, ie:
		  	   is_float_type<T>::value == true
        
         - Requires
             - points_vector.size() >= 1
        
	     - Ensures
	         - Returns a new points vector which is simply the input points vector such that the mean
	           of the points is (0,0).
	*/
	template<typename T>
	point2_vector<T> centre_shape_at_origin(const point2_vector<T>& points_vector)
	{
        COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(points_vector.size() > 0,
            "\t centre_shape_at_origin(const point2_vector<T>& points_vector)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t points_vector.size(): " << points_vector.size());

		auto mean_pt = point2<T>(0, 0);
		for (const auto& pt : points_vector)
		{
			mean_pt += pt;
		}
		mean_pt /= static_cast<T>(points_vector.size());
		point2_vector<T> output;
		for (const auto& pt : points_vector)
		{
			output.push_back(pt - mean_pt);
		}
		return output;
	}
	
    /*!
        Compute the norm of a shape
     
        - REQUIREMENTS ON T
	 	       - Must be either float, double, or long double, ie:
	 		     is_float_type<T>::value == true
     
        - Requires
            - points_vector.size() >= 1
     
	    - Ensures
	        - Compute the norm of the input points vector, that is:
	               sqrt(sum(x*x-y*y)) over all the points
	*/
	template<typename T>
	T shape_norm(const point2_vector<T>& points_vector)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(points_vector.size() >= 1,
            "\t shape_norm(const point2_vector<T>& points_vector)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t points_vector.size(): " << points_vector.size());

		T norm_squared = 0.0f;

		for (auto point : points_vector)
		{
			norm_squared += point.x()*point.x() + point.y()*point.y();
		}
        return std::sqrt(norm_squared);
	}

    /*!
        Transform a shape to have unit norm
    
        - REQUIREMENTS ON T
		       - Must be either float, double, or long double, ie:
			     is_float_type<T>::value == true
    
        - Requires
            - points_vector.size() >= 1
    
	    - Ensures
	        - Returns a new points vector which is simply the input points vector adjusted
	          to have a norm of 1.0
	*/
	template<typename T>
	point2_vector<T> scale_shape_to_unit_norm(const point2_vector<T>& points_vector)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(points_vector.size() >= 1,
            "\t scale_shape_to_unit_norm(const point2_vector<T>& points_vector)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t points_vector.size(): " << points_vector.size());

        auto norm = shape_norm(points_vector);
		point2_vector<T> output;
		for (auto point : points_vector)
		{
			output.push_back(point2<T>(point.x() / norm, point.y() / norm));
		}
		return output;
	}
	
    /*!
        Returns the distance norm between two shapes
    
        - REQUIREMENTS ON T
		       - Must be either float, double, or long double, ie:
			     is_float_type<T>::value == true
    
        - Requires
            - points_vector_a.size() >= 1
	        - points_vector_a.size() == points_vector_b.size()
    
	    - Ensures
	        - Returns the distance norm between two shapes
	*/
	template<typename T>
	T norm_between_shapes(const point2_vector<T>& points_vector_a, const point2_vector<T>& points_vector_b)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(points_vector_a.size() >= 1 && (points_vector_a.size() == points_vector_b.size()),
            "\t norm_between_shapes(const point2_vector<T>& points_vector_a, const point2_vector<T>& points_vector_b)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t points_vector_a.size(): " << points_vector_a.size()
            << "\n\t points_vector_b.size(): " << points_vector_b.size());

		T norm_squared = 0.0f;
		const auto n_points = static_cast<int>(points_vector_a.size());

		for (auto i_point = 0; i_point<n_points; ++i_point)
		{
			auto dx = points_vector_a[i_point].x() - points_vector_b[i_point].x();
			auto dy = points_vector_a[i_point].y() - points_vector_b[i_point].y();
			norm_squared += dx * dx + dy * dy;
		}
		return std::sqrt(norm_squared);
	}

    /*!
        Adds two shape vectors in a pointwise fashion.
    
        - REQUIREMENTS ON T
		       - Must be either float, double, or long double, ie:
			     is_float_type<T>::value == true
    
        - Requires
            - a.size()==b.size()
    
	    - Ensures
	        - Returns a new vector where each point in b is added to the corresponding one in a.
	*/
    template<typename T>
    point2_vector<T> operator+(const point2_vector<T>& a, const point2_vector<T>& b)
    {
        DLIB_ASSERT(a.size()==b.size(),
            "\t operator+(const point2_vector<T>& a, const point2_vector<T>& b)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t a.size(): " << a.size()
            << "\n\t b.size(): " << b.size());

        point2_vector<T> result(a);
        for(auto i=0; i<b.size(); ++i)
        {
            result[i] += b[i];
        }
        return result;
    }

    /*!
        Subtracts two shape vectors in a pointwise fashion.
    
        - REQUIREMENTS ON T
		       - Must be either float, double, or long double, ie:
			     is_float_type<T>::value == true
    
        - Requires
            - a.size()==b.size()
    
	    - Ensures
	        - Returns a new vector where each point in b is subtracted from the corresponding one in a.
	*/
    template<typename T>
    point2_vector<T> operator-(const point2_vector<T>& a, const point2_vector<T>& b)
    {
        DLIB_ASSERT(a.size() == b.size(),
            "\t operator+(const point2_vector<T>& a, const point2_vector<T>& b)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t a.size(): " << a.size()
            << "\n\t b.size(): " << b.size());

        point2_vector<T> result(a);
        for (auto i = 0; i<b.size(); ++i)
        {
            result[i] -= b[i];
        }
        return result;
    }

	/*!
        Multplies a shape vector by a scalar.
    
        - REQUIREMENTS ON T
		       - Must be either float, double, or long double, ie:
			     is_float_type<T>::value == true
    
        - Requires
    
	    - Ensures
	        - Returns a new vector where each point in a is multiplied by b.
	*/
    template<typename T>
    point2_vector<T> operator*(const point2_vector<T>& a, const T& b)
    {
        point2_vector<T> result(a.size());
        for (auto i = 0; i<a.size(); ++i)
        {
            result[i] = a[i]*b;
        }
        return result;
    }

    /*!
        Computes the sum of distances between corresponding points in a shape.
    
        - REQUIREMENTS ON T
		       - Must be either float, double, or long double, ie:
			     is_float_type<T>::value == true
    
        - Requires
            - a.size()==b.size()
    
	    - Ensures
	        - Returns the sum of distances between corresponding points in a shape.
	*/
    template<typename T>
    T sum_of_point_distances(const point2_vector<T>& a, const point2_vector<T>& b)
    {
        DLIB_ASSERT(a.size() == b.size(),
            "\t operator+(const point2_vector<T>& a, const point2_vector<T>& b)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t a.size(): " << a.size()
            << "\n\t b.size(): " << b.size());

        T sum = 0;
        for (auto i = 0; i<b.size(); ++i)
        {
            sum += (a[i] - b[i]).length();
        }
        return sum;
    }
 
	
    /*!
        Returns the mean of all the shapes in a set.
    
        - REQUIREMENTS ON T
		    - Must be either float, double, or long double, ie:
			  is_float_type<T>::value == true
    
        - Requires
            - shapes.size() > 0
	        - every item in shapes is the same size.
    
	    - Ensures
	        - Returns the mean shape of all the shapes.
	*/
	template<typename T>
	point2_vector<T> mean_shape(const std::vector<point2_vector<T>>& shapes)
	{
        COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(shapes.size()>0 && all_vectors_are_same_size(shapes),
            "\t mean_shape(const std::vector<point2_vector<T>>& shapes)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t shapes.size(): " << shapes.size()
            << "\n\t all_vectors_are_same_size(shapes): " << all_vectors_are_same_size(shapes));

		point2_vector<T> mean_shape(shapes[0].size(), point2<double>(0, 0));
		for (auto shape : shapes)
		{
			for (auto i : range(0, static_cast<long>(shape.size()) - 1))
			{
				mean_shape[i] += shape[i];
			}
		}
		for (auto& pt : mean_shape)
		{
			pt /= static_cast<T>(shapes.size());
		}
		return mean_shape;
	}
	

	
    /*!
        Find the range of the points across a set of point vectors
    
        - REQUIREMENTS ON T
		       - Must be either float, double, or long double, ie:
			     is_float_type<T>::value == true
    
        - Requires
            -  points.size() > 0
	        -  points[0].size() > 0
    
	    - Ensures
	        - Returns a single column matrix with 4 elements giving the ranges covered by the all the point vectors.
	*/
	template<typename T>
	matrix<T, 4, 1> minx_maxx_miny_maxy(const std::vector<point2_vector<T>>& points)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(points.size()>0 && points[0].size() > 0,
            "\t minx_maxx_miny_maxy(const std::vector<point2_vector<T>>& points)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t points.size(): " << points.size()
            << "\n\t points[0].size(): " << points[0].size());

		auto result = minx_maxx_miny_maxy(points[0]);
		for (auto item : points)
		{
			auto this_result = minx_maxx_miny_maxy(item);
			result(0) = std::min(result(0), this_result(0));
			result(1) = std::max(result(1), this_result(1));
			result(2) = std::min(result(2), this_result(2));
			result(3) = std::max(result(3), this_result(3));
		}
		return result;
	}

	/*!
        Find the range of the points across a set of point maps
    
        - REQUIREMENTS ON T
		       - Must be either float, double, or long double, ie:
			     is_float_type<T>::value == true
    
        - Requires
            -  points.size() > 0
	        -  points[0].size() > 0
    
	    - Ensures
	        - Returns a single column matrix with 4 elements giving the ranges covered by the all the point maps.
	 */
	template<typename T, typename K>
	matrix<T, 4, 1> minx_maxx_miny_maxy(const std::vector<std::map<K,point2<T>>>& points)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(points.size()>0 && points[0].size() > 0,
            "\t minx_maxx_miny_maxy(const std::vector<point2_vector<T>>& points)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t points.size(): " << points.size()
            << "\n\t points[0].size(): " << points[0].size());

		auto result = minx_maxx_miny_maxy(points[0]);
		for (auto item : points)
		{
			auto this_result = minx_maxx_miny_maxy(item);
			result(0) = std::min(result(0), this_result(0));
			result(1) = std::max(result(1), this_result(1));
			result(2) = std::min(result(2), this_result(2));
			result(3) = std::max(result(3), this_result(3));
		}
		return result;
	}

    /*!
        Find the "mean shape" of a set using the Procrustes alignment process.
    
        - REQUIREMENTS ON T
		    - Must be either float, double, or long double, ie:
			  is_float_type<T>::value == true
    
        - Requires
            - vectors_all_same_size_with_at_least_n_elements(points_data,2) == true
    
	    - Ensures
	        - Returns a the 'mean shape' (in the appropriate sense) of the Procrustes process, for
	          the points specified by alignment_ids using the align_method specified.
	*/
	template<typename T>
	point2_vector<T> compute_procrustes_base_shape(const std::vector<point2_vector<T>>& points_data, const alignment_type& align_method)
	{
        COMPILE_TIME_ASSERT(is_float_type<T>::value == true);
        DLIB_ASSERT(vectors_all_same_size_with_at_least_n_elements(points_data, 2),
            "\t compute_procrustes_base_shape(const std::vector<point2_vector<T>>& points_data, const alignment_type& align_method)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t vectors_all_same_size_with_at_least_n_elements(points_data, 2): " << vectors_all_same_size_with_at_least_n_elements(points_data, 2));

        if(align_method == alignment_type::none)
        {
            return mean_shape(points_data);
        }

		using shape = point2_vector<T>;
		using set_of_shapes = std::vector<shape>;

		const T tolerance = 1e-6f;

		set_of_shapes centred_shapes;
		for (const auto& item : points_data)
		{
			centred_shapes.push_back(centre_shape_at_origin(item));
		}
		shape current_mean_shape;
        auto new_mean_shape = centred_shapes[0];
        const auto has_scale = (alignment_type::similarity == align_method ||
            alignment_type::affine == align_method ||
            alignment_type::scale_translate == align_method ||
            alignment_type::projective == align_method);
        
	    if(has_scale)
        {
            new_mean_shape = scale_shape_to_unit_norm(centred_shapes[0]);
        }

		const auto reference_shape = shape(new_mean_shape);
		auto current_aligned_set_of_shapes(centred_shapes);
		do
		{
			current_mean_shape = shape(new_mean_shape);
			for (auto& this_shape : current_aligned_set_of_shapes)
			{
                point_transform_projective xform = find_transform_as_projective(this_shape, current_mean_shape, align_method);
                for (auto &pt : this_shape)
                {
                    pt = xform(pt);
                }
			}
            new_mean_shape = mean_shape(current_aligned_set_of_shapes);
            point_transform_projective xform = find_transform_as_projective(new_mean_shape, reference_shape, align_method);
            for(auto &pt : new_mean_shape)
            {
                pt = xform(pt);
            }
            if(has_scale)
            {
                new_mean_shape = scale_shape_to_unit_norm(new_mean_shape);
            }
			
		} while (norm_between_shapes(new_mean_shape, current_mean_shape) > tolerance);
	
        return current_mean_shape;
	}

}
