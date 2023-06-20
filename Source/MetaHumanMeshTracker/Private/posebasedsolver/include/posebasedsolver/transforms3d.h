// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/geometry.h>
POSEBASEDSOLVER_RENABLE_WARNINGS
#include "data_types.h"
#include "data_utils.h"

namespace cm
{

   /*!
      Create a 3D similarity transform from scale, Euler angles (heading, attitude, bank), dx, dy, dz
      returning the result as a (special case) affine3d transform

      - Ensures
          - returns a point_transform_affine3d object, P, representing
            the similarity transform given by scale, heading (radians), attitude (radians), bank (radians), dx, dy, dz
            this conversion uses NASA standard aeroplane conventions for Euler angles as described on page:
            https://www.euclideanspace.com/maths/geometry/rotations/euler/index.htm
            Coordinate System: right hand
            Positive angle: right hand
            Order of euler angles: heading first, then attitude, then bank
   */
    template <typename T>
	dlib::point_transform_affine3d create_similarity_transform3d_as_affine3d(
        T scale,
        T heading,
        T attitude,
        T bank,
        T dx,
        T dy,
        T dz
    )
    {

        T ch = cos(heading);
        T sh = sin(heading);
        T ca = cos(attitude);
        T sina = sin(attitude);
        T cb = cos(bank);
        T sb = sin(bank);

        dlib::matrix<T, 3, 3> m;

        m(0,0) = scale * ch * ca;
        m(0,1) = scale * (sh * sb - ch * sina *cb);
        m(0,2) = scale * (ch * sina * sb + sh * cb);
        m(1,0) = scale * sina;
        m(1,1) = scale * ca * cb;
        m(1,2) = scale * -ca * sb;
        m(2,0) = scale * -sh * ca;
        m(2,1) = scale * (sh * sina * cb + ch * sb);
        m(2,2) = scale * (-sh * sina * sb + ch * cb);

        dlib::vector<T, 3> B;
        B(0) = dx;
        B(1) = dy;
        B(2) = dz;

        return dlib::point_transform_affine3d(m, B);
    }

	/*!
	requires
	- points_vector must contain at least one point
	ensures
	-  Returns the mean of a set of 3D points
	!*/
	template <typename T>
	point3<T> mean_point3d(const std::vector<point3<T>>& points_vector)
	{
		DLIB_ASSERT(points_vector.size() > 0, "mean_point3d points_vector must have at least one point");

		auto mean_point = point3<T>(0, 0, 0);
		for (const auto & pt : points_vector)
		{
			mean_point += pt;
		}
		mean_point /= static_cast<int>(points_vector.size());

		return mean_point;
	}


    /*!
    Calculate the weighted mean of the supplied 3d point vector
    - Requires
        - points_vector must have at least one element
        - weights vector must be same size as points_vector

    - Ensures
        - Returns the weighted mean of the supplied points
    */
    template <typename T>
    point3<T> mean_point3d(const std::vector<point3<T>>& points_vector,
        const std::vector<T> & weights)
    {
        DLIB_ASSERT(points_vector.size() > 0, "mean_point_3d points vector must have at least one point");
        DLIB_ASSERT(weights.size() > 0, "mean_point_3d weights vector must have at least one point");
        DLIB_ASSERT(weights.size() == points_vector.size(), "mean_point_3d weights vector must be same length as points vector");


        auto mean_point = point3<T>(0, 0, 0);
        T total_weight = 0;
        for (unsigned int i = 0; i < points_vector.size(); i++)
        {
            mean_point += points_vector[i] * weights[i];
            total_weight += weights[i];
        }
        mean_point /= total_weight;

        return mean_point;
    }


	/*!
	Centre the supplied vector of 3D points at the origin
	- Requires
		- points_vector must have at least one element

	- Ensures
		- Returns a new shape vector where the mean of the vector is at the origin
	*/
	template <typename T>
	std::vector<point3<T>> centre_shape3d_at_origin(const std::vector<point3<T>>& points_vector)
	{
		DLIB_ASSERT(points_vector.size() > 0, "centre_shape3d_at_origin: points vector must have at least one point");
		auto mean_point = point3<T>(0, 0, 0);
		for (const auto & pt : points_vector)
		{
			mean_point += pt;
		}
		mean_point /= static_cast<int>(points_vector.size());
		std::vector<point3<T>> output;
        output.reserve(points_vector.size());
		for (const auto & pt : points_vector)
		{
			output.push_back(pt - mean_point);
		}

		return output;
	}

    /*!
    Centre the supplied vector of 3D points at the origin
    - Requires
        - points_vector must have at least one element

    - Ensures
        - Returns a new shape vector where the mean of the vector is at the origin
    */
    template <typename T>
    std::vector<point3<T>> centre_shape3d_at_origin(const std::vector<point3<T>>& points_vector, const std::vector<T> & weights)
    {
        DLIB_ASSERT(points_vector.size() > 0, "centre_shape3d_at_origin points vector must have at least one point");
        DLIB_ASSERT(weights.size() > 0, "centre_shape3d_at_origin weights vector must have at least one point");
        DLIB_ASSERT(weights.size() == points_vector.size(), "centre_shape3d_at_origin weights vector must be same length as points vector");


        auto mean_point = mean_point3d(points_vector, weights);

        std::vector<point3<T>> output;
        output.reserve(points_vector.size());
        for (unsigned int i = 0; i < points_vector.size(); i++)
        {
            output.push_back((points_vector[i] - mean_point) * weights[i]);
        }

        return output;
    }

	/*!
	Calculate the sum of squared magnitudes of a vector of 3D points

	- Ensures
		- Returns sum squared magnitude
	*/
	template <typename T>
	T points3d_vector_magnitude_squared(const std::vector<point3<T>>& points_vector)
	{
		T mag = 0.0;
		for (const auto & pt : points_vector)
		{
			mag += (pt.x() * pt.x());
			mag += (pt.y() * pt.y());
			mag += (pt.z() * pt.z());
		}

		return mag;
	}

	/*!
	Convert a vector of 3D points to a matrix.

	- Ensures
		- Returns the matrix of n_points x 3 size
	*/
	template <typename T>
	dlib::matrix<T> points3d_vector_to_matrix(const std::vector<point3<T>>& points_vector)
	{
		auto n_points = points_vector.size();
		dlib::matrix<T> points_matrix(static_cast<long>(n_points), 3);
		int count = 0;
		for (const auto & pt : points_vector)
		{
			points_matrix(count, 0) = pt.x();
			points_matrix(count, 1) = pt.y();
			points_matrix(count, 2) = pt.z();
			count++;
		}

		return points_matrix;

	}


	/*!
	Calculate the sqrt of a matrix M using svd

	- Ensures
		- Returns the matrix of n_points x 3 size
	*/
	template <typename T>
	dlib::matrix<T> sqrtm(const dlib::matrix<T>& M)
	{
		/*
		sqrtM:
		Compute the matrix square root using svd
		(U, S, VT) = numpy.linalg.svd(M)
		D = numpy.diag(numpy.sqrt(S))
		return numpy.dot(numpy.dot(U, D), VT)
		*/

		dlib::matrix<T> sqrt_M;
		dlib::matrix<T> u;
		dlib::matrix<T> w;
		dlib::matrix<T> v;

		dlib::svd(M, u, w, v);

		dlib::matrix<T> sqrtw(w.nr(), w.nc());
		for (auto i_r = 0; i_r < w.nr(); i_r++)
		{
			for (auto i_c = 0; i_c < w.nc(); i_c++)
			{
				sqrtw(i_r, i_c) = std::sqrt(w(i_r, i_c));
			}
		}

		sqrt_M = (u * sqrtw) * trans(v);

		return sqrt_M;
	}


	/*!
	ensures
	-  Returns a 3d similarity transform minimising the L2 norm between the target and transformed source points
	- NOTE: This is an implementation of Horn's method
	- Horn, B, K, P, Hilden, H, M and Negahdaripour, S. "Closed-form solution of absolute orientation using orthonormal matrices", Journal of the Optical Society of America A, Vol. 5, page 1127, July 1988
	!*/
	template <typename T>
	dlib::point_transform_affine3d compute_alignment_similarity3d(const std::vector<point3<T>>& points_src, const std::vector<point3<T>>& points_tgt)
	{
		/*
		# Centre the data
		source_t = numpy.mean(sourcePoints[:,alignmentIndices],1)
		target_t = numpy.mean(targetPoints[:,alignmentIndices],1)

		sourcePointsCentred = sourcePoints[:,alignmentIndices] - numpy.tile(source_t,(numPoints,1)).T
		targetPointsCentred = targetPoints[:,alignmentIndices] - numpy.tile(target_t,(numPoints,1)).T

		# Work out the scale
		s = numpy.sqrt(  (sum(targetPointsCentred.flatten(1)**2))/(sum(sourcePointsCentred.flatten(1)**2))  )

		# Work out the rotation matrix
		M = numpy.dot(targetPointsCentred,sourcePointsCentred.T)
		#R = numpy.linalg.solve(M,SqrtM(numpy.dot(M.T,M)))
		R = numpy.dot(M,numpy.linalg.pinv(matrix.SqrtM(numpy.dot(M.T,M))))

		# Work out the translation
		t = target_t - numpy.dot(s*R,source_t)

		sqrtM:
		Compute the matrix square root using svd
		(U, S, VT) = numpy.linalg.svd(M)
		D = numpy.diag(numpy.sqrt(S))
		return numpy.dot(numpy.dot(U, D), VT)
		*/

		// Centre the point sets
		auto mean_point_src = mean_point3d(points_src);
		auto mean_point_tgt = mean_point3d(points_tgt);

		auto points_src_centred = centre_shape3d_at_origin(points_src);
		auto points_tgt_centred = centre_shape3d_at_origin(points_tgt);

		// Work out the scale
		auto points_src_mag_sq = points3d_vector_magnitude_squared(points_src_centred);
		auto points_tgt_mag_sq = points3d_vector_magnitude_squared(points_tgt_centred);
		auto s = std::sqrt(points_tgt_mag_sq / points_src_mag_sq);

		// Rotation matrix
		auto points_src_centred_matrix = points3d_vector_to_matrix(points_src_centred);
		auto points_tgt_centred_matrix = points3d_vector_to_matrix(points_tgt_centred);
		dlib::matrix<T> M = trans(points_tgt_centred_matrix) * points_src_centred_matrix;

		dlib::matrix<T> MTM = trans(M) * M;
		dlib::matrix<T> sqrt_MTM = sqrtm(MTM);

		dlib::matrix<T> R = M * pinv(sqrt_MTM);

		// Translation
		dlib::matrix<T, 3, 1> t_src;
		dlib::matrix<T, 3, 1> t_tgt;
		t_src(0, 0) = mean_point_src.x();
		t_src(1, 0) = mean_point_src.y();
		t_src(2, 0) = mean_point_src.z();
		t_tgt(0, 0) = mean_point_tgt.x();
		t_tgt(1, 0) = mean_point_tgt.y();
		t_tgt(2, 0) = mean_point_tgt.z();
		dlib::matrix<T> t = t_tgt - ((s * R) * t_src);

		return dlib::point_transform_affine3d(s * R, t);
	}

	/*!
	ensures
	-  Returns a 3d similarity transform minimising the L2 norm between the target and transformed source points
	- NOTE: This is an implementation of Horn's method
	- Horn, B, K, P, Hilden, H, M and Negahdaripour, S. "Closed-form solution of absolute orientation using orthonormal matrices", Journal of the Optical Society of America A, Vol. 5, page 1127, July 1988
	!*/
	template <typename T>
	dlib::point_transform_affine3d compute_alignment_rigid3d(const std::vector<point3<T>>& points_src, const std::vector<point3<T>>& points_tgt)
	{
		/*
		# Centre the data
		source_t = numpy.mean(sourcePoints[:,alignmentIndices],1)
		target_t = numpy.mean(targetPoints[:,alignmentIndices],1)

		sourcePointsCentred = sourcePoints[:,alignmentIndices] - numpy.tile(source_t,(numPoints,1)).T
		targetPointsCentred = targetPoints[:,alignmentIndices] - numpy.tile(target_t,(numPoints,1)).T

		# Work out the scale
		s = numpy.sqrt(  (sum(targetPointsCentred.flatten(1)**2))/(sum(sourcePointsCentred.flatten(1)**2))  )

		# Work out the rotation matrix
		M = numpy.dot(targetPointsCentred,sourcePointsCentred.T)
		#R = numpy.linalg.solve(M,SqrtM(numpy.dot(M.T,M)))
		R = numpy.dot(M,numpy.linalg.pinv(matrix.SqrtM(numpy.dot(M.T,M))))

		# Work out the translation
		t = target_t - numpy.dot(s*R,source_t)

		sqrtM:
		Compute the matrix square root using svd
		(U, S, VT) = numpy.linalg.svd(M)
		D = numpy.diag(numpy.sqrt(S))
		return numpy.dot(numpy.dot(U, D), VT)
		*/

		// Centre the point sets
		auto mean_point_src = mean_point3d(points_src);
		auto mean_point_tgt = mean_point3d(points_tgt);

		auto points_src_centred = centre_shape3d_at_origin(points_src);
		auto points_tgt_centred = centre_shape3d_at_origin(points_tgt);


		// Rotation matrix
		auto points_src_centred_matrix = points3d_vector_to_matrix(points_src_centred);
		auto points_tgt_centred_matrix = points3d_vector_to_matrix(points_tgt_centred);
		dlib::matrix<T> M = trans(points_tgt_centred_matrix) * points_src_centred_matrix;

		dlib::matrix<T> MTM = trans(M) * M;
		dlib::matrix<T> sqrt_MTM = sqrtm(MTM);
		dlib::matrix<T> R = M * pinv(sqrt_MTM);

		// Translation
		dlib::matrix<T, 3, 1> t_src;
		dlib::matrix<T, 3, 1> t_tgt;
		t_src(0, 0) = mean_point_src.x();
		t_src(1, 0) = mean_point_src.y();
		t_src(2, 0) = mean_point_src.z();
		t_tgt(0, 0) = mean_point_tgt.x();
		t_tgt(1, 0) = mean_point_tgt.y();
		t_tgt(2, 0) = mean_point_tgt.z();
		dlib::matrix<T> t = t_tgt - (R * t_src);

		return dlib::point_transform_affine3d(R, t);
	}

	/*!
	ensures
	-  Returns the translation between the target and transformed source points
	!*/
	template <typename T>
	dlib::point_transform_affine3d compute_alignment_translation3d(const std::vector<point3<T>>& points_src, const std::vector<point3<T>>& points_tgt)
	{
		// Centre the point sets
		auto mean_point_src = mean_point3d(points_src);
		auto mean_point_tgt = mean_point3d(points_tgt);


		// Translation
		dlib::matrix<T, 3, 1> t_src;
		dlib::matrix<T, 3, 1> t_tgt;
		t_src(0, 0) = mean_point_src.x();
		t_src(1, 0) = mean_point_src.y();
		t_src(2, 0) = mean_point_src.z();
		t_tgt(0, 0) = mean_point_tgt.x();
		t_tgt(1, 0) = mean_point_tgt.y();
		t_tgt(2, 0) = mean_point_tgt.z();
		dlib::matrix<T> t = t_tgt - t_src;

		return dlib::point_transform_affine3d(identity_matrix<T>(3), t);

	}


	/*!
	 Find the transform which best aligns two sets of 3d points
	 using a given alignment method.

	 - Requires
		 - from_points.size() == to_points.size()
		 - from_points.size() >= 0,1 or 3 depending on the alignment type,
		 which can be: none, translation, rigid or similarity
		 - none simply returns the null transform and requires no points
		 - translation needs 1 or more points
		 - rigid, similarity need 3 or more points
	
	 - Ensures
		 - returns a point_transform_affine3d object, P, such that for all valid i:
		 length(P(from_points[i]) - to_points[i])
		 is minimized as often as possible.  That is, this function finds the
		 transform that maps points in from_points to points in to_points.  If no
		 transform exists which performs this mapping exactly then the one
		 which minimizes the mean squared error is selected.
		 - NOTE: the result is returned as a general affine 3d transform, even though
		 all cases are special cases of the affine 3d transform.
	*/
	template <typename T>
	dlib::point_transform_affine3d find_transform_as_affine3d(
		const std::vector<point3<T>>& from_points,
		const std::vector<point3<T>>& to_points,
		const alignment3d_type align_method
	)
	{
		// make sure requires clause is not broken
		DLIB_ASSERT(from_points.size() == to_points.size() &&
			((align_method == alignment3d_type::none && from_points.size() >= 0) ||
			(align_method == alignment3d_type::translation && from_points.size() >= 1) ||
				(align_method == alignment3d_type::rigid && from_points.size() >= 3) ||
				(align_method == alignment3d_type::similarity && from_points.size() >= 3) ||
				(from_points.size() >= 3)),
			"\t point_transform_projective find_affine_transform_as_projective(from_points, to_points)"
			<< "\n\t Invalid inputs were given to this function."
			<< "\n\t from_points.size(): " << from_points.size()
			<< "\n\t to_points.size():   " << to_points.size()
			<< "\n\t align_method:   " << to_string(align_method)
		);

		dlib::point_transform_affine3d xform;

		switch (align_method)
		{
		case alignment3d_type::none:
			xform = dlib::point_transform_affine3d();
			break;
		case alignment3d_type::similarity:
			xform = compute_alignment_similarity3d(from_points, to_points);
			break;
		case alignment3d_type::rigid:
			xform = compute_alignment_rigid3d(from_points, to_points);
			break;
		case alignment3d_type::translation:
			xform = compute_alignment_translation3d(from_points, to_points);
			break;
		default:
			throw std::runtime_error("find_transform_as_affine3d: unexpected 3d alignment type");
			break;
		}
		return xform;
	}


	/*!
	Divide the supplied vector of points by its overall norm
	- Requires
		- points_vector must have at least one element

	- Ensures
		- Returns a new shape vector which is the input vector divided by its overall norm
	*/
	template <typename T>
	std::vector<point3<T>> scale_to_unit_norm3d(const std::vector<point3<T>>& points_vector)
	{
		DLIB_ASSERT(points_vector.size() > 0, "scale_to_unit_norm3d points vector must have at least one point");

		auto norm_squared = 0.0;
		for (const auto & point : points_vector)
		{
			norm_squared += point.x()*point.x() + point.y()*point.y() + point.z()*point.z();
		}
		auto norm = std::sqrt(norm_squared);

		std::vector<point3<T>> output;
        output.reserve(points_vector.size());
		for (const auto & point : points_vector)
		{
			output.push_back(point3<T>(point.x() / norm, point.y() / norm, point.z() / norm));
		}
		return output;
	}


	/*!
		Apply a 3d affine transform to every point in the shape vector

		- Ensures
			- Returns a new shape vector where every point has been transformed
			  by the given 3d affine transform
	*/
	template <typename T>
	std::vector<point3<T>> transform_shape_affine3d(
		const std::vector<point3<T>>& from_points,
		const dlib::point_transform_affine3d& xform
	)
	{
		std::vector<point3<T>> result(from_points.size());
		auto count = 0;
		for (auto &pt : from_points)
		{
			result[count++] = xform(pt);
		}
		return result;
	}


	/*!
	requires
	- shapes must have at least one element, and all shapes must contain the same number of points
	ensures
	-  Returns the mean shape of a set of shapes
	!*/
	template <typename T>
	std::vector<point3<T>> mean_shape3d(const std::vector<std::vector<point3<T>>>& shapes)
	{
		DLIB_ASSERT(shapes.size() > 0, "cm::mean_shape_3d shapes must have size() > 0");
#if defined(ENABLE_ASSERTS)
		for (const auto & shape : shapes)
		{
			DLIB_ASSERT(shape.size() == shapes[0].size(), "cm::mean_shape_3d all shapes must have same size.");
		}
#endif

		std::vector<point3<T>> mean_shape(shapes[0].size(), point3<T>(0, 0, 0));
		for (const auto & shape : shapes)
		{
			for (unsigned i = 0; i < shape.size(); i++)
			{
				mean_shape[i] += shape[i];
			}
		}
		for (auto& pt : mean_shape)
		{
			pt /= static_cast<int>(shapes.size());
		}
		return mean_shape;
	}


	/*!
	Calculate the norm between 2 3D shapes
	- Requires
		- points_vectors must have at least one element
		- points_vector_.size() must equal points_vector_b.size()

	- Ensures
		- Returns the calculated norm between the 2 shapes
	*/
	template <typename T>
	T norm_between_shapes3d(const std::vector<point3<T>>& points_vector_a, 
		const std::vector<point3<T>>& points_vector_b)
	{
		DLIB_ASSERT(points_vector_a.size() > 0, "norm_between_shapes3d: points_vector_a must have size() > 0");
		DLIB_ASSERT(points_vector_a.size() == points_vector_b.size(), "norm_between_shapes3d: points_vector_a and points_vector_b must have same size.");

		auto norm_squared = 0.0;
		const unsigned n_points = static_cast<unsigned>(points_vector_a.size());

		for (unsigned i_point = 0; i_point < n_points; ++i_point)
		{
			auto dx = points_vector_a[i_point].x() - points_vector_b[i_point].x();
			auto dy = points_vector_a[i_point].y() - points_vector_b[i_point].y();
			auto dz = points_vector_a[i_point].z() - points_vector_b[i_point].z();
			norm_squared += dx * dx + dy * dy + dz * dz;
		}
		return std::sqrt(norm_squared);
	}

	/*!
	requires
	- vectors_all_same_size_with_at_least_n_elements(points_data,2) == true
	ensures
	- Returns a the 'mean shape' (in the appropriate sense) of the Procrustes process, for
	the points specified by alignment_ids using the align_method specified.
	!*/
	template <typename T>
	std::vector<point3<T>> compute_procrustes_base_shape3d(const std::vector<std::vector<point3<T>>>& points_data,
		const alignment3d_type& align_method)
	{
		//Checks
		//-----------------------------------------------------------------------------------------------------------------
		DLIB_ASSERT(vectors_all_same_size_with_at_least_n_elements(points_data, 2), "compute_procrustes_base_shape3d:: all points_data must be the same size");
		//-----------------------------------------------------------------------------------------------------------------
		using shape = std::vector<point3<T>>;
		using set_of_shapes = std::vector<shape>;

		auto tolerance = 1e-10;

		set_of_shapes centred_shapes;
        centred_shapes.reserve(points_data.size());
		for (const auto& item : points_data)
		{
			centred_shapes.push_back(centre_shape3d_at_origin(item));
		}
		shape current_mean_shape;

		if (alignment3d_type::similarity == align_method)
		{
			auto new_mean_shape = scale_to_unit_norm3d(centred_shapes[0]);
			auto reference_shape = shape(new_mean_shape);
			auto current_aligned_set_of_shapes(centred_shapes);
			do
			{
				current_mean_shape = shape(new_mean_shape);
				for (auto& this_shape : current_aligned_set_of_shapes)
				{
					this_shape = transform_shape_affine3d(this_shape, compute_alignment_similarity3d(this_shape, current_mean_shape));
				}
				new_mean_shape = mean_shape3d(current_aligned_set_of_shapes);
				new_mean_shape = transform_shape_affine3d(new_mean_shape, compute_alignment_similarity3d(new_mean_shape, reference_shape));
				new_mean_shape = scale_to_unit_norm3d(new_mean_shape);
			} while (norm_between_shapes3d(new_mean_shape, current_mean_shape) > tolerance);
		}
		else if (alignment3d_type::rigid == align_method)
		{
			auto new_mean_shape = centred_shapes[0];
			auto reference_shape = shape(new_mean_shape);
			auto current_aligned_set_of_shapes(centred_shapes);
			do
			{
				current_mean_shape = shape(new_mean_shape);
				for (auto& this_shape : current_aligned_set_of_shapes)
				{
					this_shape = transform_shape_affine3d(this_shape, compute_alignment_rigid3d(this_shape, current_mean_shape));
				}
				new_mean_shape = mean_shape3d(current_aligned_set_of_shapes);
				new_mean_shape = transform_shape_affine3d(new_mean_shape, compute_alignment_rigid3d(new_mean_shape, reference_shape));
			} while (norm_between_shapes3d(new_mean_shape, current_mean_shape) > tolerance);
		}
		else if (alignment3d_type::translation == align_method)
		{
			current_mean_shape = mean_shape3d(centred_shapes);
		}
		else if (alignment3d_type::none == align_method)
		{
			current_mean_shape = mean_shape3d(points_data);
		}
		return current_mean_shape;
	}

	/*!
	ensures
	-  Returns a 3d similarity transform minimising the L2 norm between the target and transformed source points
	- A weighted version of the algorithm above
	!*/
	template <typename T>
	dlib::point_transform_affine3d compute_alignment_similarity3d(const std::vector<point3<T>>& points_src, const std::vector<point3<T>>& points_tgt,
		const std::vector<T> & weights)
	{


		// Centre the point sets (weighted)
		auto mean_point_src = mean_point3d(points_src, weights);
		auto mean_point_tgt = mean_point3d(points_tgt, weights);

		// work out the centred points (weighted)
		auto points_src_centred = centre_shape3d_at_origin(points_src, weights);
		auto points_tgt_centred = centre_shape3d_at_origin(points_tgt, weights);

		// Work out the scale
		auto points_src_mag_sq = points3d_vector_magnitude_squared(points_src_centred);
		auto points_tgt_mag_sq = points3d_vector_magnitude_squared(points_tgt_centred);
		T s = std::sqrt(points_tgt_mag_sq / points_src_mag_sq);
		//cout << "scale: " << s << endl;

		// Rotation matrix
		auto points_src_centred_matrix = points3d_vector_to_matrix(points_src_centred);
		auto points_tgt_centred_matrix = points3d_vector_to_matrix(points_tgt_centred);
		dlib::matrix<T> M = trans(points_tgt_centred_matrix) * points_src_centred_matrix;

		dlib::matrix<T> MTM = trans(M) * M;
		dlib::matrix<T> sqrt_MTM = sqrtm(MTM);

		dlib::matrix<T> R = M * pinv(sqrt_MTM);

		// Translation
		dlib::matrix<T, 3, 1> t_src;
		dlib::matrix<T, 3, 1> t_tgt;
		t_src(0, 0) = mean_point_src.x();
		t_src(1, 0) = mean_point_src.y();
		t_src(2, 0) = mean_point_src.z();
		t_tgt(0, 0) = mean_point_tgt.x();
		t_tgt(1, 0) = mean_point_tgt.y();
		t_tgt(2, 0) = mean_point_tgt.z();
		dlib::matrix<T> t = t_tgt - ((s * R) * t_src);

		return dlib::point_transform_affine3d(s * R, t);
	}

	/*!
	ensures
	-  Returns a 3d similarity transform minimising the L2 norm between the target and transformed source points
	-  A weighted version of the algorithm above
	!*/
	template <typename T>
	dlib::point_transform_affine3d compute_alignment_rigid3d(const std::vector<point3<T>>& points_src, const std::vector<point3<T>>& points_tgt,
		const std::vector<T> & weights)

	{

		// Centre the point sets (weighted)
		auto mean_point_src = mean_point3d(points_src, weights);
		auto mean_point_tgt = mean_point3d(points_tgt, weights);

		// work out the centred points (weighted)
		auto points_src_centred = centre_shape3d_at_origin(points_src, weights);
		auto points_tgt_centred = centre_shape3d_at_origin(points_tgt, weights);


		// Rotation matrix
		auto points_src_centred_matrix = points3d_vector_to_matrix(points_src_centred);
		auto points_tgt_centred_matrix = points3d_vector_to_matrix(points_tgt_centred);
		dlib::matrix<T> M = trans(points_tgt_centred_matrix) * points_src_centred_matrix;

		dlib::matrix<T> MTM = trans(M) * M;
		dlib::matrix<T> sqrt_MTM = sqrtm(MTM);
		dlib::matrix<T> R = M * pinv(sqrt_MTM);

		// Translation
		dlib::matrix<T, 3, 1> t_src;
		dlib::matrix<T, 3, 1> t_tgt;
		t_src(0, 0) = mean_point_src.x();
		t_src(1, 0) = mean_point_src.y();
		t_src(2, 0) = mean_point_src.z();
		t_tgt(0, 0) = mean_point_tgt.x();
		t_tgt(1, 0) = mean_point_tgt.y();
		t_tgt(2, 0) = mean_point_tgt.z();
		dlib::matrix<T> t = t_tgt - (R * t_src);

		return dlib::point_transform_affine3d(R, t);
	}

	/*!
	ensures
	-  Returns the translation between the target and transformed source points
	-  A weighted version of the algorithm above
	!*/
	template <typename T>
	dlib::point_transform_affine3d compute_alignment_translation3d(const std::vector<point3<T>>& points_src, const std::vector<point3<T>>& points_tgt,
		const std::vector<T> & weights)
	{
		// Centre the point sets
		auto mean_point_src = mean_point3d(points_src, weights);
		auto mean_point_tgt = mean_point3d(points_tgt, weights);


		// Translation
		dlib::matrix<T, 3, 1> t_src;
		dlib::matrix<T, 3, 1> t_tgt;
		t_src(0, 0) = mean_point_src.x();
		t_src(1, 0) = mean_point_src.y();
		t_src(2, 0) = mean_point_src.z();
		t_tgt(0, 0) = mean_point_tgt.x();
		t_tgt(1, 0) = mean_point_tgt.y();
		t_tgt(2, 0) = mean_point_tgt.z();
		dlib::matrix<T> t = t_tgt - t_src;

		return dlib::point_transform_affine3d(identity_matrix<T>(3), t);

	}

}
