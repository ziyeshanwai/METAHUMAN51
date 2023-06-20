// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "data_types.h"
#include "disable_dlib_warnings.h"
#include "simple_geometry.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/geometry.h>
POSEBASEDSOLVER_RENABLE_WARNINGS

namespace cm
{
    using namespace dlib;

    /*!
        Exception in transform computation or application
    */
    struct transform_error : public error
    {
        /*!
            Tranform error constructor
        */
        explicit transform_error(const std::string& message) : error(message) {}
    };

    /*!
        Apply a projective transform to every point in the shape vector
    
        - Ensures
            - Returns a new shape vector where every point has been transformed
              by the given projective transform
    */
    template <typename T>
    std::vector<point2<T>> transform_shape(
        const std::vector<point2<T>>& from_points,
        const point_transform_projective& xform
    )
    {
        std::vector<point2<T>> result(from_points.size());
        auto count = 0;
        for (auto &pt : from_points)
        {
            result[count++] = xform(pt);
        }
        return result;
    }

	/*!
		 Compute a weighted mean of the points
		 - Requires
		     - shape.size() == weights.size()
		 - Ensures
			 - Returns a new shape map where every point has been transformed
			   by the given projective transform. If the weights sum to zero this
			   function returns (0,0). Note - weights are always considered positive
			   even if you supply negative values - you shouldn't!
	*/
	template<typename T>
	point2<T> weighted_mean_point(const std::vector<point2<T>>& shape, const std::vector<T>& weights)
	{
		// make sure requires clause is not broken
		DLIB_ASSERT(shape.size() == weights.size(),
			"\t weighted_mean_point(shape, weights)"
			<< "\n\t Invalid inputs were given to this function."
			<< "\n\t shape.size(): " << shape.size()
			<< "\n\t weights.size():   " << weights.size()
		);
		point2<T> mean_pt{ 0,0 };
		T total_weight = 0;
		for (size_t i = 0; i < shape.size(); ++i)
		{
			mean_pt += shape[i] * std::abs(weights[i]);
			total_weight += std::abs(weights[i]);
		}
		if (total_weight > 0)
		{
			mean_pt /= total_weight;
		}
		else
		{
			mean_pt = point2<T>(0, 0);
		}
		return mean_pt;
	}

	
    /*!
         Apply a projective transform to every point in the shape map
     
         - Ensures
             - Returns a new shape map where every point has been transformed
               by the given projective transform
    */
    template <typename T, typename K>
    std::map<K,point2<T>> transform_shape(
        const std::map<K, point2<T>>& from_points,
        const point_transform_projective& xform
    )
    {
        std::map<K, point2<T>> result;
        for (auto &item : from_points)
        {
            result[item.first] = xform(item.second);
        }
        return result;
    }

    /*!
        Find the projective transform which best aligns two sets of 2d points.
     
        - Requires
            - from_points.size() == to_points.size()
            - from_points.size() >= 4
        - Ensures
            - returns a point_transform_projective object, P, such that for all valid i:
              length(P(from_points[i]) - to_points[i])
              is minimized as often as possible.  That is, this function finds the
              transform that maps points in from_points to points in to_points.  If no
              transform exists which performs this mapping exactly then the one
              which minimizes the mean squared error is selected.
    */
    template <typename T>
    point_transform_projective find_projective_transform_as_projective(
        const std::vector<point2<T> >& from_points,
        const std::vector<point2<T> >& to_points
    )
	{
        // make sure requires clause is not broken
        DLIB_ASSERT(from_points.size() == to_points.size() &&
                    from_points.size() >= 4,
            "\t point_transform_projective find_projective_transform_as_projective(from_points, to_points)"
            << "\n\t Invalid inputs were given to this function."
            << "\n\t from_points.size(): " << from_points.size()
            << "\n\t to_points.size():   " << to_points.size()
            );

        return find_projective_transform(cast_points_to_double(from_points), cast_points_to_double(to_points));              
	}

    /*!
        Find the affine transform which best aligns two sets of 2d points,
        returning the result as a (special case) projective transform
        
        - Requires
            - from_points.size() == to_points.size()
            - from_points.size() >= 3
        - Ensures
            - returns a point_transform_projective object, P, such that for all valid i:
              length(P(from_points[i]) - to_points[i])
              is minimized as often as possible using an affine transform.  
              That is, this function finds the transform that maps points in from_points 
              to points in to_points.  If no transform exists which performs this mapping 
              exactly then the one which minimizes the mean squared error is selected.
    */
    template <typename T>
    point_transform_projective find_affine_transform_as_projective(
        const std::vector<point2<T> >& from_points,
        const std::vector<point2<T> >& to_points
    )
	{
        // make sure requires clause is not broken
        DLIB_ASSERT(from_points.size() == to_points.size() &&
                    from_points.size() >= 3,
            "\t point_transform_projective find_affine_transform_as_projective(from_points, to_points)"
            << "\n\t Invalid inputs were given to this function."
            << "\n\t from_points.size(): " << from_points.size()
            << "\n\t to_points.size():   " << to_points.size()
            );

        return (is_same_type<T, double>::value) ?
            point_transform_projective(find_affine_transform(from_points, to_points)) :
            point_transform_projective(find_affine_transform(cast_points_to_double(from_points), cast_points_to_double(to_points)));       
	}

    /*!
        Find the similarity transform which best aligns two sets of 2d points,
        returning the result as a (special case) projective transform
        
        - Requires
            - from_points.size() == to_points.size()
            - from_points.size() >= 2
        - Ensures
            - returns a point_transform_projective object, P, such that for all valid i:
              length(P(from_points[i]) - to_points[i])
              is minimized as often as possible using a similarity transform.  
              That is, this function finds the transform that maps points in from_points 
              to points in to_points.  If no transform exists which performs this mapping 
              exactly then the one which minimizes the mean squared error is selected.
    */
    template <typename T>
    point_transform_projective find_similarity_transform_as_projective(
        const std::vector<point2<T> >& from_points,
        const std::vector<point2<T> >& to_points
    )
	{
        // make sure requires clause is not broken
        DLIB_ASSERT(from_points.size() == to_points.size() &&
                    from_points.size() >= 2,
            "\t find_similarity_transform_as_projective(from_points, to_points)"
            << "\n\t Invalid inputs were given to this function."
            << "\n\t from_points.size(): " << from_points.size()
            << "\n\t to_points.size():   " << to_points.size()
            );


        auto mean_from = mean_point(from_points);
        auto mean_to = mean_point(to_points);

        dlib::matrix<T, 1, 0> centred_from_xvec(static_cast<long>(from_points.size()));
        dlib::matrix<T, 1, 0> centred_from_yvec(static_cast<long>(from_points.size()));
        dlib::matrix<T, 1, 0> centred_to_xvec(static_cast<long>(to_points.size()));
        dlib::matrix<T, 1, 0> centred_to_yvec(static_cast<long>(to_points.size()));
        for (size_t i = 0; i < from_points.size(); ++i)
        {
            centred_from_xvec(static_cast<long>(i)) = from_points[i].x() - mean_from.x();
            centred_from_yvec(static_cast<long>(i)) = from_points[i].y() - mean_from.y();
        }
        for (size_t i = 0; i < to_points.size(); ++i)
        {
            centred_to_xvec(static_cast<long>(i)) = to_points[i].x() - mean_to.x();
            centred_to_yvec(static_cast<long>(i)) = to_points[i].y() - mean_to.y();
        }

        auto s_xsxt = dlib::sum(dlib::pointwise_multiply(centred_from_xvec, centred_to_xvec));
        auto s_ysyt = dlib::sum(dlib::pointwise_multiply(centred_from_yvec, centred_to_yvec));
        auto s_xsyt = dlib::sum(dlib::pointwise_multiply(centred_from_xvec, centred_to_yvec));
        auto s_ysxt = dlib::sum(dlib::pointwise_multiply(centred_from_yvec, centred_to_xvec));

        auto v = dlib::sum(dlib::pointwise_multiply(centred_from_xvec, centred_from_xvec) + dlib::pointwise_multiply(centred_from_yvec, centred_from_yvec));

        auto a = (s_xsxt + s_ysyt) / v;
        auto b = (s_xsyt - s_ysxt) / v;
        auto tx = mean_to.x() - (a * mean_from.x()) + (b * mean_from.y());
        auto ty = mean_to.y() - (b * mean_from.x()) - (a * mean_from.y());

        dlib::matrix<double, 2, 2> m;
        dlib::matrix<double, 2, 1> B;
        m(0, 0) = a; m(0, 1) = -b;
        m(1, 0) = b; m(1, 1) = a;
        B(0) = tx;
        B(1) = ty;


        return point_transform_projective(join_cols(join_rows(m, B), matrix<double, 1, 3>({ 0,0,1 })));
	} 

	/*!
		Find the similarity transform which best aligns two sets of 2d points,
		returning the result as a (special case) projective transform

		- Requires
			- from_points.size() == to_points.size()
			- from_points.size() >= 2
			- weight.size() == from_points.size()
		- Ensures
			- returns a point_transform_projective object, P, such that for all valid i:
			  weights[i] * length(P(from_points[i]) - to_points[i])
			  is minimized as often as possible using a similarity transform.
			  That is, this function finds the transform that maps points in from_points
			  to points in to_points.  If no transform exists which performs this mapping
			  exactly then the one which minimizes the mean squared error is selected.
	*/
	template <typename T>
	point_transform_projective find_similarity_transform_as_projective_weighted(
		const std::vector<point2<T> >& from_points,
		const std::vector<point2<T> >& to_points,
		const std::vector<T>& weights
	)
	{
		// make sure requires clause is not broken
		DLIB_ASSERT(from_points.size() == weights.size() &&
			from_points.size() == to_points.size() &&
			from_points.size() >= 2,
			"\t find_similarity_transform_as_projective_wieghted(from_points, to_points, weights)"
			<< "\n\t Invalid inputs were given to this function."
			<< "\n\t from_points.size(): " << from_points.size()
			<< "\n\t to_points.size():   " << to_points.size()
			<< "\n\t weights.size():   " << weights.size()
		);

		auto mean_from = weighted_mean_point(from_points, weights);
		auto mean_to = weighted_mean_point(to_points, weights);

		dlib::matrix<T> weights_vec = dlib::ones_matrix<T>(1, static_cast<long>(weights.size()));

		dlib::matrix<T, 1, 0> centred_from_xvec(static_cast<long>(from_points.size()));
		dlib::matrix<T, 1, 0> centred_from_yvec(static_cast<long>(from_points.size()));
		dlib::matrix<T, 1, 0> centred_to_xvec(static_cast<long>(to_points.size()));
		dlib::matrix<T, 1, 0> centred_to_yvec(static_cast<long>(to_points.size()));
		for (size_t i = 0; i < from_points.size(); ++i)
		{
			centred_from_xvec(static_cast<unsigned>(i)) = from_points[i].x() - mean_from.x();
			centred_from_yvec(static_cast<unsigned>(i)) = from_points[i].y() - mean_from.y();
			weights_vec(static_cast<unsigned>(i)) = weights[i];
		}
		for (size_t i = 0; i < to_points.size(); ++i)
		{
			centred_to_xvec(static_cast<unsigned>(i)) = to_points[i].x() - mean_to.x();
			centred_to_yvec(static_cast<unsigned>(i)) = to_points[i].y() - mean_to.y();
		}

		auto s_xsxt = dlib::sum(dlib::pointwise_multiply(dlib::pointwise_multiply(centred_from_xvec, weights_vec), centred_to_xvec));
		auto s_ysyt = dlib::sum(dlib::pointwise_multiply(dlib::pointwise_multiply(centred_from_yvec, weights_vec), centred_to_yvec));
		auto s_xsyt = dlib::sum(dlib::pointwise_multiply(dlib::pointwise_multiply(centred_from_xvec, weights_vec), centred_to_yvec));
		auto s_ysxt = dlib::sum(dlib::pointwise_multiply(dlib::pointwise_multiply(centred_from_yvec, weights_vec), centred_to_xvec));

		auto v = dlib::sum(dlib::pointwise_multiply(weights_vec, dlib::pointwise_multiply(centred_from_xvec, centred_from_xvec) + dlib::pointwise_multiply(centred_from_yvec, centred_from_yvec)));

		auto a = (s_xsxt + s_ysyt) / v;
		auto b = (s_xsyt - s_ysxt) / v;
		auto tx = mean_to.x() - (a * mean_from.x()) + (b * mean_from.y());
		auto ty = mean_to.y() - (b * mean_from.x()) - (a * mean_from.y());

		dlib::matrix<double, 2, 2> m;
		dlib::matrix<double, 2, 1> B;
		m(0, 0) = a; m(0, 1) = -b;
		m(1, 0) = b; m(1, 1) = a;
		B(0) = tx;
		B(1) = ty;

		return point_transform_projective(join_cols(join_rows(m, B), matrix<double, 1, 3>({ 0,0,1 })));
	}

    /*!
        Create a similarity transform from scale, angle, dx, dy
        returning the result as a (special case) projective transform
        
        - Ensures
            - returns a point_transform_projective object, P, representing
              the similarity transform given by scale, angle, dx, dy          
    */
    template <typename T>
    point_transform_projective create_similarity_transform_as_projective(
        T scale,
        T angle,
        T dx,
        T dy
    )
	{
        const T a = scale * std::cos(angle);
        const T b = scale * std::sin(angle);
        dlib::matrix<double, 2, 2> m;
        dlib::matrix<double, 2, 1> B;
        m(0, 0) = a; m(0, 1) = -b;
        m(1, 0) = b; m(1, 1) = a;
        B(0) = dx;
        B(1) = dy;
        return point_transform_projective(join_cols(join_rows(m, B), matrix<double, 1, 3>({ 0,0,1 })));
	} 

    /*!
        Find the rigid transform which best aligns two sets of 2d points,
        returning the result as a (special case) projective transform
        
        - Requires
            - from_points.size() == to_points.size()
            - from_points.size() >= 2
        - Ensures
            - returns a point_transform_projective object, P, such that for all valid i:
              length(P(from_points[i]) - to_points[i])
              is minimized as often as possible using a rigid transform.  
              That is, this function finds the transform that maps points in from_points 
              to points in to_points.  If no transform exists which performs this mapping 
              exactly then the one which minimizes the mean squared error is selected.
    */
    template <typename T>
    point_transform_projective find_rigid_transform_as_projective(
        const std::vector<point2<T> >& from_points,
        const std::vector<point2<T> >& to_points
    )
	{
        // make sure requires clause is not broken
        DLIB_ASSERT(from_points.size() == to_points.size() &&
                    from_points.size() >= 2,
            "\t find_rigid_transform_as_projective(from_points, to_points)"
            << "\n\t Invalid inputs were given to this function."
            << "\n\t from_points.size(): " << from_points.size()
            << "\n\t to_points.size():   " << to_points.size()
            );

        dlib::vector<double, 2> mean_from, mean_to;
        matrix<double, 2, 2> cov({ 0,0,0,0 });

		for (unsigned long i = 0; i < from_points.size(); ++i)
		{
			mean_from += from_points[i];
			mean_to += to_points[i];
		}
		mean_from /= static_cast<double>( from_points.size());
		mean_to /= static_cast<double>( from_points.size());

		for (unsigned long i = 0; i < from_points.size(); ++i)
		{
			cov += (to_points[i] - mean_to)*trans(from_points[i] - mean_from);
		}

		cov /= static_cast<double>(from_points.size());

		matrix<double, 2, 2> u, v, d;
		svd(cov, u, d, v);
        matrix<double, 2, 2> s = identity_matrix(cov);
		if (det(cov) < 0 || (det(cov) == 0 && det(u)*det(v)<0))
		{
			if (d(1, 1) < d(0, 0))
				s(1, 1) = -1;
			else
				s(0, 0) = -1;
		}

		matrix<double, 2, 2> r = u*s*trans(v);
		dlib::vector<double, 2> t = mean_to - r*mean_from;

        return point_transform_projective(join_cols(join_rows(r,t), matrix<double, 1, 3>({ 0,0,1 })));
	}

	/*!
		Find the scale_translate transform which best aligns two sets of 2d points weighted,
		returning the result as a (special case) projective transform

		- Requires
			- from_points.size() == to_points.size()
			- from_points.size() >= 2
			- weights.size() == from_points.size()
		- Ensures
			- returns a point_transform_projective object, P, such that for all valid i:
			  weights[i]*(length(P(from_points[i]) - to_points[i])
			  is minimized as often as possible using a scale_translate transform.
			  That is, this function finds the transform that maps points in from_points
			  to points in to_points.  If no transform exists which performs this mapping
			  exactly then the one which minimizes the mean squared error is selected.
	*/
	template <typename T>
	point_transform_projective find_scale_translate_transform_as_projective_weighted(
		const std::vector<point2<T> >& from_points,
		const std::vector<point2<T> >& to_points,
		const std::vector<T>& weights
	)
	{
		auto mean_from = weighted_mean_point(from_points, weights);
		auto mean_to = weighted_mean_point(to_points, weights);

		T mag_to = 0.0;
		T mag_from = 0.0;
		for (unsigned long i = 0; i < from_points.size(); ++i)
		{
			mag_to += weights[i] * (to_points[i] - mean_to).length();
			mag_from += weights[i] * (from_points[i] - mean_from).length();
		}
		matrix<double, 2, 2> sr;
		sr(0, 0) = mag_to / mag_from; sr(0, 1) = 0;
		sr(1, 0) = 0, sr(1, 1) = sr(0, 0);
		dlib::vector<double, 2> t = mean_to - sr(0, 0)*mean_from;
		auto xform = point_transform_affine(sr, t);
		return xform;
	}

	/*!
		Find the rigid transform which best aligns two sets of 2d points weighted,
		returning the result as a (special case) projective transform

		- Requires
			- from_points.size() == to_points.size()
			- from_points.size() >= 2
			- weights.size() == from_points.size()
		- Ensures
			- returns a point_transform_projective object, P, such that for all valid i:
			  weights[i]*(length(P(from_points[i]) - to_points[i])
			  is minimized as often as possible using a rigid transform.
			  That is, this function finds the transform that maps points in from_points
			  to points in to_points.  If no transform exists which performs this mapping
			  exactly then the one which minimizes the mean squared error is selected.
	*/
	template <typename T>
	point_transform_projective find_rigid_transform_as_projective_weighted(
		const std::vector<point2<T> >& from_points,
		const std::vector<point2<T> >& to_points,
		const std::vector<T>& weights
	)
	{
		// make sure requires clause is not broken
		DLIB_ASSERT(from_points.size() == to_points.size() &&
			from_points.size() >= 2,
			"\t find_rigid_transform_as_projective_weighted(from_points, to_points, weights)"
			<< "\n\t Invalid inputs were given to this function."
			<< "\n\t from_points.size(): " << from_points.size()
			<< "\n\t to_points.size():   " << to_points.size()
			<< "\n\t weights.size():   " << weights.size()
		);


		auto mean_from = weighted_mean_point(from_points, weights);
		auto mean_to = weighted_mean_point(to_points, weights);
		dlib::matrix<double> weights_vec = dlib::ones_matrix<double>(1, static_cast<long>(weights.size()));

		dlib::matrix<double, 1, 0> centred_from_xvec(static_cast<long>(from_points.size()));
		dlib::matrix<double, 1, 0> centred_from_yvec(static_cast<long>(from_points.size()));
		dlib::matrix<double, 1, 0> centred_to_xvec(static_cast<long>(to_points.size()));
		dlib::matrix<double, 1, 0> centred_to_yvec(static_cast<long>(to_points.size()));
		for (size_t i = 0; i < from_points.size(); ++i)
		{
			centred_from_xvec(static_cast<unsigned>(i)) = from_points[i].x() - mean_from.x();
			centred_from_yvec(static_cast<unsigned>(i)) = from_points[i].y() - mean_from.y();
			weights_vec(static_cast<unsigned>(i)) = weights[i];
		}
		for (size_t i = 0; i < to_points.size(); ++i)
		{
			centred_to_xvec(static_cast<unsigned>(i)) = to_points[i].x() - mean_to.x();
			centred_to_yvec(static_cast<unsigned>(i)) = to_points[i].y() - mean_to.y();
		}

		auto s_xsxt = dlib::sum(dlib::pointwise_multiply(dlib::pointwise_multiply(centred_from_xvec, weights_vec), centred_to_xvec));
		auto s_ysyt = dlib::sum(dlib::pointwise_multiply(dlib::pointwise_multiply(centred_from_yvec, weights_vec), centred_to_yvec));
		auto s_xsyt = dlib::sum(dlib::pointwise_multiply(dlib::pointwise_multiply(centred_from_xvec, weights_vec), centred_to_yvec));
		auto s_ysxt = dlib::sum(dlib::pointwise_multiply(dlib::pointwise_multiply(centred_from_yvec, weights_vec), centred_to_xvec));

		auto v = dlib::sum(dlib::pointwise_multiply(weights_vec, dlib::pointwise_multiply(centred_from_xvec, centred_from_xvec) + dlib::pointwise_multiply(centred_from_yvec, centred_from_yvec)));

		auto a = (s_xsxt + s_ysyt) / v;
		auto b = (s_xsyt - s_ysxt) / v;
		auto norm = std::sqrt(a*a + b * b);
		a /= norm;
		b /= norm;
		auto tx = mean_to.x() - (a * mean_from.x()) + (b * mean_from.y());
		auto ty = mean_to.y() - (b * mean_from.x()) - (a * mean_from.y());

		dlib::matrix<double, 2, 2> m;
		dlib::matrix<double, 2, 1> B;
		m(0, 0) = a; m(0, 1) = -b;
		m(1, 0) = b; m(1, 1) = a;
		B(0) = tx;
		B(1) = ty;

		return point_transform_projective(join_cols(join_rows(m, B), matrix<double, 1, 3>({ 0,0,1 })));
	}

    /*!
        Find the translation transform which best aligns two sets of 2d points,
        returning the result as a (special case) projective transform
        
        - Requires
            - from_points.size() == to_points.size()
            - from_points.size() >= 2
        - Ensures
            - returns a point_transform_projective object, P, such that for all valid i:
              length(P(from_points[i]) - to_points[i])
              is minimized as often as possible using a translation transform.  
              That is, this function finds the transform that maps points in from_points 
              to points in to_points.  If no transform exists which performs this mapping 
              exactly then the one which minimizes the mean squared error is selected.
    */
    template <typename T>
    point_transform_projective find_translate_transform_as_projective(
        const std::vector<point2<T> >& from_points,
        const std::vector<point2<T> >& to_points
    )
	{
        // make sure requires clause is not broken
        DLIB_ASSERT(from_points.size() == to_points.size() &&
                    from_points.size() >= 1,
            "\t     find_translate_transform_as_projective(from_points, to_points)"
            << "\n\t Invalid inputs were given to this function."
            << "\n\t from_points.size(): " << from_points.size()
            << "\n\t to_points.size():   " << to_points.size()
            );

        auto mean_from = mean_point(from_points);
		auto mean_to = mean_point(to_points);
		auto diff = mean_to - mean_from;

        return point_transform_projective(dlib::matrix<double,3,3>({1,0,diff(0),0,1,diff(1),0,0,1}));
	}

	/*!
		Find the translation transform which best aligns two sets of 2d points with weights,
		returning the result as a (special case) projective transform

		- Requires
			- from_points.size() == to_points.size()
			- from_points.size() >= 2
			- weights.size() >= 2
		- Ensures
			- returns a point_transform_projective object, P, such that for all valid i:
			  weights[i]*length(P(from_points[i]) - to_points[i])
			  is minimized as often as possible using a translation transform.
			  That is, this function finds the transform that maps points in from_points
			  to points in to_points.  If no transform exists which performs this mapping
			  exactly then the one which minimizes the mean squared error is selected.
	*/
	template <typename T>
	point_transform_projective find_translate_transform_as_projective_weighted(
		const std::vector<point2<T> >& from_points,
		const std::vector<point2<T> >& to_points,
		const std::vector<T>& weights
	)
	{
		// make sure requires clause is not broken
		DLIB_ASSERT(weights.size() == from_points.size() &&
			from_points.size() == to_points.size() &&
			from_points.size() >= 1,
			"\t     find_translate_transform_as_projective_weights(from_points, to_points)"
			<< "\n\t Invalid inputs were given to this function."
			<< "\n\t from_points.size(): " << from_points.size()
			<< "\n\t to_points.size():   " << to_points.size()
			<< "\n\t weights.size():   " << to_points.size()
		);

		auto mean_from = weighted_mean_point(from_points, weights);
		auto mean_to = weighted_mean_point(to_points, weights);
		auto diff = mean_to - mean_from;

		return point_transform_projective(dlib::matrix<double, 3, 3>({ 1,0,diff(0),0,1,diff(1),0,0,1 }));
	}

    /*!
        Find the scale_translate transform which best aligns two sets of 2d points,
        returning the result as a (special case) projective transform
        
        - Requires
            - from_points.size() == to_points.size()
            - from_points.size() >= 2
        - Ensures
            - returns a point_transform_projective object, P, such that for all valid i:
              length(P(from_points[i]) - to_points[i])
              is minimized as often as possible using a scale_translate transform.  
              That is, this function finds the transform that maps points in from_points 
              to points in to_points.  If no transform exists which performs this mapping 
              exactly then the one which minimizes the mean squared error is selected.
    */
    template <typename T>
    point_transform_projective find_scale_translate_transform_as_projective(
        const std::vector<point2<T> >& from_points,
        const std::vector<point2<T> >& to_points
    )
	{
        // make sure requires clause is not broken
        DLIB_ASSERT(from_points.size() == to_points.size() &&
                    from_points.size() >= 2,
            "\t find_scale_translate_transform_as_projective(from_points, to_points)"
            << "\n\t Invalid inputs were given to this function."
            << "\n\t from_points.size(): " << from_points.size()
            << "\n\t to_points.size():   " << to_points.size()
            );

        dlib::vector<double, 2> mean_from, mean_to;

		for (unsigned long i = 0; i < from_points.size(); ++i)
		{
			mean_from += from_points[i];
			mean_to += to_points[i];
		}
		mean_from /= static_cast<double>(from_points.size());
		mean_to /= static_cast<double>(to_points.size());
	
		auto mag_to = 0.0;
		auto mag_from = 0.0;
		for (unsigned long i = 0; i < from_points.size(); ++i)
		{
			mag_to += (to_points[i] - mean_to).length();
			mag_from += (from_points[i] - mean_from).length();
		}
		matrix<double, 2, 2> sr;
		sr(0, 0) = mag_to / mag_from; sr(0, 1) = 0;
		sr(1, 0) = 0, sr(1, 1) = sr(0, 0);
		const dlib::vector<double, 2> t = mean_to - sr(0, 0)*mean_from;
        auto result = point_transform_projective(point_transform_affine(sr, t));
        return result;
	}

    /*
        Rapidly compute the inverse of a 3x3 matrix
    */
    template<typename T>
    dlib::matrix<T,3,3> fast_inverse(const dlib::matrix<T, 3, 3>& m)
    {
        dlib::matrix<T, 3, 3> minv;
        auto det = m(0, 0) * (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) -
            m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
            m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));

        auto invdet = 1 / det;

        minv(0, 0) = (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) * invdet;
        minv(0, 1) = (m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2)) * invdet;
        minv(0, 2) = (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1)) * invdet;
        minv(1, 0) = (m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2)) * invdet;
        minv(1, 1) = (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0)) * invdet;
        minv(1, 2) = (m(1, 0) * m(0, 2) - m(0, 0) * m(1, 2)) * invdet;
        minv(2, 0) = (m(1, 0) * m(2, 1) - m(2, 0) * m(1, 1)) * invdet;
        minv(2, 1) = (m(2, 0) * m(0, 1) - m(0, 0) * m(2, 1)) * invdet;
        minv(2, 2) = (m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1)) * invdet;

        return minv;
    }

    /*
        Returns a fast computation of the projective transform using four points
        
        - REQUIREMENTS ON T
		    - Must be either float, double, or long double, ie:
			  is_float_type<T>::value == true
        - Requires

        - Ensures
            - Returns a fast computation of the projective transform using exactly four points.
 
        - Throws
           - transform_error
               This exception is thrown if we are unable to compute the transform for some 
               reason (probably due to identical or colinear points being passed to the function)  
    */
    template<typename T>
    point_transform_projective find_fast_four_point_projective_xform(const quad<T>& from_points, const quad<T>& to_points)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);

        auto x1 = static_cast<double>(from_points[0].x());
        auto x2 = static_cast<double>(from_points[1].x());
        auto x3 = static_cast<double>(from_points[2].x());
        auto x4 = static_cast<double>(from_points[3].x());
        auto y1 = static_cast<double>(from_points[0].y());
        auto y2 = static_cast<double>(from_points[1].y());
        auto y3 = static_cast<double>(from_points[2].y());
        auto y4 = static_cast<double>(from_points[3].y());

        dlib::matrix<double, 3, 3> a;
        a = x1, x2, x3,
            y1, y2, y3,
            1, 1, 1;
        dlib::matrix<double, 3, 1> b;
        b = x4,
            y4,
            1;
        dlib::matrix<double, 3, 1> coeffs = fast_inverse(a)*b;
        dlib::matrix<double, 3, 3> A;
        A = coeffs(0)*x1, coeffs(1)*x2, coeffs(2)*x3,
            coeffs(0)*y1, coeffs(1)*y2, coeffs(2)*y3,
            coeffs(0)*1,  coeffs(1)*1,  coeffs(2)*1;

        x1 = static_cast<double>(to_points[0].x());
        x2 = static_cast<double>(to_points[1].x());
        x3 = static_cast<double>(to_points[2].x());
        x4 = static_cast<double>(to_points[3].x());
        y1 = static_cast<double>(to_points[0].y());
        y2 = static_cast<double>(to_points[1].y());
        y3 = static_cast<double>(to_points[2].y());
        y4 = static_cast<double>(to_points[3].y());
        a = x1, x2, x3,
            y1, y2, y3,
            1, 1, 1;
        b = x4,
            y4,
            1;
        coeffs = fast_inverse(a)*b;
        dlib::matrix<double, 3, 3> B;
        B = coeffs(0)*x1, coeffs(1)*x2, coeffs(2)*x3,
            coeffs(0)*y1, coeffs(1)*y2, coeffs(2)*y3,
            coeffs(0) * 1, coeffs(1) * 1, coeffs(2) * 1;

        dlib::matrix<double, 3, 3> C = B*fast_inverse(A);

        return point_transform_projective(C);
    }
    /*
        Returns a fast computation of the affine transform using three points
        
        - REQUIREMENTS ON T
		    - Must be either float, double, or long double, ie:
			  is_float_type<T>::value == true
        - Requires
            - source.size() == 3
            - target.size() == 3
        - Ensures
            - Returns a fast computation of the affine transform using exactly three points.
            
        - Throws
           - transform_error
               This exception is thrown if we are unable to compute the transform for some 
               reason (probably due to identical or colinear points being passed to the function)  
    */
    template<typename T>
    point_transform_affine find_fast_three_point_affine_xform(const std::vector<point2<T>>& from_points, const std::vector<point2<T>>& to_points)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);
        // make sure requires clause is not broken
        DLIB_ASSERT(from_points.size() == to_points.size() &&
            from_points.size() >= 2,
            "\t point_transform_affine compute_fast_three_point_affine_xform(const std::vector<dpoint>& source, const std::vector<dpoint>& target)"
            << "\n\t Invalid inputs were given to this function."
            << "\n\t from_points.size(): " << from_points.size()
            << "\n\t to_points.size():   " << to_points.size()
        );

        auto p = from_points[0].x();
        auto q = from_points[0].y();
        auto r = from_points[1].x();
        auto s = from_points[1].y();
        auto t = from_points[2].x();
        auto u = from_points[2].y();
        auto g = to_points[0].x();
        auto h = to_points[0].y();
        auto j = to_points[1].x();
        auto k = to_points[1].y();
        auto l = to_points[2].x();
        auto m = to_points[2].y();

        auto scaling = 1.0;
        try
        {
            scaling = 1.0 / (-q * r + p * s + q * t - s * t - p * u + r * u);
        }
        catch(...)
        {
            throw transform_error("compute_fast_three_point_affine_xform failed to compute affine transform\
                - divide by zero error, probably due to identical or colinear points being passed to the function.");
        }
       
        dlib::matrix<double, 2, 2> M;
        M(0, 0) = scaling * (l * (q - s) + g * (s - u) + j * (-q + u)); M(0, 1) = scaling * (l*(-p + r) + j * (p - t) + g * (-r + t));
        M(1, 0) = scaling * (m * (q - s) + h * (s - u) + k * (-q + u)); M(1, 1) = scaling * (m*(-p + r) + k * (p - t) + h * (-r + t));

        dlib::vector<double, 2> B;
        auto b1 = -q * r + p * s;
        auto b2 = q * t - p * u;
        auto b3 = -s * t + r * u;
        B(0) = scaling * (l*b1 + j * b2 + g * b3);
        B(1) = scaling * (m*b1 + k * b2 + h * b3);

        return point_transform_affine(M, B);
    }

    /*!
        Find the transform which best aligns two sets of 2d points
        using a given alignment method.
        
        - Requires
            - from_points.size() == to_points.size()
            - from_points.size() >= 0,1,2 or 3 depending on the alignment type,
            which can be: none, translation, rigid, projection, scale_translate or affine.
            - none simply returns the null transform and requires no points
            - translation needs 1 or more points
            - rigid, similarity, and scale_translate need 2 or more points
            - projective needs 4 or more points
            - All others need 3 or more points

        - Ensures
            - returns a point_transform_projective object, P, such that for all valid i:
            length(P(from_points[i]) - to_points[i])
            is minimized as often as possible.  That is, this function finds the
            transform that maps points in from_points to points in to_points.  If no
            transform exists which performs this mapping exactly then the one
            which minimizes the mean squared error is selected.
            - NOTE: the result is returned as a general projective transform, even though
            all cases except projective are special cases of the projective transform.
    */
    template <typename T>
    point_transform_projective find_transform_as_projective(
        const std::vector<point2<T>>& from_points,
        const std::vector<point2<T>>& to_points,
        const alignment_type align_method
    )
    {
        // make sure requires clause is not broken
        DLIB_ASSERT(from_points.size() == to_points.size() &&
            ((align_method == alignment_type::none && from_points.size() >= 0) ||
            (align_method == alignment_type::translation && from_points.size() >= 1) ||
                (align_method == alignment_type::rigid && from_points.size() >= 2) ||
                (align_method == alignment_type::similarity && from_points.size() >= 2) ||
                (align_method == alignment_type::scale_translate && from_points.size() >= 2) ||
                (align_method == alignment_type::projective && from_points.size() >= 4) ||
                (from_points.size() >= 3)),
            "\t point_transform_projective find_affine_transform_as_projective(from_points, to_points)"
            << "\n\t Invalid inputs were given to this function."
            << "\n\t from_points.size(): " << from_points.size()
            << "\n\t to_points.size():   " << to_points.size()
            << "\n\t align_method:   " << to_string(align_method)
        );

        point_transform_projective xform;

        switch (align_method)
        {
        case alignment_type::none:
            xform = point_transform_projective(dlib::matrix<double, 3, 3>({ 1,0,0,0,1,0,0,0,1 }));
            break;
        case alignment_type::affine:
            xform = find_affine_transform_as_projective(from_points, to_points);
            break;
        case alignment_type::similarity:
            xform = find_similarity_transform_as_projective(from_points, to_points);
            break;
        case alignment_type::scale_translate:
            xform = find_scale_translate_transform_as_projective(from_points, to_points);
            break;
        case alignment_type::projective:
            xform = find_projective_transform_as_projective(from_points, to_points);
            break;
        case alignment_type::rigid:
            xform = find_rigid_transform_as_projective(from_points, to_points);
            break;
        case alignment_type::translation:
            xform = find_translate_transform_as_projective(from_points, to_points);
            break;
        default:
            break;
        }
        return xform;
    }

	/*!
		Find the transform which best aligns two sets of 2d points
		using a given alignment method and a set of point weights.

		- Requires
			- from_points.size() == to_points.size()
			- weights.size() == from_points.size()
			- from_points.size() >= 0,1,2 or 3 depending on the alignment type,
			which can be: none, translation, rigid or similarity.
			- none simply returns the null transform and requires no points
			- translation needs 1 or more points
			- rigid, similarity, and scale_translate need 2 or more points
			- no other type of transform are allowed

		- Ensures
			- returns a point_transform_projective object, P, such that for all valid i:
			sum( weights[i] * length(P(from_points[i]) - to_points[i]))
			is minimized as often as possible.  That is, this function finds the
			transform that maps points in from_points to points in to_points, weighted
			by some factor for each point.  If no
			transform exists which performs this mapping exactly then the one
			which minimizes the mean squared error is selected.
			- NOTE: the result is returned as a general projective transform, even though
			all cases except projective are special cases of the projective transform.
	*/
	template <typename T>
	point_transform_projective find_transform_as_projective_weighted(
		const std::vector<point2<T>>& from_points,
		const std::vector<point2<T>>& to_points,
		const std::vector<T>& weights,
		const alignment_type align_method	
	)
	{
		// make sure requires clause is not broken
		DLIB_ASSERT(from_points.size() == weights.size() &&
			from_points.size() == to_points.size() &&
			((align_method == alignment_type::none && from_points.size() >= 0) ||
			(align_method == alignment_type::translation && from_points.size() >= 1) ||
				(align_method == alignment_type::rigid && from_points.size() >= 2) ||
				(align_method == alignment_type::similarity && from_points.size() >= 2)) ||
				(from_points.size() >= 3),
			"\t point_transform_projective find_affine_transform_as_projective(from_points, to_points)"
			<< "\n\t Invalid inputs were given to this function."
			<< "\n\t from_points.size(): " << from_points.size()
			<< "\n\t weights.size(): " << weights.size()
			<< "\n\t to_points.size():   " << to_points.size()
			<< "\n\t align_method:   " << to_string(align_method)
			);

			point_transform_projective xform;

			switch (align_method)
			{
			case alignment_type::none:
				xform = point_transform_projective(dlib::matrix<double, 3, 3>({ 1,0,0,0,1,0,0,0,1 }));
				break;
			case alignment_type::similarity:
				xform = find_similarity_transform_as_projective_weighted(from_points, to_points, weights);
				break;
			case alignment_type::rigid:
				xform = find_rigid_transform_as_projective_weighted(from_points, to_points, weights);
				break;
			case alignment_type::translation:
				xform = find_translate_transform_as_projective_weighted(from_points, to_points, weights);
				break;
			case alignment_type::scale_translate:
				xform = find_scale_translate_transform_as_projective_weighted(from_points, to_points, weights);
                break;
			default:
				break;
			}
			return xform;
	}

	/*!
			Get a fast approximation to scale and angle of a set of points compared to the base shape
			- REQUIREMENTS ON T
				- Must be either float, double, or long double, ie:
				  is_float_type<T>::value == true
			- Requires
				- This can only be used if align_method() ==  none, translation, rigid, similarity or scale_translate
			- Ensures
				- Returns a pair of doubles where the first item is a fast approximation to scale and the second item
				  is a fast approximation to angle.

	*/
	template <typename T>
	std::pair<double, double> fast_approx_scale_and_angle(
		const std::vector<point2<T> >& from_points,
		const std::vector<point2<T> >& to_points,
		alignment_type align_method)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);
		DLIB_CASSERT(align_method == alignment_type::none ||
			align_method == alignment_type::translation ||
			align_method == alignment_type::rigid ||
			align_method == alignment_type::similarity ||
			align_method == alignment_type::scale_translate
		);

		auto xform = find_transform_as_projective(from_points, to_points, align_method);
		double scale = std::sqrt(xform.get_m()(0, 0)*xform.get_m()(0, 0) + xform.get_m()(0, 1)*xform.get_m()(0, 1));
		double angle = atan2(xform.get_m()(1, 0) / scale, xform.get_m()(0, 0) / scale);
		return std::make_pair(scale, angle);
	}

}

