// Copyright Epic Games, Inc. All Rights Reserved.


#pragma once

#include "../include/rlibv/geometry.h"
#include "../include/rlibv/data_utils.h"

namespace rlibv
{
    template <typename T>
    shape2d<T> transform_shape(const shape2d<T>& points, const dlib::point_transform_projective& xform)
    {
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

        DLIB_ASSERT(points.size() >= 1);

        shape2d<T> result(points.size());
        auto count = 0;
        for (auto& pt : points)
        {
            result[count++] = xform(pt);
        }
        return result;
    }

    std::vector<dlib::drectangle> transform_rects(const std::vector<dlib::drectangle>& rects, const dlib::point_transform_projective& xform)
	{

        std::vector<dlib::drectangle> result(rects.size());
		auto count = 0;
		for (auto& rect : rects)
		{         
            result[count] = dlib::drectangle(xform(rect.tl_corner()), xform(rect.tl_corner()));
            ++count;
		}
		return result;
	}


	
	template<typename T>
	point2d<T> weighted_mean_point(const shape2d<T>& shape, const std::vector<T>& weights)
	{
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

		DLIB_ASSERT(shape.size() == weights.size());

		point2d<T> mean_pt{ 0,0 };
		T total_weight = 0;
		for (auto i = 0; i < shape.size(); ++i)
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
			mean_pt = point2d<T>(0, 0);
		}
		return mean_pt;
	}

    template <typename T>
    dlib::point_transform_projective find_projective_transform_as_projective(
        const shape2d<T>& from_points, const shape2d<T>& to_points)
    {
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");

        DLIB_ASSERT(from_points.size() == to_points.size());
        DLIB_ASSERT(from_points.size() >= 4);

        return dlib::find_projective_transform(cast_points_to_double(from_points), cast_points_to_double(to_points));
    }

    template <typename T>
    dlib::point_transform_projective find_similarity_transform_as_projective(
        const shape2d<T>& from_points, const shape2d<T>& to_points)
    {
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
        DLIB_ASSERT(from_points.size() == to_points.size());
        DLIB_ASSERT(from_points.size() >= 2);

        auto mean_from = mean_point(from_points);
        auto mean_to = mean_point(to_points);

        long n_points = static_cast<long>(from_points.size());
        dlib::matrix<T, 1, 0> centred_from_xvec(n_points);
        dlib::matrix<T, 1, 0> centred_from_yvec(n_points);
        dlib::matrix<T, 1, 0> centred_to_xvec(n_points);
        dlib::matrix<T, 1, 0> centred_to_yvec(n_points);
        for (int i = 0; i < static_cast<int>(from_points.size()); ++i)
        {
            centred_from_xvec(i) = from_points[i].x() - mean_from.x();
            centred_from_yvec(i) = from_points[i].y() - mean_from.y();
        }
        for (int i = 0; i < static_cast<int>(to_points.size()); ++i)
        {
            centred_to_xvec(i) = to_points[i].x() - mean_to.x();
            centred_to_yvec(i) = to_points[i].y() - mean_to.y();
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

        return dlib::point_transform_projective(join_cols(join_rows(m, B), dlib::matrix<double, 1, 3>({ 0,0,1 })));
    }

    template <typename T>
    dlib::point_transform_projective find_rigid_transform_as_projective(
        const std::vector<point2d<T> >& from_points, const std::vector<point2d<T> >& to_points)
    {
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
        DLIB_ASSERT(from_points.size() == to_points.size());
        DLIB_ASSERT(from_points.size() >= 2);

        dlib::vector<double, 2> mean_from, mean_to;
        dlib::matrix<double, 2, 2> cov({ 0,0,0,0 });

        for (unsigned long i = 0; i < from_points.size(); ++i)
        {
            mean_from += from_points[i];
            mean_to += to_points[i];
        }
        mean_from /= static_cast<double>(from_points.size());
        mean_to /= static_cast<double>(from_points.size());

        for (unsigned long i = 0; i < from_points.size(); ++i)
        {
            cov += (to_points[i] - mean_to) * trans(from_points[i] - mean_from);
        }

        cov /= static_cast<double>(from_points.size());

        dlib::matrix<double, 2, 2> u, v, d;
        svd(cov, u, d, v);
        dlib::matrix<double, 2, 2> s = identity_matrix(cov);
        if (det(cov) < 0 || (det(cov) == 0 && det(u) * det(v) < 0))
        {
            if (d(1, 1) < d(0, 0))
                s(1, 1) = -1;
            else
                s(0, 0) = -1;
        }

        dlib::matrix<double, 2, 2> r = u * s * trans(v);
        dlib::vector<double, 2> t = mean_to - r * mean_from;

        return dlib::point_transform_projective(join_cols(join_rows(r, t), dlib::matrix<double, 1, 3>({ 0,0,1 })));
    }

    template <typename T>
    dlib::point_transform_projective find_translate_transform_as_projective(
        const std::vector<point2d<T> >& from_points, const std::vector<point2d<T> >& to_points)
    {
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
        DLIB_ASSERT(from_points.size() == to_points.size());
        DLIB_ASSERT(from_points.size() >= 1);

        auto mean_from = mean_point(from_points);
        auto mean_to = mean_point(to_points);
        auto diff = mean_to - mean_from;

        return dlib::point_transform_projective(dlib::matrix<double, 3, 3>({ 1,0,diff(0),0,1,diff(1),0,0,1 }));
    }

    template <typename T>
    dlib::point_transform_projective find_scale_translate_transform_as_projective(
        const std::vector<point2d<T> >& from_points, const std::vector<point2d<T> >& to_points)
    {
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
        DLIB_ASSERT(from_points.size() == to_points.size());
        DLIB_ASSERT(from_points.size() >= 2);

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
        dlib::matrix<double, 2, 2> sr;
        sr(0, 0) = mag_to / mag_from; sr(0, 1) = 0;
        sr(1, 0) = 0, sr(1, 1) = sr(0, 0);
        const dlib::vector<double, 2> t = mean_to - sr(0, 0) * mean_from;
        auto result = dlib::point_transform_projective(dlib::point_transform_affine(sr, t));
        return result;
    }

    template <typename T>
    dlib::point_transform_projective find_transform_as_projective(
        const std::vector<point2d<T>>& from_points, const std::vector<point2d<T>>& to_points, const alignment_type align_method)
    {
		static_assert(
			std::is_same<T, float>::value ||
			std::is_same<T, double>::value,
			"T must be a float or double type");
        DLIB_ASSERT(from_points.size() == to_points.size());
        DLIB_ASSERT((align_method == alignment_type::none && from_points.size() >= 0) ||
            (align_method == alignment_type::translation && from_points.size() >= 1) ||
            (align_method == alignment_type::rigid && from_points.size() >= 2) ||
            (align_method == alignment_type::similarity && from_points.size() >= 2) ||
            (align_method == alignment_type::scale_translate && from_points.size() >= 2) ||
            (align_method == alignment_type::projective && from_points.size() >= 4) ||
            (from_points.size() >= 3));

        dlib::point_transform_projective xform;

        switch (align_method)
        {
        case alignment_type::none:
            xform = dlib::point_transform_projective(dlib::matrix<double, 3, 3>({ 1,0,0,0,1,0,0,0,1 }));
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

    template <typename T>
    std::array<T, 4> approx_similarity_transform(const dlib::point_transform_projective& xform)
	{
		std::array<T, 4> result;
		result[0] = static_cast<T>(std::sqrt(xform.get_m()(0, 0) * xform.get_m()(0, 0) + xform.get_m()(0, 1) * xform.get_m()(0, 1)));
		result[1] = static_cast<T>(atan2(xform.get_m()(1, 0) / result[0], xform.get_m()(0, 0) / result[0]));
        result[2] = static_cast<T>(xform.get_m()(0, 2));
		result[3] = static_cast<T>(xform.get_m()(1, 2));
        return result;
	}

    template <typename T>
    dlib::point_transform_projective similarity_params_as_projective(const std::array<T, 4>& params)
    {
		dlib::matrix<double, 2, 2> m;
		dlib::matrix<double, 2, 1> B;
		m(0, 0) = params[0]*std::cos(params[1]); m(0, 1) = -params[0] * std::sin(params[1]);
		m(1, 0) = params[0] * std::sin(params[1]); m(1, 1) = params[0] * std::cos(params[1]);
		B(0) = params[2];
		B(1) = params[3];
        return dlib::point_transform_projective(join_cols(join_rows(m, B), dlib::matrix<double, 1, 3>({ 0,0,1 })));
    }

}