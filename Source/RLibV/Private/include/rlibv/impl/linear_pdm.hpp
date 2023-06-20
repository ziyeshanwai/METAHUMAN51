// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "../include/rlibv/transforms.h"
#include "../include/rlibv/geometry.h"
#include "../include/rlibv/data_utils.h"

#include <numeric>

namespace rlibv
{
    template <typename T>
    void linear_pdm<T>::reconstruct(
        const dlib::matrix<T, 0, 1>& parameters,
        const dlib::point_transform_projective& pose,
        const shape2d<T>& residuals_in_aligned_frame,
        const std::vector<T>& residual_limit_in_model_frame,
        shape2d<T>& shape) const
    {

        DLIB_ASSERT(parameters.nr() <= n_params());
        DLIB_ASSERT(residuals_in_aligned_frame.size() == n_points());
        DLIB_ASSERT(residual_limit_in_model_frame.size() == n_points());

        shape2d<T> result_in_model_space;
        if (is_null_)
        {
            result_in_model_space = base_shape_;
        }
        else
        {
            auto pts_data_vector = linear_pca_.reconstruct(parameters);
            result_in_model_space = col_matrix_to_shape2d(pts_data_vector);
        }

        if (!residual_limit_in_model_frame.empty())
        {
            auto index = 0;
            for (auto item : residuals_in_aligned_frame)
            {
                auto length = static_cast<T>(item.length());
                T multiplier = 1;
                if (length > residual_limit_in_model_frame[index] && length > 0.000000001f)
                {
                    multiplier = residual_limit_in_model_frame[index] / length;
                }
                result_in_model_space[index] += multiplier * item;
                index++;
            }
        }
        shape = transform_shape(result_in_model_space, pose);
    }

	template <typename T>
	void rlibv::linear_pdm<T>::parameterize_with_confidence(const shape2d<T>& shape, const std::vector<double>& confidence, dlib::matrix<T, 0, 1>& parameters, dlib::point_transform_projective& pose, shape2d<T>& residuals_in_aligned_frame, T param_limit_scaling, int n_modes, int max_iterations) const
	{
		DLIB_ASSERT(shape.size() == n_points());
		DLIB_ASSERT(confidence.size() == n_points());

		if (align_indices_.size() != base_shape_.size())
		{
			parameterize(shape, parameters, pose, residuals_in_aligned_frame, param_limit_scaling, n_modes);
			return;
		}

		std::vector<T> res_limits(shape.size(), 0.0);

		auto recon_shape(shape);
		parameterize(recon_shape, parameters, pose, residuals_in_aligned_frame, param_limit_scaling, n_modes);
		for (auto i = 0; i < max_iterations; ++i)
		{
			reconstruct(parameters, pose, residuals_in_aligned_frame, res_limits, recon_shape);
			for (int p=0; p<shape.size(); ++p)
			{
                double conf = std::max(0.0,std::min(1.0, confidence[p]));
                recon_shape[p] = conf * shape[p] + (1.0 - conf) * recon_shape[p];
			}
			parameterize(recon_shape, parameters, pose, residuals_in_aligned_frame, param_limit_scaling, n_modes);
		}
	}


	template <typename T>
    void rlibv::linear_pdm<T>::parameterize_with_missing_points(const shape2d<T>& shape, const std::vector<int>& good_point_indices, dlib::matrix<T, 0, 1>& parameters, dlib::point_transform_projective& pose, shape2d<T>& residuals_in_aligned_frame, T param_limit_scaling, int n_modes, int max_iterations) const
    {
        DLIB_ASSERT(shape.size() == n_points());
        DLIB_ASSERT(good_point_indices.size() > 1);

        if (align_indices_.size() != base_shape_.size())
        {
            parameterize(shape, parameters, pose, residuals_in_aligned_frame, param_limit_scaling, n_modes);
            return;
        }

        std::vector<T> res_limits(shape.size(), 0.0);

        auto recon_shape(shape);
		parameterize(recon_shape, parameters, pose, residuals_in_aligned_frame, param_limit_scaling, n_modes);
        for (auto i = 0; i < max_iterations; ++i)
        {
            reconstruct(parameters, pose, residuals_in_aligned_frame, res_limits, recon_shape);
            for (int p : good_point_indices)
            {
                recon_shape[p] = shape[p];
            }
			parameterize(recon_shape, parameters, pose, residuals_in_aligned_frame, param_limit_scaling, n_modes);
        }
	}


    template <typename T>
    void linear_pdm<T>::parameterize(
        const shape2d<T>& shape,
        dlib::matrix<T, 0, 1>& parameters,
        dlib::point_transform_projective& pose,
        shape2d<T>& residuals_in_aligned_frame,
        T param_limit_scaling,
        int n_modes) const
    {

        DLIB_ASSERT(shape.size() == n_points());

        point2d<T> zero_pt(0, 0);
        residuals_in_aligned_frame = std::vector<point2d<T>>(shape.size(), zero_pt);
        std::vector<T> residual_limit_in_model_frame(shape.size(), 0);

        parameters.set_size(std::min(n_params(), n_modes));
        set_all_elements(parameters, 0);
        shape2d<T> model_points;
        const auto n_iterations =
            ((align_method_ == alignment_type::none) ||
                (align_method_ == alignment_type::translation) ||
                 is_null_) ? 1 : 4;

        for (auto i = 0; i < n_iterations; ++i)
        {
            pose = dlib::point_transform_projective();
            {
                reconstruct(parameters, pose, residuals_in_aligned_frame, residual_limit_in_model_frame, model_points);
            }
            pose = find_transform_as_projective(
                subvector(model_points, align_indices_),
                subvector(shape, align_indices_),
                align_method_
            );

            if (!is_null_)
            {
                auto target_points_in_model_frame = transform_shape(shape, inv(pose));
                parameters = linear_pca_.parameterize(
                    shape2d_to_col_matrix(target_points_in_model_frame),
                    param_limit_scaling,
                    n_modes);
            }
        }

        pose = find_transform_as_projective(
            subvector(model_points, align_indices_),
            subvector(shape, align_indices_),
            align_method_
        );

        reconstruct(parameters, dlib::point_transform_projective(), residuals_in_aligned_frame, residual_limit_in_model_frame, model_points);
        auto target_points_in_model_frame = transform_shape(shape, inv(pose));
        residuals_in_aligned_frame.resize(model_points.size());
        auto index = 0;
        for (auto item : model_points)
        {
            residuals_in_aligned_frame[index] = target_points_in_model_frame[index] - item;
            index++;
        }
    }

    template <typename T>
    dlib::point_transform_projective linear_pdm<T>::find_pose(const shape2d<T>& shape) const
    {
		DLIB_ASSERT(shape.size() == n_points());

        dlib::matrix<T, 0, 1> parameters;
        dlib::point_transform_projective pose;
        shape2d<T> residuals_in_aligned_frame;
        T param_limit_scaling = 999999;
        parameterize(shape, parameters, pose, residuals_in_aligned_frame, param_limit_scaling, n_params());
        return pose;
    }

    template<typename T>
    void linear_pdm<T>::train_linear_pdm(
        const std::vector<shape2d<T>>& shapes,
        T variance,
        int max_modes,
        alignment_type align_type,
        std::vector<int> align_indices)
    {
        DLIB_ASSERT(shapes.size() > 0);
        DLIB_ASSERT((align_type == alignment_type::none && shapes[0].size() >= 0) ||
            (align_type == alignment_type::translation && shapes[0].size() >= 1) ||
            (align_type == alignment_type::rigid && shapes[0].size() >= 2) ||
            (align_type == alignment_type::similarity && shapes[0].size() >= 2) ||
            (align_type == alignment_type::scale_translate && shapes[0].size() >= 2) ||
            (align_type == alignment_type::projective && shapes[0].size() >= 4));
        DLIB_ASSERT((align_type == alignment_type::none && align_indices.size() >= 0) ||
            (align_type == alignment_type::translation && align_indices.size() >= 1) ||
            (align_type == alignment_type::rigid && align_indices.size() >= 2) ||
            (align_type == alignment_type::similarity && align_indices.size() >= 2) ||
            (align_type == alignment_type::scale_translate && align_indices.size() >= 2) ||
            (align_type == alignment_type::projective && align_indices.size() >= 4) ||
            (align_indices.size() == 0));
  
        DLIB_ASSERT(max_modes > 0);
        DLIB_ASSERT(all_shapes_are_same_size(shapes));
        DLIB_ASSERT(is_unique(align_indices));
        DLIB_ASSERT(all_items_less_than(align_indices, static_cast<int>(shapes[0].size())));

        align_method_ = align_type;
        if (align_indices.size() == 0)
        {
            align_indices_.resize(shapes[0].size());
            std::iota(align_indices_.begin(), align_indices_.end(), 0);
        }
        else
        {
            align_indices_ = std::move(align_indices);
        }

        std::vector<shape2d<T>> alignment_shapes;
        for (const auto& full_shape : shapes)
        {
            alignment_shapes.push_back(subvector(full_shape, align_indices_));
        }
        base_shape_ = compute_procrustes_base_shape(alignment_shapes, align_type);

        std::vector<dlib::matrix<T, 0, 1>> shapes_as_col_matrices;
        for (const auto& full_shape : shapes)
        {
            auto xform = find_transform_as_projective(
                subvector(full_shape, align_indices_),
                base_shape_,
                align_type);
            shapes_as_col_matrices.emplace_back(shape2d_to_col_matrix(transform_shape(full_shape, xform)));
        }
            
        if (max_modes > 0 && shapes.size()>1)
        {
            linear_pca_.fast_train_to_variance(
                shapes_as_col_matrices,
                variance,
                max_modes);
            if (linear_pca_.n_params() > 0)
            {
                is_null_ = false;
            }
        }

        is_trained_ = true;
    }


    template<typename U>
    void serialize(const linear_pdm<U>& item, std::ostream& out)
    {
        serialize(item.is_trained_, out);
        serialize(item.is_null_, out);
        serialize(item.align_method_, out);
        serialize(item.linear_pca_, out);
        serialize(item.base_shape_, out);
        serialize(item.align_indices_, out);
    }

    template<typename U>
    void deserialize(linear_pdm<U>& item, std::istream& in)
    {
        deserialize(item.is_trained_, in);
        deserialize(item.is_null_, in);
        deserialize(item.align_method_, in);
        deserialize(item.linear_pca_, in);
        deserialize(item.base_shape_, in);
        deserialize(item.align_indices_, in);
    }

}