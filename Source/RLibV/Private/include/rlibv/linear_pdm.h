// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "basic_types.h"
#include "pca_model.h"

#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/geometry.h>
RLIBV_RENABLE_WARNINGS

namespace rlibv 
{
    /**
    * \defgroup Learning Machine learning
    * @{
    */

    using dlib::serialize;
    using dlib::deserialize;

    /**
    * @brief A class representing a Linear Point Distribution Model
    * @tparam T T must be either float or double
    */
    template <typename T>
    class linear_pdm 
    {
    public:
        /**
         * @brief Reconstructs a shape based on parameters, pose and residuals.
         * @details Populates 'shape' and 'pose' based on parameters, pose and residuals.
         *          The magnitude of the residual part of the reconstruction is capped per-point
         *          (in the model frame, before pose transformation) according to the residual limits
         *          given in 'residual_limit_in_model_frame'
		 *          The parameters vector can be shorter than the number of parameters in the model, in which case it is
		 *          assumed that the parameters represent the coefficients for the corresponding largest eigenvectors.
         * @param parameters 
         * @param pose Projective transformation
         * @param residuals_in_aligned_frame 
         * @param residual_limit_in_model_frame
         * @param shape
         * @pre parameters.nr() <= n_params();
         * @pre residuals_in_aligned_frame.size() == n_points();
         * @pre residual_limit_in_model_frame.size() == n_points();
        */
        void reconstruct(
            const dlib::matrix<T, 0, 1>& parameters,
            const dlib::point_transform_projective& pose,
            const shape2d<T>& residuals_in_aligned_frame,
            const std::vector<T>& residual_limit_in_model_frame,
            shape2d<T>& shape) const;

        /**
         * @brief Finds the best set of parameters and pose for the given points.
         * @details Populates 'parameters' and 'pose' with the best estimates of these values for the given shape.
         *          The parameterization can be clamped so that the values of each parameter do not exceed the
         *          corresponding maximum and minimum values for the parameter as seen when parameterizing the
         *          training examples. These ranges have been determined during
         *          training by one of the training functions. In calling this function, param_limit_scaling allows
         *          you to stretch or shrink these ranges by the given factor. A value of 1.0 will clamp the output
         *          such that each parameter never exceeds the training range. A value of 0.9 would clamp the output
         *          10 percent more aggressively. If you pass a negative value for param_limit_scaling, no clamping is applied.
         *          The default value for param_limit_scaling is -1.
         *          Populates 'residuals_in_aligned_frame' with the difference between the true points and the modeled
         *          points in the aligned frame of reference.
         * @param shape
         * @param parameters
         * @param pose Projective transformation
         * @param residuals_in_aligned_frame
         * @param param_limit_scaling
         * @param n_modes the maximum number of parameters to return
         * @pre shape.size() == n_points()
         */
        void parameterize(
            const shape2d<T>& shape,
            dlib::matrix<T, 0, 1>& parameters, 
            dlib::point_transform_projective& pose,
            shape2d<T>& residuals_in_aligned_frame,
            T param_limit_scaling,
            int n_modes) const;

        /**
         * @brief Finds the best set of parameters and pose for the given points but with some marked as missing
         * @details Populates 'parameters' and 'pose' with the best estimates of these values for the given shape.
         *          The parameterization can be clamped so that the values of each parameter do not exceed the
         *          corresponding maximum and minimum values for the parameter as seen when parameterizing the
         *          training examples. These ranges have been determined during
         *          training by one of the training functions. In calling this function, param_limit_scaling allows
         *          you to stretch or shrink these ranges by the given factor. A value of 1.0 will clamp the output
         *          such that each parameter never exceeds the training range. A value of 0.9 would clamp the output
         *          10 percent more aggressively. If you pass a negative value for param_limit_scaling, no clamping is applied.
         *          The default value for param_limit_scaling is -1.
         *          Populates 'residuals_in_aligned_frame' with the difference between the true points and the modeled
         *          points in the aligned frame of reference.
         *          The key difference from the standard "parameterize" function is that this function also takes
         *          a vector of integers indicating which points are 'good'. An iterative method then attempts to 
         *          incrementally ignore the bad points (i.e. any not in the list).
         * @note This function only works iteratively on models with a full set of alignment points (i.e. every point) in
         *       a shape is used for alignment. If this isn't true then the basic 'parameterize' function is called.
         * @param shape
         * @param good_point_indices vector of indices of those points considered 'good'
         * @param parameters
         * @param pose Projective transformation
         * @param residuals_in_aligned_frame
         * @param param_limit_scaling
         * @param n_modes the maximum number of parameters to return
         * @param max_iterations the maximum number of iterations to use (10 is usually convergent but you might want a lower
                  number in speed-critical applications)
         * @pre shape.size() == n_points()
         * @pre good_point_indices.size() > 1
         */
		void parameterize_with_missing_points(
			const shape2d<T>& shape,
            const std::vector<int>& good_point_indices,
			dlib::matrix<T, 0, 1>& parameters,
			dlib::point_transform_projective& pose,
			shape2d<T>& residuals_in_aligned_frame,
			T param_limit_scaling,
			int n_modes,
            int max_iterations) const;

        /**
         * @brief Finds the best set of parameters and pose for the given points but with some marked as missing
         * @details Populates 'parameters' and 'pose' with the best estimates of these values for the given shape.
         *          The parameterization can be clamped so that the values of each parameter do not exceed the
         *          corresponding maximum and minimum values for the parameter as seen when parameterizing the
         *          training examples. These ranges have been determined during
         *          training by one of the training functions. In calling this function, param_limit_scaling allows
         *          you to stretch or shrink these ranges by the given factor. A value of 1.0 will clamp the output
         *          such that each parameter never exceeds the training range. A value of 0.9 would clamp the output
         *          10 percent more aggressively. If you pass a negative value for param_limit_scaling, no clamping is applied.
         *          The default value for param_limit_scaling is -1.
         *          Populates 'residuals_in_aligned_frame' with the difference between the true points and the modeled
         *          points in the aligned frame of reference.
         *          The key difference from the standard "parameterize" function is that this function also takes
         *          a vector of doubles indicating the confidence in each point. An iterative method then attempts to
         *          incrementally fit based on this confidence (which should always be between 0.0 and 1.0 where
         *          1.0 indicates perfect confidence.
         * @note This function only works iteratively on models with a full set of alignment points (i.e. every point) in
         *       a shape is used for alignment. If this isn't true then the basic 'parameterize' function is called.
         * @param shape
         * @param confidence vector (each element should have a value between 0.0 and 1.0)
         * @param parameters
         * @param pose Projective transformation
         * @param residuals_in_aligned_frame
         * @param param_limit_scaling
         * @param n_modes the maximum number of parameters to return
         * @param max_iterations the maximum number of iterations to use (10 is usually convergent but you might want a lower
                  number in speed-critical applications)
         * @pre shape.size() == n_points()
         * @pre confidence.size() == n_points()
         */
        void parameterize_with_confidence(
            const shape2d<T>& shape,
            const std::vector<double>& confidence,
            dlib::matrix<T, 0, 1>& parameters,
            dlib::point_transform_projective& pose,
            shape2d<T>& residuals_in_aligned_frame,
            T param_limit_scaling,
            int n_modes,
            int max_iterations) const;


        /**
        * @brief Finds the best set pose parameters for the given points.
        * @param shape
        * @pre shape.size() == n_points()
        * @returns pose Projective transformation
       
        */
        dlib::point_transform_projective find_pose(const shape2d<T>& shape) const;

        /**
         * @brief Trains a linear PDM up to a maximum number of modes using randomized SVD pca training
         * @tparam T must be either float or double
         * @param shapes
         * @param variance variance to retain
         * @param max_modes
         * @param align_type
         * @param align_indices
         * @pre shapes.size() > 0
         * @pre shapes[0].size() >= 0,1,2 or 3 depending on the alignment type
         * @pre align_indices.size() == 0 or align_indices.size() >= 0,1,2 or 3 depending on the alignment type
         * @remark Alignment type can be: none, translation, rigid, similarity, projection or scale_translate.
         *         - none simply returns the null transform and requires no points
         *         - translation needs 1 or more points
         *         - rigid, similarity, and scale_translate need 2 or more points
         *         - projective needs 4 or more points
         *         - All others need 3 or more points
         * @pre all_vectors_are_same_size(shapes) == True
         * @pre max_modes >= 0
         * @pre is_unique(align_indices) == true
         * @pre every element of align_indices < shapes[0].size()
         */
        void train_linear_pdm(
            const std::vector<shape2d<T>>& shapes,
            T variance,
            int max_modes,
            alignment_type align_type,
            std::vector<int> align_indices);

        /**
        * @brief Get the number of points in the PDM
        * @returns Number of points in the PDM.
        */
        int n_points() const
        {
            return static_cast<int>(base_shape_.size());
        }

        /**
         * @brief The number of parameters in the model
         * @return Number of parameters in the model
         */
        int n_params() const
        {
            return linear_pca_.n_params();
        }

        /**
         * @brief Get the alignment method
         * @returns The alignment_type of the PDM.
        */
        alignment_type align_method() const
        {
            return align_method_;
        }

        /**
         * @brief Get the base shape
         * @returns The base shape of the PDM.
        */
        shape2d<T> base_shape() const
        {
            return base_shape_;
        }

        /**
         * @brief Is the model 'null' (i.e. has zero shape modes)
         * @returns True iff the model is null.
        */
        bool is_null() const
        {
            return is_null_;
        }

        /**
         * @brief Get the pca model
         * @returns Constant reference to the pca model
        */
        const rlibv::pca_model<T> pca_model() const
        {
            return linear_pca_;
        }

        /**
         * @brief Serialization
         * @tparam U must be either float or double
         * @param item The Linear PDM
         * @param out The output stream
         */
        template<typename U>
        friend void serialize(const linear_pdm<U>& item, std::ostream& out);

        /**
         * @brief Deserialization
         * @tparam U must be either float or double
         * @param item The resulting Linear PDM
         * @param in The input stream
         */
        template<typename U>
        friend void deserialize(linear_pdm<U>& item, std::istream& in);

    private:
        bool is_trained_ = false;
        bool is_null_ = true;
        alignment_type align_method_ = alignment_type::similarity;
        rlibv::pca_model<T> linear_pca_;
        shape2d<T> base_shape_;
        std::vector<int> align_indices_;
    };

	template<typename U>
	void serialize(const linear_pdm<U>& item, std::ostream& out);


	template<typename U>
	void deserialize(linear_pdm<U>& item, std::istream& in);

    /**@}*/
}

#include "impl/linear_pdm.hpp"