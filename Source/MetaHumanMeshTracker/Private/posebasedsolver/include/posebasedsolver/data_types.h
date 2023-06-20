// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once 

#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
POSEBASEDSOLVER_RENABLE_WARNINGS
#include "enum_ext.h"
#include "serialization_utils.h"

namespace cm
{
	//! typedef for a column vector of type float, double, or long double
	template<typename T, typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>
	using col_vector = dlib::matrix<T, 0, 1>;

	//! typedef for a row vector of type float, double, or long double
	template<typename T, typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>
	using row_vector = dlib::matrix<T, 1, 0>;

	//! typedef for a 2d point of type float, double, or long double
	template<typename T, typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>
	using point2 = dlib::vector<T, 2>;

	//! typedef for a 3d point of type float, double, or long double
	template<typename T, typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>
	using point3 = dlib::vector<T, 3>;

	//! vector of point2
	template<typename T>
	using point2_vector = std::vector<point2<T>>;

	//! vectorof point3
	template<typename T>
	using point3_vector = std::vector<point3<T>>;

	//! map of point2
	template<typename T>
	using point2_map = std::map<std::string, point2<T>>;

	//! map of point3
	template<typename T>
	using point3_map = std::map<std::string, point3<T>>;

	//! quadrilateral
	template<typename T>
	using quad = std::array<point2<T>, 4>;

	/* One sampling line */
	struct sampling_line
	{
		point2<double> start_point;
		point2<double> end_point;
	};

	/* A vector of sampling lines */
	struct sampling_star
	{
		std::vector<sampling_line> star_lines;
	};

	/* A vector of sampling stars */
	struct multi_scale_sampling_star
	{
		std::vector<sampling_star> multi_scale_star_lines;
	};

	// Defines the indices of a curve and whether or not it's a closed curve.
	struct curve_definition
	{
		std::vector<int> indices;
		bool closed = false;
	};

	inline void serialize(const curve_definition& item, std::ostream& out)
	{
		dlib::serialize(item.indices, out);
		dlib::serialize(item.closed, out);
	}
	inline void deserialize(curve_definition& item, std::istream& in)
	{
		dlib::deserialize(item.indices, in);
		dlib::deserialize(item.closed, in);
	}

   

	//! Alignment types
	DECLARE_ENUM(alignment_type, none, translation, rigid, similarity, affine, projective, scale_translate)

	//! Alignment 3d types
	DECLARE_ENUM(alignment3d_type, none, translation, rigid, similarity)

    //! Shape difference types
    DECLARE_ENUM(shape_diff_type, pointwise_l2, pointwise_l1)

    //! Sampling types
    DECLARE_ENUM(sample_method, grey, colour, fhog)

    //! Feature reduction methods
    DECLARE_ENUM(reduction_method, cubic_merit_greedy, pearson, spearman, quadfit, linfit, none)

    //! Animation control types
    DECLARE_ENUM(ctrl_type, normal, sparse, ultra)

	//! Prediction model methods
	DECLARE_ENUM(prediction_model_method, pca_krr_linear, pca_krr_rbf, krr_linear, krr_rbf, pca_svr_rbf, simple_random_forest, pca_krr_rbf_16, svr_linear, pca_krls, pca_rr, rr)

	//! Classifier model methods
	DECLARE_ENUM(classifier_model_method, krr_rbf, svr_rbf)

    //! Sample processing methods
    DECLARE_ENUM(sample_processing, none, normalize_global, local_absdiffmean, local_normalize, local_rank, white_to_black)

	//! Profile processing methods
	DECLARE_ENUM(profile_processing, 
	        order,
			loc_max_abs_gradient,
			loc_min_max_gradient,
			none,
			cross_diffs,
			gradient,
			gradient_magnitude,
			moments,
			mean_diff_moments,
		    binary_mean_diff_moments,
		    binary_grad_moments,
		    gm_moments,
			grad_moments,
			legendre_moments,
			gm_legendre_moments,
			grad_legendre_moments,
			squared,
			fft,
			fft_mag,
			fft_phase,
                // TODO the two values below are deprecated and should be removed once all configs have been updated
            normalized_moments,
            gm_normalized_moments,
			poly2,
			grad_poly2,
			grad_mag_poly2,
			poly3)

	//! Normlization methods
	DECLARE_ENUM(normalization,
	    none,
		zero_mean,
		zero_mean_unit_std,
		zero_to_one)

	//! Profile classifier methods
	DECLARE_ENUM(profile_search_method,krr_pca_rbf_regression,svr_pca_rbf_regression,krr_linear_regression, krr_pca_linear_regression, classifier)

    //! Point constraint types
    DECLARE_ENUM(point_constraint_type, 
        fix, impute, none)

    //! Angle sampling alignment approaches
    DECLARE_ENUM(angle_sampling_alignment_method,
        tangent, global, none)

	//! Angle sampling scaling approaches
	DECLARE_ENUM(scale_sampling_alignment_method,
		curve_length, global, none)
}
