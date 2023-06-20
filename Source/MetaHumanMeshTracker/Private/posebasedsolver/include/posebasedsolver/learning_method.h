// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "enum_ext.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
#include <dlib/svm.h>
#include <dlib/svm/cm_function.h>
#include <dlib/svm/cm_kernel.h>
#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_RENABLE_WARNINGS

namespace cm
{
    using namespace dlib;

    DECLARE_ENUM(learning_method, krr_linear, krr_rbf, krr_poly3, krr_poly2, krls)

    template<typename T>
    using linear_predictor = cm_normalized_function<cm_decision_function<cm_linear_kernel<dlib::matrix<T, 0, 1>>>>;
    template<typename T, unsigned int N>
    using rbf_predictor = normalized_function<decision_function<radial_basis_kernel<dlib::matrix<T, N, 1>>>>;
    template<typename T>
    using polynomial_predictor = normalized_function<decision_function<polynomial_kernel<dlib::matrix<T, 0, 1>>>>;


    template<typename K>
    struct kernel_traits;

    template<typename T>
    struct kernel_traits<cm_linear_kernel<dlib::matrix<T, 0, 1>>>
    {
        using scalar_type = T;

        template<typename T2>
        using as = cm_linear_kernel<dlib::matrix<T2, 0, 1>>;
    };


    template<typename P>
    struct predictor_traits;

    template<typename T>
    struct predictor_traits<linear_predictor<T>>
    {
        using scalar_type = T;
        using kernel_type = linear_kernel<dlib::matrix<T, 0, 1>>;

        template<typename T2>
        using as = linear_predictor<T2>;
    };

}