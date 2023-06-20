// Copyright (C) 2007  Davis E. King (davis@dlib.net)
// License: Boost Software License   See LICENSE.txt for the full license.

// Modifications (indicated by comment) Copyright Epic Games, Inc. All Rights Reserved.
// Modifications subject to applicable Unreal Engine license agreement.
// This file adds classe cm_linear_kernel which is a modified (and renamed) version of the original
// linear_kernel classes in dlib.
// Class has been placed in a cm namespace.
// Individual modifications beyond this are commented in the code.

#pragma once

#include <dlib/svm/kernel_abstract.h>
#include <cmath>
#include <limits>
#include <sstream>
#include <dlib/matrix.h>
#include <dlib/algs.h>
#include <dlib/serialize.h>

namespace cm
{

// ----------------------------------------------------------------------------------------

    template < typename kernel_type > struct cm_kernel_derivative;


// ----------------------------------------------------------------------------------------

    template <typename T>
    struct cm_linear_kernel
    {
        typedef typename T::type scalar_type;
        typedef T sample_type;
        typedef typename T::mem_manager_type mem_manager_type;

        // T must be capable of representing a column vector.
        COMPILE_TIME_ASSERT(T::NC == 1 || T::NC == 0);

        // MODIFICATION START: added functions below
        cm_linear_kernel() = default;
        cm_linear_kernel(const cm_linear_kernel& ) = default;
        cm_linear_kernel& operator=(const cm_linear_kernel& ) = default;

        template<typename T2>
        cm_linear_kernel(const cm_linear_kernel<T2>& ) {}

        template<typename T2>
        cm_linear_kernel& operator=(const cm_linear_kernel<T2>& )
        {
            return *this;
        }
        // MODIFICATION END

        scalar_type operator() (
            const sample_type& a,
            const sample_type& b
        ) const
        { 
            return trans(a)*b;
        }

        bool operator== (
            const cm_linear_kernel& 
        ) const
        {
            return true;
        }
    };

    template <
        typename T
        >
    void serialize (
        const cm_linear_kernel<T>& ,
        std::ostream& 
    ){}

    template <
        typename T
        >
    void deserialize (
        cm_linear_kernel<T>& ,
        std::istream&  
    ){}

    template <
        typename T 
        >
    struct cm_kernel_derivative<cm_linear_kernel<T> >
    {
        typedef typename T::type scalar_type;
        typedef T sample_type;
        typedef typename T::mem_manager_type mem_manager_type;

        cm_kernel_derivative(const cm_linear_kernel<T>& k_) : k(k_){}

        const sample_type& operator() (const sample_type& x, const sample_type& ) const
        {
            return x;
        }

        const cm_linear_kernel<T>& k;
    };


// ----------------------------------------------------------------------------------------

}



