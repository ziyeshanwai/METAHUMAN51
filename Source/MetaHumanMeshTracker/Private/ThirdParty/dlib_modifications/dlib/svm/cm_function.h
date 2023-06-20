// Copyright (C) 2007  Davis E. King (davis@dlib.net)
// License: Boost Software License   See LICENSE.txt for the full license.

// Modifications (indicated by comment) Copyright Epic Games, Inc. All Rights Reserved.
// Modifications subject to applicable Unreal Engine license agreement.
// This file adds classes cm_decision_function and cm_normalized_function which are modified (and renamed) versions of the original
// decision_function and normalized_function classes in dlib.
// Classes have been placed in a cm namespace.
// Individual modifications beyond this are commented in the code.


#pragma once

#include <dlib/svm/function_abstract.h>
#include <cmath>
#include <limits>
#include <sstream>
#include <dlib/matrix.h>
#include <dlib/algs.h>
#include <dlib/serialize.h>
#include <dlib/rand.h>
#include <dlib/statistics.h>
#include <dlib/svm/kernel_matrix.h>
#include "cm_kernel.h"
#include <dlib/statistics/cm_statistics.h>
#include <dlib/svm/sparse_kernel.h>

namespace cm
{

// ----------------------------------------------------------------------------------------

    template <
        typename K
        >
    struct cm_decision_function
    {
        typedef K kernel_type;
        typedef typename K::scalar_type scalar_type;
        typedef typename K::scalar_type result_type;
        typedef typename K::sample_type sample_type;
        typedef typename K::mem_manager_type mem_manager_type;

        typedef matrix<scalar_type,0,1,mem_manager_type> scalar_vector_type;
        typedef matrix<sample_type,0,1,mem_manager_type> sample_vector_type;

        scalar_vector_type alpha;
        scalar_type b;
        K kernel_function;
        sample_vector_type basis_vectors;

        cm_decision_function (
        ) : b(0), kernel_function(K()) {}

        cm_decision_function (
            const cm_decision_function& d
        ) : 
            alpha(d.alpha), 
            b(d.b),
            kernel_function(d.kernel_function),
            basis_vectors(d.basis_vectors) 
        {}

        // MODIFICATION: default assignment operator
        cm_decision_function& operator=(const cm_decision_function& other) = default;

        cm_decision_function (
            const scalar_vector_type& alpha_,
            const scalar_type& b_,
            const K& kernel_function_,
            const sample_vector_type& basis_vectors_
        ) :
            alpha(alpha_),
            b(b_),
            kernel_function(kernel_function_),
            basis_vectors(basis_vectors_)
        {}

        // MODIFICATION START: added functions below
        template<typename U>
        cm_decision_function(const cm_decision_function<U>& other)
        {
            this->operator=<U>(other);
        }

        template<typename U>
        cm_decision_function& operator=(const cm_decision_function<U>& other)
        {
            alpha = matrix_cast<scalar_type>(other.alpha);
            b = static_cast<scalar_type>(other.b);
            kernel_function = other.kernel_function;
            basis_vectors.set_size(other.basis_vectors.size());
            for(long i = 0; i < basis_vectors.size(); ++i)
            {
                using target_scalar_type = typename sample_type::type;
                basis_vectors(i) = matrix_cast<target_scalar_type>(other.basis_vectors(i));
            }

            return *this;
        }
        // MODIFICATION END

        result_type operator() (
            const sample_type& x
        ) const
        {
            result_type temp = 0;
            for (long i = 0; i < alpha.nr(); ++i)
                temp += alpha(i) * kernel_function(x,basis_vectors(i));

            return temp - b;
        }
    };

    template <
        typename K
        >
    void serialize (
        const cm_decision_function<K>& item,
        std::ostream& out
    )
    {
        try
        {
            using dlib::serialize;
            serialize(item.alpha, out);
            serialize(item.b,     out);
            serialize(item.kernel_function, out);
            serialize(item.basis_vectors, out);
        }
        catch (serialization_error& e)
        { 
            throw serialization_error(e.info + "\n   while serializing object of type cm_decision_function"); 
        }
    }

    template <
        typename K
        >
    void deserialize (
        cm_decision_function<K>& item,
        std::istream& in 
    )
    {
        try
        {
            using dlib::deserialize;
            deserialize(item.alpha, in);
            deserialize(item.b, in);
            deserialize(item.kernel_function, in);
            deserialize(item.basis_vectors, in);
        }
        catch (serialization_error& e)
        { 
            throw serialization_error(e.info + "\n   while deserializing object of type cm_decision_function"); 
        }
    }


// ----------------------------------------------------------------------------------------

    template <
        typename function_type,
        typename normalizer_type = cm_vector_normalizer<typename function_type::sample_type>
        >
    struct cm_normalized_function 
    {
        typedef typename function_type::result_type result_type;
        typedef typename function_type::sample_type sample_type;
        typedef typename function_type::mem_manager_type mem_manager_type;

        normalizer_type normalizer;
        function_type function;

        // MODIFICATION START: added functions below
        cm_normalized_function() = default;
        cm_normalized_function(const cm_normalized_function& other) = default;
        cm_normalized_function& operator=(const cm_normalized_function& other) = default;

        template<typename function_type2, typename normalizer_type2>
        cm_normalized_function(
            const cm_normalized_function<function_type2, normalizer_type2>& f
        )
        {
            this->operator=<function_type2, normalizer_type2>(f);
        }

        template<typename function_type2, typename normalizer_type2>
        cm_normalized_function& operator=(const cm_normalized_function<function_type2, normalizer_type2>& other)
        {
            normalizer = other.normalizer;
            function = other.function;

            return *this;
        }
        // MODIFICATION END

        const std::vector<result_type> get_labels(
        ) const { return function.get_labels(); }

        unsigned long number_of_classes (
        ) const { return function.number_of_classes(); }

        cm_normalized_function (
            const vector_normalizer<sample_type>& normalizer_,
            const function_type& funct 
        ) : normalizer(normalizer_), function(funct) {}

        result_type operator() (
            const sample_type& x
        ) const { return function(normalizer(x)); }
    };

    template <
        typename function_type,
        typename normalizer_type 
        >
    void serialize (
        const cm_normalized_function<function_type,normalizer_type>& item,
        std::ostream& out
    )
    {
        try
        {
            serialize(item.normalizer, out);
            serialize(item.function,     out);
        }
        catch (serialization_error& e)
        { 
            throw serialization_error(e.info + "\n   while serializing object of type cm_normalized_function"); 
        }
    }

    template <
        typename function_type,
        typename normalizer_type 
        >
    void deserialize (
        cm_normalized_function<function_type,normalizer_type>& item,
        std::istream& in 
    )
    {
        try
        {
            deserialize(item.normalizer, in);
            deserialize(item.function, in);
        }
        catch (serialization_error& e)
        { 
            throw serialization_error(e.info + "\n   while deserializing object of type cm_normalized_function"); 
        }
    }

}



