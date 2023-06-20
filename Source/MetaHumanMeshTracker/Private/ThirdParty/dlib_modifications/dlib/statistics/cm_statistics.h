// Copyright (C) 2008  Davis E. King (davis@dlib.net), Steve Taylor
// License: Boost Software License   See LICENSE.txt for the full license.

// Modifications (indicated by comment) Copyright Epic Games, Inc. All Rights Reserved.
// Modifications subject to applicable Unreal Engine license agreement.
// This file adds classe cm_vector_normalizer which is a modified (and renamed) version of the original
// vector_normalizer classes in dlib.
// Class has been placed in a cm namespace.
// Individual modifications beyond this are commented in the code.


#pragma once


#include <dlib/statistics/statistics_abstract.h>
#include <limits>
#include <cmath>
#include <dlib/algs.h>
#include <dlib/matrix.h>
#include <dlib/sparse_vector.h>

namespace cm
{

    template <
        typename matrix_type
        >
    class cm_vector_normalizer
    {
        // MODIFICATION: added friend declaration
        template<typename matrix_type2>
        friend class cm_vector_normalizer;

    public:
        typedef typename matrix_type::mem_manager_type mem_manager_type;
        typedef typename matrix_type::type scalar_type;
        typedef matrix_type result_type;

        // MODIFICATION START: added functions below
        cm_vector_normalizer() = default;
        cm_vector_normalizer(const cm_vector_normalizer& other) = default;
        cm_vector_normalizer& operator=(const cm_vector_normalizer& other) = default;

        template<typename matrix_type2>
        cm_vector_normalizer(const cm_vector_normalizer<matrix_type2>& other)
        {
            this->operator=<matrix_type2>(other);
        }

        template<typename matrix_type2>
        cm_vector_normalizer& operator=(const cm_vector_normalizer<matrix_type2>& other)
        {
            m = matrix_cast<scalar_type>(other.m);
            sd = matrix_cast<scalar_type>(other.sd);

            return *this;
        }
        // MODIFICATION END

        template <typename vector_type>
        void train (
            const vector_type& samples
        )
        {
            // make sure requires clause is not broken
            DLIB_ASSERT(samples.size() > 0,
                "\tvoid cm_vector_normalizer::train()"
                << "\n\tyou have to give a nonempty set of samples to this function"
                << "\n\tthis: " << this
                );

            m = mean(mat(samples));
            sd = reciprocal(sqrt(variance(mat(samples))));

            DLIB_ASSERT(is_finite(m), "Some of the input vectors to cm_vector_normalizer::train() have infinite or NaN values");
        }

        long in_vector_size (
        ) const
        {
            return m.nr();
        }

        long out_vector_size (
        ) const
        {
            return m.nr();
        }

        const matrix_type& means (
        ) const
        {
            return m;
        }

        const matrix_type& std_devs (
        ) const
        {
            return sd;
        }

        const result_type& operator() (
            const matrix_type& x
        ) const
        {
            // make sure requires clause is not broken
            DLIB_ASSERT(x.nr() == in_vector_size() && x.nc() == 1,
                "\tmatrix cm_vector_normalizer::operator()"
                << "\n\t you have given invalid arguments to this function"
                << "\n\t x.nr():           " << x.nr()
                << "\n\t in_vector_size(): " << in_vector_size()
                << "\n\t x.nc():           " << x.nc()
                << "\n\t this:             " << this
                );

            temp_out = pointwise_multiply(x-m, sd);
            return temp_out;
        }

        void swap (
            cm_vector_normalizer& item
        )
        {
            m.swap(item.m);
            sd.swap(item.sd);
            temp_out.swap(item.temp_out);
        }

        template <typename mt>
        friend void deserialize (
            cm_vector_normalizer<mt>& item, 
            std::istream& in
        ); 

        template <typename mt>
        friend void serialize (
            const cm_vector_normalizer<mt>& item, 
            std::ostream& out 
        );

        // MODIFICATION: added constructor
		cm_vector_normalizer(const matrix_type& mean, const matrix_type& standard_deviation) : m(mean), sd(standard_deviation)
		{

		}

    private:

        // ------------------- private data members -------------------

        matrix_type m, sd;

        // This is just a temporary variable that doesn't contribute to the
        // state of this object.
        mutable matrix_type temp_out;
    };

// ----------------------------------------------------------------------------------------

    template <
        typename matrix_type
        >
    inline void swap (
        cm_vector_normalizer<matrix_type>& a, 
        cm_vector_normalizer<matrix_type>& b 
    ) { a.swap(b); }   

// ----------------------------------------------------------------------------------------

    template <
        typename matrix_type
        >
    void deserialize (
        cm_vector_normalizer<matrix_type>& item, 
        std::istream& in
    )   
    {
        deserialize(item.m, in);
        deserialize(item.sd, in);
        // Keep deserializing the pca matrix for backwards compatibility.
        matrix<double> pca;
        deserialize(pca, in);

        if (pca.size() != 0)
            throw serialization_error("Error deserializing object of type cm_vector_normalizer\n"   
                                        "It looks like a serialized cm_vector_normalizer_pca was accidentally deserialized into \n"
                                        "a cm_vector_normalizer object.");
    }

// ----------------------------------------------------------------------------------------

    template <
        typename matrix_type
        >
    void serialize (
        const cm_vector_normalizer<matrix_type>& item, 
        std::ostream& out 
    )
    {
        serialize(item.m, out);
        serialize(item.sd, out);
        // Keep serializing the pca matrix for backwards compatibility.
        matrix<double> pca;
        serialize(pca, out);
    }

}


