// Copyright Epic Games, Inc. All Rights Reserved.

/*
    Utilities for interoperability with other libraries.

    This is an internal header, and it is not meant for API use.
    It is only included through carbon/data/Pixel.h file.
*/


#pragma once

#ifndef _CARBON_PIXEL_IS_INCLUDED
    #error "This is an implementation header of <carbon/data/Pixel.h> and is not supposed to be included on its own."
#endif


/*
 Supporting OpenCV traits, if integration is enabled

    This enables us to use cv::Mat with carbon::Pixel<T, N> structure,
    as we would use cv::Vec for e.g:

    @code
        cv::Mat mat(10, 10, CV_8UC3);
        mat.at<Pixel<unsigned char, 3>>(i, j) = 0; // this should work because of these traits.
    @endcode
*/
#ifdef CARBON_USE_OPENCV_MAT
    #include <opencv2/core/traits.hpp>

    namespace cv {
    namespace traits {
    template<class T, std::size_t N>
    struct Depth<epic::carbon::Pixel<T, N> > {
        enum {
            value = Depth<T>::value
        };
    };
    }

    template<class T, std::size_t N>
    struct DataType<epic::carbon::Pixel<T, N> > {
        typedef T value_type;
        typedef int work_type;
        typedef uchar channel_type;
        enum {
            type = cv::DataType<T>::type,
            channels = N,
            depth = cv::traits::Depth<T>::value
        };
    };
    }
#endif
