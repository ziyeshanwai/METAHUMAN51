// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/data/CameraModelPinhole.h>


#ifdef _MSC_VER
__pragma(warning(push))
__pragma(warning(disable:4324)) 
// see explanation of warning: https://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html#StructHavingEigenMembers_othersolutions
// this should not be an issue due to use of the EIGEN_MAKE_ALIGNED_OPERATOR_NEW macro
#endif


namespace epic {
namespace carbon {

/**
    @brief The camera model with lens distortion parameters.
    Distortion parameters are stored in OpenCV format (K1, K2, P1, P2, K3)

    x = X/Z
    y = Y/Z
    r = sqrt(x^2 + y^2)
    x' = x (1 + K1 r^2 + K2 r^4 + K3 r^6) + (P1 (r^2 + 2x^2) + 2 P2 x y)
    y' = y (1 + K1 r^2 + K2 r^4 + K3 r^6) + (P2 (r^2 + 2y^2) + 2 P1 x y)
*/

template<class T>
class CameraModelOpenCV : public CameraModelPinhole<T> {
    public:
        // --------------------------------------------------------------------
        // API
        // --------------------------------------------------------------------
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // --------------------------------------------------------------------
        // Constructors
        // --------------------------------------------------------------------

        /*!
          @brief Construct the distortion camera model.
        */

        CameraModelOpenCV() noexcept;

        // --------------------------------------------------------------------
        // Destructors
        // --------------------------------------------------------------------

        /**
            Destructor for camera models created using this class.
         */

        virtual ~CameraModelOpenCV() noexcept;

        /**
            @brief Get camera distortion parameters.

            @return
                Distortion parameters stored in a vector of 5 elements - k1, k2, p1, p2, k3.
        */

        const Eigen::Vector<T, 5>& GetDistortionParams() const noexcept;

        /**
            @brief Set distortion parameters

            @param distortion
                Distortion parameters stored in a vector of 5 elements - k1, k2, p1, p2, k3.
        */

        void SetDistortionParams(const Eigen::Vector<T, 5>& distortion) noexcept;

    private:
        // OpenCV format distortion parameters (k1, k2, p1, p2, k3)
        Eigen::Vector<T, 5> m_distortion;
};

}
}


#ifdef _MSC_VER
__pragma(warning(pop))
#endif
