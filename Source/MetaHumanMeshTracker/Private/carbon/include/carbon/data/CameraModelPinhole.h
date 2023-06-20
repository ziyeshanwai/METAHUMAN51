// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

#include <carbon/common/Common.h>

#include <carbon/common/EigenDenseBackwardsCompatible.h>

namespace epic {
namespace carbon {

/**
    @brief Simple pinhole camera model with the following projection equation:

    @code
    x = K * [R|t] * X
    @endcode

    where K is a 3x3 upper triangular camera intrinsic matrix, [R|t] is 3x4 affine transform
    matrix where R is a 3x3 3D rotation matrix, t is a 3x1 translation vector, X is a 3D point
    in world coordinates, and x is the projected point on the image plane (in pixels).

    K =  | fx  0  cx |
         |  0  fy cy |
         |  0  0  1  |

    Subclasses of the camera may also provide additional distortion parameters.

*/

template<class T>
class CameraModelPinhole {
    public:
        // --------------------------------------------------------------------
        // API
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // Constructors
        // --------------------------------------------------------------------

        /*!
          @brief Construct the pinhole camera model.
        */

        CameraModelPinhole() noexcept;

        // --------------------------------------------------------------------
        // Destructors
        // --------------------------------------------------------------------

        /**
            Destructor for camera models created using this class.
         */

        virtual ~CameraModelPinhole() noexcept;

        /**
            @brief Get camera intrinsic matrix.

            Intrinsic matrix with shape 3x3, including individual parameters fx, fy, cx, cy.
         */

        const Eigen::Matrix<T, 3, 3>& GetIntrinsics() const noexcept;

        /**
            @brief Set intrinsic matrix

            Set intrinsic matrix to the camera model.

            @param intrinsics
                Intrinsic (upper triangular) matrix with shape 3x3.
         */

        void SetIntrinsics(const Eigen::Matrix<T, 3, 3>& intrinsics);

        /**
            @brief Get extrinsics matrix.

            Get camera position and orienatation in referent coordinate system (3x4 transform matrix relative to referent origin).

            @return
                Affine transformation matrix with 3x4 shape [R|t].
        */

        const Eigen::Matrix<T, 3, 4>& GetExtrinsics() const noexcept;

        /**
            @brief Set extrinsics matrix.

            Set camera position and orienatation in referent coordinate system (3x4 transform matrix relative to referent origin).

            @param extrinsics
                Affine transform matrix with shape 3x4, including rotation matrix and translation vector [R|t].
        */

        void SetExtrinsics(const Eigen::Matrix<T, 3, 4>& extrinsics);

        /**
            @brief Get width of the image frame.

            @return
                Width of the image frame.
        */

        int GetWidth() const noexcept;

        /**
            @brief Set the width of the image frame.

            @param width
                Image frame width in pixels.
        */

        void SetWidth(const int width) noexcept;

        /**
            @brief Get height of the image frame.

            @return
            Height of the image frame.
        */

        int GetHeight() const noexcept;

        /**
            @brief Set the width of the image frame.

            @param height
                Image frame height in pixels.
        */

        void SetHeight(const int height) noexcept;


        /**
            @brief Get label of the camera object.

            @return
                String representing camera model name or ID.
        */

        std::string GetLabel() const noexcept;

        /**
            @brief Set label of the camera object.

            @param label
                String representing camera name or ID.
        */

        void SetLabel(std::string label) noexcept;

        /**
            @brief Get camera object model string.

            @return
                String representing camera model name.
        */

        std::string GetModel() const noexcept;

        /**
            @brief Set camera object model string.

            @param model
                String representing camera model name.
        */

        void SetModel(std::string model) noexcept;

    protected:
        // Intrinsics matrix
        Eigen::Matrix<T, 3, 3> m_intrinsics;

        // Extrinsics matrix [R|t]
        Eigen::Matrix<T, 3, 4> m_extrinsics;

        // Image dimensions
        int m_width;
        int m_height;

        // Label of the camera
        std::string m_label;

        // Camera model
        std::string m_model;
};
}
}
