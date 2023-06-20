// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/Camera.h>

namespace epic::nls {
    // ! Rectified stereo camera pair
    template <class T>
    class StereoCameraPair {
        public:
            StereoCameraPair() = default;
            StereoCameraPair(const Camera<T>& cameraLeft, const Camera<T>& cameraRight) {
                SetCameras(cameraLeft, cameraRight);
            }

            void SetCameras(const Camera<T>& cameraLeft, const Camera<T>& cameraRight) {
                m_cameraLeft = cameraLeft;
                m_cameraRight = cameraRight;
                if (!m_cameraLeft.Extrinsics().Linear().isApprox(m_cameraRight.Extrinsics().Linear(), 1e-5f) ||
                    !m_cameraLeft.Extrinsics().Transform(m_cameraRight.Origin()).normalized().isApprox(Eigen::Vector3f(1, 0, 0),
                                                                                                       1e-5f) ||
                    (m_cameraLeft.Intrinsics()(0, 0) != m_cameraRight.Intrinsics()(0, 0)) ||
                    (m_cameraLeft.Intrinsics()(1, 1) != m_cameraRight.Intrinsics()(1, 1)) ||
                    (m_cameraLeft.Intrinsics()(1, 2) != m_cameraRight.Intrinsics()(1, 2)) ||
                    (m_cameraLeft.Height() != m_cameraRight.Height())) {
                    throw std::runtime_error("camera pair is not rectified");
                }
            }

            const T Baseline() const {
                return (m_cameraLeft.Origin() - m_cameraRight.Origin()).norm();
            }

            const T DisparityOffset() const {
                return m_cameraRight.Intrinsics()(0, 2) - m_cameraLeft.Intrinsics()(0, 2);
            }

            const Camera<T>& CameraLeft() const {
                return m_cameraLeft;
            }

            const Camera<T>& CameraRight() const {
                return m_cameraRight;
            }

        private:
            Camera<T> m_cameraLeft;
            Camera<T> m_cameraRight;
    };
}  // namespace epic::nls
