// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/Camera.h>
#include <nls/math/Math.h>

#include <string>

namespace epic {
namespace nls {

/**
 * The camera model as used in Agisoft PhotoScan/MetaShape.
 * @see (https://www.agisoft.com/pdf/metashape-pro_1_6_en.pdf, https://www.agisoft.com/pdf/metashape-pro_1_5_en.pdf)
 * We use the more generic version of tangential distortion parameters as used in version 1.5.
 *
 * x = X/Z
 * y = Y/Z
 * r = sqrt(x^2 + y^2)
 * x' = x (1 + K1 r^2 + K2 r^4 + K3 r^6 + K4 r^8) + (P1 (r^2 + 2x^2) + 2 P2 x y) (1 + P3 r^2 + P4 r^4)
 * y' = y (1 + K1 r^2 + K2 r^4 + K3 r^6 + K4 r^8) + (P2 (r^2 + 2y^2) + 2 P1 x y) (1 + P3 r^2 + P4 r^4)
 * px = width * 0.5 + cx + x' f + x' B1 + y' B2
 * py = height * 0.5 + cy + y' f
 *
 * Thereoretically B2 is the skew parameter "a" in the intrinsics, and B1 can be added to the focal length fx = f + B1
 * K =  |  fx a  cx' |
 *      |  0  fy cy' |
 *      |  0  0  1   |
 * However, in Agisoft photoscan undistortion also removes B1 and B2, and therefore we do not map the skew parameters into the intrinsics either.
 *
 * Important note: the tangential distortion parameters are swapped relative to the opencv model. Also the OpenCV does not support the K4 parameter.
 * https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 */
template <class T>
class MetaShapeCamera : public Camera<T>
{
public:
  	MetaShapeCamera() : Camera<T>()
        , m_radialDistortion(0, 0, 0, 0)
        , m_tangentialDistortion(0, 0, 0, 0)
        , m_skew(0, 0)
        , m_sensorID(-1)
        , m_cameraID(-1)
    {}

    virtual ~MetaShapeCamera() {}

    const Eigen::Vector<T, 4>& RadialDistortion() const { return m_radialDistortion; }
    void SetRadialDistortion(const Eigen::Vector<T, 4>& radialDistortion) { m_radialDistortion = radialDistortion; }

    const Eigen::Vector<T, 4>& TangentialDistortion() const { return m_tangentialDistortion; }
    void SetTangentialDistortion(const Eigen::Vector<T, 4>& tangentialDistortion) { m_tangentialDistortion = tangentialDistortion; }

    const Eigen::Vector<T, 2>& Skew() const { return m_skew; }
    void SetSkew(const Eigen::Vector<T, 2>& skew) { m_skew = skew; }

    int SensorID() const { return m_sensorID; }
    void SetSensorID(int id) { m_sensorID = id; }

    int CameraID() const { return m_cameraID; }
    void SetCameraID(int id) { m_cameraID = id; }

    //! @return the location of the distorted pixel
    Eigen::Vector2<T> Distort(const Eigen::Vector2<T>& pix) const
    {
        const Eigen::Vector3<T> pt = Camera<T>::ToImagePlane(pix);
        const T x = pt[0];
        const T y = pt[1];
        const T r2 = x * x + y * y;
        const T r4 = r2 * r2;
        const T r6 = r4 * r2;
        const T r8 = r4 * r4;
        const T K1 = m_radialDistortion[0];
        const T K2 = m_radialDistortion[1];
        const T K3 = m_radialDistortion[2];
        const T K4 = m_radialDistortion[3];
        const T P1 = m_tangentialDistortion[0];
        const T P2 = m_tangentialDistortion[1];
        const T P3 = m_tangentialDistortion[2];
        const T P4 = m_tangentialDistortion[3];
        const T radial = (T(1) + K1 * r2 + K2 * r4 + K3 * r6 + K4 * r8);
        const T tangentialX = P1 * (r2 + 2 * x * x) + 2 * P2 * x * y;
        const T tangentialY = P2 * (r2 + 2 * y * y) + 2 * P1 * x * y;
        const T xdash = x * radial + tangentialX * (T(1) + P3 * r2 + P4 * r4);
        const T ydash = y * radial + tangentialY * (T(1) + P3 * r2 + P4 * r4);
        const T u = (Camera<T>::Intrinsics()(0,0) + m_skew[0]) * xdash + m_skew[1] * ydash + Camera<T>::Intrinsics()(0,2);
        const T v = Camera<T>::Intrinsics()(1,1) * ydash + Camera<T>::Intrinsics()(1,2);
        return Eigen::Vector2<T>(u,v);
    }

    //! @return the location of the undistorted pixel
    Eigen::Vector2<T> Undistort(const Eigen::Vector2<T>& pix, T eps = T(1e-6), int maxIter = 100) const
    {
        const T ydash = (pix[1] - Camera<T>::Intrinsics()(1, 2)) / Camera<T>::Intrinsics()(1,1);
        const T xdash = (pix[0] - Camera<T>::Intrinsics()(0,2) - m_skew[1] * ydash) / (Camera<T>::Intrinsics()(0,0) + m_skew[0]);
        Eigen::Vector2<T> curr(xdash, ydash);
        Eigen::Vector2<T> next = curr;
        int iter = 0;
        do {
            curr = next;
            const T x = curr[0];
            const T y = curr[1];
            const T r2 = x * x + y * y;
            const T r4 = r2 * r2;
            const T r6 = r4 * r2;
            const T r8 = r4 * r4;
            const T K1 = m_radialDistortion[0];
            const T K2 = m_radialDistortion[1];
            const T K3 = m_radialDistortion[2];
            const T K4 = m_radialDistortion[3];
            const T P1 = m_tangentialDistortion[0];
            const T P2 = m_tangentialDistortion[1];
            const T P3 = m_tangentialDistortion[2];
            const T P4 = m_tangentialDistortion[3];
            const T radial = (T(1) + K1 * r2 + K2 * r4 + K3 * r6 + K4 * r8);
            const T tangentialX = P1 * (r2 + T(2) * x * x) + T(2) * P2 * x * y;
            const T tangentialY = P2 * (r2 + T(2) * y * y) + T(2) * P1 * x * y;
            next[0] = (xdash - tangentialX * (T(1) + P3 * r2 + P4 * r4)) / radial;
            next[1] = (ydash - tangentialY * (T(1) + P3 * r2 + P4 * r4)) / radial;
        } while ((curr - next).norm() > eps && iter++ < maxIter);
        curr = next;

        return Camera<T>::Project(Eigen::Vector3<T>(curr[0], curr[1], T(1)), /*withExtrinsics=*/false);
    }

private:
    //! The radial distortion parameters (K1, K2, K3, K4)
    Eigen::Vector<T, 4> m_radialDistortion;

    //! The tangential distortion parameters (P1, P2, P3, P4)
    Eigen::Vector<T, 4> m_tangentialDistortion;

    //! The skew parameters (B1, B2)
    Eigen::Vector<T, 2> m_skew;

    //! Sensor ID
    int m_sensorID;

    //! Camera ID
    int m_cameraID;
};


} // namespace nls
} //namespace epic
