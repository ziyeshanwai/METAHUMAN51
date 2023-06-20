// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/DiffDataAffine.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {

/**
 * A simple pinhole Camera with the following projection equation:
 *
 * @code
 * px = K * [R|t] * Pw
 * @code
 *
 * where K is a 3x3 triangular camera intrinsic matrix, R is a 3x3 3D rotation matrix, t is a 3x1 translation vector, Pw is a 3D point
 * in world coordinates, and px is the projected point on the image plane (in pixels).
 *
* K =  | fx  a  cx |
*      |  0  fy cy |
*      |  0  0  1  |
 *
 * Subclasses of the camera may also provide additional distortion parameters.
 *
 * Important: the camera model assumes the origin of the image is the corner of the top left pixel. so the center
 *            of the top-left pixel has coordinates (0.5, 0.5). This is different to opencv where the origin is at the
 *            center of the top-left pixel.
 */
template <class T>
class Camera
{
public:
  	Camera()
      : m_intrinsics(Eigen::Matrix<T, 3, 3>::Identity())
      , m_extrinsics()
      , m_width(0)
      , m_height(0)
	{}

    virtual ~Camera() {}

    int Width() const { return m_width; }
    void SetWidth(int width) { m_width = width; }

    int Height() const { return m_height; }
    void SetHeight(int height) { m_height = height; }

    const std::string& Label() const { return m_label; }
    void SetLabel(const std::string& label) { m_label = label; }

    const Eigen::Matrix<T, 3, 3>& Intrinsics() const { return m_intrinsics; }
    void SetIntrinsics(const Eigen::Matrix<T, 3, 3>& intrinsics)
    {
        if (intrinsics(1, 0) != 0 || intrinsics(2, 0) != 0 || intrinsics(2, 1) != 0 || intrinsics(2, 2) != 1) {
            CARBON_CRITICAL("invalid intrinsics - needs to be an upper triangular matrix with m(2,2) == 1");
        }
        m_intrinsics = intrinsics;
    }

    const Affine<T, 3, 3>& Extrinsics() const { return m_extrinsics; }
    void SetExtrinsics(const Affine<T, 3, 3>& extrinsics) { m_extrinsics = extrinsics; }

    //! @return the camera origin in world coordinates
    const Eigen::Vector3<T> Origin() const { return -m_extrinsics.Linear().transpose() * m_extrinsics.Translation(); }

    //! Projects a 3d point into the image
    Eigen::Vector2<T> Project(const Eigen::Vector3<T>& pt, bool withExtrinsics) const
    {
        Eigen::Vector3<T> projected;
        if (withExtrinsics) {
            projected = m_intrinsics * m_extrinsics.Transform(pt);
        } else {
            projected = m_intrinsics * pt;
        }
        if (projected[2] != 0) {
            projected /= projected[2];
        }
        return Eigen::Vector2<T>(projected[0], projected[1]);
    }

    //! @return the image plane position for pixel @pix
    Eigen::Vector3<T> ToImagePlane(const Eigen::Vector2<T>& pix) const
    {
        const T y = (pix[1] - m_intrinsics(1, 2)) / m_intrinsics(1, 1);
        const T x = (pix[0] - m_intrinsics(0, 2) - y * m_intrinsics(0, 1)) / m_intrinsics(0, 0);
        return Eigen::Vector3<T>(x, y, 1);
    }

    //! Unprojects a 2d point into 3D
    Eigen::Vector3<T> Unproject(const Eigen::Vector2<T>& pix, T depth, bool withExtrinsics) const
    {
        Eigen::Vector3<T> posInCamera = Intrinsics().inverse() * Eigen::Vector3<T>(pix[0], pix[1], 1) * depth;
        if (withExtrinsics) {
            return Extrinsics().Inverse().Transform(posInCamera);
        } else {
            return posInCamera;
        }
    }

    //! Scales the camera parameters which is equivalent to scaling images associated with the camera.
    void Scale(const T scale)
    {
        m_width = int(scale * m_width);
        m_height = int(scale * m_height);
        m_intrinsics.row(0) *= scale;
        m_intrinsics.row(1) *= scale;
    }

    /**
     * Calculates a rendering projection matrix without extrinsics.
     * x is mapped between [-1, 1]
     * y is mapped between [-1, 1]
     * z is mapped between [1, 0] where 1=depthNear, and 0=depthFar
     *
     * Calculation to pixel coordinates: [0, width], [0, height]
     * x' = fx X/Z + skew Y/Z + cx
     * y' = fy X/Z + cy
     *
     * Calculation to normalized coordinates: [-1, 1], [-1, 1]:
     * x'' = 2 * x' / width - 1
     * y'' = 2 * y' / height - 1
     *
     * Depth:
     * depthNear maps to 1, depthFar maps to 0
     * a/depthNear + b = 1
     * a/depthFar + b = 0
     * => a = depthNear * depthFar / (depthFar - depthNear)
     * => b = - depthNear / (depthFar - depthNear)
     *
     * Important notes:
     * 1) depthNear maps to 1, depthFar maps to 0 (as opposed to the common depthNear=0 and depthFar=1)
     * 2) -1 is the top of the image, not the bottom
     * 3) the camera is looking in positive z direction
     */
    Eigen::Matrix<T, 4, 4> RenderingProjectionMatrix(const T depthNear, const T depthFar) const
    {
        Eigen::Matrix<T, 4, 4> m = Eigen::Matrix<T, 4, 4>::Zero();
        m(0,0) = T(2) / T(m_width) * m_intrinsics(0,0);
        m(0,1) = T(2) / T(m_width) * m_intrinsics(0,1);
        m(0,2) = T(2) / T(m_width) * m_intrinsics(0,2) - T(1);
        m(1,1) = T(2) / T(m_height) * m_intrinsics(1,1);
        m(1,2) = T(2) / T(m_height) * m_intrinsics(1,2) - T(1);
        m(2,2) = - depthNear / (depthFar - depthNear);
        m(2,3) = depthNear * depthFar / (depthFar - depthNear);
        m(3,2) = 1;
        return m;
    }

    /**
     * Projects the input points into the image and also calculates the Jacobian
     *
     * px = (K(0,0) * p[0] + K(0, 1) * p[1] + K(0,2)) / p[2]
     * py = (K(1,1) * p[1] + K(1,2)) / p[2]
     */
    DiffDataMatrix<T, 2, -1> Project(const DiffDataMatrix<T, 3, -1>& pts, bool withExtrinsics) const
    {
        const int numPts = pts.Cols();
        JacobianConstPtr<T> Jacobian;
        VectorPtr<T> values = std::make_shared<Vector<T>>(numPts * 2);
        Eigen::Map<Eigen::Matrix<T, 2, -1>> pixs(values->data(), 2, numPts);

        DiffDataAffine<T, 3, 3> diffExtrinsics(Extrinsics());
        DiffDataMatrix<T, 3, -1> transformedPts = withExtrinsics ? diffExtrinsics.Transform(pts) : pts;
        std::vector<Eigen::Triplet<T>> triplets;

        const auto& fx = Intrinsics()(0,0);
        const auto& skew = Intrinsics()(0,1);
        const auto& cx = Intrinsics()(0,2);
        const auto& fy = Intrinsics()(1,1);
        const auto& cy = Intrinsics()(1,2);

        for (int i = 0; i < numPts; i++) {
            const T x = transformedPts.Matrix()(0,i);
            const T y = transformedPts.Matrix()(1,i);
            const T z = transformedPts.Matrix()(2,i);
            const T invZ = T(1) / z;
            const T xn = x * invZ;
            const T yn = y * invZ;
            pixs(0,i) = fx * xn + skew * yn + cx;
            pixs(1,i) = fy * yn + cy;
            if (transformedPts.HasJacobian()) {
                triplets.push_back(Eigen::Triplet<T>(2 * i + 0, 3 * i + 0, fx * invZ));
                triplets.push_back(Eigen::Triplet<T>(2 * i + 0, 3 * i + 1, skew * invZ));
                triplets.push_back(Eigen::Triplet<T>(2 * i + 0, 3 * i + 2, (- fx * xn - skew * yn) * invZ));
                triplets.push_back(Eigen::Triplet<T>(2 * i + 1, 3 * i + 1, fy * invZ));
                triplets.push_back(Eigen::Triplet<T>(2 * i + 1, 3 * i + 2, - fy * yn * invZ));
            }
        }

        if (transformedPts.HasJacobian()) {
            SparseMatrix<T> localJacobian(2 * numPts, 3 * numPts);
            localJacobian.setFromTriplets(triplets.begin(), triplets.end());
            Jacobian = transformedPts.Jacobian().Premultiply(localJacobian);
        }

        return DiffDataMatrix<T, 2, -1>(2, numPts, DiffData<T>(values, Jacobian));
    }

private:
    //! The intrinsics K
    Eigen::Matrix<T, 3, 3> m_intrinsics;

    //! The camera extrinsics [R|t]
	Affine<T, 3, 3> m_extrinsics;

    //! The image dimensions
    int m_width;
    int m_height;

    //! Label of the camera (optional)
    std::string m_label;
};


} // namespace nls
} //namespace epic
