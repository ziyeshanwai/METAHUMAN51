// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/Mesh.h>
#include <nls/math/Math.h>

namespace epic::nls {
    /**
     * Calculates the intrinsic camera parameters from a camera resolution and the vertical field of view.
     */
    template<class T>
    epic::nls::Camera<T> CameraFromVerticalFOV(const T angleInRadians, int width, int height) {
        epic::nls::Camera<T> camera;
        camera.SetWidth(width);
        camera.SetHeight(height);
        const T tanHalfFovy = T(tan(angleInRadians / 2.0));
        const T aspect = T(width) / T(height);
        Eigen::Matrix3f intrinsics = Eigen::Matrix3f::Identity();
        intrinsics(0, 0) = T(width / 2.0 / (tanHalfFovy * aspect));
        intrinsics(0, 2) = T(width / 2.0);
        intrinsics(1, 1) = T(height / 2.0 / tanHalfFovy);
        intrinsics(1, 2) = T(height / 2.0);
        camera.SetIntrinsics(intrinsics);
        return camera;
    }

    /**
     * Calculates the extrinsics for a camera using a modified version of "lookat". In this version the caller needs
     * to define the down vector which corresponds to the positive y direction in the camera which is pointing downwards
     * in the camera image.
     */
    template<class T>
    epic::nls::Affine<T, 3, 3> LookAt(const Eigen::Vector3<T>& eye, const Eigen::Vector3<T>& center,
                                      const Eigen::Vector3<T>& down) {
        Eigen::Vector3<T> zDir = (center - eye).normalized();
        Eigen::Vector3<T> yDir = down;
        Eigen::Vector3<T> xDir = yDir.cross(zDir).normalized();
        yDir = zDir.cross(xDir).normalized();
        Eigen::Matrix3<T> rot;
        rot.row(0) = xDir;
        rot.row(1) = yDir;
        rot.row(2) = zDir;
        epic::nls::Affine<T, 3, 3> aff;
        aff.SetLinear(rot);
        aff.SetTranslation(- rot * eye);
        return aff;
    }

    /**
     * Creates an image backplane for \p camera at \p distance.
     * @param [in] camera          The camera for which the image plane is created.
     * @param [in] distance        The distance at which the image plane is created.
     * @param [in] withExtrinsics  Whether to create the image plane in world coordinates (true) or in camera coordinates (false).
     * @return A quad mesh with a single quad with the vertices and the texture coordinates so that a camera image can be assigned as texture.
     */
    template<class T>
    Mesh<T> CreateImagePlane(const Camera<T>& camera, const T distance, bool withExtrinsics) {
        Eigen::Matrix<float, 3, -1> quadVertices(3, 4);
        quadVertices.col(0) = camera.Unproject(Eigen::Vector2f(0, 0), distance, withExtrinsics);
        quadVertices.col(1) = camera.Unproject(Eigen::Vector2f(0, camera.Height()), distance, withExtrinsics);
        quadVertices.col(2) = camera.Unproject(Eigen::Vector2f(camera.Width(), camera.Height()), distance, withExtrinsics);
        quadVertices.col(3) = camera.Unproject(Eigen::Vector2f(camera.Width(), 0), distance, withExtrinsics);
        Eigen::Matrix<float, 2, -1> quadTexcoords(2, 4);
        quadTexcoords.col(0) = Eigen::Vector2f(0, 0);
        quadTexcoords.col(1) = Eigen::Vector2f(0, 1);
        quadTexcoords.col(2) = Eigen::Vector2f(1, 1);
        quadTexcoords.col(3) = Eigen::Vector2f(1, 0);
        Eigen::Vector4i quad(0, 1, 2, 3);
        Mesh<float> quadMesh;
        quadMesh.SetVertices(quadVertices);
        quadMesh.SetTexcoords(quadTexcoords);
        quadMesh.SetQuads(quad);
        quadMesh.SetTexQuads(quad);
        return quadMesh;
    }
}  // namespace epic::nls
