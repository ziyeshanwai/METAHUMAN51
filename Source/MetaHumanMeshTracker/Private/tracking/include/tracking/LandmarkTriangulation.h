// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/Camera.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/MultiCameraTriangulation.h>
#include <nls/geometry/RayTriangleIntersection.h>
#include <nrr/landmarks/LandmarkInstance.h>
#include <reconstruction/MultiCameraSetup.h>

#include <limits>
#include <map>
#include <set>

namespace epic::nls {

/**
 * Triangulate landmarks in 3D.
 */
template <class T>
std::map<std::string, Eigen::Vector3<T>> TriangulateLandmarks(const MultiCameraSetup<T>& cameraSetup, const std::map<std::string, LandmarkInstance<T, 2>>& landmarkInstances) {

    std::map<std::string, Eigen::Vector3<T>> reconstructedLandmarkPositions;

    if (landmarkInstances.size() > 1) {
        std::set<std::string> landmarkNames;
        for (const auto& [_, landmarkInstance] : landmarkInstances) {
            if (landmarkInstance.GetLandmarkConfiguration() == nullptr) {
                CARBON_CRITICAL("landmark configuration is not set for the landmark instance");
            }
            for (const auto& [landmarkName, _2] : landmarkInstance.GetLandmarkConfiguration()->LandmarkMapping()) {
                landmarkNames.emplace(landmarkName);
            }
        }

        for (const std::string& landmarkName : landmarkNames) {
            std::vector<Eigen::Vector2f> pixels;
            std::vector<float> confidences;

            std::vector<Camera<float> > undistortedLandmarkCameras;

            for (const auto&[cameraName, landmarkInstance] : landmarkInstances) {
                if (landmarkInstance.GetLandmarkConfiguration()->HasLandmark(landmarkName)) {
                    const int landmarkIndex = landmarkInstance.GetLandmarkConfiguration()->IndexForLandmark(landmarkName);
                    if (landmarkInstance.Confidence()[landmarkIndex] > 0) {
                        undistortedLandmarkCameras.push_back(cameraSetup.GetCamera(cameraName));  // converting MetaShapeCamera to Camera implicitly
                        pixels.push_back(landmarkInstance.Points().col(landmarkIndex));
                        confidences.push_back(landmarkInstance.Confidence()[landmarkIndex]);
                    }
                }
            }

            if (pixels.size() > 1) {
                MultiCameraTriangulation<float> multiCameraTriangulation;
                multiCameraTriangulation.SetCameras(undistortedLandmarkCameras);
                Eigen::Vector3f pos = multiCameraTriangulation.Triangulate(pixels);
                pos = multiCameraTriangulation.TriangulateNonlinear(pos, pixels, confidences);
                reconstructedLandmarkPositions[landmarkName] = pos;
            }
        }
    } else {
        CARBON_CRITICAL("at least two cameras with landmarks are required for triangulation");
    }

    return reconstructedLandmarkPositions;
}

/**
 * Triangulate landmarks by intersecting the 2D landmarks with the Mesh
 */
template <class T>
std::map<std::string, Eigen::Vector3<T>> TriangulateLandmarksViaRayCasting(const Camera<T>& camera, const LandmarkInstance<T, 2>& landmarkInstance, const Mesh<T>& mesh) {

    if (landmarkInstance.GetLandmarkConfiguration() == nullptr) {
        CARBON_CRITICAL("landmark configuration is not set for the landmark instance");
    }

    std::map<std::string, Eigen::Vector3<T>> reconstructedLandmarkPositions;

    for (const auto& [landmarkName, landmarkIndex] : landmarkInstance.GetLandmarkConfiguration()->LandmarkMapping()) {
        const Eigen::Vector3<T> origin = camera.Origin();
        const Eigen::Vector3<T> direction = camera.Unproject(landmarkInstance.Points().col(landmarkIndex), 1.0f, /*withExtrinsics=*/true) - origin;
        T bestAlpha = std::numeric_limits<T>::max();
        Eigen::Vector3<T> bestPos;
        for (int j = 0; j < mesh.NumTriangles(); ++j) {
            const Eigen::Vector3<T> v1 = mesh.Vertices().col(mesh.Triangles()(0, j));
            const Eigen::Vector3<T> v2 = mesh.Vertices().col(mesh.Triangles()(1, j));
            const Eigen::Vector3<T> v3 = mesh.Vertices().col(mesh.Triangles()(2, j));
            Eigen::Vector3<T> pos;
            T alpha;
            if (RayTriangleIntersection(origin, direction, v1, v2, v3, &alpha, &pos)) {
                if (alpha < bestAlpha) {
                    bestPos = pos;
                    bestAlpha = alpha;
                }
            }
        }
        if (bestAlpha < std::numeric_limits<T>::max()) {
            reconstructedLandmarkPositions[landmarkName] = bestPos;
        }
    }

    return reconstructedLandmarkPositions;
}

/**
 * Triangulate landmarks by looking up the position in a depthmap
 */
template <class T>
std::map<std::string, Eigen::Vector3<T>> TriangulateLandmarksViaDepthmap(
    const Camera<T>& camera,
    const LandmarkInstance<T, 2>& landmarkInstance,
    const Camera<T>& depthmapCamera,
    const Eigen::Matrix<T, 4, -1>& depthAndNormals
) {

    if (landmarkInstance.GetLandmarkConfiguration() == nullptr) {
        CARBON_CRITICAL("landmark configuration is not set for the landmark instance");
    }

    if ( (camera.Origin() - depthmapCamera.Origin()).norm() > 1e-6) {
        CARBON_CRITICAL("camera and depthmap camera are not at the same position");
    }

    std::map<std::string, Eigen::Vector3<T>> reconstructedLandmarkPositions;

    for (const auto& [landmarkName, landmarkIndex] : landmarkInstance.GetLandmarkConfiguration()->LandmarkMapping()) {
        const Eigen::Vector2<T> pix = depthmapCamera.Project(camera.Unproject(landmarkInstance.Points().col(landmarkIndex), 1.0f, /*withExtrinsics=*/true), /*withExtrinsics=*/true);
        const int ix = static_cast<int>(pix[0]);
        const int iy = static_cast<int>(pix[1]);
        if (ix >= 0 && ix < depthmapCamera.Width() && iy >= 0 && iy < depthmapCamera.Height()) {
            const T depth = depthAndNormals(0, iy * depthmapCamera.Width() + ix);
            if (depth > 0) {
                reconstructedLandmarkPositions.emplace(landmarkName, depthmapCamera.Unproject(pix, depth, /*withExtrinsics=*/true));
            }
        }
    }

    return reconstructedLandmarkPositions;
}

} // namespace epic::nls
