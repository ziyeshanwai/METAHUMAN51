// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/MetaShapeCamera.h>
#include <nls/geometry/MultiCameraTriangulation.h>

#include <reconstruction/StereoCameraPair.h>

#include <map>
#include <string>
#include <vector>

namespace epic::nls {

/**
 * Finds the optimal 3d position that minimizes the L2 distance to the image center of each camera.
 */
template <class T>
Eigen::Vector3<T> FindCenterOfCameras(const std::vector<MetaShapeCamera<T>>& cameras)
{
  MultiCameraTriangulation<T> multiCameraTriangulation;
  std::vector<Camera<T>> cameraVector;
  for (const auto& cam : cameras) {
    cameraVector.push_back(cam);
  }
  multiCameraTriangulation.SetCameras(cameraVector);
  std::vector<Eigen::Vector2<T>> imageCenters;
  std::vector<float> confidences;
  for (int i = 0; i < int(cameraVector.size()); ++i) {
    imageCenters.push_back(Eigen::Vector2<T>(T(cameraVector[i].Width())/T(2), T(cameraVector[i].Height())/T(2)));
    confidences.push_back(T(1));
  }
  Eigen::Vector3<T> pos = multiCameraTriangulation.Triangulate(imageCenters);
  pos = multiCameraTriangulation.TriangulateNonlinear(pos, imageCenters, confidences);
  return pos;
}


/**
 * Basic heuristic to find the depth range for each camera:
 * First it projects the center into each camera and calculates the bounding box at the center distance.
 * To calculate the range for each camera it projects the bound points into the each image and measures min and max distance.
 */
template <class T>
std::map<std::string, std::pair<T, T>> EstimateCameraRanges(const std::vector<MetaShapeCamera<T>>& cameras)
{
  const Eigen::Vector3<T> center = FindCenterOfCameras(cameras);

  std::vector<Eigen::Vector3<T>> boundPoints;
  boundPoints.reserve(cameras.size() * 4);
  for (int k = 0; k < int(cameras.size()); k++) {
    const auto& camera = cameras[k];
    const Eigen::Vector3<T> centerInCamera = camera.Extrinsics().Transform(center);
    const float depth = centerInCamera[2];
    boundPoints.push_back(camera.Unproject(Eigen::Vector2<T>(0, 0), depth, /*withExtrinsics=*/true));
    boundPoints.push_back(camera.Unproject(Eigen::Vector2<T>(camera.Width(), 0), depth, /*withExtrinsics=*/true));
    boundPoints.push_back(camera.Unproject(Eigen::Vector2<T>(0, camera.Height()), depth, /*withExtrinsics=*/true));
    boundPoints.push_back(camera.Unproject(Eigen::Vector2<T>(camera.Width(), camera.Height()), depth, /*withExtrinsics=*/true));
  }

  std::map<std::string, std::pair<T,T>> ranges;
  for (int k = 0; k < int(cameras.size()); k++) {
    std::pair<T, T> range(T(1e9), T(-1e9));
    for (const Eigen::Vector3<T>& pos : boundPoints) {
      const Eigen::Vector3<T> posInCamera = cameras[k].Extrinsics().Transform(pos);
      const T depth = posInCamera[2];
      range.first = std::min(range.first, depth);
      range.second = std::max(range.second, depth);
    }
    ranges[cameras[k].Label()] = range;
  }
  return ranges;
}

/**
 * Estimate the depth range for each camera by projecting the mesh into each image and measuring min and max value.
 */
template <class T>
std::map<std::string, std::pair<T, T>> EstimateCameraRangesByProjection(const std::vector<MetaShapeCamera<T>>& cameras, const Mesh<T>& mesh, const Affine<T, 3, 3>& mesh2cameras, const T rangeScaling = T(1))
{
  if (!mesh.HasVertexNormals()) {
    throw std::runtime_error("Projection-based range estimation requires normals to only use camera facing vertices in the estimation");
  }
  std::map<std::string, std::pair<T,T>> ranges;
  for (int k = 0; k < int(cameras.size()); k++) {
    std::pair<T, T> range(T(1e9), T(-1e9));
    for (int i = 0; i < mesh.NumVertices(); i++) {
      const Eigen::Vector3<T> normal = cameras[k].Extrinsics().Linear() * mesh2cameras.Linear() * mesh.VertexNormals().col(i);
      const Eigen::Vector3<T> vertex = cameras[k].Extrinsics().Transform(mesh2cameras.Transform(mesh.Vertices().col(i)));
      if (normal.dot(vertex) < 0) {
        const T depth = vertex[2];
        range.first = std::min(range.first, depth);
        range.second = std::max(range.second, depth);
      }
    }
    T deltaRange = (range.second - range.first) * (rangeScaling - T(1)) * T(0.5);
    range.first -= deltaRange;
    range.second += deltaRange;
    ranges[cameras[k].Label()] = range;
  }
  return ranges;
}


/**
 * @returns for each camera the sorted list of other cameras based on overlap (mapping direction towards center)
 */
template <class T, typename CameraType>
std::vector<std::vector<std::pair<int, T>>> CameraNeighbors(const std::vector<CameraType>& cameras, Eigen::Vector3<T> center)
{

  std::vector<std::vector<std::pair<int, T>>> allNeighbors;

  for (size_t i = 0; i < cameras.size(); ++i) {
    const CameraType& camI = cameras[i];
    const Eigen::Vector3<T> dirI = (center - camI.Extrinsics().Inverse().Translation()).normalized();
    std::vector<std::pair<int, T>> neighbors;
    for (size_t j = 0; j < cameras.size(); ++j) {
      if (i == j) continue;
      const CameraType& camJ = cameras[j];
      const Eigen::Vector3<T> dirJ = (center - camJ.Extrinsics().Inverse().Translation()).normalized();
      neighbors.push_back(std::pair<int, T>(int(j), dirJ.dot(dirI)));
    }
    std::sort(neighbors.begin(), neighbors.end(), [](const std::pair<int, T>& a, const std::pair<int, T>&b) { return a.second > b.second; });
    allNeighbors.push_back(neighbors);
  }

  return allNeighbors;
}

} // namespace epic::nls
