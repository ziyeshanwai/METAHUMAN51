// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/Camera.h>
#include <nls/geometry/Interpolation.h>
#include <nls/geometry/MetaShapeCamera.h>

namespace epic::nls {


template <class T>
bool IsRectification(const Camera<T>& cameraSrc, const Camera<T>& cameraTarget)
{
  Eigen::Matrix<T, 3, 3> rotPrev = cameraSrc.Extrinsics().Linear();
  Eigen::Matrix<T, 3, 3> rotNew = cameraTarget.Extrinsics().Linear();
  Eigen::Matrix<T, 3, 3> rectRotation = rotNew * rotPrev.transpose();
  Eigen::Vector3<T> tPrev = cameraSrc.Extrinsics().Translation();
  Eigen::Vector3<T> tNew = cameraTarget.Extrinsics().Translation();
  Eigen::Vector3<T> tNewCalculated = rectRotation * tPrev;
  T norm = (tNewCalculated - tNew).norm();
  return (norm <= 1e-3);
}


/**
 * Rectifies the cameras. Outputs the minimum negative disparity between left and right i.e.
 * minimum disparity = 0 (looking at maximum range)
 * maximum disparity = -value (looking at minimum range)
 *
 * @param keepImageDimensions  If True, then the new cameras will have the same width and height as the left input camera.
 *                             If False, the camera dimensions are adapted to fit the rectified images optimally.
 */
template <class T>
T RectifyCameras(
  const Camera<T>& cameraLeft,
  const Camera<T>& cameraRight,
  const std::pair<T, T>& rangeLeft,
  const std::pair<T, T>& rangeRight,
  Camera<T>& newCameraLeft,
  Camera<T>& newCameraRight,
  bool keepImageDimensions
)
{
  const Eigen::Vector3<T> cameraPositionLeftWorld = cameraLeft.Origin();
  const Eigen::Vector3<T> cameraNearRangePositionLeftWorld = cameraLeft.Unproject(Eigen::Vector2<T>(T(cameraLeft.Width())/T(2), T(cameraLeft.Height())/T(2)), rangeLeft.first, true);
  const Eigen::Vector3<T> cameraFarRangePositionLeftWorld = cameraLeft.Unproject(Eigen::Vector2<T>(T(cameraLeft.Width())/T(2), T(cameraLeft.Height())/T(2)), rangeLeft.second, true);
  // const Eigen::Vector3<T> cameraLeftDir = (cameraFarRangePositionLeftWorld - cameraPositionLeftWorld).normalized();
  const Eigen::Vector3<T> cameraPositionRightWorld = cameraRight.Origin();
  const Eigen::Vector3<T> cameraNearRangePositionRightWorld = cameraRight.Unproject(Eigen::Vector2<T>(T(cameraRight.Width())/T(2), T(cameraRight.Height())/T(2)), rangeRight.first, true);
  const Eigen::Vector3<T> cameraFarRangePositionRightWorld = cameraRight.Unproject(Eigen::Vector2<T>(T(cameraRight.Width())/T(2), T(cameraRight.Height())/T(2)), rangeRight.second, true);
  // const Eigen::Vector3<T> cameraRightDir = (cameraFarRangePositionRightWorld - cameraPositionRightWorld).normalized();
  const Eigen::Vector3<T> cameraMidPointWorld = (cameraPositionLeftWorld + cameraPositionRightWorld) / T(2);
  const Eigen::Vector3<T> nearRangePosition = (cameraNearRangePositionLeftWorld + cameraNearRangePositionRightWorld) / T(2);
  const Eigen::Vector3<T> farRangePosition = (cameraFarRangePositionLeftWorld + cameraFarRangePositionRightWorld) / T(2);
  Eigen::Vector3<T> zDir = (farRangePosition - cameraMidPointWorld).normalized();
  const Eigen::Vector3<T> xDir = (cameraPositionRightWorld - cameraPositionLeftWorld).normalized();
  const Eigen::Vector3<T> yDir = zDir.cross(xDir).normalized();
  zDir = xDir.cross(yDir);
  Eigen::Matrix3<T> rectificationRotation;
  rectificationRotation.row(0) = xDir;
  rectificationRotation.row(1) = yDir;
  rectificationRotation.row(2) = zDir;
  Eigen::Matrix<T, 3, 3> rectificationRotationLeft = rectificationRotation * cameraLeft.Extrinsics().Linear().transpose();
  Eigen::Matrix<T, 3, 3> rectificationRotationRight = rectificationRotation * cameraRight.Extrinsics().Linear().transpose();
  const T f = T(0.25) * (cameraLeft.Intrinsics()(0,0) + cameraLeft.Intrinsics()(1, 1) +
                             cameraRight.Intrinsics()(0,0) + cameraRight.Intrinsics()(1, 1));

  Eigen::Matrix<T, 3, 3> initialIntrinsics = Eigen::Matrix<T, 3, 3>::Identity();
  initialIntrinsics(0, 0) = f;
  initialIntrinsics(1, 1) = f;
  newCameraLeft.SetIntrinsics(initialIntrinsics);
  newCameraRight.SetIntrinsics(initialIntrinsics);
  Affine<T, 3, 3> rectificationAffineLeft;
  rectificationAffineLeft.SetLinear(rectificationRotationLeft);
  Affine<T, 3, 3> rectificationAffineRight;
  rectificationAffineRight.SetLinear(rectificationRotationRight);
  newCameraLeft.SetExtrinsics(rectificationAffineLeft * cameraLeft.Extrinsics());
  newCameraRight.SetExtrinsics(rectificationAffineRight * cameraRight.Extrinsics());

  // maximum range needs to project to the same location on both images i.e. disparity = 0
  Eigen::Matrix<T, 3, 3> newIntrinsicsLeft = initialIntrinsics;
  Eigen::Matrix<T, 3, 3> newIntrinsicsRight = initialIntrinsics;
  const Eigen::Vector2<T> pixLeftMaxZ = newCameraLeft.Project(farRangePosition, /*withExtrinsics=*/true);
  const Eigen::Vector2<T> pixRightMaxZ = newCameraRight.Project(farRangePosition, /*withExtrinsics=*/true);
  newIntrinsicsLeft(1,2) = cameraLeft.Height() / T(2);
  newIntrinsicsLeft(0,2) = cameraLeft.Width() / T(2) - pixLeftMaxZ[0];
  newIntrinsicsRight(1,2) = cameraLeft.Height() / T(2);
  newIntrinsicsRight(0,2) = cameraLeft.Width() / T(2) - pixRightMaxZ[0];

  newCameraLeft.SetIntrinsics(newIntrinsicsLeft);
  newCameraRight.SetIntrinsics(newIntrinsicsRight);

  newCameraLeft.SetWidth(cameraLeft.Width());
  newCameraLeft.SetHeight(cameraLeft.Height());
  newCameraRight.SetWidth(cameraLeft.Width());
  newCameraRight.SetHeight(cameraLeft.Height());

  // homography from old camera to new camera, fix so that previous image centers project to the image center of the new camera
  auto applyHomography = [](const Eigen::Matrix<T, 3, 3>& H, const Eigen::Vector2<T>& pix) {
    Eigen::Vector3<T> pixNew = H * Eigen::Vector3<T>(pix[0], pix[1], 1);
    pixNew /= pixNew[2];
    return Eigen::Vector2<T>(pixNew.template head<2>());
  };
  Eigen::Matrix<T, 3, 3> Hleft = newCameraLeft.Intrinsics() * newCameraLeft.Extrinsics().Linear() * cameraLeft.Extrinsics().Linear().transpose() * cameraLeft.Intrinsics().inverse();
  Eigen::Matrix<T, 3, 3> Hright = newCameraRight.Intrinsics() * newCameraRight.Extrinsics().Linear() * cameraRight.Extrinsics().Linear().transpose() * cameraRight.Intrinsics().inverse();
  Eigen::Vector2<T> posLeft = applyHomography(Hleft, Eigen::Vector2<T>(cameraLeft.Width()/2, cameraLeft.Height()/2));
  Eigen::Vector2<T> posRight = applyHomography(Hright, Eigen::Vector2<T>(cameraRight.Width()/2, cameraRight.Height()/2));
  T recenterShiftX = T(0.5) * (newCameraLeft.Width()/2 - posLeft[0] + newCameraRight.Width()/2 - posRight[0]);
  newIntrinsicsLeft(0, 2) += recenterShiftX;
  newIntrinsicsRight(0, 2) += recenterShiftX;
  T recenterShiftY = T(0.5) * (newCameraLeft.Height()/2 - posLeft[1] + newCameraRight.Height()/2 - posRight[1]);
  newIntrinsicsLeft(1, 2) += recenterShiftY;
  newIntrinsicsRight(1, 2) += recenterShiftY;
  newCameraLeft.SetIntrinsics(newIntrinsicsLeft);
  newCameraRight.SetIntrinsics(newIntrinsicsRight);

  if (!keepImageDimensions) {
    // recalculate image crop
    Hleft = newCameraLeft.Intrinsics() * newCameraLeft.Extrinsics().Linear() * cameraLeft.Extrinsics().Linear().transpose() * cameraLeft.Intrinsics().inverse();
    Hright = newCameraRight.Intrinsics() * newCameraRight.Extrinsics().Linear() * cameraRight.Extrinsics().Linear().transpose() * cameraRight.Intrinsics().inverse();
    Eigen::Matrix<T, 2, -1> boundingPoints(2, 8);
    boundingPoints.col(0) = applyHomography(Hleft, Eigen::Vector2<T>(cameraLeft.Width()/2, 0));
    boundingPoints.col(1) = applyHomography(Hright, Eigen::Vector2<T>(cameraRight.Width()/2, 0));
    boundingPoints.col(2) = applyHomography(Hleft, Eigen::Vector2<T>(cameraLeft.Width()/2, cameraLeft.Height()));
    boundingPoints.col(3) = applyHomography(Hright, Eigen::Vector2<T>(cameraRight.Width()/2, cameraRight.Height()));
    boundingPoints.col(4) = applyHomography(Hleft, Eigen::Vector2<T>(0, cameraLeft.Height()/2));
    boundingPoints.col(5) = applyHomography(Hright, Eigen::Vector2<T>(0, cameraRight.Height()/2));
    boundingPoints.col(6) = applyHomography(Hleft, Eigen::Vector2<T>(cameraLeft.Width(), cameraLeft.Height()/2));
    boundingPoints.col(7) = applyHomography(Hright, Eigen::Vector2<T>(cameraRight.Width(), cameraRight.Height()/2));
    const T minx = boundingPoints.row(0).minCoeff();
    const T maxx = boundingPoints.row(0).maxCoeff() - cameraLeft.Width();
    const T miny = boundingPoints.row(1).minCoeff();
    const T maxy = boundingPoints.row(1).maxCoeff() - cameraLeft.Height();
    const int deltaWidth = int(std::ceil(-minx + maxx));
    const int deltaHeight = int(std::ceil(-miny + maxy));
    const int shiftX = int(std::ceil(-minx));
    const int shiftY = int(std::ceil(-miny));

    newCameraLeft.SetWidth(newCameraLeft.Width() + deltaWidth);
    newCameraRight.SetWidth(newCameraRight.Width() + deltaWidth);
    newCameraLeft.SetHeight(newCameraLeft.Height() + deltaHeight);
    newCameraRight.SetHeight(newCameraRight.Height() + deltaHeight);
    newIntrinsicsLeft(0,2) += shiftX;
    newIntrinsicsLeft(1,2) += shiftY;
    newIntrinsicsRight(0,2) += shiftX;
    newIntrinsicsRight(1,2) += shiftY;
    newCameraLeft.SetIntrinsics(newIntrinsicsLeft);
    newCameraRight.SetIntrinsics(newIntrinsicsRight);
  }

  // calcuate maximum disparity between the images based on the minimum range
  const Eigen::Vector2<T> newPixLeftMinZ = newCameraLeft.Project(nearRangePosition, /*withExtrinsics=*/true);
  const Eigen::Vector2<T> newPixRightMinZ = newCameraRight.Project(nearRangePosition, /*withExtrinsics=*/true);
  T maximumDisparity = newPixLeftMinZ[0] - newPixRightMinZ[0];

  return maximumDisparity;
}


template <class T>
void RectifyAndUndistort(
  const MetaShapeCamera<T>& camera,
  const Camera<T>& rectifiedCamera,
  const T* input,
  T* output,
  InterpolationMethod interpolationMethod
)
{
  // homography from new camera to old camera
  Eigen::Matrix<T, 3, 3> H = camera.Intrinsics() * camera.Extrinsics().Linear() * rectifiedCamera.Extrinsics().Linear().transpose() * rectifiedCamera.Intrinsics().inverse();

  auto outputPixelToInputPixel = [&](T x, T y) {
    Eigen::Vector3<T> outputPixel(x, y, T(1));
    Eigen::Vector3<T> inputPixelUndistorted = H * outputPixel;
    inputPixelUndistorted /= inputPixelUndistorted[2];
    return camera.Distort(inputPixelUndistorted.template head<2>());
  };

  for (int y = 0; y < rectifiedCamera.Height(); y++) {
    for (int x = 0; x < rectifiedCamera.Width(); x++) {
      // calculate the transform in the center of the pixel
      Eigen::Vector2<T> inputPixel = outputPixelToInputPixel(x + T(0.5), y + T(0.5));
      // transform to opencv coordinate system
      T px = inputPixel[0] - T(0.5);
      T py = inputPixel[1] - T(0.5);
      switch (interpolationMethod) {
          case InterpolationMethod::NEAREST: {
              output[y * rectifiedCamera.Width() + x] = nearestInterpolate<float>(input, px, py, camera.Width() - 1, camera.Height() - 1);
          } break;
          case InterpolationMethod::LINEAR: {
              output[y * rectifiedCamera.Width() + x] = bilinearInterpolate<float>(input, px, py, camera.Width() - 1, camera.Height() - 1);
          } break;
          case InterpolationMethod::CUBIC: {
              output[y * rectifiedCamera.Width() + x] = cubicInterpolate<float>(input, px, py, camera.Width() - 1, camera.Height() - 1);
          } break;
      }
    }
  }
}


} // namespace epic::nls
