// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <MeshInputData.h>
#include "FrameInputData.h"
#include <LandmarkData.h>
#include <reconstruction/MultiCameraSetup.h>

#include <map>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

namespace titan {
namespace api {

epic::nls::MultiCameraSetup<float> ScaledCameras(const epic::nls::MultiCameraSetup<float>& cameras, float scale);

std::pair<float, epic::nls::Affine<float, 3, 3>> EstimateRigidAndScaleUsingLandmarks(const std::shared_ptr<FrameInputData>& frameData,
                                                                                     const epic::nls::MultiCameraSetup<float>& cameras,
                                                                                     std::map<std::string, Eigen::Vector3f> meshPositions);

std::shared_ptr<const epic::nls::LandmarkInstance<float, 2> > CreateLandmarkInstanceForCamera(const std::map<std::string, FaceTrackingLandmarkData>& perCameraLandmarkData,
                                                                                              const std::map<std::string, std::vector<std::string> >& curvesToMerge,
                                                                                              const epic::nls::MetaShapeCamera<float>& camera);

std::shared_ptr<const epic::nls::LandmarkInstance<float, 3> > Create3dLandmarkInstance(const std::map<std::string, const FaceTrackingLandmarkData>& landmarkData,
                                                                                       const std::map<std::string, std::vector<std::string> >& curvesToMerge);

std::shared_ptr<const epic::nls::Mesh<float> > ConstructMesh(const MeshInputData& InScanData);

std::shared_ptr<const epic::nls::DepthmapData<float> > ConstructDepthmapData(std::string cameraName,
                                                                             const float* depthMaps,
                                                                             const epic::nls::MultiCameraSetup<float>& cameraSetup);

std::pair<std::vector<Eigen::VectorX<float>>, std::vector<std::shared_ptr<const epic::nls::Mesh<float>>>> PrepareMeshes(const std::map<int, std::shared_ptr<FrameInputData> >& frameData,
                                                                                                                        std::map<int, float> scale = std::map<int, float>{});

std::vector<std::vector<std::shared_ptr<const epic::nls::DepthmapData<float>>>> PrepareDepths(const std::map<int, std::shared_ptr<FrameInputData> >& frameData);

std::vector<std::vector<std::pair<epic::nls::LandmarkInstance<float, 2>, epic::nls::Camera<float> > > > Prepare2DLandmarks(const std::map<int, std::shared_ptr<FrameInputData> >& frameData,
                                                                                                                           const epic::nls::MultiCameraSetup<float>& cameras);

std::vector<std::vector<std::pair<epic::nls::LandmarkInstance<float, 2>, epic::nls::Camera<float> > > > Prepare2DLandmarks(const std::map<int, std::shared_ptr<FrameInputData> >& frameData,
                                                                                                                           const std::map<int, epic::nls::MultiCameraSetup<float>>& camerasPerFrame);

std::vector<epic::nls::LandmarkInstance<float, 3>> Prepare3DLandmarks(const std::map<int, std::shared_ptr<FrameInputData> >& frameData);

}
}
