// Copyright Epic Games, Inc. All Rights Reserved.

#include "ActorCreationUtils.h"

#include <carbon/Common.h>
#include <nls/geometry/MetaShapeCamera.h>
#include <pma/PolyAllocator.h>
#include <tracking/LandmarkTriangulation.h>
#include <nls/geometry/Procrustes.h>

#include <cstring>
#include <filesystem>
#include <map>

using namespace epic::nls;
using namespace pma;


namespace titan::api {

epic::nls::MultiCameraSetup<float> ScaledCameras(const epic::nls::MultiCameraSetup<float>& cameras, float scale) {

    epic::nls::MultiCameraSetup<float> outputCameras;
    std::vector<MetaShapeCamera<float>> currentCameras = cameras.GetCamerasAsVector();

    for (auto& cam : currentCameras) {
        // scale from meter to cm
        Affine<float, 3, 3> extrinsics = cam.Extrinsics();
        extrinsics.SetTranslation(extrinsics.Translation() * scale);
        cam.SetExtrinsics(extrinsics);
    }
    outputCameras.Init(currentCameras);

    return outputCameras;
}

std::map<std::string, Eigen::Vector3f> TriangulateLandmarks(const std::shared_ptr<FrameInputData>& frameData, const epic::nls::MultiCameraSetup<float> &cameras) {
    std::map<std::string, Eigen::Vector3f> output;

    if (!frameData->Scan()) {
        CARBON_CRITICAL("Failed to triangulate landmarks. No scan in frame data.");
    }

    const auto& [cameraName, landmarkInstancePtr] = *(frameData->LandmarksPerCamera().begin());
    output = TriangulateLandmarksViaRayCasting(cameras.GetCamera(cameraName), *landmarkInstancePtr, *frameData->Scan());

    return output;
}

std::pair<float, epic::nls::Affine<float, 3, 3>> EstimateRigidAndScaleUsingLandmarks(const std::shared_ptr<FrameInputData>& frameData,
                                                                                     const epic::nls::MultiCameraSetup<float>& cameras,
                                                                                     std::map<std::string, Eigen::Vector3f> meshPositions) {
    const std::map<std::string, Eigen::Vector3f> reconstructedLandmarkPositions = TriangulateLandmarks(frameData, cameras);

    std::vector<Eigen::Vector3f> srcPts;
    std::vector<Eigen::Vector3f> targetPts;
    for (const auto& [landmarkName, srcPt] : meshPositions) {
        auto targetIt = reconstructedLandmarkPositions.find(landmarkName);
        if (targetIt != reconstructedLandmarkPositions.end()) {
            srcPts.push_back(srcPt);
            targetPts.push_back(targetIt->second);
        }
    }
    if (srcPts.size() < 3) {
        CARBON_CRITICAL("cannot estimate the rigid alignment as there are not sufficient triangulated landmarks (only {})", srcPts.size());
    }

    Eigen::Matrix<float, 3, -1> srcPtsMap = Eigen::Map<const Eigen::Matrix3Xf>((const float*)srcPts.data(), 3, srcPts.size());
    Eigen::Matrix<float, 3, -1> targetPtsMap = Eigen::Map<const Eigen::Matrix3Xf>((const float*)targetPts.data(), 3, targetPts.size());
    return epic::nls::Procrustes<float, 3>::AlignRigidAndScale(srcPtsMap, targetPtsMap);
}


std::shared_ptr<const LandmarkInstance<float, 2> > CreateLandmarkInstanceForCamera(const std::map<std::string, FaceTrackingLandmarkData>& perCameraLandmarkData,
                                                                                   const std::map<std::string, std::vector<std::string> >& curvesToMerge,
                                                                                   const MetaShapeCamera<float>& camera) {

    std::shared_ptr<LandmarkConfiguration> landmarkConfiguration = std::allocate_shared<LandmarkConfiguration>(PolyAllocator<LandmarkConfiguration>(
                                                                                                                   MEM_RESOURCE));
    for (const auto& [landmarkOrCurveName, faceTrackingLandmarkData] : perCameraLandmarkData) {
        if (faceTrackingLandmarkData.NumPoints() == 1) {
            landmarkConfiguration->AddLandmark(landmarkOrCurveName);
        } else if (faceTrackingLandmarkData.NumPoints() > 1) {
            landmarkConfiguration->AddCurve(landmarkOrCurveName, faceTrackingLandmarkData.NumPoints());
        } else {
            CARBON_CRITICAL("at least one point per landmark/curve required");
        }
    }

    Eigen::Matrix<float, 2, -1> landmarks(2, landmarkConfiguration->NumPoints());
    Eigen::Vector<float, -1> confidence(landmarkConfiguration->NumPoints());
    for (const auto& [landmarkOrCurveName, faceTrackingLandmarkData] : perCameraLandmarkData) {
        if (faceTrackingLandmarkData.PointsDimension() != 2) {
            CARBON_CRITICAL("input landmark data is not in 2D.");
        }
        if (faceTrackingLandmarkData.NumPoints() == 1) {
            const int index = landmarkConfiguration->IndexForLandmark(landmarkOrCurveName);
            for (int d = 0; d < 2; ++d) {
                landmarks(d, index) = faceTrackingLandmarkData.PointsData()[d];
            }
            confidence[index] = faceTrackingLandmarkData.ConfidenceData()[0];
        } else {
            const std::vector<int>& indices = landmarkConfiguration->IndicesForCurve(landmarkOrCurveName);
            for (int32_t i = 0; i < faceTrackingLandmarkData.NumPoints(); ++i) {
                const int index = indices[i];
                for (int d = 0; d < 2; ++d) {
                    landmarks(d, index) = faceTrackingLandmarkData.PointsData()[2 * i + d];
                }
                confidence[index] = faceTrackingLandmarkData.ConfidenceData()[i];
            }
        }
    }
    for (const auto& [mergedCurve, listOfCurves] : curvesToMerge) {
        landmarkConfiguration->MergeCurves(listOfCurves, mergedCurve, landmarks,  /*ignoreMissingCurves=*/ true);
    }

    std::shared_ptr<LandmarkInstance<float, 2> > landmarkInstance = std::allocate_shared<LandmarkInstance<float, 2> >(PolyAllocator<LandmarkInstance<float, 2> >(
                                                                                                                      MEM_RESOURCE),
                                                                                                                      landmarks,
                                                                                                                      confidence);
    landmarkInstance->SetLandmarkConfiguration(landmarkConfiguration);
    for (int i = 0; i < landmarkInstance->NumLandmarks(); ++i) {
        const Eigen::Vector2f pix = camera.Undistort(landmarkInstance->Points().col(i));
        landmarkInstance->SetLandmark(i,
                                      pix,
                                      landmarkInstance->Confidence()[i]);
    }
    return landmarkInstance;
}

std::shared_ptr<const epic::nls::LandmarkInstance<float, 3> > Create3dLandmarkInstance(const std::map<std::string, const FaceTrackingLandmarkData>& landmarkData,
                                                                                       const std::map<std::string, std::vector<std::string> >& curvesToMerge) {
    if (landmarkData.empty()) {
        return std::shared_ptr<const epic::nls::LandmarkInstance<float, 3> >{};
    }

    std::shared_ptr<LandmarkConfiguration> landmarkConfiguration = std::allocate_shared<LandmarkConfiguration>(PolyAllocator<LandmarkConfiguration>(
        MEM_RESOURCE));
    for (const auto& [landmarkOrCurveName, faceTrackingLandmarkData] : landmarkData) {
        if (faceTrackingLandmarkData.NumPoints() == 1) {
            landmarkConfiguration->AddLandmark(landmarkOrCurveName);
        }
        else if (faceTrackingLandmarkData.NumPoints() > 1) {
            landmarkConfiguration->AddCurve(landmarkOrCurveName, faceTrackingLandmarkData.NumPoints());
        }
        else {
            CARBON_CRITICAL("at least one point per landmark/curve required");
        }
    }

    Eigen::Matrix<float, 3, -1> landmarks(3, landmarkConfiguration->NumPoints());
    Eigen::Vector<float, -1> confidence(landmarkConfiguration->NumPoints());
    for (const auto& [landmarkOrCurveName, faceTrackingLandmarkData] : landmarkData) {
        if (faceTrackingLandmarkData.PointsDimension() != 3) {
            CARBON_CRITICAL("input landmark data is not in 3D.");
        }
        if (faceTrackingLandmarkData.NumPoints() == 1) {
            const int index = landmarkConfiguration->IndexForLandmark(landmarkOrCurveName);
            landmarks(0, index) = faceTrackingLandmarkData.PointsData()[0];
            landmarks(1, index) = faceTrackingLandmarkData.PointsData()[1];
            landmarks(2, index) = faceTrackingLandmarkData.PointsData()[2];
            confidence[index] = faceTrackingLandmarkData.ConfidenceData()[0];
        }
        else {
            const std::vector<int>& indices = landmarkConfiguration->IndicesForCurve(landmarkOrCurveName);
            for (int32_t i = 0; i < faceTrackingLandmarkData.NumPoints(); ++i) {
                const int index = indices[i];
                landmarks(0, index) = faceTrackingLandmarkData.PointsData()[3 * i + 0];
                landmarks(1, index) = faceTrackingLandmarkData.PointsData()[3 * i + 1];
                landmarks(2, index) = faceTrackingLandmarkData.PointsData()[3 * i + 2];
                confidence[index] = faceTrackingLandmarkData.ConfidenceData()[i];
            }
        }
    }
    for (const auto& [mergedCurve, listOfCurves] : curvesToMerge) {
        landmarkConfiguration->MergeCurves(listOfCurves, mergedCurve, landmarks,  /*ignoreMissingCurves=*/ true);
    }

    std::shared_ptr<LandmarkInstance<float, 3> > landmarkInstance = std::allocate_shared<LandmarkInstance<float, 3> >(PolyAllocator<LandmarkInstance<float, 3> >(
                                                                                                                      MEM_RESOURCE),
                                                                                                                      landmarks,
                                                                                                                      confidence);

    landmarkInstance->SetLandmarkConfiguration(landmarkConfiguration);

    return landmarkInstance;
}

std::shared_ptr<const Mesh<float> > ConstructMesh(const MeshInputData& InScanData) {
    auto [numTriangles, triangles, numVertices, vertices] = InScanData;

    Eigen::Matrix3Xf verticesMap = Eigen::Map<const Eigen::Matrix<float, 3, -1, Eigen::ColMajor> >(
        (const float*)vertices,
        3,
        numVertices);

    Eigen::Matrix3Xi trianglesMap = Eigen::Map<const Eigen::Matrix<int, 3, -1, Eigen::ColMajor> >(
        (const int*)triangles,
        3,
        numTriangles);

    // verify that all vertices are valid
    int numInvalidVertices = 0;
    for (int i = 0; i < 3 * numVertices; ++i) {
        if (!std::isfinite(vertices[i])) {
            numInvalidVertices++;
        }
    }
    if (numInvalidVertices > 0) {
        CARBON_CRITICAL("mesh contains {} vertices", numInvalidVertices);
    }

    // verify that all triangles index into valid vertices
    int numInvalidTriangles = 0;
    for (int i = 0; i < 3 * numTriangles; ++i) {
        if (triangles[i] < 0 || triangles[i] >= numVertices) {
            numInvalidTriangles++;
        }
    }
    if (numInvalidTriangles > 0) {
        CARBON_CRITICAL("mesh contains triangles with invalid vertex IDs (total {} invalid vertex IDs", numInvalidTriangles);
    }

    Mesh<float> scanMesh;
    scanMesh.SetVertices(verticesMap);
    scanMesh.SetTriangles(trianglesMap);
    scanMesh.CalculateVertexNormals();

    std::shared_ptr<const Mesh<float> > outputMeshPtr = std::allocate_shared<const Mesh<float> >(PolyAllocator<const Mesh<float> >(
                                                                                                     MEM_RESOURCE),
                                                                                                 scanMesh);

    return outputMeshPtr;
}

std::shared_ptr<const DepthmapData<float> > ConstructDepthmapData(std::string cameraName,
                                                                  const float* depthMaps,
                                                                  const MultiCameraSetup<float>& cameraSetup) {
    CARBON_PRECONDITION(cameraSetup.HasCamera(cameraName), "no camera {}", cameraName);

    PolyAllocator<DepthmapData<float> > depthPolyAllocator{MEM_RESOURCE};

    auto pixToPosition = [](int px, int py, float depth, float fx, float fy, float skew, float cx, float cy)
        {
            const float y = (static_cast<float>(py) + 0.5f - cy) / fy;
            const float x = (static_cast<float>(px) + 0.5f - cx - y * skew) / fx;
            return Eigen::Vector3f(x * depth, y * depth, depth);
        };

    std::shared_ptr<DepthmapData<float> > depthmapData = std::allocate_shared<DepthmapData<float> >(depthPolyAllocator);
    const MetaShapeCamera<float>& camera = cameraSetup.GetCamera(cameraName);

    depthmapData->camera = camera;
    depthmapData->depthAndNormals = Eigen::Matrix<float, 4, -1>(4, camera.Width() * camera.Height());

    const float fx = camera.Intrinsics()(0, 0);
    const float fy = camera.Intrinsics()(1, 1);
    const float skew = camera.Intrinsics()(0, 1);
    const float cx = camera.Intrinsics()(0, 2);
    const float cy = camera.Intrinsics()(1, 2);

    for (int y = 1; y < camera.Height() - 1; ++y) {
        const int index = y * camera.Width();
        for (int x = 1; x < camera.Width() - 1; ++x) {
            depthmapData->depthAndNormals(0, index + x) = depthMaps[index + x];

            const Eigen::Vector3f pos = pixToPosition(x, y, depthMaps[index + x], fx, fy, skew, cx, cy);
            const float depthXM = depthMaps[index + x - 1];
            const Eigen::Vector3f posXM = pixToPosition(x - 1, y, depthXM, fx, fy, skew, cx, cy);
            const float depthXP = depthMaps[index + x + 1];
            const Eigen::Vector3f posXP = pixToPosition(x + 1, y, depthXP, fx, fy, skew, cx, cy);
            const float depthYM = depthMaps[index - camera.Width() + x];
            const Eigen::Vector3f posYM = pixToPosition(x, y - 1, depthYM, fx, fy, skew, cx, cy);
            const float depthYP = depthMaps[index + camera.Width() + x];
            const Eigen::Vector3f posYP = pixToPosition(x, y + 1, depthYP, fx, fy, skew, cx, cy);

            Eigen::Vector3f normal = Eigen::Vector3f::Zero();
            if ((depthXP > 0) && (depthYM > 0)) {
                normal += (posXP - pos).cross(posYM - pos);
            }
            if ((depthYM > 0) && (depthXM > 0)) {
                normal += (posYM - pos).cross(posXM - pos);
            }
            if ((depthXM > 0) && (depthYP > 0)) {
                normal += (posXM - pos).cross(posYP - pos);
            }
            if ((depthYP > 0) && (depthXP > 0)) {
                normal += (posYP - pos).cross(posXP - pos);
            }
            if (normal.squaredNorm() > 0) {
                normal.normalize();
            }
            depthmapData->depthAndNormals(1, y * camera.Width() + x) = normal[0];
            depthmapData->depthAndNormals(2, y * camera.Width() + x) = normal[1];
            depthmapData->depthAndNormals(3, y * camera.Width() + x) = normal[2];
        }
    }

    return depthmapData;
}

std::pair<std::vector<Eigen::VectorX<float> >, std::vector<std::shared_ptr<const Mesh<float> > > > PrepareMeshes(const std::map<int,std::shared_ptr<FrameInputData> >& frameData, std::map<int, float> scale)
{
    std::vector<Eigen::VectorX<float> > weights;
    std::vector<std::shared_ptr<const Mesh<float> > > meshes;

    for (auto& [frameNum, frame] : frameData) {
        std::shared_ptr<Mesh<float>> currentScan = std::make_shared<Mesh<float>>(*frame->Scan());
        if (scale.size() > 0) {
            currentScan->SetVertices(scale[frameNum] * currentScan->Vertices());
            currentScan->CalculateVertexNormals();
        }

        meshes.emplace_back(currentScan);
        Eigen::VectorXf scanMask = Eigen::VectorXf::Ones(frame->Scan()->NumVertices());
        const std::vector<int> borderVertices = frame->Scan()->CalculateBorderVertices();
        for (int vID : borderVertices) {
            scanMask[vID] = 0.0f;
        }
        // vertices that have zero vertex normals (either as they are not part of triangles, or the triangles have zero area)
        // should also not be used
        int outlierCounter = 0;
        for (int i = 0; i < frame->Scan()->NumVertices(); ++i) {
            if (frame->Scan()->VertexNormals().col(i).squaredNorm() < 0.05f) {
                scanMask[i] = 0.0f;
                outlierCounter++;
            }
        }
        if (borderVertices.size() == frame->Scan()->NumVertices() || outlierCounter == frame->Scan()->NumVertices()) {
            CARBON_CRITICAL("Bad input data: All vertices on the input mesh marked as invalid. Please check input mesh topology.");
        }
        weights.emplace_back(scanMask);
    }

    return std::pair<std::vector<Eigen::VectorX<float> >, std::vector<std::shared_ptr<const Mesh<float> > > >(weights,
                                                                                                              meshes);
}

std::vector<std::vector<std::shared_ptr<const DepthmapData<float> > > > PrepareDepths(const std::map<int, std::shared_ptr<FrameInputData> >& frameData)
{
    std::vector<std::vector<std::shared_ptr<const DepthmapData<float> > > > output;

    for (auto& [frameNum, frame] : frameData) {
        std::vector<std::shared_ptr<const DepthmapData<float> > > depthByCamera;
        for (const auto& [_, depthData] : frame->Depthmaps()) {
            depthByCamera.push_back(depthData);
        }
        output.emplace_back(depthByCamera);
    }

    return output;
}

std::vector<std::vector<std::pair<LandmarkInstance<float, 2>, Camera<float> > > > Prepare2DLandmarks(const std::map<int, std::shared_ptr<FrameInputData> >& frameData,
                                                                                                     const MultiCameraSetup<float>& cameras)
{
    std::vector<std::vector<std::pair<LandmarkInstance<float, 2>, Camera<float> > > > output;

    for (auto& [_, frame] : frameData) {
        std::vector<std::pair<LandmarkInstance<float, 2>, Camera<float> > > landmarks;
        for (const auto& [cameraName, landmarkInstance] : frame->LandmarksPerCamera()) {
            landmarks.emplace_back(std::pair<LandmarkInstance<float, 2>, Camera<float> >(*landmarkInstance,
                                                                                         cameras.GetCamera(cameraName)));
        }
        output.emplace_back(landmarks);
    }

    return output;
}

std::vector<std::vector<std::pair<LandmarkInstance<float, 2>, Camera<float> > > > Prepare2DLandmarks(const std::map<int, std::shared_ptr<FrameInputData> >& frameData,
                                                                                                     const std::map <int, MultiCameraSetup<float>>& camerasPerFrame)
{
    std::vector<std::vector<std::pair<LandmarkInstance<float, 2>, Camera<float> > > > output;

    for (auto& [frameNum, frame] : frameData) {

        auto it = camerasPerFrame.find(frameNum);
        if (it == camerasPerFrame.end()) {
            CARBON_CRITICAL("No cameras for the frame num {}", frameNum);
        }
        MultiCameraSetup<float> cameras = it->second;

        std::vector<std::pair<LandmarkInstance<float, 2>, Camera<float> > > landmarks;
        for (const auto& [cameraName, landmarkInstance] : frame->LandmarksPerCamera()) {
            landmarks.emplace_back(std::pair<LandmarkInstance<float, 2>, Camera<float> >(*landmarkInstance,
                                                                                         cameras.GetCamera(cameraName)));
        }
        output.emplace_back(landmarks);
    }

    return output;
}

std::vector<LandmarkInstance<float, 3>> Prepare3DLandmarks(const std::map<int, std::shared_ptr<FrameInputData> >& frameData)
{
    std::vector<LandmarkInstance<float, 3>> output;
    for (auto& [frameNum, frame] : frameData) {
        auto landmarks = frame->Landmarks3D();
        if (!landmarks) {
            continue;
        }
        output.emplace_back(*landmarks);
    }

    return output;
}

}
