// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoReconstruction.h"
#include "Common.h"
#include "Internals/StereoReconstructionHelper.h"

#include <nls/geometry/MetaShapeCamera.h>
#include <nls/serialization/CameraSerialization.h>
#include <nls/vulkan/common/VulkanInstance.h>
#include <nls/vulkan/common/VulkanInstance.h>
#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <reconstruction/vulkan/ImagePreprocessorVulkan.h>
#include <reconstruction/vulkan/StereoReconstructionVulkan.h>

#include <cstring>
#include <map>
#include <vector>

using namespace epic::nls;


namespace titan {
namespace api {

struct StereoReconstruction::Private {
    MultiCameraSetup<float> cameraSetup;
    StereoReconstructionHelper stereoReconstructionHelper;

    std::shared_ptr<const DepthmapData<float>> depthData;

    // std::map<std::string, MetaShapeCamera<float>> cameras;
    // std::map<std::string, std::pair<float, float>> ranges;

    // std::unique_ptr<ImagePreprocessorVulkan> imagePreprocessorVulkan;
    // std::unique_ptr<StereoReconstructionVulkan> stereoReconstructionVulkan;

    // Camera<float> depthCamera;
    // std::vector<float> depthData;
    std::vector<int32_t> depthIndices;
    int numVertices;
    int numTriangles;
};

StereoReconstruction::StereoReconstruction()
    : m(new Private)
{

}

StereoReconstruction::~StereoReconstruction()
{
    if (m) {
        delete m;
        m = nullptr;
    }
}

bool StereoReconstruction::Init()
{
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(VulkanLoader::Init(), false, "failed to initialize vulkan");
        auto vulkanInstance = VulkanInstance::CreateInstance(false, {});
        CHECK_OR_RETURN(vulkanInstance, false, "could not create vulkan instance");
        auto vulkanDevice = VulkanDevice::CreateDevice(vulkanInstance, nullptr);
        CHECK_OR_RETURN(vulkanDevice, false, "could not create vulkan device");

        return m->stereoReconstructionHelper.Init(vulkanDevice);
    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to initialize: {}", e.what());
    }
}

bool StereoReconstruction::LoadCalibrationFromFile(const char* Filename)
{
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(Filename, false, "invalid filename");
        std::vector<MetaShapeCamera<float>> cameras;
        bool success = ReadMetaShapeCamerasFromJsonFile(Filename, cameras);
        if (success) {
            m->cameraSetup.Init(cameras);
            std::map<std::string, std::pair<float, float>> ranges;
            for (const auto& camera : cameras) {
                ranges[camera.Label()] = std::pair<float, float>(15.0f, 25.0f);
            }
            m->cameraSetup.SetCameraRanges(ranges);
        }

        return success;
    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to load calibration file: {}", e.what());
    }
}


bool StereoReconstruction::GetImageSize(const char* CameraName, int32_t& widthOut, int32_t& heightOut) const
{
    RESET_ERROR;
    CHECK_OR_RETURN(CameraName, false, "invalid camera name");
    CHECK_OR_RETURN(m->cameraSetup.HasCamera(CameraName), false, "camera {} does not exist", CameraName);
    const MetaShapeCamera<float>& camera = m->cameraSetup.GetCamera(CameraName);
    widthOut = static_cast<int32_t>(camera.Width());
    heightOut = static_cast<int32_t>(camera.Height());
    return true;
}

bool StereoReconstruction::SetCameraRange(const char* CameraName, float minDepth, float maxDepth)
{
    RESET_ERROR;
    CHECK_OR_RETURN(CameraName, false, "invalid camera name");
    CHECK_OR_RETURN(m->cameraSetup.HasCamera(CameraName), false, "camera {} does not exist", CameraName);
    m->cameraSetup.SetCameraRange(CameraName, std::pair<float, float>(minDepth, maxDepth));
    return true;
}

int32_t StereoReconstruction::ReconstructPointCloud(const char* CameraNameLeft,
                                                    const char* CameraNameRight,
                                                    const unsigned char* ImageDataLeft,
                                                    const unsigned char* ImageDataRight)
{
    try {
        RESET_ERROR;

        m->numVertices = 0;
        m->numTriangles = 0;
        m->depthData = nullptr;

        CHECK_OR_RETURN(CameraNameLeft, -1, "invalid camera name");
        CHECK_OR_RETURN(CameraNameRight, -1, "invalid camera name");
        CHECK_OR_RETURN(ImageDataLeft, -1, "invalid image data");
        CHECK_OR_RETURN(ImageDataRight, -1, "invalid image data");
        CHECK_OR_RETURN(m->stereoReconstructionHelper.IsInitialized(), -1, "stereo reconstruction is not initialized");
        CHECK_OR_RETURN(m->cameraSetup.HasCamera(CameraNameLeft), -1, "camera {} does not exist", CameraNameLeft);
        CHECK_OR_RETURN(m->cameraSetup.HasCamera(CameraNameRight), -1, "camera {} does not exist", CameraNameRight);

        const MetaShapeCamera<float>& cameraLeft = m->cameraSetup.GetCamera(CameraNameLeft);
        const MetaShapeCamera<float>& cameraRight = m->cameraSetup.GetCamera(CameraNameRight);

        ConstImageView imgViewLeft(ImageDataLeft, cameraLeft.Width(), cameraLeft.Height(), PixelFormat::RGBA, PixelDepth::UINT8);
        ConstImageView imgViewRight(ImageDataRight, cameraRight.Width(), cameraRight.Height(), PixelFormat::RGBA, PixelDepth::UINT8);
        m->stereoReconstructionHelper.SetCameraSetup(m->cameraSetup);
        m->depthData = m->stereoReconstructionHelper.Reconstruct(CameraNameLeft, CameraNameRight, imgViewLeft, imgViewRight);
        if (!m->depthData) {
            return -1;
        }

        const Camera<float>& depthCamera = m->depthData->camera;
        const int w = depthCamera.Width();
        const int h = depthCamera.Height();
        const Eigen::Matrix<float, 4, -1>& depthAndNormals = m->depthData->depthAndNormals;

        m->depthIndices.resize(depthAndNormals.cols());
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const int pixelIndex = y * w + x;
                const float depth = depthAndNormals(0, pixelIndex);
                if (depth > 0) {
                    m->depthIndices[pixelIndex] = m->numVertices++;
                } else {
                    m->depthIndices[pixelIndex] = -1;
                }
            }
        }
        for (int y = 0; y < h - 1; ++y) {
            for (int x = 0; x < w - 1; ++x) {
                const float depth00 = depthAndNormals(0, y * w + x);
                const float depth01 = depthAndNormals(0, y * w + x + 1);
                const float depth10 = depthAndNormals(0, (y + 1) * w + x);
                const float depth11 = depthAndNormals(0, (y + 1) * w + x + 1);
                if (depth00 > 0 && depth10 > 0 && depth01 > 0) {
                    m->numTriangles++;
                }
                if (depth10 > 0 && depth11 > 0 && depth01 > 0) {
                    m->numTriangles++;
                }
            }
        }

        return static_cast<int32_t>(m->numVertices);

    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to reconstruct: {}", e.what());
    }
}

int32_t StereoReconstruction::RetrievePointCloud(float* PointCloudDataOut, int32_t MaximumNumberOfPoints) const
{
    RESET_ERROR;
    CHECK_OR_RETURN(PointCloudDataOut, -1, "invalid output pointer");
    CHECK_OR_RETURN(MaximumNumberOfPoints >= 0, -1, "invalid maximum number of points");
    CHECK_OR_RETURN(m->numVertices > 0, -1, "no point cloud was reconstructed");
    CHECK_OR_RETURN(m->numVertices > 0, -1, "no point cloud was reconstructed");
    CHECK_OR_RETURN(m->depthData, -1, "no point cloud was reconstructed");

    const Camera<float>& depthCamera = m->depthData->camera;
    const int w = depthCamera.Width();
    const int h = depthCamera.Height();
    const Eigen::Matrix<float, 4, -1>& depthAndNormals = m->depthData->depthAndNormals;

    int32_t PointsToCopy = std::min<int32_t>(MaximumNumberOfPoints, m->numVertices);

    int vID = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (vID >= PointsToCopy) break;
            const float depth = depthAndNormals(0, y * w + x);
            if (depth > 0) {
                const Eigen::Vector3f pt = depthCamera.Unproject(Eigen::Vector2f(x + 0.5f, y + 0.5f), depth, /*withExtrinsics=*/true);
                PointCloudDataOut[vID++] = pt[0];
                PointCloudDataOut[vID++] = pt[1];
                PointCloudDataOut[vID++] = pt[2];
            }
        }
    }
    CHECK_OR_RETURN(PointsToCopy * 3 == vID, -1, "inconsistent point cloud data");

    return PointsToCopy;
}

void StereoReconstruction::GetReconstructedTriangleMeshSize(int32_t& NumberOfPoints, int32_t& NumberOfTriangles) const
{
    NumberOfPoints = static_cast<int32_t>(m->numVertices);
    NumberOfTriangles = static_cast<int32_t>(m->numTriangles);
}

bool StereoReconstruction::RetrieveReconstructedTriangleMesh(float* VerticesDataOut, int32_t* TriangleIndicesOut) const
{
    RESET_ERROR;
    CHECK_OR_RETURN(VerticesDataOut, false, "invalid output pointer");
    CHECK_OR_RETURN(TriangleIndicesOut, false, "invalid output triangle indices data pointer");
    CHECK_OR_RETURN(m->numVertices > 0, false, "no point cloud was reconstructed");
    CHECK_OR_RETURN(m->depthData, false, "no point cloud was reconstructed");

    const Camera<float>& depthCamera = m->depthData->camera;
    const int w = depthCamera.Width();
    const int h = depthCamera.Height();
    const Eigen::Matrix<float, 4, -1>& depthAndNormals = m->depthData->depthAndNormals;

    int vID = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const float depth = depthAndNormals(0, y * w + x);
            if (depth > 0) {
                const Eigen::Vector3f pt = depthCamera.Unproject(Eigen::Vector2f(x + 0.5f, y + 0.5f), depth, /*withExtrinsics=*/true);
                VerticesDataOut[vID++] = pt[0];
                VerticesDataOut[vID++] = pt[1];
                VerticesDataOut[vID++] = pt[2];
            }
        }
    }

    int tID = 0;
    for (int y = 0; y < h - 1; ++y) {
        for (int x = 0; x < w - 1; ++x) {
            const int32_t index00 = m->depthIndices[y * w + x];
            const int32_t index01 = m->depthIndices[y * w + x + 1];
            const int32_t index10 = m->depthIndices[(y + 1) * w + x];
            const int32_t index11 = m->depthIndices[(y + 1) * w + x + 1];
            if (index00 >= 0 && index10 >= 0 && index01 >= 0) {
                TriangleIndicesOut[tID++] = index00;
                TriangleIndicesOut[tID++] = index10;
                TriangleIndicesOut[tID++] = index01;
            }
            if (index10 >= 0 && index11 >= 0 && index01 >= 0) {
                TriangleIndicesOut[tID++] = index10;
                TriangleIndicesOut[tID++] = index11;
                TriangleIndicesOut[tID++] = index01;
            }
        }
    }

    return true;
}

} // namespace api
} // namespace titan
