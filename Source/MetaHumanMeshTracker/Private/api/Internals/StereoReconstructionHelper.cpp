// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoReconstructionHelper.h"
#include "../Common.h"
#include<pma/utils/ManagedInstance.h>

namespace titan {
namespace api {

using namespace epic::nls;
using namespace pma;

bool StereoReconstructionHelper::Init(std::shared_ptr<epic::nls::VulkanDevice>& vulkanDevice)
{
    RESET_ERROR;
    auto memoryResource = MEM_RESOURCE;
    CHECK_OR_RETURN(vulkanDevice, false, "non existent vulkan device passed to stereo reconstruction helper.");
    auto computeContext = VulkanComputeContext::Create(vulkanDevice);
    CHECK_OR_RETURN(computeContext, false, "could not create compute context");
    m_imagePreprocessorVulkan = pma::UniqueInstance<epic::nls::ImagePreprocessorVulkan>::with(memoryResource).create(computeContext);
    m_stereoReconstructionVulkan = pma::UniqueInstance<epic::nls::StereoReconstructionVulkan>::with(memoryResource).create(computeContext);
    return true;
}

std::shared_ptr<DepthmapData<float>> StereoReconstructionHelper::Reconstruct(
    const std::string& cameraLeft,
    const std::string& cameraRight,
    const ConstImageView& imageViewLeft,
    const ConstImageView& imageViewRight,
    int endLevel)
{
    if (!m_cameraSetup.HasCamera(cameraLeft) ||
        !m_cameraSetup.HasCamera(cameraRight) ||
        !m_cameraSetup.HasCameraRange(cameraLeft) ||
        !m_cameraSetup.HasCameraRange(cameraRight)) {
        return nullptr;
    }
    const int numLevels = 4;
    std::shared_ptr<const CameraImagePairPyramidVulkan> imagePyramidLeftVulkan = m_imagePreprocessorVulkan->Process(m_cameraSetup.GetCamera(cameraLeft), imageViewLeft, {}, numLevels);
    std::shared_ptr<const CameraImagePairPyramidVulkan> imagePyramidRightVulkan = m_imagePreprocessorVulkan->Process(m_cameraSetup.GetCamera(cameraRight), imageViewRight, {}, numLevels);
    auto stereoResult = m_stereoReconstructionVulkan->Reconstruct(imagePyramidLeftVulkan, imagePyramidRightVulkan, m_cameraSetup.GetCameraRange(cameraLeft), m_cameraSetup.GetCameraRange(cameraRight), endLevel);
    
    PolyAllocator<DepthmapData<float>> polyAllocator{ MEM_RESOURCE };
    std::shared_ptr<DepthmapData<float>> depthmapData = std::allocate_shared<DepthmapData<float>>(polyAllocator);
    depthmapData->camera = stereoResult.Data(endLevel).stereoCameraPair.CameraLeft();
    auto ptr = stereoResult.Data(endLevel).DepthAndNormalLeft();
    depthmapData->depthAndNormals.resize(4, depthmapData->camera.Width() * depthmapData->camera.Height());
    ptr->CopyFromDevice(depthmapData->depthAndNormals.data(), depthmapData->depthAndNormals.size() * sizeof(float));

    return depthmapData;
}

} // namespace api
} // namespace titan
