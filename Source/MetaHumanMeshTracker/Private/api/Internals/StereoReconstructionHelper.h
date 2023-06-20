// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/MetaShapeCamera.h>
#include <nls/vulkan/common/VulkanInstance.h>
#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <reconstruction/MultiCameraSetup.h>
#include <reconstruction/vulkan/ImagePreprocessorVulkan.h>
#include <reconstruction/vulkan/StereoReconstructionVulkan.h>
#include <tracking/FaceTracking.h>

namespace titan {
namespace api {

class StereoReconstructionHelper {

public:
    StereoReconstructionHelper() {}
    ~StereoReconstructionHelper() {}

    bool Init(std::shared_ptr<epic::nls::VulkanDevice>& vulkanDevice);

    bool IsInitialized() const { return bool(m_imagePreprocessorVulkan.get()); }

    void SetCameraSetup(const epic::nls::MultiCameraSetup<float>& cameraSetup)
    {
        m_cameraSetup = cameraSetup;
    }

    std::shared_ptr<epic::nls::DepthmapData<float>> Reconstruct(const std::string& cameraLeft, const std::string& cameraRight, const epic::nls::ConstImageView& imageViewLeft, const epic::nls::ConstImageView& imageViewRight, int endLevel = 0);

private:
    std::unique_ptr<epic::nls::ImagePreprocessorVulkan, std::function<void(epic::nls::ImagePreprocessorVulkan*)>> m_imagePreprocessorVulkan;
    std::unique_ptr<epic::nls::StereoReconstructionVulkan, std::function<void(epic::nls::StereoReconstructionVulkan*)>> m_stereoReconstructionVulkan;
    epic::nls::MultiCameraSetup<float> m_cameraSetup;
};

} // namespace api
} // namespace titan
