// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

/**
 * Mask disparity by comparing the brightness scaling.
 */
class VulkanStereoMaskByBrightnessScaling : public VulkanComputeContext::Kernel {

public:
    VulkanStereoMaskByBrightnessScaling(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeStereoMaskByBrightnessScaling")
    {
        VulkanStereoMaskByBrightnessScaling::Init();
    }

    VulkanStereoMaskByBrightnessScaling(VulkanStereoMaskByBrightnessScaling&&) = default;

    bool Init() override;

    void MaskByBrightnessScaling(
        std::shared_ptr<VulkanComputeBuffer> disparityBuffer,
        std::shared_ptr<VulkanComputeBuffer> srcBuffer,
        std::shared_ptr<VulkanComputeBuffer> targetBuffer,
        float mainScale,
        float scaleDifferenceThreshold,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
