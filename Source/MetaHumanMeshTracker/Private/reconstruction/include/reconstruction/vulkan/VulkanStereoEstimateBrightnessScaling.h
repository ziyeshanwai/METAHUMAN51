// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

/**
 * Estimate the brightness scaling between left and right image.
 */
class VulkanStereoEstimateBrightnessScaling : public VulkanComputeContext::Kernel {

public:
    VulkanStereoEstimateBrightnessScaling(std::shared_ptr<VulkanComputeContext> vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeStereoEstimateBrightnessScaling")
    {
        VulkanStereoEstimateBrightnessScaling::Init();
    }

    VulkanStereoEstimateBrightnessScaling(VulkanStereoEstimateBrightnessScaling&&) = default;

    bool Init() override;

    void EstimateBrightnessScaling(
        std::shared_ptr<VulkanComputeBuffer> disparityBuffer,
        std::shared_ptr<VulkanComputeBuffer> srcBuffer,
        std::shared_ptr<VulkanComputeBuffer> targetBuffer,
        std::shared_ptr<VulkanComputeBuffer> scalingImageBuffer,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
