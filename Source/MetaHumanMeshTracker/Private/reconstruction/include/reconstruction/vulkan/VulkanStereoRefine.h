// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanStereoRefine : public VulkanComputeContext::Kernel {

public:
    VulkanStereoRefine(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeStereoRefine")
    {
        VulkanStereoRefine::Init();
    }

    VulkanStereoRefine(VulkanStereoRefine&&) = default;

    bool Init() override;

    void Refine(
        std::shared_ptr<VulkanComputeBuffer> imgLeft,
        std::shared_ptr<VulkanComputeBuffer> imgRight,
        std::shared_ptr<VulkanComputeBuffer> inputDisparityBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputDisparityBuffer,
        float ws,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
