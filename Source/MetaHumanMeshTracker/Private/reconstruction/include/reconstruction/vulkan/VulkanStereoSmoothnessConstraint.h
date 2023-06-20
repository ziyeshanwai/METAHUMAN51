// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanStereoSmoothnessConstraint : public VulkanComputeContext::Kernel {

public:
    VulkanStereoSmoothnessConstraint(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeStereoSmoothnessConstraint")
    {
        VulkanStereoSmoothnessConstraint::Init();
    }

    VulkanStereoSmoothnessConstraint(VulkanStereoSmoothnessConstraint&&) = default;

    bool Init() override;

    void ApplySmoothnessConstraint(
        std::shared_ptr<VulkanComputeBuffer> inputDisparityBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputDisparityBuffer,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
