// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanStereoUniquenessConstraint : public VulkanComputeContext::Kernel {

public:
    VulkanStereoUniquenessConstraint(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeStereoUniquenessConstraint")
    {
        VulkanStereoUniquenessConstraint::Init();
    }

    VulkanStereoUniquenessConstraint(VulkanStereoUniquenessConstraint&&) = default;

    bool Init() override;

    void ApplyUniquenessConstraint(
        std::shared_ptr<VulkanComputeBuffer> inputDisparityBuffer,
        std::shared_ptr<VulkanComputeBuffer> otherInputDisparityBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputDisparityBuffer,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
