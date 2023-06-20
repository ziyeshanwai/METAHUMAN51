// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanStereoMaskConstraint : public VulkanComputeContext::Kernel {

public:
    VulkanStereoMaskConstraint(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeStereoMaskConstraint")
    {
        VulkanStereoMaskConstraint::Init();
    }

    VulkanStereoMaskConstraint(VulkanStereoMaskConstraint&&) = default;

    bool Init() override;

    void MaskConstraint(
        std::shared_ptr<VulkanComputeBuffer> disparityBuffer,
        std::shared_ptr<VulkanComputeBuffer> leftBuffer,
        std::shared_ptr<VulkanComputeBuffer> rightBuffer,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
