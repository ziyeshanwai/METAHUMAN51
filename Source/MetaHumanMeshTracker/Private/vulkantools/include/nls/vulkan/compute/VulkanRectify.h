// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/Camera.h>
#include <nls/geometry/Interpolation.h>

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanRectify : public VulkanComputeContext::Kernel {

public:
    VulkanRectify(std::shared_ptr<VulkanComputeContext> vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeRectify")
    {
        VulkanRectify::Init();
    }
    
    VulkanRectify(VulkanRectify&&) = default;

    bool Init() override;

    void Rectify(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        const Camera<float>& camera,
        const Camera<float>& rectifiedCamera,
        InterpolationMethod interpolationMethod,
        VulkanComputeContext::HBatch batch = nullptr
    );
};


} //namespace nls
} //namespace epic
