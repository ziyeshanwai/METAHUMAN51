// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/Interpolation.h>
#include <nls/geometry/MetaShapeCamera.h>

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanComputeUndistort : public VulkanComputeContext::Kernel {

public:
    VulkanComputeUndistort(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeUndistort")
    {
        VulkanComputeUndistort::Init();
    }

    VulkanComputeUndistort(VulkanComputeUndistort&&) = default;

    bool Init() override;

    void Undistort(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        const MetaShapeCamera<float>& camera,
        const Camera<float>& rectifiedCamera,
        InterpolationMethod interpolationMethod,
        VulkanComputeContext::HBatch batch = nullptr
    );

};


} //namespace nls
} //namespace epic
