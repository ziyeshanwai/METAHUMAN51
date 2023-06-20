// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

#include <nls/vulkan/compute/VulkanEnums.h>

namespace epic::nls
{
    class VulkanResize : public VulkanComputeContext::Kernel
    {
    public:
        VulkanResize(std::shared_ptr<VulkanComputeContext> computeContext)
            : VulkanComputeContext::Kernel{ computeContext, "ComputeResize" }
        {
            VulkanResize::Init();
        }

        VulkanResize(VulkanResize&&) = default;

        bool Init() override;

        void Resize(const std::shared_ptr<VulkanComputeBuffer>& input,
                    const std::shared_ptr<VulkanComputeBuffer>& output,
                    InterpolationMode interpMode = InterpolationMode::Linear,
                    float scale = 1.0f,
                    VulkanComputeContext::HBatch batch = nullptr);
    };
}
