// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic::nls
{
    class VulkanFillBuffer : VulkanComputeContext::Kernel
    {
    public:
        VulkanFillBuffer(std::shared_ptr<VulkanComputeContext> context)
            : VulkanComputeContext::Kernel{ context, "ComputeFillBuffer" }
        {
            VulkanFillBuffer::Init();
        }

        VulkanFillBuffer(VulkanFillBuffer&&) = default;

        bool Init() override;

        void Fill(const std::shared_ptr<VulkanComputeBuffer>& buffer,
                  float value = 0.0f,
                  VulkanComputeContext::HBatch batch = nullptr);
    };
}
