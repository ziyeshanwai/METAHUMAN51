// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic::nls
{
    class VulkanCopyBuffer : VulkanComputeContext::Kernel
    {
    public:
        VulkanCopyBuffer(std::shared_ptr<VulkanComputeContext> context)
            : VulkanCopyBuffer::Kernel{ context, "ComputeCopyBuffer" }
        {
            VulkanCopyBuffer::Init();
        }

        VulkanCopyBuffer(VulkanCopyBuffer&&) = default;

        bool Init() override;

        void Copy(const std::shared_ptr<VulkanComputeBuffer>& input,
                  const std::shared_ptr<VulkanComputeBuffer>& output,
                  VulkanComputeContext::HBatch batch = nullptr);
    };
}
