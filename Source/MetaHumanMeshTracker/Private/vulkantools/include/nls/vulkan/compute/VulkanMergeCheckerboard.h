// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic::nls
{
    class VulkanMergeCheckerboard : public VulkanComputeContext::Kernel
    {
    public:
        VulkanMergeCheckerboard(std::shared_ptr<VulkanComputeContext> context)
            : VulkanComputeContext::Kernel{ context, "ComputeMergeCheckerboard" }
        {
            VulkanMergeCheckerboard::Init();
        }

        VulkanMergeCheckerboard(VulkanMergeCheckerboard&&) = default;

        bool Init() override;

        void MergeCheckerboard(const std::shared_ptr<VulkanComputeBuffer>& output,
                               const std::shared_ptr<VulkanComputeBuffer>& red,
                               const std::shared_ptr<VulkanComputeBuffer>& black,
                               VulkanComputeContext::HBatch batch = nullptr);
    };
}
