// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic::nls
{
    class VulkanSplitCheckerboard : public VulkanComputeContext::Kernel
    {
    public:
        VulkanSplitCheckerboard(std::shared_ptr<VulkanComputeContext> context)
            : VulkanComputeContext::Kernel{ context, "ComputeSplitCheckerboard" }
        {
            VulkanSplitCheckerboard::Init();
        }

        VulkanSplitCheckerboard(VulkanSplitCheckerboard&&) = default;

        bool Init() override;

        void SplitCheckerboard(const std::shared_ptr<VulkanComputeBuffer>& input,
                               const std::shared_ptr<VulkanComputeBuffer>& red,
                               const std::shared_ptr<VulkanComputeBuffer>& black,
                               VulkanComputeContext::HBatch batch = nullptr);
    };
}
