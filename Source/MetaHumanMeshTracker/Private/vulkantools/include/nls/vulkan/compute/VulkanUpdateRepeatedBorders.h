// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>
#include <nls/vulkan/compute/VulkanEnums.h>

namespace epic::nls
{
    class VulkanUpdateRepeatedBorders : public VulkanComputeContext::Kernel
    {
    public:
        VulkanUpdateRepeatedBorders(std::shared_ptr<VulkanComputeContext> context)
            : VulkanComputeContext::Kernel{ context, "ComputeUpdateRepeatedBorders" }
        {
            VulkanUpdateRepeatedBorders::Init();
        }

        VulkanUpdateRepeatedBorders(VulkanUpdateRepeatedBorders&&) = default;

        bool Init() override;

        void UpdateRepeatedBorders(const std::shared_ptr<VulkanComputeBuffer>& red,
                                   const std::shared_ptr<VulkanComputeBuffer>& black,
                                   int numRedEven,
                                   int numRedOdd,
                                   int numBlackEven,
                                   int numBlackOdd,
                                   Borders borders,
                                   VulkanComputeContext::HBatch batch = nullptr);
    };
}
