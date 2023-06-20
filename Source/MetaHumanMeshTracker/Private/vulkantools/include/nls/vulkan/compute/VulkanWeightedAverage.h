// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic::nls
{
    class VulkanWeightedAverage : public VulkanComputeContext::Kernel
    {
    public:
        VulkanWeightedAverage(std::shared_ptr<VulkanComputeContext> context)
            : VulkanComputeContext::Kernel{ context, "ComputeWeighedAverage" }
        {
            VulkanWeightedAverage::Init();
        }

        VulkanWeightedAverage(VulkanWeightedAverage&&) = default;

        bool Init() override;

        void WeightedAverage(const std::shared_ptr<VulkanComputeBuffer>& inputA,
                             const std::shared_ptr<VulkanComputeBuffer>& inputB,
                             const std::shared_ptr<VulkanComputeBuffer>& output,
                             float weightA,
                             float weightB,
                             float delta = 0.0f,
                             VulkanComputeContext::HBatch batch = nullptr);
    };
}
