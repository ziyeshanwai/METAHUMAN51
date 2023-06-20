// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>
#include <nls/vulkan/compute/VulkanEnums.h>

namespace epic::nls
{
    class VulkanFlowConfidence : VulkanComputeContext::Kernel
    {
    public:
        VulkanFlowConfidence(std::shared_ptr<VulkanComputeContext> context)
            : VulkanComputeContext::Kernel{ context, "ComputeFlowConfidence" }
        {
            VulkanFlowConfidence::Init();
        }

        VulkanFlowConfidence(VulkanFlowConfidence&&) = default;

        bool Init() override;

        void FlowConfidence(const std::shared_ptr<VulkanComputeBuffer>& flowU,
                            const std::shared_ptr<VulkanComputeBuffer>& flowV,
                            const std::shared_ptr<VulkanComputeBuffer>& invFlowU,
                            const std::shared_ptr<VulkanComputeBuffer>& invFlowV,
                            const std::shared_ptr<VulkanComputeBuffer>& outputConfidence,
                            InterpolationMode interpMode = InterpolationMode::Linear,
                            VulkanComputeContext::HBatch batch = nullptr);
    };
}
