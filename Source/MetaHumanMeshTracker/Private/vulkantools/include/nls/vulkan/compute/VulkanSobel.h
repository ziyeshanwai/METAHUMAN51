// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>
#include <nls/vulkan/compute/VulkanEnums.h>

namespace epic::nls
{
    class VulkanSobel : public VulkanComputeContext::Kernel
    {
    public:
        VulkanSobel(std::shared_ptr<VulkanComputeContext> context)
            : VulkanComputeContext::Kernel{ context, "ComputeSobel" }
        {
            VulkanSobel::Init();
        }

        VulkanSobel(VulkanSobel&&) = default;

        bool Init() override;

        void Sobel(const std::shared_ptr<VulkanComputeBuffer>& input,
                   const std::shared_ptr<VulkanComputeBuffer>& output,
                   Direction direction,
                   VulkanComputeContext::HBatch batch = nullptr);
    };
}
