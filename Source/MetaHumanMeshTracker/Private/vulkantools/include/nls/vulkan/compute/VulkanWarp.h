// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>
#include <nls/vulkan/compute/VulkanEnums.h>

namespace epic::nls
{
    class VulkanWarp : public VulkanComputeContext::Kernel
    {
    public:
        VulkanWarp(std::shared_ptr<VulkanComputeContext> computeContext)
            : VulkanComputeContext::Kernel{ computeContext, "ComputeWarp" }
        {
            VulkanWarp::Init();
        }

        VulkanWarp(VulkanWarp&&) = default;

        bool Init() override;

        void Warp(const std::shared_ptr<VulkanComputeBuffer>& input,
                  const std::shared_ptr<VulkanComputeBuffer>& offsetX,
                  const std::shared_ptr<VulkanComputeBuffer>& offsetY,
                  const std::shared_ptr<VulkanComputeBuffer>& output,
                  InterpolationMode interpMode = InterpolationMode::Linear,
                  VulkanComputeContext::HBatch batch = nullptr);
    };
}