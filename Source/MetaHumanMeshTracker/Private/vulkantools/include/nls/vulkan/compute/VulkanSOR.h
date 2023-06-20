// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic::nls
{
    class VulkanSOR : public VulkanComputeContext::Kernel
    {
    public:
        VulkanSOR(std::shared_ptr<VulkanComputeContext> context)
            : VulkanComputeContext::Kernel{ context, "ComputeSOR" }
        {
            VulkanSOR::Init();
        }

        VulkanSOR(VulkanSOR&&) = default;

        bool Init() override;

        void SOR(const std::shared_ptr<VulkanComputeBuffer>& du_curr,
                 const std::shared_ptr<VulkanComputeBuffer>& dv_curr,
                 const std::shared_ptr<VulkanComputeBuffer>& du_other,
                 const std::shared_ptr<VulkanComputeBuffer>& dv_other,
                 const std::shared_ptr<VulkanComputeBuffer>& weights_curr,
                 const std::shared_ptr<VulkanComputeBuffer>& weights_other,
                 const std::shared_ptr<VulkanComputeBuffer>& A11_curr,
                 const std::shared_ptr<VulkanComputeBuffer>& A12_curr,
                 const std::shared_ptr<VulkanComputeBuffer>& A22_curr,
                 const std::shared_ptr<VulkanComputeBuffer>& b1_curr,
                 const std::shared_ptr<VulkanComputeBuffer>& b2_curr,
                 bool isRedPass,
                 int evenLen,
                 int oddLen,
                 int height,
                 float omega,
                 VulkanComputeContext::HBatch batch = nullptr);
    };
}
