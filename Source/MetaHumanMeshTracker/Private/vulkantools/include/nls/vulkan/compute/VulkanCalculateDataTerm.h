// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

#include <nls/vulkan/compute/VulkanDataTermParameters.h>

namespace epic::nls
{
    class VulkanCalculateDataTerm : public VulkanComputeContext::Kernel
    {
    public:
        VulkanCalculateDataTerm(std::shared_ptr<VulkanComputeContext> context)
            : VulkanComputeContext::Kernel{ context, "ComputeDataTerm" }
        {
            VulkanCalculateDataTerm::Init();
        }

        VulkanCalculateDataTerm(VulkanCalculateDataTerm&&) = default;

        bool Init() override;

        void CalculateDataTerm(const std::shared_ptr<VulkanComputeBuffer>& Ix_rb,
                               const std::shared_ptr<VulkanComputeBuffer>& Iy_rb,
                               const std::shared_ptr<VulkanComputeBuffer>& Iz_rb,
                               const std::shared_ptr<VulkanComputeBuffer>& Ixx_rb,
                               const std::shared_ptr<VulkanComputeBuffer>& Ixy_rb,
                               const std::shared_ptr<VulkanComputeBuffer>& Iyy_rb,
                               const std::shared_ptr<VulkanComputeBuffer>& Ixz_rb,
                               const std::shared_ptr<VulkanComputeBuffer>& Iyz_rb,
                               const std::shared_ptr<VulkanComputeBuffer>& A11,
                               const std::shared_ptr<VulkanComputeBuffer>& A12,
                               const std::shared_ptr<VulkanComputeBuffer>& A22,
                               const std::shared_ptr<VulkanComputeBuffer>& b1,
                               const std::shared_ptr<VulkanComputeBuffer>& b2,
                               const std::shared_ptr<VulkanComputeBuffer>& dW_u,
                               const std::shared_ptr<VulkanComputeBuffer>& dw_v,
                               int evenLen,
                               int oddLen,
                               int height,
                               DataTermParams params,
                               VulkanComputeContext::HBatch batch = nullptr);
    };
}
