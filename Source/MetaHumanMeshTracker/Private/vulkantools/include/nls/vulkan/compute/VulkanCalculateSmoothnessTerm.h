// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>
#include <nls/vulkan/compute/VulkanDataTermParameters.h>
#include <nls/vulkan/compute/VulkanEnums.h>

namespace epic::nls
{
    class VulkanCalculateSmoothnessTerm : public VulkanComputeContext::Kernel
    {
    public:
        VulkanCalculateSmoothnessTerm(std::shared_ptr<VulkanComputeContext> context)
            : VulkanComputeContext::Kernel{ context, "ComputeSmoothnessTerm" }
        {
            VulkanCalculateSmoothnessTerm::Init();
        }

        VulkanCalculateSmoothnessTerm(VulkanCalculateSmoothnessTerm&&) = default;

        bool Init() override;

        void CalculateSmoothnessTerm(const std::shared_ptr<VulkanComputeBuffer>& u_const_curr,
                                     const std::shared_ptr<VulkanComputeBuffer>& v_const_curr,
                                     const std::shared_ptr<VulkanComputeBuffer>& u_const_other,
                                     const std::shared_ptr<VulkanComputeBuffer>& v_const_other,

                                     const std::shared_ptr<VulkanComputeBuffer>& u_update_curr,
                                     const std::shared_ptr<VulkanComputeBuffer>& v_update_curr,
                                     const std::shared_ptr<VulkanComputeBuffer>& u_update_other,
                                     const std::shared_ptr<VulkanComputeBuffer>& v_update_other,

                                     const std::shared_ptr<VulkanComputeBuffer>& weights_curr,
                                     const std::shared_ptr<VulkanComputeBuffer>& A11_curr,
                                     const std::shared_ptr<VulkanComputeBuffer>& b1_curr,
                                     const std::shared_ptr<VulkanComputeBuffer>& A22_curr,
                                     const std::shared_ptr<VulkanComputeBuffer>& b2_curr,
                                     const std::shared_ptr<VulkanComputeBuffer>& A11_other,
                                     const std::shared_ptr<VulkanComputeBuffer>& b1_other,
                                     const std::shared_ptr<VulkanComputeBuffer>& A22_other,
                                     const std::shared_ptr<VulkanComputeBuffer>& b2_other,

                                     bool isRedPass,
                                     int evenLen,
                                     int oddLen,
                                     int height,

                                     const DataTermParams& params,

                                     Direction direction,

                                     VulkanComputeContext::HBatch batch = nullptr);
    };
}
