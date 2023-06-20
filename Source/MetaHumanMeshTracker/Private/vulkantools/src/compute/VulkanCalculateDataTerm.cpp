// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanCalculateDataTerm.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_CalculateDataTerm.comp.spv.h"

#include "shaders/CalculateDataTerm.comp.h"
#include "shaders/CalculateDataTerm.comp.refl.h"

#include <algorithm>
#include <cmath>

namespace epic::nls
{
    bool VulkanCalculateDataTerm::Init()
    {
        return Kernel::Init<CalculateDataTermReflection, CalculateDataTermPushConstants>(SPV_CalculateDataTerm_comp_spv, sizeof(SPV_CalculateDataTerm_comp_spv));
    }

    void VulkanCalculateDataTerm::CalculateDataTerm(const std::shared_ptr<VulkanComputeBuffer>& Ix_rb,
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
                                                    VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        static_assert(sizeof(DataTermParams) == sizeof(::DataTermShaderParams), "DataTermsParameters.h");

        CARBON_PRECONDITION(Ix_rb, "Ix_rb buffer must be valid");
        CARBON_PRECONDITION(Iy_rb, "Iy_rb buffer must be valid");
        CARBON_PRECONDITION(Iz_rb, "Iz_rb buffer must be valid");
        CARBON_PRECONDITION(Ixx_rb, "Ixx_rb buffer must be valid");
        CARBON_PRECONDITION(Ixy_rb, "Ixy_rb buffer must be valid");
        CARBON_PRECONDITION(Iyy_rb, "Iyy_rb buffer must be valid");
        CARBON_PRECONDITION(Ixz_rb, "Ixz_rb buffer must be valid");
        CARBON_PRECONDITION(Iyz_rb, "Iyz_rb buffer must be valid");
        CARBON_PRECONDITION(A11, "A11 buffer must be valid");
        CARBON_PRECONDITION(A12, "A12 buffer must be valid");
        CARBON_PRECONDITION(A22, "A22 buffer must be valid");
        CARBON_PRECONDITION(b1, "b1 buffer must be valid");
        CARBON_PRECONDITION(b2, "b2 buffer must be valid");
        CARBON_PRECONDITION(dW_u, "dW_u buffer must be valid");
        CARBON_PRECONDITION(dw_v, "dw_v buffer must be valid");

        CalculateDataTermPushConstants ubo;
        ubo.evenLen = evenLen;
        ubo.oddLen = oddLen;
        ubo.height = height;
        ubo.width = Ix_rb->Width();
        std::memcpy(&ubo.params, &params, sizeof(DataTermParams));

        CalculateDataTermReflection::DescSet0_Update dsu{
            Ix_rb->ManagedBuffer()->Buffer(),
            Iy_rb->ManagedBuffer()->Buffer(),
            Iz_rb->ManagedBuffer()->Buffer(),
            Ixx_rb->ManagedBuffer()->Buffer(),
            Ixy_rb->ManagedBuffer()->Buffer(),
            Iyy_rb->ManagedBuffer()->Buffer(),
            Ixz_rb->ManagedBuffer()->Buffer(),
            Iyz_rb->ManagedBuffer()->Buffer(),
            dW_u->ManagedBuffer()->Buffer(),
            dw_v->ManagedBuffer()->Buffer(),
            A11->ManagedBuffer()->Buffer(),
            A12->ManagedBuffer()->Buffer(),
            A22->ManagedBuffer()->Buffer(),
            b1->ManagedBuffer()->Buffer(),
            b2->ManagedBuffer()->Buffer()
        };

        const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(std::max(evenLen, oddLen) / static_cast<float>(CalculateDataTermThreadSizeX)));
        const uint32_t numGroupsY = static_cast<uint32_t>(std::ceil(height / static_cast<float>(CalculateDataTermThreadSizeY)));

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}
