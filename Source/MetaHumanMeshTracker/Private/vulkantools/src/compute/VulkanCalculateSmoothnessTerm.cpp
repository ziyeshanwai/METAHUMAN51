// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanCalculateSmoothnessTerm.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_CalculateSmoothnessTerm.comp.spv.h"

#include "shaders/CalculateSmoothnessTerm.comp.h"
#include "shaders/CalculateSmoothnessTerm.comp.refl.h"

#include <algorithm>
#include <cmath>

namespace epic::nls
{
    bool VulkanCalculateSmoothnessTerm::Init()
    {
        return Kernel::Init<CalculateSmoothnessTermReflection, CalculateSmoothnessTermPushConstants>(SPV_CalculateSmoothnessTerm_comp_spv, sizeof(SPV_CalculateSmoothnessTerm_comp_spv));
    }

    void VulkanCalculateSmoothnessTerm::CalculateSmoothnessTerm(const std::shared_ptr<VulkanComputeBuffer>& u_const_curr,
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

                                                                VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        static_assert(sizeof(DataTermParams) == sizeof(::DataTermShaderParams), "DataTermParams layout must match the one defined in DataTermsParameters.h");

        CARBON_PRECONDITION(u_const_curr, "u_const_curr buffer must be valid");
        CARBON_PRECONDITION(v_const_curr, "v_const_curr buffer must be valid");
        CARBON_PRECONDITION(u_const_other, "u_const_other buffer must be valid");
        CARBON_PRECONDITION(v_const_other, "v_const_other buffer must be valid");
        CARBON_PRECONDITION(u_update_curr, "u_update_curr buffer must be valid");
        CARBON_PRECONDITION(v_update_curr, "v_update_curr buffer must be valid");
        CARBON_PRECONDITION(u_update_other, "u_update_other buffer must be valid");
        CARBON_PRECONDITION(v_update_other, "v_update_other buffer must be valid");
        CARBON_PRECONDITION(weights_curr, "weights_curr buffer must be valid");
        CARBON_PRECONDITION(A11_curr, "A11_curr buffer must be valid");
        CARBON_PRECONDITION(b1_curr, "b1_curr buffer must be valid");
        CARBON_PRECONDITION(A22_curr, "A22_curr buffer must be valid");
        CARBON_PRECONDITION(b2_curr, "b2_curr buffer must be valid");
        CARBON_PRECONDITION(A11_other, "A11_other buffer must be valid");
        CARBON_PRECONDITION(b1_other, "b1_other buffer must be valid");
        CARBON_PRECONDITION(A22_other, "A22_other buffer must be valid");
        CARBON_PRECONDITION(b2_other, "b2_other buffer must be valid");

        CalculateSmoothnessTermPushConstants ubo;
        ubo.isRedPass = isRedPass;
        ubo.evenLen = evenLen;
        ubo.oddLen = oddLen;
        ubo.width = u_const_curr->Width();
        ubo.height = height;
        std::memcpy(&ubo.params, &params, sizeof(DataTermParams));

        switch (direction)
        {
        case Direction::Horizontal:
            ubo.direction = SMOOTHNESS_TERM_DIRECTION_HORIZONTAL;
            break;
        case Direction::Vertical:
            ubo.direction = SMOOTHNESS_TERM_DIRECTION_VERTICAL;
            break;
        default:
            CARBON_CRITICAL("direction is not valid");
        }

        CalculateSmoothnessTermReflection::DescSet0_Update dsu{
            u_const_curr->ManagedBuffer()->Buffer(),
            v_const_curr->ManagedBuffer()->Buffer(),
            u_const_other->ManagedBuffer()->Buffer(),
            v_const_other->ManagedBuffer()->Buffer(),
            weights_curr->ManagedBuffer()->Buffer(),
            A11_curr->ManagedBuffer()->Buffer(),
            b1_curr->ManagedBuffer()->Buffer(),
            A22_curr->ManagedBuffer()->Buffer(),
            b2_curr->ManagedBuffer()->Buffer(),
            A11_other->ManagedBuffer()->Buffer(),
            b1_other->ManagedBuffer()->Buffer(),
            A22_other->ManagedBuffer()->Buffer(),
            b2_other->ManagedBuffer()->Buffer(),
            u_update_curr->ManagedBuffer()->Buffer(),
            v_update_curr->ManagedBuffer()->Buffer(),
            u_update_other->ManagedBuffer()->Buffer(),
            v_update_other->ManagedBuffer()->Buffer()
        };

        const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(std::max(evenLen, oddLen) / static_cast<float>(CalculateSmoothnessTermThreadSizeX)));
        const uint32_t numGroupsY = static_cast<uint32_t>(std::ceil(height / static_cast<float>(CalculateSmoothnessTermThreadSizeY)));

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}
