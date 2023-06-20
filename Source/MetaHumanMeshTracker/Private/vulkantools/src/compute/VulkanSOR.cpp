// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanSOR.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_SOR.comp.spv.h"

#include "shaders/SOR.comp.h"
#include "shaders/SOR.comp.refl.h"

#include <algorithm>
#include <cmath>

namespace epic::nls
{
    bool VulkanSOR::Init()
    {
        return Kernel::Init<SORReflection, SORPushConstants>(SPV_SOR_comp_spv, sizeof(SPV_SOR_comp_spv));
    }

    void VulkanSOR::SOR(const std::shared_ptr<VulkanComputeBuffer>& du_curr,
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
                        VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        CARBON_PRECONDITION(du_curr, "du_curr buffer must be valid");
        CARBON_PRECONDITION(dv_curr, "dv_curr buffer must be valid");
        CARBON_PRECONDITION(du_other, "du_other buffer must be valid");
        CARBON_PRECONDITION(dv_other, "du_curr buffer must be valid");
        CARBON_PRECONDITION(weights_curr, "weights_curr buffer must be valid");
        CARBON_PRECONDITION(weights_other, "weights_other buffer must be valid");
        CARBON_PRECONDITION(A11_curr, "A11_curr buffer must be valid");
        CARBON_PRECONDITION(A12_curr, "A12_curr buffer must be valid");
        CARBON_PRECONDITION(A22_curr, "A22_curr buffer must be valid");
        CARBON_PRECONDITION(b1_curr, "b1_curr buffer must be valid");
        CARBON_PRECONDITION(b2_curr, "b2_curr buffer must be valid");

        SORPushConstants ubo;
        ubo.isRedPass = isRedPass;
        ubo.evenLen = evenLen;
        ubo.oddLen = oddLen;
        ubo.width = du_curr->Width();
        ubo.height = height;
        ubo.omega = omega;

        SORReflection::DescSet0_Update dsu{
            du_curr->ManagedBuffer()->Buffer(),
            dv_curr->ManagedBuffer()->Buffer(),
            du_other->ManagedBuffer()->Buffer(),
            dv_other->ManagedBuffer()->Buffer(),
            weights_curr->ManagedBuffer()->Buffer(),
            weights_other->ManagedBuffer()->Buffer(),
            A11_curr->ManagedBuffer()->Buffer(),
            A12_curr->ManagedBuffer()->Buffer(),
            A22_curr->ManagedBuffer()->Buffer(),
            b1_curr->ManagedBuffer()->Buffer(),
            b2_curr->ManagedBuffer()->Buffer()
        };

        const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(std::max(evenLen, oddLen) / static_cast<float>(SORThreadSizeX))) + 1;
        const uint32_t numGroupsY = static_cast<uint32_t>(std::ceil(height / static_cast<float>(SORThreadSizeY)));

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}
