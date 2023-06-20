// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanUpdateRepeatedBorders.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_UpdateRepeatedBorders.comp.spv.h"

#include "shaders/UpdateRepeatedBorders.comp.h"
#include "shaders/UpdateRepeatedBorders.comp.refl.h"

#include <cmath>

namespace epic::nls
{
    bool VulkanUpdateRepeatedBorders::Init()
    {
        return Kernel::Init<UpdateRepeatedBordersReflection, UpdateRepeatedBordersPushConstants>(SPV_UpdateRepeatedBorders_comp_spv, sizeof(SPV_UpdateRepeatedBorders_comp_spv));
    }

    void VulkanUpdateRepeatedBorders::UpdateRepeatedBorders(const std::shared_ptr<VulkanComputeBuffer>& red,
                                                            const std::shared_ptr<VulkanComputeBuffer>& black,
                                                            int numRedEven,
                                                            int numRedOdd,
                                                            int numBlackEven,
                                                            int numBlackOdd,
                                                            Borders borders,
                                                            VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        CARBON_PRECONDITION(red, "red buffer must be valid");
        CARBON_PRECONDITION(black, "black buffer must be valid");
        CARBON_PRECONDITION(red->Width() == black->Width() &&
                            red->Height() == black->Height(), "red and black buffers must have the same size");

        const int width = red->Width();
        const int height = red->Height();

        UpdateRepeatedBordersPushConstants ubo;
        ubo.width = width;
        ubo.height = height;
        ubo.numRedEven = numRedEven;
        ubo.numRedOdd = numRedOdd;
        ubo.numBlackEven = numBlackEven;
        ubo.numBlackOdd = numBlackOdd;

        uint32_t numGroupsX = 0;
        const uint32_t numGroupsY = 2;

        switch (borders)
        {
        case Borders::LeftRight:
            numGroupsX = static_cast<uint32_t>(std::ceil(height / static_cast<float>(UpdateRepeatedBordersThreadSizeX)));
            ubo.borders = UPDATE_BORDERS_LEFT_RIGHT;
            break;
        case Borders::TopBottom:
            numGroupsX = static_cast<uint32_t>(std::ceil(width / static_cast<float>(UpdateRepeatedBordersThreadSizeX)));
            ubo.borders = UPDATE_BORDERS_TOP_BOTTOM;
            break;
        default:
            CARBON_CRITICAL("Border is not valid");
        }

        UpdateRepeatedBordersReflection::DescSet0_Update dsu{
            red->ManagedBuffer()->Buffer(),
            black->ManagedBuffer()->Buffer()
        };

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}
