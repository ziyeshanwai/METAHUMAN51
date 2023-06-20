// Copyright Epic Games, Inc. All Rights Reserved.


#include <nls/vulkan/compute/VulkanSobel.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_Sobel.comp.spv.h"

#include "shaders/Sobel.comp.h"
#include "shaders/Sobel.comp.refl.h"

#include <cmath>

namespace epic::nls
{
    bool VulkanSobel::Init()
    {
        return Kernel::Init<SobelReflection, SobelPushConstants>(SPV_Sobel_comp_spv, sizeof(SPV_Sobel_comp_spv));
    }

    void VulkanSobel::Sobel(const std::shared_ptr<VulkanComputeBuffer>& input,
                            const std::shared_ptr<VulkanComputeBuffer>& output,
                            Direction direction,
                            VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        CARBON_PRECONDITION(input, "input buffer must be valid");
        CARBON_PRECONDITION(output, "output buffer must be valid");
        CARBON_PRECONDITION(input->Width() == output->Width() &&
                            input->Height() == output->Height(), "input and output buffers must have the same size");

        const int width = output->Width();
        const int height = output->Height();

        SobelPushConstants ubo;
        ubo.width = width;
        ubo.height = height;

        switch (direction)
        {
        case Direction::Horizontal:
            ubo.direction = SOBEL_DIRECTION_HORIZONTAL;
            break;

        case Direction::Vertical:
            ubo.direction = SOBEL_DIRECTION_VERTICAL;
            break;

        default:
            CARBON_CRITICAL("invalid direction for sobel filter");
        }

        SobelReflection::DescSet0_Update dsu{
            input->ManagedBuffer()->Buffer(),
            output->ManagedBuffer()->Buffer()
        };

        const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(width / static_cast<float>(SobelComputeThreadSizeX)));
        const uint32_t numGroupsY = static_cast<uint32_t>(std::ceil(height / static_cast<float>(SobelComputeThreadSizeY)));

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}
