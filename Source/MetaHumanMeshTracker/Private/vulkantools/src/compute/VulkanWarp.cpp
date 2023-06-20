// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanWarp.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_Warp.comp.spv.h"

#include "shaders/Warp.comp.h"
#include "shaders/Warp.comp.refl.h"

#include <cmath>

namespace epic::nls
{
    bool VulkanWarp::Init()
    {
        return Kernel::Init<WarpReflection, WarpPushConstants>(SPV_Warp_comp_spv, sizeof(SPV_Warp_comp_spv));
    }

    void VulkanWarp::Warp(const std::shared_ptr<VulkanComputeBuffer>& input,
                          const std::shared_ptr<VulkanComputeBuffer>& offsetX,
                          const std::shared_ptr<VulkanComputeBuffer>& offsetY,
                          const std::shared_ptr<VulkanComputeBuffer>& output,
                          InterpolationMode interpMode /* = InterpolationMode::Linear */,
                          VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        CARBON_PRECONDITION(input, "input buffer must be valid");
        CARBON_PRECONDITION(offsetX, "offsetX buffer must be valid");
        CARBON_PRECONDITION(offsetX, "offsetY buffer must be valid");
        CARBON_PRECONDITION(output, "output buffer must be valid");

        const int inputWidth = input->Width();
        const int inputHeight = input->Height();
        const int outputWidth = output->Width();
        const int outputHeight = output->Height();

        WarpPushConstants ubo;
        ubo.outputWidth = outputWidth;
        ubo.outputHeight = outputHeight;
        ubo.maxX = inputWidth - 1;
        ubo.maxY = inputHeight - 1;
        ubo.interpolationMode = static_cast<int>(interpMode);

        WarpReflection::DescSet0_Update dsu{
           input->ManagedBuffer()->Buffer(),
           offsetX->ManagedBuffer()->Buffer(),
           offsetY->ManagedBuffer()->Buffer(),
           output->ManagedBuffer()->Buffer()
        };

        const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(outputWidth / static_cast<float>(WarpComputeThreadSizeX)));
        const uint32_t numGroupsY = static_cast<uint32_t>(std::ceil(outputHeight / static_cast<float>(WarpComputeThreadSizeY)));

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}
