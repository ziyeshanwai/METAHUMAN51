// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanSplitCheckerboard.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_SplitCheckerboard.comp.spv.h"

#include "shaders/SplitCheckerboard.comp.h"
#include "shaders/SplitCheckerboard.comp.refl.h"

#include <cmath>

namespace epic::nls
{
    bool VulkanSplitCheckerboard::Init()
    {
        return Kernel::Init<SplitCheckerboardReflection, SplitCheckerboardPushConstants>(SPV_SplitCheckerboard_comp_spv, sizeof(SPV_SplitCheckerboard_comp_spv));
    }

    void VulkanSplitCheckerboard::SplitCheckerboard(const std::shared_ptr<VulkanComputeBuffer>& input,
                                                    const std::shared_ptr<VulkanComputeBuffer>& red,
                                                    const std::shared_ptr<VulkanComputeBuffer>& black,
                                                    VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        CARBON_PRECONDITION(input, "input buffer must be valid");
        CARBON_PRECONDITION(red, "redOutput buffer must be valid");
        CARBON_PRECONDITION(black, "blackOutput buffer must be valid");
        CARBON_PRECONDITION(red->Width() == black->Width() &&
                            red->Height() == black->Height(), "Red and Black buffers must be of the same size");

        const int inputWidth = input->Width();
        const int inputHeight = input->Height();

        SplitCheckerboardPushConstants ubo;
        ubo.width = inputWidth;
        ubo.height = inputHeight;
        ubo.redBlackWidth = red->Width();

        SplitCheckerboardReflection::DescSet0_Update dsu{
            input->ManagedBuffer()->Buffer(),
            red->ManagedBuffer()->Buffer(),
            black->ManagedBuffer()->Buffer()
        };

        const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(inputWidth / static_cast<float>(SplitCheckerboardThreadSizeX)));
        const uint32_t numGroupsY = static_cast<uint32_t>(inputHeight);

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}