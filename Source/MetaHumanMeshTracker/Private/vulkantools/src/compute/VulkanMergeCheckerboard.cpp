// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanMergeCheckerboard.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_MergeCheckerboard.comp.spv.h"

#include "shaders/MergeCheckerboard.comp.h"
#include "shaders/MergeCheckerboard.comp.refl.h"

#include <cmath>

namespace epic::nls
{
    bool VulkanMergeCheckerboard::Init()
    {
        return Kernel::Init<MergeCheckerboardReflection, MergeCheckerboardPushConstants>(SPV_MergeCheckerboard_comp_spv, sizeof(SPV_MergeCheckerboard_comp_spv));
    }

    void VulkanMergeCheckerboard::MergeCheckerboard(const std::shared_ptr<VulkanComputeBuffer>& output,
                                                    const std::shared_ptr<VulkanComputeBuffer>& red,
                                                    const std::shared_ptr<VulkanComputeBuffer>& black,
                                                    VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        CARBON_PRECONDITION(output, "output buffer must be valid");
        CARBON_PRECONDITION(red, "red buffer must be valid");
        CARBON_PRECONDITION(black, "black buffer must be valid");
        CARBON_PRECONDITION(red->Width() == black->Width() &&
                            red->Height() == black->Height(), "Red and Black buffers must be of the same size");

        const int outputWidth = output->Width();
        const int outputHeight = output->Height();

        MergeCheckerboardPushConstants ubo;
        ubo.width = outputWidth;
        ubo.height = outputHeight;
        ubo.redBlackWidth = red->Width();

        MergeCheckerboardReflection::DescSet0_Update dsu{
            red->ManagedBuffer()->Buffer(),
            black->ManagedBuffer()->Buffer(),
            output->ManagedBuffer()->Buffer()
        };

        const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(outputWidth / static_cast<float>(MergeCheckerboardThreadSizeX)));
        const uint32_t numGroupsY = static_cast<uint32_t>(outputHeight);

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}
