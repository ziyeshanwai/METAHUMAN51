// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanResize.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_Resize.comp.spv.h"

#include "shaders/Resize.comp.h"
#include "shaders/Resize.comp.refl.h"

#include <cmath>

namespace epic::nls
{
    bool VulkanResize::Init()
    {
        return Kernel::Init<ResizeReflection, ResizePushConstants>(SPV_Resize_comp_spv, sizeof(SPV_Resize_comp_spv));
    }

    void VulkanResize::Resize(const std::shared_ptr<VulkanComputeBuffer>& input,
                              const std::shared_ptr<VulkanComputeBuffer>& output,
                              InterpolationMode interpMode /* = InterpolationMode::Linear */,
                              float scale /* = 1.0f */,
                              VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        CARBON_PRECONDITION(input, "input buffer must be valid");
        CARBON_PRECONDITION(output, "output buffer must be valid");

        const int inputWidth = input->Width();
        const int inputHeight = input->Height();
        const int outputWidth = output->Width();
        const int outputHeight = output->Height();
        const float scaleX = inputWidth / static_cast<float>(outputWidth);
        const float scaleY = inputHeight / static_cast<float>(outputHeight);

        ResizePushConstants ubo;
        ubo.outputWidth = outputWidth;
        ubo.outputHeight = outputHeight;
        ubo.maxX = inputWidth - 1;
        ubo.maxY = inputHeight - 1;
        ubo.scaleX = scaleX;
        ubo.scaleY = scaleY;
        ubo.scale = scale;
        ubo.interpolationMode = static_cast<int>(interpMode);

        ResizeReflection::DescSet0_Update dsu{
            input->ManagedBuffer()->Buffer(),
            output->ManagedBuffer()->Buffer()
        };

        const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(outputWidth / static_cast<float>(ResizeComputeThreadSizeX)));
        const uint32_t numGroupsY = static_cast<uint32_t>(std::ceil(outputHeight / static_cast<float>(ResizeComputeThreadSizeY)));

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}
