// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanWeightedAverage.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_WeightedAverage.comp.spv.h"

#include "shaders/WeightedAverage.comp.h"
#include "shaders/WeightedAverage.comp.refl.h"

#include <cmath>

namespace epic::nls
{
    bool VulkanWeightedAverage::Init()
    {
        return Kernel::Init<WeightedAverageReflection, WeightedAveragePushConstants>(SPV_WeightedAverage_comp_spv, sizeof(SPV_WeightedAverage_comp_spv));
    }

    void VulkanWeightedAverage::WeightedAverage(const std::shared_ptr<VulkanComputeBuffer>& inputA,
                                                const std::shared_ptr<VulkanComputeBuffer>& inputB,
                                                const std::shared_ptr<VulkanComputeBuffer>& output,
                                                float weightA,
                                                float weightB,
                                                float delta /* = 0.0f */,
                                                VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        CARBON_PRECONDITION(inputA, "inputA buffer must be valid");
        CARBON_PRECONDITION(inputB, "inputB buffer must be valid");
        CARBON_PRECONDITION(output, "output buffer must be valid");

        const int outputWidth = output->Width();
        const int outputHeight = output->Height();

        WeightedAveragePushConstants ubo;
        ubo.width = outputWidth;
        ubo.height = outputHeight;
        ubo.weightA = weightA;
        ubo.weightB = weightB;
        ubo.delta = delta;

        WeightedAverageReflection::DescSet0_Update dsu{
            inputA->ManagedBuffer()->Buffer(),
            inputB->ManagedBuffer()->Buffer(),
            output->ManagedBuffer()->Buffer()
        };

        const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(outputWidth / static_cast<float>(WeightedAverageComputeThreadSizeX)));
        const uint32_t numGroupsY = static_cast<uint32_t>(std::ceil(outputHeight / static_cast<float>(WeightedAverageComputeThreadSizeY)));

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}
