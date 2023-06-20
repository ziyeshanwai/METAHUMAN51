// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanFlowConfidence.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_FlowConfidence.comp.spv.h"

#include "shaders/FlowConfidence.comp.h"
#include "shaders/FlowConfidence.comp.refl.h"

#include <cmath>

namespace epic::nls
{
    bool VulkanFlowConfidence::Init()
    {
        return Kernel::Init<FlowConfidenceReflection, FlowConfidencePushConstants>(SPV_FlowConfidence_comp_spv, sizeof(SPV_FlowConfidence_comp_spv));
    }

    void VulkanFlowConfidence::FlowConfidence(const std::shared_ptr<VulkanComputeBuffer>& flowU,
                                              const std::shared_ptr<VulkanComputeBuffer>& flowV,
                                              const std::shared_ptr<VulkanComputeBuffer>& invFlowU,
                                              const std::shared_ptr<VulkanComputeBuffer>& invFlowV,
                                              const std::shared_ptr<VulkanComputeBuffer>& outputConfidence,
                                              InterpolationMode interpMode /* = InterpolationMode:InterpolationMode::Linear */,
                                              VulkanComputeContext::HBatch batch /* = nullptr */)
    {
        CARBON_PRECONDITION(flowU, "flowU buffer must be valid");
        CARBON_PRECONDITION(flowV, "flowV buffer must be valid");
        CARBON_PRECONDITION(invFlowU, "invFlowU buffer must be valid");
        CARBON_PRECONDITION(invFlowV, "invFlowV buffer must be valid");
        CARBON_PRECONDITION(outputConfidence, "outputConfidence buffer must be valid");
        CARBON_PRECONDITION(flowU->Width() == flowV->Width() &&
                            flowU->Height() == flowV->Height() &&
                            flowU->Width() == invFlowU->Width() &&
                            flowU->Height() == invFlowU->Height() &&
                            flowV->Width() == flowV->Width() &&
                            flowV->Height() == flowV->Height() &&
                            flowU->Width() == outputConfidence->Width() &&
                            flowU->Height() == outputConfidence->Height(),
                            "flowU, flowV, invFlowU, invFlowV and outputConfidence buffers must all have the same size");

        const int width = flowU->Width();
        const int height = flowU->Height();

        FlowConfidencePushConstants ubo;
        ubo.width = width;
        ubo.height = height;
        ubo.interpolationMode = static_cast<int>(interpMode);

        FlowConfidenceReflection::DescSet0_Update dsu{
            flowU->ManagedBuffer()->Buffer(),
            flowV->ManagedBuffer()->Buffer(),
            invFlowU->ManagedBuffer()->Buffer(),
            invFlowV->ManagedBuffer()->Buffer(),
            outputConfidence->ManagedBuffer()->Buffer()
        };

        const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(width / static_cast<float>(FlowConfidenceThreadSizeX)));
        const uint32_t numGroupsY = static_cast<uint32_t>(std::ceil(height / static_cast<float>(FlowConfidenceThreadSizeY)));

        Run(batch,
            { numGroupsX, numGroupsY, 1 },
            dsu.writes,
            dsu.nWrites,
            &ubo);
    }
}
