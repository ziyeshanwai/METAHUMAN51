// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

#include <carbon/Common.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/compute/common/VulkanComputeContext.h>

namespace epic::nls {

VulkanComputeShaderPipeline::VulkanComputeShaderPipeline(std::shared_ptr<VulkanComputeContext> vulkanComputeContext, const unsigned char* spirv, size_t spirvSize, const std::vector<std::shared_ptr<VulkanDescriptorSetLayout>>& descriptorSetLayouts, size_t uboSize)
    : m_computeContext(vulkanComputeContext)
    , m_pipelineLayout(VK_NULL_HANDLE)
    , m_computePipeline(VK_NULL_HANDLE)
    , m_uboSize(uboSize)
{
    std::unique_ptr<VulkanShaderModule> compShaderModule = VulkanShaderModule::Create(m_computeContext->Device(), spirv, spirvSize);

    VkPipelineShaderStageCreateInfo compShaderStageInfo = {};
    compShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    compShaderStageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    compShaderStageInfo.module = compShaderModule->ShaderModule();
    compShaderStageInfo.pName = "main";

    VkPushConstantRange pushConstantRange = {};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = uint32_t(m_uboSize);


    VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;

    m_dsetLayouts.reserve(descriptorSetLayouts.size());

    // FIXME: @@ Discuss this
    for (const std::shared_ptr<VulkanDescriptorSetLayout>& descriptorSetLayout : descriptorSetLayouts) {
       auto dsetLayout = const_cast<VkDescriptorSetLayout>(descriptorSetLayout->Layout());
       m_dsetLayouts.push_back(dsetLayout);
    }

    pipelineLayoutInfo.pSetLayouts = m_dsetLayouts.data();
    pipelineLayoutInfo.setLayoutCount = static_cast<uint32_t>(m_dsetLayouts.size());
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;
    pipelineLayoutInfo.pushConstantRangeCount = (m_uboSize > 0 ? 1 : 0);

    if (vkCreatePipelineLayout(m_computeContext->Device()->Device(), &pipelineLayoutInfo, nullptr, &m_pipelineLayout) != VK_SUCCESS) {
        CARBON_CRITICAL("failed to create pipeline layout");
    }

    VkComputePipelineCreateInfo pipelineCreateInfo = {};
    pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineCreateInfo.stage = compShaderStageInfo;
    pipelineCreateInfo.layout = m_pipelineLayout;

    if (vkCreateComputePipelines(m_computeContext->Device()->Device(), VK_NULL_HANDLE, 1, &pipelineCreateInfo, NULL, &m_computePipeline) != VK_SUCCESS) {
        CARBON_CRITICAL("failed to create compute pipeline");
    }
}

VulkanComputeShaderPipeline::~VulkanComputeShaderPipeline()
{
    if (m_computePipeline) {
        vkDestroyPipeline(m_computeContext->Device()->Device(), m_computePipeline, nullptr);
        m_computePipeline = VK_NULL_HANDLE;
    }
    if (m_pipelineLayout) {
        vkDestroyPipelineLayout(m_computeContext->Device()->Device(), m_pipelineLayout, nullptr);
        m_pipelineLayout = VK_NULL_HANDLE;
    }
}

std::shared_ptr<VulkanComputeShaderPipeline> VulkanComputeShaderPipeline::Create(std::shared_ptr<VulkanComputeContext> computeContext, const unsigned char* spirv, size_t spirvSize, const std::vector<std::shared_ptr<VulkanDescriptorSetLayout>>& descriptorSetLayouts, size_t uboSize)
{
    return std::shared_ptr<VulkanComputeShaderPipeline>(new VulkanComputeShaderPipeline(computeContext, spirv, spirvSize, descriptorSetLayouts, uboSize));
}

} // namespace epic::nls
