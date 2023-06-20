// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDescriptorSetLayout.h>

namespace epic {
namespace nls {

class VulkanComputeContext;

class VulkanComputeShaderPipeline {

public:
    ~VulkanComputeShaderPipeline();

    static std::shared_ptr<VulkanComputeShaderPipeline> Create(std::shared_ptr<VulkanComputeContext> computeContext,
                                                               const unsigned char* spirv,
                                                               size_t spirvSize,
                                                               const std::vector<std::shared_ptr<VulkanDescriptorSetLayout>>& descriptorSetLayouts,
                                                               size_t uboSize);

    VkPipelineLayout PipelineLayout() const { return m_pipelineLayout; }
    VkPipeline Pipeline() const { return m_computePipeline; }
    const std::vector<VkDescriptorSetLayout>& DescSetLayouts() const { return m_dsetLayouts; }
    size_t UboSize() const { return m_uboSize; }

private:
    VulkanComputeShaderPipeline(std::shared_ptr<VulkanComputeContext> computeContext,
                                const unsigned char* spirv,
                                size_t spirvSize,
                                const std::vector<std::shared_ptr<VulkanDescriptorSetLayout>>& descriptorSetLayouts,
                                size_t uboSize);

private:
    std::shared_ptr<VulkanComputeContext> m_computeContext;
    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_computePipeline = VK_NULL_HANDLE;
    std::vector<VkDescriptorSetLayout> m_dsetLayouts;
    size_t m_uboSize = 0;
};



} //namespace nls
} //namespace epic
