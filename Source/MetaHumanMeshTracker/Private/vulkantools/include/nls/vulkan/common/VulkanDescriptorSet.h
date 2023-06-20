// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanDescriptorSetLayout.h>
#include <nls/vulkan/common/VulkanDescriptorSetPool.h>

namespace epic {
namespace nls {

/**
 * Creates a descriptorset.
 */
class VulkanDescriptorSet
{
public:
    ~VulkanDescriptorSet()
    {
        if (m_descriptorSet) {
            vkFreeDescriptorSets(m_descriptorSetPool->Device()->Device(), m_descriptorSetPool->DescriptorPool(), 1, &m_descriptorSet);
        }
    }

    static std::shared_ptr<VulkanDescriptorSet> CreateDescriptorSet(std::shared_ptr<VulkanDescriptorSetLayout> layout) {

        if (!layout) {
            return nullptr;

        }
        return std::shared_ptr<VulkanDescriptorSet>(new VulkanDescriptorSet(layout));
    }

    const VkDescriptorSet& DescriptorSet() const { return m_descriptorSet; }


private:
    VulkanDescriptorSet(std::shared_ptr<VulkanDescriptorSetLayout> layout)
    {
        m_descriptorSetLayout = layout;
        m_descriptorSetPool = VulkanDescriptorSetPool::Create(layout->Device());

        VkDescriptorSetAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = m_descriptorSetPool->DescriptorPool();
        allocInfo.pSetLayouts = &(m_descriptorSetLayout->Layout());
        allocInfo.descriptorSetCount = 1;

        if (vkAllocateDescriptorSets(m_descriptorSetPool->Device()->Device(), &allocInfo, &m_descriptorSet) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate descriptor sets!");
        }
    }

    VulkanDescriptorSet(const VulkanDescriptorSet&) = delete;
    VulkanDescriptorSet& operator=(const VulkanDescriptorSet&) = delete;

private:
    std::shared_ptr<VulkanDescriptorSetPool> m_descriptorSetPool;
    std::shared_ptr<VulkanDescriptorSetLayout> m_descriptorSetLayout;
    VkDescriptorSet m_descriptorSet = VK_NULL_HANDLE;
};


} // namespace nls
} //namespace epic
