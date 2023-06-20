// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDevice.h>

namespace epic {
namespace nls {


class VulkanCommandPool
{
public:
    ~VulkanCommandPool() {
        if (m_commandPool) {
            vkDestroyCommandPool(m_device->Device(), m_commandPool, nullptr);
        }
    }

    static std::unique_ptr<VulkanCommandPool> Create(std::shared_ptr<VulkanDevice> device, bool transient)
    {
        std::unique_ptr<VulkanCommandPool> ptr(new VulkanCommandPool);
        ptr->m_device = device;

        VkCommandPoolCreateInfo poolInfo = {};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolInfo.queueFamilyIndex = device->AllFamily().value();
        poolInfo.flags = transient ? VK_COMMAND_POOL_CREATE_TRANSIENT_BIT : 0;

        if (vkCreateCommandPool(device->Device(), &poolInfo, nullptr, &ptr->m_commandPool) != VK_SUCCESS) {
            throw std::runtime_error("failed to create command pool!");
        }

        return ptr;
    }

    VkCommandPool CommandPool() const { return m_commandPool; }

    VkCommandBuffer BeginSingleTimeCommands() {
        VkCommandBufferAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = CommandPool();
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBuffer;
        vkAllocateCommandBuffers(m_device->Device(), &allocInfo, &commandBuffer);

        VkCommandBufferBeginInfo beginInfo = {};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        vkBeginCommandBuffer(commandBuffer, &beginInfo);

        return commandBuffer;
    }

    void EndSingleTimeCommands(VkCommandBuffer commandBuffer) {
        vkEndCommandBuffer(commandBuffer);

        VkSubmitInfo submitInfo = {};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;

        {
            std::lock_guard lock(m_device->Mutex());
            // VkQueue queue = m_compute ? m_device->ComputeQueue() : m_device->GraphicsQueue();
            VkQueue queue = m_device->AllQueue();
            vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
            vkQueueWaitIdle(queue);
        }


        vkFreeCommandBuffers(m_device->Device(), CommandPool(), 1, &commandBuffer);
    }

private:
    VulkanCommandPool() = default;

private:
    std::shared_ptr<VulkanDevice> m_device;
    VkCommandPool m_commandPool;
};


} // namespace nls
} //namespace epic
