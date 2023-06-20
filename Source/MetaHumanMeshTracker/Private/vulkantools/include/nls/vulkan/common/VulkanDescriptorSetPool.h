// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDevice.h>

#include <map>
#include <mutex>

namespace epic::nls {

class VulkanDescriptorSetPool
{
public:
    ~VulkanDescriptorSetPool() {
        if (m_descriptorPool) {
            vkDestroyDescriptorPool(m_device->Device(), m_descriptorPool, nullptr);
            m_descriptorPool = VK_NULL_HANDLE;
        }
    }

    static std::shared_ptr<VulkanDescriptorSetPool> Create(std::shared_ptr<VulkanDevice> device)
    {
        static std::mutex mutex;
        std::lock_guard lock(mutex);

        static std::map<uint64_t, std::weak_ptr<VulkanDescriptorSetPool>> poolPerDevice;

        uint64_t deviceID = uint64_t(device.get());
        std::shared_ptr<VulkanDescriptorSetPool> ptr;

        auto it = poolPerDevice.find(deviceID);
        if (it != poolPerDevice.end()) {
            ptr = it->second.lock();
        }
        if (!ptr) {
            ptr =  std::shared_ptr<VulkanDescriptorSetPool>(new VulkanDescriptorSetPool());
            ptr->m_device = device;

            // Create Descriptor Pool
            VkDescriptorPoolSize pool_sizes[] =
            {
                { VK_DESCRIPTOR_TYPE_SAMPLER, 1000 },
                { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000 },
                { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000 },
                { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000 },
                { VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000 },
                { VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000 },
                { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000 },
                { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000 },
                { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000 },
                { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000 },
                { VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000 }
            };
            const int numPoolSizes = int(sizeof(pool_sizes))/int(sizeof(pool_sizes[0]));
            VkDescriptorPoolCreateInfo pool_info = {};
            pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
            pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
            pool_info.maxSets = 1000 * numPoolSizes;
            pool_info.poolSizeCount = (uint32_t)numPoolSizes;
            pool_info.pPoolSizes = pool_sizes;
            vkCreateDescriptorPool(device->Device(), &pool_info, nullptr, &ptr->m_descriptorPool);

            poolPerDevice[deviceID] = ptr;
        }

        return ptr;
    }

    std::shared_ptr<VulkanDevice> Device() const { return m_device; }
    VkDescriptorPool DescriptorPool() const { return m_descriptorPool; }

private:
    VulkanDescriptorSetPool() = default;

private:
    std::shared_ptr<VulkanDevice> m_device;
    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;
};


} // namespace epic::nls
