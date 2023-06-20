// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDevice.h>

namespace epic::nls {
    /**
     * Managed descriptor set layout with a convenience function to create the layout for a single texture.
     */
    class VulkanDescriptorSetLayout {
        public:
            ~VulkanDescriptorSetLayout() {
                if (m_descriptorSetLayout) {
                    vkDestroyDescriptorSetLayout(m_device->Device(), m_descriptorSetLayout, nullptr);
                    m_descriptorSetLayout = VK_NULL_HANDLE;
                }
            }

            static std::shared_ptr<VulkanDescriptorSetLayout> CreateDescriptorSetLayout(std::shared_ptr<VulkanDevice> device,
                                                                                        const std::vector<VkDescriptorSetLayoutBinding>& bindings, 
                                                                                        uint32_t createFlags = 0)
            {
                return std::shared_ptr<VulkanDescriptorSetLayout>(new VulkanDescriptorSetLayout(device, bindings, createFlags));
            }

            static std::shared_ptr<VulkanDescriptorSetLayout> CreatePushDescriptorSetLayout(std::shared_ptr<VulkanDevice> device,
                                                                                            const std::vector<VkDescriptorSetLayoutBinding>& bindings)
            {
                return std::shared_ptr<VulkanDescriptorSetLayout>(new VulkanDescriptorSetLayout(device, bindings, VK_DESCRIPTOR_SET_LAYOUT_CREATE_PUSH_DESCRIPTOR_BIT_KHR));
            }

            static std::shared_ptr<VulkanDescriptorSetLayout> CreateTextureDescriptorSetLayout(
                std::shared_ptr<VulkanDevice> device,
                int binding = 0,
                VkShaderStageFlags shaderStageFlags = VK_SHADER_STAGE_FRAGMENT_BIT) {
                VkDescriptorSetLayoutBinding samplerLayoutBinding = {};
                samplerLayoutBinding.binding = binding;
                samplerLayoutBinding.descriptorCount = 1;
                samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                samplerLayoutBinding.stageFlags = shaderStageFlags;
                samplerLayoutBinding.pImmutableSamplers = nullptr;

                return CreateDescriptorSetLayout(device, {samplerLayoutBinding});
            }

            std::shared_ptr<VulkanDevice> Device() const {
                return m_device;
            }

            const VkDescriptorSetLayout& Layout() const {
                return m_descriptorSetLayout;
            }

        private:
            VulkanDescriptorSetLayout(const std::shared_ptr<VulkanDevice>& device,
                                      const std::vector<VkDescriptorSetLayoutBinding>& bindings,
                                      uint32_t createFlags)
                : m_device(device) {
                VkDescriptorSetLayoutCreateInfo layoutInfo = {};
                layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layoutInfo.flags = createFlags;
                layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
                layoutInfo.pBindings = bindings.data();
                
                if (vkCreateDescriptorSetLayout(device->Device(), &layoutInfo, nullptr,
                                                &m_descriptorSetLayout) != VK_SUCCESS) {
                    throw std::runtime_error("failed to create descriptor set layout!");
                }
            }

            VulkanDescriptorSetLayout(const VulkanDescriptorSetLayout&) = delete;
            VulkanDescriptorSetLayout& operator=(const VulkanDescriptorSetLayout&) = delete;

        private:
            std::shared_ptr<VulkanDevice> m_device;
            VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
    };
}  // namespace epic::nls
