// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDevice.h>

#include <string>

namespace epic {
namespace nls {

class VulkanShaderModule
{
public:
    ~VulkanShaderModule()
    {
        if (m_shaderModule) {
            vkDestroyShaderModule(m_vulkanDevice->Device(), m_shaderModule, nullptr);
        }
    }

    static std::unique_ptr<VulkanShaderModule> Create(const std::shared_ptr<VulkanDevice>& device, const std::string& code)
    {
        VkShaderModuleCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());
        VkShaderModule shaderModule;
        if (vkCreateShaderModule(device->Device(), &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
            throw std::runtime_error("failed to create shader module!");
        }

        std::unique_ptr<VulkanShaderModule> instance(new VulkanShaderModule());
        instance->m_vulkanDevice = device;
        instance->m_shaderModule = shaderModule;
        return instance;
    }

    static std::unique_ptr<VulkanShaderModule> Create(const std::shared_ptr<VulkanDevice>& device, const unsigned char* code, size_t codeSize)
    {
        VkShaderModuleCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = codeSize;
        createInfo.pCode = reinterpret_cast<const uint32_t*>(code);
        VkShaderModule shaderModule;
        if (vkCreateShaderModule(device->Device(), &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
            throw std::runtime_error("failed to create shader module!");
        }

        std::unique_ptr<VulkanShaderModule> instance(new VulkanShaderModule());
        instance->m_vulkanDevice = device;
        instance->m_shaderModule = shaderModule;
        return instance;
    }

    VkShaderModule ShaderModule() const { return m_shaderModule; }

private:
    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    VkShaderModule m_shaderModule;
};

} // namespace nls
} //namespace epic
