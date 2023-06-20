// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDevice.h>

namespace epic {
namespace nls {

class VulkanTextureSampler
{
public:
    ~VulkanTextureSampler()
    {
        if (m_textureSampler) {
            vkDestroySampler(m_device->Device(), m_textureSampler, nullptr);
        }
    }

    static std::unique_ptr<VulkanTextureSampler> CreateBasicSampler(std::shared_ptr<VulkanDevice> device, VkSamplerAddressMode addressMode = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE) {
        VkSamplerCreateInfo samplerInfo = {};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.addressModeU = addressMode;
        samplerInfo.addressModeV = addressMode;
        samplerInfo.addressModeW = addressMode;
        samplerInfo.anisotropyEnable = VK_FALSE; //VK_TRUE;
        samplerInfo.maxAnisotropy = 16;
        samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
        samplerInfo.unnormalizedCoordinates = VK_FALSE;
        samplerInfo.compareEnable = VK_FALSE;
        samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerInfo.mipLodBias = 0.0f;
        samplerInfo.minLod = 0.0f;
        samplerInfo.maxLod = 0.0f;
        return Create(device, samplerInfo);
    }


    static std::unique_ptr<VulkanTextureSampler> Create(std::shared_ptr<VulkanDevice> device, VkSamplerCreateInfo samplerInfo) {
        std::unique_ptr<VulkanTextureSampler> obj = std::unique_ptr<VulkanTextureSampler>(new VulkanTextureSampler());
        if (vkCreateSampler(device->Device(), &samplerInfo, nullptr, &obj->m_textureSampler) != VK_SUCCESS) {
            throw std::runtime_error("failed to create texture sampler!");
        }
        obj->m_device = device;
        return obj;
    }

    VkSampler Sampler() const { return m_textureSampler; }

private:
    VulkanTextureSampler() = default;

private:
    std::shared_ptr<VulkanDevice> m_device;
    VkSampler m_textureSampler = VK_NULL_HANDLE;
};

} // namespace nls
} //namespace epic
