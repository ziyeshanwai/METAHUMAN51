// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>

#include <nls/geometry/PixelFormat.h>
#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanDescriptorSet.h>
#include <nls/vulkan/common/VulkanDescriptorSetLayout.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/common/VulkanTextureSampler.h>

namespace epic {
namespace nls {

/**
 * Class that contains a texture, the sampler, the layout, and the descriptor set.
 */
class VulkanAllInOneTexture
{
public:
    ~VulkanAllInOneTexture()
    {}

    static std::shared_ptr<VulkanAllInOneTexture> Create(const std::shared_ptr<VulkanDevice>& device);

    static std::shared_ptr<VulkanDescriptorSetLayout> CreateLayout(const std::shared_ptr<VulkanDevice>& device);

    void SetTextureImage(std::shared_ptr<VulkanManagedImage> texImage);

    void LoadTextureImage(const float* imgData, int width, int height, PixelFormat pixelFormat);

    void Create(int width, int height, VkFormat format, VkComponentMapping componentMapping = {});

    /**
     * Copies the data from the device buffer to the texture. This requires the buffer to have
     * the exact same size in bytes as the texture.
     */
    void CopyFromDeviceBuffer(std::shared_ptr<VulkanManagedBuffer> buffer);

    const VkDescriptorSetLayout& Layout() const { return m_layout->Layout(); }
    const VkDescriptorSet& DescriptorSet() const { return m_descriptorSet->DescriptorSet(); }
    const VulkanManagedImage& Image() const { return *m_texImage; }

private:
    VulkanAllInOneTexture(const std::shared_ptr<VulkanDevice>& device);

    VulkanAllInOneTexture(const VulkanAllInOneTexture&) = delete;
    VulkanAllInOneTexture& operator=(const VulkanAllInOneTexture&) = delete;
    VulkanAllInOneTexture(const VulkanAllInOneTexture&&) = delete;
    VulkanAllInOneTexture& operator=(const VulkanAllInOneTexture&&) = delete;

    void UpdateDescriptorSet();

private:
    std::shared_ptr<VulkanDevice> m_device;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;
    std::shared_ptr<VulkanDescriptorSetLayout> m_layout;
    std::shared_ptr<VulkanDescriptorSet> m_descriptorSet;
    std::shared_ptr<VulkanManagedImage> m_texImage;
    std::unique_ptr<VulkanTextureSampler> m_texSampler;

};


} // namespace nls
} //namespace epic
