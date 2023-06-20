// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/common/VulkanAllInOneTexture.h>

namespace epic::nls {

std::shared_ptr<VulkanAllInOneTexture> VulkanAllInOneTexture::Create(const std::shared_ptr<VulkanDevice>& device)
{
    std::shared_ptr<VulkanAllInOneTexture> ptr(new VulkanAllInOneTexture(device));
    return ptr;
}

std::shared_ptr<VulkanDescriptorSetLayout> VulkanAllInOneTexture::CreateLayout(const std::shared_ptr<VulkanDevice>& device)
{
    return VulkanDescriptorSetLayout::CreateTextureDescriptorSetLayout(device, /*binding=*/0, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT);
}

void VulkanAllInOneTexture::Create(int width, int height, VkFormat format, VkComponentMapping componentMapping)
{
    if (!m_texImage || m_texImage->Width() != width || m_texImage->Height() != height || m_texImage->Format() != format) {
        m_texImage = m_vulkanMemory->createImageAndView(width, height, format, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_SAMPLE_COUNT_1_BIT, componentMapping);
        UpdateDescriptorSet();
    }
}

void VulkanAllInOneTexture::SetTextureImage(std::shared_ptr<VulkanManagedImage> texImage)
{
    m_texImage = texImage;
    UpdateDescriptorSet();
}


void VulkanAllInOneTexture::LoadTextureImage(const float* imgData, int width, int height, PixelFormat pixelFormat)
{
    Create(width, height, VK_FORMAT_R32G32B32A32_SFLOAT);

    VkDeviceSize imageSize = width * height * 4 * sizeof(float);
    std::unique_ptr<VulkanManagedBuffer> stagingBuffer = m_vulkanMemory->createBuffer(imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    void* data;
    vkMapMemory(stagingBuffer->Device(), stagingBuffer->DeviceMemory(), 0, imageSize, 0, &data);
    Convert(imgData, (float*)data, width, height, pixelFormat, PixelFormat::RGBA);
    vkUnmapMemory(stagingBuffer->Device(), stagingBuffer->DeviceMemory());

    VkCommandBuffer commandBuffer = m_vulkanMemory->TransientCommandPool()->BeginSingleTimeCommands();
    m_vulkanMemory->transitionImageLayout(commandBuffer, m_texImage->Image(), VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, m_texImage->Aspect());
    m_vulkanMemory->copyBufferToImage(commandBuffer, stagingBuffer->Buffer(), m_texImage->Image(), static_cast<uint32_t>(width), static_cast<uint32_t>(height));
    m_vulkanMemory->transitionImageLayout(commandBuffer, m_texImage->Image(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, m_texImage->Aspect());
    m_vulkanMemory->TransientCommandPool()->EndSingleTimeCommands(commandBuffer);
}

void VulkanAllInOneTexture::CopyFromDeviceBuffer(std::shared_ptr<VulkanManagedBuffer> buffer)
{
    VkCommandBuffer commandBuffer = m_vulkanMemory->TransientCommandPool()->BeginSingleTimeCommands();
    m_vulkanMemory->transitionImageLayout(commandBuffer, m_texImage->Image(), VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, m_texImage->Aspect());
    m_vulkanMemory->copyBufferToImage(commandBuffer, buffer->Buffer(), m_texImage->Image(), m_texImage->Width(), m_texImage->Height());
    m_vulkanMemory->transitionImageLayout(commandBuffer, m_texImage->Image(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, m_texImage->Aspect());
    m_vulkanMemory->TransientCommandPool()->EndSingleTimeCommands(commandBuffer);
}

VulkanAllInOneTexture::VulkanAllInOneTexture(const std::shared_ptr<VulkanDevice>& device)
    : m_device(device)
{
    m_vulkanMemory = VulkanMemory::Create(m_device);
    m_layout = CreateLayout(m_device);
    m_descriptorSet = VulkanDescriptorSet::CreateDescriptorSet(m_layout);
    m_texSampler = VulkanTextureSampler::CreateBasicSampler(m_device);
}

void VulkanAllInOneTexture::UpdateDescriptorSet()
{
    std::vector<VkWriteDescriptorSet> descriptorWrites;
    VkWriteDescriptorSet descriptorWrite = {};

    VkDescriptorImageInfo imageInfo = {};
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfo.imageView = m_texImage->ImageView();
    imageInfo.sampler = m_texSampler->Sampler();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_descriptorSet->DescriptorSet();
    descriptorWrite.dstBinding = 0;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pImageInfo = &imageInfo;
    descriptorWrites.push_back(descriptorWrite);

    vkUpdateDescriptorSets(m_device->Device(), static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
}


} // namespace epic::nls
