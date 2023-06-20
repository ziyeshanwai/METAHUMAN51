// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/common/VulkanComputeImage.h>

#include <carbon/Common.h>

namespace epic {
namespace nls {

VulkanComputeImage::VulkanComputeImage(int width, int height, std::shared_ptr<VulkanComputeContext> computeContext)
        : m_width(width)
        , m_height(height)
        , m_computeContext(computeContext)
{
    m_vulkanImage = computeContext->Memory()->createImageAndView(width, height, VK_FORMAT_R32_SFLOAT, VK_IMAGE_TILING_OPTIMAL,  VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_GENERAL, VK_SAMPLE_COUNT_1_BIT);
    m_descriptorSet = VulkanDescriptorSet::CreateDescriptorSet(m_computeContext->ImageDescriptorSetLayout());
    UpdateDescriptorSet();
}


void VulkanComputeImage::UpdateDescriptorSet()
{
    std::vector<VkWriteDescriptorSet> descriptorWrites;
    VkWriteDescriptorSet descriptorWrite = {};

    VkDescriptorImageInfo imageOutInfo = {};
    imageOutInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    imageOutInfo.imageView = m_vulkanImage->ImageView();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_descriptorSet->DescriptorSet();
    descriptorWrite.dstBinding = 0;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pImageInfo = &imageOutInfo;
    descriptorWrites.push_back(descriptorWrite);

    vkUpdateDescriptorSets(m_computeContext->Device()->Device(), static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
}


void VulkanComputeImage::CopyToDevice(const void* data, size_t size)
{
    VkDeviceSize imageSize = Width() * Height() * sizeof(float);

    if (imageSize != size) {
        CARBON_CRITICAL("data size ({}) does not match size ({}) of vulkan compute image", size, imageSize);
    }

    std::unique_ptr<VulkanManagedBuffer> stagingBuffer = m_computeContext->Memory()->createBuffer(imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    void* stagingData;
    vkMapMemory(stagingBuffer->Device(), stagingBuffer->DeviceMemory(), 0, imageSize, 0, &stagingData);
    memcpy(stagingData, data, size);
    vkUnmapMemory(stagingBuffer->Device(), stagingBuffer->DeviceMemory());

    VkCommandBuffer buffer = m_computeContext->TransientCommandPool()->BeginSingleTimeCommands();
    m_computeContext->Memory()->transitionImageLayout(buffer, m_vulkanImage->Image(), VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, m_vulkanImage->Aspect());
    m_computeContext->Memory()->copyBufferToImage(buffer, stagingBuffer->Buffer(), m_vulkanImage->Image(), static_cast<uint32_t>(Width()), static_cast<uint32_t>(Height()));
    m_computeContext->Memory()->transitionImageLayout(buffer, m_vulkanImage->Image(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL, m_vulkanImage->Aspect());
    m_computeContext->TransientCommandPool()->EndSingleTimeCommands(buffer);
}

void VulkanComputeImage::CopyFromDevice(void* data, size_t size)
{
    VkDeviceSize imageSize = Width() * Height() * sizeof(float);

    if (imageSize != size) {
        CARBON_CRITICAL("data size ({}) does not match size ({}) of vulkan compute image", size, imageSize);
    }

    std::unique_ptr<VulkanManagedBuffer> stagingBuffer = m_computeContext->Memory()->createBuffer(imageSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    VkCommandBuffer buffer = m_computeContext->TransientCommandPool()->BeginSingleTimeCommands();
    m_computeContext->Memory()->transitionImageLayout(buffer, m_vulkanImage->Image(), VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, m_vulkanImage->Aspect());
    m_computeContext->Memory()->copyImageToBuffer(buffer, m_vulkanImage->Image(), stagingBuffer->Buffer(), m_vulkanImage->Width(), m_vulkanImage->Height());
    m_computeContext->Memory()->transitionImageLayout(buffer, m_vulkanImage->Image(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL, m_vulkanImage->Aspect());
    m_computeContext->TransientCommandPool()->EndSingleTimeCommands(buffer);

    void* stagingData = nullptr;
    vkMapMemory(m_computeContext->Device()->Device(), stagingBuffer->DeviceMemory(),  stagingBuffer->Offset(), stagingBuffer->Size(), 0, &stagingData);
    memcpy(data, stagingData, imageSize);
    vkUnmapMemory(m_computeContext->Device()->Device(), stagingBuffer->DeviceMemory());
}


} //namespace nls
} //namespace epic
