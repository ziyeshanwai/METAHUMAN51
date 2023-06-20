// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>

#include <carbon/common/Log.h>
#include <carbon/utils/Timer.h>

namespace epic {
namespace nls {

VulkanComputeBuffer::VulkanComputeBuffer(int width, int height, std::shared_ptr<VulkanComputeContext> computeContext)
    : m_width(width)
    , m_height(height)
    , m_fullWidth(width)
    , m_fullHeight(height)
    , m_computeContext(computeContext) {
    m_vulkanBuffer = computeContext->Memory()->createBuffer(FullSizeInBytes(),
                                                            VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                                            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    m_descriptorSet = VulkanDescriptorSet::CreateDescriptorSet(m_computeContext->BufferDescriptorSetLayout());
    UpdateDescriptorSet();
}

void VulkanComputeBuffer::UpdateDescriptorSet() {
    std::vector<VkWriteDescriptorSet> descriptorWrites;
    VkWriteDescriptorSet descriptorWrite = {};

    VkDescriptorBufferInfo bufferInfo = {};
    bufferInfo.buffer = m_vulkanBuffer->Buffer();
    bufferInfo.offset = m_vulkanBuffer->Offset();
    bufferInfo.range = m_vulkanBuffer->Size();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_descriptorSet->DescriptorSet();
    descriptorWrite.dstBinding = 0;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfo;
    descriptorWrites.push_back(descriptorWrite);

    vkUpdateDescriptorSets(m_computeContext->Device()->Device(),
                           static_cast<uint32_t>(descriptorWrites.size()),
                           descriptorWrites.data(),
                           0,
                           nullptr);
}

void VulkanComputeBuffer::CopyToDevice(const void* data, size_t size) {
    if (m_vulkanBuffer->Size() != size) {
        CARBON_CRITICAL("data size ({}) does not match size ({}) of vulkan compute image", size, m_vulkanBuffer->Size());
    }
    m_computeContext->Memory()->updateBuffer(m_vulkanBuffer.get(), data, size);
}

void VulkanComputeBuffer::CopyFromDevice(void* data, size_t size) {
    if (m_vulkanBuffer->Size() != size) {
        CARBON_CRITICAL("data size ({}) does not match size ({}) of vulkan compute image ({} {})",
                        size,
                        m_vulkanBuffer->Size(),
                        Width(),
                        Height());
    }

    m_computeContext->Memory()->CopyFromDevice(data, size, m_vulkanBuffer->Buffer());
}

void VulkanComputeBuffer::CopySubsetToDevice(const void* data, size_t offset, size_t size) {
    if (m_vulkanBuffer->Size() < offset + size) {
        CARBON_CRITICAL("buffer size ({}) is too small for offset {} and size {} ", m_vulkanBuffer->Size(), offset, size);
    }

    m_computeContext->Memory()->CopySubsetToDevice(data, offset, size, m_vulkanBuffer->Buffer(), m_computeContext->TransientCommandPool());
}

void VulkanComputeBuffer::CopySubsetFromDevice(void* data, size_t offset, size_t size) {
    if (m_vulkanBuffer->Size() < offset + size) {
        CARBON_CRITICAL("buffer size ({}) is too small for offset {} and size {} ", m_vulkanBuffer->Size(), offset, size);
    }
    m_computeContext->Memory()->CopySubsetFromDevice(data, offset, size, m_vulkanBuffer->Buffer(), m_computeContext->TransientCommandPool());
}

void VulkanComputeBuffer::CopyTo(std::shared_ptr<VulkanComputeBuffer> dst)
{
    CARBON_PRECONDITION(SizeInBytes() <= dst->SizeInBytes(), "dst is too small");
    m_computeContext->Memory()->copyBuffer(ManagedBuffer()->Buffer(), dst->ManagedBuffer()->Buffer(), FullSizeInBytes());
}

void VulkanComputeBuffer::Resize(int newWidth, int newHeight)
{
    CARBON_PRECONDITION(newWidth * newHeight <= MaximumWidth() * MaximumHeight(), "invalid width and height - needs to be smaller or equal to original size of allocation");
    m_width = newWidth;
    m_height = newHeight;
}

}  // namespace nls
}  // namespace epic
