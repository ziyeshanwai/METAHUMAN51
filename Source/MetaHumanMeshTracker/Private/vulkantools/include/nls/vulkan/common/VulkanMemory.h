// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/vulkan/common/VulkanCommandPool.h>
#include <nls/vulkan/common/VulkanDevice.h>

#include <map>
#include <mutex>
#include<pma/utils/ManagedInstance.h>

namespace epic {
namespace nls {



class VulkanManagedBuffer {
public:
    VulkanManagedBuffer(std::shared_ptr<VulkanDevice> device, VkBuffer buffer, VkDeviceMemory bufferMemory, size_t size)
        : m_device(device)
        , m_buffer(buffer)
        , m_bufferMemory(bufferMemory)
        , m_size(size)
    {}

    ~VulkanManagedBuffer()
    {
        if (m_buffer) vkDestroyBuffer(m_device->Device(), m_buffer, nullptr);
        if (m_bufferMemory) vkFreeMemory(m_device->Device(), m_bufferMemory, nullptr);
    }

    VulkanManagedBuffer(const VulkanManagedBuffer&) = delete;
    VulkanManagedBuffer& operator=(const VulkanManagedBuffer&) = delete;
    VulkanManagedBuffer(const VulkanManagedBuffer&&) = delete;
    VulkanManagedBuffer& operator=(const VulkanManagedBuffer&&) = delete;

    VkDevice Device() const { return m_device->Device(); }
    VkBuffer Buffer() const { return m_buffer; }
    VkDeviceMemory DeviceMemory() const { return m_bufferMemory; }
    size_t Size() const { return m_size; }
    size_t Offset() const { return 0; }

private:
    std::shared_ptr<VulkanDevice> m_device;
    VkBuffer m_buffer = VK_NULL_HANDLE;
    VkDeviceMemory m_bufferMemory = VK_NULL_HANDLE;
    size_t m_size;

};



class VulkanManagedImage {
public:
    VulkanManagedImage(std::shared_ptr<VulkanDevice> device, int width, int height, VkFormat format, VkImage image, VkDeviceMemory bufferMemory, VkImageView imageView, VkImageAspectFlags aspect)
        : m_device(device)
        , m_width(width)
        , m_height(height)
        , m_format(format)
        , m_image(image)
        , m_bufferMemory(bufferMemory)
        , m_imageView(imageView)
        , m_aspect(aspect)
    {}

    ~VulkanManagedImage()
    {
        if (m_imageView) vkDestroyImageView(m_device->Device(), m_imageView, nullptr);
        if (m_image) vkDestroyImage(m_device->Device(), m_image, nullptr);
        if (m_bufferMemory) vkFreeMemory(m_device->Device(), m_bufferMemory, nullptr);
    }

    VulkanManagedImage(const VulkanManagedImage&) = delete;
    VulkanManagedImage& operator=(const VulkanManagedImage&) = delete;
    VulkanManagedImage(const VulkanManagedImage&&) = delete;
    VulkanManagedImage& operator=(const VulkanManagedImage&&) = delete;

    int Width() const { return m_width; }
    int Height() const { return m_height; }
    VkFormat Format() const { return m_format; }
    VkDevice Device() const { return m_device->Device(); }
    VkImage Image() const { return m_image; }
    VkDeviceMemory DeviceMemory() const { return m_bufferMemory; }
    VkImageView ImageView() const { return m_imageView; }
    VkImageAspectFlags Aspect() const { return m_aspect; }

private:
    std::shared_ptr<VulkanDevice> m_device;
    int m_width;
    int m_height;
    VkFormat m_format;
    VkImage m_image = VK_NULL_HANDLE;
    VkDeviceMemory m_bufferMemory = VK_NULL_HANDLE;
    VkImageView m_imageView = VK_NULL_HANDLE;
    VkImageAspectFlags m_aspect;
};



class VulkanMemory {
public:
    static std::shared_ptr<VulkanMemory> Create(std::shared_ptr<VulkanDevice> device) {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);

        static std::map<uint64_t, std::weak_ptr<VulkanMemory>> memoryPerDevice;

        uint64_t deviceID = uint64_t(device.get());
        std::shared_ptr<VulkanMemory> ptr;

        auto it = memoryPerDevice.find(deviceID);
        if (it != memoryPerDevice.end()) {
            ptr = it->second.lock();
        }

        if (!ptr) {
            // pma::PolyAllocator<VulkanMemory> polyAlloc{ MEM_RESOURCE };
            // ptr = std::allocate_shared<VulkanMemory>(polyAlloc);
            ptr = std::make_shared<VulkanMemory>();
            ptr->m_device = device;
            ptr->m_transientCommandPool = VulkanCommandPool::Create(device, /*transient=*/true);
            memoryPerDevice[deviceID] = ptr;
        }

        return ptr;
    }

    VulkanMemory() = default;

    VulkanCommandPool* TransientCommandPool() { return m_transientCommandPool.get(); }

    std::unique_ptr<VulkanManagedBuffer> createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties) {

        VkBuffer buffer;
        VkDeviceMemory bufferMemory;

        VkBufferCreateInfo bufferInfo = {};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateBuffer(m_device->Device(), &bufferInfo, nullptr, &buffer) != VK_SUCCESS) {
            CARBON_CRITICAL("failed to create buffer!");
        }

        VkMemoryRequirements memRequirements;
        vkGetBufferMemoryRequirements(m_device->Device(), buffer, &memRequirements);

        VkMemoryAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(m_device->Device(), &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS) {
            CARBON_CRITICAL("failed to allocate buffer memory!");
        }

        vkBindBufferMemory(m_device->Device(), buffer, bufferMemory, 0);

        return std::make_unique<VulkanManagedBuffer>(m_device, buffer, bufferMemory, size);
    }


    std::shared_ptr<VulkanManagedImage> createImageAndView(uint32_t width, uint32_t height,
                                                           VkFormat format,
                                                           VkImageTiling tiling,
                                                           VkImageUsageFlags usage,
                                                           VkMemoryPropertyFlags properties,
                                                           VkImageAspectFlags aspect,
                                                           VkImageLayout finalLayout,
                                                           VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT,
                                                           VkComponentMapping componentMapping = {}) {

        VkImage image;
        VkDeviceMemory imageMemory;
        VkImageView imageView;

        VkImageCreateInfo imageInfo = {};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.extent.width = width;
        imageInfo.extent.height = height;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.format = format;
        imageInfo.tiling = tiling;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = usage;
        imageInfo.samples = samples;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateImage(m_device->Device(), &imageInfo, nullptr, &image) != VK_SUCCESS) {
            CARBON_CRITICAL("failed to create image!");
        }

        VkMemoryRequirements memRequirements;
        vkGetImageMemoryRequirements(m_device->Device(), image, &memRequirements);

        VkMemoryAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(m_device->Device(), &allocInfo, nullptr, &imageMemory) != VK_SUCCESS) {
            CARBON_CRITICAL("failed to allocate image memory!");
        }

        vkBindImageMemory(m_device->Device(), image, imageMemory, 0);

        VkImageViewCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        createInfo.image = image;
        createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        createInfo.format = format;
        createInfo.components = componentMapping;
        createInfo.subresourceRange.aspectMask = aspect;
        createInfo.subresourceRange.baseMipLevel = 0;
        createInfo.subresourceRange.levelCount = 1;
        createInfo.subresourceRange.baseArrayLayer = 0;
        createInfo.subresourceRange.layerCount = 1;

        if (vkCreateImageView(m_device->Device(), &createInfo, nullptr, &imageView) != VK_SUCCESS) {
            CARBON_CRITICAL("failed to create image views!");
        }

        if (finalLayout != VK_IMAGE_LAYOUT_UNDEFINED) {
            // transition the image
            transitionImageLayout(image, VK_IMAGE_LAYOUT_UNDEFINED, finalLayout, aspect);
        }

        return std::make_unique<VulkanManagedImage>(m_device, width, height, format, image, imageMemory, imageView, aspect);
    }


    std::unique_ptr<VulkanManagedBuffer> createBufferOnDeviceAndCopy(const void* bufferData, const size_t bufferSize, VkBufferUsageFlags usage)
    {
        std::unique_ptr<VulkanManagedBuffer> deviceBuffer = createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | usage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (!deviceBuffer) {
            CARBON_CRITICAL("failed to create device buffer");
        }

        updateBuffer(deviceBuffer.get(), bufferData, bufferSize);

        return deviceBuffer;
    }

    void updateBuffer(VulkanManagedBuffer* deviceBuffer, const void* newBufferData, const size_t bufferSize)
    {
        if (!deviceBuffer || !newBufferData) {
            CARBON_CRITICAL("invalid to pass nullptr");
        }
        if (bufferSize != deviceBuffer->Size()) {
            CARBON_CRITICAL("buffer size does not match size of device buffer");
        }

        static std::mutex mutex;
        std::unique_lock<std::mutex> lock(mutex);

        
        if (!stagingBuffer || stagingBuffer->Size() < bufferSize) {
            stagingBuffer = createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        }

        void* data;
        vkMapMemory(stagingBuffer->Device(), stagingBuffer->DeviceMemory(), 0, bufferSize, 0, &data);
        memcpy(data, newBufferData, bufferSize);
        vkUnmapMemory(stagingBuffer->Device(), stagingBuffer->DeviceMemory());
        // in case we don't use  VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, you would
        // call vkFlushMappedMemoryRanges to after writing to the mapped memory,
        // and call vkInvalidateMappedMemoryRanges before reading from the mapped memory

        copyBuffer(stagingBuffer->Buffer(), deviceBuffer->Buffer(), bufferSize);
    }


    void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size)
    {
        VkCommandBuffer commandBuffer = m_transientCommandPool->BeginSingleTimeCommands();
        copyBuffer(commandBuffer, srcBuffer, dstBuffer, size);
        m_transientCommandPool->EndSingleTimeCommands(commandBuffer);
    }

    void copyBuffer(VkCommandBuffer commandBuffer, VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size)
    {
        VkBufferCopy copyRegion = {};
        copyRegion.size = size;
        vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion);
    }

    void copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height)
    {
        VkCommandBuffer commandBuffer = m_transientCommandPool->BeginSingleTimeCommands();
        copyBufferToImage(commandBuffer, buffer, image, width, height);
        m_transientCommandPool->EndSingleTimeCommands(commandBuffer);
    }


    void copyBufferToImage(VkCommandBuffer commandBuffer, VkBuffer buffer, VkImage image, uint32_t width, uint32_t height)
    {
        VkBufferImageCopy region = {};
        region.bufferOffset = 0;
        region.bufferRowLength = 0;
        region.bufferImageHeight = 0;

        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.mipLevel = 0;
        region.imageSubresource.baseArrayLayer = 0;
        region.imageSubresource.layerCount = 1;

        region.imageOffset = {0, 0, 0};
        region.imageExtent = {
            width,
            height,
            1
        };

        vkCmdCopyBufferToImage(
            commandBuffer,
            buffer,
            image,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            1,
            &region
        );
    }

    void copyImageToBuffer(VkImage image, VkBuffer buffer, uint32_t width, uint32_t height)
    {
        VkCommandBuffer commandBuffer = m_transientCommandPool->BeginSingleTimeCommands();
        copyImageToBuffer(commandBuffer, image, buffer, width, height);
        m_transientCommandPool->EndSingleTimeCommands(commandBuffer);
    }


    void copyImageToBuffer(VkCommandBuffer commandBuffer, VkImage image, VkBuffer buffer, uint32_t width, uint32_t height, VkImageAspectFlags aspect = VK_IMAGE_ASPECT_COLOR_BIT)
    {
        VkBufferImageCopy region = {};
        region.bufferOffset = 0;
        region.bufferRowLength = 0;
        region.bufferImageHeight = 0;

        region.imageSubresource.aspectMask = aspect;
        region.imageSubresource.mipLevel = 0;
        region.imageSubresource.baseArrayLayer = 0;
        region.imageSubresource.layerCount = 1;

        region.imageOffset = {0, 0, 0};
        region.imageExtent = {
            width,
            height,
            1
        };

        vkCmdCopyImageToBuffer(
            commandBuffer,
            image,
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            buffer,
            1,
            &region
        );
    }

    void copyPixelToBuffer(VkCommandBuffer commandBuffer, VkImage image, VkBuffer buffer, int x, int y, VkImageAspectFlags aspect = VK_IMAGE_ASPECT_COLOR_BIT)
    {
        VkBufferImageCopy region = {};
        region.bufferOffset = 0;
        region.bufferRowLength = 0;
        region.bufferImageHeight = 0;

        region.imageSubresource.aspectMask = aspect;
        region.imageSubresource.mipLevel = 0;
        region.imageSubresource.baseArrayLayer = 0;
        region.imageSubresource.layerCount = 1;

        region.imageOffset = {x, y, 0};
        region.imageExtent = {
            1,
            1,
            1
        };

        vkCmdCopyImageToBuffer(
            commandBuffer,
            image,
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            buffer,
            1,
            &region
        );
    }

    void transitionImageLayout(VkImage image, VkImageLayout oldLayout, VkImageLayout newLayout, VkImageAspectFlags aspect)
    {
        VkCommandBuffer commandBuffer = m_transientCommandPool->BeginSingleTimeCommands();
        transitionImageLayout(commandBuffer, image, oldLayout, newLayout, aspect);
        m_transientCommandPool->EndSingleTimeCommands(commandBuffer);
    }

    static void transitionImageLayout(VkCommandBuffer commandBuffer, VkImage image, VkImageLayout oldLayout, VkImageLayout newLayout, VkImageAspectFlags aspect)
    {
        VkImageMemoryBarrier barrier = {};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = oldLayout;
        barrier.newLayout = newLayout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;
        barrier.subresourceRange.aspectMask = aspect;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;

        VkPipelineStageFlags sourceStage;
        VkPipelineStageFlags destinationStage;

        if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL) {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        }else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_GENERAL) {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            destinationStage = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

            sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

            sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL) {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_GENERAL && newLayout == VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL) {
            barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

            sourceStage = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
            destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_GENERAL && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
            barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
            destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_GENERAL) {
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            destinationStage = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_GENERAL) {
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            destinationStage = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        } else {
            throw std::invalid_argument("unsupported layout transition!");
        }

        vkCmdPipelineBarrier(
            commandBuffer,
            sourceStage, destinationStage,
            0,
            0, nullptr,
            0, nullptr,
            1, &barrier
        );
    }

    void CopyFromDevice(void* data, size_t size, VkBuffer buffer)
    {
        static std::mutex mutex;
        std::unique_lock<std::mutex> lock(mutex);

        if (!vulkanOutputArrayHostBufferCFD || vulkanOutputArrayHostBufferCFD->Size() < size) {
            vulkanOutputArrayHostBufferCFD = createBuffer(size,
                VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |
                VK_MEMORY_PROPERTY_HOST_CACHED_BIT);
        }

        copyBuffer(buffer, vulkanOutputArrayHostBufferCFD->Buffer(), size);

        void* mappedData;
        vkMapMemory(vulkanOutputArrayHostBufferCFD->Device(),
            vulkanOutputArrayHostBufferCFD->DeviceMemory(),
            0,
            size,
            0,
            &mappedData);
        memcpy(data, mappedData, size);
        vkUnmapMemory(vulkanOutputArrayHostBufferCFD->Device(), vulkanOutputArrayHostBufferCFD->DeviceMemory());
    }

    void CopySubsetToDevice(const void* data, size_t offset, size_t size, VkBuffer buffer, VulkanCommandPool* commandPool) {
        static std::mutex mutex;
        std::unique_lock<std::mutex> lock(mutex);


        if (!vulkanOutputArrayHostBufferCSTD || vulkanOutputArrayHostBufferCSTD->Size() < size) {
            vulkanOutputArrayHostBufferCSTD = createBuffer(size,
                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        }

        void* mappedData;
        vkMapMemory(vulkanOutputArrayHostBufferCSTD->Device(), vulkanOutputArrayHostBufferCSTD->DeviceMemory(), offset, size, 0, &mappedData);
        memcpy(mappedData, data, size);
        vkUnmapMemory(vulkanOutputArrayHostBufferCSTD->Device(), vulkanOutputArrayHostBufferCSTD->DeviceMemory());

        VkCommandBuffer commandBuffer = commandPool->BeginSingleTimeCommands();
        VkBufferCopy copyRegion = {};
        copyRegion.srcOffset = 0;
        copyRegion.dstOffset = offset;
        copyRegion.size = size;
        vkCmdCopyBuffer(commandBuffer, vulkanOutputArrayHostBufferCSTD->Buffer(), buffer, 1, &copyRegion);
        commandPool->EndSingleTimeCommands(commandBuffer);
    }


    void CopySubsetFromDevice(void* data, size_t offset, size_t size, VkBuffer buffer, VulkanCommandPool* commandPool) {

        if (!vulkanOutputArrayHostBufferCSFD || vulkanOutputArrayHostBufferCSFD->Size() < size) {
            vulkanOutputArrayHostBufferCSFD = createBuffer(size,
                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        }

        VkCommandBuffer commandBuffer = commandPool->BeginSingleTimeCommands();
        VkBufferCopy copyRegion = {};
        copyRegion.srcOffset = offset;
        copyRegion.dstOffset = 0;
        copyRegion.size = size;
        vkCmdCopyBuffer(commandBuffer, buffer, vulkanOutputArrayHostBufferCSFD->Buffer(), 1, &copyRegion);
        commandPool->EndSingleTimeCommands(commandBuffer);

        void* mappedData;
        vkMapMemory(vulkanOutputArrayHostBufferCSFD->Device(), vulkanOutputArrayHostBufferCSFD->DeviceMemory(), offset, size, 0, &mappedData);
        memcpy(data, mappedData, size);
        vkUnmapMemory(vulkanOutputArrayHostBufferCSFD->Device(), vulkanOutputArrayHostBufferCSFD->DeviceMemory());
    }

private:
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
        VkPhysicalDeviceMemoryProperties memProperties;
        vkGetPhysicalDeviceMemoryProperties(m_device->PhysicalDevice(), &memProperties);

        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
            if (typeFilter & (1 << i) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }

        CARBON_CRITICAL("failed to find suitable memory type!");
    }

private:
    std::shared_ptr<VulkanDevice> m_device;
    std::unique_ptr<VulkanCommandPool> m_transientCommandPool;
    // we keep only one buffer to copy data to GPU
    std::unique_ptr<VulkanManagedBuffer> stagingBuffer;
    std::unique_ptr<VulkanManagedBuffer> vulkanOutputArrayHostBufferCFD;
    std::unique_ptr<VulkanManagedBuffer> vulkanOutputArrayHostBufferCSTD;
    std::unique_ptr<VulkanManagedBuffer> vulkanOutputArrayHostBufferCSFD;
};

} // namespace nls
} //namespace epic
