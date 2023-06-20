// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDescriptorSet.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <pma/PolyAllocator.h>

namespace epic {
namespace nls {

/**
 * A VulkanComputeBuffer represents similar to VulkanComputeImage an image of size width x height of float values. Contrary to VulkanComputeImage
 * the data is in linear layout and can be read in HLSL using StructuredBuffer<float> and RWStructuredBuffer<float>.
 */
class VulkanComputeBuffer {

public:
    VulkanComputeBuffer(int width, int height, std::shared_ptr<VulkanComputeContext> computeContext);
    ~VulkanComputeBuffer() {}

    static std::shared_ptr<VulkanComputeBuffer> Create(int width, int height, std::shared_ptr<VulkanComputeContext> computeContext)
    {
        pma::PolyAllocator<VulkanComputeBuffer> polyAlloc{ MEM_RESOURCE };
        return std::allocate_shared<VulkanComputeBuffer>(polyAlloc, width, height, computeContext);
    }

    std::shared_ptr<VulkanDescriptorSet> DescriptorSet() { return m_descriptorSet; }

    VkDescriptorBufferInfo DescriptorInfo() const
    {
        return VkDescriptorBufferInfo{ m_vulkanBuffer->Buffer(), m_vulkanBuffer->Offset(), m_vulkanBuffer->Size() };
    }

    int Width() const { return m_width; }
    int Height() const { return m_height; }
    int MaximumWidth() const { return m_fullWidth; }
    int MaximumHeight() const { return m_fullHeight; }
    int SizeInBytes() const { return m_width * m_height * sizeof(float); }
    int FullSizeInBytes() const { return m_fullWidth * m_fullHeight * sizeof(float); }

    void CopyToDevice(const void* data, size_t size);
    void CopyFromDevice(void* data, size_t size);

    void CopySubsetToDevice(const void* data, size_t offset, size_t size);
    void CopySubsetFromDevice(void* data, size_t offset, size_t size);

	// Copy the contents of this buffer to another buffer in the device
	void CopyTo(std::shared_ptr<VulkanComputeBuffer> dst);

    /**
     * Resize the array object without reallocating data. This allows you to reuse the same memory for multiple array sizes.
     * Throws an exception if width or height are larger than the original allocation.
     */
    void Resize(int newWidth, int newHeight);

    const std::shared_ptr<VulkanComputeContext>& ComputeContext() const { return m_computeContext; }

    const std::shared_ptr<VulkanManagedBuffer>& ManagedBuffer() const { return m_vulkanBuffer; }

private:
    void UpdateDescriptorSet();

    VulkanComputeBuffer(const VulkanComputeBuffer&) = delete;
    VulkanComputeBuffer& operator=(const VulkanComputeBuffer&) = delete;

private:
    int m_width;
    int m_height;
    int m_fullWidth;
    int m_fullHeight;
    std::shared_ptr<VulkanComputeContext> m_computeContext;
    std::shared_ptr<VulkanManagedBuffer> m_vulkanBuffer;
    std::shared_ptr<VulkanDescriptorSet> m_descriptorSet;
};



} //namespace nls
} //namespace epic
