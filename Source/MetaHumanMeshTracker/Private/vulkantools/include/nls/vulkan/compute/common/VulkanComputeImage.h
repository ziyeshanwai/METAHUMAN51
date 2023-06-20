// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDescriptorSet.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/compute/common/VulkanComputeContext.h>

namespace epic {
namespace nls {

class VulkanComputeImage {

public:
    ~VulkanComputeImage() {}

    static std::shared_ptr<VulkanComputeImage> Create(int width, int height, std::shared_ptr<VulkanComputeContext> computeContext)
    {
        return std::shared_ptr<VulkanComputeImage>(new VulkanComputeImage(width, height, computeContext));
    }

    std::shared_ptr<VulkanDescriptorSet> DescriptorSet() { return m_descriptorSet; }

    int Width() const { return m_width; }
    int Height() const { return m_height; }

    void CopyToDevice(const void* data, size_t size);
    void CopyFromDevice(void* data, size_t size);

private:
    VulkanComputeImage(int width, int height, std::shared_ptr<VulkanComputeContext> computeContext);

    void UpdateDescriptorSet();

    VulkanComputeImage(const VulkanComputeImage&) = delete;
    VulkanComputeImage& operator=(const VulkanComputeImage&) = delete;

private:
    int m_width;
    int m_height;
    std::shared_ptr<VulkanComputeContext> m_computeContext;
    std::shared_ptr<VulkanManagedImage> m_vulkanImage;
    std::shared_ptr<VulkanDescriptorSet> m_descriptorSet;
};



} //namespace nls
} //namespace epic
