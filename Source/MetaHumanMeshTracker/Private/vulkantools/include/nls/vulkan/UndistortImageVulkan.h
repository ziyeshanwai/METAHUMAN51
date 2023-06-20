// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanCommandPool.h>
#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanTextureSampler.h>

#include <nls/geometry/MetaShapeCamera.h>

namespace epic {
namespace nls {

/**
 * Class to undistort an image using a Vulkan compute shader
 */
class UndistortImageVulkan {

    struct ShaderData {
        // focal length and principal point need to map to [0, 1] instead of [0, width]
        int width;
        int height;
        float fx;
        float fy;
        float cx;
        float cy;
        float B1;
        float B2;
        Eigen::Vector4f radialDistortion;
        Eigen::Vector4f tangentialDistortion;
    };

public:
    UndistortImageVulkan(std::shared_ptr<VulkanDevice> vulkanDevice);

    ~UndistortImageVulkan();

    //! Undistort @p distortedImage and save to @p undistortedImage using the distortion parameters of @p camera. The image is assumed to be a 4-channel image.
    void Undistort(const float* distortedImage, float* undistortedImage, const MetaShapeCamera<float>& camera);

    void Reset();

    void CreateDescriptorSetLayout();
    void CreateDescriptorPool();
    void CreateDescriptorSets();
    void CreateComputePipeline();
    void CreateCommandBuffers();
    void CreateSyncObjects();

private:
    int m_width = 0;
    int m_height = 0;
    MetaShapeCamera<float> m_camera;

    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;

    VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_computePipeline = VK_NULL_HANDLE;
    VkFence m_fence = VK_NULL_HANDLE;
    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;
    VkDescriptorSet m_descriptorSet; // actual binding of buffers

    std::unique_ptr<VulkanTextureSampler> m_texSampler;
    std::unique_ptr<VulkanManagedBuffer> m_uniformBuffer;

    // distorted image on device
    std::shared_ptr<VulkanManagedImage> m_distortedImage;

    // undistorted image on device
    std::shared_ptr<VulkanManagedImage> m_undistortedImage;

    // copied host memory of image
    std::unique_ptr<VulkanManagedBuffer> m_undistortedBuffer;

    // rendering commands
    std::unique_ptr<VulkanCommandPool> m_transientCommandPool;
    VkCommandBuffer m_commandBuffer;
};


} // namespace nls
} //namespace epic
