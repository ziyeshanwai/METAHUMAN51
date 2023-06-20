// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanCommandPool.h>
#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanFramebuffer.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanTextureSampler.h>

#include <nls/geometry/Camera.h>

namespace epic {
namespace nls {

/**
 * Class to render a depth map as a point cloud using Vulkan.
 */
class DepthmapVulkanRenderer {
public:
    struct RenderData {
        std::shared_ptr<VulkanManagedBuffer> buffer;
        Camera<float> camera;
    };
    typedef std::shared_ptr<const RenderData> RenderDataConstPtr;

public:
    DepthmapVulkanRenderer(std::shared_ptr<VulkanDevice> vulkanDevice);
    ~DepthmapVulkanRenderer();

    void Reset();

    /**
     * Setup the mesh rendering based on the input renderpass and the number of samples.
     */
    void Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples);

    //! Upload the depth and normal data
    void SetDepthAndNormalData(const epic::nls::Camera<float>& depthCamera, const float* depthAndNormals);

    //! Set the depth and normal buffer. It is required that the buffer is not changed.
    void SetDepthAndNormalData(const epic::nls::Camera<float>& depthCamera, const std::shared_ptr<VulkanManagedBuffer>& buffer);

    //! Render the depthmap
    void Render(const Eigen::Matrix4f& projectionMatrix, const Eigen::Matrix4f& modelViewMatrix, VkCommandBuffer buffer, const Eigen::Vector4f& color);

    RenderDataConstPtr CreateDepthAndNormalData(const epic::nls::Camera<float>& depthCamera, const float* depthAndNormals) const;

    void Render(RenderDataConstPtr renderData, const Eigen::Matrix4f& projectionMatrix, const Eigen::Matrix4f& modelViewMatrix, VkCommandBuffer buffer, const Eigen::Vector4f& color);

private:
    struct UniformBufferObject;

private:
    void CreateRenderPass(VkFormat format);
    void CreateGraphicsPipeline(VkRenderPass renderPass, VkSampleCountFlagBits samples);

private:
    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_graphicsPipeline = VK_NULL_HANDLE;

    RenderDataConstPtr m_renderData;
};


} // namespace nls
} //namespace epic
