// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanCommandPool.h>
#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanFramebuffer.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/common/VulkanMeshBuffer.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanTextureSampler.h>

#include <nls/geometry/Camera.h>
#include <nls/geometry/Mesh.h>

namespace epic::nls {


/**
 * Class to render a colorered mesh using Vulkan.
 */
class ColoredMeshVulkanRenderer {
public:
    ColoredMeshVulkanRenderer(std::shared_ptr<VulkanDevice> vulkanDevice);

    ~ColoredMeshVulkanRenderer();

    void Reset();

    //! Setup the graphics pipeline based on the input renderpass and the number of samples.
    void Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples);

    //! Update the color buffer only (assumes vertex, normal, and index buffer are already set)
    void UpdateMeshBuffer(VulkanMeshBuffer& meshBuffer, const Eigen::Matrix<float, 4, -1>& colors) const;

    //! Render the mesh.
    void Render(VkCommandBuffer buffer,
                const Eigen::Matrix4f& projectionMatrix,
                const Eigen::Matrix4f& modelViewMatrix,
                const VulkanMeshBuffer& meshBuffer,
                bool useLighting = true) const;


private:
    struct UniformBufferObject;

private:
    VkPipeline CreateGraphicsPipeline(VkRenderPass renderPass, VkSampleCountFlagBits samples);

private:
    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_graphicsPipeline = VK_NULL_HANDLE;
};


} // namespace epic::nls
