// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanAllInOneTexture.h>
#include <nls/vulkan/common/VulkanCommandPool.h>
#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanDescriptorSet.h>
#include <nls/vulkan/common/VulkanDescriptorSetLayout.h>
#include <nls/vulkan/common/VulkanDescriptorSetPool.h>
#include <nls/vulkan/common/VulkanFramebuffer.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/common/VulkanMeshBuffer.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanTextureSampler.h>

#include <nls/geometry/Camera.h>
#include <nls/geometry/Mesh.h>


namespace epic::nls {

/**
 * Class to render a textured mesh using Vulkan. The texture coordinates are calculated by projecting the vertices into a texture image.
 */
class ProjectedTexturedMeshVulkanRenderer {
public:
    ProjectedTexturedMeshVulkanRenderer(std::shared_ptr<VulkanDevice> vulkanDevice);

    ~ProjectedTexturedMeshVulkanRenderer();

    void Reset();

    //! Setup the mesh rendering based on the input renderpass and the number of samples.
    void Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples);

    //! Render the mesh.
    void Render(VkCommandBuffer buffer,
                const Eigen::Matrix4f& projectionMatrix,
                const Eigen::Matrix4f& textureProjectionMatrix,
                const VulkanMeshBuffer& meshBuffer,
                const VulkanAllInOneTexture& allInOneTexture);

private:
    struct UniformBufferObject;

private:
    void CreateDescriptorSetLayout();
    void CreateGraphicsPipeline(VkRenderPass renderPass, VkSampleCountFlagBits samples);

private:
    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_graphicsPipeline = VK_NULL_HANDLE;
};


} // namespace epic::nls
