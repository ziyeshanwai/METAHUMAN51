// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/PixelFormat.h>
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
 * Class to render a textured mesh using Vulkan. Works for meshes that have multiple texture coordinates per vertex index.
 * The render method can only be called once per commandbuffer submission as it requires the update of the descriptor sets which
 * is not part of the commandbuffer pipeline and needs to happen before the submission.
 */
class TexturedMeshVulkanRenderer {
public:
    TexturedMeshVulkanRenderer(std::shared_ptr<VulkanDevice> vulkanDevice);

    ~TexturedMeshVulkanRenderer();

    void Reset();

    //! Setup the mesh rendering based on the input renderpass and the number of samples.
    void Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples);

    //! Create just the texture index and texcoord buffer which is compatible with rendering
    void CreateTexcoordAndTexcoordIndexBuffer(VulkanMeshBuffer& meshBuffer, const epic::nls::Mesh<float>& mesh);

    //! Create the mesh buffer which is compatible with rendering
    VulkanMeshBuffer CreateMeshBuffer(const epic::nls::Mesh<float>& mesh);

    //! Load the texture image
    void LoadTextureImage(const float* imgData, int width, int height, PixelFormat pixelFormat);

    //! Render the mesh.
    void Render(VkCommandBuffer buffer,
                const Eigen::Matrix4f& projectionMatrix,
                const Eigen::Matrix4f& modelViewMatrix,
                const VulkanMeshBuffer& meshBuffer);

    //! Render the mesh.
    void Render(VkCommandBuffer buffer,
                const Eigen::Matrix4f& projectionMatrix,
                const Eigen::Matrix4f& modelViewMatrix,
                const VulkanMeshBuffer& meshBuffer,
                const VulkanAllInOneTexture& allInOneTexture);

private:
    struct UniformBufferObject;

private:
    void CreateDescriptorSetLayout();
    void CreateGraphicsPipeline(VkRenderPass renderPass, VkSampleCountFlagBits samples);
    void CreateDescriptorSets();
    void UpdateDescriptorSets(const VulkanMeshBuffer& meshBuffer);

private:

    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;

    //! texture with layout info, descriptor set, sampler, and the texture image
    std::shared_ptr<VulkanAllInOneTexture> m_allInOneTexture;

    std::shared_ptr<VulkanDescriptorSetLayout> m_drawDescriptorSetLayout;
    std::shared_ptr<VulkanDescriptorSet> m_drawDescriptorSet;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_graphicsPipeline = VK_NULL_HANDLE;
};


} // namespace epic::nls
