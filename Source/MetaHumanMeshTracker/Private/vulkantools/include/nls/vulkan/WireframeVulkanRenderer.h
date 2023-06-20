// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanCommandPool.h>
#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanFramebuffer.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/common/VulkanMeshBuffer.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanTextureSampler.h>

#include <nls/geometry/Mesh.h>

namespace epic::nls {


/**
 * Class to render a wireframe model using Vulkan.
 */
class WireframeVulkanRenderer {
public:
    WireframeVulkanRenderer(std::shared_ptr<VulkanDevice> vulkanDevice);

    ~WireframeVulkanRenderer();

    void Reset();

    //! Setup the mesh rendering based on the input renderpass and the number of samples.
    void Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples);

    //! Create just the wireframe index buffer which is compatible with rendering
    void CreateWireframeIndexBuffer(VulkanMeshBuffer& meshBuffer, const epic::nls::Mesh<float>& mesh);

    //! Create the wireframe buffer which is compatible with rendering
    VulkanMeshBuffer CreateWireframeBuffer(const epic::nls::Mesh<float>& mesh);

    //! Render the mesh.
    void Render(VkCommandBuffer buffer,
                const Eigen::Matrix4f& projectionMatrix,
                const Eigen::Matrix4f& modelViewMatrix,
                const VulkanMeshBuffer& wireframeBuffer,
                const Eigen::Vector4f& color = Eigen::Vector4f(0.05f, 0.05f, 0.05f, 1.0f)) const;

private:
    struct UniformBufferObject;

private:
    void CreateDescriptorSetLayout();
    VkPipeline CreateGraphicsPipeline(VkRenderPass renderPass, VkSampleCountFlagBits samples);

private:
    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;

    VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_graphicsPipeline = VK_NULL_HANDLE;

    float m_depthBias = 1e-8f;
    float m_depthScale = 1.001f;
};


} // namespace epic::nls
