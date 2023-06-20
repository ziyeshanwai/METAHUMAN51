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

namespace epic {
namespace nls {


/**
 * Class to render a mesh using Vulkan.
 */
class MeshVulkanRenderer {
public:
    MeshVulkanRenderer(std::shared_ptr<VulkanDevice> vulkanDevice);

    ~MeshVulkanRenderer();

    void Reset();

    //! Setup the mesh rendering based on the input renderpass, the number of samples, culling mode and wether depth test is enabled.
    void Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples, VkCullModeFlagBits cullMode = VK_CULL_MODE_NONE, bool withDepthWrite = true);

    //! Update the vertex and normal buffer (and the index buffer if it is not set)
    void UpdateMeshBuffer(VulkanMeshBuffer& meshBuffer, const epic::nls::Mesh<float>& mesh) const;

    //! Create the mesh buffer which is compatible with rendering
    VulkanMeshBuffer CreateMeshBuffer(const epic::nls::Mesh<float>& mesh) const;

    //! Render the mesh.
    void Render(VkCommandBuffer buffer,
                const Eigen::Matrix4f& projectionMatrix,
                const Eigen::Matrix4f& modelViewMatrix,
                const VulkanMeshBuffer& meshBuffer,
                const Eigen::Vector4f& color = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f),
                const Eigen::Vector4f& backColor = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
                bool useLighting = true) const;

    struct Material {
        Eigen::Vector4f color = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
        Eigen::Vector4f backColor = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        float shininess = 30;
        float specIntensity = 0;
    };

    //! Render the mesh.
    void Render(VkCommandBuffer buffer,
                const Eigen::Matrix4f& projectionMatrix,
                const Eigen::Matrix4f& modelViewMatrix,
                const VulkanMeshBuffer& meshBuffer,
                const Material& material,
                bool useLighting = true) const;

private:
    struct UniformBufferObject;

private:
    VkPipeline CreateGraphicsPipeline(VkRenderPass renderPass, VkSampleCountFlagBits samples, VkCullModeFlagBits cullMode, bool withDepthWrite);

private:
    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_graphicsPipeline = VK_NULL_HANDLE;
};


} // namespace nls
} //namespace epic
