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
 * Class to calculate the visibility of vertices with respect to a depthmap.
 */
class VertexVisibilityVulkan {
public:
    VertexVisibilityVulkan(std::shared_ptr<VulkanDevice> vulkanDevice);

    ~VertexVisibilityVulkan();

    void Reset();

    //! Setup the mesh rendering based on the input renderpass and the number of samples.
    void Setup();

    //! Compute the vertex visibility
    Eigen::VectorXf Compute(const Eigen::Matrix4f& textureProjectionMatrix,
                                const Eigen::Matrix4f& textureModelViewMatrix,
                                const VulkanMeshBuffer& meshBuffer,
                                const VulkanAllInOneTexture& depthTexture,
                                const VulkanAllInOneTexture& depthAlphaTexture);

private:
    struct UniformBufferObject;

private:
    void CreateDescriptorSetLayout();
    void CreateComputePipeline();
    void CreateDescriptorSets();
    void UpdateDescriptorSets(const VulkanMeshBuffer& meshBuffer, std::shared_ptr<VulkanManagedBuffer> visibilityBuffer);

private:

    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;

    std::shared_ptr<VulkanDescriptorSetLayout> m_descriptorSetLayout;
    std::shared_ptr<VulkanDescriptorSet> m_descriptorSet;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_computePipeline = VK_NULL_HANDLE;

    std::unique_ptr<VulkanCommandPool> m_transientCommandPool;

    std::shared_ptr<VulkanManagedBuffer> m_visibilityBufferHost;
    std::shared_ptr<VulkanManagedBuffer> m_visibilityBufferDevice;
};


} // namespace epic::nls
