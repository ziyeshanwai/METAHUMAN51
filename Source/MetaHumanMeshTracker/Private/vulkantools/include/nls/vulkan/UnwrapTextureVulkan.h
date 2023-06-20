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
 * Class to render to the UV space of the mesh. Pixels are colored by projecting the mesh vetices into an input texture.
 */
class UnwrapTextureVulkan {
public:
    UnwrapTextureVulkan(std::shared_ptr<VulkanDevice> vulkanDevice, bool forVisualization, bool outputImageCoordinates);

    ~UnwrapTextureVulkan();

    void Reset();

    //! Setup the mesh rendering based on the input renderpass and the number of samples.
    void Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples);

    //! Create just the texture index and texcoord buffer which is compatible with rendering
    void CreateTexcoordAndTexcoordIndexBuffer(VulkanMeshBuffer& meshBuffer, const epic::nls::Mesh<float>& mesh);

    //! Create the mesh buffer which is compatible with rendering
    VulkanMeshBuffer CreateMeshBuffer(const epic::nls::Mesh<float>& mesh);

    //! Render the mesh.
    void Render(VkCommandBuffer buffer,
                const Eigen::Matrix4f& uvTransformMatrix,
                const Eigen::Matrix4f& textureProjectionMatrix,
                const Eigen::Matrix4f& textureModelViewMatrix,
                const VulkanMeshBuffer& meshBuffer,
                const VulkanAllInOneTexture& imageTexture,
                const VulkanAllInOneTexture& depthTexture,
                const VulkanAllInOneTexture& depthAlphaTexture);

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

    std::shared_ptr<VulkanDescriptorSetLayout> m_drawDescriptorSetLayout;
    std::shared_ptr<VulkanDescriptorSet> m_drawDescriptorSet;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_graphicsPipeline = VK_NULL_HANDLE;

    /**
     * Flag whether the unwrapping is used for visualization purposes or to reconstruct texture.
     * For visualization the alpha computation is slightly modified and alpha blending is used to only show visible unwrapped textured.
     * For reconstruction the alpha value is set, but it is not used for alpha blending.
     */
    bool m_forVisualization;

    /**
     * Flag whether the unwrapped texture should have the projected image coordinate in the green and blue color channel, and therefore
     * only the red color channel will be unwrapped.
     */
    bool m_outputImageCoordinates;
};


} // namespace epic::nls
