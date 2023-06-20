// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>

#include <nls/vulkan/common/VulkanCommandPool.h>
#include <nls/vulkan/common/VulkanDescriptorSetLayout.h>
#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanFramebuffer.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/common/VulkanMeshBuffer.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanTextureSampler.h>


namespace epic::nls {


/**
 * Class to render colored lines using Vulkan.
 */
class ColoredLinesVulkanRenderer {
public:
    ColoredLinesVulkanRenderer(std::shared_ptr<VulkanDevice> vulkanDevice);

    ~ColoredLinesVulkanRenderer();

    void Reset();

    //! Setup the graphics pipline based on the input renderpass and the number of samples.
    void Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples);

    //! Create the colored lines buffer which is compatible with rendering
    VulkanMeshBuffer CreateColoredLinesBuffer(const Eigen::Ref<const Eigen::Matrix<float, 3, -1>>& vertices, const Eigen::Ref<const Eigen::Matrix<float, 4, -1>>& colors);

    //! Render the mesh.
    void Render(VkCommandBuffer buffer,
                const Eigen::Matrix4f& projectionMatrix,
                const Eigen::Matrix4f& modelViewMatrix,
                const VulkanMeshBuffer& coloredLinesBuffer) const;

private:
    struct UniformBufferObject;

private:
    void CreateDescriptorSetLayout();
    VkPipeline CreateGraphicsPipeline(VkRenderPass renderPass, VkSampleCountFlagBits samples);

private:
    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;
    std::shared_ptr<VulkanDescriptorSetLayout> m_descriptorSetLayout;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_graphicsPipeline = VK_NULL_HANDLE;
};


} // namespace epic::nls
