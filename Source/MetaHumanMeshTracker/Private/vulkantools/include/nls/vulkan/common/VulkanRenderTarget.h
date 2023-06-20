// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>

#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/common/VulkanFramebuffer.h>
#include <nls/vulkan/common/VulkanCommandPool.h>
#include <nls/vulkan/common/VulkanAllInOneTexture.h>

#include <array>

namespace epic {
namespace nls {

/**
 * A vulkan render target consists of two renderpasses (with clear color/depth and without), associated RGB/depth images, and a framebuffer.
 * The render target format is by default BGRA in 8bit uniform (not SRGB!).
 */
class VulkanRenderTarget {
public:
    static std::unique_ptr<VulkanRenderTarget> Create(std::shared_ptr<VulkanDevice> vulkanDevice, int width, int height, VkSampleCountFlagBits samples, VkFormat format = VK_FORMAT_B8G8R8A8_UNORM)
    {
        return std::unique_ptr<VulkanRenderTarget>(new VulkanRenderTarget(vulkanDevice, width, height, samples, format));
    }

    ~VulkanRenderTarget()
    {
        if (m_renderPassClear) {
            vkDestroyRenderPass(m_vulkanDevice->Device(), m_renderPassClear, nullptr);
            m_renderPassClear = VK_NULL_HANDLE;
        }
        if (m_renderPassNoClear) {
            vkDestroyRenderPass(m_vulkanDevice->Device(), m_renderPassNoClear, nullptr);
            m_renderPassNoClear = VK_NULL_HANDLE;
        }
    }

    void Resize(int width, int height)
    {
        if (!m_rgbaImage || m_rgbaImage->Width() != width || m_rgbaImage->Height() != height) {
            if (m_samples == VK_SAMPLE_COUNT_1_BIT) {
                m_rgbaImage = m_vulkanMemory->createImageAndView(width, height, m_format, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_SAMPLE_COUNT_1_BIT);
                m_multiDepthImage = m_vulkanMemory->createImageAndView(width, height, VK_FORMAT_D32_SFLOAT, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_SAMPLE_COUNT_1_BIT);
                m_rgbaFrameBuffer = VulkanFramebuffer::Create(m_vulkanDevice, m_rgbaImage->Width(), m_rgbaImage->Height(), m_renderPassClear, {m_rgbaImage->ImageView(), m_multiDepthImage->ImageView()});
            } else {
                m_rgbaImage = m_vulkanMemory->createImageAndView(width, height, m_format, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_SAMPLE_COUNT_1_BIT);
                m_multiRgbaImage = m_vulkanMemory->createImageAndView(width, height, m_format, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, m_samples);
                m_multiDepthImage = m_vulkanMemory->createImageAndView(width, height, VK_FORMAT_D32_SFLOAT, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, m_samples);
                m_rgbaFrameBuffer = VulkanFramebuffer::Create(m_vulkanDevice, m_rgbaImage->Width(), m_rgbaImage->Height(), m_renderPassClear, {m_multiRgbaImage->ImageView(), m_multiDepthImage->ImageView(), m_rgbaImage->ImageView()});
            }
            m_depthBuffer.reset();
            m_rgbaBuffer.reset();
            m_rgbaTexture.reset();
            m_depthTexture.reset();
        }
    }

    VkImage Image() const { return m_rgbaImage->Image(); }
    VkImageView ImageView() const { return m_rgbaImage->ImageView(); }

    int Width() const { return m_rgbaImage ? m_rgbaImage->Width() : 0; }
    int Height() const { return m_rgbaImage ? m_rgbaImage->Height() : 0; }
    VkSampleCountFlagBits Samples() const { return m_samples; }

    VkCommandBuffer StartRender(bool clear, const Eigen::Vector4f& clearColor = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f))
    {
        return StartRender(clear, clearColor, Eigen::Vector4i(0, 0, Width(), Height()), Eigen::Vector4i(0, 0, Width(), Height()));
    }

    VkCommandBuffer StartRender(bool clear,
                                const Eigen::Vector4f& clearColor,
                                Eigen::Vector4i viewportRect,
                                Eigen::Vector4i scissorsRect)
    {
        auto fitRect = [](const Eigen::Vector4i& innerRect, const Eigen::Vector4i& outerRect) {
          Eigen::Vector4i fittedRect = innerRect;
          fittedRect[0] = std::max<int>(fittedRect[0], outerRect[0]);
          fittedRect[0] = std::min<int>(fittedRect[0], outerRect[0] + outerRect[2] - 1);
          fittedRect[1] = std::max<int>(fittedRect[1], outerRect[1]);
          fittedRect[1] = std::min<int>(fittedRect[1], outerRect[1] + outerRect[3] - 1);
          fittedRect[2] = std::min<int>(fittedRect[2], outerRect[0] + outerRect[2] - fittedRect[0]);
          fittedRect[3] = std::min<int>(fittedRect[3], outerRect[1] + outerRect[3] - fittedRect[1]);
          return fittedRect;
        };
        // make sure offset and extent are inbound
        viewportRect = fitRect(viewportRect, Eigen::Vector4i(0, 0, Width(), Height()));
        scissorsRect = fitRect(scissorsRect, viewportRect);

        VkCommandBuffer buffer = m_transientCommandPool->BeginSingleTimeCommands();

        VkRenderPassBeginInfo renderPassInfo = {};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = clear ? m_renderPassClear : m_renderPassNoClear;
        renderPassInfo.framebuffer = m_rgbaFrameBuffer->Framebuffer();
        renderPassInfo.renderArea.offset = {0, 0};
        renderPassInfo.renderArea.extent = {uint32_t(Width()), uint32_t(Height())};

        std::array<VkClearValue, 2> clearValues = {};
        Eigen::Map<Eigen::Vector4f>((float*)&(clearValues[0].color)) = clearColor;
        clearValues[1].depthStencil = {0.0f, 0};
        renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
        renderPassInfo.pClearValues = clearValues.data();

        vkCmdBeginRenderPass(buffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

        VkViewport viewport = {};
        viewport.x = float(viewportRect[0]);
        viewport.y = float(viewportRect[1]);
        viewport.width = float(viewportRect[2]);
        viewport.height = float(viewportRect[3]);
        viewport.minDepth = 0.0f; // invert with 0 being far?!
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(buffer, 0, 1, &viewport);

        VkRect2D scissor = {};
        scissor.offset = {scissorsRect[0], scissorsRect[1]};
        scissor.extent = {uint32_t(scissorsRect[2]), uint32_t(scissorsRect[3])};
        vkCmdSetScissor(buffer, 0, 1, &scissor);

        return buffer;
    }

    void FinalizeRender(VkCommandBuffer buffer, bool prepareForSampling = false)
    {
        vkCmdEndRenderPass(buffer);

        if (prepareForSampling) {
            PrepareForSampling(buffer);
        }

        m_transientCommandPool->EndSingleTimeCommands(buffer);
    }

    void PrepareForSampling(VkCommandBuffer buffer)
    {
        m_vulkanMemory->transitionImageLayout(buffer, m_rgbaImage->Image(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, m_rgbaImage->Aspect());
        if (Samples() == VK_SAMPLE_COUNT_1_BIT) {
            m_vulkanMemory->transitionImageLayout(buffer, m_multiDepthImage->Image(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, m_multiDepthImage->Aspect());
        }
    }

    void PrepareForSampling()
    {
        VkCommandBuffer buffer = m_transientCommandPool->BeginSingleTimeCommands();
        PrepareForSampling(buffer);
        m_transientCommandPool->EndSingleTimeCommands(buffer);
    }

    const VulkanAllInOneTexture& RGBATexture()
    {
        if (!m_rgbaTexture) {
            m_rgbaTexture = VulkanAllInOneTexture::Create(m_vulkanDevice);
            m_rgbaTexture->SetTextureImage(m_rgbaImage);
        }
        return *m_rgbaTexture;
    }

    const VulkanAllInOneTexture& DepthTexture()
    {
        if (!m_depthTexture) {
            if (m_samples != VK_SAMPLE_COUNT_1_BIT) {
                CARBON_CRITICAL("depth texture is only supported for single sample images");
            }

            m_depthTexture = VulkanAllInOneTexture::Create(m_vulkanDevice);
            m_depthTexture->SetTextureImage(m_multiDepthImage);
        }
        return *m_depthTexture;
    }

    VkRenderPass RenderPass(bool clear) { return clear ? m_renderPassClear : m_renderPassNoClear; }

    VkImage Image() { return m_rgbaImage->Image(); }

    template <class T>
    void CopyImage(T* dataOut)
    {
        if (!m_rgbaBuffer) {
            m_rgbaBuffer = m_vulkanMemory->createBuffer(Width() * Height() * 4 * sizeof(T), VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        }

        VkCommandBuffer buffer = m_transientCommandPool->BeginSingleTimeCommands();
        m_vulkanMemory->copyImageToBuffer(buffer, m_rgbaImage->Image(), m_rgbaBuffer->Buffer(), m_rgbaImage->Width(), m_rgbaImage->Height());
        m_transientCommandPool->EndSingleTimeCommands(buffer);

        void* data = nullptr;
        vkMapMemory(m_vulkanDevice->Device(), m_rgbaBuffer->DeviceMemory(),  m_rgbaBuffer->Offset(), m_rgbaBuffer->Size(), 0, &data);
        memcpy(dataOut, data, m_rgbaBuffer->Size());
        vkUnmapMemory(m_vulkanDevice->Device(), m_rgbaBuffer->DeviceMemory());
    }

    void CopyDepthImage(float* depthOutput)
    {
        if (m_samples != VK_SAMPLE_COUNT_1_BIT) {
            CARBON_CRITICAL("depth read is not compatible with multisampling");
        }

        if (!m_depthBuffer) {
            m_depthBuffer = m_vulkanMemory->createBuffer(Width() * Height() * sizeof(float), VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        }

        VkCommandBuffer buffer = m_transientCommandPool->BeginSingleTimeCommands();
        m_vulkanMemory->copyImageToBuffer(buffer, m_multiDepthImage->Image(), m_depthBuffer->Buffer(), m_multiDepthImage->Width(), m_multiDepthImage->Height(), VK_IMAGE_ASPECT_DEPTH_BIT);
        m_transientCommandPool->EndSingleTimeCommands(buffer);

        void* data = nullptr;
        vkMapMemory(m_vulkanDevice->Device(), m_depthBuffer->DeviceMemory(),  m_depthBuffer->Offset(), m_depthBuffer->Size(), 0, &data);
        memcpy(depthOutput, data, m_depthBuffer->Size());
        vkUnmapMemory(m_vulkanDevice->Device(), m_depthBuffer->DeviceMemory());
    }

    float CopyDepthPixel(int x, int y)
    {
        if (m_samples != VK_SAMPLE_COUNT_1_BIT) {
            CARBON_CRITICAL("depth read is not compatible with multisampling");
        }

        if (x < 0 || x >= m_multiDepthImage->Width() || y < 0 || y >= m_multiDepthImage->Height()) {
            return 0;
        }

        if (!m_depthBuffer) {
            m_depthBuffer = m_vulkanMemory->createBuffer(Width() * Height() * 4, VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        }

        VkCommandBuffer buffer = m_transientCommandPool->BeginSingleTimeCommands();
        m_vulkanMemory->copyPixelToBuffer(buffer, m_multiDepthImage->Image(), m_depthBuffer->Buffer(), x, y, VK_IMAGE_ASPECT_DEPTH_BIT);
        m_transientCommandPool->EndSingleTimeCommands(buffer);

        void* data = nullptr;
        vkMapMemory(m_vulkanDevice->Device(), m_depthBuffer->DeviceMemory(),  m_depthBuffer->Offset(), sizeof(float), 0, &data);
        float depth = ((float*)data)[0];
        vkUnmapMemory(m_vulkanDevice->Device(), m_depthBuffer->DeviceMemory());
        return depth;
    }

    const std::shared_ptr<VulkanDevice>& Device() const { return m_vulkanDevice; }
    const std::shared_ptr<VulkanMemory>& Memory() const { return m_vulkanMemory; }


private:
    VulkanRenderTarget(std::shared_ptr<VulkanDevice> vulkanDevice, int width, int height, VkSampleCountFlagBits samples, VkFormat format)
    {
        const bool useMultiSampling = (samples != VK_SAMPLE_COUNT_1_BIT);
        m_samples = samples;
        m_format = format;
        m_vulkanDevice = vulkanDevice;
        m_vulkanMemory = VulkanMemory::Create(m_vulkanDevice);
        m_transientCommandPool = VulkanCommandPool::Create(m_vulkanDevice, /*transient=*/true);

        VkAttachmentDescription colorAttachment = {};
        colorAttachment.format = m_format;
        colorAttachment.samples = m_samples;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        colorAttachment.finalLayout = useMultiSampling ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL : VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

        VkAttachmentReference colorAttachmentRef = {};
        colorAttachmentRef.attachment = 0;
        colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentDescription depthAttachment = {};
        depthAttachment.format = VK_FORMAT_D32_SFLOAT;
        depthAttachment.samples = m_samples;
        depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        depthAttachment.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

        VkAttachmentReference depthAttachmentRef = {};
        depthAttachmentRef.attachment = 1;
        depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentDescription colorAttachmentResolve{};
        colorAttachmentResolve.format = m_format;
        colorAttachmentResolve.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachmentResolve.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachmentResolve.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachmentResolve.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachmentResolve.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachmentResolve.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        colorAttachmentResolve.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

        VkAttachmentReference colorAttachmentResolveRef{};
        colorAttachmentResolveRef.attachment = 2;
        colorAttachmentResolveRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass = {};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorAttachmentRef;
        subpass.pDepthStencilAttachment = &depthAttachmentRef;
        if (useMultiSampling) {
            subpass.pResolveAttachments = &colorAttachmentResolveRef;
        }

        std::array<VkSubpassDependency, 2> dependencies;
        dependencies[0].dependencyFlags = 0;
        dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[0].dstSubpass = 0;
        dependencies[0].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[0].srcAccessMask = 0;
        dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

        dependencies[1].dependencyFlags = 0;
        dependencies[1].srcSubpass = 0;
        dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[1].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;

        std::vector<VkAttachmentDescription> attachments;
        if (useMultiSampling) {
            attachments = {colorAttachment, depthAttachment, colorAttachmentResolve};
        } else {
            attachments = {colorAttachment, depthAttachment};
        }
        VkRenderPassCreateInfo renderPassInfo = {};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        renderPassInfo.pAttachments = attachments.data();
        renderPassInfo.subpassCount = 1;
        renderPassInfo.pSubpasses = &subpass;
        renderPassInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
        renderPassInfo.pDependencies = dependencies.data();

        if (vkCreateRenderPass(vulkanDevice->Device(), &renderPassInfo, nullptr, &m_renderPassClear) != VK_SUCCESS) {
            CARBON_CRITICAL("failed to create render pass!");
        }

        // using the render pass without clear assumes that we already ran a renderpass with clear. therefore the layout should be the same as the final layout
        attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
        attachments[0].initialLayout = attachments[0].finalLayout;
        attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
        attachments[1].initialLayout = attachments[1].finalLayout;


        if (vkCreateRenderPass(vulkanDevice->Device(), &renderPassInfo, nullptr, &m_renderPassNoClear) != VK_SUCCESS) {
            CARBON_CRITICAL("failed to create render pass!");
        }

        Resize(width, height);
    }

    VulkanRenderTarget(const VulkanRenderTarget&) = delete;
    VulkanRenderTarget& operator=(const VulkanRenderTarget&) = delete;

private:
    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;
    VkSampleCountFlagBits m_samples;
    VkFormat m_format;
    VkRenderPass m_renderPassClear = VK_NULL_HANDLE;
    VkRenderPass m_renderPassNoClear = VK_NULL_HANDLE;
    std::shared_ptr<VulkanManagedImage> m_multiRgbaImage;
    std::shared_ptr<VulkanManagedImage> m_multiDepthImage;
    std::shared_ptr<VulkanAllInOneTexture> m_depthTexture;
    std::shared_ptr<VulkanManagedImage> m_rgbaImage;
    std::shared_ptr<VulkanAllInOneTexture> m_rgbaTexture;
    std::unique_ptr<VulkanFramebuffer> m_rgbaFrameBuffer;
    std::unique_ptr<VulkanManagedBuffer> m_rgbaBuffer;
    std::unique_ptr<VulkanManagedBuffer> m_depthBuffer;


    std::unique_ptr<VulkanCommandPool> m_transientCommandPool;
};



} // namespace nls
} //namespace epic
