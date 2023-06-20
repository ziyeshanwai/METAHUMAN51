// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDevice.h>

namespace epic {
namespace nls {


class VulkanFramebuffer
{
public:
    ~VulkanFramebuffer()
    {
        if (m_frameBuffer) {
            vkDestroyFramebuffer(m_device->Device(), m_frameBuffer, nullptr);
        }
    }

    static std::unique_ptr<VulkanFramebuffer> Create(std::shared_ptr<VulkanDevice> device, int width, int height, VkRenderPass renderPass, const std::vector<VkImageView>& attachments) {
        VkFramebufferCreateInfo framebufferInfo = {};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = renderPass;
        framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width = width;
        framebufferInfo.height = height;
        framebufferInfo.layers = 1;

        VkFramebuffer frameBuffer;
        if (vkCreateFramebuffer(device->Device(), &framebufferInfo, nullptr, &frameBuffer) != VK_SUCCESS) {
            throw std::runtime_error("failed to create framebuffer!");
        }
        std::unique_ptr<VulkanFramebuffer> instance(new VulkanFramebuffer);
        instance->m_frameBuffer = frameBuffer;
        instance->m_device = device;
        return instance;
    }

    VkFramebuffer Framebuffer() const { return m_frameBuffer; }

private:
    VulkanFramebuffer() = default;

private:
    std::shared_ptr<VulkanDevice> m_device;
    VkFramebuffer m_frameBuffer = VK_NULL_HANDLE;
};


} // namespace nls
} //namespace epic
