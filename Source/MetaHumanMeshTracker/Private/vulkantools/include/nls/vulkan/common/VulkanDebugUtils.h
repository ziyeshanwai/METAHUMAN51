// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <stdint.h>
#include "VulkanFunctions.h"

//-----------------------------------------------------------------------------
//! Insert debug label for command buffer for current scope
//-----------------------------------------------------------------------------
#define VK_DEBUG_SCOPE_COMMAND_BUFFER(CmdBuff, Name, Color) \
    epic::nls::VulkanScopedDebugLabel< epic::nls::VulkanCommandBufferDebugLabel > dbgCmdBuff##Name(CmdBuff, #Name, Color)

//-----------------------------------------------------------------------------
//! Insert debug label for queue for current scope
//-----------------------------------------------------------------------------
#define VK_DEBUG_SCOPE_QUEUE(Queue, Name, Color) \
    epic::nls::VulkanScopedDebugLabel< epic::nls::VulkanQueueDebugLabel > dbgCmdBuff##Name(Queue, #Name, Color)

namespace epic {
namespace nls {

//-----------------------------------------------------------------------------
//! Helper class for inserting debug labels into Vulkan command buffer
//-----------------------------------------------------------------------------
class VulkanDebugLabel
{
public:

    using ColorType = uint32_t;

    static constexpr ColorType cColRed         = 0xfff44336;
    static constexpr ColorType cColDarkRed     = 0xffb71c1c;
    static constexpr ColorType cColCoral       = 0xffef9a9a;
    static constexpr ColorType cColRichRed     = 0xffff0000;
    static constexpr ColorType cColPink        = 0xffe91e63;
    static constexpr ColorType cColRose        = 0xffff80ab;
    static constexpr ColorType cColPurple      = 0xff9c27b0;
    static constexpr ColorType cColMagenta     = 0xffe040fb;
    static constexpr ColorType cColDarkMagenta = 0xffaa00ff;
    static constexpr ColorType cColDeepPurple  = 0xff673ab7;
    static constexpr ColorType cColIndigo      = 0xff3f51b5;
    static constexpr ColorType cColBlue        = 0xff2196f3;
    static constexpr ColorType cColDarkBlue    = 0xff0d47a1;
    static constexpr ColorType cColRichBlue    = 0xff0000ff;
    static constexpr ColorType cColLightBlue   = 0xff03a9f4;
    static constexpr ColorType cColSkyBlue     = 0xff80d8ff;
    static constexpr ColorType cColNavy        = 0xff0277bd;
    static constexpr ColorType cColCyan        = 0xff00bcd4;
    static constexpr ColorType cColDarkCyan    = 0xff006064;
    static constexpr ColorType cColTeal        = 0xff009688;
    static constexpr ColorType cColDarkTeal    = 0xff004d40;
    static constexpr ColorType cColGreen       = 0xff4caf50;
    static constexpr ColorType cColDarkGreen   = 0xff1b5e20;
    static constexpr ColorType cColRichGreen   = 0xff00ff00;
    static constexpr ColorType cColLightGreen  = 0xff8bc34a;
    static constexpr ColorType cColMint        = 0xff33691e;
    static constexpr ColorType cColLime        = 0xffcddc39;
    static constexpr ColorType cColOlive       = 0xff827717;
    static constexpr ColorType cColYellow      = 0xffffeb3b;
    static constexpr ColorType cColRichYellow  = 0xffffff00;
    static constexpr ColorType cColAmber       = 0xffffc107;
    static constexpr ColorType cColGold        = 0xffffd54f;
    static constexpr ColorType cColPaleGold    = 0xffffe57f;
    static constexpr ColorType cColOrange      = 0xffff9800;
    static constexpr ColorType cColSkin        = 0xffffccbc;
    static constexpr ColorType cColDeepOrange  = 0xffff5722;
    static constexpr ColorType cColBrick       = 0xffbf360c;
    static constexpr ColorType cColBrown       = 0xff795548;
    static constexpr ColorType cColDarkBrown   = 0xff3e2723;
    static constexpr ColorType cColCreamWhite  = 0xfffff3e0;
    static constexpr ColorType cColWheat       = 0xffffecb3;
    static constexpr ColorType cColGrey        = 0xff9e9e9e;
    static constexpr ColorType cColDark        = 0xff212121;
    static constexpr ColorType cColSilver      = 0xffe0e0e0;
    static constexpr ColorType cColBlueGrey    = 0xff607d8b;

    static constexpr ColorType cColDefault     = cColGrey;

protected:

    //! Helper function to populate the VkDebugUtilsLabelEXT
    VkDebugUtilsLabelEXT MakeLabel(const char* name, ColorType col)
    {
        VkDebugUtilsLabelEXT lbl = {};
        lbl.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;

        lbl.pLabelName = name;
        lbl.color[3] = ((col & 0x00ff0000) >> 24) / 255.0f;
        lbl.color[0] = ((col & 0x00ff0000) >> 16) / 255.0f;
        lbl.color[1] = ((col & 0x0000ff00) >>  8) / 255.0f;
        lbl.color[2] =  (col & 0x00000000)        / 255.0f;

        return lbl;

    }

};

//-----------------------------------------------------------------------------
//! Debug label for command buffer
//-----------------------------------------------------------------------------
class VulkanCommandBufferDebugLabel : public VulkanDebugLabel
{
public:

    //! Initialize the command buffer
    VulkanCommandBufferDebugLabel(VkCommandBuffer cmdBuff)
        : m_cmdBuff(cmdBuff)
    {}

    //! Insert begin
    void Begin(const char* name, ColorType color = cColDefault)
    {
        VkDebugUtilsLabelEXT lbl = MakeLabel(name, color);

        vkCmdBeginDebugUtilsLabelEXT(m_cmdBuff, &lbl);
    }

    //! Insert end
    void End()
    {
        vkCmdEndDebugUtilsLabelEXT(m_cmdBuff);
    }

    //! Insert label
    void Insert(const char* name, ColorType color = cColDefault)
    {
        VkDebugUtilsLabelEXT lbl = MakeLabel(name, color);
        
        vkCmdInsertDebugUtilsLabelEXT(m_cmdBuff, &lbl);
    }

private:

    VkCommandBuffer     m_cmdBuff{ nullptr };
};

//-----------------------------------------------------------------------------
//! Debug label for queue
//-----------------------------------------------------------------------------
class VulkanQueueDebugLabel : public VulkanDebugLabel
{
public:

    //! Initialize the queue
    VulkanQueueDebugLabel(VkQueue queue)
        : m_queue(queue)
    {}

    //! Insert begin
    void Begin(const char* name, ColorType color = cColDefault)
    {
        VkDebugUtilsLabelEXT lbl = MakeLabel(name, color);

        vkQueueBeginDebugUtilsLabelEXT(m_queue, &lbl);
    }

    //! Insert end
    void End()
    {
        vkQueueEndDebugUtilsLabelEXT(m_queue);
    }

    //! Insert label
    void Insert(const char* name, ColorType color = cColDefault)
    {
       VkDebugUtilsLabelEXT lbl = MakeLabel(name, color);

       vkQueueInsertDebugUtilsLabelEXT(m_queue, &lbl);
    }

private:

    VkQueue     m_queue{ nullptr };
};

//-----------------------------------------------------------------------------
//! Scoped lablel is a helper class for inserting label for a scope
//-----------------------------------------------------------------------------
template< typename ObjTypeT >
class VulkanScopedDebugLabel;

template< >
class VulkanScopedDebugLabel< VulkanCommandBufferDebugLabel >
{
public:

    VulkanScopedDebugLabel(VkCommandBuffer cmdBuff, const char* name, VulkanDebugLabel::ColorType color)
        : m_label(cmdBuff)
    {
        m_label.Begin(name, color);
    }

    ~VulkanScopedDebugLabel()
    {
        m_label.End();
    }

private:

    VulkanCommandBufferDebugLabel m_label;
};


template< >
class VulkanScopedDebugLabel< VulkanQueueDebugLabel >
{
public:

    VulkanScopedDebugLabel(VkQueue queue, const char* name, VulkanDebugLabel::ColorType color)
        : m_label(queue)
    {
        m_label.Begin(name, color);
    }

    ~VulkanScopedDebugLabel()
    {
        m_label.End();
    }

private:

    VulkanQueueDebugLabel m_label;
};

//-----------------------------------------------------------------------------
//! Set an object name (visible with Vulkan debugging tools)
//-----------------------------------------------------------------------------
class VulkanObjectName
{
public:

    //! Set queue name
    static void Set(VkDevice dev, VkQueue queue, const char* name)
    {
        auto info = MakeInfo(VK_OBJECT_TYPE_QUEUE, queue, name);

        vkSetDebugUtilsObjectNameEXT(dev, &info);
    }

    //! Set semaphore name
    static void Set(VkDevice dev, VkSemaphore semaphore, const char* name)
    {
        auto info = MakeInfo(VK_OBJECT_TYPE_SEMAPHORE, semaphore, name);

        vkSetDebugUtilsObjectNameEXT(dev, &info);
    }

    //! Set fence name
    static void Set(VkDevice dev, VkFence fence, const char* name)
    {
        auto info = MakeInfo(VK_OBJECT_TYPE_FENCE, fence, name);

        vkSetDebugUtilsObjectNameEXT(dev, &info);
    }

    //! Set descriptor set layout name
    static void Set(VkDevice dev, VkDescriptorSetLayout descLayout, const char* name)
    {
        auto info = MakeInfo(VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT, descLayout, name);

        vkSetDebugUtilsObjectNameEXT(dev, &info);
    }

    //! Set pipeline name
    static void Set(VkDevice dev, VkPipeline pipe, const char* name)
    {
        auto info = MakeInfo(VK_OBJECT_TYPE_PIPELINE, pipe, name);

        vkSetDebugUtilsObjectNameEXT(dev, &info);
    }

    //! Set pipeline layout name
    static void Set(VkDevice dev, VkPipelineLayout pipeLayout, const char* name)
    {
        auto info = MakeInfo(VK_OBJECT_TYPE_PIPELINE_LAYOUT, pipeLayout, name);

        vkSetDebugUtilsObjectNameEXT(dev, &info);
    }

    //! Set shader module name
    static void Set(VkDevice dev, VkShaderModule shdMod, const char* name)
    {
        auto info = MakeInfo(VK_OBJECT_TYPE_SHADER_MODULE, shdMod, name);

        vkSetDebugUtilsObjectNameEXT(dev, &info);
    }

    //! Set command buffer name
    static void Set(VkDevice dev, VkCommandBuffer cmdBuff, const char* name)
    {
        auto info = MakeInfo(VK_OBJECT_TYPE_COMMAND_BUFFER, cmdBuff, name);

        vkSetDebugUtilsObjectNameEXT(dev, &info);
    }

    //! Set buffer name
    static void Set(VkDevice dev, VkBuffer buffer, const char* name)
    {
        auto info = MakeInfo(VK_OBJECT_TYPE_BUFFER, buffer, name);

        vkSetDebugUtilsObjectNameEXT(dev, &info);
    }

    //! Set image name
    static void Set(VkDevice dev, VkImage image, const char* name)
    {
        auto info = MakeInfo(VK_OBJECT_TYPE_IMAGE, image, name);

        vkSetDebugUtilsObjectNameEXT(dev, &info);
    }

private:

    //! Utility function to populate the VkDebugUtilsObjectNameInfoEXT
    template< typename ObjHandleT >
    static VkDebugUtilsObjectNameInfoEXT MakeInfo(VkObjectType type, ObjHandleT handle, const char* name)
    {
        VkDebugUtilsObjectNameInfoEXT   info = {};
        info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT;

        info.objectType = type;
        info.objectHandle = (uint64_t) handle;
        info.pObjectName = name;

        return info;
    }
};

} // nls
} // epic
