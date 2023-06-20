// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <memory>

#ifndef VK_NO_PROTOTYPES
#define VK_NO_PROTOTYPES
#endif
#include <vulkan/vulkan.h>

namespace epic {
namespace nls {

class VulkanLibrary;

//-----------------------------------------------------------------------------
//! VulkanLoader is loading Vulkan functions dynamically
//! There are functions that can be used
//! - prior VkInstance is created (loaded with Init())
//! - once VkInstance is created (loaded with InitFunc(VkInstance)
//! - once VkDevice is created (loaded with InitFunc(VkDevice)
//! 
//! VulkanLoader is created by VulkanLoader::Init().
//! There's always one instance of VulkanLoader (set by Init()), which is obtained
//! by VulkanLoader::Instance().
//! This is done to simplify the code, i.e. not to set VulkanLoader as argument.
//! 
//! How to use:
//! // Include this prior any code that needs Vulkan functions
//! #include <nls/Vulkan/common/VulkanFunctions.h>
//!
//! int main()
//! ...
//!    // Initialize functions
//!    VulkanLoader::Instance()->Init(opts);
//!    ...
//!    VulkanInstance::Create() will call VulkanLoader::Instance()->InitFunc(VkInstance);
//!    ...
//!    VulkanDevice::Create() will call VulkanLoader::Instance()->InitFunc(VkDevice);
//-----------------------------------------------------------------------------
class VulkanLoader
{
public:

	enum OptionType
	{
		cOptUseSurface,
		cOptUseDebugUtils,

		cOptCount
	};

	//! Loader options (which extensions to enable/load)
	class Options
	{
	public:

		Options& OptionSet(OptionType type, uint32_t value = 1)
		{
			m_options[type] = value;
			return *this;
		}

		uint32_t OptionGet(OptionType type) const
		{
			return m_options[type];
		}

	private:

		uint32_t	m_options[cOptCount];
	};

	//! Initialize loader
	static bool Init(const Options& options = Options())
	{
		if (s_inst.m_init)
			return true;

		return s_inst.Load(options);
	}

	//! Return loader instance
	static VulkanLoader* Instance() { return &s_inst; }

	~VulkanLoader();

	//! Return if library is loaded
	bool IsLoaded() const { return m_init; }

	//! Initialize Vulkan functions that require Vulkan instance
	bool InitFunc(VkInstance inst);

	//! Initialize Vulkan functions that require Vulkan device
	bool InitFunc(VkDevice dev);

private:

	VulkanLoader();

	//! Load Vulkan library
	bool Load(const Options& options);

	//! Initialize Vulkan functions that don't require Vulkan instance
	bool InitFunc();

	// Members:
	static VulkanLoader					s_inst;

	std::unique_ptr< VulkanLibrary >	m_lib{};
	Options								m_options{};
	bool								m_init = false;
};

} //namespace nls
} //namespace epic

extern PFN_vkGetInstanceProcAddr                                        vkGetInstanceProcAddr;
extern PFN_vkCreateInstance                                             vkCreateInstance;
extern PFN_vkEnumerateInstanceExtensionProperties                       vkEnumerateInstanceExtensionProperties;
extern PFN_vkEnumerateInstanceLayerProperties                           vkEnumerateInstanceLayerProperties;

extern PFN_vkDestroyInstance                                            vkDestroyInstance;
extern PFN_vkEnumeratePhysicalDevices                                   vkEnumeratePhysicalDevices;
extern PFN_vkGetPhysicalDeviceFeatures                                  vkGetPhysicalDeviceFeatures;
extern PFN_vkGetPhysicalDeviceFormatProperties                          vkGetPhysicalDeviceFormatProperties;
extern PFN_vkGetPhysicalDeviceImageFormatProperties                     vkGetPhysicalDeviceImageFormatProperties;
extern PFN_vkGetPhysicalDeviceProperties                                vkGetPhysicalDeviceProperties;
extern PFN_vkGetPhysicalDeviceQueueFamilyProperties                     vkGetPhysicalDeviceQueueFamilyProperties;
extern PFN_vkGetPhysicalDeviceMemoryProperties                          vkGetPhysicalDeviceMemoryProperties;
extern PFN_vkGetDeviceProcAddr                                          vkGetDeviceProcAddr;
extern PFN_vkCreateDevice                                               vkCreateDevice;
extern PFN_vkEnumerateDeviceExtensionProperties                         vkEnumerateDeviceExtensionProperties;
extern PFN_vkEnumerateDeviceLayerProperties                             vkEnumerateDeviceLayerProperties;
extern PFN_vkQueueSubmit                                                vkQueueSubmit;
extern PFN_vkQueueWaitIdle                                              vkQueueWaitIdle;
extern PFN_vkGetPhysicalDeviceSparseImageFormatProperties               vkGetPhysicalDeviceSparseImageFormatProperties;
extern PFN_vkQueueBindSparse                                            vkQueueBindSparse;

extern PFN_vkDestroyDevice                                              vkDestroyDevice;
extern PFN_vkGetDeviceQueue                                             vkGetDeviceQueue;
extern PFN_vkDeviceWaitIdle                                             vkDeviceWaitIdle;
extern PFN_vkAllocateMemory                                             vkAllocateMemory;
extern PFN_vkFreeMemory                                                 vkFreeMemory;
extern PFN_vkMapMemory                                                  vkMapMemory;
extern PFN_vkUnmapMemory                                                vkUnmapMemory;
extern PFN_vkFlushMappedMemoryRanges                                    vkFlushMappedMemoryRanges;
extern PFN_vkInvalidateMappedMemoryRanges                               vkInvalidateMappedMemoryRanges;
extern PFN_vkGetDeviceMemoryCommitment                                  vkGetDeviceMemoryCommitment;
extern PFN_vkBindBufferMemory                                           vkBindBufferMemory;
extern PFN_vkBindImageMemory                                            vkBindImageMemory;
extern PFN_vkGetBufferMemoryRequirements                                vkGetBufferMemoryRequirements;
extern PFN_vkGetImageMemoryRequirements                                 vkGetImageMemoryRequirements;
extern PFN_vkGetImageSparseMemoryRequirements                           vkGetImageSparseMemoryRequirements;
extern PFN_vkCreateFence                                                vkCreateFence;
extern PFN_vkDestroyFence                                               vkDestroyFence;
extern PFN_vkResetFences                                                vkResetFences;
extern PFN_vkGetFenceStatus                                             vkGetFenceStatus;
extern PFN_vkWaitForFences                                              vkWaitForFences;
extern PFN_vkCreateSemaphore                                            vkCreateSemaphore;
extern PFN_vkDestroySemaphore                                           vkDestroySemaphore;
extern PFN_vkCreateEvent                                                vkCreateEvent;
extern PFN_vkDestroyEvent                                               vkDestroyEvent;
extern PFN_vkGetEventStatus                                             vkGetEventStatus;
extern PFN_vkSetEvent                                                   vkSetEvent;
extern PFN_vkResetEvent                                                 vkResetEvent;
extern PFN_vkCreateQueryPool                                            vkCreateQueryPool;
extern PFN_vkDestroyQueryPool                                           vkDestroyQueryPool;
extern PFN_vkGetQueryPoolResults                                        vkGetQueryPoolResults;
extern PFN_vkCreateBuffer                                               vkCreateBuffer;
extern PFN_vkDestroyBuffer                                              vkDestroyBuffer;
extern PFN_vkCreateBufferView                                           vkCreateBufferView;
extern PFN_vkDestroyBufferView                                          vkDestroyBufferView;
extern PFN_vkCreateImage                                                vkCreateImage;
extern PFN_vkDestroyImage                                               vkDestroyImage;
extern PFN_vkGetImageSubresourceLayout                                  vkGetImageSubresourceLayout;
extern PFN_vkCreateImageView                                            vkCreateImageView;
extern PFN_vkDestroyImageView                                           vkDestroyImageView;
extern PFN_vkCreateShaderModule                                         vkCreateShaderModule;
extern PFN_vkDestroyShaderModule                                        vkDestroyShaderModule;
extern PFN_vkCreatePipelineCache                                        vkCreatePipelineCache;
extern PFN_vkDestroyPipelineCache                                       vkDestroyPipelineCache;
extern PFN_vkGetPipelineCacheData                                       vkGetPipelineCacheData;
extern PFN_vkMergePipelineCaches                                        vkMergePipelineCaches;
extern PFN_vkCreateGraphicsPipelines                                    vkCreateGraphicsPipelines;
extern PFN_vkCreateComputePipelines                                     vkCreateComputePipelines;
extern PFN_vkDestroyPipeline                                            vkDestroyPipeline;
extern PFN_vkCreatePipelineLayout                                       vkCreatePipelineLayout;
extern PFN_vkDestroyPipelineLayout                                      vkDestroyPipelineLayout;
extern PFN_vkCreateSampler                                              vkCreateSampler;
extern PFN_vkDestroySampler                                             vkDestroySampler;
extern PFN_vkCreateDescriptorSetLayout                                  vkCreateDescriptorSetLayout;
extern PFN_vkDestroyDescriptorSetLayout                                 vkDestroyDescriptorSetLayout;
extern PFN_vkCreateDescriptorPool                                       vkCreateDescriptorPool;
extern PFN_vkDestroyDescriptorPool                                      vkDestroyDescriptorPool;
extern PFN_vkResetDescriptorPool                                        vkResetDescriptorPool;
extern PFN_vkAllocateDescriptorSets                                     vkAllocateDescriptorSets;
extern PFN_vkFreeDescriptorSets                                         vkFreeDescriptorSets;
extern PFN_vkUpdateDescriptorSets                                       vkUpdateDescriptorSets;
extern PFN_vkCreateFramebuffer                                          vkCreateFramebuffer;
extern PFN_vkDestroyFramebuffer                                         vkDestroyFramebuffer;
extern PFN_vkCreateRenderPass                                           vkCreateRenderPass;
extern PFN_vkDestroyRenderPass                                          vkDestroyRenderPass;
extern PFN_vkGetRenderAreaGranularity                                   vkGetRenderAreaGranularity;
extern PFN_vkCreateCommandPool                                          vkCreateCommandPool;
extern PFN_vkDestroyCommandPool                                         vkDestroyCommandPool;
extern PFN_vkResetCommandPool                                           vkResetCommandPool;
extern PFN_vkAllocateCommandBuffers                                     vkAllocateCommandBuffers;
extern PFN_vkFreeCommandBuffers                                         vkFreeCommandBuffers;
extern PFN_vkBeginCommandBuffer                                         vkBeginCommandBuffer;
extern PFN_vkEndCommandBuffer                                           vkEndCommandBuffer;
extern PFN_vkResetCommandBuffer                                         vkResetCommandBuffer;
extern PFN_vkCmdBindPipeline                                            vkCmdBindPipeline;
extern PFN_vkCmdSetViewport                                             vkCmdSetViewport;
extern PFN_vkCmdSetScissor                                              vkCmdSetScissor;
extern PFN_vkCmdSetLineWidth                                            vkCmdSetLineWidth;
extern PFN_vkCmdSetDepthBias                                            vkCmdSetDepthBias;
extern PFN_vkCmdSetBlendConstants                                       vkCmdSetBlendConstants;
extern PFN_vkCmdSetDepthBounds                                          vkCmdSetDepthBounds;
extern PFN_vkCmdSetStencilCompareMask                                   vkCmdSetStencilCompareMask;
extern PFN_vkCmdSetStencilWriteMask                                     vkCmdSetStencilWriteMask;
extern PFN_vkCmdSetStencilReference                                     vkCmdSetStencilReference;
extern PFN_vkCmdBindDescriptorSets                                      vkCmdBindDescriptorSets;
extern PFN_vkCmdBindIndexBuffer                                         vkCmdBindIndexBuffer;
extern PFN_vkCmdBindVertexBuffers                                       vkCmdBindVertexBuffers;
extern PFN_vkCmdDraw                                                    vkCmdDraw;
extern PFN_vkCmdDrawIndexed                                             vkCmdDrawIndexed;
extern PFN_vkCmdDrawIndirect                                            vkCmdDrawIndirect;
extern PFN_vkCmdDrawIndexedIndirect                                     vkCmdDrawIndexedIndirect;
extern PFN_vkCmdDispatch                                                vkCmdDispatch;
extern PFN_vkCmdDispatchIndirect                                        vkCmdDispatchIndirect;
extern PFN_vkCmdCopyBuffer                                              vkCmdCopyBuffer;
extern PFN_vkCmdCopyImage                                               vkCmdCopyImage;
extern PFN_vkCmdBlitImage                                               vkCmdBlitImage;
extern PFN_vkCmdCopyBufferToImage                                       vkCmdCopyBufferToImage;
extern PFN_vkCmdCopyImageToBuffer                                       vkCmdCopyImageToBuffer;
extern PFN_vkCmdUpdateBuffer                                            vkCmdUpdateBuffer;
extern PFN_vkCmdFillBuffer                                              vkCmdFillBuffer;
extern PFN_vkCmdClearColorImage                                         vkCmdClearColorImage;
extern PFN_vkCmdClearDepthStencilImage                                  vkCmdClearDepthStencilImage;
extern PFN_vkCmdClearAttachments                                        vkCmdClearAttachments;
extern PFN_vkCmdResolveImage                                            vkCmdResolveImage;
extern PFN_vkCmdSetEvent                                                vkCmdSetEvent;
extern PFN_vkCmdResetEvent                                              vkCmdResetEvent;
extern PFN_vkCmdWaitEvents                                              vkCmdWaitEvents;
extern PFN_vkCmdPipelineBarrier                                         vkCmdPipelineBarrier;
extern PFN_vkCmdBeginQuery                                              vkCmdBeginQuery;
extern PFN_vkCmdEndQuery                                                vkCmdEndQuery;
extern PFN_vkCmdResetQueryPool                                          vkCmdResetQueryPool;
extern PFN_vkCmdWriteTimestamp                                          vkCmdWriteTimestamp;
extern PFN_vkCmdCopyQueryPoolResults                                    vkCmdCopyQueryPoolResults;
extern PFN_vkCmdPushConstants                                           vkCmdPushConstants;
extern PFN_vkCmdBeginRenderPass                                         vkCmdBeginRenderPass;
extern PFN_vkCmdNextSubpass                                             vkCmdNextSubpass;
extern PFN_vkCmdEndRenderPass                                           vkCmdEndRenderPass;
extern PFN_vkCmdExecuteCommands                                         vkCmdExecuteCommands;

// KHR_surface
extern PFN_vkDestroySurfaceKHR                                          vkDestroySurfaceKHR;
extern PFN_vkGetPhysicalDeviceSurfaceSupportKHR                         vkGetPhysicalDeviceSurfaceSupportKHR;
extern PFN_vkGetPhysicalDeviceSurfaceCapabilitiesKHR                    vkGetPhysicalDeviceSurfaceCapabilitiesKHR;
extern PFN_vkGetPhysicalDeviceSurfaceFormatsKHR                         vkGetPhysicalDeviceSurfaceFormatsKHR;
extern PFN_vkGetPhysicalDeviceSurfacePresentModesKHR                    vkGetPhysicalDeviceSurfacePresentModesKHR;

// KHR_swapchain
extern PFN_vkCreateSwapchainKHR                                         vkCreateSwapchainKHR;
extern PFN_vkDestroySwapchainKHR                                        vkDestroySwapchainKHR;
extern PFN_vkGetSwapchainImagesKHR                                      vkGetSwapchainImagesKHR;
extern PFN_vkAcquireNextImageKHR                                        vkAcquireNextImageKHR;
extern PFN_vkQueuePresentKHR                                            vkQueuePresentKHR;
extern PFN_vkGetDeviceGroupPresentCapabilitiesKHR                       vkGetDeviceGroupPresentCapabilitiesKHR;

// FIXME: This is Vulkan 1.1
//extern PFN_vkAcquireNextImage2KHR                                       vkAcquireNextImage2KHR;
//extern PFN_vkGetDeviceGroupSurfacePresentModesKHR                       vkGetDeviceGroupSurfacePresentModesKHR;
//extern PFN_vkGetPhysicalDevicePresentRectanglesKHR                      vkGetPhysicalDevicePresentRectanglesKHR;

// KHR_get_physical_device_properties2
extern PFN_vkGetPhysicalDeviceFeatures2KHR								vkGetPhysicalDeviceFeatures2KHR;
extern PFN_vkGetPhysicalDeviceProperties2KHR							vkGetPhysicalDeviceProperties2KHR;
extern PFN_vkGetPhysicalDeviceFormatProperties2KHR						vkGetPhysicalDeviceFormatProperties2KHR;
extern PFN_vkGetPhysicalDeviceImageFormatProperties2KHR					vkGetPhysicalDeviceImageFormatProperties2KHR;
extern PFN_vkGetPhysicalDeviceQueueFamilyProperties2KHR					vkGetPhysicalDeviceQueueFamilyProperties2KHR;
extern PFN_vkGetPhysicalDeviceMemoryProperties2KHR						vkGetPhysicalDeviceMemoryProperties2KHR;
extern PFN_vkGetPhysicalDeviceSparseImageFormatProperties2KHR			vkGetPhysicalDeviceSparseImageFormatProperties2KHR;

// KHR_push_descriptor
extern PFN_vkCmdPushDescriptorSetKHR									vkCmdPushDescriptorSetKHR;

// EXT_debug_utils
extern PFN_vkSetDebugUtilsObjectNameEXT                                 vkSetDebugUtilsObjectNameEXT;
extern PFN_vkSetDebugUtilsObjectTagEXT                                  vkSetDebugUtilsObjectTagEXT;
extern PFN_vkQueueBeginDebugUtilsLabelEXT                               vkQueueBeginDebugUtilsLabelEXT;
extern PFN_vkQueueEndDebugUtilsLabelEXT                                 vkQueueEndDebugUtilsLabelEXT;
extern PFN_vkQueueInsertDebugUtilsLabelEXT                              vkQueueInsertDebugUtilsLabelEXT;
extern PFN_vkCmdBeginDebugUtilsLabelEXT                                 vkCmdBeginDebugUtilsLabelEXT;
extern PFN_vkCmdEndDebugUtilsLabelEXT                                   vkCmdEndDebugUtilsLabelEXT;
extern PFN_vkCmdInsertDebugUtilsLabelEXT                                vkCmdInsertDebugUtilsLabelEXT;
extern PFN_vkCreateDebugUtilsMessengerEXT                               vkCreateDebugUtilsMessengerEXT;
extern PFN_vkDestroyDebugUtilsMessengerEXT                              vkDestroyDebugUtilsMessengerEXT;
extern PFN_vkSubmitDebugUtilsMessageEXT                                 vkSubmitDebugUtilsMessageEXT;

// EXT_host_query_reset
extern PFN_vkResetQueryPoolEXT											vkResetQueryPoolEXT;
