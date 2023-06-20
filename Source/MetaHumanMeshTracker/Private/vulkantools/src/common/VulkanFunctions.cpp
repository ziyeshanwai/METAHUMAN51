// Copyright Epic Games, Inc. All Rights Reserved.

#include  <nls/vulkan/common/VulkanFunctions.h>

#if defined(_MSC_VER)
__pragma(warning(push))
__pragma(warning(disable:4191))
#endif

#if defined(_WIN64)
	#define VK_USE_PLATFORM_WIN32_KHR 1
	#define VK_LIBRARY_NAME "vulkan-1"

extern "C"
{
	__declspec(dllimport) void* __stdcall LoadLibraryA(const char*);
	__declspec(dllimport) PFN_vkVoidFunction __stdcall GetProcAddress(void*, const char*);
}

	static void* _ShLibLoad(const char* name)
	{
		return ::LoadLibraryA(name);
	}

	static PFN_vkVoidFunction _ShLibSymbol(void* shlib, const char* symName)
	{
		return ::GetProcAddress(shlib, symName);
	}

#ifdef MINIMAL_WINDOWS_API
	// in UE5 FreeLibrary is declared as Windows::FreeLibrary in "Windows/MinimalWindowsApi.h" and which may be included by the build tool
	static void _ShLibUnload(void* shlib)
	{
		Windows::FreeLibrary((Windows::HMODULE)shlib);
	}
#else
extern "C"
{
	__declspec(dllimport) int  __stdcall FreeLibrary(void*);
}
	static void _ShLibUnload(void* shlib)
	{
		FreeLibrary(shlib);
	}
#endif

#elif (defined(__APPLE__) || defined(__linux__)) && !defined(__ANDROID__)

	#if defined(__APPLE__)
		#define VK_LIBRARY_NAME "libvulkan.1.dylib"
	#else
		#define VK_LIBRARY_NAME "libvulkan.so"
	#endif

	#include <dlfcn.h>

	static void* _ShLibLoad(const char* name)
	{
		return dlopen(name, RTLD_NOW | RTLD_LOCAL);
	}

	static PFN_vkVoidFunction _ShLibSymbol(void* shlib, const char* symName)
	{
		return (PFN_vkVoidFunction) dlsym(shlib, symName);
	}

	static void _ShLibUnload(void* shlib)
	{
		dlclose(shlib);
	}

#else
	#error "Unsupported platform!"
#endif

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
epic::nls::VulkanLoader epic::nls::VulkanLoader::s_inst;

namespace epic {
namespace nls {

//-----------------------------------------------------------------------------
//! Vulkan Library helper for windows
//-----------------------------------------------------------------------------
class VulkanLibrary
{
public:

	VulkanLibrary() = default;

	//! Load the library
	bool Load()
	{
		m_lib = _ShLibLoad(VK_LIBRARY_NAME);
		if (!m_lib)
			return false;

		vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr) _ShLibSymbol(m_lib, "vkGetInstanceProcAddr");
		if (!vkGetInstanceProcAddr)
			return false;

		return true;
	}

	//! Unload the library
	void Unload()
	{
		if (m_lib)
		{
			_ShLibUnload(m_lib);
			m_lib = nullptr;
		}
	}

private:

	void*		m_lib{ nullptr };
};

} // nls
} // epic

PFN_vkGetInstanceProcAddr                                        vkGetInstanceProcAddr = nullptr;
PFN_vkCreateInstance                                             vkCreateInstance = nullptr;
PFN_vkEnumerateInstanceExtensionProperties                       vkEnumerateInstanceExtensionProperties = nullptr;
PFN_vkEnumerateInstanceLayerProperties                           vkEnumerateInstanceLayerProperties = nullptr;

PFN_vkDestroyInstance                                            vkDestroyInstance = nullptr;
PFN_vkEnumeratePhysicalDevices                                   vkEnumeratePhysicalDevices = nullptr;
PFN_vkGetPhysicalDeviceFeatures                                  vkGetPhysicalDeviceFeatures = nullptr;
PFN_vkGetPhysicalDeviceFormatProperties                          vkGetPhysicalDeviceFormatProperties = nullptr;
PFN_vkGetPhysicalDeviceImageFormatProperties                     vkGetPhysicalDeviceImageFormatProperties = nullptr;
PFN_vkGetPhysicalDeviceProperties                                vkGetPhysicalDeviceProperties = nullptr;
PFN_vkGetPhysicalDeviceQueueFamilyProperties                     vkGetPhysicalDeviceQueueFamilyProperties = nullptr;
PFN_vkGetPhysicalDeviceMemoryProperties                          vkGetPhysicalDeviceMemoryProperties = nullptr;
PFN_vkGetDeviceProcAddr                                          vkGetDeviceProcAddr = nullptr;
PFN_vkCreateDevice                                               vkCreateDevice = nullptr;
PFN_vkEnumerateDeviceExtensionProperties                         vkEnumerateDeviceExtensionProperties = nullptr;
PFN_vkEnumerateDeviceLayerProperties                             vkEnumerateDeviceLayerProperties = nullptr;
PFN_vkQueueSubmit                                                vkQueueSubmit = nullptr;
PFN_vkQueueWaitIdle                                              vkQueueWaitIdle = nullptr;
PFN_vkGetPhysicalDeviceSparseImageFormatProperties               vkGetPhysicalDeviceSparseImageFormatProperties = nullptr;
PFN_vkQueueBindSparse                                            vkQueueBindSparse = nullptr;

PFN_vkDestroyDevice                                              vkDestroyDevice = nullptr;
PFN_vkGetDeviceQueue                                             vkGetDeviceQueue = nullptr;
PFN_vkDeviceWaitIdle                                             vkDeviceWaitIdle = nullptr;
PFN_vkAllocateMemory                                             vkAllocateMemory = nullptr;
PFN_vkFreeMemory                                                 vkFreeMemory = nullptr;
PFN_vkMapMemory                                                  vkMapMemory = nullptr;
PFN_vkUnmapMemory                                                vkUnmapMemory = nullptr;
PFN_vkFlushMappedMemoryRanges                                    vkFlushMappedMemoryRanges = nullptr;
PFN_vkInvalidateMappedMemoryRanges                               vkInvalidateMappedMemoryRanges = nullptr;
PFN_vkGetDeviceMemoryCommitment                                  vkGetDeviceMemoryCommitment = nullptr;
PFN_vkBindBufferMemory                                           vkBindBufferMemory = nullptr;
PFN_vkBindImageMemory                                            vkBindImageMemory = nullptr;
PFN_vkGetBufferMemoryRequirements                                vkGetBufferMemoryRequirements = nullptr;
PFN_vkGetImageMemoryRequirements                                 vkGetImageMemoryRequirements = nullptr;
PFN_vkGetImageSparseMemoryRequirements                           vkGetImageSparseMemoryRequirements = nullptr;
PFN_vkCreateFence                                                vkCreateFence = nullptr;
PFN_vkDestroyFence                                               vkDestroyFence = nullptr;
PFN_vkResetFences                                                vkResetFences = nullptr;
PFN_vkGetFenceStatus                                             vkGetFenceStatus = nullptr;
PFN_vkWaitForFences                                              vkWaitForFences = nullptr;
PFN_vkCreateSemaphore                                            vkCreateSemaphore = nullptr;
PFN_vkDestroySemaphore                                           vkDestroySemaphore = nullptr;
PFN_vkCreateEvent                                                vkCreateEvent = nullptr;
PFN_vkDestroyEvent                                               vkDestroyEvent = nullptr;
PFN_vkGetEventStatus                                             vkGetEventStatus = nullptr;
PFN_vkSetEvent                                                   vkSetEvent = nullptr;
PFN_vkResetEvent                                                 vkResetEvent = nullptr;
PFN_vkCreateQueryPool                                            vkCreateQueryPool = nullptr;
PFN_vkDestroyQueryPool                                           vkDestroyQueryPool = nullptr;
PFN_vkGetQueryPoolResults                                        vkGetQueryPoolResults = nullptr;
PFN_vkCreateBuffer                                               vkCreateBuffer = nullptr;
PFN_vkDestroyBuffer                                              vkDestroyBuffer = nullptr;
PFN_vkCreateBufferView                                           vkCreateBufferView = nullptr;
PFN_vkDestroyBufferView                                          vkDestroyBufferView = nullptr;
PFN_vkCreateImage                                                vkCreateImage = nullptr;
PFN_vkDestroyImage                                               vkDestroyImage = nullptr;
PFN_vkGetImageSubresourceLayout                                  vkGetImageSubresourceLayout = nullptr;
PFN_vkCreateImageView                                            vkCreateImageView = nullptr;
PFN_vkDestroyImageView                                           vkDestroyImageView = nullptr;
PFN_vkCreateShaderModule                                         vkCreateShaderModule = nullptr;
PFN_vkDestroyShaderModule                                        vkDestroyShaderModule = nullptr;
PFN_vkCreatePipelineCache                                        vkCreatePipelineCache = nullptr;
PFN_vkDestroyPipelineCache                                       vkDestroyPipelineCache = nullptr;
PFN_vkGetPipelineCacheData                                       vkGetPipelineCacheData = nullptr;
PFN_vkMergePipelineCaches                                        vkMergePipelineCaches = nullptr;
PFN_vkCreateGraphicsPipelines                                    vkCreateGraphicsPipelines = nullptr;
PFN_vkCreateComputePipelines                                     vkCreateComputePipelines = nullptr;
PFN_vkDestroyPipeline                                            vkDestroyPipeline = nullptr;
PFN_vkCreatePipelineLayout                                       vkCreatePipelineLayout = nullptr;
PFN_vkDestroyPipelineLayout                                      vkDestroyPipelineLayout = nullptr;
PFN_vkCreateSampler                                              vkCreateSampler = nullptr;
PFN_vkDestroySampler                                             vkDestroySampler = nullptr;
PFN_vkCreateDescriptorSetLayout                                  vkCreateDescriptorSetLayout = nullptr;
PFN_vkDestroyDescriptorSetLayout                                 vkDestroyDescriptorSetLayout = nullptr;
PFN_vkCreateDescriptorPool                                       vkCreateDescriptorPool = nullptr;
PFN_vkDestroyDescriptorPool                                      vkDestroyDescriptorPool = nullptr;
PFN_vkResetDescriptorPool                                        vkResetDescriptorPool = nullptr;
PFN_vkAllocateDescriptorSets                                     vkAllocateDescriptorSets = nullptr;
PFN_vkFreeDescriptorSets                                         vkFreeDescriptorSets = nullptr;
PFN_vkUpdateDescriptorSets                                       vkUpdateDescriptorSets = nullptr;
PFN_vkCreateFramebuffer                                          vkCreateFramebuffer = nullptr;
PFN_vkDestroyFramebuffer                                         vkDestroyFramebuffer = nullptr;
PFN_vkCreateRenderPass                                           vkCreateRenderPass = nullptr;
PFN_vkDestroyRenderPass                                          vkDestroyRenderPass = nullptr;
PFN_vkGetRenderAreaGranularity                                   vkGetRenderAreaGranularity = nullptr;
PFN_vkCreateCommandPool                                          vkCreateCommandPool = nullptr;
PFN_vkDestroyCommandPool                                         vkDestroyCommandPool = nullptr;
PFN_vkResetCommandPool                                           vkResetCommandPool = nullptr;
PFN_vkAllocateCommandBuffers                                     vkAllocateCommandBuffers = nullptr;
PFN_vkFreeCommandBuffers                                         vkFreeCommandBuffers = nullptr;
PFN_vkBeginCommandBuffer                                         vkBeginCommandBuffer = nullptr;
PFN_vkEndCommandBuffer                                           vkEndCommandBuffer = nullptr;
PFN_vkResetCommandBuffer                                         vkResetCommandBuffer = nullptr;
PFN_vkCmdBindPipeline                                            vkCmdBindPipeline = nullptr;
PFN_vkCmdSetViewport                                             vkCmdSetViewport = nullptr;
PFN_vkCmdSetScissor                                              vkCmdSetScissor = nullptr;
PFN_vkCmdSetLineWidth                                            vkCmdSetLineWidth = nullptr;
PFN_vkCmdSetDepthBias                                            vkCmdSetDepthBias = nullptr;
PFN_vkCmdSetBlendConstants                                       vkCmdSetBlendConstants = nullptr;
PFN_vkCmdSetDepthBounds                                          vkCmdSetDepthBounds = nullptr;
PFN_vkCmdSetStencilCompareMask                                   vkCmdSetStencilCompareMask = nullptr;
PFN_vkCmdSetStencilWriteMask                                     vkCmdSetStencilWriteMask = nullptr;
PFN_vkCmdSetStencilReference                                     vkCmdSetStencilReference = nullptr;
PFN_vkCmdBindDescriptorSets                                      vkCmdBindDescriptorSets = nullptr;
PFN_vkCmdBindIndexBuffer                                         vkCmdBindIndexBuffer = nullptr;
PFN_vkCmdBindVertexBuffers                                       vkCmdBindVertexBuffers = nullptr;
PFN_vkCmdDraw                                                    vkCmdDraw = nullptr;
PFN_vkCmdDrawIndexed                                             vkCmdDrawIndexed = nullptr;
PFN_vkCmdDrawIndirect                                            vkCmdDrawIndirect = nullptr;
PFN_vkCmdDrawIndexedIndirect                                     vkCmdDrawIndexedIndirect = nullptr;
PFN_vkCmdDispatch                                                vkCmdDispatch = nullptr;
PFN_vkCmdDispatchIndirect                                        vkCmdDispatchIndirect = nullptr;
PFN_vkCmdCopyBuffer                                              vkCmdCopyBuffer = nullptr;
PFN_vkCmdCopyImage                                               vkCmdCopyImage = nullptr;
PFN_vkCmdBlitImage                                               vkCmdBlitImage = nullptr;
PFN_vkCmdCopyBufferToImage                                       vkCmdCopyBufferToImage = nullptr;
PFN_vkCmdCopyImageToBuffer                                       vkCmdCopyImageToBuffer = nullptr;
PFN_vkCmdUpdateBuffer                                            vkCmdUpdateBuffer = nullptr;
PFN_vkCmdFillBuffer                                              vkCmdFillBuffer = nullptr;
PFN_vkCmdClearColorImage                                         vkCmdClearColorImage = nullptr;
PFN_vkCmdClearDepthStencilImage                                  vkCmdClearDepthStencilImage = nullptr;
PFN_vkCmdClearAttachments                                        vkCmdClearAttachments = nullptr;
PFN_vkCmdResolveImage                                            vkCmdResolveImage = nullptr;
PFN_vkCmdSetEvent                                                vkCmdSetEvent = nullptr;
PFN_vkCmdResetEvent                                              vkCmdResetEvent = nullptr;
PFN_vkCmdWaitEvents                                              vkCmdWaitEvents = nullptr;
PFN_vkCmdPipelineBarrier                                         vkCmdPipelineBarrier = nullptr;
PFN_vkCmdBeginQuery                                              vkCmdBeginQuery = nullptr;
PFN_vkCmdEndQuery                                                vkCmdEndQuery = nullptr;
PFN_vkCmdResetQueryPool                                          vkCmdResetQueryPool = nullptr;
PFN_vkCmdWriteTimestamp                                          vkCmdWriteTimestamp = nullptr;
PFN_vkCmdCopyQueryPoolResults                                    vkCmdCopyQueryPoolResults = nullptr;
PFN_vkCmdPushConstants                                           vkCmdPushConstants = nullptr;
PFN_vkCmdBeginRenderPass                                         vkCmdBeginRenderPass = nullptr;
PFN_vkCmdNextSubpass                                             vkCmdNextSubpass = nullptr;
PFN_vkCmdEndRenderPass                                           vkCmdEndRenderPass = nullptr;
PFN_vkCmdExecuteCommands                                         vkCmdExecuteCommands = nullptr;

// KHR_surface
PFN_vkDestroySurfaceKHR                                          vkDestroySurfaceKHR = nullptr;
PFN_vkGetPhysicalDeviceSurfaceSupportKHR                         vkGetPhysicalDeviceSurfaceSupportKHR = nullptr;
PFN_vkGetPhysicalDeviceSurfaceCapabilitiesKHR                    vkGetPhysicalDeviceSurfaceCapabilitiesKHR = nullptr;
PFN_vkGetPhysicalDeviceSurfaceFormatsKHR                         vkGetPhysicalDeviceSurfaceFormatsKHR = nullptr;
PFN_vkGetPhysicalDeviceSurfacePresentModesKHR                    vkGetPhysicalDeviceSurfacePresentModesKHR = nullptr;

// KHR_swapchain
PFN_vkCreateSwapchainKHR                                         vkCreateSwapchainKHR = nullptr;
PFN_vkDestroySwapchainKHR                                        vkDestroySwapchainKHR = nullptr;
PFN_vkGetSwapchainImagesKHR                                      vkGetSwapchainImagesKHR = nullptr;
PFN_vkAcquireNextImageKHR                                        vkAcquireNextImageKHR = nullptr;
PFN_vkQueuePresentKHR                                            vkQueuePresentKHR = nullptr;
// Vulkan 1.1
//PFN_vkGetDeviceGroupPresentCapabilitiesKHR                       vkGetDeviceGroupPresentCapabilitiesKHR = nullptr;
//PFN_vkGetDeviceGroupSurfacePresentModesKHR                       vkGetDeviceGroupSurfacePresentModesKHR = nullptr;
//PFN_vkGetPhysicalDevicePresentRectanglesKHR                      vkGetPhysicalDevicePresentRectanglesKHR = nullptr;
//PFN_vkAcquireNextImage2KHR                                       vkAcquireNextImage2KHR = nullptr;

// KHR_get_physical_device_properties2
PFN_vkGetPhysicalDeviceFeatures2KHR                              vkGetPhysicalDeviceFeatures2KHR = nullptr;
PFN_vkGetPhysicalDeviceProperties2KHR                            vkGetPhysicalDeviceProperties2KHR = nullptr;
PFN_vkGetPhysicalDeviceFormatProperties2KHR                      vkGetPhysicalDeviceFormatProperties2KHR = nullptr;
PFN_vkGetPhysicalDeviceImageFormatProperties2KHR                 vkGetPhysicalDeviceImageFormatProperties2KHR = nullptr;
PFN_vkGetPhysicalDeviceQueueFamilyProperties2KHR                 vkGetPhysicalDeviceQueueFamilyProperties2KHR = nullptr;
PFN_vkGetPhysicalDeviceMemoryProperties2KHR                      vkGetPhysicalDeviceMemoryProperties2KHR = nullptr;
PFN_vkGetPhysicalDeviceSparseImageFormatProperties2KHR           vkGetPhysicalDeviceSparseImageFormatProperties2KHR = nullptr;

// KHR_push_descriptor
PFN_vkCmdPushDescriptorSetKHR									 vkCmdPushDescriptorSetKHR = nullptr;

// EXT_debug_util
PFN_vkSetDebugUtilsObjectNameEXT                                 vkSetDebugUtilsObjectNameEXT = nullptr;
PFN_vkSetDebugUtilsObjectTagEXT                                  vkSetDebugUtilsObjectTagEXT = nullptr;
PFN_vkQueueBeginDebugUtilsLabelEXT                               vkQueueBeginDebugUtilsLabelEXT = nullptr;
PFN_vkQueueEndDebugUtilsLabelEXT                                 vkQueueEndDebugUtilsLabelEXT = nullptr;
PFN_vkQueueInsertDebugUtilsLabelEXT                              vkQueueInsertDebugUtilsLabelEXT = nullptr;
PFN_vkCmdBeginDebugUtilsLabelEXT                                 vkCmdBeginDebugUtilsLabelEXT = nullptr;
PFN_vkCmdEndDebugUtilsLabelEXT                                   vkCmdEndDebugUtilsLabelEXT = nullptr;
PFN_vkCmdInsertDebugUtilsLabelEXT                                vkCmdInsertDebugUtilsLabelEXT = nullptr;
PFN_vkCreateDebugUtilsMessengerEXT                               vkCreateDebugUtilsMessengerEXT = nullptr;
PFN_vkDestroyDebugUtilsMessengerEXT                              vkDestroyDebugUtilsMessengerEXT = nullptr;
PFN_vkSubmitDebugUtilsMessageEXT                                 vkSubmitDebugUtilsMessageEXT = nullptr;

VkResult Null_vkSetDebugUtilsObjectNameEXT(VkDevice /*device*/, const VkDebugUtilsObjectNameInfoEXT* /*pNameInfo*/)
{
	return VK_SUCCESS;
}

VkResult Null_vkSetDebugUtilsObjectTagEXT(VkDevice /*device*/, const VkDebugUtilsObjectTagInfoEXT* /*pTagInfo*/)
{
	return VK_SUCCESS;
}

void Null_vkQueueBeginDebugUtilsLabelEXT(VkQueue /*queue*/, const VkDebugUtilsLabelEXT* /*pLabelInfo*/)
{
}

void Null_vkQueueEndDebugUtilsLabelEXT(VkQueue /*queue*/)
{
}

void Null_vkQueueInsertDebugUtilsLabelEXT(VkQueue /*queue*/, const VkDebugUtilsLabelEXT* /*pLabelInfo*/)
{
}

void Null_vkCmdBeginDebugUtilsLabelEXT(VkCommandBuffer /*commandBuffer*/, const VkDebugUtilsLabelEXT* /*pLabelInfo*/)
{
}

void Null_vkCmdEndDebugUtilsLabelEXT(VkCommandBuffer /*commandBuffer*/)
{
}

void Null_vkCmdInsertDebugUtilsLabelEXT(VkCommandBuffer /*commandBuffer*/, const VkDebugUtilsLabelEXT* /*pLabelInfo*/)
{
}

VkResult Null_vkCreateDebugUtilsMessengerEXT(VkInstance /*instance*/, const VkDebugUtilsMessengerCreateInfoEXT* /*pCreateInfo*/, const VkAllocationCallbacks* /*pAllocator*/, VkDebugUtilsMessengerEXT* pMessenger)
{
	pMessenger[0] = nullptr;
	return VK_SUCCESS;
}

void Null_vkDestroyDebugUtilsMessengerEXT(VkInstance /*instance*/, VkDebugUtilsMessengerEXT /*messenger*/, const VkAllocationCallbacks* /*pAllocator*/)
{
}

void Null_vkSubmitDebugUtilsMessageEXT(VkInstance /*instance*/, VkDebugUtilsMessageSeverityFlagBitsEXT /*messageSeverity*/, VkDebugUtilsMessageTypeFlagsEXT /*messageTypes*/, const VkDebugUtilsMessengerCallbackDataEXT* /*pCallbackData*/)
{
}

// EXT_host_query_reset
PFN_vkResetQueryPoolEXT											 vkResetQueryPoolEXT = nullptr;

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#define VK_GetProcAddr(Proc) \
	Proc = (PFN_##Proc) (vkGetInstanceProcAddr(NULL, #Proc)); \
	if (!Proc) \
	{ \
		Proc = nullptr; \
		return false; \
	}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#define VK_GetInstanceProcAddr(Inst, Proc) \
	Proc = (PFN_##Proc) (vkGetInstanceProcAddr(Inst, #Proc)); \
	if (!Proc) \
	{ \
		Proc = nullptr; \
		return false; \
	}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#define VK_GetDeviceProcAddr(Dev, Proc) \
	Proc = (PFN_##Proc) (vkGetDeviceProcAddr(Dev, #Proc)); \
	if (!Proc) \
	{ \
		Proc = nullptr; \
		return false; \
	}

namespace epic::nls {


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
VulkanLoader::VulkanLoader()
	: m_lib(new VulkanLibrary())
{
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
VulkanLoader::~VulkanLoader()
{
	m_lib->Unload();
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool VulkanLoader::Load(const VulkanLoader::Options& options)
{
	m_options = options;
	m_init = false;

	if (m_lib->Load())
		m_init = InitFunc();

	return m_init;
}


//-----------------------------------------------------------------------------
// Initialize Vulkan functions that don't require Vulkan instance
//-----------------------------------------------------------------------------
bool VulkanLoader::InitFunc()
{
	if (!m_lib)
		return false;

	VK_GetProcAddr(vkCreateInstance);
	VK_GetProcAddr(vkEnumerateInstanceExtensionProperties);
	VK_GetProcAddr(vkEnumerateInstanceLayerProperties);
	return true;
}

//-----------------------------------------------------------------------------
// Initialize Vulkan functions that require Vulkan instance
//-----------------------------------------------------------------------------
bool VulkanLoader::InitFunc(VkInstance inst)
{
	if (!m_lib)
		return false;

	VK_GetInstanceProcAddr(inst, vkDestroyInstance);
	VK_GetInstanceProcAddr(inst, vkEnumeratePhysicalDevices);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceFeatures);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceFormatProperties);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceImageFormatProperties);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceProperties);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceQueueFamilyProperties);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceMemoryProperties);
	VK_GetInstanceProcAddr(inst, vkGetDeviceProcAddr);
	VK_GetInstanceProcAddr(inst, vkCreateDevice);
	VK_GetInstanceProcAddr(inst, vkEnumerateDeviceExtensionProperties);
	VK_GetInstanceProcAddr(inst, vkEnumerateDeviceLayerProperties);
	VK_GetInstanceProcAddr(inst, vkQueueSubmit);
	VK_GetInstanceProcAddr(inst, vkQueueWaitIdle);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceSparseImageFormatProperties);
	VK_GetInstanceProcAddr(inst, vkQueueBindSparse);

	// KHR_surface
	if (m_options.OptionGet(cOptUseSurface))
	{
		VK_GetInstanceProcAddr(inst, vkDestroySurfaceKHR);
		VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceSurfaceSupportKHR);
		VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceSurfaceCapabilitiesKHR);
		VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceSurfaceFormatsKHR);
		VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceSurfacePresentModesKHR);
	}

	// KHR_get_physical_device_properties2
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceFeatures2KHR);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceProperties2KHR);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceFormatProperties2KHR);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceImageFormatProperties2KHR);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceQueueFamilyProperties2KHR);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceMemoryProperties2KHR);
	VK_GetInstanceProcAddr(inst, vkGetPhysicalDeviceSparseImageFormatProperties2KHR);

	// EXT_debug_util
	if (m_options.OptionGet(cOptUseDebugUtils))
	{
		VK_GetInstanceProcAddr(inst, vkSetDebugUtilsObjectNameEXT);
		VK_GetInstanceProcAddr(inst, vkSetDebugUtilsObjectTagEXT);
		VK_GetInstanceProcAddr(inst, vkQueueBeginDebugUtilsLabelEXT);
		VK_GetInstanceProcAddr(inst, vkQueueEndDebugUtilsLabelEXT);
		VK_GetInstanceProcAddr(inst, vkQueueInsertDebugUtilsLabelEXT);
		VK_GetInstanceProcAddr(inst, vkCmdBeginDebugUtilsLabelEXT);
		VK_GetInstanceProcAddr(inst, vkCmdEndDebugUtilsLabelEXT);
		VK_GetInstanceProcAddr(inst, vkCmdInsertDebugUtilsLabelEXT);
		VK_GetInstanceProcAddr(inst, vkCreateDebugUtilsMessengerEXT);
		VK_GetInstanceProcAddr(inst, vkDestroyDebugUtilsMessengerEXT);
		VK_GetInstanceProcAddr(inst, vkSubmitDebugUtilsMessageEXT);
	}
	else
	{
		vkSetDebugUtilsObjectNameEXT	= Null_vkSetDebugUtilsObjectNameEXT;
		vkSetDebugUtilsObjectTagEXT		= Null_vkSetDebugUtilsObjectTagEXT;
		vkQueueBeginDebugUtilsLabelEXT	= Null_vkQueueBeginDebugUtilsLabelEXT;
		vkQueueEndDebugUtilsLabelEXT	= Null_vkQueueEndDebugUtilsLabelEXT;
		vkQueueInsertDebugUtilsLabelEXT = Null_vkQueueInsertDebugUtilsLabelEXT;
		vkCmdBeginDebugUtilsLabelEXT	= Null_vkCmdBeginDebugUtilsLabelEXT;
		vkCmdEndDebugUtilsLabelEXT		= Null_vkCmdEndDebugUtilsLabelEXT;
		vkCmdInsertDebugUtilsLabelEXT	= Null_vkCmdInsertDebugUtilsLabelEXT;
		vkCreateDebugUtilsMessengerEXT	= Null_vkCreateDebugUtilsMessengerEXT;
		vkDestroyDebugUtilsMessengerEXT = Null_vkDestroyDebugUtilsMessengerEXT;
		vkSubmitDebugUtilsMessageEXT	= Null_vkSubmitDebugUtilsMessageEXT;
	}

	return true;
}

//-----------------------------------------------------------------------------
// Initialize Vulkan functions that require Vulkan device
//-----------------------------------------------------------------------------
bool VulkanLoader::InitFunc(VkDevice dev)
{
	if (!m_lib)
		return false;

	VK_GetDeviceProcAddr(dev, vkDestroyDevice);
	VK_GetDeviceProcAddr(dev, vkGetDeviceQueue);
	VK_GetDeviceProcAddr(dev, vkDeviceWaitIdle);
	VK_GetDeviceProcAddr(dev, vkAllocateMemory);
	VK_GetDeviceProcAddr(dev, vkFreeMemory);
	VK_GetDeviceProcAddr(dev, vkMapMemory);
	VK_GetDeviceProcAddr(dev, vkUnmapMemory);
	VK_GetDeviceProcAddr(dev, vkFlushMappedMemoryRanges);
	VK_GetDeviceProcAddr(dev, vkInvalidateMappedMemoryRanges);
	VK_GetDeviceProcAddr(dev, vkGetDeviceMemoryCommitment);
	VK_GetDeviceProcAddr(dev, vkBindBufferMemory);
	VK_GetDeviceProcAddr(dev, vkBindImageMemory);
	VK_GetDeviceProcAddr(dev, vkGetBufferMemoryRequirements);
	VK_GetDeviceProcAddr(dev, vkGetImageMemoryRequirements);
	VK_GetDeviceProcAddr(dev, vkGetImageSparseMemoryRequirements);
	VK_GetDeviceProcAddr(dev, vkCreateFence);
	VK_GetDeviceProcAddr(dev, vkDestroyFence);
	VK_GetDeviceProcAddr(dev, vkResetFences);
	VK_GetDeviceProcAddr(dev, vkGetFenceStatus);
	VK_GetDeviceProcAddr(dev, vkWaitForFences);
	VK_GetDeviceProcAddr(dev, vkCreateSemaphore);
	VK_GetDeviceProcAddr(dev, vkDestroySemaphore);
	VK_GetDeviceProcAddr(dev, vkCreateEvent);
	VK_GetDeviceProcAddr(dev, vkDestroyEvent);
	VK_GetDeviceProcAddr(dev, vkGetEventStatus);
	VK_GetDeviceProcAddr(dev, vkSetEvent);
	VK_GetDeviceProcAddr(dev, vkResetEvent);
	VK_GetDeviceProcAddr(dev, vkCreateQueryPool);
	VK_GetDeviceProcAddr(dev, vkDestroyQueryPool);
	VK_GetDeviceProcAddr(dev, vkGetQueryPoolResults);
	VK_GetDeviceProcAddr(dev, vkCreateBuffer);
	VK_GetDeviceProcAddr(dev, vkDestroyBuffer);
	VK_GetDeviceProcAddr(dev, vkCreateBufferView);
	VK_GetDeviceProcAddr(dev, vkDestroyBufferView);
	VK_GetDeviceProcAddr(dev, vkCreateImage);
	VK_GetDeviceProcAddr(dev, vkDestroyImage);
	VK_GetDeviceProcAddr(dev, vkGetImageSubresourceLayout);
	VK_GetDeviceProcAddr(dev, vkCreateImageView);
	VK_GetDeviceProcAddr(dev, vkDestroyImageView);
	VK_GetDeviceProcAddr(dev, vkCreateShaderModule);
	VK_GetDeviceProcAddr(dev, vkDestroyShaderModule);
	VK_GetDeviceProcAddr(dev, vkCreatePipelineCache);
	VK_GetDeviceProcAddr(dev, vkDestroyPipelineCache);
	VK_GetDeviceProcAddr(dev, vkGetPipelineCacheData);
	VK_GetDeviceProcAddr(dev, vkMergePipelineCaches);
	VK_GetDeviceProcAddr(dev, vkCreateGraphicsPipelines);
	VK_GetDeviceProcAddr(dev, vkCreateComputePipelines);
	VK_GetDeviceProcAddr(dev, vkDestroyPipeline);
	VK_GetDeviceProcAddr(dev, vkCreatePipelineLayout);
	VK_GetDeviceProcAddr(dev, vkDestroyPipelineLayout);
	VK_GetDeviceProcAddr(dev, vkCreateSampler);
	VK_GetDeviceProcAddr(dev, vkDestroySampler);
	VK_GetDeviceProcAddr(dev, vkCreateDescriptorSetLayout);
	VK_GetDeviceProcAddr(dev, vkDestroyDescriptorSetLayout);
	VK_GetDeviceProcAddr(dev, vkCreateDescriptorPool);
	VK_GetDeviceProcAddr(dev, vkDestroyDescriptorPool);
	VK_GetDeviceProcAddr(dev, vkResetDescriptorPool);
	VK_GetDeviceProcAddr(dev, vkAllocateDescriptorSets);
	VK_GetDeviceProcAddr(dev, vkFreeDescriptorSets);
	VK_GetDeviceProcAddr(dev, vkUpdateDescriptorSets);
	VK_GetDeviceProcAddr(dev, vkCreateFramebuffer);
	VK_GetDeviceProcAddr(dev, vkDestroyFramebuffer);
	VK_GetDeviceProcAddr(dev, vkCreateRenderPass);
	VK_GetDeviceProcAddr(dev, vkDestroyRenderPass);
	VK_GetDeviceProcAddr(dev, vkGetRenderAreaGranularity);
	VK_GetDeviceProcAddr(dev, vkCreateCommandPool);
	VK_GetDeviceProcAddr(dev, vkDestroyCommandPool);
	VK_GetDeviceProcAddr(dev, vkResetCommandPool);
	VK_GetDeviceProcAddr(dev, vkAllocateCommandBuffers);
	VK_GetDeviceProcAddr(dev, vkFreeCommandBuffers);
	VK_GetDeviceProcAddr(dev, vkBeginCommandBuffer);
	VK_GetDeviceProcAddr(dev, vkEndCommandBuffer);
	VK_GetDeviceProcAddr(dev, vkResetCommandBuffer);
	VK_GetDeviceProcAddr(dev, vkCmdBindPipeline);
	VK_GetDeviceProcAddr(dev, vkCmdSetViewport);
	VK_GetDeviceProcAddr(dev, vkCmdSetScissor);
	VK_GetDeviceProcAddr(dev, vkCmdSetLineWidth);
	VK_GetDeviceProcAddr(dev, vkCmdSetDepthBias);
	VK_GetDeviceProcAddr(dev, vkCmdSetBlendConstants);
	VK_GetDeviceProcAddr(dev, vkCmdSetDepthBounds);
	VK_GetDeviceProcAddr(dev, vkCmdSetStencilCompareMask);
	VK_GetDeviceProcAddr(dev, vkCmdSetStencilWriteMask);
	VK_GetDeviceProcAddr(dev, vkCmdSetStencilReference);
	VK_GetDeviceProcAddr(dev, vkCmdBindDescriptorSets);
	VK_GetDeviceProcAddr(dev, vkCmdBindIndexBuffer);
	VK_GetDeviceProcAddr(dev, vkCmdBindVertexBuffers);
	VK_GetDeviceProcAddr(dev, vkCmdDraw);
	VK_GetDeviceProcAddr(dev, vkCmdDrawIndexed);
	VK_GetDeviceProcAddr(dev, vkCmdDrawIndirect);
	VK_GetDeviceProcAddr(dev, vkCmdDrawIndexedIndirect);
	VK_GetDeviceProcAddr(dev, vkCmdDispatch);
	VK_GetDeviceProcAddr(dev, vkCmdDispatchIndirect);
	VK_GetDeviceProcAddr(dev, vkCmdCopyBuffer);
	VK_GetDeviceProcAddr(dev, vkCmdCopyImage);
	VK_GetDeviceProcAddr(dev, vkCmdBlitImage);
	VK_GetDeviceProcAddr(dev, vkCmdCopyBufferToImage);
	VK_GetDeviceProcAddr(dev, vkCmdCopyImageToBuffer);
	VK_GetDeviceProcAddr(dev, vkCmdUpdateBuffer);
	VK_GetDeviceProcAddr(dev, vkCmdFillBuffer);
	VK_GetDeviceProcAddr(dev, vkCmdClearColorImage);
	VK_GetDeviceProcAddr(dev, vkCmdClearDepthStencilImage);
	VK_GetDeviceProcAddr(dev, vkCmdClearAttachments);
	VK_GetDeviceProcAddr(dev, vkCmdResolveImage);
	VK_GetDeviceProcAddr(dev, vkCmdSetEvent);
	VK_GetDeviceProcAddr(dev, vkCmdResetEvent);
	VK_GetDeviceProcAddr(dev, vkCmdWaitEvents);
	VK_GetDeviceProcAddr(dev, vkCmdPipelineBarrier);
	VK_GetDeviceProcAddr(dev, vkCmdBeginQuery);
	VK_GetDeviceProcAddr(dev, vkCmdEndQuery);
	VK_GetDeviceProcAddr(dev, vkCmdResetQueryPool);
	VK_GetDeviceProcAddr(dev, vkCmdWriteTimestamp);
	VK_GetDeviceProcAddr(dev, vkCmdCopyQueryPoolResults);
	VK_GetDeviceProcAddr(dev, vkCmdPushConstants);
	VK_GetDeviceProcAddr(dev, vkCmdBeginRenderPass);
	VK_GetDeviceProcAddr(dev, vkCmdNextSubpass);
	VK_GetDeviceProcAddr(dev, vkCmdEndRenderPass);
	VK_GetDeviceProcAddr(dev, vkCmdExecuteCommands);

	// KHR_Swapchain
	if (m_options.OptionGet(cOptUseSurface))
	{
		VK_GetDeviceProcAddr(dev, vkCreateSwapchainKHR);
		VK_GetDeviceProcAddr(dev, vkDestroySwapchainKHR);
		VK_GetDeviceProcAddr(dev, vkGetSwapchainImagesKHR);
		VK_GetDeviceProcAddr(dev, vkAcquireNextImageKHR);
		VK_GetDeviceProcAddr(dev, vkQueuePresentKHR);

		// FIXME: This is Vulkan 1.1
		//VK_GetDeviceProcAddr(dev, vkAcquireNextImage2KHR);
		//VK_GetDeviceProcAddr(dev, vkGetDeviceGroupPresentCapabilitiesKHR);
		//VK_GetDeviceProcAddr(dev, vkGetPhysicalDevicePresentRectanglesKHR);
	}

	// KHR_push_descriptor
	VK_GetDeviceProcAddr(dev, vkCmdPushDescriptorSetKHR);

	// EXT_host_query_reset (is required by default)
	{
		VK_GetDeviceProcAddr(dev, vkResetQueryPoolEXT);
	}

	return true;
}


} // epic::nls

#if defined(_MSC_VER)
__pragma(warning(pop))
#endif
