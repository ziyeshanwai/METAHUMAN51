// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "VulkanFunctions.h"

#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

namespace epic {
namespace nls {

class VulkanInstance
{
public:
    VulkanInstance(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger);

    ~VulkanInstance();

    /**
     * Creates a vulkan instance and optionally enables validation layers as well as loads the required extensions
     */
    static std::shared_ptr<VulkanInstance> CreateInstance(bool enableValidationLayers, const std::vector<const char*>& requiredExtensions);

    VkInstance Instance() const { return m_instance; }

private:

    //! checks whether the validation layers are supported
    static bool checkValidationLayerSupport(const std::vector<const char*>& validationLayers);

    static void PopulateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo);

    static VkResult CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger);

    static void DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator);

    static VKAPI_ATTR VkBool32 VKAPI_CALL DebugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                                                        VkDebugUtilsMessageTypeFlagsEXT messageType,
                                                        const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
                                                        void* pUserData);

private:
    VkInstance m_instance = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT m_debugMessenger = VK_NULL_HANDLE;
};

} // namespace nls
} //namespace epic
