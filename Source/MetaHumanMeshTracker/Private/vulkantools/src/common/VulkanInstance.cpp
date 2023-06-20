// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/common/VulkanInstance.h>
#include <carbon/common/External.h>
#include<pma/PolyAllocator.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4191)
#endif

using namespace pma;

namespace epic {
namespace nls {

VulkanInstance::~VulkanInstance() {
    if (m_debugMessenger) {
        DestroyDebugUtilsMessengerEXT(m_instance, m_debugMessenger, nullptr);
        m_debugMessenger = VK_NULL_HANDLE;
    }
    if (m_instance) {
        vkDestroyInstance(m_instance, nullptr);
        m_instance = VK_NULL_HANDLE;
    }
}

std::shared_ptr<VulkanInstance> VulkanInstance::CreateInstance(bool enableValidationLayers, const std::vector<const char*>& requiredExtensions) {
    VkInstance instance = VK_NULL_HANDLE;

    const std::vector<const char*> validationLayers = {"VK_LAYER_KHRONOS_validation"};
    if (enableValidationLayers && !checkValidationLayerSupport(validationLayers)) {
        throw std::runtime_error("validation layers requested, but not available!");
    }

    VkInstanceCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.pApplicationInfo = nullptr;

    // add extensions
    std::vector<const char*> extRequiredExtensions = requiredExtensions;

    extRequiredExtensions.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);

    if (enableValidationLayers) {
        extRequiredExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }
    extRequiredExtensions.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
    createInfo.enabledExtensionCount = static_cast<uint32_t>(extRequiredExtensions.size());
    createInfo.ppEnabledExtensionNames = extRequiredExtensions.data();

    // enable validation layers
    VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo;
    if (enableValidationLayers) {
        createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
        createInfo.ppEnabledLayerNames = validationLayers.data();

        PopulateDebugMessengerCreateInfo(debugCreateInfo);
        createInfo.pNext = (VkDebugUtilsMessengerCreateInfoEXT*) &debugCreateInfo;
    } else {
        createInfo.enabledLayerCount = 0;
    }

    if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
        throw std::runtime_error("failed to create VkInstance");
    }

    if (!VulkanLoader::Instance()->InitFunc(instance)) {
        throw std::runtime_error("VulkanLoader failed to load instance functions");
    }

    VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
    if (enableValidationLayers) {
        VkDebugUtilsMessengerCreateInfoEXT debugMsgCreateInfo = {};
        PopulateDebugMessengerCreateInfo(debugMsgCreateInfo);

        if (CreateDebugUtilsMessengerEXT(instance, &debugMsgCreateInfo, nullptr, &debugMessenger) != VK_SUCCESS) {
            throw std::runtime_error("Failed to set up debug messenger. Make sure to set VulkanLoader::cOptUseDebugUtils in VulkanLoader::Init()");
        }
    }

    PolyAllocator<VulkanInstance> polyAllocator{ MEM_RESOURCE };
    return std::allocate_shared<VulkanInstance>(polyAllocator, instance, debugMessenger);
}

VulkanInstance::VulkanInstance(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger)
    : m_instance(instance)
    , m_debugMessenger(debugMessenger)
    {}

bool VulkanInstance::checkValidationLayerSupport(const std::vector<const char*>& validationLayers) {

    uint32_t layerCount;
    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

    std::vector<VkLayerProperties> availableLayers(layerCount);
    vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

    for (const char* layerName : validationLayers) {
        bool layerFound = false;

        for (const auto& layerProperties : availableLayers) {
            if (strcmp(layerName, layerProperties.layerName) == 0) {
                layerFound = true;
                break;
            }
        }

        if (!layerFound) {
            printf("Did not find vulkan layer \"%s\"\n", layerName);
            return false;
        }
    }

    return true;
}

void VulkanInstance::PopulateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo) {
    createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    //createInfo.messageSeverity |= VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT; // enable this to get more info from Vulkan
    createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    createInfo.pfnUserCallback = DebugCallback;
}

VkResult VulkanInstance::CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger) {
    if (vkCreateDebugUtilsMessengerEXT != nullptr) {
        return vkCreateDebugUtilsMessengerEXT(instance, pCreateInfo, pAllocator, pDebugMessenger);
    } else {
        return VK_ERROR_EXTENSION_NOT_PRESENT;
    }
}

void VulkanInstance::DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator) {
    if (vkDestroyDebugUtilsMessengerEXT != nullptr) {
        vkDestroyDebugUtilsMessengerEXT(instance, debugMessenger, pAllocator);
    }
}

VKAPI_ATTR VkBool32 VKAPI_CALL VulkanInstance::DebugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT /*messageSeverity*/,
                                                                    VkDebugUtilsMessageTypeFlagsEXT /*messageType*/,
                                                                    const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
                                                                    void* /*pUserData*/) {
    fprintf(stderr, "validation layer: %s\n", pCallbackData->pMessage);
    return VK_FALSE;
}

} // namespace nls
} //namespace epic

#ifdef _MSC_VER
#pragma warning(pop)
#endif
