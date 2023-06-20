// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanInstance.h>

// We need VK_KHR_portability_subset for macOS
#if defined(__APPLE__)
#include <vulkan/vulkan_beta.h>
#endif

#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <pma/PolyAllocator.h>
#include <carbon/common/External.h>

namespace epic {
namespace nls {


class VulkanPresentationSurface {
public:
    virtual bool IsDeviceSupported(VkPhysicalDevice device) const = 0;

    virtual bool IsPresentationSupported(VkPhysicalDevice physicalDevice, int queueFamilyIndex) const = 0;

    virtual void AppendRequiredDeviceExtensions(std::vector<const char*>& extensions) const = 0;
};


class VulkanDevice {
    struct QueueFamilyIndices {
        std::optional<uint32_t> allFamily;
        std::optional<uint32_t> graphicsFamily;
        std::optional<uint32_t> computeFamily;
        std::optional<uint32_t> transferFamily;
        std::optional<uint32_t> presentFamily;
    };

public:
    VulkanDevice() = default;

    ~VulkanDevice()
    {
        if (m_device) {
            vkDestroyDevice(m_device, nullptr);
            m_device = VK_NULL_HANDLE;
        }
    }

    static std::shared_ptr<VulkanDevice> CreateDevice(std::shared_ptr<VulkanInstance> vulkanInstance, VulkanPresentationSurface* presentationSurface)
    {
        pma::PolyAllocator<VulkanDevice> polyAlloc{ MEM_RESOURCE };
        std::shared_ptr<VulkanDevice> vulkanDevice = std::allocate_shared<VulkanDevice>(polyAlloc);
        vulkanDevice->m_vulkanInstance = vulkanInstance;
        vulkanDevice->pickPhysicalDevice(presentationSurface);
        vulkanDevice->createLogicalDevice(presentationSurface);
        return vulkanDevice;
    }

    VkPhysicalDevice PhysicalDevice() const { return m_physicalDevice; }
    VkDevice Device() const { return m_device; }

    std::optional<uint32_t> AllFamily() const { return m_queueFamilyIndices.allFamily; }
    // std::optional<uint32_t> GraphicsFamily() const { return m_queueFamilyIndices.graphicsFamily; }
    // std::optional<uint32_t> ComputeFamily() const { return m_queueFamilyIndices.computeFamily; }
    // std::optional<uint32_t> TransferFamily() const { return m_queueFamilyIndices.transferFamily; }
    // std::optional<uint32_t> PresentFamily() const { return m_queueFamilyIndices.presentFamily; }

    VkQueue AllQueue() const { return m_allQueue; }
    // VkQueue GraphicsQueue() const { return m_graphicsQueue; }
    // VkQueue ComputeQueue() const { return m_computeQueue; }
    // VkQueue TransferQueue() const { return m_transferQueue; }
    // VkQueue PresentQueue() const { return m_presentQueue; }

    VkPhysicalDeviceProperties Properties() const { return m_properties; }

    std::mutex& Mutex() { return m_mutex; }

private:
    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device, VulkanPresentationSurface* presentationSurface) {
        QueueFamilyIndices indices;

        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

        for (uint32_t i = 0; i < queueFamilies.size(); i++)
        {
            // family that supports all transfers
            if (!indices.allFamily.has_value() &&
                queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT &&
                queueFamilies[i].queueFlags & VK_QUEUE_COMPUTE_BIT &&
                queueFamilies[i].queueFlags & VK_QUEUE_TRANSFER_BIT &&
                (!presentationSurface || presentationSurface->IsPresentationSupported(device, i))) {
                indices.allFamily = i;
            }
            if (!indices.graphicsFamily.has_value() && queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                indices.graphicsFamily = i;
            }
            if (!indices.computeFamily.has_value() && queueFamilies[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
                indices.computeFamily = i;
            }
            if (!indices.transferFamily.has_value() && queueFamilies[i].queueFlags & VK_QUEUE_TRANSFER_BIT) {
                indices.transferFamily = i;
            }
            if (!indices.presentFamily.has_value() && presentationSurface && presentationSurface->IsPresentationSupported(device, i)) {
                indices.presentFamily = i;
            }
        }

        return indices;
    }

    int rateDeviceSuitability(VkPhysicalDevice device, VulkanPresentationSurface* presentationSurface) {
        VkPhysicalDeviceProperties deviceProperties;
        vkGetPhysicalDeviceProperties(device, &deviceProperties);
        VkPhysicalDeviceFeatures deviceFeatures;
        vkGetPhysicalDeviceFeatures(device, &deviceFeatures);

        QueueFamilyIndices queueFamilyIndices = findQueueFamilies(device, presentationSurface);

        int score = 0;

        // Discrete GPUs have a significant performance advantage
        if (deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            score += 1000;
        }

        // Maximum possible size of textures affects graphics quality
        score += deviceProperties.limits.maxImageDimension2D;

        if (!queueFamilyIndices.allFamily.has_value()) {
            return 0;
        }

        // check if the device has presentation support (if required)
        if (presentationSurface && !presentationSurface->IsDeviceSupported(device)) {
            return 0;
        }


        //Application can't function without anisotropy filtering
        if (!deviceFeatures.samplerAnisotropy) {
            return 0;
        }

        return score;
    }

    void pickPhysicalDevice(VulkanPresentationSurface* presentationSurface) {

        uint32_t deviceCount = 0;
        vkEnumeratePhysicalDevices(m_vulkanInstance->Instance(), &deviceCount, nullptr);
        if (deviceCount == 0) {
            throw std::runtime_error("failed to find GPUs with Vulkan support!");
        }
        std::vector<VkPhysicalDevice> devices(deviceCount);
        vkEnumeratePhysicalDevices(m_vulkanInstance->Instance(), &deviceCount, devices.data());

        // Use an ordered map to automatically sort candidates by increasing score
        std::multimap<int, VkPhysicalDevice> candidates;
        for (const auto& device : devices) {
            candidates.insert(std::make_pair(rateDeviceSuitability(device, presentationSurface), device));
        }

        // Check if the best candidate is suitable at all
        if (candidates.rbegin()->first > 0) {
            m_physicalDevice = candidates.rbegin()->second;
        } else {
            throw std::runtime_error("failed to find a suitable GPU!");
        }
    }

    void createLogicalDevice(VulkanPresentationSurface* presentationSurface) {
        QueueFamilyIndices indices = findQueueFamilies(m_physicalDevice, presentationSurface);
        std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
        std::set<std::optional<uint32_t>> uniqueQueueFamilies = {indices.allFamily,
                                                                 indices.graphicsFamily,
                                                                 indices.computeFamily,
                                                                 indices.transferFamily,
                                                                 indices.presentFamily};
        float queuePriority = 1.0f;
        for (std::optional<uint32_t> queueFamily : uniqueQueueFamilies) {
            if (queueFamily.has_value()) {
                VkDeviceQueueCreateInfo queueCreateInfo = {};
                queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
                queueCreateInfo.queueFamilyIndex = queueFamily.value();
                queueCreateInfo.queueCount = 1;
                queueCreateInfo.pQueuePriorities = &queuePriority;
                queueCreateInfos.push_back(queueCreateInfo);
            }
        }

        VkPhysicalDeviceFeatures2KHR deviceFeatures2 = {};
        deviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2_KHR;

        VkPhysicalDeviceFeatures& deviceFeatures = deviceFeatures2.features;
        deviceFeatures.samplerAnisotropy = VK_TRUE;
        deviceFeatures.fillModeNonSolid = VK_TRUE;
        deviceFeatures.depthBiasClamp = VK_TRUE;

        VkPhysicalDeviceHostQueryResetFeaturesEXT hostQueryFeature = {};
        hostQueryFeature.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_HOST_QUERY_RESET_FEATURES_EXT;
        hostQueryFeature.hostQueryReset = true;

        deviceFeatures2.pNext = &hostQueryFeature;

#if defined(__APPLE__)
        VkPhysicalDevicePortabilitySubsetFeaturesKHR portabilityFeatures = {};
        portabilityFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PORTABILITY_SUBSET_FEATURES_KHR;
        portabilityFeatures.imageViewFormatSwizzle = VK_TRUE;
        hostQueryFeature.pNext = &portabilityFeatures;
#endif

        VkDeviceCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        createInfo.pNext = &deviceFeatures2;
        createInfo.pQueueCreateInfos = queueCreateInfos.data();
        createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());

        std::vector<const char*> deviceExtensions;
        if (presentationSurface) {
            presentationSurface->AppendRequiredDeviceExtensions(deviceExtensions);
        }

        // We must enable device extension VK_KHR_portability_subset on macOS
#if defined(__APPLE__)
        deviceExtensions.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
#endif

        deviceExtensions.push_back(VK_KHR_PUSH_DESCRIPTOR_EXTENSION_NAME);

        deviceExtensions.push_back(VK_EXT_HOST_QUERY_RESET_EXTENSION_NAME);
        createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
        createInfo.ppEnabledExtensionNames = deviceExtensions.data();

        if (vkCreateDevice(m_physicalDevice, &createInfo, nullptr, &m_device) != VK_SUCCESS) {
            throw std::runtime_error("failed to create logical device!");
        }

        if (!VulkanLoader::Instance()->InitFunc(m_device))
            throw std::runtime_error("VulkanLoader failed to load Vulkan device functions");

        m_queueFamilyIndices = indices;
        if (indices.allFamily.has_value()) {
            vkGetDeviceQueue(m_device, indices.allFamily.value(), 0, &m_allQueue);
        }
        if (indices.graphicsFamily.has_value()) {
            vkGetDeviceQueue(m_device, indices.graphicsFamily.value(), 0, &m_graphicsQueue);
        }
        if (indices.computeFamily.has_value()) {
            vkGetDeviceQueue(m_device, indices.computeFamily.value(), 0, &m_computeQueue);
        }
        if (indices.transferFamily.has_value()) {
            vkGetDeviceQueue(m_device, indices.transferFamily.value(), 0, &m_transferQueue);
        }
        if (indices.presentFamily.has_value()) {
            vkGetDeviceQueue(m_device, indices.presentFamily.value(), 0, &m_presentQueue);
        }

        vkGetPhysicalDeviceProperties(m_physicalDevice, &m_properties);
    }


private:
    std::shared_ptr<VulkanInstance> m_vulkanInstance;

    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkDevice m_device = VK_NULL_HANDLE;

    QueueFamilyIndices m_queueFamilyIndices;
    VkQueue m_allQueue = VK_NULL_HANDLE;
    VkQueue m_graphicsQueue = VK_NULL_HANDLE;
    VkQueue m_computeQueue = VK_NULL_HANDLE;
    VkQueue m_transferQueue = VK_NULL_HANDLE;
    VkQueue m_presentQueue = VK_NULL_HANDLE;

    VkPhysicalDeviceProperties m_properties;

    std::mutex m_mutex;
};

} // namespace nls
} //namespace epic
