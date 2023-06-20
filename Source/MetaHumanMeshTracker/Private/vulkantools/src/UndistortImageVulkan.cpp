// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/UndistortImageVulkan.h>

#include <nls/geometry/PixelFormat.h>

#include <nls/vulkan/shaders/embed_undistort.comp.spv.h>

#include <array>

namespace epic {
namespace nls {

UndistortImageVulkan::UndistortImageVulkan(std::shared_ptr<VulkanDevice> vulkanDevice)
    : m_vulkanDevice(vulkanDevice)
{
    m_vulkanMemory = VulkanMemory::Create(m_vulkanDevice);
    m_transientCommandPool = VulkanCommandPool::Create(m_vulkanDevice, /*transient=*/true);//, /*compute=*/false);
}

UndistortImageVulkan::~UndistortImageVulkan()
{
    Reset();
}

void UndistortImageVulkan::Undistort(const float* distortedImage, float* undistortedImage, const MetaShapeCamera<float>& camera)
{
    const int width = camera.Width();
    const int height = camera.Height();

    auto t1 = std::chrono::high_resolution_clock::now();

    if (width != m_width || height != m_height) {
        Reset();

        m_width = width;
        m_height = height;
        m_camera = camera;

        m_distortedImage = m_vulkanMemory->createImageAndView(m_width, m_height, VK_FORMAT_R32G32B32A32_SFLOAT, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED);
        m_undistortedImage = m_vulkanMemory->createImageAndView(m_width, m_height, VK_FORMAT_R32G32B32A32_SFLOAT, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED);
        m_undistortedBuffer = m_vulkanMemory->createBuffer(m_width * m_height * 4 * sizeof(float), VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        m_uniformBuffer = m_vulkanMemory->createBuffer(sizeof(ShaderData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        m_texSampler = VulkanTextureSampler::CreateBasicSampler(m_vulkanDevice, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER);

        CreateDescriptorSetLayout();
        CreateComputePipeline();
        CreateDescriptorPool();
        CreateDescriptorSets();
        CreateCommandBuffers();
        CreateSyncObjects();
    }

    auto t2 = std::chrono::high_resolution_clock::now();

    // copy distorted image
    VkDeviceSize imageSize = width * height * 4 * sizeof(float);
    std::unique_ptr<VulkanManagedBuffer> stagingBuffer = m_vulkanMemory->createBuffer(imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    void* data;
    vkMapMemory(stagingBuffer->Device(), stagingBuffer->DeviceMemory(), 0, imageSize, 0, &data);
    Convert(distortedImage, (float*)data, width, height, PixelFormat::RGBA, PixelFormat::RGBA);
    vkUnmapMemory(stagingBuffer->Device(), stagingBuffer->DeviceMemory());

    auto t3 = std::chrono::high_resolution_clock::now();

    VkCommandBuffer commandBuffer = m_transientCommandPool->BeginSingleTimeCommands();
    m_vulkanMemory->transitionImageLayout(commandBuffer, m_distortedImage->Image(), VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, m_distortedImage->Aspect());
    m_vulkanMemory->copyBufferToImage(commandBuffer, stagingBuffer->Buffer(), m_distortedImage->Image(), static_cast<uint32_t>(m_width), static_cast<uint32_t>(m_height));
    m_vulkanMemory->transitionImageLayout(commandBuffer, m_distortedImage->Image(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, m_distortedImage->Aspect());
    m_transientCommandPool->EndSingleTimeCommands(commandBuffer);

    vkWaitForFences(m_vulkanDevice->Device(), 1, &m_fence, VK_TRUE, UINT64_MAX);

    auto t4 = std::chrono::high_resolution_clock::now();

    // update with undistortion information
    ShaderData ubo;
    ubo.width = m_width;
    ubo.height = m_height;
    ubo.fx = m_camera.Intrinsics()(0,0) / float(m_camera.Width());
    ubo.fy = m_camera.Intrinsics()(1,1) / float(m_camera.Height());
    ubo.cx = m_camera.Intrinsics()(0,2) / float(m_camera.Width());
    ubo.cy = m_camera.Intrinsics()(1,2) / float(m_camera.Height());
    ubo.B1 = m_camera.Skew()[0];
    ubo.B2 = m_camera.Skew()[1];
    ubo.radialDistortion = m_camera.RadialDistortion();
    ubo.tangentialDistortion = m_camera.TangentialDistortion();

    vkMapMemory(m_vulkanDevice->Device(), m_uniformBuffer->DeviceMemory(), 0, sizeof(ubo), 0, &data);
    memcpy(data, &ubo, sizeof(ubo));
    vkUnmapMemory(m_vulkanDevice->Device(), m_uniformBuffer->DeviceMemory());


    VkSubmitInfo submitInfo = {};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffer;

    auto t5 = std::chrono::high_resolution_clock::now();

    vkResetFences(m_vulkanDevice->Device(), 1, &m_fence);

    auto t6 = std::chrono::high_resolution_clock::now();

    if (vkQueueSubmit(m_vulkanDevice->AllQueue(), 1, &submitInfo, m_fence) != VK_SUCCESS) {
        throw std::runtime_error("failed to submit draw command buffer!");
    }

    vkWaitForFences(m_vulkanDevice->Device(), 1, &m_fence, VK_TRUE, UINT64_MAX);

    auto t7 = std::chrono::high_resolution_clock::now();

    auto t8 = std::chrono::high_resolution_clock::now();

    vkMapMemory(m_vulkanDevice->Device(), m_undistortedBuffer->DeviceMemory(),  m_undistortedBuffer->Offset(), m_undistortedBuffer->Size(), 0, &data);
    Convert((float*)data, undistortedImage, width, height, PixelFormat::RGBA, PixelFormat::RGBA);
    vkUnmapMemory(m_vulkanDevice->Device(), m_undistortedBuffer->DeviceMemory());

    auto t9 = std::chrono::high_resolution_clock::now();

    double t12 = std::chrono::duration<double, std::chrono::seconds::period>(t2 - t1).count() * 1000;
    double t23 = std::chrono::duration<double, std::chrono::seconds::period>(t3 - t2).count() * 1000;
    double t34 = std::chrono::duration<double, std::chrono::seconds::period>(t4 - t3).count() * 1000;
    double t45 = std::chrono::duration<double, std::chrono::seconds::period>(t5 - t4).count() * 1000;
    double t56 = std::chrono::duration<double, std::chrono::seconds::period>(t6 - t5).count() * 1000;
    double t67 = std::chrono::duration<double, std::chrono::seconds::period>(t7 - t6).count() * 1000;
    double t78 = std::chrono::duration<double, std::chrono::seconds::period>(t8 - t7).count() * 1000;
    double t89 = std::chrono::duration<double, std::chrono::seconds::period>(t9 - t8).count() * 1000;
    printf("undistortion time: %f %f %f %f %f %f %f %fms\n", t12, t23, t34, t45, t56, t67, t78, t89);
}

void UndistortImageVulkan::Reset()
{
    if (m_descriptorPool) {
        vkDestroyDescriptorPool(m_vulkanDevice->Device(), m_descriptorPool, nullptr);
        m_descriptorPool = VK_NULL_HANDLE;
    }
    if (m_fence) {
        vkDestroyFence(m_vulkanDevice->Device(), m_fence, nullptr);
        m_fence = VK_NULL_HANDLE;
    }
    if (m_computePipeline) {
        vkDestroyPipeline(m_vulkanDevice->Device(), m_computePipeline, nullptr);
        m_computePipeline = VK_NULL_HANDLE;
    }
    if (m_pipelineLayout) {
        vkDestroyPipelineLayout(m_vulkanDevice->Device(), m_pipelineLayout, nullptr);
        m_pipelineLayout = VK_NULL_HANDLE;
    }
    if (m_descriptorSetLayout) {
        vkDestroyDescriptorSetLayout(m_vulkanDevice->Device(), m_descriptorSetLayout, nullptr);
        m_descriptorSetLayout = VK_NULL_HANDLE;
    }
}


void UndistortImageVulkan::CreateDescriptorSetLayout()
{
    std::vector<VkDescriptorSetLayoutBinding> bindings;
    {
        // distoration parameters
        VkDescriptorSetLayoutBinding uboLayoutBinding = {};
        uboLayoutBinding.binding = 0;
        uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        uboLayoutBinding.descriptorCount = 1;
        uboLayoutBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        uboLayoutBinding.pImmutableSamplers = nullptr;
        bindings.push_back(uboLayoutBinding);
    }

    {
        // distorted image texture
        VkDescriptorSetLayoutBinding samplerLayoutBinding = {};
        samplerLayoutBinding.binding = 1;
        samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        samplerLayoutBinding.descriptorCount = 1;
        samplerLayoutBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        samplerLayoutBinding.pImmutableSamplers = nullptr;
        bindings.push_back(samplerLayoutBinding);
    }

    {
        // undistorted image
        VkDescriptorSetLayoutBinding outputLayoutBinding = {};
        outputLayoutBinding.binding = 2;
        outputLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        outputLayoutBinding.descriptorCount = 1;
        outputLayoutBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        outputLayoutBinding.pImmutableSamplers = nullptr;
        bindings.push_back(outputLayoutBinding);
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo = {};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    if (vkCreateDescriptorSetLayout(m_vulkanDevice->Device(), &layoutInfo, nullptr, &m_descriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor set layout!");
    }
}


void UndistortImageVulkan::CreateDescriptorPool()
{
    std::array<VkDescriptorPoolSize, 3> poolSizes = {};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = 1;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[1].descriptorCount =  1;
    poolSizes[2].type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    poolSizes[2].descriptorCount =  1;

    VkDescriptorPoolCreateInfo poolInfo = {};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    poolInfo.maxSets = 1;

    if (vkCreateDescriptorPool(m_vulkanDevice->Device(), &poolInfo, nullptr, &m_descriptorPool) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor pool!");
    }
}


void UndistortImageVulkan::CreateDescriptorSets()
{
    VkDescriptorSetAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = m_descriptorPool;
    allocInfo.pSetLayouts = &m_descriptorSetLayout;
    allocInfo.descriptorSetCount = 1;

    if (vkAllocateDescriptorSets(m_vulkanDevice->Device(), &allocInfo, &m_descriptorSet) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate descriptor sets!");
    }

    std::vector<VkWriteDescriptorSet> descriptorWrites;
    VkWriteDescriptorSet descriptorWrite = {};

    VkDescriptorBufferInfo bufferInfo = {};
    bufferInfo.buffer = m_uniformBuffer->Buffer();
    bufferInfo.offset = m_uniformBuffer->Offset();
    bufferInfo.range = m_uniformBuffer->Size();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_descriptorSet;
    descriptorWrite.dstBinding = 0;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfo;
    descriptorWrites.push_back(descriptorWrite);


    VkDescriptorImageInfo imageInfo = {};
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfo.imageView = m_distortedImage->ImageView();
    imageInfo.sampler = m_texSampler->Sampler();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_descriptorSet;
    descriptorWrite.dstBinding = 1;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pImageInfo = &imageInfo;
    descriptorWrites.push_back(descriptorWrite);


    VkDescriptorImageInfo imageOutInfo = {};
    imageOutInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    imageOutInfo.imageView = m_undistortedImage->ImageView();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_descriptorSet;
    descriptorWrite.dstBinding = 2;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pImageInfo = &imageOutInfo;
    descriptorWrites.push_back(descriptorWrite);

    vkUpdateDescriptorSets(m_vulkanDevice->Device(), static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
}


void UndistortImageVulkan::CreateComputePipeline()
{
    std::unique_ptr<VulkanShaderModule> compShaderModule = VulkanShaderModule::Create(m_vulkanDevice, SPV_undistort_comp_spv, sizeof(SPV_undistort_comp_spv));

    VkPipelineShaderStageCreateInfo compShaderStageInfo = {};
    compShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    compShaderStageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    compShaderStageInfo.module = compShaderModule->ShaderModule();
    compShaderStageInfo.pName = "main";

    //VkPipelineShaderStageCreateInfo shaderStages[] = {compShaderStageInfo};

    VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1; // Optional
    pipelineLayoutInfo.pSetLayouts = &m_descriptorSetLayout; // Optional
    pipelineLayoutInfo.pushConstantRangeCount = 0; // Optional
    pipelineLayoutInfo.pPushConstantRanges = nullptr; // Optional

    if (vkCreatePipelineLayout(m_vulkanDevice->Device(), &pipelineLayoutInfo, nullptr, &m_pipelineLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create pipeline layout!");
    }

    VkComputePipelineCreateInfo pipelineCreateInfo = {};
    pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineCreateInfo.stage = compShaderStageInfo;
    pipelineCreateInfo.layout = m_pipelineLayout;

    /*
    Now, we finally create the compute pipeline.
    */
    if (vkCreateComputePipelines(m_vulkanDevice->Device(), VK_NULL_HANDLE, 1, &pipelineCreateInfo, NULL, &m_computePipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create compute pipeline!");
    }

}

void UndistortImageVulkan::CreateCommandBuffers()
{
    VkCommandBufferAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = m_transientCommandPool->CommandPool();
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1;

    if (vkAllocateCommandBuffers(m_vulkanDevice->Device(), &allocInfo, &m_commandBuffer) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate command buffers!");
    }

    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = 0; // Optional
    beginInfo.pInheritanceInfo = nullptr; // Optional

    if (vkBeginCommandBuffer(m_commandBuffer, &beginInfo) != VK_SUCCESS) {
        throw std::runtime_error("failed to begin recording command buffer!");
    }

    m_vulkanMemory->transitionImageLayout(m_commandBuffer, m_undistortedImage->Image(), VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL, m_undistortedImage->Aspect());

    vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_computePipeline);
    vkCmdBindDescriptorSets(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);
    const int WORKGROUP_SIZE = 32;
    vkCmdDispatch(m_commandBuffer, (uint32_t)ceil(m_width / float(WORKGROUP_SIZE)), (uint32_t)ceil(m_height / float(WORKGROUP_SIZE)), 1);

    m_vulkanMemory->transitionImageLayout(m_commandBuffer, m_undistortedImage->Image(), VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, m_undistortedImage->Aspect());

    m_vulkanMemory->copyImageToBuffer(m_commandBuffer, m_undistortedImage->Image(), m_undistortedBuffer->Buffer(), m_undistortedImage->Width(), m_undistortedImage->Height());

    if (vkEndCommandBuffer(m_commandBuffer) != VK_SUCCESS) {
        throw std::runtime_error("failed to record command buffer!");
    }
}

void UndistortImageVulkan::CreateSyncObjects()
{
    VkFenceCreateInfo fenceInfo = {};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    if (vkCreateFence(m_vulkanDevice->Device(), &fenceInfo, nullptr, &m_fence) != VK_SUCCESS) {
        throw std::runtime_error("failed to create semaphores for a frame!");
    }
}

} // namespace nls
} //namespace epic
