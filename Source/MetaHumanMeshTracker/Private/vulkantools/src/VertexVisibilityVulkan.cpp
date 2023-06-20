// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/VertexVisibilityVulkan.h>

#include <nls/vulkan/shaders/embed_vertexvisibility.comp.spv.h>

namespace epic::nls {

struct VertexVisibilityVulkan::UniformBufferObject {
    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> textureModelViewProjectionMatrix;
    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> textureModelViewMatrix;
    Eigen::Matrix<float, 2, 2, Eigen::DontAlign> textureProjectionMatrixInverse;
    int numVertices;
};

VertexVisibilityVulkan::VertexVisibilityVulkan(std::shared_ptr<VulkanDevice> vulkanDevice)
    : m_vulkanDevice(vulkanDevice)
{
    m_vulkanMemory = VulkanMemory::Create(m_vulkanDevice);
    m_transientCommandPool = VulkanCommandPool::Create(m_vulkanDevice, /*transient=*/true);//, /*compute=*/false);
}


VertexVisibilityVulkan::~VertexVisibilityVulkan()
{
    Reset();
}


void VertexVisibilityVulkan::Reset()
{
    if (m_computePipeline) {
        vkDestroyPipeline(m_vulkanDevice->Device(), m_computePipeline, nullptr);
        m_computePipeline = VK_NULL_HANDLE;
    }
    if (m_pipelineLayout) {
        vkDestroyPipelineLayout(m_vulkanDevice->Device(), m_pipelineLayout, nullptr);
        m_pipelineLayout = VK_NULL_HANDLE;
    }
}


void VertexVisibilityVulkan::Setup()
{
    CreateDescriptorSetLayout();
    CreateComputePipeline();
    CreateDescriptorSets();
}


Eigen::VectorXf VertexVisibilityVulkan::Compute(const Eigen::Matrix4f& textureProjectionMatrix,
                                                   const Eigen::Matrix4f& textureModelViewMatrix,
                                                   const VulkanMeshBuffer& meshBuffer,
                                                   const VulkanAllInOneTexture& depthTexture,
                                                   const VulkanAllInOneTexture& depthAlphaTexture)
{
    if (!m_computePipeline) {
        Setup();
    }

    const int numVertices = static_cast<int>(meshBuffer.m_vertexBuffer->Size() / 3 / sizeof(float));
    if (!m_visibilityBufferDevice || m_visibilityBufferDevice->Size() != numVertices * sizeof(float)) {
        m_visibilityBufferDevice = m_vulkanMemory->createBuffer(numVertices * sizeof(float), VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        m_visibilityBufferHost = m_vulkanMemory->createBuffer(numVertices * sizeof(float), VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }
    UpdateDescriptorSets(meshBuffer, m_visibilityBufferDevice);

    VkCommandBuffer commandBuffer = m_transientCommandPool->BeginSingleTimeCommands();

    UniformBufferObject ubo;
    ubo.textureModelViewProjectionMatrix = textureProjectionMatrix * textureModelViewMatrix;
    ubo.textureModelViewMatrix = textureModelViewMatrix;
    ubo.textureProjectionMatrixInverse = textureProjectionMatrix.cast<double>().inverse().cast<float>().block(2,2,2,2);
    ubo.numVertices = numVertices;

    vkCmdPushConstants(commandBuffer, m_pipelineLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(ubo), &ubo);

    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_computePipeline);
    std::vector<VkDescriptorSet> descriptorSets = {m_descriptorSet->DescriptorSet(), depthTexture.DescriptorSet(), depthAlphaTexture.DescriptorSet()};
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineLayout, 0, 3, descriptorSets.data(), 0, nullptr);
    const int WORKGROUP_SIZE = 32;
    vkCmdDispatch(commandBuffer, (uint32_t)ceil(numVertices / float(WORKGROUP_SIZE)), 1, 1);

    m_vulkanMemory->copyBuffer(commandBuffer, m_visibilityBufferDevice->Buffer(), m_visibilityBufferHost->Buffer(), m_visibilityBufferDevice->Size());

    m_transientCommandPool->EndSingleTimeCommands(commandBuffer);

    Eigen::VectorXf visibility(numVertices);
    void* data;
    vkMapMemory(m_visibilityBufferHost->Device(), m_visibilityBufferHost->DeviceMemory(), 0, m_visibilityBufferHost->Size(), 0, &data);
    memcpy(visibility.data(), data, m_visibilityBufferHost->Size());
    vkUnmapMemory(m_visibilityBufferHost->Device(), m_visibilityBufferHost->DeviceMemory());

    return visibility;
}

void VertexVisibilityVulkan::CreateComputePipeline()
{
    std::unique_ptr<VulkanShaderModule> compShaderModule = VulkanShaderModule::Create(m_vulkanDevice, SPV_vertexvisibility_comp_spv, sizeof(SPV_vertexvisibility_comp_spv));

    VkPipelineShaderStageCreateInfo compShaderStageInfo = {};
    compShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    compShaderStageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    compShaderStageInfo.module = compShaderModule->ShaderModule();
    compShaderStageInfo.pName = "main";

    // VkPipelineShaderStageCreateInfo shaderStages[] = {compShaderStageInfo};

    VkPushConstantRange pushConstantRange = {};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(UniformBufferObject);


    VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    // we need to explicitly create the layout here as the all in one texture is not set yet
    std::shared_ptr<VulkanDescriptorSetLayout> texLayout = VulkanAllInOneTexture::CreateLayout(m_vulkanDevice);
    std::vector<VkDescriptorSetLayout> pSetLayouts = {m_descriptorSetLayout->Layout(), texLayout->Layout(), texLayout->Layout()};
    pipelineLayoutInfo.pSetLayouts = pSetLayouts.data();
    pipelineLayoutInfo.setLayoutCount = static_cast<uint32_t>(pSetLayouts.size());
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;
    pipelineLayoutInfo.pushConstantRangeCount = 1;

    if (vkCreatePipelineLayout(m_vulkanDevice->Device(), &pipelineLayoutInfo, nullptr, &m_pipelineLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create pipeline layout!");
    }

    VkComputePipelineCreateInfo pipelineCreateInfo = {};
    pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineCreateInfo.stage = compShaderStageInfo;
    pipelineCreateInfo.layout = m_pipelineLayout;

    if (vkCreateComputePipelines(m_vulkanDevice->Device(), VK_NULL_HANDLE, 1, &pipelineCreateInfo, NULL, &m_computePipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create compute pipeline!");
    }
}


void VertexVisibilityVulkan::CreateDescriptorSetLayout()
{
    std::vector<VkDescriptorSetLayoutBinding> bindings;

    for (int i = 0; i < 3; i++) {
        // vertices, normals, visibility output
        VkDescriptorSetLayoutBinding storageLayoutBinding = {};
        storageLayoutBinding = {};
        storageLayoutBinding.binding = i;
        storageLayoutBinding.descriptorCount = 1;
        storageLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        storageLayoutBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        storageLayoutBinding.pImmutableSamplers = nullptr;
        bindings.push_back(storageLayoutBinding);
    }

    m_descriptorSetLayout = VulkanDescriptorSetLayout::CreateDescriptorSetLayout(m_vulkanDevice, bindings);
}


void VertexVisibilityVulkan::CreateDescriptorSets()
{
    m_descriptorSet = VulkanDescriptorSet::CreateDescriptorSet(m_descriptorSetLayout);
}

void VertexVisibilityVulkan::UpdateDescriptorSets(const VulkanMeshBuffer& meshBuffer, std::shared_ptr<VulkanManagedBuffer> visibilityBuffer)
{
    if (!visibilityBuffer) {
        throw std::runtime_error("missing visibility buffer");
    }
    if (!meshBuffer.m_vertexBuffer || !meshBuffer.m_normalBuffer) {
        throw std::runtime_error("missing vertex and/or normal buffer");
    }
    if (meshBuffer.m_vertexBuffer->Size() != 3 * visibilityBuffer->Size()) {
        throw std::runtime_error("vertex buffer and visibility buffer size are not matching");
    }

    std::vector<VkWriteDescriptorSet> descriptorWrites;
    VkWriteDescriptorSet descriptorWrite = {};

    descriptorWrites.clear();

    VkDescriptorBufferInfo bufferInfoVertices = {};
    bufferInfoVertices.buffer = meshBuffer.m_vertexBuffer->Buffer();
    bufferInfoVertices.offset = meshBuffer.m_vertexBuffer->Offset();
    bufferInfoVertices.range = meshBuffer.m_vertexBuffer->Size();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_descriptorSet->DescriptorSet();
    descriptorWrite.dstBinding = 0;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfoVertices;
    descriptorWrites.push_back(descriptorWrite);

    VkDescriptorBufferInfo bufferInfoNormals = {};
    bufferInfoNormals.buffer = meshBuffer.m_normalBuffer->Buffer();
    bufferInfoNormals.offset = meshBuffer.m_normalBuffer->Offset();
    bufferInfoNormals.range = meshBuffer.m_normalBuffer->Size();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_descriptorSet->DescriptorSet();
    descriptorWrite.dstBinding = 1;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfoNormals;
    descriptorWrites.push_back(descriptorWrite);

    VkDescriptorBufferInfo bufferInfoVisibility = {};
    bufferInfoVisibility.buffer = visibilityBuffer->Buffer();
    bufferInfoVisibility.offset = visibilityBuffer->Offset();
    bufferInfoVisibility.range = visibilityBuffer->Size();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_descriptorSet->DescriptorSet();
    descriptorWrite.dstBinding = 2;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfoVisibility;
    descriptorWrites.push_back(descriptorWrite);

    vkUpdateDescriptorSets(m_vulkanDevice->Device(), static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
}

} // namespace epic::nls
