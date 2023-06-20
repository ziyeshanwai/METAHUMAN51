// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/UnwrapTextureVulkan.h>

#include <nls/vulkan/shaders/embed_unwraptexture.vert.spv.h>
#include <nls/vulkan/shaders/embed_unwraptexture.frag.spv.h>


namespace epic::nls {

struct UnwrapTextureVulkan::UniformBufferObject {
    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> uvTransformMatrix;
    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> textureModelViewProjectionMatrix;
    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> textureModelViewMatrix;
    Eigen::Matrix<float, 2, 2, Eigen::DontAlign> textureProjectionMatrixInverse;
    uint32_t forVisualization;
    uint32_t redAndUV;  // only output the red channel (or gray for monochrome images), and output the projected camera position in the green and blue color channel
};

UnwrapTextureVulkan::UnwrapTextureVulkan(std::shared_ptr<VulkanDevice> vulkanDevice, bool forVisualization, bool outputImageCoordinates)
    : m_vulkanDevice(vulkanDevice)
    , m_forVisualization(forVisualization)
    , m_outputImageCoordinates(outputImageCoordinates)
{
    m_vulkanMemory = VulkanMemory::Create(m_vulkanDevice);
}


UnwrapTextureVulkan::~UnwrapTextureVulkan()
{
    Reset();
}


void UnwrapTextureVulkan::Reset()
{
    if (m_graphicsPipeline) {
        vkDestroyPipeline(m_vulkanDevice->Device(), m_graphicsPipeline, nullptr);
        m_graphicsPipeline = VK_NULL_HANDLE;
    }
    if (m_pipelineLayout) {
        vkDestroyPipelineLayout(m_vulkanDevice->Device(), m_pipelineLayout, nullptr);
        m_pipelineLayout = VK_NULL_HANDLE;
    }
}


void UnwrapTextureVulkan::Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples)
{
    CreateDescriptorSetLayout();
    CreateGraphicsPipeline(renderPass, samples);
    CreateDescriptorSets();
}


void UnwrapTextureVulkan::CreateTexcoordAndTexcoordIndexBuffer(VulkanMeshBuffer& meshBuffer, const epic::nls::Mesh<float>& mesh)
{
    if (mesh.TexTriangles().cols() != mesh.Triangles().cols()) {
        CARBON_CRITICAL("Number of texture triangles ({}) do not match the number of triangles in the mesh ({}).", mesh.TexTriangles().cols(), mesh.Triangles().cols());
    }
    if (mesh.Triangles() == mesh.TexTriangles()) {
        CARBON_CRITICAL("Triangles and texture triangles are not the same ({} vs {}). Use TexturedMeshVulkanRenderer in that case.", mesh.Triangles(), mesh.TexTriangles());
    }

    meshBuffer.m_texcoordBuffer = m_vulkanMemory->createBufferOnDeviceAndCopy(mesh.Texcoords().data(), sizeof(mesh.Texcoords()(0,0)) * mesh.Texcoords().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    meshBuffer.m_texcoordIndexBuffer = m_vulkanMemory->createBufferOnDeviceAndCopy(mesh.TexTriangles().data(), sizeof(mesh.TexTriangles()(0,0)) * mesh.TexTriangles().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
}


VulkanMeshBuffer UnwrapTextureVulkan::CreateMeshBuffer(const epic::nls::Mesh<float>& mesh)
{
    if (!mesh.HasVertexNormals()) {
        throw std::runtime_error("mesh is missing vertex normals");
    }
    VulkanMeshBuffer meshBuffer;
    meshBuffer.m_vertexBuffer = m_vulkanMemory->createBufferOnDeviceAndCopy(mesh.Vertices().data(), sizeof(mesh.Vertices()(0,0)) * mesh.Vertices().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    meshBuffer.m_normalBuffer = m_vulkanMemory->createBufferOnDeviceAndCopy(mesh.VertexNormals().data(), sizeof(mesh.VertexNormals()(0,0)) * mesh.VertexNormals().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    meshBuffer.m_indexBuffer = m_vulkanMemory->createBufferOnDeviceAndCopy(mesh.Triangles().data(), sizeof(mesh.Triangles()(0,0)) * mesh.Triangles().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    CreateTexcoordAndTexcoordIndexBuffer(meshBuffer, mesh);
    return meshBuffer;
}


void UnwrapTextureVulkan::Render(VkCommandBuffer buffer,
                              const Eigen::Matrix4f& uvTransformMatrix,
                              const Eigen::Matrix4f& textureProjectionMatrix,
                              const Eigen::Matrix4f& textureModelViewMatrix,
                              const VulkanMeshBuffer& meshBuffer,
                              const VulkanAllInOneTexture& imageTexture,
                              const VulkanAllInOneTexture& depthTexture,
                              const VulkanAllInOneTexture& depthAlphaTexture)
{
    UpdateDescriptorSets(meshBuffer);

    UniformBufferObject ubo;
    ubo.uvTransformMatrix = uvTransformMatrix;
    ubo.textureModelViewProjectionMatrix = textureProjectionMatrix * textureModelViewMatrix;
    ubo.textureModelViewMatrix = textureModelViewMatrix;
    ubo.textureProjectionMatrixInverse = textureProjectionMatrix.cast<double>().inverse().cast<float>().block(2,2,2,2);
    ubo.forVisualization = m_forVisualization ? 1 : 0;
    ubo.redAndUV = m_outputImageCoordinates ? 1 : 0;

    vkCmdPushConstants(buffer, m_pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(ubo), &ubo);
    vkCmdBindPipeline(buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_graphicsPipeline);

    std::vector<VkDescriptorSet> descriptorSets = {imageTexture.DescriptorSet(), m_drawDescriptorSet->DescriptorSet(), depthTexture.DescriptorSet(), depthAlphaTexture.DescriptorSet()};
    vkCmdBindDescriptorSets(buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayout, 0, 4, descriptorSets.data(), 0, nullptr);

    vkCmdDraw(buffer, int(meshBuffer.m_indexBuffer->Size() / sizeof(int)), 1, 0, 0);
}

void UnwrapTextureVulkan::CreateGraphicsPipeline(VkRenderPass renderPass, VkSampleCountFlagBits samples)
{
    std::unique_ptr<VulkanShaderModule> vertShaderModule = VulkanShaderModule::Create(m_vulkanDevice, SPV_unwraptexture_vert_spv, sizeof(SPV_unwraptexture_vert_spv));
    std::unique_ptr<VulkanShaderModule> fragShaderModule = VulkanShaderModule::Create(m_vulkanDevice, SPV_unwraptexture_frag_spv, sizeof(SPV_unwraptexture_frag_spv));

    VkPipelineShaderStageCreateInfo vertShaderStageInfo = {};
    vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertShaderStageInfo.module = vertShaderModule->ShaderModule();
    vertShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo fragShaderStageInfo = {};
    fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragShaderStageInfo.module = fragShaderModule->ShaderModule();
    fragShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo, fragShaderStageInfo};

    VkVertexInputBindingDescription posBindingDescription = {};
    posBindingDescription.binding = 0;
    posBindingDescription.stride = static_cast<uint32_t>(3 * sizeof(float));
    posBindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputBindingDescription texcoordBindingDescription = {};
    texcoordBindingDescription.binding = 1;
    texcoordBindingDescription.stride = static_cast<uint32_t>(3 * sizeof(float));
    texcoordBindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputBindingDescription> bindingDescriptions = {posBindingDescription, texcoordBindingDescription};

    VkVertexInputAttributeDescription posAttributeDescription = {};
    posAttributeDescription.binding = 0;
    posAttributeDescription.location = 0;
    posAttributeDescription.format = VK_FORMAT_R32G32B32_SFLOAT;
    posAttributeDescription.offset = 0;

    VkVertexInputAttributeDescription texcoordAttributeDescription = {};
    texcoordAttributeDescription.binding = 1;
    texcoordAttributeDescription.location = 1;
    texcoordAttributeDescription.format = VK_FORMAT_R32G32_SFLOAT;
    texcoordAttributeDescription.offset = 0;

    std::vector<VkVertexInputAttributeDescription> attributeDescriptions = {posAttributeDescription, texcoordAttributeDescription};

    // no vertex attributes as we use the gl_VertexIndex to index into the vertex and texcoords index arrays, and then the vertex and texcoords arrays
    VkPipelineVertexInputStateCreateInfo vertexInputInfo = {};
    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInputInfo.vertexBindingDescriptionCount = 0;//static_cast<uint32_t>(bindingDescriptions.size());
    vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions.data();
    vertexInputInfo.vertexAttributeDescriptionCount = 0;//static_cast<uint32_t>(attributeDescriptions.size());
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly = {};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    VkPipelineViewportStateCreateInfo viewportState = {};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.pViewports = nullptr;
    viewportState.scissorCount = 1;
    viewportState.pScissors = nullptr;

    VkPipelineRasterizationStateCreateInfo rasterizer = {};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_NONE; //VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.depthBiasEnable = VK_FALSE;
    rasterizer.depthBiasConstantFactor = 0.0f; // Optional
    rasterizer.depthBiasClamp = 0.0f; // Optional
    rasterizer.depthBiasSlopeFactor = 0.0f; // Optional

    VkPipelineDepthStencilStateCreateInfo depthStencil = {};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_TRUE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL ;
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.minDepthBounds = 0.0f; // Optional
    depthStencil.maxDepthBounds = 1.0f; // Optional
    depthStencil.stencilTestEnable = VK_FALSE;
    depthStencil.front = {}; // Optional
    depthStencil.back = {}; // Optional

    VkPipelineMultisampleStateCreateInfo multisampling = {};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = samples;
    multisampling.minSampleShading = 1.0f; // Optional
    multisampling.pSampleMask = nullptr; // Optional
    multisampling.alphaToCoverageEnable = VK_FALSE; // Optional
    multisampling.alphaToOneEnable = VK_FALSE; // Optional

    VkPipelineColorBlendAttachmentState colorBlendAttachment = {};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = m_forVisualization ? VK_TRUE : VK_FALSE;
    colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
    colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

    VkPipelineColorBlendStateCreateInfo colorBlending = {};
    colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.logicOp = VK_LOGIC_OP_COPY; // Optional
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;
    colorBlending.blendConstants[0] = 0.0f; // Optional
    colorBlending.blendConstants[1] = 0.0f; // Optional
    colorBlending.blendConstants[2] = 0.0f; // Optional
    colorBlending.blendConstants[3] = 0.0f; // Optional

    VkPushConstantRange pushConstantRange = {};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(UniformBufferObject);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    // we need to explicitly create the layout here as the all in one texture is not set yet
    std::shared_ptr<VulkanDescriptorSetLayout> texLayout = VulkanAllInOneTexture::CreateLayout(m_vulkanDevice);
    std::vector<VkDescriptorSetLayout> pSetLayouts = {texLayout->Layout(), m_drawDescriptorSetLayout->Layout(), texLayout->Layout(), texLayout->Layout() };
    pipelineLayoutInfo.setLayoutCount = static_cast<uint32_t>(pSetLayouts.size());
    pipelineLayoutInfo.pSetLayouts = pSetLayouts.data();
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    if (vkCreatePipelineLayout(m_vulkanDevice->Device(), &pipelineLayoutInfo, nullptr, &m_pipelineLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create pipeline layout!");
    }

     VkPipelineDynamicStateCreateInfo dynamicStateInfo = {};
     dynamicStateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
     dynamicStateInfo.dynamicStateCount = 2;
     VkDynamicState dynamicStates[] = {VK_DYNAMIC_STATE_VIEWPORT , VK_DYNAMIC_STATE_SCISSOR};
     dynamicStateInfo.pDynamicStates = dynamicStates;

    VkGraphicsPipelineCreateInfo pipelineInfo = {};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages;
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicStateInfo; // Optional
    pipelineInfo.layout = m_pipelineLayout;
    pipelineInfo.renderPass = renderPass;
    pipelineInfo.subpass = 0;
    pipelineInfo.basePipelineHandle = VK_NULL_HANDLE; // Optional
    pipelineInfo.basePipelineIndex = -1; // Optional

    if (vkCreateGraphicsPipelines(m_vulkanDevice->Device(), VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_graphicsPipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create graphics pipeline!");
    }
}


void UnwrapTextureVulkan::CreateDescriptorSetLayout()
{
    std::vector<VkDescriptorSetLayoutBinding> bindings;
    for (int i = 0; i < 5; i++) {
        // vertex indices, texcoord indices, vertices, texcoords, normals
        VkDescriptorSetLayoutBinding storageLayoutBinding = {};
        storageLayoutBinding = {};
        storageLayoutBinding.binding = i;
        storageLayoutBinding.descriptorCount = 1;
        storageLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        storageLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        storageLayoutBinding.pImmutableSamplers = nullptr;
        bindings.push_back(storageLayoutBinding);
    }

    m_drawDescriptorSetLayout = VulkanDescriptorSetLayout::CreateDescriptorSetLayout(m_vulkanDevice, bindings);
}


void UnwrapTextureVulkan::CreateDescriptorSets()
{
    m_drawDescriptorSet = VulkanDescriptorSet::CreateDescriptorSet(m_drawDescriptorSetLayout);
}

void UnwrapTextureVulkan::UpdateDescriptorSets(const VulkanMeshBuffer& meshBuffer)
{
    std::vector<VkWriteDescriptorSet> descriptorWrites;
    VkWriteDescriptorSet descriptorWrite = {};

    descriptorWrites.clear();

    VkDescriptorBufferInfo bufferInfoIndices = {};
    bufferInfoIndices.buffer = meshBuffer.m_indexBuffer->Buffer();
    bufferInfoIndices.offset = meshBuffer.m_indexBuffer->Offset();
    bufferInfoIndices.range = meshBuffer.m_indexBuffer->Size();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_drawDescriptorSet->DescriptorSet();
    descriptorWrite.dstBinding = 0;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfoIndices;
    descriptorWrites.push_back(descriptorWrite);

    VkDescriptorBufferInfo bufferInfoTexcoordIndices = {};
    descriptorWrite.dstBinding = 1;
    bufferInfoTexcoordIndices.buffer = meshBuffer.m_texcoordIndexBuffer->Buffer();
    bufferInfoTexcoordIndices.offset = meshBuffer.m_texcoordIndexBuffer->Offset();
    bufferInfoTexcoordIndices.range = meshBuffer.m_texcoordIndexBuffer->Size();
    descriptorWrite.pBufferInfo = &bufferInfoTexcoordIndices;
    descriptorWrites.push_back(descriptorWrite);


    VkDescriptorBufferInfo bufferInfoVertices = {};
    bufferInfoVertices.buffer = meshBuffer.m_vertexBuffer->Buffer();
    bufferInfoVertices.offset = meshBuffer.m_vertexBuffer->Offset();
    bufferInfoVertices.range = meshBuffer.m_vertexBuffer->Size();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_drawDescriptorSet->DescriptorSet();
    descriptorWrite.dstBinding = 2;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfoVertices;
    descriptorWrites.push_back(descriptorWrite);


    VkDescriptorBufferInfo bufferInfoTexcoords = {};
    bufferInfoTexcoords.buffer = meshBuffer.m_texcoordBuffer->Buffer();
    bufferInfoTexcoords.offset = meshBuffer.m_texcoordBuffer->Offset();
    bufferInfoTexcoords.range = meshBuffer.m_texcoordBuffer->Size();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_drawDescriptorSet->DescriptorSet();
    descriptorWrite.dstBinding = 3;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfoTexcoords;
    descriptorWrites.push_back(descriptorWrite);

    VkDescriptorBufferInfo bufferInfoNormals = {};
    bufferInfoNormals.buffer = meshBuffer.m_normalBuffer->Buffer();
    bufferInfoNormals.offset = meshBuffer.m_normalBuffer->Offset();
    bufferInfoNormals.range = meshBuffer.m_normalBuffer->Size();

    descriptorWrite = {};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = m_drawDescriptorSet->DescriptorSet();
    descriptorWrite.dstBinding = 4;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfoNormals;
    descriptorWrites.push_back(descriptorWrite);


    vkUpdateDescriptorSets(m_vulkanDevice->Device(), static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
}

} // namespace epic::nls
