// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/DepthmapVulkanRenderer.h>

#include <nls/vulkan/shaders/embed_depthmaprenderer.vert.spv.h>
#include <nls/vulkan/shaders/embed_depthmaprenderer.frag.spv.h>

namespace epic::nls {

struct DepthmapVulkanRenderer::UniformBufferObject {
    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> projectionMatrix;
    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> modelViewMatrix;
    Eigen::Matrix<float, 4, 1, Eigen::DontAlign> color;
    uint32_t depthMapWidth;
    float fx;
    float fy;
    float skew;
    float cx;
    float cy;
};

DepthmapVulkanRenderer::DepthmapVulkanRenderer(std::shared_ptr<VulkanDevice> vulkanDevice)
    : m_vulkanDevice(vulkanDevice)
{
    m_vulkanMemory = VulkanMemory::Create(m_vulkanDevice);
}


DepthmapVulkanRenderer::~DepthmapVulkanRenderer()
{
    Reset();
}


void DepthmapVulkanRenderer::Reset()
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

void DepthmapVulkanRenderer::Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples)
{
    CreateGraphicsPipeline(renderPass, samples);
}

DepthmapVulkanRenderer::RenderDataConstPtr DepthmapVulkanRenderer::CreateDepthAndNormalData(const epic::nls::Camera<float>& depthCamera, const float* depthAndNormals) const
{
    std::shared_ptr<RenderData> renderData = std::make_shared<RenderData>();
    renderData->buffer = m_vulkanMemory->createBufferOnDeviceAndCopy(depthAndNormals, depthCamera.Width() * depthCamera.Height() * 4 * sizeof(float), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);;
    renderData->camera = depthCamera;
    return renderData;
}

void DepthmapVulkanRenderer::SetDepthAndNormalData(const epic::nls::Camera<float>& depthCamera, const std::shared_ptr<VulkanManagedBuffer>& buffer)
{
    auto renderData = std::make_shared<RenderData>();
    renderData->buffer = buffer;
    renderData->camera = depthCamera;
    m_renderData = renderData;
}

void DepthmapVulkanRenderer::SetDepthAndNormalData(const epic::nls::Camera<float>& depthCamera, const float* depthAndNormals)
{
    m_renderData = CreateDepthAndNormalData(depthCamera, depthAndNormals);
}

void DepthmapVulkanRenderer::Render(const Eigen::Matrix4f& projectionMatrix, const Eigen::Matrix4f& modelViewMatrix, VkCommandBuffer buffer, const Eigen::Vector4f& color)
{
    if (m_renderData) {
        Render(m_renderData, projectionMatrix, modelViewMatrix, buffer, color);
    }
}

void DepthmapVulkanRenderer::Render(RenderDataConstPtr renderData, const Eigen::Matrix4f& projectionMatrix, const Eigen::Matrix4f& modelViewMatrix, VkCommandBuffer buffer, const Eigen::Vector4f& color)
{
    if (!renderData || !renderData->buffer) {
        throw std::runtime_error("set the depth and normal data using CreateDepthAndNormalData() before calling Render()");
    }

    UniformBufferObject ubo;
    Eigen::Matrix4f extrinsics = modelViewMatrix * renderData->camera.Extrinsics().Inverse().Matrix();
    ubo.projectionMatrix = projectionMatrix * extrinsics;
    ubo.modelViewMatrix = extrinsics;
    ubo.color = color;
    ubo.depthMapWidth = renderData->camera.Width();
    ubo.fx = renderData->camera.Intrinsics()(0,0);
    ubo.fy = renderData->camera.Intrinsics()(1,1);
    ubo.skew = renderData->camera.Intrinsics()(0,1);
    ubo.cx = renderData->camera.Intrinsics()(0,2);
    ubo.cy = renderData->camera.Intrinsics()(1,2);

    vkCmdPushConstants(buffer, m_pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(ubo), &ubo);

    vkCmdBindPipeline(buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_graphicsPipeline);

    VkBuffer buffers[] = {renderData->buffer->Buffer()};
    VkDeviceSize offsets[] = {renderData->buffer->Offset()};
    vkCmdBindVertexBuffers(buffer, 0, 1, buffers, offsets);

    vkCmdDraw(buffer, static_cast<uint32_t>(renderData->camera.Width() * renderData->camera.Height()), 1, 0, 0);
}


void DepthmapVulkanRenderer::CreateGraphicsPipeline(VkRenderPass renderPass, VkSampleCountFlagBits samples)
{
    std::unique_ptr<VulkanShaderModule> vertShaderModule = VulkanShaderModule::Create(m_vulkanDevice, SPV_depthmaprenderer_vert_spv, sizeof(SPV_depthmaprenderer_vert_spv));
    std::unique_ptr<VulkanShaderModule> fragShaderModule = VulkanShaderModule::Create(m_vulkanDevice, SPV_depthmaprenderer_frag_spv, sizeof(SPV_depthmaprenderer_frag_spv));

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
    posBindingDescription.stride = static_cast<uint32_t>(4 * sizeof(float));
    posBindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputBindingDescription> bindingDescriptions = { posBindingDescription };

    VkVertexInputAttributeDescription posAttributeDescription = {};
    posAttributeDescription.binding = 0;
    posAttributeDescription.location = 0;
    posAttributeDescription.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    posAttributeDescription.offset = 0;

    std::vector<VkVertexInputAttributeDescription> attributeDescriptions = { posAttributeDescription };

    VkPipelineVertexInputStateCreateInfo vertexInputInfo = {};
    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInputInfo.vertexBindingDescriptionCount = static_cast<uint32_t>(bindingDescriptions.size());
    vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions.data();
    vertexInputInfo.vertexAttributeDescriptionCount =static_cast<uint32_t>(attributeDescriptions.size());
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly = {};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    // inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
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
    depthStencil.depthCompareOp = VK_COMPARE_OP_GREATER;
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
    colorBlendAttachment.blendEnable = VK_FALSE;
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
    pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(UniformBufferObject);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 0; // Optional
    pipelineLayoutInfo.pSetLayouts = nullptr; // Optional
    pipelineLayoutInfo.pushConstantRangeCount = 1; // Optional
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange; // Optional

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


} // namespace epic::nls
