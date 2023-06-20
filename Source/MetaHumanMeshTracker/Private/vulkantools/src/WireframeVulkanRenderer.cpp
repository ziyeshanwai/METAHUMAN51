// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/WireframeVulkanRenderer.h>

#include <nls/vulkan/shaders/embed_wireframerenderer.vert.spv.h>
#include <nls/vulkan/shaders/embed_wireframerenderer.frag.spv.h>

#include <set>
#include <vector>

namespace epic {
namespace nls {

struct WireframeVulkanRenderer::UniformBufferObject {
    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> projectionMatrix;
    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> modelViewMatrix;
    Eigen::Matrix<float, 4, 1, Eigen::DontAlign> color;
};

WireframeVulkanRenderer::WireframeVulkanRenderer(std::shared_ptr<VulkanDevice> vulkanDevice)
    : m_vulkanDevice(vulkanDevice)
{
    m_vulkanMemory = VulkanMemory::Create(m_vulkanDevice);
}


WireframeVulkanRenderer::~WireframeVulkanRenderer()
{
    Reset();
}


void WireframeVulkanRenderer::Reset()
{
    if (m_graphicsPipeline) {
        vkDestroyPipeline(m_vulkanDevice->Device(), m_graphicsPipeline, nullptr);
        m_graphicsPipeline = VK_NULL_HANDLE;
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


void WireframeVulkanRenderer::Setup(VkRenderPass renderPass, VkSampleCountFlagBits samples)
{
    CreateDescriptorSetLayout();
    m_graphicsPipeline = CreateGraphicsPipeline(renderPass, samples);
}

void WireframeVulkanRenderer::CreateWireframeIndexBuffer(VulkanMeshBuffer& meshBuffer, const epic::nls::Mesh<float>& mesh)
{
    std::vector<std::pair<int, int>> edgeVector = mesh.GetEdges({});
    meshBuffer.m_wireframeIndexBuffer = m_vulkanMemory->createBufferOnDeviceAndCopy(edgeVector.data(), sizeof(edgeVector[0]) * edgeVector.size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
}

VulkanMeshBuffer WireframeVulkanRenderer::CreateWireframeBuffer(const epic::nls::Mesh<float>& mesh)
{
    VulkanMeshBuffer meshBuffer;
    meshBuffer.m_vertexBuffer = m_vulkanMemory->createBufferOnDeviceAndCopy(mesh.Vertices().data(), sizeof(mesh.Vertices()(0,0)) * mesh.Vertices().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    CreateWireframeIndexBuffer(meshBuffer, mesh);
    return meshBuffer;
}

void WireframeVulkanRenderer::Render(VkCommandBuffer buffer,
                                const Eigen::Matrix4f& projectionMatrix,
                                const Eigen::Matrix4f& modelViewMatrix,
                                const VulkanMeshBuffer& meshBuffer,
                                const Eigen::Vector4f& color) const
{
    if (!meshBuffer.m_vertexBuffer) {
        throw std::runtime_error("meshbuffer does not contain any vertices");
    }
    if (!meshBuffer.m_wireframeIndexBuffer) {
        throw std::runtime_error("meshbuffer does not contain any wireframe indices");
    }

    UniformBufferObject ubo;
    Eigen::Matrix4f depthBiasMatrix = Eigen::Matrix4f::Identity();
    depthBiasMatrix(2, 2) = m_depthScale;
    depthBiasMatrix(2, 3) = m_depthBias;
    ubo.projectionMatrix = depthBiasMatrix * projectionMatrix * modelViewMatrix;
    ubo.modelViewMatrix = modelViewMatrix;
    ubo.color = color;

    vkCmdPushConstants(buffer, m_pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(ubo), &ubo);
    vkCmdBindPipeline(buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_graphicsPipeline);

    VkBuffer buffers[] = {meshBuffer.m_vertexBuffer->Buffer()};
    VkDeviceSize offsets[] = {meshBuffer.m_vertexBuffer->Offset()};
    vkCmdBindVertexBuffers(buffer, 0, 1, buffers, offsets);
    vkCmdBindIndexBuffer(buffer, meshBuffer.m_wireframeIndexBuffer->Buffer(), meshBuffer.m_wireframeIndexBuffer->Offset(), VK_INDEX_TYPE_UINT32);

    vkCmdDrawIndexed(buffer, int(meshBuffer.m_wireframeIndexBuffer->Size() / sizeof(int)), 1, 0, 0, 0);
}

void WireframeVulkanRenderer::CreateDescriptorSetLayout()
{
    std::vector<VkDescriptorSetLayoutBinding> bindings;
    {
        // camera projection matrix
        VkDescriptorSetLayoutBinding uboLayoutBinding = {};
        uboLayoutBinding.binding = 0;
        uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        uboLayoutBinding.descriptorCount = 1;
        uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        uboLayoutBinding.pImmutableSamplers = nullptr;
        bindings.push_back(uboLayoutBinding);
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo = {};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    if (vkCreateDescriptorSetLayout(m_vulkanDevice->Device(), &layoutInfo, nullptr, &m_descriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor set layout!");
    }
}


VkPipeline WireframeVulkanRenderer::CreateGraphicsPipeline(VkRenderPass renderPass, VkSampleCountFlagBits samples)
{
    std::unique_ptr<VulkanShaderModule> vertShaderModule = VulkanShaderModule::Create(m_vulkanDevice, SPV_wireframerenderer_vert_spv, sizeof(SPV_wireframerenderer_vert_spv));
    std::unique_ptr<VulkanShaderModule> fragShaderModule = VulkanShaderModule::Create(m_vulkanDevice, SPV_wireframerenderer_frag_spv, sizeof(SPV_wireframerenderer_frag_spv));

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

    std::vector<VkVertexInputBindingDescription> bindingDescriptions = {posBindingDescription};

    VkVertexInputAttributeDescription posAttributeDescription = {};
    posAttributeDescription.binding = 0;
    posAttributeDescription.location = 0;
    posAttributeDescription.format = VK_FORMAT_R32G32B32_SFLOAT;
    posAttributeDescription.offset = 0;

    std::vector<VkVertexInputAttributeDescription> attributeDescriptions = {posAttributeDescription};

    VkPipelineVertexInputStateCreateInfo vertexInputInfo = {};
    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInputInfo.vertexBindingDescriptionCount = static_cast<uint32_t>(bindingDescriptions.size());
    vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions.data();
    vertexInputInfo.vertexAttributeDescriptionCount =static_cast<uint32_t>(attributeDescriptions.size());
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly = {};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
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
    rasterizer.polygonMode = VK_POLYGON_MODE_LINE;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_NONE; //VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.depthBiasEnable = VK_FALSE;
    rasterizer.depthBiasConstantFactor = 1.0f; // Optional
    rasterizer.depthBiasClamp = 0.000001f; // Optional
    rasterizer.depthBiasSlopeFactor = 1.0f; // Optional

    VkPipelineDepthStencilStateCreateInfo depthStencil = {};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_TRUE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;
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
    colorBlendAttachment.blendEnable = VK_TRUE;
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

    if (m_pipelineLayout == VK_NULL_HANDLE) {
        VkPushConstantRange pushConstantRange = {};
        pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        pushConstantRange.offset = 0;
        pushConstantRange.size = sizeof(UniformBufferObject);

        VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
        pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipelineLayoutInfo.setLayoutCount = 1; // Optional
        pipelineLayoutInfo.pSetLayouts = &m_descriptorSetLayout; // Optional
        pipelineLayoutInfo.pushConstantRangeCount = 1; // Optional
        pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange; // Optional


        if (vkCreatePipelineLayout(m_vulkanDevice->Device(), &pipelineLayoutInfo, nullptr, &m_pipelineLayout) != VK_SUCCESS) {
            throw std::runtime_error("failed to create pipeline layout!");
        }
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

    VkPipeline graphicsPipeline = VK_NULL_HANDLE;
    if (vkCreateGraphicsPipelines(m_vulkanDevice->Device(), VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &graphicsPipeline) != VK_SUCCESS) {
        throw std::runtime_error("failed to create graphics pipeline!");
    }
    return graphicsPipeline;
}

} // namespace nls
} //namespace epic
