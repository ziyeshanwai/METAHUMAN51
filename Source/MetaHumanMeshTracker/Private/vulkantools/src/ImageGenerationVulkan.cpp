// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/ImageGenerationVulkan.h>


namespace epic::nls {

ImageGenerationVulkan::ImageGenerationVulkan(std::shared_ptr<VulkanDevice> vulkanDevice, VkSampleCountFlagBits samples)
    : m_vulkanDevice(vulkanDevice)
    , m_samples(samples)
{
    m_vulkanMemory = VulkanMemory::Create(m_vulkanDevice);
}


ImageGenerationVulkan::~ImageGenerationVulkan()
{
}

void ImageGenerationVulkan::Setup()
{
    if (!m_renderTarget) {
        m_renderTarget = VulkanRenderTarget::Create(m_vulkanDevice, std::max<int>(1, m_camera.Width()), std::max<int>(1, m_camera.Height()), m_samples);
    }
    if (!m_texturedMeshRenderer) {
        m_texturedMeshRenderer = std::make_unique<TexturedMeshVulkanRenderer>(m_vulkanDevice);
        m_texturedMeshRenderer->Setup(m_renderTarget->RenderPass(/*clear=*/true), m_renderTarget->Samples());
    }
}

void ImageGenerationVulkan::SetMesh(const Mesh<float>& mesh)
{
    Setup();
    Mesh<float> triMesh = mesh;
    triMesh.Triangulate();
    m_meshBuffer = m_texturedMeshRenderer->CreateMeshBuffer(triMesh);
}

void ImageGenerationVulkan::SetCamera(const Camera<float>& camera, float near_, float far_)
{
    m_camera = camera;
    m_near = near_;
    m_far = far_;
    Setup();
    m_renderTarget->Resize(camera.Width(), camera.Height());
}

void ImageGenerationVulkan::Render(const Eigen::Matrix<float, 3, -1>& vertices, const VulkanAllInOneTexture& texture, const Affine<float, 3, 3>& mesh2camera)
{
    if (!m_renderTarget || !m_texturedMeshRenderer) {
        throw std::runtime_error("first call SetCamera()");
    }

    m_vulkanMemory->updateBuffer(m_meshBuffer.m_vertexBuffer.get(), vertices.data(), sizeof(vertices(0,0)) * vertices.size());
    VkCommandBuffer buffer = m_renderTarget->StartRender(/*clear=*/true, Eigen::Vector4f(0, 0, 0, 0));
    m_texturedMeshRenderer->Render(buffer, ProjectionMatrix(), ModelViewMatrix(mesh2camera), m_meshBuffer, texture);
    m_renderTarget->FinalizeRender(buffer, /*prepareForSampling=*/false);
}

void ImageGenerationVulkan::CopyImage(uint8_t* dataOut)
{
    return m_renderTarget->CopyImage<uint8_t>(dataOut);
}

const VulkanAllInOneTexture& ImageGenerationVulkan::RGBATexture()
{
    return m_renderTarget->RGBATexture();
}

const VulkanAllInOneTexture& ImageGenerationVulkan::DepthTexture()
{
    if (m_samples != VK_SAMPLE_COUNT_1_BIT) {
        throw std::runtime_error("depth texture is only supported for single sample images");
    }
    return m_renderTarget->DepthTexture();
}

Eigen::Matrix4f ImageGenerationVulkan::ModelViewMatrix(const Affine<float, 3, 3>& mesh2camera) const
{
    return m_camera.Extrinsics().Matrix() * mesh2camera.Matrix();
}

Eigen::Matrix4f ImageGenerationVulkan::ProjectionMatrix() const
{
    return m_camera.RenderingProjectionMatrix(m_near, m_far);
}

} // namespace epic::nls
