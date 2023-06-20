// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/DepthmapGenerationVulkan.h>


namespace epic::nls {

DepthmapGenerationVulkan::DepthmapGenerationVulkan(std::shared_ptr<VulkanDevice> vulkanDevice)
    : m_vulkanDevice(vulkanDevice)
{
    m_vulkanMemory = VulkanMemory::Create(m_vulkanDevice);
}


DepthmapGenerationVulkan::~DepthmapGenerationVulkan()
{
}


void DepthmapGenerationVulkan::SetCamera(const Camera<float>& camera, float near_, float far_)
{
    m_camera = camera;
    m_near = near_;
    m_far = far_;
    if (!m_depthRenderTarget) {
        m_depthRenderTarget = VulkanRenderTarget::Create(m_vulkanDevice, camera.Width(), camera.Height(), VK_SAMPLE_COUNT_1_BIT);
    }
    if (!m_meshRenderer) {
        m_meshRenderer = std::make_unique<MeshVulkanRenderer>(m_vulkanDevice);
        m_meshRenderer->Setup(m_depthRenderTarget->RenderPass(/*clear=*/true), m_depthRenderTarget->Samples(), VK_CULL_MODE_BACK_BIT);
    }
    m_depthRenderTarget->Resize(camera.Width(), camera.Height());
}

void DepthmapGenerationVulkan::Render(const VulkanMeshBuffer& meshBuffer, const Affine<float, 3, 3>& mesh2camera, bool clear)
{
    if (!m_depthRenderTarget || !m_meshRenderer) {
        throw std::runtime_error("first call SetCamera()");
    }

    VkCommandBuffer depthRenderBuffer = m_depthRenderTarget->StartRender(clear, Eigen::Vector4f(0, 0, 0, 0));
    m_meshRenderer->Render(depthRenderBuffer, m_camera.RenderingProjectionMatrix(m_near, m_far), m_camera.Extrinsics().Matrix() * mesh2camera.Matrix(), meshBuffer);
    m_depthRenderTarget->FinalizeRender(depthRenderBuffer, /*prepareForSampling=*/true);
}

const VulkanAllInOneTexture& DepthmapGenerationVulkan::RGBATexture()
{
    return m_depthRenderTarget->RGBATexture();
}

const VulkanAllInOneTexture& DepthmapGenerationVulkan::DepthTexture()
{
    return m_depthRenderTarget->DepthTexture();
}

// void DepthmapGenerationVulkan::SaveMesh(const std::string& filename)
// {
//     cv::Mat1f depthImage = m_depthRenderTarget->CopyDepthImage();

//     Eigen::Matrix4f mat = m_camera.RenderingProjectionMatrix(0.01f, 1000.0f).inverse();
//     for (int y = 0; y < depthImage.rows; ++y) {
//         for (int x = 0; x < depthImage.cols; ++x) {
//             if (depthImage(y, x) > 0) {
//                 Eigen::Vector4f pos = mat * Eigen::Vector4f(0, 0, depthImage(y, x), 1.0f);
//                 depthImage(y, x) = pos[2] / pos[3];
//             } else {
//                 depthImage(y, x) = 0;
//             }
//         }
//     }

//     const Mesh<float> mesh = DepthToMesh<float>(depthImage, m_camera);
//     ObjFileWriter<float> objWriter;
//     objWriter.writeObj(mesh, filename);
// }

} // namespace epic::nls
