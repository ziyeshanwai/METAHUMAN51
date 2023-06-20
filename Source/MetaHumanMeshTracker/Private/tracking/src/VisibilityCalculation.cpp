// Copyright Epic Games, Inc. All Rights Reserved.

#include <tracking/VisibilityCalculation.h>

namespace epic::nls {

VisibilityCalculation::VisibilityCalculation(std::shared_ptr<VulkanDevice> vulkanDevice)
{
    m_depthmapGeneration = std::make_unique<DepthmapGenerationVulkan>(vulkanDevice);
    m_vertexVisibility = std::make_unique<VertexVisibilityVulkan>(vulkanDevice);
    m_vulkanMemory = VulkanMemory::Create(vulkanDevice);
}

Eigen::VectorXf VisibilityCalculation::CalculateVisibility(const Mesh<float>& mesh, const Camera<float>& visibilityCamera, const Eigen::Matrix4f& rigidMotion)
{
    if (!m_meshBuffer.m_vertexBuffer) {
        m_meshBuffer.CreateBuffers(m_vulkanMemory, mesh);
    } else {
        m_meshBuffer.UpdateDynamicBuffers(m_vulkanMemory, mesh);
    }

    m_depthmapGeneration->SetCamera(visibilityCamera, 1.0f, 1000.0f);
    m_depthmapGeneration->Render(m_meshBuffer, rigidMotion, /*clear=*/true);

    Eigen::Matrix4f toTexture = Eigen::Matrix4f::Identity();
    toTexture(0,0) = 0.5f;
    toTexture(1,1) = 0.5f;
    toTexture(0,3) = 0.5f;
    toTexture(1,3) = 0.5f;
    const Eigen::Matrix4f visibilityModelViewMatrix = visibilityCamera.Extrinsics().Matrix() * rigidMotion;
    const Eigen::Matrix4f visibilityProjectionMatrix = toTexture * visibilityCamera.RenderingProjectionMatrix(1.0f, 1000.0f);
    return m_vertexVisibility->Compute(visibilityProjectionMatrix, visibilityModelViewMatrix, m_meshBuffer, m_depthmapGeneration->DepthTexture(), m_depthmapGeneration->RGBATexture());
}

} // namespace epic::nls
