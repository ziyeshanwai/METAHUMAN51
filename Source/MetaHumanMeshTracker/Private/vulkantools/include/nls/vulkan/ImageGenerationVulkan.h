// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanRenderTarget.h>
#include <nls/vulkan/common/VulkanAllInOneTexture.h>
#include <nls/vulkan/common/VulkanMeshBuffer.h>
#include <nls/vulkan/TexturedMeshVulkanRenderer.h>

#include <nls/geometry/Camera.h>
#include <nls/geometry/Mesh.h>

namespace epic::nls {

/**
 * Class to render textured meshes into an image.
 */
class ImageGenerationVulkan {
public:
    ImageGenerationVulkan(std::shared_ptr<VulkanDevice> vulkanDevice, VkSampleCountFlagBits samples);

    ~ImageGenerationVulkan();

    void SetMesh(const Mesh<float>& mesh);

    void SetCamera(const Camera<float>& camera, float near_, float far_);

    void Render(const Eigen::Matrix<float, 3, -1>& vertices, const VulkanAllInOneTexture& texture, const Affine<float, 3, 3>& mesh2camera);

    //! copy the generated image to @p dataOut. dataOut needs to be an image of size 4(channels) * width * height
    void CopyImage(uint8_t* dataOut);

    VulkanRenderTarget* RenderTarget() { return m_renderTarget.get(); }
    const VulkanAllInOneTexture& RGBATexture();

    const VulkanAllInOneTexture& DepthTexture();

    const VulkanMeshBuffer& CurrentMeshBuffer() const { return m_meshBuffer; }

    Eigen::Matrix4f ModelViewMatrix(const Affine<float, 3, 3>& mesh2camera) const;

    Eigen::Matrix4f ProjectionMatrix() const;

private:
    void Setup();

private:

    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;
    VkSampleCountFlagBits m_samples;
    VulkanMeshBuffer m_meshBuffer;

    std::unique_ptr<VulkanRenderTarget> m_renderTarget;
    std::unique_ptr<TexturedMeshVulkanRenderer> m_texturedMeshRenderer;

    Camera<float> m_camera;
    float m_near;
    float m_far;
};


} // namespace epic::nls
