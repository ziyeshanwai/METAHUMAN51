// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanRenderTarget.h>
#include <nls/vulkan/MeshVulkanRenderer.h>

#include <nls/geometry/Camera.h>
#include <nls/geometry/Mesh.h>

namespace epic::nls {

/**
 * Class to render meshes into a depthmap.
 */
class DepthmapGenerationVulkan {
public:
    DepthmapGenerationVulkan(std::shared_ptr<VulkanDevice> vulkanDevice);

    ~DepthmapGenerationVulkan();

    void SetCamera(const Camera<float>& camera, float near_, float far_);

    void Render(const VulkanMeshBuffer& meshBuffer, const Affine<float, 3, 3>& mesh2camera, bool clear);

    VulkanRenderTarget* RenderTarget() { return m_depthRenderTarget.get(); }

    const VulkanAllInOneTexture& RGBATexture();
    const VulkanAllInOneTexture& DepthTexture();

private:

    std::shared_ptr<VulkanDevice> m_vulkanDevice;
    std::shared_ptr<VulkanMemory> m_vulkanMemory;

    std::unique_ptr<VulkanRenderTarget> m_depthRenderTarget;
    std::unique_ptr<MeshVulkanRenderer> m_meshRenderer;

    Camera<float> m_camera;
    float m_near;
    float m_far;
};


} // namespace epic::nls
