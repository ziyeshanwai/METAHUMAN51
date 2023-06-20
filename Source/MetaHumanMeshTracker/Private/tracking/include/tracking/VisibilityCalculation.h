// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>

#include <nls/geometry/Mesh.h>

#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/common/VulkanMeshBuffer.h>
#include <nls/vulkan/DepthmapGenerationVulkan.h>
#include <nls/vulkan/VertexVisibilityVulkan.h>

namespace epic::nls {

/**
 * Helper class for efficient visibility calculation.
 * The visibility is calculated by rendering the mesh into a depthmap, then for each vertex look up whether the vertex is occluded, and if not then
 * set visibility based on the dot product between the vertex normal and the camera ray i.e. a vertex facing the camera has the highest score.
 */
class VisibilityCalculation
{
public:
    VisibilityCalculation(std::shared_ptr<VulkanDevice> vulkanDevice);

    Eigen::VectorXf CalculateVisibility(const Mesh<float>& mesh, const Camera<float>& visibilityCamera, const Eigen::Matrix4f& rigidMotion);

private:
    //! vulkan memory
    std::shared_ptr<VulkanMemory> m_vulkanMemory;
    //! used to create a depthmap of the mesh
    std::unique_ptr<DepthmapGenerationVulkan> m_depthmapGeneration;
    //! used to calculate the visibility of each vertex
    std::unique_ptr<VertexVisibilityVulkan> m_vertexVisibility;
    //! mesh buffer containing mesh on GPU
    VulkanMeshBuffer m_meshBuffer;
};

} // namespace epic::nls
