// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/Mesh.h>
#include <nls/vulkan/common/VulkanMemory.h>

namespace epic {
namespace nls {


struct VulkanMeshBuffer
{
    std::shared_ptr<VulkanManagedBuffer> m_vertexBuffer;
    std::shared_ptr<VulkanManagedBuffer> m_normalBuffer;
    std::shared_ptr<VulkanManagedBuffer> m_colorBuffer;
    std::shared_ptr<VulkanManagedBuffer> m_texcoordBuffer;
    std::shared_ptr<VulkanManagedBuffer> m_indexBuffer;
    std::shared_ptr<VulkanManagedBuffer> m_wireframeIndexBuffer;
    // texcoord index buffer when indices for texcoords and vertices do not match (e.g. meshes with multiple texcoords per vertex)
    std::shared_ptr<VulkanManagedBuffer> m_texcoordIndexBuffer;

    //! Reseet dynamic data (vertices, normals, colors)
    void ResetDynamic()
    {
        m_vertexBuffer.reset();
        m_normalBuffer.reset();
        m_colorBuffer.reset();
    }

    //! Reset all buffers
    void Reset()
    {
        ResetDynamic();
        m_texcoordBuffer.reset();
        m_indexBuffer.reset();
        m_wireframeIndexBuffer.reset();
        m_texcoordIndexBuffer.reset();
    }

    void CreateBuffers(std::shared_ptr<VulkanMemory> memory, const Mesh<float>& mesh)
    {
        m_vertexBuffer = memory->createBufferOnDeviceAndCopy(mesh.Vertices().data(), sizeof(mesh.Vertices()(0,0)) * mesh.Vertices().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        if (mesh.HasVertexNormals()) {
            m_normalBuffer = memory->createBufferOnDeviceAndCopy(mesh.VertexNormals().data(), sizeof(mesh.VertexNormals()(0,0)) * mesh.VertexNormals().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        }
        m_indexBuffer = memory->createBufferOnDeviceAndCopy(mesh.Triangles().data(), sizeof(mesh.Triangles()(0,0)) * mesh.Triangles().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);

        if (mesh.HasTexcoords()) {
            m_texcoordBuffer = memory->createBufferOnDeviceAndCopy(mesh.Texcoords().data(), sizeof(mesh.Texcoords()(0,0)) * mesh.Texcoords().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);

            if (mesh.Triangles() != mesh.TexTriangles()) {
                m_texcoordIndexBuffer = memory->createBufferOnDeviceAndCopy(mesh.TexTriangles().data(), sizeof(mesh.TexTriangles()(0,0)) * mesh.TexTriangles().size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            }
        }
    }

    void UpdateDynamicBuffers(std::shared_ptr<VulkanMemory> memory, const Mesh<float>& mesh)
    {
        memory->updateBuffer(m_vertexBuffer.get(), mesh.Vertices().data(), sizeof(mesh.Vertices()(0,0)) * mesh.Vertices().size());
        if (mesh.HasVertexNormals() && m_normalBuffer) {
            memory->updateBuffer(m_normalBuffer.get(), mesh.VertexNormals().data(), sizeof(mesh.VertexNormals()(0,0)) * mesh.VertexNormals().size());
        }
    }
};


} // namespace nls
} //namespace epic

