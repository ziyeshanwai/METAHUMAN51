// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

#include <carbon/common/Common.h>
#include <carbon/common/EigenDenseBackwardsCompatible.h>

namespace epic {
namespace carbon {

template<class T>
using Vec2m = Eigen::Map<Eigen::Matrix<T, 1, 2>, 0, Eigen::Stride<1, Eigen::Dynamic>>;

template<class T>
using Vec3m = Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>>;

template<class T>
class MeshDataBuilder;

template<class T>
class UvSet {
    friend class MeshDataBuilder<T>;

    public:
        UvSet();

        void MoveFrom(UvSet<T>& move);
        void CopyFrom(const UvSet<T>& copy);

        UvSet(UvSet<T>&& move);
        UvSet(const UvSet<T>& copy);

        ~UvSet();

        UvSet<T>& operator=(UvSet<T>&& move);
        UvSet<T>& operator=(const UvSet<T>& copy);

        UvSet<T> Clone() const;

        std::size_t NumTextureVertices() const;
        std::size_t NumTextureVertexIds() const;
        std::size_t NumTextureVertexFaceIds() const;

        Eigen::MatrixX2<T>& GetTextureVertices();
        Eigen::VectorX<std::size_t>& GetTextureVertexIds();
        Eigen::VectorX<std::size_t>& GetTextureFaceVertexIds();

        const Eigen::MatrixX2<T>& GetTextureVertices() const;
        const Eigen::VectorX<std::size_t>& GetTextureVertexIds() const;
        const Eigen::VectorX<std::size_t>& GetTextureFaceVertexIds() const;

        void SetTextureVertices(const Eigen::MatrixX2<T>& texVertices);
        void SetTextureVertexIds(const Eigen::VectorX<std::size_t>& texVertexIds);
        void SetTextureFaceVertexIds(const Eigen::VectorX<std::size_t>& texVertexFaceIds);

        Eigen::Map<Eigen::Vector2<T> > GetTextureVertex(std::size_t index);
        std::size_t GetTextureVertexId(std::size_t index);
        std::size_t GetTextureFaceVertexId(std::size_t index);

        const Eigen::Vector2<T> GetTextureVertex(std::size_t index) const;
        std::size_t GetTextureVertexId(std::size_t index) const;
        std::size_t GetTextureFaceVertexId(std::size_t index) const;

        void SetTextureVertex(const Eigen::Vector2<T>& texVertex, std::size_t index);
        void SetTextureVertexId(std::size_t texVertexId, std::size_t index);
        void SetTextureFaceVertexId(std::size_t texVertexFaceId, std::size_t index);

    private:
        Eigen::MatrixX2<T> m_texVertices;
        Eigen::VectorX<std::size_t> m_texVertexId;  // map between texture vertex and vertex
        Eigen::VectorX<std::size_t> m_texVertexFaceId;  // map telling which face does i-th texture vertex correspond to
};


template<class T  /*float or double*/>
class MeshData {
    friend class MeshDataBuilder<T>;

    public:
        MeshData();

        void MoveFrom(MeshData<T>& move);
        void CopyFrom(const MeshData<T>& copy);

        MeshData(MeshData<T>&& move);
        MeshData(const MeshData<T>& copy);

        virtual ~MeshData();

        MeshData<T>& operator=(MeshData<T>&& move);
        MeshData<T>& operator=(const MeshData<T>& copy);

        MeshData<T> Clone() const;
        bool IsEmpty() const;

        std::size_t NumVertices() const;
        std::size_t NumFaces() const;
        std::size_t NumUvSets() const;
        std::size_t NumUvSetCoords(std::size_t setIndex) const;
        std::size_t NumNormals() const;

        T* GetVertexDataPtr();
        T* GetVertexColorDataPtr();
        T* GetUvDataPtr(std::size_t setIndex = 0);
        T* GetNormalDataPtr();

        size_t* GetFaceVertexCountsDataPtr();
        size_t* GetFaceVertexIndicesDataPtr();
        size_t* GetFaceUvIndicesDataPtr();
        size_t* GetFaceNormalIndicesDataPtr();

        T* GetSingleVertexDataPtr(std::size_t index);
        T* GetSingleVertexColorDataPtr(std::size_t index);
        T* GetSingleUvDataPtr(std::size_t setIndex, std::size_t index);
        T* GetSingleNormalDataPtr(std::size_t index);

        Vec3m<T> GetVertexPosition(std::size_t index);
        Vec3m<T> GetVertexColor(std::size_t index);
        Vec3m<T> GetNormal(std::size_t index);
        Vec2m<T> GetUvCoord(std::size_t index, std::size_t setIndex = 0);

        void GenerateMeshFaceVertexIndices();
        void GenerateMeshFaceNormalIndices();
        void GenerateMeshFaceUvIndices();

        Eigen::MatrixX3<T>& GetVertexPositions();
        const Eigen::MatrixX3<T>& GetVertexPositions() const;

        Eigen::MatrixX3<T>& GetVertexColors();
        const Eigen::MatrixX3<T>& GetVertexColors() const;

        Eigen::MatrixX3<T>& GetNormals();
        const Eigen::MatrixX3<T>& GetNormals() const;

        std::vector<std::vector<size_t>>& GetFaceVertexIndexLists();
        const std::vector<std::vector<size_t>> GetFaceVertexIndexLists() const;

        std::vector<std::vector<size_t>>& GetFaceUvIndexLists();
        const std::vector<std::vector<size_t>> GetFaceUvIndexLists() const;

        std::vector<std::vector<size_t>>& GetFaceNormalIndexLists();
        const std::vector<std::vector<size_t>> GetFaceNormalIndexLists() const;

        Eigen::VectorX<std::size_t>& GetFaceVertexCountsArray();
        const Eigen::VectorX<std::size_t>& GetFaceVertexCountsArray() const;

        Eigen::VectorX<std::size_t>& GetFaceVertexIdsArray();
        const Eigen::VectorX<std::size_t>& GetFaceVertexIdsArray() const;

        Eigen::VectorX<std::size_t>& GetFaceNormalIdsArray();
        const Eigen::VectorX<std::size_t>& GetFaceNormalIdsArray() const;

        Eigen::VectorX<std::size_t>& GetFaceUvIdsArray();
        const Eigen::VectorX<std::size_t>& GetFaceUvIdsArray() const;

        Eigen::MatrixX2<T>& GetTextureVertices(std::size_t uvSet = 0);
        const Eigen::MatrixX2<T>& GetTextureVertices(std::size_t uvSet = 0) const;

        Eigen::VectorX<std::size_t>& GetTextureVertexIdsArray(std::size_t uvSet = 0);
        const Eigen::VectorX<std::size_t>& GetTextureVertexIdsArray(std::size_t uvSet = 0) const;

        Eigen::VectorX<std::size_t>& GetTextureFaceVertexIdsArray(std::size_t uvSet = 0);
        const Eigen::VectorX<std::size_t>& GetTextureFaceVertexIdsArray(std::size_t uvSet = 0) const;

        void SetVertexPositions(const Eigen::MatrixX3<T>& vertices);
        void SetColors(const Eigen::MatrixX3<T>& colors);
        void SetUvSet(size_t setIndex, const Eigen::MatrixX2<T>& uvs);
        void SetNormals(const Eigen::MatrixX3<T>& normals);
        void SetFaceNormalIndices(const Eigen::VectorX<std::size_t>& faceNormalIndices);
        void SetFaceIndices(const std::vector<std::vector<size_t>>& faceIndices);
        void SetFaceVertexCounts(const Eigen::VectorX<std::size_t>& faceVertexCounts);
        void SetFaceVertexIds(const Eigen::VectorX<std::size_t>& faceVertexIds);
        void SetTextureVertices(const Eigen::MatrixX2<T>& textureVertices,std::size_t uvSet = 0);
        void SetTextureVertexIds(const Eigen::VectorX<std::size_t>& textureVertexIds, std::size_t uvSet = 0);
        void SetTextureFaceVertexIds(const Eigen::VectorX<std::size_t>& textureFaceVertexIds, std::size_t uvSet = 0);
        void SetFaceUvIndices(const Eigen::VectorX<std::size_t>& faceUvIndices);

    protected:
        // TODO: consider hiding to enable DLL interface?
        Eigen::MatrixX3<T> m_vertices;

        Eigen::MatrixX3<T> m_vertexColors;

        // face-vertex normals
        Eigen::MatrixX3<T> m_normals;

        // vertex count per face
        Eigen::VectorX<std::size_t> m_faceVertexCount;

        // mapping of face-vertices to vertex IDs (iterate sequentially based on m_faceVertexCount)
        Eigen::VectorX<std::size_t> m_faceVertexId;

        // face uv indices
        Eigen::VectorX<std::size_t> m_faceUvIndices;

        // face normal indices
        Eigen::VectorX<std::size_t> m_faceNormalIndices;

        std::vector<UvSet<T> > m_uvSets;

        std::vector<std::vector<size_t>> m_faceVertexIndices;
        std::vector<std::vector<size_t>> m_faceVertexNormalIndices;
        std::vector<std::vector<size_t>> m_faceVertexUvIndices;
};

class MeshBuildException : public std::runtime_error {
    public:
        MeshBuildException(const char* msg);
};

template<class T>
class MeshDataBuilder {
    public:
        MeshDataBuilder();
        ~MeshDataBuilder();

        MeshDataBuilder(const MeshDataBuilder&) = delete;
        MeshDataBuilder(MeshDataBuilder&&) = delete;

        MeshDataBuilder& operator=(const MeshDataBuilder&) = delete;
        MeshDataBuilder& operator=(MeshDataBuilder&&) = delete;

        MeshDataBuilder& SetVertices(const std::size_t count, const T* data);  // unsafe
        MeshDataBuilder& SetVertices(const Eigen::MatrixX3<T>& data);
        MeshDataBuilder& SetVertices(Eigen::MatrixX3<T>&& data);

        MeshDataBuilder& SetVertexColors(std::size_t count, const T* data);
        MeshDataBuilder& SetVertexColors(const Eigen::MatrixX3<T>& data);
        MeshDataBuilder& SetVertexColors(Eigen::MatrixX3<T>&& data);

        MeshDataBuilder& SetNormals(std::size_t count, const T* data);  // unsafe
        MeshDataBuilder& SetNormals(const Eigen::MatrixX3<T>& data);
        MeshDataBuilder& SetNormals(Eigen::MatrixX3<T>&& data);

        MeshDataBuilder& SetFaces(const Eigen::VectorX<std::size_t>& faceVertexCount,
                                  const Eigen::VectorX<std::size_t>& faceVertexId);

        MeshDataBuilder& SetFaces(Eigen::VectorX<std::size_t>&& faceVertexCount, Eigen::VectorX<std::size_t>&& faceVertexId);
        MeshDataBuilder& SetFaces(std::vector<unsigned int>& faceVertexCounts,
                                  std::vector<unsigned int>& faceVertexIndices,
                                  std::vector<unsigned int>& faceUvIndices,
                                  std::vector<unsigned int>& faceNormalIndices);

        MeshDataBuilder& AddUvSet(std::size_t count, const T* data);
        MeshDataBuilder& AddUvSet(const Eigen::MatrixX2<T>& texVertices,
                                  const Eigen::VectorX<std::size_t>& texVertexIds,
                                  const Eigen::VectorX<std::size_t>& texVertexFaceIds);

        MeshDataBuilder& AddUvSet(Eigen::MatrixX2<T>&& texVertices, Eigen::VectorX<std::size_t>&& texVertexIds,
                                  Eigen::VectorX<std::size_t>&& texVertexFaceIds);


        MeshData<T>&& Build();

        void Reset();

    private:
        MeshData<T> obj;
};

}
}
