// Copyright Epic Games, Inc. All Rights Reserved.

#include "../../include/carbon/data/MeshData.h"


namespace epic {
namespace carbon {
    ////////////////////////////////////////////
    // UvSet
    ////////////////////////////////////////////

    template<class T>
    UvSet<T>::UvSet() {
    }

    template<class T>
    void UvSet<T>::MoveFrom(UvSet<T>& move) {
        this->m_texVertices = std::move(move.m_texVertices);
        this->m_texVertexId = std::move(move.m_texVertexId);
        this->m_texVertexFaceId = std::move(move.m_texVertexFaceId);
    }

    template<class T>
    void UvSet<T>::CopyFrom(const UvSet<T>& copy) {
        this->m_texVertices = copy.m_texVertices;
        this->m_texVertexId = copy.m_texVertexId;
        this->m_texVertexFaceId = copy.m_texVertexFaceId;
    }

    template<class T>
    UvSet<T>::UvSet(UvSet<T>&& move) {
        this->MoveFrom(move);
    }

    template<class T>
    UvSet<T>::UvSet(const UvSet<T>& copy) {
        this->CopyFrom(copy);
    }

    template<class T>
    UvSet<T>::~UvSet() {
    }

    template<class T>
    UvSet<T>& UvSet<T>::operator=(UvSet<T>&& move) {
        if (this != &move) {
            this->MoveFrom(move);
        }
        return *this;
    }

    template<class T>
    UvSet<T>& UvSet<T>::operator=(const UvSet<T>& copy) {
        if (this != &copy) {
            this->CopyFrom(copy);
        }
        return *this;
    }

    template<class T>
    UvSet<T> UvSet<T>::Clone() const {
        UvSet<T> clone;
        clone.m_texVertexFaceId = this->m_texVertexFaceId;
        clone.m_texVertexId = this->m_texVertexId;
        clone.m_texVertices = this->m_texVertices;
        return clone;
    }

    template<class T>
    std::size_t UvSet<T>::NumTextureVertices() const {
        return this->m_texVertices.rows();
    }

    template<class T>
    std::size_t UvSet<T>::NumTextureVertexIds() const {
        return this->m_texVertexId.rows();
    }

    template<class T>
    std::size_t UvSet<T>::NumTextureVertexFaceIds() const {
        return this->m_texVertexFaceId.rows();
    }

    template<class T>
    Eigen::MatrixX2<T>& UvSet<T>::GetTextureVertices() {
        return this->m_texVertices;
    }

    template<class T>
    Eigen::VectorX<std::size_t>& UvSet<T>::GetTextureVertexIds() {
        return this->m_texVertexId;
    }

    template<class T>
    Eigen::VectorX<std::size_t>& UvSet<T>::GetTextureFaceVertexIds() {
        return this->m_texVertexFaceId;
    }

    template<class T>
    const Eigen::MatrixX2<T>& UvSet<T>::GetTextureVertices() const {
        return this->m_texVertices;
    }

    template<class T>
    const Eigen::VectorX<std::size_t>& UvSet<T>::GetTextureVertexIds() const {
        return this->m_texVertexId;
    }

    template<class T>
    const Eigen::VectorX<std::size_t>& UvSet<T>::GetTextureFaceVertexIds() const {
        return this->m_texVertexFaceId;
    }

    template<class T>
    void UvSet<T>::SetTextureVertices(const Eigen::MatrixX2<T>& texVertices) {
        this->m_texVertices = texVertices;
    }

    template<class T>
    void UvSet<T>::SetTextureVertexIds(const Eigen::VectorX<std::size_t>& texVertexIds) {
        this->m_texVertexId = texVertexIds;
    }

    template<class T>
    void UvSet<T>::SetTextureFaceVertexIds(const Eigen::VectorX<std::size_t>& texVertexFaceIds) {
        this->m_texVertexFaceId = texVertexFaceIds;
    }

    template<class T>
    Eigen::Map<Eigen::Vector2<T>> UvSet<T>::GetTextureVertex(std::size_t index) {
        return Eigen::Map<Eigen::Vector2<T>>(this->m_texVertices.row(index).data());
    }
    template<class T>
    std::size_t UvSet<T>::GetTextureVertexId(std::size_t index) {
        return this->m_texVertexId(index);
    }
    template<class T>
    std::size_t UvSet<T>::GetTextureFaceVertexId(std::size_t index) {
        return this->m_texVertexFaceId(index);
    }

    template<class T>
    const Eigen::Vector2<T> UvSet<T>::GetTextureVertex(std::size_t index) const {
        return this->m_texVertices.row(index);
    }
    template<class T>
    std::size_t UvSet<T>::GetTextureVertexId(std::size_t index) const {
        return this->m_texVertexId(index);
    }
    template<class T>
    std::size_t UvSet<T>::GetTextureFaceVertexId(std::size_t index) const {
        return this->m_texVertexFaceId(index);
    }

    template<class T>
    void UvSet<T>::SetTextureVertex(const Eigen::Vector2<T>& texVertex, std::size_t index) {
        this->m_texVertices.row(index) = texVertex;
    }

    template<class T>
    void UvSet<T>::SetTextureVertexId(std::size_t texVertexId, std::size_t index) {
        this->m_texVertexId(index) = texVertexId;
    }

    template<class T>
    void UvSet<T>::SetTextureFaceVertexId(std::size_t texVertexFaceId, std::size_t index) {
        this->m_texVertexFaceId(index) = texVertexFaceId;
    }

////////////////////////////////////////////
// MeshData
////////////////////////////////////////////

    template<class T>
    MeshData<T>::MeshData() {
    }

    template<class T>
    void MeshData<T>::MoveFrom(MeshData<T>& move) {
        this->m_vertices = std::move(move.m_vertices);
        this->m_vertexColors = std::move(move.m_vertexColors);
        this->m_normals = std::move(move.m_normals);
        this->m_faceVertexId = std::move(move.m_faceVertexId);
        this->m_faceVertexCount = std::move(move.m_faceVertexCount);
        this->m_faceUvIndices = std::move(move.m_faceUvIndices);
        this->m_faceNormalIndices = std::move(move.m_faceNormalIndices);
        this->m_faceVertexIndices = std::move(move.m_faceVertexIndices);
        this->m_uvSets = std::move(move.m_uvSets);
        this->m_faceVertexNormalIndices = std::move(move.m_faceVertexNormalIndices);
        this->m_faceVertexUvIndices = std::move(move.m_faceVertexUvIndices);
    }

    template<class T>
    void MeshData<T>::CopyFrom(const MeshData<T>& copy) {
        this->m_vertices = copy.m_vertices;
        this->m_vertexColors = copy.m_vertexColors;
        this->m_normals = copy.m_normals;
        this->m_faceVertexId = copy.m_faceVertexId;
        this->m_faceVertexCount = copy.m_faceVertexCount;
        this->m_faceUvIndices = copy.m_faceUvIndices;
        this->m_faceNormalIndices = copy.m_faceNormalIndices;
        this->m_faceVertexIndices = copy.m_faceVertexIndices;
        this->m_uvSets = copy.m_uvSets;
        this->m_faceVertexNormalIndices = copy.m_faceVertexNormalIndices;
        this->m_faceVertexUvIndices = copy.m_faceVertexUvIndices;
    }

    template<class T>
    MeshData<T>::MeshData(MeshData<T>&& move) {
        this->MoveFrom(move);
    }

    template<class T>
    MeshData<T>::MeshData(const MeshData<T>& copy) {
        this->CopyFrom(copy);
    }

    template<class T>
    MeshData<T>::~MeshData() {
    }

    template<class T>
    MeshData<T>& MeshData<T>::operator=(MeshData<T>&& move) {
        if (&move != this) {
            this->MoveFrom(move);
        }
        return *this;
    }

    template<class T>
    MeshData<T>& MeshData<T>::operator=(const MeshData<T>& copy) {
        if (&copy != this) {
            this->CopyFrom(copy);
        }
        return *this;
    }

    template<class T>
    MeshData<T> MeshData<T>::Clone() const {
        MeshData<T> clone;

        clone.m_vertices = this->m_vertices;
        clone.m_vertexColors = this->m_vertexColors;
        clone.m_normals = this->m_normals;
        clone.m_faceVertexId = this->m_faceVertexId;
        clone.m_faceVertexCount = this->m_faceVertexCount;
        clone.m_faceUvIndices = this->m_faceUvIndices;
        clone.m_faceNormalIndices = this->m_faceNormalIndices;
        clone.m_faceVertexIndices = this->m_faceVertexIndices;
        clone.m_uvSets = this->m_uvSets;

        return clone;
    }

    template<class T>
    bool MeshData<T>::IsEmpty() const {
        return (this->NumVertices() == 0);          // if there is no vertices - there is no mesh.
    }

    template<class T>
    std::size_t MeshData<T>::NumVertices() const {
        return static_cast<std::size_t>(this->m_vertices.rows());
    }

    template<class T>
    std::size_t MeshData<T>::NumFaces() const {
        return static_cast<std::size_t>(this->m_faceVertexCount.rows());
    }

    template<class T>
    std::size_t MeshData<T>::NumUvSets() const {
        return static_cast<std::size_t>(this->m_uvSets.size());
    }

    template<class T>
    std::size_t MeshData<T>::NumUvSetCoords(size_t setIndex) const {
        return static_cast<std::size_t>(this->m_uvSets[setIndex].NumTextureVertices());
    }

    template<class T>
    std::size_t MeshData<T>::NumNormals() const {
        return static_cast<std::size_t>(this->m_normals.rows());
    }

    template<class T>
    T* MeshData<T>::GetVertexDataPtr() {
        return reinterpret_cast<T*>(m_vertices.data());
    }

    template<class T>
    T* MeshData<T>::GetVertexColorDataPtr() {
        return reinterpret_cast<T*>(m_vertexColors.data());
    }

    template<class T>
    T* MeshData<T>::GetUvDataPtr(size_t setIndex) {
        return reinterpret_cast<T*>(this->m_uvSets[setIndex].GetTextureVertices().data());
    }

    template<class T>
    T* MeshData<T>::GetNormalDataPtr() {
        return reinterpret_cast<T*>(m_normals.data());
    }

    template<class T>
    size_t* MeshData<T>::GetFaceVertexCountsDataPtr() {
        return reinterpret_cast<size_t*>(m_faceVertexCount.data());
    }

    template<class T>
    size_t* MeshData<T>::GetFaceVertexIndicesDataPtr() {
        return reinterpret_cast<size_t*>(m_faceVertexId.data());
    }

    template<class T>
    size_t* MeshData<T>::GetFaceUvIndicesDataPtr() {
        return reinterpret_cast<size_t*>(m_faceUvIndices.data());
    }

    template<class T>
    size_t* MeshData<T>::GetFaceNormalIndicesDataPtr() {
        return reinterpret_cast<size_t*>(m_faceNormalIndices.data());
    }

    template<class T>
    T* MeshData<T>::GetSingleVertexDataPtr(std::size_t index) {
        return reinterpret_cast<T*>(m_vertices.row(index).data());
    }

    template<class T>
    Vec3m<T> MeshData<T>::GetVertexPosition(std::size_t index) {
        static std::size_t outerStride = this->m_vertices.rows();

        return Vec3m<T>(this->m_vertices.row(index).data(), 1, 3, Eigen::Stride<1, Eigen::Dynamic>(1, outerStride));
    }
    
    template<class T>
    Vec3m<T> MeshData<T>::GetVertexColor(std::size_t index) {
        static std::size_t outerStride = this->m_vertexColors.rows();

        return Vec3m<T>(this->m_vertexColors.row(index).data(), 1, 3, Eigen::Stride<1, Eigen::Dynamic>(1, outerStride));
    }

    template<class T>
    Vec3m<T> MeshData<T>::GetNormal(std::size_t index) {
        static std::size_t outerStride = this->m_normals.rows();

        return Vec3m<T>(this->m_normals.row(index).data(), 1, 3, Eigen::Stride<1, Eigen::Dynamic>(1, outerStride));
    }
    
    template<class T>
    Vec2m<T> MeshData<T>::GetUvCoord(std::size_t index, std::size_t setIndex) {
        static std::size_t outerStride = this->m_uvSets[setIndex].GetTextureVertices().rows();

        return Vec2m<T>(this->m_uvSets[setIndex].GetTextureVertices().row(index).data(), 1, 2, Eigen::Stride<1, Eigen::Dynamic>(1, outerStride));
    }

    template<class T>
    T* MeshData<T>::GetSingleVertexColorDataPtr(std::size_t index) {
        return reinterpret_cast<T*>(m_vertexColors.row(index).data());
    }

    template<class T>
    T* MeshData<T>::GetSingleUvDataPtr(std::size_t setIndex, std::size_t index) {
        return reinterpret_cast<T*>(this->m_uvSets[setIndex].GetTextureVertices().row(index).data());
    }

    template<class T>
    T* MeshData<T>::GetSingleNormalDataPtr(std::size_t index) {
        return reinterpret_cast<T*>(m_normals.row(index).data());
    }

    template<class T>
    void MeshData<T>::GenerateMeshFaceVertexIndices() {
        size_t current = 0;
        size_t faceCount = NumFaces();

        this->m_faceVertexIndices.clear();
        this->m_faceVertexIndices.reserve(faceCount);

        for (size_t i = 0; i < faceCount; i++) {

            std::vector<size_t> currentFaceVertexIndices;
            size_t currentFaceVertexCount = this->m_faceVertexCount[i];
            currentFaceVertexIndices.reserve(currentFaceVertexCount);

            for (size_t j = 0; j < currentFaceVertexCount; j++) {
                size_t currentVertexId = this->m_faceVertexId[current];
                currentFaceVertexIndices.push_back(currentVertexId);

                current++;
            }

            this->m_faceVertexIndices.push_back(currentFaceVertexIndices);
        }
    }

    template<class T>
    void MeshData<T>::GenerateMeshFaceNormalIndices() {
        size_t current = 0;
        size_t faceCount = NumFaces();

        this->m_faceVertexNormalIndices.clear();
        this->m_faceVertexNormalIndices.reserve(faceCount);

        for (size_t i = 0; i < faceCount; i++) {

            std::vector<size_t> currentFaceNormalIndices;
            size_t currentFaceNormalCount = this->m_faceVertexCount[i];
            currentFaceNormalIndices.reserve(currentFaceNormalCount);

            for (size_t j = 0; j < currentFaceNormalCount; j++) {
                size_t currentNormalId = this->m_faceNormalIndices[current];
                currentFaceNormalIndices.push_back(currentNormalId);

                current++;
            }

            this->m_faceVertexNormalIndices.push_back(currentFaceNormalIndices);
        }
    }

    template<class T>
    void MeshData<T>::GenerateMeshFaceUvIndices() {
        size_t current = 0;
        size_t faceCount = NumFaces();

        this->m_faceVertexUvIndices.clear();
        this->m_faceVertexUvIndices.reserve(faceCount);

        for (size_t i = 0; i < faceCount; i++) {

            std::vector<size_t> currentFaceUvIndices;
            size_t currentFaceUvCount = this->m_faceVertexCount[i];
            currentFaceUvIndices.reserve(currentFaceUvCount);

            for (size_t j = 0; j < currentFaceUvCount; j++) {
                size_t currentUvId = this->m_faceUvIndices[current];
                currentFaceUvIndices.push_back(currentUvId);

                current++;
            }

            this->m_faceVertexUvIndices.push_back(currentFaceUvIndices);
        }
    }

    template<class T>
    Eigen::MatrixX3<T>& MeshData<T>::GetVertexPositions() {
        if (this->m_vertices.rows() == 0) {
            throw std::runtime_error("Vertices are missing.");
        }
        return this->m_vertices;
    }

    template<class T>
    Eigen::MatrixX3<T>& MeshData<T>::GetVertexColors() {
        if (this->m_vertexColors.rows() == 0) {
            throw std::runtime_error("Vertex colors are missing.");
        }
        return this->m_vertexColors;
    }

    template<class T>
    Eigen::MatrixX3<T>& MeshData<T>::GetNormals() {
        if (this->m_normals.rows() == 0) {
            throw std::runtime_error("Normals are missing.");
        }
        return this->m_normals;
    }

    template<class T>
    std::vector<std::vector<size_t>>& MeshData<T>::GetFaceVertexIndexLists() {
        return m_faceVertexIndices;
    }

    template<class T>
    std::vector<std::vector<size_t>>& MeshData<T>::GetFaceUvIndexLists() {
        return m_faceVertexUvIndices;
    }

    template<class T>
    std::vector<std::vector<size_t>>& MeshData<T>::GetFaceNormalIndexLists() {
        return m_faceVertexNormalIndices;
    }

    template<class T>
    Eigen::VectorX<std::size_t>& MeshData<T>::GetFaceVertexCountsArray() {
        if (this->m_faceVertexCount.rows() == 0) {
            throw std::runtime_error("Face-vertex counts are missing.");
        }
        return this->m_faceVertexCount;
    }

    template<class T>
    Eigen::VectorX<std::size_t>& MeshData<T>::GetFaceVertexIdsArray() {
        if (this->m_faceVertexId.rows() == 0) {
            throw std::runtime_error("Face-vertex IDs are missing.");
        }
        return this->m_faceVertexId;
    }

    template<class T>
    Eigen::VectorX<std::size_t>& MeshData<T>::GetFaceNormalIdsArray() {
        if (this->m_faceNormalIndices.rows() == 0) {
            throw std::runtime_error("Face-vertex IDs are missing.");
        }
        return this->m_faceNormalIndices;
    }

    template<class T>
    Eigen::VectorX<std::size_t>& MeshData<T>::GetFaceUvIdsArray() {
        if (this->m_faceUvIndices.rows() == 0) {
            throw std::runtime_error("Face-vertex IDs are missing.");
        }
        return this->m_faceUvIndices;
    }

    template<class T>
    Eigen::MatrixX2<T>& MeshData<T>::GetTextureVertices(std::size_t uvSet) {
        if (this->NumUvSets() < uvSet) {
            throw std::runtime_error("Requested UV set index is out of bounts.");
        }
        if (this->m_uvSets[uvSet].NumTextureVertices() == 0) {
            throw std::runtime_error("Texture vertices of the requested UV set are empty.");
        }
        return this->m_uvSets[uvSet].GetTextureVertices();
    }

    template<class T>
    Eigen::VectorX<std::size_t>& MeshData<T>::GetTextureVertexIdsArray(std::size_t uvSet) {
        if (this->NumUvSets() < uvSet) {
            throw std::runtime_error("Requested UV set index is out of bounts.");
        }
        if (this->m_uvSets[uvSet].NumTextureVertexIds() == 0) {
            throw std::runtime_error("Texture vertex IDs of the requested UV set are empty.");
        }
        return this->m_uvSets[uvSet].GetTextureVertexIds();
    }

    template<class T>
    Eigen::VectorX<std::size_t>& MeshData<T>::GetTextureFaceVertexIdsArray(std::size_t uvSet) {
        if (this->NumUvSets() < uvSet) {
            throw std::runtime_error("Requested UV set index is out of bounts.");
        }
        if (this->m_uvSets[uvSet].NumTextureVertexFaceIds() == 0) {
            throw std::runtime_error("Texture vertex face IDs of the requested UV set are empty.");
        }
        return this->m_uvSets[uvSet].GetTextureFaceVertexIds();
    }

    template<class T>
    const Eigen::MatrixX3<T>& MeshData<T>::GetVertexPositions() const {
        if (this->m_vertices.rows() == 0) {
            throw std::runtime_error("Vertices are missing.");
        }
        return this->m_vertices;
    }

    template<class T>
    const Eigen::MatrixX3<T>& MeshData<T>::GetVertexColors() const {
        if (this->m_vertexColors.rows() == 0) {
            throw std::runtime_error("Vertex colors are missing.");
        }
        return this->m_vertexColors;
    }

    template<class T>
    const Eigen::MatrixX3<T>& MeshData<T>::GetNormals() const {
        if (this->m_normals.rows() == 0) {
            throw std::runtime_error("Normals are missing.");
        }
        return this->m_normals;
    }

    template<class T>
    const std::vector<std::vector<size_t>> MeshData<T>::GetFaceVertexIndexLists() const {
        return m_faceVertexIndices;
    }

    template<class T>
    const std::vector<std::vector<size_t>> MeshData<T>::GetFaceUvIndexLists() const {
        return m_faceVertexUvIndices;
    }

    template<class T>
    const std::vector<std::vector<size_t>> MeshData<T>::GetFaceNormalIndexLists() const {
        return m_faceVertexNormalIndices;
    }

    template<class T>
    const Eigen::VectorX<std::size_t>& MeshData<T>::GetFaceVertexCountsArray() const {
        if (this->m_faceVertexCount.rows() == 0) {
            throw std::runtime_error("Face-vertex counts are missing.");
        }
        return this->m_faceVertexCount;
    }

    template<class T>
    const Eigen::VectorX<std::size_t>& MeshData<T>::GetFaceVertexIdsArray() const {
        if (this->m_faceVertexId.rows() == 0) {
            throw std::runtime_error("Face-vertex IDs are missing.");
        }
        return this->m_faceVertexId;
    }

    template<class T>
    const Eigen::VectorX<std::size_t>& MeshData<T>::GetFaceNormalIdsArray() const {
        return this->m_faceNormalIndices;
    }

    template<class T>
    const Eigen::VectorX<std::size_t>& MeshData<T>::GetFaceUvIdsArray() const {
        return this->m_faceUvIndices;
    }

    template<class T>
    const Eigen::MatrixX2<T>& MeshData<T>::GetTextureVertices(std::size_t uvSet) const {
        if (this->NumUvSets() < uvSet) {
            throw std::runtime_error("Requested UV set index is out of bounts.");
        }
        if (this->m_uvSets[uvSet].NumTextureVertices() == 0) {
            throw std::runtime_error("Texture vertices of the requested UV set are empty.");
        }
        return this->m_uvSets[uvSet].GetTextureVertices();
    }

    template<class T>
    const Eigen::VectorX<std::size_t>& MeshData<T>::GetTextureVertexIdsArray(std::size_t uvSet) const {
        if (this->NumUvSets() < uvSet) {
            throw std::runtime_error("Requested UV set index is out of bounts.");
        }
        if (this->m_uvSets[uvSet].NumTextureVertexIds() == 0) {
            throw std::runtime_error("Texture vertex IDs of the requested UV set are empty.");
        }
        return this->m_uvSets[uvSet].GetTextureVertexIds();
    }

    template<class T>
    const Eigen::VectorX<std::size_t>& MeshData<T>::GetTextureFaceVertexIdsArray(std::size_t uvSet) const {
        if (this->NumUvSets() < uvSet) {
            throw std::runtime_error("Requested UV set index is out of bounts.");
        }
        if (this->m_uvSets[uvSet].NumTextureVertexFaceIds() == 0) {
            throw std::runtime_error("Texture vertex face IDs of the requested UV set are empty.");
        }
        return this->m_uvSets[uvSet].GetTextureFaceVertexIds();
    }

    template<class T>
    void MeshData<T>::SetVertexPositions(const Eigen::MatrixX3<T>& vertices) {
        this->m_vertices = vertices;
    }

    template<class T>
    void MeshData<T>::SetColors(const Eigen::MatrixX3<T>& colors) {
        this->m_vertexColors = colors;
    }

    template<class T>
    void MeshData<T>::SetUvSet(size_t setIndex, const Eigen::MatrixX2<T>& uvs) {
        this->m_uvSets[setIndex].SetTextureVertices(uvs);
    }

    template<class T>
    void MeshData<T>::SetNormals(const Eigen::MatrixX3<T>& normals) {
        this->m_normals = normals;
    }

    template<class T>
    void MeshData<T>::SetFaceNormalIndices(const Eigen::VectorX<std::size_t>& faceNormalIndices) {
        this->m_faceNormalIndices = faceNormalIndices;
    }

    template<class T>
    void MeshData<T>::SetFaceIndices(const std::vector<std::vector<size_t>>& faceIndices) {
        this->m_faceVertexIndices = faceIndices;
    }

    template<class T>
    void MeshData<T>::SetFaceVertexCounts(const Eigen::VectorX<std::size_t>& faceVertexCounts) {
        this->m_faceVertexCount = faceVertexCounts;
    }

    template<class T>
    void MeshData<T>::SetFaceVertexIds(const Eigen::VectorX<std::size_t>& faceVertexIds) {
        this->m_faceVertexId = faceVertexIds;
    }

    template<class T>
    void MeshData<T>::SetTextureVertices(const Eigen::MatrixX2<T>& textureVertices, std::size_t uvSet) {
        if (this->m_uvSets.size() == 0) {

            UvSet<T> set;

            this->m_uvSets.push_back(std::forward<UvSet<T>>(set));
        }
        this->m_uvSets[uvSet].SetTextureVertices(textureVertices);
    }

    template<class T>
    void MeshData<T>::SetTextureVertexIds(const Eigen::VectorX<std::size_t>& textureVertexIds, std::size_t uvSet) {
        if (this->m_uvSets.size() == 0) {

            UvSet<T> set;

            this->m_uvSets.push_back(std::forward<UvSet<T>>(set));
        }
        this->m_uvSets[uvSet].SetTextureVertexIds(textureVertexIds);
    }

    template<class T>
    void MeshData<T>::SetTextureFaceVertexIds(const Eigen::VectorX<std::size_t>& textureFaceVertexIds, std::size_t uvSet) {
        if(this->m_uvSets.size() == 0){

            UvSet<T> set;

            this->m_uvSets.push_back(std::forward<UvSet<T>>(set));
        }
        this->m_uvSets[uvSet].SetTextureFaceVertexIds(textureFaceVertexIds);
    }

    template<class T>
    void MeshData<T>::SetFaceUvIndices(const Eigen::VectorX<std::size_t>& faceUvIndices) {
        this->m_faceUvIndices = faceUvIndices;
    }


    ////////////////////////////////////////////
    // Exceptions
    ////////////////////////////////////////////

    MeshBuildException::MeshBuildException(const char* msg) : std::runtime_error(msg) {
    }


    ////////////////////////////////////////////
    // MeshDataBuilder
    ////////////////////////////////////////////

    template<class T>
    MeshDataBuilder<T>::MeshDataBuilder() {
    }

    template<class T>
    MeshDataBuilder<T>::~MeshDataBuilder() {
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetVertices(const std::size_t count, const T* data) {
        if (count == 0) {
            throw MeshBuildException("Given vertex count is 0.");
        }
        if (data == nullptr) {
            throw MeshBuildException("Given vertex data is null.");
        }

        this->obj.m_vertices = Eigen::Matrix<T, Eigen::Dynamic, 3>::Map(data, count, 3, Eigen::Stride<1, 3>());

        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetVertices(const Eigen::MatrixX3<T>& data) {
        this->obj.m_vertices = data;
        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetVertices(Eigen::MatrixX3<T>&& data) {
        this->obj.m_vertices = std::move(data);
        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetVertexColors(std::size_t count, const T* data) {
        if (count != static_cast<size_t>(this->obj.m_vertices.rows())) {
            throw MeshBuildException("Invalid vertex color count - should be the same as vertex count.");
        }

        this->obj.m_vertexColors = Eigen::Matrix<T, Eigen::Dynamic, 3>::Map(data, count, 3, Eigen::Stride<1, 3>());

        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetVertexColors(const Eigen::MatrixX3<T>& data) {
        if (data.rows() != this->obj.m_vertices.rows()) {
            throw MeshBuildException("Invalid vertex color count - should be the same as vertex count.");
        }
        this->obj.m_vertexColors = data;
        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetVertexColors(Eigen::MatrixX3<T>&& data) {
        if (data.rows() != this->obj.m_vertices.rows()) {
            throw MeshBuildException("Invalid vertex color count - should be the same as vertex count.");
        }
        this->obj.m_vertexColors = std::move(data);
        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetNormals(std::size_t count, const T* data) {
        if (!count) {
            throw MeshBuildException("Invalid normal count - given 0 normals.");
        }

        this->obj.m_normals = Eigen::Matrix<T, Eigen::Dynamic, 3>::Map(data, count, 3, Eigen::Stride<1, 3>());

        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetNormals(const Eigen::MatrixX3<T>& data) {
        this->obj.m_normals = data;
        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetNormals(Eigen::MatrixX3<T>&& data) {
        this->obj.m_normals = std::move(data);
        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetFaces(const Eigen::VectorX<std::size_t>& faceVertexCount,
                                                     const Eigen::VectorX<std::size_t>& faceVertexId) {
        this->obj.m_faceVertexCount = faceVertexCount;
        this->obj.m_faceVertexId = faceVertexId;
        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetFaces(Eigen::VectorX<std::size_t>&& faceVertexCount,
                                                     Eigen::VectorX<std::size_t>&& faceVertexId) {
        this->obj.m_faceVertexCount = std::move(faceVertexCount);
        this->obj.m_faceVertexId = std::move(faceVertexId);
        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::SetFaces(std::vector<unsigned int>& faceVertexCounts,
                                                     std::vector<unsigned int>& faceVertexIndices,
                                                     std::vector<unsigned int>& faceUvIndices,
                                                     std::vector<unsigned int>& faceNormalIndices) {

        this->obj.m_faceVertexCount.resize(faceVertexCounts.size());
        std::copy(faceVertexCounts.begin(), faceVertexCounts.end(), this->obj.m_faceVertexCount.data());

        this->obj.m_faceVertexId.resize(faceVertexIndices.size());
        std::copy(faceVertexIndices.begin(), faceVertexIndices.end(), this->obj.m_faceVertexId.data());

        this->obj.m_faceUvIndices.resize(faceUvIndices.size());
        std::copy(faceUvIndices.begin(), faceUvIndices.end(), this->obj.m_faceUvIndices.data());

        this->obj.m_faceNormalIndices.resize(faceNormalIndices.size());
        std::copy(faceNormalIndices.begin(), faceNormalIndices.end(), this->obj.m_faceNormalIndices.data());

        return *this;
    }


    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::AddUvSet(std::size_t count, const T* data) {
        UvSet<T> set;

        set.m_texVertices = Eigen::Matrix<T, Eigen::Dynamic, 2>::Map(data, count, 2, Eigen::Stride<1, 2>());

        this->obj.m_uvSets.push_back(std::forward<UvSet<T> >(set));

        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::AddUvSet(const Eigen::MatrixX2<T>& texVertices,
                                                     const Eigen::VectorX<std::size_t>& texVertexIds,
                                                     const Eigen::VectorX<std::size_t>& texVertexFaceIds) {
        UvSet<T> set;
        set.m_texVertices = texVertices;
        set.m_texVertexId = texVertexIds;
        set.m_texVertexFaceId = texVertexFaceIds;
        this->obj.m_uvSets.push_back(std::forward<UvSet<T> >(set));
        return *this;
    }

    template<class T>
    MeshDataBuilder<T>& MeshDataBuilder<T>::AddUvSet(Eigen::MatrixX2<T>&& texVertices,
                                                     Eigen::VectorX<std::size_t>&& texVertexIds,
                                                     Eigen::VectorX<std::size_t>&& texVertexFaceIds) {
        UvSet<T> set;
        set.m_texVertices = std::move(texVertices);
        set.m_texVertexId = std::move(texVertexIds);
        set.m_texVertexFaceId = std::move(texVertexFaceIds);
        this->obj.m_uvSets.push_back(std::forward<UvSet<T> >(set));
        return *this;
    }

    template<class T>
    MeshData<T> && MeshDataBuilder<T>::Build() {
        return std::move(this->obj);
    }

    template<class T>
    void MeshDataBuilder<T>::Reset() {
        this->obj = MeshData<T>();
    }

    ////////////////////////////////////////////
    // Template instantiations for float and double
    ////////////////////////////////////////////

    template class UvSet<float>;
    template class UvSet<double>;
    template class MeshData<float>;
    template class MeshData<double>;
    template class MeshDataBuilder<float>;
    template class MeshDataBuilder<double>;

}
}
