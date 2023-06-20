// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/data/MeshData.h>


namespace epic {
namespace carbon {
    template<class T>
    class Vertex {
    public:
        Vertex();
        Vertex(Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> position, std::size_t index);
        ~Vertex();

        Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> GetPosition();
        std::size_t GetIndex();

        void SetPosition(Eigen::Vector3<T>& position);

    private:
        Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> m_position;
        std::size_t m_index;
    };

    template<class T>
    class Polygon {
    public:
        Polygon();
        Polygon(std::vector<Vertex<T>*>& vertices, std::size_t index);
        ~Polygon();

        std::vector<Polygon<T>> Triangulate();

        void ComputeNormal();
        Eigen::Map<Eigen::Vector3<T>> GetNormal();
        void SetNormal(const Eigen::Vector3<T>& normal);
        std::vector<Vertex<T>*> GetVertices();
        std::size_t GetIndex();
        std::size_t GetVertexCount();
        void SortVertices();
        std::vector<Vertex<T>*> MakeSortedVertices();

    private:
        std::vector<Vertex<T>*> m_vertices;
        std::size_t m_index;
        Eigen::Vector3<T> m_normal;
        T m_area;
    };

    template<class T>
    class Ray {
    public:
        Ray();
        Ray(Eigen::Vector3<T>& origin, Eigen::Vector3<T>& direction);
        ~Ray();

        Eigen::Vector3<T>& GetOrigin();
        Eigen::Vector3<T>& GetDirection();

    private:
        Eigen::Vector3<T> m_origin;
        Eigen::Vector3<T> m_direction;
    };

    template<class T>
    class Intersection {
    public:
        Intersection();
        Intersection(Eigen::Vector3<T>& position, std::size_t polygonIndex);
        ~Intersection();

        Eigen::Vector3<T>& GetPosition();
        std::size_t GetPolygonIndex();
        Eigen::Vector2<T>& GetBarycentric();
        T GetDistance();

        void SetPosition(Eigen::Vector3<T>& position);
        void SetPolygonIndex(std::size_t polygonIndex);
        void SetBarycentric(Eigen::Vector2<T>& barycentric);
        void SetDistance(T distance);

    private:
        std::size_t m_polygonIndex;
        Eigen::Vector3<T> m_position;
        Eigen::Vector2<T> m_barycentric;
        T m_distance;
    };

template<class T>
class Mesh : public MeshData<T> {

    public:
        Mesh();
        ~Mesh();

        Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> GetVertexPosition(size_t index);
        Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> GetVertexColor(size_t index);
        Eigen::Map<Eigen::Matrix<T, 1, 2>, 0, Eigen::Stride<1, Eigen::Dynamic>> GetUvCoord(size_t coordinateIndex, size_t setIndex = 0);
        Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> GetNormal(size_t index);
        std::vector<size_t>& GetFaceVertexIndices(size_t index);

        const Eigen::Vector3<T> GetVertexPosition(size_t index) const;
        const Eigen::Vector3<T> GetVertexColor(size_t index) const;
        const Eigen::Vector2<T> GetUvCoord(size_t coordinateIndex, size_t setIndex = 0) const;
        const Eigen::Vector3<T> GetNormal(size_t index) const;

        const std::vector<size_t> GetFaceVertexIndices(size_t index) const;
        const std::vector<size_t> GetFaceVertexUvIndices(size_t index) const;
        const std::vector<size_t> GetFaceVertexNormalIndices(size_t index) const;

        std::size_t GetFaceVertexCount(size_t index) const;
        std::size_t GetFaceVertexId(size_t index) const;
        std::size_t GetFaceUvId(size_t coordinateIndex) const;
        std::size_t GetFaceNormalId(size_t index) const;

        Vertex<T>* GetVertexElementPtr(std::size_t index);
        Polygon<T>* GetPolygonElementPtr(std::size_t index);
        const Vertex<T> GetVertexElement(std::size_t index) const;
        const Polygon<T> GetPolygonElement(std::size_t index) const;

        void SetVertexElement(const Vertex<T>& vertexElement, std::size_t index);
        void SetPolygonElement(const Polygon<T>& polygonElement, std::size_t index);

        const std::vector<Vertex<T>>& GetVertexElements() const;
        const std::vector<Polygon<T>>& GetPolygonElements() const;

        void SetVertexElements(const std::vector<Vertex<T>>& vertexElements);
        void SetPolygonElements(const std::vector<Polygon<T>>& polygonElements);

        void SetVertexPosition(size_t index, const Eigen::Vector3<T>& vertex);
        void SetVertexColor(size_t index, const Eigen::Vector3<T>& color);
        void SetUvCoord(size_t coordinateIndex, const Eigen::Vector2<T>& uv, std::size_t setIndex = 0);
        void SetNormal(size_t index, const Eigen::Vector3<T>& normal);
    
        bool RayIntersectsTriangle(Ray<T>& ray, Polygon<T>& triangle, Intersection<T>& intersection, bool culling);
        std::vector<Intersection<T>> FindIntersections(Ray<T>& ray, bool culling);
        Intersection<T> FindClosestIntersection(Ray<T>& ray, bool culling);

        void ConstructMeshElements();

        Eigen::Vector3<T> CartesianToBarycentric(const Eigen::Vector3<T>& p, std::size_t polygonIndex);
        Eigen::Vector3<T> BarycentricToCartesian(const Eigen::Vector3<T>& barycentric, std::size_t polygonIndex);

        Eigen::Vector3<T> UvToBarycentric(const Eigen::Vector2<T>& uv, std::size_t polygonIndex);
        Eigen::Vector2<T> BarycentricToUv(const Eigen::Vector3<T>& barycentric, std::size_t polygonIndex);

        Eigen::Vector2<T> CartesianToUv(const Eigen::Vector3<T>& cartesian, std::size_t polygonIndex);
        Eigen::Vector3<T> UvToCartesian(const Eigen::Vector2<T>& uv, std::size_t polygonIndex);

    private:
        std::vector<Vertex<T>> m_vertexElements;
        std::vector<Polygon<T>> m_polygonElements;
    };
}
}
