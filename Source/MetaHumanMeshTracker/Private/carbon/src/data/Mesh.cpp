// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/data/Mesh.h>
#include <carbon/algorithm/MeshUtils.h>

namespace epic {
    namespace carbon {

        // Vertex
        template<class T>
        Vertex<T>::Vertex() : m_position(NULL) {
        }

        template<class T>
        Vertex<T>::Vertex(Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> position, std::size_t index) : m_position(position), m_index(index) {
        }

        template<class T>
        Vertex<T>::~Vertex() {

        }

        template<class T>
        Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> Vertex<T>::GetPosition() {
            return this->m_position;
        }

        template<class T>
        std::size_t Vertex<T>::GetIndex() {
            return this->m_index;
        }

        template<class T>
        void Vertex<T>::SetPosition(Eigen::Vector3<T>& position) {
            this->m_position = Eigen::Map<Eigen::Vector3<T>>(position.data());
        }

        // Polygon
        template<class T>
        Polygon<T>::Polygon() {

        }

        // Polygon
        template<class T>
        Polygon<T>::Polygon(std::vector<Vertex<T>*>& vertices, std::size_t index) :
            m_vertices(vertices),
            m_index(index)
        {
            this->ComputeNormal();
        }

        template<class T>
        Polygon<T>::~Polygon() {

        }

        template<class T>
        std::vector<Polygon<T>> Polygon<T>::Triangulate() {
            std::vector<Polygon<T>> triangles;

            return triangles;
        }

        // Assumes polygon vertices are sorted in a counter-clockwise order in the right hand coordinate system, which can be expected from .obj files.
        template<class T>
        void Polygon<T>::ComputeNormal() {
            std::vector<Vertex<T>*> vertices = this->GetVertices();

            Eigen::Vector3<T> A = vertices[0]->GetPosition();
            Eigen::Vector3<T> B = vertices[1]->GetPosition();
            Eigen::Vector3<T> C = vertices[2]->GetPosition();

            Eigen::Vector3<T> AB = B - A;
            Eigen::Vector3<T> AC = C - A;

            Eigen::Vector3<T> normal = AB.cross(AC).normalized();

            this->SetNormal(normal);
        }

        template<class T>
        Eigen::Map<Eigen::Vector3<T>> Polygon<T>::GetNormal() {
            return Eigen::Map<Eigen::Vector3<T>>(this->m_normal.data());
        }

        template<class T>
        void Polygon<T>::SetNormal(const Eigen::Vector3<T>& normal) {
            this->m_normal = normal;
        }

        template<class T>
        std::vector<Vertex<T>*> Polygon<T>::GetVertices() {
            return this->m_vertices;
        }

        template<class T>
        std::size_t Polygon<T>::GetIndex() {
            return this->m_index;
        }

        template<class T>
        std::size_t Polygon<T>::GetVertexCount() {
            return this->m_vertices.size();
        }

        template<class T>
        void Polygon<T>::SortVertices() {
            std::size_t vertexCount = this->m_vertices.size();

            Eigen::Vector3<T> centroid = Eigen::Vector3<T>::Zero();

            for (std::size_t i = 0; i < vertexCount; i++) {
                centroid += this->m_vertices[i]->GetPosition();
            }
            centroid /= static_cast<T>(vertexCount);

            Eigen::Vector3<T> referencePoint = this->m_vertices[0]->GetPosition();
            OrientationComparator comparator = OrientationComparator(this->m_vertices, centroid, referencePoint);

            std::sort(this->m_vertices.begin(), this->m_vertices.end(), comparator);
        }

        template<class T>
        std::vector<Vertex<T>*> Polygon<T>::MakeSortedVertices() {
            std::vector<Vertex<T>*> outputVertices = this->m_vertices;
            std::size_t vertexCount = this->m_vertices.size();

            Eigen::Vector3<T> centroid = Eigen::Vector3<T>::Zero();
            for (std::size_t i = 0; i < vertexCount; i++) {
                centroid += outputVertices[i]->GetPosition();
            }
            centroid /= static_cast<T>(vertexCount);

            Eigen::Vector3<T> referencePoint = outputVertices[0]->GetPosition();
            OrientationComparator comparator = OrientationComparator(outputVertices, centroid, referencePoint);
            std::sort(outputVertices.begin(), outputVertices.end(), comparator);

            return outputVertices;
        }

        // Ray
        template<class T>
        Ray<T>::Ray() {

        }

        template<class T>
        Ray<T>::Ray(Eigen::Vector3<T>& origin, Eigen::Vector3<T>& direction)
        {
            this->m_origin = origin;
            this->m_direction = direction;
        }

        template<class T>
        Ray<T>::~Ray() {

        }

        template<class T>
        Eigen::Vector3<T>& Ray<T>::GetOrigin() {
            return  this->m_origin;
        }

        template<class T>
        Eigen::Vector3<T>& Ray<T>::GetDirection() {
            return  this->m_direction;
        }

        // Intersection
        template<class T>
        Intersection<T>::Intersection() {

        }

        template<class T>
        Intersection<T>::Intersection(Eigen::Vector3<T>& position, std::size_t polygonIndex)
        {
            this->m_position = position;
            this->m_polygonIndex = polygonIndex;
        }

        template<class T>
        Intersection<T>::~Intersection() {

        }

        template<class T>
        Eigen::Vector3<T>& Intersection<T>::GetPosition() {
            return this->m_position;
        }

        template<class T>
        std::size_t Intersection<T>::GetPolygonIndex() {
            return this->m_polygonIndex;
        }


        template<class T>
        Eigen::Vector2<T>& Intersection<T>::GetBarycentric() {
            return this->m_barycentric;
        }

        template<class T>
        T Intersection<T>::GetDistance() {
            return this->m_distance;
        }

        template<class T>
        void Intersection<T>::SetPosition(Eigen::Vector3<T>& position) {
            this->m_position = position;
        }

        template<class T>
        void Intersection<T>::SetPolygonIndex(std::size_t polygonIndex) {
            this->m_polygonIndex = polygonIndex;
        }

        template<class T>
        void Intersection<T>::SetBarycentric(Eigen::Vector2<T>& barycentric) {
            this->m_barycentric = barycentric;
        }

        template<class T>
        void Intersection<T>::SetDistance(T distance) {
            this->m_distance = distance;
        }

        // Mesh
        template<class T>
        Mesh<T>::Mesh() {
        }

        template<class T>
        Mesh<T>::~Mesh() {
        }

        template<class T>
        Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> Mesh<T>::GetVertexPosition(size_t index) {
            static std::size_t outerStride = this->m_vertices.rows();

            return Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>>(this->m_vertices.row(index).data(), 1, 3, Eigen::Stride<1, Eigen::Dynamic>(1, outerStride));
        }

        template<class T>
        Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> Mesh<T>::GetVertexColor(size_t index) {
            static std::size_t outerStride = this->m_vertexColors.rows();

            return Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>>(this->m_vertexColors.row(index).data(), 1, 3, Eigen::Stride<1, Eigen::Dynamic>(1, outerStride));
        }

        template<class T>
        Eigen::Map<Eigen::Matrix<T, 1, 2>, 0, Eigen::Stride<1, Eigen::Dynamic>> Mesh<T>::GetUvCoord(size_t coordinateIndex, size_t setIndex) {
            static std::size_t outerStride = this->m_uvSets[setIndex].GetTextureVertices().rows();

            return Eigen::Map<Eigen::Matrix<T, 1, 2>, 0, Eigen::Stride<1, Eigen::Dynamic>>(this->m_uvSets[setIndex].GetTextureVertex(coordinateIndex).data(), 1, 2, Eigen::Stride<1, Eigen::Dynamic>(1, outerStride));
        }

        template<class T>
        Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>> Mesh<T>::GetNormal(size_t index) {
            static std::size_t outerStride = this->m_normals.rows();

            return Eigen::Map<Eigen::Matrix<T, 1, 3>, 0, Eigen::Stride<1, Eigen::Dynamic>>(this->m_normals.row(index).data(), 1, 3, Eigen::Stride<1, Eigen::Dynamic>(1, outerStride));
        }

        template<class T>
        std::vector<size_t>& Mesh<T>::GetFaceVertexIndices(size_t index) {
            return this->m_faceVertexIndices[index];
        }

        template<class T>
        const Eigen::Vector3<T> Mesh<T>::GetVertexPosition(size_t index) const {
            return this->m_vertices.row(index);
        }

        template<class T>
        const Eigen::Vector3<T> Mesh<T>::GetVertexColor(size_t index) const {
            return this->m_vertexColors.row(index);
        }

        template<class T>
        const Eigen::Vector2<T> Mesh<T>::GetUvCoord(size_t coordinateIndex, size_t setIndex) const {
            return this->m_uvSets[setIndex].GetTextureVertex(coordinateIndex);
        }

        template<class T>
        const Eigen::Vector3<T> Mesh<T>::GetNormal(size_t index) const {
            return this->m_normals.row(index);
        }

        template<class T>
        const std::vector<size_t> Mesh<T>::GetFaceVertexIndices(size_t index) const {
            return this->m_faceVertexIndices[index];
        }

        template<class T>
        const std::vector<size_t> Mesh<T>::GetFaceVertexUvIndices(size_t index) const {
            return this->m_faceVertexUvIndices[index];
        }

        template<class T>
        const std::vector<size_t> Mesh<T>::GetFaceVertexNormalIndices(size_t index) const {
            return this->m_faceVertexNormalIndices[index];
        }

        template<class T>
        std::size_t Mesh<T>::GetFaceVertexCount(size_t index) const {
            return this->m_faceVertexCount(index);
        }

        template<class T>
        std::size_t Mesh<T>::GetFaceVertexId(size_t index) const {
            return this->m_faceVertexId(index);
        }

        template<class T>
        std::size_t Mesh<T>::GetFaceUvId(size_t coordinateIndex) const {
            return this->m_faceUvIndices(coordinateIndex);
        }

        template<class T>
        std::size_t Mesh<T>::GetFaceNormalId(size_t index) const {
            return this->m_faceNormalIndices(index);
        }

        template<class T>
        Vertex<T>* Mesh<T>::GetVertexElementPtr(std::size_t index) {
            return &this->m_vertexElements[index];
        }

        template<class T>
        Polygon<T>* Mesh<T>::GetPolygonElementPtr(std::size_t index) {
            return &this->m_polygonElements[index];
        }

        template<class T>
        const Vertex<T> Mesh<T>::GetVertexElement(std::size_t index) const {
            return this->m_vertexElements[index];
        }

        template<class T>

        const Polygon<T> Mesh<T>::GetPolygonElement(std::size_t index) const {
            return this->m_polygonElements[index];
        }

        template<class T>
        void Mesh<T>::SetVertexElement(const Vertex<T>& vertexElement, std::size_t index) {
            this->m_vertexElements[index] = vertexElement;
        }

        template<class T>
        void Mesh<T>::SetPolygonElement(const Polygon<T>& polygonElement, std::size_t index) {
            this->m_polygonElements[index] = polygonElement;
        }

        template<class T>
        const std::vector<Vertex<T>>& Mesh<T>::GetVertexElements() const {
            return this->m_vertexElements;
        }

        template<class T>
        const std::vector<Polygon<T>>& Mesh<T>::GetPolygonElements() const {
            return this->m_polygonElements;
        }

        template<class T>
        void Mesh<T>::SetVertexElements(const std::vector<Vertex<T>>& vertexElements) {
            this->m_vertexElements = vertexElements;
        }

        template<class T>
        void Mesh<T>::SetPolygonElements(const std::vector<Polygon<T>>& polygonElements) {
            this->m_polygonElements = polygonElements;
        }

        template<class T>
        void Mesh<T>::SetVertexPosition(size_t index, const Eigen::Vector3<T>& vertex) {
            this->m_vertices.row(index) = vertex;
        }

        template<class T>
        void Mesh<T>::SetVertexColor(size_t index, const Eigen::Vector3<T>& color) {
            this->m_vertexColors.row(index) = color;
        }

        template<class T>
        void Mesh<T>::SetUvCoord(size_t index, const Eigen::Vector2<T>& uv, size_t setIndex) {
            this->m_uvSets[setIndex].SetTextureVertex(uv, index);
        }

        template<class T>
        void Mesh<T>::SetNormal(size_t index, const Eigen::Vector3<T>& normal) {
            this->m_normals.row(index) = normal;
        }

        template<class T>
        bool Mesh<T>::RayIntersectsTriangle(Ray<T>& ray, Polygon<T>& triangle, Intersection<T>& intersection, bool culling) {

            const T epsilon = std::numeric_limits<T>::epsilon();

            Eigen::Vector3<T> edge1, edge2, pvec, tvec, qvec;
            T det, invDet, u, v;

            std::vector<Vertex<T>*> faceVertices = triangle.GetVertices();

            Eigen::Vector3<T> vertex0 = faceVertices[0]->GetPosition();
            Eigen::Vector3<T> vertex1 = faceVertices[1]->GetPosition();
            Eigen::Vector3<T> vertex2 = faceVertices[2]->GetPosition();

            std::size_t id = triangle.GetIndex();
            if (!id) {
                return false;
            }

            Eigen::Vector3<T> rayOrigin = ray.GetOrigin();
            Eigen::Vector3<T> rayDirection = ray.GetDirection();

            if (rayDirection.norm() != 1) {
                rayDirection.normalize();
            }

            edge1 = vertex1 - vertex0;
            edge2 = vertex2 - vertex0;

            pvec = rayDirection.cross(edge2);
            det = edge1.dot(pvec);

            // culling - dont consider back-facing  polygons
            if (culling) {
                if (det < epsilon) {
                    return false;
                }
            }

            if (det > -epsilon && det < epsilon) {
                return false;    // This ray is parallel to this triangle.
            }

            invDet = static_cast<T>(1.0) / det;

            tvec = rayOrigin - vertex0;
            u = tvec.dot(pvec) * invDet;

            if (u < 0.0 || u > 1.0) {
                return false;
            }

            qvec = tvec.cross(edge1);
            v = invDet * rayDirection.dot(qvec);

            if (v < 0.0 || (u + v ) > 1.0) {
                return false;
            }

            // At this stage we can compute t to find out where the intersection point is on the line.
            T t = invDet * edge2.dot(qvec);

            if (t < epsilon) {
                return false;
            }

            Eigen::Vector3<T> intersectionCoordinates = rayOrigin + rayDirection * t;
            intersection.SetPosition(intersectionCoordinates);

            intersection.SetPolygonIndex(triangle.GetIndex());

            Eigen::Vector2<T> barycentric(u, v);
            intersection.SetBarycentric(barycentric);

            intersection.SetDistance(t);

            return true;
        }

        template<class T>
        std::vector<Intersection<T>> Mesh<T>::FindIntersections(Ray<T>& ray, bool culling) {

            std::vector<Intersection<T>> intersections;

            std::size_t faceCount = this->NumFaces();

            for (std::size_t i = 0; i < faceCount; ++i) {
                Intersection<T> possibleIntersection;

                Polygon<T> currentTriangle = this->m_polygonElements[i];

                if (RayIntersectsTriangle(ray, currentTriangle, possibleIntersection, culling)) {
                    intersections.push_back(possibleIntersection);
                }
            }

            return intersections;
        }

        template<class T>
        Intersection<T> Mesh<T>::FindClosestIntersection(Ray<T>& ray, bool culling) {

            Intersection<T> closestIntersection;

            std::vector<Intersection<T>> intersections = FindIntersections(ray, culling);

            // sort intersections
            std::size_t intersectionCount = intersections.size();
            T minDistance = std::numeric_limits<T>::max();
            
            for (std::size_t i = 0; i < intersectionCount; i++) {

                Intersection<T> currentIntersection = intersections[i];
                T currentDistance = static_cast<T>((ray.GetOrigin() - currentIntersection.GetPosition()).norm());

                if (currentDistance < minDistance) {
                    minDistance = currentDistance;
                    closestIntersection = currentIntersection;
                }
            }

            return closestIntersection;
        }

        template<class T>
        void Mesh<T>::ConstructMeshElements() {

            // map vertices
            std::size_t vertexCount = this->NumVertices();

            for (std::size_t i = 0; i < vertexCount; i++) {
                Vertex<T> currentVertex(this->GetVertexPosition(i), i);
                this->m_vertexElements.push_back(currentVertex);
            }

            // map polygons
            std::size_t faceCount = this->NumFaces();

            for (std::size_t i = 0; i < faceCount; i++) {
                //get indices
                std::vector<size_t> polygonVertexIndices = this->m_faceVertexIndices[i];
                std::size_t sideCount = polygonVertexIndices.size();
                std::vector<Vertex<T>*> currentFaceVertices;

                for (std::size_t j = 0; j < sideCount; j++) {
                    std::size_t currentIndex = polygonVertexIndices[j];
                    Vertex<T>* vptr = &this->m_vertexElements[currentIndex];
                    currentFaceVertices.push_back(vptr);
                }

                Polygon<T> currentPolygon(currentFaceVertices, i);
                this->m_polygonElements.push_back(currentPolygon);
            }
        }

        template<class T>
        Eigen::Vector3<T> Mesh<T>::CartesianToBarycentric(const Eigen::Vector3<T>& p, std::size_t polygonIndex) {

            Polygon<T>* currentPolygon = this->GetPolygonElementPtr(polygonIndex);
            std::vector<Vertex<T>*> vertices = currentPolygon->GetVertices();

            assert(vertices.size() == 3);

            Eigen::Vector3<T> a = vertices[0]->GetPosition();
            Eigen::Vector3<T> b = vertices[1]->GetPosition();
            Eigen::Vector3<T> c = vertices[2]->GetPosition();
            
            Eigen::Vector3<T> e0 = b - a;
            Eigen::Vector3<T> e1 = c - a;
            Eigen::Vector3<T> pa = p - a;

            T den = e0(0) * e1(1) - e1(0) * e0(1);
            T v = (pa(0) * e1(1) - e1(0) * pa(1)) / den;
            T w = (e0(0) * pa(1) - pa(0) * e0(1)) / den;
            T u = static_cast<T>(1.0) - w - v;

            return Eigen::Vector3<T>(u, v, w);
        }

        template<class T>
        Eigen::Vector3<T> Mesh<T>::BarycentricToCartesian(const Eigen::Vector3<T>& barycentric, std::size_t polygonIndex) {
            
            Polygon<T>* currentPolygon = this->GetPolygonElementPtr(polygonIndex);
            std::vector<Vertex<T>*> vertices = currentPolygon->GetVertices();

            assert(vertices.size() == 3);

            Eigen::Vector3<T> v1 = vertices[0]->GetPosition();
            Eigen::Vector3<T> v2 = vertices[1]->GetPosition();
            Eigen::Vector3<T> v3 = vertices[2]->GetPosition();
            
            return barycentric(0) * v1 + barycentric(1) * v2 + barycentric(2) * v3;
        }

        template<class T>
        Eigen::Vector3<T> Mesh<T>::UvToBarycentric(const Eigen::Vector2<T>& uv, std::size_t polygonIndex)
        {
            std::vector<size_t> uvids = this->GetFaceVertexUvIndices(polygonIndex);

            assert(uvids.size() == 3);

            std::size_t i0 = uvids[0];
            std::size_t i1 = uvids[1];
            std::size_t i2 = uvids[2];

            Eigen::Vector2<T> uv0 = this->GetUvCoord(i0);
            Eigen::Vector2<T> uv1 = this->GetUvCoord(i1);
            Eigen::Vector2<T> uv2 = this->GetUvCoord(i2);

            Eigen::Vector2<T> e0 = uv1 - uv0;
            Eigen::Vector2<T> e1 = uv2 - uv0;
            Eigen::Vector2<T> euv = uv - uv0;

            T den = e0(0) * e1(1) - e1(0) * e0(1);
            T v = (euv(0) * e1(1) - e1(0) * euv(1)) / den;
            T w = (e0(0) * euv(1) - euv(0) * e0(1)) / den;
            T u = static_cast<T>(1.0) - w - v;

            return Eigen::Vector3<T>(u, v, w);
        }

        template<class T>
        Eigen::Vector2<T> Mesh<T>::BarycentricToUv(const Eigen::Vector3<T>& barycentric, std::size_t polygonIndex)
        {
            std::vector<size_t> uvids = this->GetFaceVertexUvIndices(polygonIndex);

            assert(uvids.size() == 3);

            std::size_t i0 = uvids[0];
            std::size_t i1 = uvids[1];
            std::size_t i2 = uvids[2];

            Eigen::Vector2<T> uv0 = this->GetUvCoord(i0);
            Eigen::Vector2<T> uv1 = this->GetUvCoord(i1);
            Eigen::Vector2<T> uv2 = this->GetUvCoord(i2);

            return uv0 * barycentric(0) + uv1 * barycentric(1) + uv2 * barycentric(2);
        }

        template<class T>
        Eigen::Vector2<T> Mesh<T>::CartesianToUv(const Eigen::Vector3<T>& cartesian, std::size_t polygonIndex) {

            Eigen::Vector3<T> barycentric = this->CartesianToBarycentric(cartesian, polygonIndex);
            Eigen::Vector2<T> uv = this->BarycentricToUv(barycentric, polygonIndex);

            return uv;
        }

        template<class T>
        Eigen::Vector3<T> Mesh<T>::UvToCartesian(const Eigen::Vector2<T>& uv, std::size_t polygonIndex) {

            Eigen::Vector3<T> barycentric = this->UvToBarycentric(uv, polygonIndex);
            Eigen::Vector3<T> cartesian = this->BarycentricToCartesian(barycentric, polygonIndex);

            return cartesian;
        }

    template class Vertex<float>;
    template class Vertex<double>;
    template class Polygon<float>;
    template class Polygon<double>;
    template class Ray<float>;
    template class Ray<double>;
    template class Intersection<float>;
    template class Intersection<double>;
    template class Mesh<float>;
    template class Mesh<double>;
}
}
