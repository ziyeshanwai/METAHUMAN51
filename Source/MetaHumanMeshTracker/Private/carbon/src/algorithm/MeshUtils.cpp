// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/algorithm/MeshUtils.h>


namespace epic {
namespace carbon {

    template<class T>
    void Triangulate(Mesh<T>& inputMesh) {

        std::vector<std::vector<size_t>> originalFaceVertexIndices = inputMesh.GetFaceVertexIndexLists();
        std::vector<std::vector<size_t>> originalFaceUvIndices = inputMesh.GetFaceUvIndexLists();
        std::vector<std::vector<size_t>> originalFaceNormalIndices = inputMesh.GetFaceNormalIndexLists();

        std::vector<Polygon<T>> tempPolygonElements;
        std::vector<std::vector<size_t>> tempFaceVertexIndices;

        std::vector<std::size_t> fvidTemp;
        std::vector<std::size_t> fuvTemp;
        std::vector<std::size_t> fniTemp;

        std::size_t faceCount = inputMesh.NumFaces();
        std::size_t addedTriangles = 0;
        std::size_t idIndex = 0;

        for (std::size_t i = 0; i < faceCount; i++) {
            // create a temporary holder of current polygon
            std::vector<size_t> currentPolygonVertexIndices = inputMesh.GetFaceVertexIndices(i);

            // create a temporary Polygon instance
            Polygon<T> currentPolygonElement = inputMesh.GetPolygonElement(i);

            // sort vertices in ccw order 
            std::vector<Vertex<T>*> unsortedVertices = currentPolygonElement.GetVertices();
            std::vector<Vertex<T>*> sortedVertices = currentPolygonElement.MakeSortedVertices();

            std::size_t vertexCount = currentPolygonElement.GetVertexCount();
            std::size_t trianglesToGenerate = vertexCount - 2;

            std::size_t v0 = 0;
            std::size_t v1 = 0;
            std::size_t v2 = 0;

            if (vertexCount > 3) {
                // create new triangles
                for (std::size_t j = 0; j < trianglesToGenerate; j++) {

                    // vertex indices
                    v0 = 0;
                    v1 = j + 1;
                    v2 = j + 2;

                    // INDEX LIST
                    std::vector<size_t> currentTriangleVertexIndices;

                    currentTriangleVertexIndices.push_back(currentPolygonVertexIndices[v0]);
                    currentTriangleVertexIndices.push_back(currentPolygonVertexIndices[v1]);
                    currentTriangleVertexIndices.push_back(currentPolygonVertexIndices[v2]);

                    tempFaceVertexIndices.push_back(currentTriangleVertexIndices);

                    // POLYGON
                    std::vector<Vertex<T>*> newTriangleVertices;

                    newTriangleVertices.push_back(sortedVertices[v0]);
                    newTriangleVertices.push_back(sortedVertices[v1]);
                    newTriangleVertices.push_back(sortedVertices[v2]);

                    Polygon<T> currentTriangle(newTriangleVertices, addedTriangles);

                    tempPolygonElements.push_back(currentTriangle);

                    // FACE-VERTEX DATA
                    std::size_t i1 = originalFaceVertexIndices[i][v0];
                    std::size_t i2 = originalFaceVertexIndices[i][v1];
                    std::size_t i3 = originalFaceVertexIndices[i][v2];

                    fvidTemp.push_back(i1);
                    fvidTemp.push_back(i2);
                    fvidTemp.push_back(i3);

                    i1 = originalFaceUvIndices[i][v0];
                    i2 = originalFaceUvIndices[i][v1];
                    i3 = originalFaceUvIndices[i][v2];

                    fuvTemp.push_back(i1);
                    fuvTemp.push_back(i2);
                    fuvTemp.push_back(i3);

                    i1 = originalFaceNormalIndices[i][v0];
                    i2 = originalFaceNormalIndices[i][v1];
                    i3 = originalFaceNormalIndices[i][v2];

                    fniTemp.push_back(i1);
                    fniTemp.push_back(i2);
                    fniTemp.push_back(i3);

                    addedTriangles++;
                }
            }
            else {
                // INDEX LIST
                std::vector<size_t> currentTriangleVertexIndices;

                std::size_t i1 = originalFaceVertexIndices[i][0];
                std::size_t i2 = originalFaceVertexIndices[i][1];
                std::size_t i3 = originalFaceVertexIndices[i][2];

                currentTriangleVertexIndices.push_back(i1);
                currentTriangleVertexIndices.push_back(i2);
                currentTriangleVertexIndices.push_back(i3);

                tempFaceVertexIndices.push_back(currentTriangleVertexIndices);

                // POLYGON
                Polygon<T> currentTriangle(unsortedVertices, addedTriangles);
                tempPolygonElements.push_back(currentTriangle);

                // FACE-VERTEX DATA
                fvidTemp.push_back(i1);
                fvidTemp.push_back(i2);
                fvidTemp.push_back(i3);

                i1 = originalFaceUvIndices[i][v0];
                i2 = originalFaceUvIndices[i][v1];
                i3 = originalFaceUvIndices[i][v2];

                fuvTemp.push_back(i1);
                fuvTemp.push_back(i2);
                fuvTemp.push_back(i3);

                i1 = originalFaceNormalIndices[i][v0];
                i2 = originalFaceNormalIndices[i][v1];
                i3 = originalFaceNormalIndices[i][v2];

                fniTemp.push_back(i1);
                fniTemp.push_back(i2);
                fniTemp.push_back(i3);

                addedTriangles++;
            }
            idIndex += vertexCount;
        }

        size_t newFaceVertexCount = addedTriangles * 3;

        inputMesh.SetFaceIndices(tempFaceVertexIndices);
        inputMesh.SetPolygonElements(tempPolygonElements);

        Eigen::VectorX<std::size_t> fvc = Eigen::VectorX<std::size_t>::Ones(addedTriangles) * 3;
        Eigen::VectorX<std::size_t> fvid(newFaceVertexCount);
        Eigen::VectorX<std::size_t> fuv(newFaceVertexCount);
        Eigen::VectorX<std::size_t> fni(newFaceVertexCount);

        for (std::size_t i = 0; i < newFaceVertexCount; i++) {
            fvid(i) = fvidTemp[i];
            fuv(i) = fuvTemp[i];
            fni(i) = fniTemp[i];
        }

        inputMesh.SetFaceVertexCounts(fvc);
        inputMesh.SetFaceVertexIds(fvid);
        inputMesh.SetFaceUvIndices(fuv);
        inputMesh.SetFaceNormalIndices(fni);

        inputMesh.GenerateMeshFaceVertexIndices();
        inputMesh.GenerateMeshFaceUvIndices();
        inputMesh.GenerateMeshFaceNormalIndices();
    }
    template void Triangulate(Mesh<float>& inputMesh);
    template void Triangulate(Mesh<double>& inputMesh);

    template<class T>
    Mesh<T> GetTriangular(const Mesh<T>& inputMesh) {

        Mesh<T> outputMesh = inputMesh;

        Triangulate<T>(outputMesh);

        return outputMesh;
    }
    template Mesh<float> GetTriangular(const Mesh<float>& inputMesh);
    template Mesh<double> GetTriangular(const Mesh<double>& inputMesh);


    template<class T>
    Eigen::MatrixX<int> ConstructTriangularMeshTopologyMatrix(const Mesh<T>& mesh) {
        std::size_t faceCount = mesh.NumFaces();

        Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> faceVertexIndices(faceCount, 3);
        // construct mesh topology data
        for (size_t i = 0; i < faceCount; i++) {
            std::vector<size_t> ids = mesh.GetFaceVertexIndices(i);
            for (int j = 0; j < 3; j++) {
                faceVertexIndices.row(i)(j) = static_cast<int>(ids[j]);
            }
        }

        return faceVertexIndices;
    }
    template Eigen::MatrixX<int> ConstructTriangularMeshTopologyMatrix(const Mesh<float>& inputMesh);
    template Eigen::MatrixX<int> ConstructTriangularMeshTopologyMatrix(const Mesh<double>& inputMesh);

    template<class T>
    Eigen::MatrixX<T> Construct3dTextureGeometryMatrix(const Mesh<T>& mesh) {
        Eigen::MatrixX<T> uvData = mesh.GetTextureVertices();
        size_t uvCoordCount = uvData.rows();

        Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor> faceVertexUV3D(uvCoordCount, 3);

        // construct uv coordinate data, fake 3rd dimension
        for (size_t i = 0; i < uvCoordCount; i++) {
            Eigen::Vector2<T> uvCoord = uvData.row(i);
            faceVertexUV3D.row(i) = Eigen::Vector3<T>(uvCoord(0), uvCoord(1), static_cast<T>(0));
        }

        return faceVertexUV3D;
    }
    template Eigen::MatrixX<float> Construct3dTextureGeometryMatrix(const Mesh<float>& inputMesh);
    template Eigen::MatrixX<double> Construct3dTextureGeometryMatrix(const Mesh<double>& inputMesh);

    template<class T>
    Eigen::MatrixX<int> ConstructAabbTextureTopologyMatrix(const Mesh<T>& mesh) {
        std::size_t faceCount = mesh.NumFaces();

        Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> faceVertexUVIndices(faceCount, 3);

        // construct uv map topology data
        for (size_t i = 0; i < faceCount; i++) {
            std::vector<size_t> uvids = mesh.GetFaceVertexUvIndices(i);
            for (size_t j = 0; j < uvids.size(); j++) {
                faceVertexUVIndices(i, j) = static_cast<int>(uvids[j]);
            }
        }

        return faceVertexUVIndices;
    }
    template Eigen::MatrixX<int> ConstructAabbTextureTopologyMatrix(const Mesh<float>& inputMesh);
    template Eigen::MatrixX<int> ConstructAabbTextureTopologyMatrix(const Mesh<double>& inputMesh);

}
}
