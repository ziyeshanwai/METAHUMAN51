// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/Mesh.h>

#include <set>

/**
 * Creates a sphere by subdividing a diamond shape base shape and projecting the subdivision verices to the unit sphere.
 */
template <class T>
static epic::nls::Mesh<T> CreateSphere(int subdivLevels)
{
    Eigen::Matrix<int, 3, -1> tris(3, 8);
    tris.col(0) = Eigen::Vector3i(0, 1, 4);
    tris.col(1) = Eigen::Vector3i(1, 2, 4);
    tris.col(2) = Eigen::Vector3i(2, 3, 4);
    tris.col(3) = Eigen::Vector3i(3, 0, 4);
    tris.col(4) = Eigen::Vector3i(0, 5, 1);
    tris.col(5) = Eigen::Vector3i(1, 5, 2);
    tris.col(6) = Eigen::Vector3i(2, 5, 3);
    tris.col(7) = Eigen::Vector3i(3, 5, 0);
    Eigen::Matrix<T, 3, -1> vertices(3, 6);
    vertices.col(0) = Eigen::Vector3<T>(0, 0, 1);
    vertices.col(1) = Eigen::Vector3<T>(1, 0, 0);
    vertices.col(2) = Eigen::Vector3<T>(0, 0, -1);
    vertices.col(3) = Eigen::Vector3<T>(-1, 0, 0);
    vertices.col(4) = Eigen::Vector3<T>(0, 1, 0);
    vertices.col(5) = Eigen::Vector3<T>(0, -1, 0);

    for (int i = 0; i < subdivLevels; i++) {
        std::set<std::pair<int,int>> edges;
        auto makeEdge = [](int vID0, int vID1) {
            if (vID0 < vID1) {
                return std::pair<int,int>(vID0, vID1);
            } else {
                return std::pair<int,int>(vID1, vID0);
            }
        };
        for (int j = 0; j < int(tris.cols()); j++) {
            edges.insert(makeEdge(tris(0, j), tris(1, j)));
            edges.insert(makeEdge(tris(1, j), tris(2, j)));
            edges.insert(makeEdge(tris(2, j), tris(0, j)));
        }
        std::map<std::pair<int,int>, int> edgesToNewVIDs;
        int newVerticesSize = int(vertices.cols());
        for (const std::pair<int,int>& edge : edges) {
            edgesToNewVIDs[edge] = newVerticesSize++;
        }
        Eigen::Matrix<T, 3, -1> newVertices(3, newVerticesSize);
        newVertices.leftCols(vertices.cols()) = vertices;
        for (const std::pair<int,int>& edge : edges) {
            int index = edgesToNewVIDs[edge];
            newVertices.col(index) = (vertices.col(edge.first) + vertices.col(edge.second)).normalized();
        }
        vertices.swap(newVertices);

        Eigen::Matrix<int, 3, -1> newTris(3, tris.cols() * 4);
        for (int j = 0; j < int(tris.cols()); j++) {
            const std::pair<int,int> e0 = makeEdge(tris(0, j), tris(1, j));
            const std::pair<int,int> e1 = makeEdge(tris(1, j), tris(2, j));
            const std::pair<int,int> e2 = makeEdge(tris(2, j), tris(0, j));
            newTris.col(4 * j + 0) = Eigen::Vector3i(tris(0,j), edgesToNewVIDs[e0], edgesToNewVIDs[e2]);
            newTris.col(4 * j + 1) = Eigen::Vector3i(edgesToNewVIDs[e0], tris(1,j) , edgesToNewVIDs[e1]);
            newTris.col(4 * j + 2) = Eigen::Vector3i(edgesToNewVIDs[e2], edgesToNewVIDs[e0], edgesToNewVIDs[e1]);
            newTris.col(4 * j + 3) = Eigen::Vector3i(edgesToNewVIDs[e2], edgesToNewVIDs[e1], tris(2,j));
        }
        tris.swap(newTris);
    }

    epic::nls::Mesh<T> mesh;
    mesh.SetTriangles(tris);
    mesh.SetVertices(vertices);
    return mesh;
}
