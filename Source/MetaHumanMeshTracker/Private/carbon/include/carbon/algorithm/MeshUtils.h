// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once
#include <carbon/data/Mesh.h>
#include <carbon/common/EigenDenseBackwardsCompatible.h>


namespace epic {
namespace carbon {

    template<class T>
    struct OrientationComparator {

        explicit OrientationComparator(std::vector<Vertex<T>*> _vertices, Eigen::Vector3<T> _center, Eigen::Vector3<T> _referencePosition) :
            vertices(_vertices),
            center(_center),
            referencePosition(_referencePosition) {}

        bool operator() (Vertex<T>* vertex1, Vertex<T>* vertex2) {

            Eigen::Vector3<T> v1 = vertex1->GetPosition();
            Eigen::Vector3<T> v2 = vertex2->GetPosition();
            Eigen::Vector3<T> p = referencePosition.cross(center);

            Eigen::Vector3<T> u1 = v1 - center;
            Eigen::Vector3<T> u2 = v2 - center;

            T h1 = u1.dot(p);
            T h2 = u2.dot(p);

            if (h2 <= 0 && h1 > 0) {
                return false;
            }
            else if (h1 <= 0 && h2 > 0) {
                return true;
            }
            else if (h1 == 0 && h2 == 0) {
                return u1.dot(referencePosition) > 0 && u2.dot(referencePosition) < 0;
            }
            else {
                return (u1.cross(u2)).dot(center) > 0;
            }
        }
	    std::vector<Vertex<T>*> vertices;
        Eigen::Vector3<T> center;
        Eigen::Vector3<T> referencePosition;
    };

    template<class T>
    Mesh<T> GetTriangular(const Mesh<T>& inputMesh);

    template<class T>
    void Triangulate(Mesh<T>& inputMesh);


    template<class T>
    Eigen::MatrixX<int> ConstructTriangularMeshTopologyMatrix(const Mesh<T>& mesh);

    template<class T>
    Eigen::MatrixX<T> Construct3dTextureGeometryMatrix(const Mesh<T>& mesh);

    template<class T>
    Eigen::MatrixX<int> ConstructAabbTextureTopologyMatrix(const Mesh<T>& mesh);

}
}
