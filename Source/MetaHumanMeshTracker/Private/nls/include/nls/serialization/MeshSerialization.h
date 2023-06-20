// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/io/JsonIO.h>
#include <nls/geometry/Mesh.h>

#include <nls/serialization/EigenSerialization.h>

#include <fstream>
#include <sstream>
#include <string>

namespace epic {
namespace nls {


/**
 * Serializes a mesh into a json dictionary {
 *  "vertices" : 3xN matrix,
 *  "topology" : {
 *      "tris" : 3xM matrix
 *      “quads” : 4xK matrix
 *
 *      or
 *
 *      "polygons" : [3, 4, 4, 3, 3, ...]     // number of vertices per polygon
 *      "vtxIDs" : [0, 1, 2, 1, 3, 4, 2, ...] // the vertex ids for the polygons, len(vtxIDs) == sum(polygonCount)
 *  }
 *  or
 * }
 */
template <class T>
void MeshToJson(carbon::JsonElement& j, const Mesh<T>& mesh, bool exportAsPolygons) {

    if (mesh.NumVertices() > 0) {
        j.Insert("vertices", ToJson2(mesh.Vertices()));
        carbon::JsonElement jTopology(carbon::JsonElement::JsonType::Object);

        if (exportAsPolygons) {
            const int totalPolygons = mesh.NumTriangles() + mesh.NumQuads();
            if (totalPolygons > 0) {
                Eigen::Matrix<int, -1, 1> polygons(totalPolygons);
                Eigen::Matrix<int, -1, 1> vtxIDs(mesh.NumTriangles() * 3 + mesh.NumQuads() * 4);
                int polygonCounter = 0;
                int vtxIDcounter = 0;
                for (int i = 0; i < mesh.NumTriangles(); i++) {
                    polygons[polygonCounter++] = 3;
                    for (int k = 0; k < 3; k++) {
                        vtxIDs[vtxIDcounter++] = mesh.Triangles()(k, i);
                    }
                }
                for (int i = 0; i < mesh.NumQuads(); i++) {
                    polygons[polygonCounter++] = 4;
                    for (int k = 0; k < 4; k++) {
                        vtxIDs[vtxIDcounter++] = mesh.Quads()(k, i);
                    }
                }
                CARBON_ASSERT(polygonCounter == totalPolygons, "polycounter needs to have iterated through all polygons");
                CARBON_ASSERT(vtxIDcounter == mesh.NumTriangles() * 3 + mesh.NumQuads() * 4, "every polygon needs to have its own unique vertices");
                jTopology.Insert("polygons", ToJson2(polygons));
                jTopology.Insert("vtxIDs", ToJson2(vtxIDs));
            }
        } else {
            if (mesh.NumTriangles() > 0) {
                jTopology.Insert("tris", ToJson2(mesh.Triangles()));
            }
            if (mesh.NumQuads() > 0) {
                jTopology.Insert("quads", ToJson2(mesh.Quads()));
            }
        }
        j.Insert("topology", std::move(jTopology));
    }

}


/**
 * Deserializes a mesh from a json dictionary.
 * @see MeshToJson
 */
template <class T>
void MeshFromJson(const carbon::JsonElement& j, Mesh<T>& mesh)
{
    if (j.Contains("vertices")) {
        Eigen::Matrix<T, 3, -1> vertices;
        FromJson(j["vertices"], vertices);
        mesh.SetVertices(vertices);
    }

    if (j.Contains("topology")) {
        if (j["topology"].Contains("polygons") && j["topology"].Contains("vtxIDs")) {
            Eigen::Matrix<int, -1, 1> polygons;
            Eigen::Matrix<int, -1, 1> vtxIDs;
            FromJson(j["topology"]["polygons"], polygons);
            FromJson(j["topology"]["vtxIDs"], vtxIDs);
            int numTris = 0;
            int numQuads = 0;
            for (int i = 0; i < int(polygons.size()); i++) {
                if (polygons[i] == 3) {
                    numTris++;
                } else if (polygons[i] == 4) {
                    numQuads++;
                } else {
                    CARBON_CRITICAL("Mesh only supports triangles and quads.");
                }
            }
            Eigen::Matrix<int, 3, -1> tris(3, numTris);
            Eigen::Matrix<int, 4, -1> quads(4, numQuads);
            int vtxIDcounter = 0;
            int triIndex = 0;
            int quadIndex = 0;
            for (int i = 0; i < int(polygons.size()); i++) {
                if (polygons[i] == 3) {
                    for (int k = 0; k < 3; k++) {
                        tris(k, triIndex) = vtxIDs[vtxIDcounter++];
                    }
                    triIndex++;
                } else if (polygons[i] == 4) {
                    for (int k = 0; k < 4; k++) {
                        quads(k, quadIndex) = vtxIDs[vtxIDcounter++];
                    }
                    quadIndex++;
                }
            }
            mesh.SetTriangles(tris);
            mesh.SetQuads(quads);
        } else {
            if (j["topology"].Contains("tris")) {
                Eigen::Matrix<int, 3, -1> tris;
                FromJson(j["topology"]["tris"], tris);
                mesh.SetTriangles(tris);
            }
            if (j["topology"].Contains("quads")) {
                Eigen::Matrix<int, 4, -1> quads;
                FromJson(j["topology"]["quads"], quads);
                mesh.SetQuads(quads);
            }
        }
    }
}


}
}
