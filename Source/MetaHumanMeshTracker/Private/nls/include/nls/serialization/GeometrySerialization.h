// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/io/JsonIO.h>

#include <nls/serialization/EigenSerialization.h>

namespace epic {
namespace nls {

/**
 * Serializes vertices to a json dictionary {
 *  “vertices” : 3xN matrix
 * }
 */
template <class T>
void GeometryToJson(carbon::JsonElement& j, const Eigen::Matrix<T,3,Eigen::Dynamic>& vertices)
{
        j.Insert("vertices", ToJson2(vertices));
}

/**
 * Deserializes vertices from a json dictionary {
 *  “vertices” : 3xN matrix
 * }
 */
template <class T>
void GeometryFromJson(const carbon::JsonElement& j, Eigen::Matrix<T,3,Eigen::Dynamic>& vertices)
{
    FromJson(j["vertices"], vertices);
}

/**
 * Deserializes multiple geometries
 * {
 *    “geometry” : {
 *       “name of geometry” : {
 *          “vertices” : 3xN matrix
 *       },
 *       ...
 *    }
 * }
 */
template <class T>
void MultiGeometryFromJson(const carbon::JsonElement& j, std::map<std::string, Eigen::Matrix<T,3,Eigen::Dynamic>>& geometryMap)
{
    geometryMap.clear();
    for (const auto& [geometryName, verticesDict] : j["geometry"].Map()) {
        Eigen::Matrix<T,3,Eigen::Dynamic> vertices;
        GeometryFromJson(verticesDict, vertices);
        geometryMap[geometryName] = vertices;
    }
}

}
}
