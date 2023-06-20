// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/io/JsonIO.h>
#include <nls/serialization/EigenSerialization.h>
#include <nrr/RegionBlendModel.h>


namespace epic::nls {

template <class T>
void RegionBlendModelFromJson(const carbon::JsonElement& j, RegionBlendModel<T>& regionBlendModel) {

    Eigen::MatrixX<T> archetype;
    FromJson(j["archetype"], archetype);
    const int numVertices = int(archetype.cols());

    const carbon::JsonElement& jRegions = j["regions"];
    const int numRegions = int(jRegions.Size());

    std::vector<Eigen::VectorX<T>> regions;
    std::vector<std::string> regionNames;
    regionNames.resize(numRegions);
    int regionIdx = 0;
    for (const auto& region : jRegions.Array()) {
        if (region.Size() != 1) {
            CARBON_CRITICAL("incorrect model data format");
        }

        const std::string& regionName = region.Map().begin()->first;
        const carbon::JsonElement& regionData = region.Map().begin()->second;

        Eigen::VectorX<T> weights;
        FromJson(regionData, weights);
        if (weights.size() != numVertices) {
            LOG_INFO("weights size: {}", weights.size());
            LOG_INFO("numVertices: {}", numVertices);
            CARBON_CRITICAL("number of vertices for region {} and archetype do not match", regionName);
        }
        regions.push_back(weights);
        regionNames[regionIdx] = regionName;
        ++regionIdx;
    }

    const carbon::JsonElement& jCharacters = j["characters"];
    const int numCharacters = int(jCharacters.Size());

    std::vector<Eigen::MatrixX<T>> characters;
    std::vector<std::string> charNames;
    charNames.resize(numCharacters);
    int charIdx = 0;
    for (const auto& character : jCharacters.Array()) {
        if (character.Size() != 1) {
            CARBON_CRITICAL("incorrect model data format");
        }
        const std::string& charName = character.Map().begin()->first;
        const carbon::JsonElement& charData = character.Map().begin()->second;

        Eigen::MatrixX<T> vertices;
        FromJson(charData, vertices);
        if (vertices.cols() != numVertices) {
            LOG_INFO("vertices size: {}", int(vertices.cols()));
            LOG_INFO("numVertices: {}", numVertices);
            CARBON_CRITICAL("number of vertices for character {} and archetype do not match", charName);
        }
        characters.push_back(vertices);
        charNames[charIdx] = charName;
        ++charIdx;
    }

    const std::vector<std::pair<std::string, std::string>> regionPairs = j["symmetry"]["pairs"].Get<std::vector<std::pair<std::string, std::string>>>();

    regionBlendModel.SetRegionNames(regionNames);
    regionBlendModel.SetCharacterNames(charNames);
    regionBlendModel.SetArchetype(archetype);
    for (int i = 0; i < static_cast<int>(regionNames.size()); ++i) {
        regionBlendModel.SetRegion(regionNames[i], regions[i]);
    }
    for (int i = 0; i < static_cast<int>(charNames.size()); ++i) {
        regionBlendModel.SetCharacter(charNames[i], characters[i]);
    }
    regionBlendModel.SetSymmetricRegions(regionPairs);
    regionBlendModel.Generate();
}

} // namespace epic::nls