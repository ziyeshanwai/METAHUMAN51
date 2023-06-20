// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>
#include <nls/serialization/EigenSerialization.h>
#include <nls/utils/FileIO.h>

#include <string>
#include <thread>
#include <vector>

namespace epic {
namespace nls {

/**
 * Simple helper class returning the weights per vertex, the indices of all non-zero vertices,
 * or the indices and weights of all non-zero vertices.
 */
template <class T>
class VertexWeights
{
public:
    VertexWeights() {}

    VertexWeights(int numVertices, T weightForAll)
    {
        m_weights = Eigen::VectorX<T>::Constant(numVertices, weightForAll);
        CalculateNonzeroData();
    }

    VertexWeights(const carbon::JsonElement& json, const std::string& weightsName, int numVertices)
    {
        Load(json, weightsName, numVertices);
    }

    VertexWeights(const std::string& filename, const std::string& weightsName, int numVertices)
    {
        const std::string weightsData = ReadFile(filename);
        const carbon::JsonElement json = carbon::ReadJson(weightsData);
        Load(json, weightsName, numVertices);
    }

    VertexWeights(const Eigen::VectorX<T>& weights) : m_weights(weights) { CalculateNonzeroData(); }

    int NumVertices() const { return int(m_weights.size()); }

    const Eigen::VectorX<T>& Weights() const { return m_weights; }

    const std::vector<int>& NonzeroVertices() const { return m_nonzeroVertices; }

    const std::vector<std::pair<int, T>>& NonzeroVerticesAndWeights() const{ return m_nonzeroVerticesAndWeights; }

    //! Saves the vertex weights to a json structure using name @p weightsName
    void Save(carbon::JsonElement& json, const std::string& weightsName) const {
        carbon::JsonElement arr(carbon::JsonElement::JsonType::Array);
        const std::vector<std::pair<int, T>>& nonzeroWeights = NonzeroVerticesAndWeights();
        for (auto && [vID, weight] : nonzeroWeights) {
            carbon::JsonElement innerArr(carbon::JsonElement::JsonType::Array);
            innerArr.Append(carbon::JsonElement(vID));
            innerArr.Append(carbon::JsonElement(weight));
            arr.Append(std::move(innerArr));
        }
        json.Insert(weightsName, std::move(arr));
    }

    //! Loads the vertex weights from a json dictionary with key @p weightsName
    void Load(const carbon::JsonElement& json, const std::string& weightsName, int numVertices) {
        if (json.Contains(weightsName)) {
            if (json[weightsName].Size() > 0 && json[weightsName][0].IsArray()) {
                m_weights = Eigen::VectorX<T>::Zero(numVertices);
                for (const auto& item : json[weightsName].Array()) {
                    m_weights[item[0].Get<int>()] = item[1].Get<T>();
                }
            } else {
                FromJson(json[weightsName], m_weights);
            }
            CalculateNonzeroData();
        } else {
            CARBON_CRITICAL("no vertex mask data in json with name {}", weightsName);
        }
    }

    static std::map<std::string, VertexWeights> LoadAllVertexWeights(const carbon::JsonElement& json, int numVertices)
    {
        std::map<std::string, VertexWeights> vertexWeights;
        for (const auto& [regionName, _] : json.Map()) {
            vertexWeights.emplace(regionName, VertexWeights(json, regionName, numVertices));
        }
        return vertexWeights;
    }

private:
    void CalculateNonzeroData()
    {
        m_nonzeroVertices.clear();
        m_nonzeroVerticesAndWeights.clear();
        for (int i = 0; i < NumVertices(); i++) {
            if (m_weights[i] != 0) {
                m_nonzeroVertices.push_back(i);
                m_nonzeroVerticesAndWeights.push_back({i, m_weights[i]});
            }
        }
    }

private:
    Eigen::VectorX<T> m_weights;
    std::vector<int> m_nonzeroVertices;
    std::vector<std::pair<int, T>> m_nonzeroVerticesAndWeights;
};


} // namespace nls
} //namespace epic
