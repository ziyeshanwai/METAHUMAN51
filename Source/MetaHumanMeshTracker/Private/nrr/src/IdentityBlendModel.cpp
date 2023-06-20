// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/IdentityBlendModel.h>

#include <carbon/io/Utils.h>
#include <carbon/io/JsonIO.h>
#include <nls/math/Math.h>
#include <nls/geometry/Mesh.h>
#include <nls/serialization/EigenSerialization.h>
#include <nls/utils/FileIO.h>

#include <carbon/io/JsonIO.h>

namespace epic {
namespace nls {

template <class T>
struct IdentityBlendModel<T>::Private
{
    Eigen::Matrix<T, 3, -1> base;
    std::vector<RegionData> regionsData;
    SparseMatrixConstPtr<T> identityBlendModelMatrix;
    std::vector<std::pair<int, int>> regionRanges;
};


template <class T>
IdentityBlendModel<T>::IdentityBlendModel() : m(std::make_unique<Private>())
{
}

template <class T> IdentityBlendModel<T>::~IdentityBlendModel() = default;
template <class T> IdentityBlendModel<T>::IdentityBlendModel(IdentityBlendModel&&) = default;
template <class T> IdentityBlendModel<T>& IdentityBlendModel<T>::operator=(IdentityBlendModel&&) = default;

template <class T>
void IdentityBlendModel<T>::SetModel(const Eigen::Matrix<T, 3, -1>& mean, const std::vector<RegionData>& regionsData)
{
    m->base = mean;
    m->regionsData = regionsData;

    const int numVertices = int(mean.cols());

    std::vector<std::pair<int, int>> regionRanges;
    std::vector<Eigen::Triplet<T>> triplets;
    int totalModes = 0;
    for (auto& regionData : m->regionsData) {
        const int numRegionModes = int(regionData.modes.cols());

        if (regionData.vertexIDs.size() != regionData.modes.rows() / 3) {
            CARBON_CRITICAL("vertex_ids and modes matrix for region {} do not match", regionData.regionName);
        }
        if (regionData.weights.size() != regionData.modes.rows() / 3) {
            CARBON_CRITICAL("weights and modes matrix for region {} do not match", regionData.regionName);
        }

        for (int i = 0; i < int(regionData.vertexIDs.size()); i++) {
            const int vID = regionData.vertexIDs[i];
            for (int j = 0; j < numRegionModes; j++) {
                for (int k = 0; k < 3; k++) {
                    triplets.push_back(Eigen::Triplet<T>(3 * vID + k, totalModes + j, regionData.weights[i] * regionData.modes(3 * i + k, j)));
                }
            }
        }

        regionRanges.push_back({totalModes, totalModes + numRegionModes});
        totalModes += numRegionModes;

        // verify that we have names for each mode
        if (int(regionData.modeNames.size()) != numRegionModes) {
            regionData.modeNames.clear();
            for (int k = 0; k < numRegionModes; ++k) {
                regionData.modeNames.push_back("unknown " + std::to_string(k));
            }
        }
    }
    m->regionRanges = regionRanges;

    SparseMatrixPtr<T> identityBlendModelMatrix = std::make_shared<SparseMatrix<T>>(3 * numVertices, totalModes);
    identityBlendModelMatrix->setFromTriplets(triplets.begin(), triplets.end());
    m->identityBlendModelMatrix = identityBlendModelMatrix;
}

template <class T>
void IdentityBlendModel<T>::LoadModel(const std::string& identityModelFile)
{
    LoadModel(carbon::ReadJson(ReadFile(identityModelFile)));
}

template <class T>
void IdentityBlendModel<T>::LoadModel(const epic::carbon::JsonElement& identityJson)
{
    Eigen::Matrix<T, 3, -1> mean;
    FromJson(identityJson["mean"], mean);

    std::vector<RegionData> regionsData;
    const carbon::JsonElement& jRegions = identityJson["regions"];
    for (auto&& [regionName, regionJsonData] : jRegions.Map()) {
        RegionData regionData;
        regionData.regionName = regionName;

        FromJson(regionJsonData["vertex_ids"], regionData.vertexIDs);
        FromJson(regionJsonData["weights"], regionData.weights);
        FromJson(regionJsonData["modes"], regionData.modes);

        if (regionJsonData.Contains("mode names") && regionJsonData["mode names"].IsArray()) {
            regionData.modeNames = regionJsonData["mode names"].template Get<std::vector<std::string>>();
        }

        regionsData.emplace_back(std::move(regionData));
    }

    SetModel(mean, regionsData);
}

template <class T>
epic::carbon::JsonElement IdentityBlendModel<T>::SaveModel() const
{
    using namespace epic::carbon;

    JsonElement regionsJson(JsonElement::JsonType::Object);

    for (const auto& regionData : m->regionsData) {
        JsonElement regionJson(JsonElement::JsonType::Object);
        regionJson.Insert("modes", ToJson2(regionData.modes));
        regionJson.Insert("vertex_ids", ToJson2(regionData.vertexIDs));
        regionJson.Insert("weights", ToJson2(regionData.weights));
        regionJson.Insert("mode names", JsonElement(regionData.modeNames));
        regionsJson.Insert(regionData.regionName, std::move(regionJson));
    }

    JsonElement json(JsonElement::JsonType::Object);
    json.Insert("mean", ToJson2(m->base));
    json.Insert("regions", std::move(regionsJson));

    return json;
}

template <class T>
void IdentityBlendModel<T>::SaveModel(const std::string& identityModelFile) const
{
    epic::carbon::WriteFile(identityModelFile, epic::carbon::WriteJson(SaveModel(), -1));
}

template <class T>
int IdentityBlendModel<T>::NumParameters() const
{
    if (m->identityBlendModelMatrix) {
        return int(m->identityBlendModelMatrix->cols());
    } else {
        return 0;
    }
}


template <class T>
int IdentityBlendModel<T>::NumRegions() const
{
    return int(m->regionsData.size());
}


template <class T>
int IdentityBlendModel<T>::NumVertices() const
{
    return int(m->identityBlendModelMatrix->rows() / 3);
}


template <class T>
Vector<T> IdentityBlendModel<T>::DefaultParameters() const
{
    return Vector<T>::Zero(NumParameters());
}

template <class T>
const Eigen::Matrix<T, 3, -1> IdentityBlendModel<T>::Base() const
{
    return m->base;
}

template <class T>
SparseMatrixConstPtr<T> IdentityBlendModel<T>::ModelMatrix() const
{
    return m->identityBlendModelMatrix;
}


template <class T>
Eigen::Matrix<T, 3, -1> IdentityBlendModel<T>::Evaluate(const Vector<T>& parameters) const
{
    Eigen::Matrix<T, 3, -1> mat = m->base;
    Eigen::Map<Eigen::VectorX<T>>(mat.data(), mat.size()) += *(m->identityBlendModelMatrix) * parameters;
    return mat;
}


template <class T>
DiffDataMatrix<T, 3, -1> IdentityBlendModel<T>::Evaluate(const DiffData<T>& parameters) const
{
    if (parameters.Size() != NumParameters()) {
        throw std::runtime_error("parameters have incorrect size");
    }

    VectorPtr<T> mat = std::make_shared<Vector<T>>(Eigen::Map<const Eigen::VectorX<T>>(m->base.data(), m->base.size()));
    *mat += *(m->identityBlendModelMatrix) * parameters.Value();

    JacobianConstPtr<T> Jacobian;
    if (parameters.HasJacobian()) {
        Jacobian = parameters.Jacobian().Premultiply(*m->identityBlendModelMatrix);
    }
    return DiffDataMatrix<T, 3, -1>(3, NumVertices(), DiffData<T>(mat, Jacobian));

}


template <class T>
DiffData<T> IdentityBlendModel<T>::EvaluateRegularization(const DiffData<T>& parameters) const
{
    if (parameters.Size() != NumParameters()) {
        CARBON_CRITICAL("parameters have incorrect size");
    }

    // regularization is simply the L2 norm of the parameters, so we can just return the input
    return parameters;
}


template <class T>
const std::string& IdentityBlendModel<T>::RegionName(int regionIndex) const
{
    return m->regionsData[regionIndex].regionName;
}


template <class T>
const std::vector<std::string>& IdentityBlendModel<T>::ModeNames(int regionIndex) const
{
    return m->regionsData[regionIndex].modeNames;
}


template <class T>
const std::vector<std::pair<int, int>>& IdentityBlendModel<T>::RegionRanges() const
{
    return m->regionRanges;
}

// explicitly instantiate the IdentityBlendModel classes
template class IdentityBlendModel<float>;
template class IdentityBlendModel<double>;

} // namespace nls
} //namespace epic
