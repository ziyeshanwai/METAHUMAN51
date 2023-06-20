// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/RegionBlendModel.h>

#include <algorithm>
#include <iterator>

namespace epic {
namespace nls {

template <class T>
struct RegionBlendModel<T>::Private
{
    Eigen::Matrix<T, 3, -1> archetype;
    std::vector<std::string> regionNames;
    std::map<std::string, Vector<T>> regions;

    std::vector<std::string> characterNames;
    std::map<std::string, Eigen::Matrix<T, 3, -1>> characters;

    SparseMatrixConstPtr<T> regionBlendModelMatrix;
    std::vector<std::pair<int, int>> symmetricRegions;
};

template <class T>
RegionBlendModel<T>::RegionBlendModel() : m(std::make_unique<Private>())
{
}

template <class T> RegionBlendModel<T>::~RegionBlendModel() = default;
template <class T> RegionBlendModel<T>::RegionBlendModel(RegionBlendModel&& other) = default;
template <class T> RegionBlendModel<T>& RegionBlendModel<T>::operator=(RegionBlendModel&& other) = default;

template <class T>
const std::vector<std::string>& RegionBlendModel<T>::RegionNames() const
{
    return m->regionNames;
}

template <class T>
void RegionBlendModel<T>::SetRegionNames(const std::vector<std::string>& regionNames)
{
    if (m->regionNames.size() != 0) {
        if (m->regionNames.size() != regionNames.size() ||
            !std::is_permutation(m->regionNames.begin(), m->regionNames.end(), regionNames.begin(), regionNames.end())) {
            printf("new regions in region names list, invalidating region data\n");
            m->regions.clear();
        }
    }
    m->regionNames = regionNames;
}

template <class T>
void RegionBlendModel<T>::SetRegion(const std::string& regionName, const Vector<T>& regionData)
{
    if (m->regionNames.size() == 0) {
        throw std::runtime_error("please set region names list first");
    }
    if (std::find(m->regionNames.begin(), m->regionNames.end(), regionName) == m->regionNames.end()) {
        throw std::runtime_error("region: " + regionName + " is not in the list of region names");
    }
    m->regions[regionName] = regionData;
}

template <class T>
void RegionBlendModel<T>::SetRegions(const std::map<std::string, Vector<T>>& regions)
{
    if (m->regionNames.size() == 0) {
        throw std::runtime_error("please set region names list first");
    }
    if (m->regionNames.size() != regions.size()) {
        throw std::runtime_error("region names list and region list sizes differ");
    }
    const int numVertices = static_cast<int>(regions.begin()->second.size());
    for (auto&& [regionName, regionData] : regions) {
        if (std::find(m->regionNames.begin(), m->regionNames.end(), regionName) == m->regionNames.end()) {
            throw std::runtime_error("region: " + regionName + " is not in the list of region names");
        }
        if (regionData.size() != numVertices) {
            throw std::runtime_error("inconsistent number of weights for different regions");
        }
    }
    m->regions = regions;
}

template <class T>
const std::vector<std::string>& RegionBlendModel<T>::CharacterNames() const
{
    return m->characterNames;
}

template <class T>
void RegionBlendModel<T>::SetCharacterNames(const std::vector<std::string>& charNames)
{
    if (m->characterNames.size() != 0) {
        if (m->characterNames.size() != charNames.size() ||
            !std::is_permutation(m->characterNames.begin(), m->characterNames.end(), charNames.begin(), charNames.end())) {
            printf("new characters in character names list, invalidating character data\n");
            m->characters.clear();
        }
    }
    m->characterNames = charNames;
}

template <class T>
void RegionBlendModel<T>::SetCharacter(const std::string& charName, const Eigen::Matrix<T, 3, -1>& charData)
{
    if (m->characterNames.size() == 0) {
        throw std::runtime_error("please set character names list first");
    }
    if (std::find(m->characterNames.begin(), m->characterNames.end(), charName) == m->characterNames.end()) {
        throw std::runtime_error("character: " + charName + " is not in the list of character names");
    }
    m->characters[charName] = charData;
}

template <class T>
void RegionBlendModel<T>::SetCharacters(const std::map<std::string, Eigen::Matrix<T, 3, -1>>& characters)
{
    if (m->characterNames.size() == 0) {
        throw std::runtime_error("please set character names list first");
    }
    if (m->characterNames.size() != characters.size()) {
        throw std::runtime_error("character names list and character list sizes differ");
    }
    const int numVertices = static_cast<int>(characters.begin()->second.cols());
    for (auto&& [charName, charData] : characters) {
        if (std::find(m->characterNames.begin(), m->characterNames.end(), charName) == m->characterNames.end()) {
            throw std::runtime_error("character: " + charName + " is not in the list of character names");
        }
        if (charData.cols() != numVertices) {
            throw std::runtime_error("inconsistent number of vertices for different characters");
        }
    }
    m->characters = characters;
}

template <class T>
void RegionBlendModel<T>::SetArchetype(const Eigen::Matrix<T, 3, -1>& archetype)
{
    m->archetype = archetype;
}

template <class T>
void RegionBlendModel<T>::SetSymmetricRegions(const std::vector<std::pair<std::string, std::string>>& symmetricRegions)
{
    for (auto regionPair : symmetricRegions) {
        auto firstIt = std::find(m->regionNames.begin(), m->regionNames.end(), regionPair.first);
        auto secondIt = std::find(m->regionNames.begin(), m->regionNames.end(), regionPair.second);

        if (firstIt == m->regionNames.end() || secondIt == m->regionNames.end()) {
            throw std::runtime_error("region from symmetry array does not exist in region list");
        }

        int first = static_cast<int>(std::distance(m->regionNames.begin(), firstIt));
        int second = static_cast<int>(std::distance(m->regionNames.begin(), secondIt));

        if (first == second) {
            throw std::runtime_error("region symmetric to itself in symmetry array");
        }

        if (first < second) {
            m->symmetricRegions.push_back({first, second});
        } else {
            m->symmetricRegions.push_back({second, first});
        }
    }
}

template <class T>
void RegionBlendModel<T>::Generate()
{
    if (m->regions.size() == 0) {
        throw std::runtime_error("no regions set");
    }
    if (m->characters.size() == 0) {
        throw std::runtime_error("no characters set");
    }
    if (m->archetype.cols() == 0) {
        throw std::runtime_error("no archetype set");
    }
    if (m->regions.size() != m->regionNames.size()) {
        throw std::runtime_error("region names list and region list sizes differ");
    }
    if (m->characters.size() != m->characterNames.size()) {
        throw std::runtime_error("character names list and character list sizes differ");
    }
    if (m->archetype.cols() != m->characters.begin()->second.cols()) {
        throw std::runtime_error("archetype and characters don't have the same number of vertices");
    }
    if (m->archetype.cols() != m->regions.begin()->second.size()) {
        throw std::runtime_error("invalid number of weights in region data");
    }

    const int numVertices = int(m->archetype.cols());
    const int numRegions = int(m->regions.size());
    const int numCharacters = int(m->characters.size());

    for (auto&& [regionName, regionData] : m->regions) {
        if (regionData.size() != numVertices) {
            throw std::runtime_error("inconsistent number of weights for different regions");
        }
    }
    for (auto&& [charName, charData] : m->characters) {
        if (charData.cols() != numVertices) {
            throw std::runtime_error("inconsistent number of vertices for different characters");
        }
    }

    Eigen::MatrixX<T> archetype{m->archetype};

    std::vector<Eigen::VectorX<T>> regions;
    for (auto regionName : m->regionNames) {
        regions.push_back(m->regions[regionName]);
    }

    std::vector<Eigen::MatrixX<T>> characters;
    for (auto charName : m->characterNames) {
        Eigen::MatrixX<T> vertices = m->characters[charName];
        characters.push_back(vertices);
    }

    Eigen::MatrixX<T> vertexDeltas;
    vertexDeltas.resize(3 * numVertices, numRegions * numCharacters);

    int charIndex = 0;
    for (auto charData : characters) {
        Eigen::MatrixX<T> delta = charData - archetype;

        for (int j = 0; j < numRegions; ++j) {
            vertexDeltas.block(0, charIndex * numRegions + j, 3 * numVertices, 1) << Eigen::Map<Eigen::VectorX<T>>(delta.data(), delta.size());
        }
        ++charIndex;
    }

    Eigen::MatrixX<T> weightsPerVertex;
    weightsPerVertex.resize(numVertices, numRegions);

    int regionIndex = 0;
    for (auto regionData : regions) {
        weightsPerVertex.block(0, regionIndex, numVertices, 1) << Eigen::Map<Eigen::VectorX<T>>(regionData.data(), regionData.size());
        ++regionIndex;
    }

    Eigen::MatrixX<T> regionWeights;
    regionWeights.resize(3 * numVertices, numRegions * numCharacters);

    for (int i = 0; i < numCharacters; ++i) {
        for (int j = 0; j < numVertices; ++j) {
            regionWeights.block(3 * j, i * numRegions, 1, numRegions) << weightsPerVertex.block(j, 0, 1, numRegions);
            regionWeights.block(3 * j + 1, i * numRegions, 1, numRegions) << weightsPerVertex.block(j, 0, 1, numRegions);
            regionWeights.block(3 * j + 2, i * numRegions, 1, numRegions) << weightsPerVertex.block(j, 0, 1, numRegions);
        }
    }

    Eigen::MatrixX<T> A;
    A.resize(3 * numVertices, numRegions * numCharacters);
    A = vertexDeltas.cwiseProduct(regionWeights);

    m->regionBlendModelMatrix = std::make_shared<SparseMatrix<T>>(A.sparseView());
}

template <class T>
int RegionBlendModel<T>::NumParameters() const
{
    if (m->regionBlendModelMatrix) {
        return int(m->regionBlendModelMatrix->cols());
    } else {
        return 0;
    }
}


template <class T>
int RegionBlendModel<T>::NumRegions() const
{
    return static_cast<int>(m->regions.size());
}


template <class T>
int RegionBlendModel<T>::NumVertices() const
{
    return int(m->regionBlendModelMatrix->rows() / 3);
}

template <class T>
int RegionBlendModel<T>::NumCharacters() const
{
    return static_cast<int>(m->characters.size());
}

template <class T>
Vector<T> RegionBlendModel<T>::DefaultParameters() const
{
    return Vector<T>::Zero(NumParameters());
}


template <class T>
SparseMatrixConstPtr<T> RegionBlendModel<T>::ModelMatrix() const
{
    return m->regionBlendModelMatrix;
}


template <class T>
Eigen::Matrix<T, 3, -1> RegionBlendModel<T>::Evaluate(const Vector<T>& parameters) const
{
    Eigen::Matrix<T, 3, -1> mat = m->archetype;
    Eigen::Map<Eigen::VectorX<T>>(mat.data(), mat.size()) += *(m->regionBlendModelMatrix) * parameters;
    return mat;
}


template <class T>
DiffDataMatrix<T, 3, -1> RegionBlendModel<T>::Evaluate(const DiffData<T>& parameters) const
{
    if (parameters.Size() != NumParameters()) {
        throw std::runtime_error("parameters have incorrect size");
    }

    VectorPtr<T> mat = std::make_shared<Vector<T>>(Eigen::Map<const Eigen::VectorX<T>>(m->archetype.data(), m->archetype.size()));
    *mat += *(m->regionBlendModelMatrix) * parameters.Value();

    JacobianConstPtr<T> Jacobian;
    if (parameters.HasJacobian()) {
        Jacobian = parameters.Jacobian().Premultiply(*m->regionBlendModelMatrix);
    }
    return DiffDataMatrix<T, 3, -1>(3, NumVertices(), DiffData<T>(mat, Jacobian));

}


template <class T>
DiffData<T> RegionBlendModel<T>::EvaluateRegularization(const DiffData<T>& parameters) const
{
    if (parameters.Size() != NumParameters()) {
        throw std::runtime_error("parameters have incorrect size");
    }

    // regularization is simply the L2 norm of the parameters, so we can just return the input
    return parameters;
}

template <class T>
void RegionBlendModel<T>::PrintRegionSum(const DiffData<T>& parameters) const
{
    T sumTotal = 0.0f;
    for (int i = 0; i < NumRegions(); ++i) {
        T sum = 0.0f;
        for (int j = 0; j < NumCharacters(); ++j) {
            sum += parameters.Value()[j * NumRegions() + i];
        }
        sumTotal += sum;
        printf("sum for region: %d is %f\n", i, sum);
    }
    printf("sum total is %f\n", sumTotal);
}

template <class T>
const std::vector<std::pair<int, int>>& RegionBlendModel<T>::GetSymmetricRegions() const
{
    return m->symmetricRegions;
}

// explicitly instantiate the RegionBlendModel classes
template class RegionBlendModel<float>;
template class RegionBlendModel<double>;

} // namespace nls
} //namespace epic
