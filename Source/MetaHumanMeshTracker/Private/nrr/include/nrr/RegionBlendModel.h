// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/Context.h>
#include <nls/DiffDataMatrix.h>
#include <nls/VectorVariable.h>

#include <string>
#include <vector>

namespace epic::nls {

/**
 * RegionBlendModel models the geometry of a face as
 * Vertices = MeanShape + A * x
 * where A is the matrix of weighted deltas between
 * characters and archetype per character per region,
 * and x are the mixing parameters.
 */
template <class T>
class RegionBlendModel
{
public:
    RegionBlendModel();
    ~RegionBlendModel();
    RegionBlendModel(RegionBlendModel&& other);
    RegionBlendModel(const RegionBlendModel& other) = delete;
    RegionBlendModel& operator=(RegionBlendModel&& other);
    RegionBlendModel& operator=(const RegionBlendModel& other) = delete;

    const std::vector<std::string>& RegionNames() const;
    void SetRegionNames(const std::vector<std::string>& regionNames);
    void SetRegion(const std::string& regionName, const Vector<T>& regionData);
    void SetRegions(const std::map<std::string, Vector<T>>& regions);

    const std::vector<std::string>& CharacterNames() const;
    void SetCharacterNames(const std::vector<std::string>& charNames);
    void SetCharacter(const std::string& charName, const Eigen::Matrix<T, 3, -1>& charData);
    void SetCharacters(const std::map<std::string, Eigen::Matrix<T, 3, -1>>& characters);

    void SetArchetype(const Eigen::Matrix<T, 3, -1>& archetype);

    void SetSymmetricRegions(const std::vector<std::pair<std::string, std::string>>& symmetricRegions);

    void Generate();

    //! @returns the number of parameters of the model
    int NumParameters() const;

    //! @returns the number of regions in the model
    int NumRegions() const;

    //! @returns the number of vertices in the model
    int NumVertices() const;

    //! @returns the number of characters in the model
    int NumCharacters() const;

    //! @returns default parameters resulting in the archetype
    Vector<T> DefaultParameters() const;

    //! @returns the model matrix
    SparseMatrixConstPtr<T> ModelMatrix() const;

    //! @returns the evaluated model for the parameters
    Eigen::Matrix<T, 3, -1> Evaluate(const Vector<T>& parameters) const;

    //! @returns the evaluated model for the parameters
    DiffDataMatrix<T, 3, -1> Evaluate(const DiffData<T>& parameters) const;

    //! @returns evaluates the regularization on the parameters
    DiffData<T> EvaluateRegularization(const DiffData<T>& parameters) const;

    void PrintRegionSum(const DiffData<T>& parameters) const;
    const std::vector<std::pair<int, int>>& GetSymmetricRegions() const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
