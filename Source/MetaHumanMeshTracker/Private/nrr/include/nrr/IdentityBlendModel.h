// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <carbon/io/JsonIO.h>
#include <nls/Context.h>
#include <nls/DiffDataMatrix.h>
#include <nls/VectorVariable.h>

#include <string>
#include <vector>

namespace epic::nls {

/**
 * IdentityBlendModel models the geometry of a face as
 * Vertices = MeanShape + sum_regions (B_region x_region)
 * where B_region is the linear model for a region, and x_region are the coefficients.
 */
template <class T>
class IdentityBlendModel
{
public:
    struct RegionData {
        std::string regionName;
        Eigen::Matrix<T, -1, -1> modes;
        Eigen::VectorXi vertexIDs;
        Eigen::VectorX<T> weights;
        std::vector<std::string> modeNames;
    };

public:
    IdentityBlendModel();
    ~IdentityBlendModel();
    IdentityBlendModel(IdentityBlendModel&& other);
    IdentityBlendModel(const IdentityBlendModel& other) = delete;
    IdentityBlendModel& operator=(IdentityBlendModel&& other);
    IdentityBlendModel& operator=(const IdentityBlendModel& other) = delete;

    //! Set the model data
    void SetModel(const Eigen::Matrix<T, 3, -1>& mean, const std::vector<RegionData>& regionsData);

    /**
     * Loads the identity blend model from a json file of the following format:
     *
     * format of identity_model.json:
     * {
     *   'mean': [3xN matrix of the model]
     *   'regions': {
     *      'region name': {
     *          'vertex_ids': [list of vertex indices],
     *          'weights': [weights per vertex],
     *          'modes': (3 * num vertexIDs in region, num modes), // the modes still need to be multiplied by the weights
    *           'mode names': [mode 1, mode 2, ...] (optional)
     *       }, ...
     *    }
     * }
     *
     * Note that the weights for a region are also integrated into the PCA model of each region, so there is
     * no need to apply the weights to the model.
     */
    void LoadModel(const std::string& identityModelFile);
    void LoadModel(const epic::carbon::JsonElement& identityJson);

    void SaveModel(const std::string& identityModelFile) const;
    epic::carbon::JsonElement SaveModel() const;

    //! @returns the number of parameters of the model
    int NumParameters() const;

    //! @returns the number of regions in the model
    int NumRegions() const;

    //! @returns the number of vertices in the model
    int NumVertices() const;

    //! @returns default parameters resulting in an average face
    Vector<T> DefaultParameters() const;

    //! @returns the base shape
    const Eigen::Matrix<T, 3, -1> Base() const;

    //! @returns the model matrix
    SparseMatrixConstPtr<T> ModelMatrix() const;

    //! @returns the evaluated model for the parameters
    Eigen::Matrix<T, 3, -1> Evaluate(const Vector<T>& parameters) const;

    //! @returns the evaluated model for the parameters
    DiffDataMatrix<T, 3, -1> Evaluate(const DiffData<T>& parameters) const;

    //! @returns evaluates the regularization on the parameters
    DiffData<T> EvaluateRegularization(const DiffData<T>& parameters) const;

    //! @returns the name of the region at index @p regionIndex in the model.
    const std::string& RegionName(int regionIndex) const;

    //! @returns the name of the modes for a certain region.
    const std::vector<std::string>& ModeNames(int regionIndex) const;

    //! @returns for each region the indeces [startIndex, endIndex) in the parameter array.
    const std::vector<std::pair<int, int>>& RegionRanges() const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
