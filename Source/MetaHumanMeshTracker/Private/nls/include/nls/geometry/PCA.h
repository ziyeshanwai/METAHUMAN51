// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic::nls
{

/**
 * @brief Calculates PCA on the mean centered data matrix @p meanCenteredDataMatrix and returns
 * the modes that keep @p varianceToKeep of the variance.
 * @p meanCenteredDataMatrix  Rows are the data samples, columns the dimensions of the data.
 */
template <class T>
Eigen::Matrix<T, -1, -1> CreatePCAWithMeanCenteredData(
    const Eigen::Matrix<T, -1, -1>& meanCenteredDataMatrix,
    const T varianceToKeep,
    const int maxModes = 0
)
{
    const Eigen::JacobiSVD<Eigen::Matrix<T,-1,-1>> svd(meanCenteredDataMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::Vector<T, -1> stds = svd.singularValues() / sqrt(T(meanCenteredDataMatrix.rows() - 1));
    Eigen::Matrix<T, -1, -1> modes = svd.matrixV();
    for (int i = 0; i < stds.size(); i++) {
        modes.col(i) *= stds[i];
    }

    Eigen::VectorX<T> accumulatedVariance = Eigen::VectorX<T>::Zero(stds.size());
    T totalVariance = 0;
    for (int i = 0; i < int(stds.size()); i++) {
        const T variance = stds[i] * stds[i];
        totalVariance += variance;
        accumulatedVariance[i] = totalVariance;
    }
    int finalIndex = 0;
    for (int i = 0; i < int(stds.size()); i++) {
        const T variance = stds[i] * stds[i];
        if (accumulatedVariance[i] / totalVariance <= varianceToKeep) {
            finalIndex = i;
        }
        LOG_INFO("std/variance {}: {}/{}/{} => {} ({})", i, stds[i], modes.col(i).maxCoeff(), variance, variance/totalVariance, accumulatedVariance[i]/totalVariance);
    }
    int numModes = finalIndex + 1;
    if (maxModes > 0 && numModes > maxModes) {
        LOG_INFO("restricting pca to {} modes, and {} variance.", maxModes, accumulatedVariance[maxModes - 1]/totalVariance);
        numModes = maxModes;
    } else {
        LOG_INFO("use {} out {} modes", numModes + 1, int(stds.size()));
    }

    return modes.leftCols(numModes);
}


/**
 * @brief Calculates PCA on data matrix @p dataMatrix and returns
 * the modes that keep @p varianceToKeep of the variance.
 * @p dataMatrix  Rows are the data samples, columns the dimensions of the data.
 */
template <class T>
std::pair<Eigen::VectorX<T>, Eigen::Matrix<T, -1, -1>> CreatePCA(
    const Eigen::Matrix<T, -1, -1>& dataMatrix,
    const T varianceToKeep,
    const int maxModes = 0
)
{
    const Eigen::VectorX<T> mean = dataMatrix.colwise().mean();
    const Eigen::Matrix<T, -1, -1> meanCenteredDataMatrix = dataMatrix.rowwise() - mean.transpose();
    return {mean, CreatePCAWithMeanCenteredData<T>(meanCenteredDataMatrix, varianceToKeep, maxModes)};
}


/**
 * @brief Applies PCA for meshes @p meshes and the region as defined by \p vertexWeights.
 *
 * @param meshes              The meshes on which PCA is calculated.
 * @param vertexWeights       The vertices for which PCA is calculated together with a weight per vertex.
 * @param varianceToKeep      The variance to keep for PCA.
 * @param premultiplyWeight   Whether the vertex weights are multiplied before calculating PCA or after calculating PCA for the region.
 * @param maxModes            The maximum number of modes to keep. If <= 0 then it will base it based on @p varianceToKeep
 * @return std::pair<Eigen::VectorX<T>, Eigen::Matrix<T, -1, -1>>  Returns mean and modes of PCA for just the region. The modes are multiplied by the standard deviation.
 */
template <class T>
std::pair<Eigen::VectorX<T>, Eigen::Matrix<T, -1, -1>> CreatePCA(
    const std::vector<Eigen::Matrix<T, 3, -1>>& meshes,
    const std::vector<std::pair<int, T>>& vertexWeights,
    const T varianceToKeep,
    const bool premultiplyWeight, //!< flag whether to first apply the weights and then calculate PCA, or first calculate PCA, and then apply the weights
    const int maxModes = 0
)
{
    const int numVerticesInRegion = int(vertexWeights.size());
    const int numShapes = int(meshes.size());
    Eigen::Matrix<T, -1, -1> regionShapeMatrix = Eigen::Matrix<T, -1, -1>::Zero(numShapes, 3 * numVerticesInRegion);
    for (int j = 0; j < int(vertexWeights.size()); j++) {
        const int vertexIndex = vertexWeights[j].first;
        const T weight = vertexWeights[j].second;
        for (int i = 0; i < numShapes; i++) {
            for (int k = 0; k < 3; k++) {
                regionShapeMatrix(i, 3 * j + k) = (meshes[i](k, vertexIndex));
            }
        }
    }
    const Eigen::VectorX<T> mean = regionShapeMatrix.colwise().mean().transpose();
    regionShapeMatrix.rowwise() -= mean.transpose();

    for (int j = 0; j < int(vertexWeights.size()); j++) {
        const T weight = vertexWeights[j].second;
        for (int i = 0; i < numShapes; i++) {
            for (int k = 0; k < 3; k++) {
                if (premultiplyWeight) {
                    regionShapeMatrix(i, 3 * j + k) *= weight;
                }
            }
        }
    }

    const Eigen::Matrix<T, -1, -1> modes = CreatePCAWithMeanCenteredData(regionShapeMatrix, varianceToKeep, maxModes);
    return {mean, modes};
}

} // namespace epic::nls
