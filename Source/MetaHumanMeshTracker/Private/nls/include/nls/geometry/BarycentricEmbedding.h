// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>
#include <nls/geometry/BarycentricCoordinates.h>
#include <nls/geometry/TetMesh.h>

#include <vector>
#include <string>

namespace epic {
namespace nls {

/**
 * Class representing barycentric embedding of a list of points w.r.t. a tetmesh
 * Note: the points do not have to be inside the tetmesh; if outside, negative barycentrics will be used
 * Note 2: the current method to compute barycentric embedding is slow, hence serialization and deserialization is supported
 */
template <class T>
class BarycentricEmbedding
{
public:
    BarycentricEmbedding() = default;
    BarycentricEmbedding(const BarycentricEmbedding& o) = default;
    BarycentricEmbedding& operator=(const BarycentricEmbedding& o) = default;

    const std::vector<BarycentricCoordinates<T, 4>>& GetBarycentricCoordinates() const {
        return m_barycentricCoordinates;
    }

    //! Computes and sets embedding of 'points' in a 'tetMesh':
    void SetBarycentricEmbedding(const Eigen::Matrix<T, 3, -1>& points, const TetMesh<T>& tetMesh, bool printProgress = false);

    //! Saves m_barycentricCoordinates to a file:
    void Serialize(std::string fname) const;

    //! Sets m_barycentricCoordinates loaded from a file:
    void Deserialize(std::string fname);

    //! Checks if all barycentric coordinates are affine (i.e. sum to 1; they may be negative but always must sum to 1):
    bool allAffine() const {
        for (auto& bc : m_barycentricCoordinates) {
            if (fabs(bc.Weights().sum() - T(1.0)) > 1e-3) return false;
        }
        return true;
    }

    T minimalBarycentricCoordinate() const {
        if (!m_barycentricCoordinates.size()) CARBON_CRITICAL("m_barycentricCoordinates empty");
        T minc = m_barycentricCoordinates[0].Weights().minCoeff();
        for (size_t i = 1; i < m_barycentricCoordinates.size(); i++)
            minc = std::min(minc, m_barycentricCoordinates[i].Weights().minCoeff());
        return minc;
    }

private:

    std::vector<BarycentricCoordinates<T, 4> > m_barycentricCoordinates;
};

} // namespace nls
} //namespace epic
