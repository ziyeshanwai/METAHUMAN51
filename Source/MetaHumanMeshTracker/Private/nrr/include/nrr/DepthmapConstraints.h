// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/Camera.h>
#include <nls/geometry/VertexConstraints.h>

namespace epic::nls {

/**
 * Class to evaluate ICP depthmap correspondences (point 2 surface)
 */
class DepthmapConstraints
{
public:
    struct Options {
        float geometryWeight = 1.0f;
        float normalIncompatibilityThreshold = 0.5f;
        float minimumDistanceThresholdSquared = 4.0f;
    };

public:
    DepthmapConstraints(const Camera<float>& camera, const Eigen::Matrix<float, 4, -1>& depthAndNormals) : m_camera(camera), m_depthAndNormals(depthAndNormals) {}

    void SetVertexMask(const std::vector<int>& vertexMask) { m_vertexMask = vertexMask; }

    void SetupDepthConstraints(const Eigen::Transform<float, 3, Eigen::Affine>& rigidTransform,
                               const Eigen::Matrix<float, 3, -1>& vertices,
                               const Eigen::Matrix<float, 3, -1>& normals,
                               VertexConstraints<float, 1, 1>& vertexConstraints);

    //! Clear the dynamic distance threshold which is dynamically adapted in @see SetupDepthConstraints().
    void ClearDynamicDistanceThreshold(float initialThreshold = 1e6f);

    const Options& GetOptions() const { return m_options; }
    /* */ Options& GetOptions() /* */ { return m_options; }

private:
    const Camera<float>& m_camera;
    const Eigen::Matrix<float, 4, -1>& m_depthAndNormals;
    std::vector<int> m_vertexMask;
    bool m_calculateDynamicDistanceThreshold = true;
    float m_dynamicDistanceThreshold = 1e6f;
    Options m_options;
};

} // namespace epic::nls
