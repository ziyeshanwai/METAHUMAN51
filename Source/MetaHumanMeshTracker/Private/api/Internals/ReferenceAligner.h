// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/BarycentricCoordinates.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/Affine.h>

namespace titan {
namespace api {

class ReferenceAligner {
public:
    ReferenceAligner(epic::nls::Mesh<float> reference,
                     epic::nls::BarycentricCoordinates<float> fr,
                     epic::nls::BarycentricCoordinates<float> rr,
                     epic::nls::BarycentricCoordinates<float> fl,
                     epic::nls::BarycentricCoordinates<float> rl);

    ~ReferenceAligner() = default;

    std::pair<float, epic::nls::Affine<float, 3, 3>> EstimateScaleAndRigid(const Eigen::Matrix3Xf& vertices) const;
private:
    epic::nls::Mesh<float> m_reference;
    epic::nls::BarycentricCoordinates<float> m_fr;
    epic::nls::BarycentricCoordinates<float> m_rr;
    epic::nls::BarycentricCoordinates<float> m_fl;
    epic::nls::BarycentricCoordinates<float> m_rl;
};

}
}
