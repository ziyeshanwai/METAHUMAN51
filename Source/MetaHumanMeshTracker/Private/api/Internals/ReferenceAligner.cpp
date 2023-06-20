// Copyright Epic Games, Inc. All Rights Reserved.

#include "ReferenceAligner.h"
#include <nls/geometry/Procrustes.h>

using namespace epic::nls;

namespace titan::api {

ReferenceAligner::ReferenceAligner(Mesh<float> reference,
                    BarycentricCoordinates<float> fr,
                    BarycentricCoordinates<float> rr,
                    BarycentricCoordinates<float> fl,
                    BarycentricCoordinates<float> rl) : m_reference(reference), m_fr(fr), m_rr(rr), m_fl(fl), m_rl(rl) {
}

std::pair<float, epic::nls::Affine<float, 3, 3>> ReferenceAligner::EstimateScaleAndRigid(const Eigen::Matrix3Xf& vertices) const {
    Eigen::Matrix<float, 3, -1> srcPts(3, 4);
    Eigen::Matrix<float, 3, -1> targetPts(3, 4);
    targetPts.col(0) = m_fr.template Evaluate<3>(m_reference.Vertices());
    targetPts.col(1) = m_rr.template Evaluate<3>(m_reference.Vertices());
    targetPts.col(2) = m_fl.template Evaluate<3>(m_reference.Vertices());
    targetPts.col(3) = m_rl.template Evaluate<3>(m_reference.Vertices());

    srcPts.col(0) = m_fr.template Evaluate<3>(vertices);
    srcPts.col(1) = m_rr.template Evaluate<3>(vertices);
    srcPts.col(2) = m_fl.template Evaluate<3>(vertices);
    srcPts.col(3) = m_rl.template Evaluate<3>(vertices);

    return Procrustes<float, 3>::AlignRigidAndScale(srcPts, targetPts);
}
}
