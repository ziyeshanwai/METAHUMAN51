// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/DepthmapConstraints.h>
#include <carbon/simd/Simd.h>
#include <carbon/utils/Profiler.h>
#include <carbon/utils/Timer.h>

namespace epic::nls {

void DepthmapConstraints::ClearDynamicDistanceThreshold(float initialThreshold)
{
    m_calculateDynamicDistanceThreshold = true;
    m_dynamicDistanceThreshold = initialThreshold;
}

// void DepthmapConstraints::SetupDepthConstraints(const Eigen::Transform<float, 3, Eigen::Affine>& rigidTransform,
//                                                 const Eigen::Matrix<float, 3, -1>& vertices,
//                                                 const Eigen::Matrix<float, 3, -1>& normals,
//                                                 VertexConstraints<float, 1, 1>& vertexConstraints)
// {
//     PROFILING_FUNCTION(PROFILING_COLOR_CYAN);

//     const float normalIncompatibilityMultiplier = 1.0f / std::max<float>(1e-6f, (1.0f - m_options.normalIncompatibilityThreshold));

//     // optimize in camera space to minimize computation
//     const Eigen::Matrix<float, 3, 3> K = m_camera.Intrinsics();
//     Eigen::Matrix<float, 3, 3> modK = K;
//     // unprojection uses origin in center of top left pixel so that we can use the integer coordinates directly
//     modK(0, 2) -= 0.5f;
//     modK(1, 2) -= 0.5f;
//     const Eigen::Matrix<float, 3, 3> invK = modK.inverse();

//     const Eigen::Transform<float, 3, Eigen::Affine> totalTransform = Eigen::Transform<float, 3, Eigen::Affine>(m_camera.Extrinsics().Matrix()) * rigidTransform;
//     const Eigen::Transform<float, 3, Eigen::Affine> totalTransformInv = totalTransform.inverse();

//     vertexConstraints.ResizeToFitAdditionalConstraints(m_vertexMask.size());

//     for (int iter = (m_calculateDynamicDistanceThreshold ? 0 : 1); iter < 2; ++iter) {
//         float totalWeight = 0;
//         float totalDistance = 0;
//         const float distanceMultiplier = 1.0f / m_dynamicDistanceThreshold;

//         for (const int vID : m_vertexMask) {
//             const Eigen::Vector3f vertex = totalTransform * vertices.col(vID);
//             const Eigen::Vector3f normal = totalTransform.linear() * normals.col(vID);
//             const Eigen::Vector3f pix = K * vertex;

//             // nearest neighbor lookup
//             const int x = int(pix[0] / pix[2]);
//             const int y = int(pix[1] / pix[2]);
//             if (x >= 0 && x < m_camera.Width() && y >= 0 && y < m_camera.Height()) {
//                 const int depthAndNormalIndex = y * m_camera.Width() + x;
//                 const Eigen::Vector4f depthAndNormal = m_depthAndNormals.col(depthAndNormalIndex);
//                 if (depthAndNormal[0] > 0) {
//                     const Eigen::Vector3f targetVertex = invK * Eigen::Vector3f(float(x), float(y), 1.0f) * depthAndNormal[0];
//                     const Eigen::Vector3f targetNormal = depthAndNormal.tail(3);
//                     const float normalCompatibilityWeight = std::max<float>(0, (normal.dot(targetNormal) -  m_options.normalIncompatibilityThreshold) * normalIncompatibilityMultiplier);
//                     const float squaredDistance = (targetVertex - vertex).squaredNorm();
//                     const float weight = m_options.geometryWeight * normalCompatibilityWeight * std::max<float>(0, distanceMultiplier * (m_dynamicDistanceThreshold - squaredDistance));
//                     totalDistance += weight * squaredDistance;
//                     totalWeight += weight;
//                     if (weight > 0 && !m_calculateDynamicDistanceThreshold) {
//                         const Eigen::Matrix<float, 1, 3> drdV = (targetNormal.transpose() * totalTransform.linear()) * weight;
//                         const float residual = weight * targetNormal.dot(vertex - targetVertex);
//                         vertexConstraints.AddConstraint(vID, residual, std::move(drdV));
//                     }
//                 }
//             }
//         }

//         m_dynamicDistanceThreshold = std::max<float>(m_options.minimumDistanceThresholdSquared, 2.0f * totalDistance / totalWeight);
//         m_calculateDynamicDistanceThreshold = false;
//     }
// }

void DepthmapConstraints::SetupDepthConstraints(const Eigen::Transform<float, 3, Eigen::Affine>& rigidTransform,
                                                const Eigen::Matrix<float, 3, -1>& vertices,
                                                const Eigen::Matrix<float, 3, -1>& normals,
                                                VertexConstraints<float, 1, 1>& vertexConstraints)
{
    if (m_options.geometryWeight <= 0) return;

    PROFILING_FUNCTION(PROFILING_COLOR_CYAN);

    const float normalIncompatibilityMultiplier = 1.0f / std::max<float>(1e-6f, (1.0f - m_options.normalIncompatibilityThreshold));

    // optimize in camera space to minimize computation
    const Eigen::Matrix<float, 3, 3> K = m_camera.Intrinsics();
    Eigen::Matrix<float, 3, 3> modK = K;
    // unprojection uses origin in center of top left pixel so that we can use the integer coordinates directly
    modK(0, 2) -= 0.5f;
    modK(1, 2) -= 0.5f;
    const Eigen::Matrix<float, 3, 3> invK = modK.inverse();

    const Eigen::Transform<float, 3, Eigen::Affine> totalTransform = Eigen::Transform<float, 3, Eigen::Affine>(m_camera.Extrinsics().Matrix()) * rigidTransform;
    // const Eigen::Transform<float, 3, Eigen::Affine> totalTransformInv = totalTransform.inverse();

    const int numPts = int(m_vertexMask.size());
    vertexConstraints.ResizeToFitAdditionalConstraints(numPts);

     // pad to simd size
    const int numPaddedPts = SimdType::Pad(numPts);
    const int numSimdElements = numPaddedPts / SimdType::Size();

    // get only required vertices
    std::vector<SimdType> sampledPtsX(numSimdElements);
    std::vector<SimdType> sampledPtsY(numSimdElements);
    std::vector<SimdType> sampledPtsZ(numSimdElements);
    std::vector<SimdType> sampledNormalsX(numSimdElements);
    std::vector<SimdType> sampledNormalsY(numSimdElements);
    std::vector<SimdType> sampledNormalsZ(numSimdElements);
    std::vector<SimdType> residuals(numSimdElements);
    std::vector<SimdType> drdVxs(numSimdElements);
    std::vector<SimdType> drdVys(numSimdElements);
    std::vector<SimdType> drdVzs(numSimdElements);
    std::vector<SimdType> weights(numSimdElements);

    {
        alignas(SimdType::Alignment()) float ptX[SimdType::Size()];
        alignas(SimdType::Alignment()) float ptY[SimdType::Size()];
        alignas(SimdType::Alignment()) float ptZ[SimdType::Size()];
        alignas(SimdType::Alignment()) float nX[SimdType::Size()];
        alignas(SimdType::Alignment()) float nY[SimdType::Size()];
        alignas(SimdType::Alignment()) float nZ[SimdType::Size()];
        for (int simdIndex = 0; simdIndex < numSimdElements; ++simdIndex) {
            for (int k = 0; k < SimdType::Size(); ++k) {
                const int vID = simdIndex * SimdType::Size() + k;
                if (vID < numPts) {
                    const auto& pt = vertices.col(m_vertexMask[vID]);
                    const auto& normal = normals.col(m_vertexMask[vID]);

                    ptX[k] = pt[0];
                    ptY[k] = pt[1];
                    ptZ[k] = pt[2];
                    nX[k] = normal[0];
                    nY[k] = normal[1];
                    nZ[k] = normal[2];
                } else {
                    ptX[k] = 0;
                    ptY[k] = 0;
                    ptZ[k] = 0;
                    nX[k] = 0;
                    nY[k] = 0;
                    nZ[k] = 0;
                }
            }
            sampledPtsX[simdIndex].LoadAligned(ptX);
            sampledPtsY[simdIndex].LoadAligned(ptY);
            sampledPtsZ[simdIndex].LoadAligned(ptZ);
            sampledNormalsX[simdIndex].LoadAligned(nX);
            sampledNormalsY[simdIndex].LoadAligned(nY);
            sampledNormalsZ[simdIndex].LoadAligned(nZ);
        }
    }

    Eigen::Matrix<SimdType, 3, 3> mK;
    for (int k = 0; k < 3; ++k) for (int j = 0; j < 3; ++j) mK(j, k).Set(K(j, k));
    Eigen::Matrix<SimdType, 3, 3> mInvK;
    for (int k = 0; k < 3; ++k) for (int j = 0; j < 3; ++j) mInvK(j, k).Set(invK(j, k));
    const SimdTypei mCameraWidth(m_camera.Width());
    const SimdTypei mCameraHeight(m_camera.Height());
    SimdType mNormalIncompatibilityThreshold;
    mNormalIncompatibilityThreshold.Set(m_options.normalIncompatibilityThreshold);
    SimdType mNormalIncompatibilityMultiplier;
    mNormalIncompatibilityMultiplier.Set(normalIncompatibilityMultiplier);
    SimdType mGeometryWeight;
    mGeometryWeight.Set(m_options.geometryWeight);


    // LOG_INFO("depth simd 1: {}", timer.Current()); timer.Restart();

    for (int iter = (m_calculateDynamicDistanceThreshold ? 0 : 1); iter < 2; ++iter) {

        SimdType mDistanceThreshold;
        mDistanceThreshold.Set(m_dynamicDistanceThreshold);
        SimdType mDistanceMultiplier;
        mDistanceMultiplier.Set(1.0f / m_dynamicDistanceThreshold);

        const Eigen::Matrix<float, 3, 3> rot = totalTransform.linear();
        SimdType mRot[3][3];
        for (int k = 0; k < 3; ++k) for (int j = 0; j < 3; ++j) mRot[k][j].Set(rot(k, j));
        SimdType mT[3]{};
        mT[0].Set(totalTransform.translation()[0]);
        mT[1].Set(totalTransform.translation()[1]);
        mT[2].Set(totalTransform.translation()[2]);

        SimdType mTotalWeight;
        mTotalWeight.SetZero();
        SimdType mTotalDistance;
        mTotalDistance.SetZero();

        for (int vID4 = 0; vID4 < numSimdElements; ++vID4) {
            // const Eigen::Vector3f vertex = transform * sampledPts.col(vID);
            SimdType vertexX = mRot[0][0] * sampledPtsX[vID4] + mRot[0][1] * sampledPtsY[vID4] + mRot[0][2] * sampledPtsZ[vID4] + mT[0];
            SimdType vertexY = mRot[1][0] * sampledPtsX[vID4] + mRot[1][1] * sampledPtsY[vID4] + mRot[1][2] * sampledPtsZ[vID4] + mT[1];
            SimdType vertexZ = mRot[2][0] * sampledPtsX[vID4] + mRot[2][1] * sampledPtsY[vID4] + mRot[2][2] * sampledPtsZ[vID4] + mT[2];

            // const Eigen::Vector3f normal = transform.linear() * sampledNormals.col(vID);
            SimdType normalX = mRot[0][0] * sampledNormalsX[vID4] + mRot[0][1] * sampledNormalsY[vID4] + mRot[0][2] * sampledNormalsZ[vID4];
            SimdType normalY = mRot[1][0] * sampledNormalsX[vID4] + mRot[1][1] * sampledNormalsY[vID4] + mRot[1][2] * sampledNormalsZ[vID4];
            SimdType normalZ = mRot[2][0] * sampledNormalsX[vID4] + mRot[2][1] * sampledNormalsY[vID4] + mRot[2][2] * sampledNormalsZ[vID4];

            //const Eigen::Vector3f pix = K * vertex;
            SimdType pixX = mK(0, 0) * vertexX + mK(0, 1) * vertexY + mK(0, 2) * vertexZ;
            SimdType pixY = mK(1, 0) * vertexX + mK(1, 1) * vertexY + mK(1, 2) * vertexZ;
            const SimdType pixZ = mK(2, 0) * vertexX + mK(2, 1) * vertexY + mK(2, 2) * vertexZ;

            // nearest neighbor lookup
            // const int x = int(pix[0] / pix[2]);
            // const int y = int(pix[1] / pix[2]);
            const SimdType invPixZ = pixZ.Reciprocal();
            pixX *= invPixZ;
            pixY *= invPixZ;
            // pixX /= pixZ;
            // pixY /= pixZ;

            const SimdTypei pixXInt = pixX.ValueCast<int>();
            const SimdTypei pixYInt = pixY.ValueCast<int>();

            // check if in bound
            SimdTypeb valid = (SimdTypei::Zero() <= pixXInt) && (SimdTypei::Zero() <= pixYInt) && (pixXInt < mCameraWidth) && (pixYInt < mCameraHeight);
            const SimdTypei mDepthAndNormalIndex = pixXInt + pixYInt * mCameraWidth;
            alignas(SimdTypei::Alignment()) int depthAndNormalIndex[SimdType::Size()];
            mDepthAndNormalIndex.StoreAligned(depthAndNormalIndex);

            alignas(SimdType::Alignment()) float depthAndNormalD[SimdType::Size()];
            alignas(SimdType::Alignment()) float depthAndNormalNx[SimdType::Size()];
            alignas(SimdType::Alignment()) float depthAndNormalNy[SimdType::Size()];
            alignas(SimdType::Alignment()) float depthAndNormalNz[SimdType::Size()];

            alignas(SimdTypeb::Alignment()) int isValid[SimdTypeb::Size()];
            valid.StoreAligned(isValid);

            for (int k = 0; k < SimdType::Size(); ++k) {
                if (isValid[k]) {
                    const Eigen::Vector4f depthAndNormal = m_depthAndNormals.col(depthAndNormalIndex[k]);
                    depthAndNormalD[k] = depthAndNormal[0];
                    depthAndNormalNx[k] = depthAndNormal[1];
                    depthAndNormalNy[k] = depthAndNormal[2];
                    depthAndNormalNz[k] = depthAndNormal[3];
                } else {
                    depthAndNormalD[k] = 0;
                    depthAndNormalNx[k] = 0;
                    depthAndNormalNy[k] = 0;
                    depthAndNormalNz[k] = 0;
                }
            }

            SimdType mDepthAndNormalD, mDepthAndNormalNx, mDepthAndNormalNy, mDepthAndNormalNz;
            mDepthAndNormalD.LoadAligned(depthAndNormalD);
            mDepthAndNormalNx.LoadAligned(depthAndNormalNx);
            mDepthAndNormalNy.LoadAligned(depthAndNormalNy);
            mDepthAndNormalNz.LoadAligned(depthAndNormalNz);

            valid = valid && (mDepthAndNormalD > SimdType::Zero());

            // Eigen::Vector3f targetVertex = invK * Eigen::Vector3f(float(x), float(y), 1.0f) * depthAndNormal[0];
            pixX = pixXInt.ValueCast<float>();
            pixY = pixYInt.ValueCast<float>();
            const SimdType targetPtX = (mInvK(0, 0) * pixX + mInvK(0, 1) * pixY + mInvK(0, 2)) * mDepthAndNormalD;
            const SimdType targetPtY = (mInvK(1, 0) * pixX + mInvK(1, 1) * pixY + mInvK(1, 2)) * mDepthAndNormalD;
            const SimdType targetPtZ = (mInvK(2, 0) * pixX + mInvK(2, 1) * pixY + mInvK(2, 2)) * mDepthAndNormalD;

            // const float normalCompatibilityWeight = std::max<float>(0, (normal.dot(targetNormal) - normalIncompatibilityThreshold) * normalIncompatibilityMultiplier);
            const SimdType normalCompatibilityWeight = (normalX * mDepthAndNormalNx + normalY * mDepthAndNormalNy + normalZ * mDepthAndNormalNz - mNormalIncompatibilityThreshold) * mNormalIncompatibilityMultiplier;

            //const float squaredDistance = (targetVertex - vertex).squaredNorm();
            const SimdType squaredDistance = (targetPtX - vertexX).Square() + (targetPtY - vertexY).Square() + (targetPtZ - vertexZ).Square();
            const SimdType distanceWeight = mDistanceMultiplier * (mDistanceThreshold - squaredDistance);

            //const float weight = normalCompatibilityWeight * std::max<float>(0, distanceMultiplier * (distanceThreshold - squaredDistance));
            valid = valid && (normalCompatibilityWeight > SimdType::Zero());
            valid = valid && (distanceWeight > SimdType::Zero());
            SimdType weight = (mGeometryWeight * distanceWeight * normalCompatibilityWeight).ConditionalMove(valid);
            weights[vID4] = weight;

            mTotalDistance += squaredDistance * weight;
            mTotalWeight += weight;

            // const float residual = weight * targetNormal.dot(vertex - targetVertex);
            residuals[vID4] = weight * (mDepthAndNormalNx * (vertexX - targetPtX) + mDepthAndNormalNy * (vertexY - targetPtY) + mDepthAndNormalNz * (vertexZ - targetPtZ));
            // const Eigen::Matrix<float, 1, 3> drdV = (targetNormal.transpose() * totalTransform.linear()) * weight;
            drdVxs[vID4] = weight * (mDepthAndNormalNx * mRot[0][0] + mDepthAndNormalNy * mRot[1][0] + mDepthAndNormalNz * mRot[2][0]);
            drdVys[vID4] = weight * (mDepthAndNormalNx * mRot[0][1] + mDepthAndNormalNy * mRot[1][1] + mDepthAndNormalNz * mRot[2][1]);
            drdVzs[vID4] = weight * (mDepthAndNormalNx * mRot[0][2] + mDepthAndNormalNy * mRot[1][2] + mDepthAndNormalNz * mRot[2][2]);
        }

        m_dynamicDistanceThreshold = std::max<float>(m_options.minimumDistanceThresholdSquared, 2.0f * mTotalDistance.HorizontalSum() / mTotalWeight.HorizontalSum());

        mDistanceThreshold.Set(m_dynamicDistanceThreshold);
        mDistanceMultiplier.Set(1.0f / m_dynamicDistanceThreshold);
    }

    for (int vID4 = 0; vID4 < numSimdElements; ++vID4) {

        alignas(SimdType::Alignment()) float weight[SimdType::Size()];
        weights[vID4].StoreAligned(weight);
        alignas(SimdType::Alignment()) float residual[SimdType::Size()];
        residuals[vID4].StoreAligned(residual);
        alignas(SimdType::Alignment()) float drdVx[SimdType::Size()];
        alignas(SimdType::Alignment()) float drdVy[SimdType::Size()];
        alignas(SimdType::Alignment()) float drdVz[SimdType::Size()];
        drdVxs[vID4].StoreAligned(drdVx);
        drdVys[vID4].StoreAligned(drdVy);
        drdVzs[vID4].StoreAligned(drdVz);

        for (int k = 0; k < SimdType::Size(); ++k) {
            if (weight[k] > 0) {
                // const Eigen::Matrix<float, 1, 3> drdV = (targetNormal.transpose() * totalTransform.linear()) * weight;
                // const float residual = weight * targetNormal.dot(vertex - targetVertex);
                vertexConstraints.AddConstraint(m_vertexMask[vID4 * SimdType::Size() + k], residual[k], Eigen::Matrix<float, 1, 3>(drdVx[k], drdVy[k], drdVz[k]));
            }
        }
    }
}


} // namespace epic::nls
