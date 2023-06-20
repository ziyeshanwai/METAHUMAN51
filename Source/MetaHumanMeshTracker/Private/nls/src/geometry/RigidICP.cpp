// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/RigidICP.h>
#include <carbon/utils/Timer.h>

namespace epic::nls {

QRigidMotion<float> RigidICP(const Eigen::Matrix<float, 3, -1>& srcPts,
                             const Eigen::Matrix<float, 3, -1>& srcNormals,
                             const std::vector<int>& srcIndices,
                             const Camera<float>& targetCamera,
                             const Eigen::Matrix<float, 4, -1>& targetDepthAndNormals,
                             const QRigidMotion<float>& inputRigidMotion,
                             const int numIterations)
{
    const float normalIncompatibilityThreshold = 0.5f;
    const float normalIncompatibilityMultiplier = 1.0f / std::max<float>(1e-6f, (1.0f - normalIncompatibilityThreshold));
    const float minimumDistanceThresholdSquared = 4.0f;
    const float stoppingCriterionTranslationNorm = 1e-3f;
    const float stoppingCriterionRotationAngle = 0.05f / 180.0f * float(CARBON_PI);
    const float stoppingCriterionRotationW = cos(stoppingCriterionRotationAngle * 0.5f);

    const std::ptrdiff_t numPts = srcIndices.size();
    if (numPts < 7) {
        return inputRigidMotion;
    }

    // get only required vertices
    Eigen::Matrix<float, 3, -1> sampledPts(3, numPts);
    Eigen::Matrix<float, 3, -1> sampledNormals(3, numPts);
    Eigen::Matrix<float, 3, -1> targetPts = Eigen::Matrix<float, 3, -1>::Zero(3, numPts);
    Eigen::Matrix<float, 3, -1> targetNormals = Eigen::Matrix<float, 3, -1>::Zero(3, numPts);
    Eigen::VectorX<float> weights(numPts);

    Eigen::Vector3f gravity(0, 0, 0);
    for (std::ptrdiff_t i = 0; i < numPts; ++i) {
        Eigen::Vector3f pt = srcPts.col(srcIndices[i]);
        sampledPts.col(i) = pt;
        sampledNormals.col(i) = srcNormals.col(srcIndices[i]);
        gravity += pt;
    }
    gravity /= float(numPts);
    sampledPts.colwise() -= gravity;

    // optimize in camera space to minimize computation
    const QRigidMotion<float> cameraRigidMotion(targetCamera.Extrinsics().Matrix());
    const Eigen::Matrix<float, 3, 3> K = targetCamera.Intrinsics();
    Eigen::Matrix<float, 3, 3> modK = K;
    // unprojection uses origin in center of top left pixel so that we can use the integer coordinates directly
    modK(0, 2) -= 0.5f;
    modK(1, 2) -= 0.5f;
    const Eigen::Matrix<float, 3, 3> invK = modK.inverse();

    QRigidMotion<float> rigidMotion = cameraRigidMotion * inputRigidMotion;

    // optimize rotation around gravity: R * p + t = R (p - gravity) + t + R * gravity
    rigidMotion.t = rigidMotion.t + rigidMotion.q._transformVector(gravity);

    float distanceThreshold = float(1e6);
    float distanceMultiplier = 1.0f / distanceThreshold;

    for (int iter = 0; iter < numIterations; ++iter) {
        const Eigen::Transform<float, 3, Eigen::Affine> transform = rigidMotion.ToEigenTransform();
        float totalWeight = 0;
        float totalDistance = 0;

        for (std::ptrdiff_t vID = 0; vID < numPts; ++vID) {
            const Eigen::Vector3f vertex = transform * sampledPts.col(vID);
            const Eigen::Vector3f normal = transform.linear() * sampledNormals.col(vID);
            const Eigen::Vector3f pix = K * vertex;

            // nearest neighbor lookup
            const int x = int(pix[0] / pix[2]);
            const int y = int(pix[1] / pix[2]);
            if (x >= 0 && x < targetCamera.Width() && y >= 0 && y < targetCamera.Height()) {
                const int depthAndNormalIndex = y * targetCamera.Width() + x;
                const Eigen::Vector4f depthAndNormal = targetDepthAndNormals.col(depthAndNormalIndex);
                if (depthAndNormal[0] > 0) {
                    const Eigen::Vector3f targetVertex = invK * Eigen::Vector3f(float(x), float(y), 1.0f) * depthAndNormal[0];
                    const Eigen::Vector3f targetNormal = depthAndNormal.tail(3);
                    const float normalCompatibilityWeight = std::max<float>(0, (normal.dot(targetNormal) - normalIncompatibilityThreshold) * normalIncompatibilityMultiplier);
                    const float squaredDistance = (targetVertex - vertex).squaredNorm();
                    const float weight = normalCompatibilityWeight * std::max<float>(0, distanceMultiplier * (distanceThreshold - squaredDistance));
                    totalDistance += weight * squaredDistance;
                    totalWeight += weight;
                    weights[vID] = weight;
                    targetPts.col(vID) = targetVertex;
                    targetNormals.col(vID) = targetNormal;
                } else {
                    weights[vID] = 0;
                }
            } else {
                weights[vID] = 0;
            }
        }

        distanceThreshold = std::max<float>(minimumDistanceThresholdSquared, totalDistance / totalWeight);
        distanceMultiplier = 1.0f / distanceThreshold;

        if (iter > 0) { // ignore first iteration to get a better distance threshold
            const QRigidMotion<float> deltaRigidMotion = SolveRigid<float>(sampledPts, targetPts, targetNormals, weights, rigidMotion, /*outputDeltaTransform=*/true);

            rigidMotion.q = (deltaRigidMotion.q * rigidMotion.q).normalized();
            rigidMotion.t += deltaRigidMotion.t;

            if(deltaRigidMotion.q.w() >= stoppingCriterionRotationW && deltaRigidMotion.t.norm() < stoppingCriterionTranslationNorm) {
                break;
            }

        }
    }

    // remove gravity from rigid motion: R (p + gravity) + t = R p + t - R gravity
    rigidMotion.t = rigidMotion.t - rigidMotion.q._transformVector(gravity);

    // remove camera rigid motion
    rigidMotion = cameraRigidMotion.Inverse() * rigidMotion;

    return rigidMotion;
}


QRigidMotion<float> RigidICPFast(const Eigen::Matrix<float, 3, -1>& srcPts,
                             const Eigen::Matrix<float, 3, -1>& srcNormals,
                             const std::vector<int>& srcIndices,
                             const Camera<float>& targetCamera,
                             const Eigen::Matrix<float, 4, -1>& targetDepthAndNormals,
                             const QRigidMotion<float>& inputRigidMotion,
                             const int numIterations)
{
    const float normalIncompatibilityThreshold = 0.5f;
    const float normalIncompatibilityMultiplier = 1.0f / std::max<float>(1e-6f, (1.0f - normalIncompatibilityThreshold));
    const float minimumDistanceThresholdSquared = 4.0f;
    const float stoppingCriterionTranslationNorm = 1e-3f;
    const float stoppingCriterionRotationAngle = 0.05f / 180.0f * float(CARBON_PI);
    const float stoppingCriterionRotationW = cos(stoppingCriterionRotationAngle * 0.5f);

    SimdType mNormalIncompatibilityThreshold;
    mNormalIncompatibilityThreshold.Set(normalIncompatibilityThreshold);
    SimdType mNormalIncompatibilityMultiplier;
    mNormalIncompatibilityMultiplier.Set(normalIncompatibilityMultiplier);

    const int numPts = static_cast<int>(srcIndices.size());
    if (numPts < 7) {
        return inputRigidMotion;
    }

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
    std::vector<SimdType> targetPtsX(numSimdElements);
    std::vector<SimdType> targetPtsY(numSimdElements);
    std::vector<SimdType> targetPtsZ(numSimdElements);
    std::vector<SimdType> targetNormalsX(numSimdElements);
    std::vector<SimdType> targetNormalsY(numSimdElements);
    std::vector<SimdType> targetNormalsZ(numSimdElements);
    std::vector<SimdType> weights(numSimdElements);

    SimdType gravityX, gravityY, gravityZ;
    gravityX.SetZero();
    gravityY.SetZero();
    gravityZ.SetZero();
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
                    const auto& pt = srcPts.col(srcIndices[vID]);
                    const auto& normal = srcNormals.col(srcIndices[vID]);

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
            gravityX += sampledPtsX[simdIndex];
            gravityY += sampledPtsY[simdIndex];
            gravityZ += sampledPtsZ[simdIndex];
        }
    }
    Eigen::Vector3f gravity(gravityX.HorizontalSum() / float(numPts), gravityY.HorizontalSum() / float(numPts), gravityZ.HorizontalSum() / float(numPts));
    gravityX.Set(gravity[0]);
    gravityY.Set(gravity[1]);
    gravityZ.Set(gravity[2]);

    //sampledPts.colwise() -= gravity;
    for (int i = 0; i < numPaddedPts / SimdType::Size(); ++i) {
        sampledPtsX[i] -= gravityX;
        sampledPtsY[i] -= gravityY;
        sampledPtsZ[i] -= gravityZ;
    }


    // optimize in camera space to minimize computation
    const QRigidMotion<float> cameraRigidMotion(targetCamera.Extrinsics().Matrix());
    const Eigen::Matrix<float, 3, 3> K = targetCamera.Intrinsics();
    Eigen::Matrix<float, 3, 3> modK = K;
    // unprojection uses origin in center of top left pixel so that we can use the integer coordinates directly
    modK(0, 2) -= 0.5f;
    modK(1, 2) -= 0.5f;
    const Eigen::Matrix<float, 3, 3> invK = modK.inverse();

    QRigidMotion<float> rigidMotion = cameraRigidMotion * inputRigidMotion;

    // optimize rotation around gravity: R * p + t = R (p - gravity) + t + R * gravity
    rigidMotion.t = rigidMotion.t + rigidMotion.q._transformVector(gravity);

    float distanceThreshold = float(1e6);
    SimdType mDistanceThreshold;
    mDistanceThreshold.Set(distanceThreshold);
    SimdType mDistanceMultiplier;
    mDistanceMultiplier.Set(1.0f / distanceThreshold);

    Eigen::Matrix<SimdType, 3, 3> mK;
    for (int k = 0; k < 3; ++k) for (int j = 0; j < 3; ++j) mK(j, k).Set(K(j, k));
    Eigen::Matrix<SimdType, 3, 3> mInvK;
    for (int k = 0; k < 3; ++k) for (int j = 0; j < 3; ++j) mInvK(j, k).Set(invK(j, k));


    const SimdTypei mCameraWidth(targetCamera.Width());
    const SimdTypei mCameraHeight(targetCamera.Height());

    for (int iter = 0; iter < numIterations; ++iter) {
        //const Eigen::Transform<float, 3, Eigen::Affine> transform = rigidMotion.ToEigenTransform();
        const Eigen::Matrix<float, 3, 3> rot = rigidMotion.q.toRotationMatrix();
        SimdType mRot[3][3];
        for (int k = 0; k < 3; ++k) for (int j = 0; j < 3; ++j) mRot[k][j].Set(rot(k, j));
        SimdType mT[3]{};
        mT[0].Set(rigidMotion.t[0]);
        mT[1].Set(rigidMotion.t[1]);
        mT[2].Set(rigidMotion.t[2]);

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
                    const Eigen::Vector4f depthAndNormal = targetDepthAndNormals.col(depthAndNormalIndex[k]);
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

            targetPtsX[vID4] = targetPtX;
            targetPtsY[vID4] = targetPtY;
            targetPtsZ[vID4] = targetPtZ;
            targetNormalsX[vID4] = mDepthAndNormalNx;
            targetNormalsY[vID4] = mDepthAndNormalNy;
            targetNormalsZ[vID4] = mDepthAndNormalNz;

            // const float normalCompatibilityWeight = std::max<float>(0, (normal.dot(targetNormal) - normalIncompatibilityThreshold) * normalIncompatibilityMultiplier);
            const SimdType normalCompatibilityWeight = (normalX * mDepthAndNormalNx + normalY * mDepthAndNormalNy + normalZ * mDepthAndNormalNz - mNormalIncompatibilityThreshold) * mNormalIncompatibilityMultiplier;

            //const float squaredDistance = (targetVertex - vertex).squaredNorm();
            const SimdType squaredDistance = (targetPtX - vertexX).Square() + (targetPtY - vertexY).Square() + (targetPtZ - vertexZ).Square();
            const SimdType distanceWeight = mDistanceMultiplier * (mDistanceThreshold - squaredDistance);

            //const float weight = normalCompatibilityWeight * std::max<float>(0, distanceMultiplier * (distanceThreshold - squaredDistance));
            valid = valid && (normalCompatibilityWeight > SimdType::Zero());
            valid = valid && (distanceWeight > SimdType::Zero());
            SimdType weight = (distanceWeight * normalCompatibilityWeight).ConditionalMove(valid);
            weights[vID4] = weight;

            mTotalDistance += squaredDistance * weight;
            mTotalWeight += weight;
        }

        distanceThreshold = std::max<float>(minimumDistanceThresholdSquared, mTotalDistance.HorizontalSum() / mTotalWeight.HorizontalSum());

        mDistanceThreshold.Set(distanceThreshold);
        mDistanceMultiplier.Set(1.0f / distanceThreshold);

        if (iter > 0) { // ignore first iteration to get a better distance threshold
            const QRigidMotion<float> deltaRigidMotion = SolveRigid(sampledPtsX.data(), sampledPtsY.data(), sampledPtsZ.data(),
            targetPtsX.data(), targetPtsY.data(), targetPtsZ.data(), targetNormalsX.data(), targetNormalsY.data(), targetNormalsZ.data(), weights.data(), static_cast<int>(numPaddedPts / SimdType::Size()), rigidMotion, /*outputDeltaTransform=*/true);

            rigidMotion.q = (deltaRigidMotion.q * rigidMotion.q).normalized();
            rigidMotion.t += deltaRigidMotion.t;

            // test if we can stop early
            if(deltaRigidMotion.q.w() >= stoppingCriterionRotationW && deltaRigidMotion.t.norm() < stoppingCriterionTranslationNorm) {
                break;
            }
        }
    }

    // remove gravity from rigid motion: R (p + gravity) + t = R p + t - R gravity
    rigidMotion.t = rigidMotion.t - rigidMotion.q._transformVector(gravity);

    // remove camera rigid motion
    rigidMotion = cameraRigidMotion.Inverse() * rigidMotion;

    return rigidMotion;
}


} // namespace epic::nls
