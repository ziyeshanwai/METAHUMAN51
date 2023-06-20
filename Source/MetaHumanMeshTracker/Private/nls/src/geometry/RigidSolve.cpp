// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/RigidSolve.h>
#include <carbon/simd/VertexUnpack.h>
#include <carbon/utils/Timer.h>

namespace epic::nls {

template <class T>
QRigidMotion<T> SolveRigid(const Eigen::Matrix<T, 3, -1>& srcPts,
                           const Eigen::Matrix<T, 3, -1>& targetPts,
                           const Eigen::Matrix<T, 3, -1>& targetNormals,
                           const Eigen::Vector<T, -1>& weights,
                           const QRigidMotion<T>& inputRigidMotion,
                           bool outputDeltaTransform)
{
    Eigen::Quaternion<T> q = inputRigidMotion.q;
    Eigen::Vector<T, 3> t = inputRigidMotion.t;

    const Eigen::Matrix<T, 3, 3> rot = q.toRotationMatrix();
    Eigen::Matrix<T, 6, 6> AtA = Eigen::Matrix<T, 6, 6>::Zero();
    Eigen::Vector<T, 6> Atb = Eigen::Vector<T, 6>::Zero();
    for (std::ptrdiff_t vID = 0; vID < srcPts.cols(); ++vID) {
        const T weight = weights(vID);
        const Eigen::Vector<T, 3> transformedSrcPt = rot * srcPts.col(vID);
        const Eigen::Vector<T, 3> targetPt = targetPts.col(vID);
        const Eigen::Vector<T, 3> targetNormal = targetNormals.col(vID);
        // 0 = targetNormal.dot(dR * transformedSrcPt + t + dt - targetPt)
        //     targetNormal.dot(cross(-2 * transformedSrcPt) * dL + transformedSrcPt + t + dt - targetPt)
        //
        const T b = -targetNormal.dot(transformedSrcPt + t - targetPt) * weight;
        Eigen::Vector<T, 6> At;
        // At.head(3) = weight * QuaternionDerivativeMatrix(transformedSrcPt).transpose() * targetNormal;
        At[0] = weight * (-T(2) * transformedSrcPt[2] * targetNormal[1] + T(2) * transformedSrcPt[1] * targetNormal[2]);
        At[1] = weight * (T(2) * transformedSrcPt[2] * targetNormal[0] - T(2) * transformedSrcPt[0] * targetNormal[2]);
        At[2] = weight * (-T(2) * transformedSrcPt[1] * targetNormal[0] + T(2) * transformedSrcPt[0] * targetNormal[1]);
        At[3] = weight * targetNormal[0];
        At[4] = weight * targetNormal[1];
        At[5] = weight * targetNormal[2];
        Atb += At * b;
        //AtA += At * At.transpose();
        AtA(0,0) += At[0] * At[0];
        AtA(1,0) += At[1] * At[0];
        AtA(2,0) += At[2] * At[0];
        AtA(3,0) += At[3] * At[0];
        AtA(4,0) += At[4] * At[0];
        AtA(5,0) += At[5] * At[0];
        AtA(1,1) += At[1] * At[1];
        AtA(2,1) += At[2] * At[1];
        AtA(3,1) += At[3] * At[1];
        AtA(4,1) += At[4] * At[1];
        AtA(5,1) += At[5] * At[1];
        AtA(2,2) += At[2] * At[2];
        AtA(3,2) += At[3] * At[2];
        AtA(4,2) += At[4] * At[2];
        AtA(5,2) += At[5] * At[2];
        AtA(3,3) += At[3] * At[3];
        AtA(4,3) += At[4] * At[3];
        AtA(5,3) += At[5] * At[3];
        AtA(4,4) += At[4] * At[4];
        AtA(5,4) += At[5] * At[4];
        AtA(5,5) += At[5] * At[5];
    }
    const Eigen::Vector<T, 6> delta = Eigen::LDLT<Eigen::Matrix<T, 6, 6>, Eigen::Lower>(AtA).solve(Atb);
    if (outputDeltaTransform) {
        return QRigidMotion<T>{Eigen::Quaternion<T>(T(1), delta[0], delta[1], delta[2]).normalized(), delta.segment(3, 3)};
    } else {
        q = (Eigen::Quaternion<T>(T(1), delta[0], delta[1], delta[2]) * q).normalized();
        t += delta.segment(3, 3);
        return QRigidMotion<T>{q, t};
    }
}

template QRigidMotion<float> SolveRigid(const Eigen::Matrix<float, 3, -1>& srcPts,
                                           const Eigen::Matrix<float, 3, -1>& targetPts,
                                           const Eigen::Matrix<float, 3, -1>& targetNormals,
                                           const Eigen::Vector<float, -1>& weights,
                                           const QRigidMotion<float>& inputRigidMotion,
                                           bool outputDeltaTransform);

template QRigidMotion<double> SolveRigid(const Eigen::Matrix<double, 3, -1>& srcPts,
                                           const Eigen::Matrix<double, 3, -1>& targetPts,
                                           const Eigen::Matrix<double, 3, -1>& targetNormals,
                                           const Eigen::Vector<double, -1>& weights,
                                           const QRigidMotion<double>& inputRigidMotion,
                                           bool outputDeltaTransform);

QRigidMotion<float> SolveRigid(
    const SimdType* srcPtsX,
    const SimdType* srcPtsY,
    const SimdType* srcPtsZ,
    const SimdType* targetPtsX,
    const SimdType* targetPtsY,
    const SimdType* targetPtsZ,
    const SimdType* targetNormalsX,
    const SimdType* targetNormalsY,
    const SimdType* targetNormalsZ,
    const SimdType* weights,
    const int numSimdElements,
    const QRigidMotion<float>& inputRigidMotion,
    bool outputDeltaTransform)
{
    Eigen::Quaternion<float> q = inputRigidMotion.q;
    Eigen::Vector<float, 3> t = inputRigidMotion.t;

    // Eigen::Vector<float, 21> AtA_vec = Eigen::Vector<float, 21>::Zero();
    // Eigen::Vector<float, 6> Atb = Eigen::Vector<float, 6>::Zero();
    SimdType mAtA_vec[21]{};
    SimdType mAtb[6]{};
    for (int k = 0; k < 21; ++k) mAtA_vec[k].SetZero();
    for (int k = 0; k < 6; ++k) mAtb[k].SetZero();

    const Eigen::Matrix<float, 3, 3> rot = q.toRotationMatrix();
    SimdType mRot[3][3]{};
    for (int k = 0; k < 3; ++k) for (int j = 0; j < 3; ++j) mRot[k][j].Set(rot(k, j));
    SimdType mT[3]{};
    mT[0].Set(t[0]);
    mT[1].Set(t[1]);
    mT[2].Set(t[2]);

    // Eigen::Vector<T, 6> At;
    SimdType mAt[6]{};

    SimdType mTwo{};
    mTwo.Set(2.0f);

    for (int vID = 0; vID < numSimdElements; ++vID) {

        //const Eigen::Vector<T, 3> transformedSrcPt = rot * srcPts.col(vID);
        const SimdType srcX = srcPtsX[vID];
        const SimdType srcY = srcPtsY[vID];
        const SimdType srcZ = srcPtsZ[vID];

        SimdType ptX = mRot[0][0] * srcX;
        ptX = ptX + mRot[0][1] * srcY;
        ptX = ptX + mRot[0][2] * srcZ;

        SimdType ptY = mRot[1][0] * srcX;
        ptY = ptY + mRot[1][1] * srcY;
        ptY = ptY + mRot[1][2] * srcZ;

        SimdType ptZ = mRot[2][0] * srcX;
        ptZ = ptZ + mRot[2][1] * srcY;
        ptZ = ptZ + mRot[2][2] * srcZ;

        // const Eigen::Vector<T, 3> targetPt = targetPts.col(vID);
        const SimdType targetX = targetPtsX[vID];
        const SimdType targetY = targetPtsY[vID];
        const SimdType targetZ = targetPtsZ[vID];

        // const Eigen::Vector<T, 3> targetNormal = targetNormals.col(vID);
        const SimdType targetNormalX = targetNormalsX[vID];
        const SimdType targetNormalY = targetNormalsY[vID];
        const SimdType targetNormalZ = targetNormalsZ[vID];

        const SimdType weight = weights[vID];

        // 0 = targetNormal.dot(dR * transformedSrcPt + t + dt - targetPt)
        //     targetNormal.dot(cross(-2 * transformedSrcPt) * dL + transformedSrcPt + t + dt - targetPt)
        //
        //const T b = -targetNormal.dot(transformedSrcPt + t - targetPt) * weight;
        //const T b = targetNormal.dot(targetPt - transformedSrcPt - t) * weight;
        SimdType b = targetNormalX * (targetX - ptX - mT[0]);
        b = b + targetNormalY * (targetY - ptY - mT[1]);
        b = b + targetNormalZ * (targetZ - ptZ - mT[2]);
        b = b * weight;

        // // At.head(3) = weight * QuaternionDerivativeMatrix(transformedSrcPt).transpose() * targetNormal;
        // At[0] = weight * (-T(2) * transformedSrcPt[2] * targetNormal[1] + T(2) * transformedSrcPt[1] * targetNormal[2]);
        // At[1] = weight * (T(2) * transformedSrcPt[2] * targetNormal[0] - T(2) * transformedSrcPt[0] * targetNormal[2]);
        // At[2] = weight * (-T(2) * transformedSrcPt[1] * targetNormal[0] + T(2) * transformedSrcPt[0] * targetNormal[1]);
        // At[3] = weight * targetNormal[0];
        // At[4] = weight * targetNormal[1];
        // At[5] = weight * targetNormal[2];
        mAt[0] = weight * (ptY * mTwo * targetNormalZ - ptZ * mTwo * targetNormalY);
        mAt[1] = weight * (ptZ * mTwo * targetNormalX - ptX * mTwo * targetNormalZ);
        mAt[2] = weight * (ptX * mTwo * targetNormalY - ptY * mTwo * targetNormalX);
        mAt[3] = weight * targetNormalX;
        mAt[4] = weight * targetNormalY;
        mAt[5] = weight * targetNormalZ;

        // Atb += At * b;
        mAtb[0] = mAtb[0] + mAt[0] * b;
        mAtb[1] = mAtb[1] + mAt[1] * b;
        mAtb[2] = mAtb[2] + mAt[2] * b;
        mAtb[3] = mAtb[3] + mAt[3] * b;
        mAtb[4] = mAtb[4] + mAt[4] * b;
        mAtb[5] = mAtb[5] + mAt[5] * b;

        // AtA += At * A;
        // for (int k = 0, c = 0; k < 6; ++k) {
        //     for (int j = k; j < 6; ++j, ++c) {
        //         mAtA_vec[c] = mAtA_vec[c] + mAt[j] * mAt[k];
        //     }
        // }
        mAtA_vec[0] = mAtA_vec[0] + mAt[0] * mAt[0];
        mAtA_vec[1] = mAtA_vec[1] + mAt[1] * mAt[0];
        mAtA_vec[2] = mAtA_vec[2] + mAt[2] * mAt[0];
        mAtA_vec[3] = mAtA_vec[3] + mAt[3] * mAt[0];
        mAtA_vec[4] = mAtA_vec[4] + mAt[4] * mAt[0];
        mAtA_vec[5] = mAtA_vec[5] + mAt[5] * mAt[0];
        mAtA_vec[6] = mAtA_vec[6] + mAt[1] * mAt[1];
        mAtA_vec[7] = mAtA_vec[7] + mAt[2] * mAt[1];
        mAtA_vec[8] = mAtA_vec[8] + mAt[3] * mAt[1];
        mAtA_vec[9] = mAtA_vec[9] + mAt[4] * mAt[1];
        mAtA_vec[10] = mAtA_vec[10] + mAt[5] * mAt[1];
        mAtA_vec[11] = mAtA_vec[11] + mAt[2] * mAt[2];
        mAtA_vec[12] = mAtA_vec[12] + mAt[3] * mAt[2];
        mAtA_vec[13] = mAtA_vec[13] + mAt[4] * mAt[2];
        mAtA_vec[14] = mAtA_vec[14] + mAt[5] * mAt[2];
        mAtA_vec[15] = mAtA_vec[15] + mAt[3] * mAt[3];
        mAtA_vec[16] = mAtA_vec[16] + mAt[4] * mAt[3];
        mAtA_vec[17] = mAtA_vec[17] + mAt[5] * mAt[3];
        mAtA_vec[18] = mAtA_vec[18] + mAt[4] * mAt[4];
        mAtA_vec[19] = mAtA_vec[19] + mAt[5] * mAt[4];
        mAtA_vec[20] = mAtA_vec[20] + mAt[5] * mAt[5];
    }

    Eigen::Matrix<float, 6, 6> AtA = Eigen::Matrix<float, 6, 6>::Zero();
    {
        int count = 0;
        for (int k = 0; k < 6; ++k) {
            for (int j = k; j < 6; ++j) {
                AtA(j, k) = mAtA_vec[count++].HorizontalSum();
            }
        }
    }
    Eigen::Vector<float, 6> Atb;
    for (int k = 0; k < 6; ++k) {
        Atb[k] = mAtb[k].HorizontalSum();
    }

    const Eigen::Vector<float, 6> delta = Eigen::LDLT<Eigen::Matrix<float, 6, 6>, Eigen::Lower>(AtA).solve(Atb);
    if (outputDeltaTransform) {
        return QRigidMotion<float>{Eigen::Quaternion<float>(float(1), delta[0], delta[1], delta[2]).normalized(), delta.segment(3, 3)};
    } else {
        q = (Eigen::Quaternion<float>(float(1), delta[0], delta[1], delta[2]) * q).normalized();
        t += delta.segment(3, 3);
        return QRigidMotion<float>{q, t};
    }
}

} // namespace epic::nls
