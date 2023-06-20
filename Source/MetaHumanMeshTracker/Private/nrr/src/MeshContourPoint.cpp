// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/MeshContourPoint.h>

#include <nls/geometry/Polyline.h>

namespace epic::nls {
    template<class T>
    MeshContourPoint<T> MeshContourPoint<T>::FindContourChange(const std::vector<int>& vIDs,
                                                               const Eigen::Ref<const Eigen::Matrix<T, 3, -1> >& vertices,
                                                               const Eigen::Matrix<T, 3, -1>& normals,
                                                               const Camera<T>& camera) {
        const int N = int(vIDs.size());
        if (N == 0) {
            return MeshContourPoint{0, 0, T(1), Type::INVALID};
        } else if (N == 1) {
            return MeshContourPoint{vIDs.front(), vIDs.front(), T(1), Type::INVALID};
        }

        auto CalculateCosAngle = [&](int vtxID) {
                const Eigen::Vector3<T> pos = camera.Extrinsics().Transform(vertices.col(vtxID));
                const Eigen::Vector3<T> nrm = camera.Extrinsics().Linear() * normals.col(vtxID);
                const T cosAngle = pos.normalized().dot(nrm);
                return cosAngle;
            };

        auto CalculatePix = [&](int vtxID1, int vtxID2, T w) {
                const Eigen::Vector3<T> pos =
                    camera.Extrinsics().Transform(w * vertices.col(vtxID1) + (T(1) - w) * vertices.col(vtxID2));
                const Eigen::Vector2<T> px = camera.Project(pos,  /*withExtrinsics=*/ false);
                return Eigen::Vector3<T>(px[0], px[1], pos[2]);
            };

        auto CalculatePixNormal = [&](int vtxID1,
                                      int vtxID2,
                                      T w) {
                const Eigen::Vector3<T> pos = camera.Extrinsics().Transform(w * vertices.col(vtxID1) + (T(1) - w) * vertices.col(vtxID2));
                const Eigen::Vector3<T> nrm = camera.Extrinsics().Linear() * (w * normals.col(vtxID1) + (T(1) - w) * normals.col(vtxID2));
                const Eigen::Vector2<T> pixNormal =
                    ((pos + nrm) / (pos[2] + nrm[2]) - pos / pos[2]).template head<2>().normalized();
                return pixNormal;
            };

        // contour points are sorted front to back. If the last one is visible (front facing) then we should return this one, and
        // not the front one which may be not
        // visible (back facing).
        if (CalculateCosAngle(vIDs.back()) > 0) {
            T prevAngle = 0;
            int prevID = -1;
            for (const int vID : vIDs) {
                // find camera direction
                const T cosAngle = CalculateCosAngle(vID);
                if (cosAngle >= 0) {
                    // back facing
                    if (prevID >= 0) {
                        const T delta = (cosAngle - prevAngle) + T(1e-7);
                        const T w1 = cosAngle / delta;
                        const Eigen::Vector3<T> pix = CalculatePix(prevID, vID, w1);
                        const Eigen::Vector2<T> normal = CalculatePixNormal(prevID, vID, w1);
                        return MeshContourPoint{prevID, vID, w1, Type::CONTOUR, pix, normal};
                    } else {
                        // first vertex is already back facing, so return that one
                        const T lambda = T(1);
                        return MeshContourPoint{vIDs[0], vIDs[1], lambda, Type::BORDER_FRONT,
                                                CalculatePix(vIDs[0], vIDs[1], lambda),
                                                CalculatePixNormal(vIDs[0], vIDs[1], lambda)};
                    }
                }
                prevAngle = cosAngle;
                prevID = vID;
            }
        }
        // return last point
        const T lambda = T(0);
        return MeshContourPoint{vIDs[N - 2], vIDs[N - 1], lambda, Type::BORDER_BACK,
                                CalculatePix(vIDs[N - 2], vIDs[N - 1], lambda),
                                CalculatePixNormal(vIDs[N - 2], vIDs[N - 1], lambda)};
    }

    template<class T>
    std::vector<MeshContourPoint<T> > MeshContourPoint<T>::FindContourChanges(const std::vector<std::vector<int> >& multiple_vIDs,
                                                                              const Eigen::Ref<const Eigen::Matrix<T, 3,
                                                                                                                   -1> >& vertices,
                                                                              const Eigen::Matrix<T, 3, -1>& normals,
                                                                              const Camera<T>& camera) {
        std::vector<MeshContourPoint> contourChanges;
        contourChanges.reserve(multiple_vIDs.size());
        for (const std::vector<int>& vIDs : multiple_vIDs) {
            contourChanges.push_back(FindContourChange(vIDs, vertices, normals, camera));
        }
        return contourChanges;
    }

    template<class T>
    std::pair<std::vector<MeshContourPoint<T> >,
              std::vector<MeshContourPoint<T> > > MeshContourPoint<T>::FindContourChangesAndOcclusions(
        const std::vector<std::vector<int> >& vIDsForContourA,
        const std::vector<std::vector<int> >& vIDsForContourB,
        const Eigen::Ref<const Eigen::Matrix<T, 3, -1> >& vertices,
        const Eigen::Matrix<T, 3, -1>& normals,
        const Camera<T>& camera) {

        // find contour points for each line separately
        std::vector<MeshContourPoint<T>> contourPointsA = FindContourChanges(vIDsForContourA, vertices, normals, camera);
        std::vector<MeshContourPoint<T>> contourPointsB = FindContourChanges(vIDsForContourB, vertices, normals, camera);

        // check for occlusions by comparing a contour line against another contour line
        UpdateContourBasedOnOcclusions(contourPointsA, contourPointsB);
        UpdateContourBasedOnOcclusions(contourPointsB, contourPointsA);

        return {contourPointsA, contourPointsB};
    }

template<class T>
void MeshContourPoint<T>::UpdateContourBasedOnOcclusions(std::vector<MeshContourPoint<T>>& contourA, const std::vector<MeshContourPoint<T>>& contourB)
    {
        // create a polyline for contour
        Eigen::Matrix<T, 2, -1> contourBcurve(2, contourB.size());
        for (size_t i = 0; i < contourB.size(); ++i) {
            contourBcurve.col(i) = contourB[i].pix.template head<2>();
        }
        Polyline<T, 2> curveB(contourBcurve);
        if (curveB.Valid()) {
            for (auto& contourAPoint : contourA) {
                int segment = 0;
                T lambda = 0;
                /*T dist =*/ curveB.ClosestPoint(contourAPoint.pix.template head<2>(), segment, lambda);
                const Eigen::Vector3<T> destPix = (T(1) - lambda) * contourB[segment].pix + lambda * contourB[segment + 1].pix;
                const Eigen::Vector2<T> destNormal = (T(1) - lambda) * contourB[segment].normal + lambda * contourB[segment + 1].normal;
                // use the normal of the curve to determine wether it is potentially occluded, and if so then compare the depth
                if ((contourAPoint.pix - destPix).template head<2>().dot(destNormal) < 0 && contourAPoint.pix[2] > destPix[2]) {
                    contourAPoint.type = Type::OCCLUSION;
                }
            }
        }
        else {
            LOG_ERROR("contour curve of size {} is not valid", contourB.size());
        }
    }


    template class MeshContourPoint<float>;
    template class MeshContourPoint<double>;
}  // namespace epic::nls
