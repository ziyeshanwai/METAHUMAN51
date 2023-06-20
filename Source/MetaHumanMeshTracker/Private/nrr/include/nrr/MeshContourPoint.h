// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/Camera.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic::nls {

    template<class T>
    class MeshContourPoint {
        public:
            /**
             * Finds the position where the contour line switches from front facing to back facing i.e. it finds where the actual contour is on the mesh.
             * The search is from start to end and it will return the first switch.
             */
            static MeshContourPoint FindContourChange(const std::vector<int>& vIDs,
                                                      const Eigen::Ref<const Eigen::Matrix<T, 3, -1>>& vertices,
                                                      const Eigen::Matrix<T, 3, -1>& normals,
                                                      const Camera<T>& camera);

            /**
             * Finds multiple contour points.
             * @see FindContourChange()
             */
            static std::vector<MeshContourPoint> FindContourChanges(const std::vector<std::vector<int> >& multiple_vIDs,
                                                                    const Eigen::Ref<const Eigen::Matrix<T, 3, -1>>& vertices,
                                                                    const Eigen::Matrix<T, 3, -1>& normals,
                                                                    const Camera<T>& camera);

            /**
             * Finds multiple contour points and occlusions.
             * @see FindContourChange()
             */
            static std::pair<std::vector<MeshContourPoint>, std::vector<MeshContourPoint>> FindContourChangesAndOcclusions(
                                                const std::vector<std::vector<int> >& vIDsForContourA,
                                                const std::vector<std::vector<int> >& vIDsForContourB,
                                                const Eigen::Ref<const Eigen::Matrix<T, 3, -1>>& vertices,
                                                const Eigen::Matrix<T, 3, -1>& normals,
                                                const Camera<T>& camera);


            /**
             * Determines for each contour point whether it is occluded by the other contour.
             * Algorithm:
             * - search for closest point on curve of contourB
             * - check whether the contour is occluded by checking the location relative to the curve and the depth
             * Known issue: in many cases the lips self intersect (even in neutral) and in that case one lip contour
             * is set as occluded and the other is not (based on depth), though it would be correct if both are set
             * as occluded.
             * TODO: to determine if a point is truly occluded we could search along a contour line and find the point
             * where the line is not occluded anymore and compare the depth there.
             */
            static void UpdateContourBasedOnOcclusions(std::vector<MeshContourPoint>& contourA, const std::vector<MeshContourPoint>& contourB);

            int vID1;
            int vID2;
            T w1;
            enum class Type {
                CONTOUR,
                OCCLUSION,
                BORDER_FRONT,
                BORDER_BACK,
                INVALID
            } type;
            //! position of contour point in image space and depth
            Eigen::Vector3<T> pix{0, 0, 0};
            //! normal of contour point in image space
            Eigen::Vector2<T> normal{0, 0};
    };
}  // namespace epic::nls
