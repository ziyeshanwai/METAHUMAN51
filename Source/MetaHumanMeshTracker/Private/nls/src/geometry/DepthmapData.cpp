// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/DepthmapData.h>


namespace epic::nls {

template <class T>
void DepthmapData<T>::Create(const Camera<T>& cam)
{
    camera = cam;
    if (depthAndNormals.cols() != camera.Width() * camera.Height()) {
        depthAndNormals.resize(4, camera.Width() * camera.Height());
    }
}

template <class T>
void DepthmapData<T>::Create(const Camera<T>& cam, const T* depthPtr)
{
    Create(cam);
    for (int y = 0; y < camera.Height(); ++y) {
        depthAndNormals.col(y * camera.Width() + 0).setZero();
        depthAndNormals.col(y * camera.Width() + camera.Width() - 1).setZero();
    }
    for (int x = 0; x < camera.Width(); ++x) {
        depthAndNormals.col(0 * camera.Width() + x).setZero();
        depthAndNormals.col((camera.Height() - 1) * camera.Width() + x).setZero();
    }
    // unprojection uses origin in center of top left pixel so that we can use the integer coordinates directly
    Eigen::Matrix<T, 3, 3> invK = camera.Intrinsics();
    invK(0, 2) -= T(0.5);
    invK(1, 2) -= T(0.5);
    invK = invK.inverse().eval();
    if (fabs(invK(0, 1)) > T(1e-6)) {
        LOG_ERROR("skew not supported");
    }
    const T invFx = invK(0,0);
    const T invFy = invK(1,1);
    const T invCx = invK(0,2);
    const T invCy = invK(1,2);

    const auto pixToVertex = [](T x, T y, T invFx, T invFy, T invCx, T invCy) {
        return Eigen::Vector3<T>((invFx * x + invCx), (invFy * y + invCy), T(1));
    };

    for (int y = 1; y < camera.Height() - 1; ++y) {
        Eigen::Vector3<T> base = pixToVertex(T(1), T(y), invFx, invFy, invCx, invCy);
        const Eigen::Vector3<T> stepX(invFx, 0, 0);
        const Eigen::Vector3<T> stepY(0, invFy, 0);

        for (int x = 1; x < camera.Width() - 1; ++x) {
            const T depth = depthPtr[y * camera.Width() + x];
            const T valid = depth > 0 ? T(1) : 0;
            const T depthXM = depthPtr[y * camera.Width() + x - 1];
            const T validXM = depthXM > 0 ? valid : 0;
            const T depthXP = depthPtr[y * camera.Width() + x + 1];
            const T validXP = depthXP > 0 ? valid : 0;
            const T depthYM = depthPtr[(y - 1) * camera.Width() + x];
            const T validYM = depthYM > 0 ? valid : 0;
            const T depthYP = depthPtr[(y + 1) * camera.Width() + x];
            const T validYP = depthYP > 0 ? valid : 0;

            // const Eigen::Vector3<T> pos = base * depth;
            // const Eigen::Vector3f dXM = ((base - stepX) * depthXM - pos) * validXM;
            // const Eigen::Vector3f dXP = ((base + stepX) * depthXP - pos) * validXP;
            // const Eigen::Vector3f dYM = ((base - stepY) * depthYM - pos) * validYM;
            // const Eigen::Vector3f dYP = ((base + stepY) * depthYP - pos) * validYP;
            const Eigen::Vector3<T> dXM = (base * (depthXM - depth) - stepX * depthXM) * validXM;
            const Eigen::Vector3<T> dXP = (base * (depthXP - depth) + stepX * depthXP) * validXP;
            const Eigen::Vector3<T> dYM = (base * (depthYM - depth) - stepY * depthYM) * validYM;
            const Eigen::Vector3<T> dYP = (base * (depthYP - depth) + stepY * depthYP) * validYP;
            const Eigen::Vector3<T> dXMP = dXM - dXP;
            const Eigen::Vector3<T> dYMP = dYM - dYP;
            Eigen::Vector3<T> normal = dYMP.cross(dXMP);
            normal.normalize();

            depthAndNormals(0, y * camera.Width() + x) = depth;
            depthAndNormals(1, y * camera.Width() + x) = normal[0];
            depthAndNormals(2, y * camera.Width() + x) = normal[1];
            depthAndNormals(3, y * camera.Width() + x) = normal[2];

            base += stepX;

            // if (depth > 0) {
            //     const auto pixToPosition = [](float px, float py, float depth, float fx, float fy, float skew, float cx, float cy)
            //     {
            //         const float y = (py + 0.5f - cy) / fy;
            //         const float x = (px + 0.5f - cx - y * skew) / fx;
            //         return Eigen::Vector3f(x * depth, y * depth, depth);
            //     };
            //     const float fx = camera.Intrinsics()(0,0);
            //     const float fy = camera.Intrinsics()(1,1);
            //     const float skew = camera.Intrinsics()(0,1);
            //     const float cx = camera.Intrinsics()(0,2);
            //     const float cy = camera.Intrinsics()(1,2);
            //     const Eigen::Vector3f pos = pixToPosition(x, y, depthmap(y, x), fx, fy, skew, cx, cy);
            //     const float depthXM = depthmap(y, x - 1);
            //     const Eigen::Vector3f posXM = pixToPosition(x - 1, y, depthXM, fx, fy, skew, cx, cy);
            //     const float depthXP = depthmap(y, x + 1);
            //     const Eigen::Vector3f posXP = pixToPosition(x + 1, y, depthXP, fx, fy, skew, cx, cy);
            //     const float depthYM = depthmap(y - 1, x);
            //     const Eigen::Vector3f posYM = pixToPosition(x, y - 1, depthYM, fx, fy, skew, cx, cy);
            //     const float depthYP = depthmap(y + 1, x);
            //     const Eigen::Vector3f posYP = pixToPosition(x, y + 1, depthYP, fx, fy, skew, cx, cy);

            //     Eigen::Vector3f normal2 = Eigen::Vector3f::Zero();
            //     if (depthXP > 0 && depthYM > 0) {
            //         normal2 += (posXP - pos).cross(posYM - pos);
            //     }
            //     if (depthYM > 0 && depthXM > 0) {
            //         normal2 += (posYM - pos).cross(posXM - pos);
            //     }
            //     if (depthXM > 0 && depthYP > 0) {
            //         normal2 += (posXM - pos).cross(posYP - pos);
            //     }
            //     if (depthYP > 0 && depthXP > 0) {
            //         normal2 += (posYP - pos).cross(posXP - pos);
            //     }
            //     normal2.normalize();
            //     if ((normal - normal2).norm() > 1e-4) {
            //         LOG_ERROR("error on normal {} {}: {} - {} {} {} {} {} {}", x, y, (normal - normal2).norm(), normal[0], normal[1], normal[2], normal2[0], normal2[1], normal2[2]);
            //     }
            //     // depthmapData->depthAndNormals(0, y * camera.Width() + x) = depth;
            //     // depthmapData->depthAndNormals(1, y * camera.Width() + x) = normal2[0];
            //     // depthmapData->depthAndNormals(2, y * camera.Width() + x) = normal2[1];
            //     // depthmapData->depthAndNormals(3, y * camera.Width() + x) = normal2[2];
            // } else {
            //     // depthmapData->depthAndNormals.col(y * camera.Width() + x).setZero();
            // }
        }
    }
}

template struct DepthmapData<float>;

} // namespace epic::nls
