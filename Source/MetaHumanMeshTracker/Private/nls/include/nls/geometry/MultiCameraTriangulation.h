// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Context.h>
#include <nls/Cost.h>
#include <nls/MatrixVariable.h>
#include <nls/Solver.h>
#include <nls/functions/SubtractFunction.h>
#include <nls/geometry/Camera.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic {
namespace nls {

/**
 * Class to do mutli camera triangulation. Requires at least two points.
 * Supports triangulation via DLT as well as the more accurate nonlinear variant (which should be initialized by the linear version)
 */
template<class T>
class MultiCameraTriangulation {

public:
  void SetCameras(const std::vector<Camera<T>>& cameras)
  {
    if (cameras.size() < 2) {
      throw std::runtime_error("multi camera triangulation requires at least 2 cameras");
    }
    m_cameras = cameras;
    m_Ps.clear();
    for (const Camera<T>& cam : cameras) {
      m_Ps.push_back(cam.Intrinsics() * cam.Extrinsics().Matrix().block(0, 0, 3, 4));
    }
  }

  Eigen::Vector3<T> Triangulate(const std::vector<Eigen::Vector2<T>>& pixels) const
  {
    if (pixels.size() != m_Ps.size()) {
      throw std::runtime_error("size of pixels/confidences need to match the number of cameras in multicamera triangulation");
    }

    // solve with DLT
    Eigen::Matrix<T, -1, -1> A(2 * m_Ps.size(), 4);
    A.setZero();
    for (int i = 0; i < int(m_Ps.size()); i++) {
      A.row(2 * i + 0) = pixels[i][1] * m_Ps[i].row(2) - m_Ps[i].row(1);
      A.row(2 * i + 1) = m_Ps[i].row(0) - pixels[i][0] * m_Ps[i].row(2);
    }
    const Eigen::JacobiSVD<Eigen::MatrixX<T>> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector4<T> result = svd.matrixV().col(3);
    return result.template head<3>() / result[3];
  }


  Eigen::Vector3<T> TriangulateNonlinear(const Eigen::Vector3<T>& initial, const std::vector<Eigen::Vector2<T>>& pixels, const std::vector<T>& confidences) const
  {
    if (pixels.size() != confidences.size() || pixels.size() != m_cameras.size()) {
      throw std::runtime_error("size of pixels/confidences need to match the number of cameras in multicamera triangulation");
    }
    MatrixVariable<T, 3, -1> var(3, 1);
    var.Set(initial);

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {

      Cost<T> cost;
      DiffDataMatrix<T, 3, -1> center = var.EvaluateMatrix(context);

      for (int i = 0; i < int(m_cameras.size()); i++) {
        DiffDataMatrix<T, 2, 1> projection = m_cameras[i].Project(center, /*withExtrinsics=*/true);
        DiffDataMatrix<T, 2, 1> diff = projection - DiffDataMatrix<T, 2, 1>(pixels[i]);
        cost.Add(diff, confidences[i]);
      }
      return cost.CostToDiffData();
    };

    GaussNewtonSolver<T> solver;
    solver.Solve(evaluationFunction, /*iterations=*/30);
    return var.Matrix();
  }

private:
  std::vector<Camera<T>> m_cameras;
  std::vector<Eigen::Matrix<T, 3, 4>> m_Ps;

};


} // namespace nls
} //namespace epic
