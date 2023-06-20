// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/utils/Timer.h>
#include <nls/geometry/Procrustes.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {


template <class T>
class GridDeformation {
public:
    GridDeformation(int gridSizeX, int gridSizeY, int gridSizeZ)
        : m_gridSizeX(gridSizeX)
        , m_gridSizeY(gridSizeY)
        , m_gridSizeZ(gridSizeZ)

    {
        m_vertexOffsets = Eigen::Matrix<T, 3, -1>::Zero(3, gridSizeX * gridSizeY * gridSizeZ);
    }

    GridDeformation(int gridSize) : GridDeformation(gridSize, gridSize, gridSize) {}

    GridDeformation(int gridSizeX, int gridSizeY, int gridSizeZ, const Eigen::Vector3<T>& minGridPoint, const Eigen::Vector3<T>&  maxGridPoint,
                    const Eigen::Matrix<T, 3, -1>& vertexOffsets)
        : m_gridSizeX(gridSizeX)
        , m_gridSizeY(gridSizeY)
        , m_gridSizeZ(gridSizeZ)
        , m_minGridPoint(minGridPoint)
        , m_maxGridPoint(maxGridPoint)
        , m_vertexOffsets(vertexOffsets)
    {
        CARBON_ASSERT(gridSizeX > 1, "grid size in x dimension needs to be > 1");
        CARBON_ASSERT(gridSizeY > 1, "grid size in y dimension needs to be > 1");
        CARBON_ASSERT(gridSizeZ > 1, "grid size in z dimension needs to be > 1");
        CARBON_ASSERT(vertexOffsets.cols() == gridSizeX * gridSizeY * gridSizeZ, "number of vertex offsets do not match grid dimensions");
        UpdateDelta();
    }

    void Init(const Eigen::Matrix<T, 3, -1>& srcVertices)
    {
        m_minGridPoint = srcVertices.rowwise().minCoeff() - Eigen::Vector<T, 3>::Constant(T(1e-6));
        m_maxGridPoint = srcVertices.rowwise().maxCoeff() + Eigen::Vector<T, 3>::Constant(T(1e-6));
        UpdateDelta();
    }

    int GridSizeX() const { return m_gridSizeX; }
    int GridSizeY() const { return m_gridSizeY; }
    int GridSizeZ() const { return m_gridSizeZ; }

    const Eigen::Vector<T, 3>& GetMinGridPoint() const { return m_minGridPoint; }
    const Eigen::Vector<T, 3>& GetMaxGridPoint() const { return m_maxGridPoint; }
    const Eigen::Vector<T, 3>& GetDelta() const { return m_delta; }

    Eigen::Vector3<T> EvaluateGridPosition(const Eigen::Vector3<T>& pt) {
        return EvaluateGridPosition(GetGridPosition(pt));
    };

    Affine<T, 3, 3> EvaluateGridTransform(const Eigen::Vector3<T>& pt) {
        return EvaluateGridTransform(GetGridPosition(pt));
    }

    void Solve(const Eigen::Matrix<T, 3, -1>& srcVertices,
               const Eigen::Matrix<T, 3, -1>& targetVertices,
               T boundaryWeight = T(10),
               int maxIterations = 100,
               const Eigen::VectorX<T>& areaPerVertex = Eigen::VectorX<T>())
    {
        Timer timer;
        Eigen::Matrix<T, 3, -1> gridVertices(3, GridSizeX() * GridSizeY() * GridSizeZ());
        std::vector<Eigen::Vector4i> gridQuads;
        const int numVertices = int(targetVertices.cols());
        Eigen::VectorX<T> weightPerVertex = areaPerVertex;
        if (areaPerVertex.size() == 0) {
            weightPerVertex = Eigen::VectorX<T>::Ones(numVertices);
        } else {
            const T averageWeight = weightPerVertex.sum() / T(numVertices) + T(1e-9);
            for (int i = 0; i < weightPerVertex.size(); ++i) {
                weightPerVertex[i] = sqrt(weightPerVertex[i] / averageWeight);
            }
        }

        // create sparse matrix
        std::vector<Eigen::Triplet<T>> triplets;
        int numRows = 0;
        for (int z = 0; z < GridSizeZ(); ++z) {
            for (int y = 0; y < GridSizeY(); ++y) {
                for (int x = 0; x < GridSizeX(); ++x) {
                    gridVertices.col(GridToIndex(x, y, z)) = m_minGridPoint + Eigen::Vector3<T>(m_delta[0] * x, m_delta[1] * y, m_delta[2] * z);
                    if (x > 0 && x < GridSizeX() - 1) {
                        triplets.push_back(Eigen::Triplet<T>(numRows, GridToIndex(x + 1, y, z),  T(1)));
                        triplets.push_back(Eigen::Triplet<T>(numRows, GridToIndex(x    , y, z), -T(1)));
                        numRows++;
                    }
                    if (y > 0 && y < GridSizeY() - 1) {
                        triplets.push_back(Eigen::Triplet<T>(numRows, GridToIndex(x, y + 1, z),  T(1)));
                        triplets.push_back(Eigen::Triplet<T>(numRows, GridToIndex(x, y    , z), -T(1)));
                        numRows++;
                    }
                    if (z > 0 && z < GridSizeZ() - 1) {
                        triplets.push_back(Eigen::Triplet<T>(numRows, GridToIndex(x, y, z + 1),  T(1)));
                        triplets.push_back(Eigen::Triplet<T>(numRows, GridToIndex(x, y, z    ), -T(1)));
                        numRows++;
                    }

                    if (x < GridSizeX() - 1 && y < GridSizeY() - 1) {
                        gridQuads.push_back(Eigen::Vector4i(GridToIndex(x, y, z), GridToIndex(x + 1, y, z), GridToIndex(x + 1, y + 1, z), GridToIndex(x, y + 1, z)));
                    }

                    if (x < GridSizeX() - 1 && z < GridSizeZ() - 1) {
                        gridQuads.push_back(Eigen::Vector4i(GridToIndex(x, y, z), GridToIndex(x + 1, y, z), GridToIndex(x + 1, y, z + 1), GridToIndex(x, y, z + 1)));
                    }

                    if (y < GridSizeY() - 1 && z < GridSizeZ() - 1) {
                        gridQuads.push_back(Eigen::Vector4i(GridToIndex(x, y, z), GridToIndex(x, y + 1, z), GridToIndex(x, y + 1, z + 1), GridToIndex(x, y, z + 1)));
                    }
                }
            }
        }

        const int numRegularizationTerms = numRows;

        for (int i = 0; i < numVertices; ++i) {
            GridPosition gridPos = GetGridPosition(srcVertices.col(i));
            for (int zc = 0; zc < 2; ++zc) {
                for (int yc = 0; yc < 2; ++yc) {
                    for (int xc = 0; xc < 2; ++xc) {
                        auto [gridIndex, gridWeight] = EvaluateGridPositionWeight(gridPos, xc, yc, zc);
                        triplets.push_back(Eigen::Triplet<T>(numRows, gridIndex, boundaryWeight * gridWeight * weightPerVertex[i]));
                    }
                }
            }
            numRows++;
        }

        SparseMatrix<T> A(numRows, GridSizeX() * GridSizeY() * GridSizeZ());
        A.setFromTriplets(triplets.begin(), triplets.end());
        LOG_INFO("setting matrix: {} ms", timer.Current()); timer.Restart();

        SparseMatrix<T> AtA = A.transpose() * A;
        LOG_INFO("AtA: {} ms", timer.Current()); timer.Restart();

        Eigen::ConjugateGradient<SparseMatrix<T>, Eigen::Lower| Eigen::Upper> solver;
        solver.compute(AtA);
        solver.setMaxIterations(maxIterations);
        LOG_INFO("solver setup: {} ms", timer.Current()); timer.Restart();

        for (int dim = 0; dim < 3; ++dim) {
            Eigen::VectorX<T> b(numRows);
            for (int i = 0; i < numRegularizationTerms; ++i) {
                b[i] = 0;
            }
            for (int i = 0; i < numVertices; ++i) {
                b[i + numRegularizationTerms] = weightPerVertex[i] * boundaryWeight * (targetVertices(dim, i) - srcVertices(dim, i));
            }
            m_vertexOffsets.row(dim) = solver.solve(A.transpose() * b);
            LOG_INFO("#iterations:     {}", solver.iterations());
            LOG_INFO("estimated error: {}", solver.error());
            LOG_INFO("computing solve: {} ms", timer.Current()); timer.Restart();
        }
    }

    struct GridPosition {
        int xi;
        int yi;
        int zi;
        T xw;
        T yw;
        T zw;
    };

    GridPosition GetGridPosition(const Eigen::Vector3<T>& pt) const {
        const T xg = clamp<T>((pt[0] - m_minGridPoint[0]) / m_delta[0], 0, GridSizeX() - T(1.00001));
        const T yg = clamp<T>((pt[1] - m_minGridPoint[1]) / m_delta[1], 0, GridSizeY() - T(1.00001));
        const T zg = clamp<T>((pt[2] - m_minGridPoint[2]) / m_delta[2], 0, GridSizeZ() - T(1.00001));
        const int xi = int(floor(xg));
        const int yi = int(floor(yg));
        const int zi = int(floor(zg));
        const T xw = xg - xi;
        const T yw = yg - yi;
        const T zw = zg - zi;
        return GridPosition{xi, yi, zi, xw, yw, zw};
    };

private:

    int GridToIndex(int x, int y, int z) const {
        return z * (m_gridSizeX * m_gridSizeY) + y * (m_gridSizeX) + x;
    };

    std::pair<int, T> EvaluateGridPositionWeight(const GridPosition& gridPos, int xc, int yc, int zc) const {
        const T xw = (xc > 0) ? gridPos.xw : (T(1.0) - gridPos.xw);
        const T yw = (yc > 0) ? gridPos.yw : (T(1.0) - gridPos.yw);
        const T zw = (zc > 0) ? gridPos.zw : (T(1.0) - gridPos.zw);
        return {GridToIndex(gridPos.xi + xc, gridPos.yi + yc, gridPos.zi + zc), xw * yw * zw };
    };

    Eigen::Vector3<T> EvaluateGridPosition(const GridPosition& gridPos) const {
        Eigen::Vector3<T> result(0, 0, 0);
        for (int zc = 0; zc < 2; ++zc) {
            for (int yc = 0; yc < 2; ++yc) {
                for (int xc = 0; xc < 2; ++xc) {
                    auto [gridIndex, weight] = EvaluateGridPositionWeight(gridPos, xc, yc, zc);
                    result += m_vertexOffsets.col(gridIndex) * weight;
                }
            }
        }
        return result;
    };


    Affine<T, 3, 3> EvaluateGridTransform(const GridPosition& gridPos) {
        Eigen::Matrix<T, 3, -1> src(3, 8), target(3, 8);
        for (int zc = 0; zc < 2; ++zc) {
            for (int yc = 0; yc < 2; ++yc) {
                for (int xc = 0; xc < 2; ++xc) {
                    auto [gridIndex, _] = EvaluateGridPositionWeight(gridPos, xc, yc, zc);
                    Eigen::Vector3<T> srcPt((gridPos.xi + xc) * m_delta[0], (gridPos.yi + yc) * m_delta[1], (gridPos.zi + zc) * m_delta[2]);
                    Eigen::Vector3<T> targetPt = srcPt + m_vertexOffsets.col(GridToIndex(gridPos.xi + xc, gridPos.yi + yc, gridPos.zi + zc));
                    src.col(zc * 4 + yc * 2 + xc) = srcPt;
                    target.col(zc * 4 + yc * 2 + xc) = targetPt;
                }
            }
        }
        return Procrustes<T, 3>::AlignRigid(src, target);
    };

    void UpdateDelta()
    {
        m_delta[0] = (m_maxGridPoint[0] - m_minGridPoint[0]) / T(m_gridSizeX - 1);
        m_delta[1] = (m_maxGridPoint[1] - m_minGridPoint[1]) / T(m_gridSizeY - 1);
        m_delta[2] = (m_maxGridPoint[2] - m_minGridPoint[2]) / T(m_gridSizeZ - 1);
    }

private:
    int m_gridSizeX;
    int m_gridSizeY;
    int m_gridSizeZ;
    Eigen::Vector3<T> m_minGridPoint;
    Eigen::Vector3<T> m_maxGridPoint;
    Eigen::Vector3<T> m_delta;
    Eigen::Matrix<T, 3, -1> m_vertexOffsets;
};


} // namespace nls
} //namespace epic
