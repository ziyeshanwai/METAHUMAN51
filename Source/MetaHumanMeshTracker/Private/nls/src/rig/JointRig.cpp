// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/rig/JointRig.h>

#include <nls/functions/GatherFunction.h>
#include <nls/functions/ScatterFunction.h>
#include <nls/functions/SubtractFunction.h>
#include <nls/math/SparseMatrixBuilder.h>
#include <nls/rig/Joint.h>
#include <carbon/utils/Profiler.h>

namespace epic {
namespace nls {


template <class AffineType>
Eigen::Matrix<typename JointRig<AffineType>::T, 3, -1>
JointRig<AffineType>::EvaluateGeometry(const std::string& geometryName,
                                       const Eigen::Matrix<T, 3, -1>& restVertices) const
{
    if (m_vertexInfluenceWeights.count(geometryName) == 0) {
        throw std::runtime_error("joint rig does not influence the geometry");
    }

    PROFILING_FUNCTION(PROFILING_COLOR_RED);

    Eigen::Matrix<T,3,Eigen::Dynamic> outputVertices(restVertices.rows(), restVertices.cols());

    const SparseMatrix<T>& vertexInfluenceWeights = m_vertexInfluenceWeights.find(geometryName)->second;

    if (vertexInfluenceWeights.rows() > outputVertices.cols()) {
        throw std::runtime_error("joint mapping does not match the number of vertices");
    }

    std::vector<Eigen::Matrix<T,4,4>> skinningMatrices(m_jointsVector.size());
    for (int i = 0; i < int(m_jointsVector.size()); i++) {
        skinningMatrices[i] = m_jointsVector[i]->SkinningMatrix().Matrix();
    }

    for (int vID = 0; vID < int(vertexInfluenceWeights.outerSize()); ++vID)
    {
        Eigen::Vector3<T> result(0,0,0);
        for (typename SparseMatrix<T>::InnerIterator it(vertexInfluenceWeights, vID); it; ++it)
        {
            const int jointIndex = int(it.col());
            const T weight = it.value();
            const Eigen::Matrix<T,4,4>& mat = skinningMatrices[jointIndex];
            result += weight * (mat.template topLeftCorner<3,3>() * restVertices.col(vID) + mat.template topRightCorner<3,1>());
        }
        outputVertices.col(vID) = result;
    }

    for (int vID = int(vertexInfluenceWeights.outerSize()); vID < int(restVertices.cols()); ++vID) {
        outputVertices.col(vID) = restVertices.col(vID);
    }

    return outputVertices;
}


template <class AffineType>
DiffDataMatrix<typename JointRig<AffineType>::T, 3, -1> JointRig<AffineType>::EvaluateGeometry(const std::string& geometryName,
                                            const DiffDataMatrix<T, 3, -1>& restVertices) const
{
    if (m_vertexInfluenceWeights.count(geometryName) == 0) {
        throw std::runtime_error("joint rig does not influence the geometry");
    }

    PROFILING_FUNCTION(PROFILING_COLOR_RED);

    VectorPtr<T> values;
    JacobianConstPtr<T> Jacobian;

    const int N = int(restVertices.Cols());
    values = std::make_shared<Vector<T>>(restVertices.Size());
    Eigen::Map<Eigen::Matrix<T, 3, -1>> outputVerticesMap(values->data(), restVertices.Rows(), restVertices.Cols());

    const SparseMatrix<T>& vertexInfluenceWeights = m_vertexInfluenceWeights.find(geometryName)->second;

    // first evaluate each joint independently
    PROFILING_BLOCK("part 1");
    std::vector<AffineType> skinningMatrices(m_jointsVector.size());
    std::vector<SparseMatrixConstPtr<T>> linearJacobians(m_jointsVector.size());
    std::vector<SparseMatrixConstPtr<T>> tJacobians(m_jointsVector.size());
    for (int i = 0; i < int(m_jointsVector.size()); i++) {
        skinningMatrices[i] = m_jointsVector[i]->SkinningMatrix();
        if (skinningMatrices[i].Linear().HasJacobian()) {
            linearJacobians[i] = skinningMatrices[i].Linear().Jacobian().AsSparseMatrix();
        }
        if (skinningMatrices[i].Translation().HasJacobian()) {
            tJacobians[i] = skinningMatrices[i].Translation().Jacobian().AsSparseMatrix();
        }
    }
    PROFILING_END_BLOCK;

    // get column size for jacobian
    int maxCols = -1;
    for (const AffineType& aff : skinningMatrices) {
        if (aff.Linear().HasJacobian()) {
            maxCols = std::max<int>(int(aff.Linear().Jacobian().Cols()), maxCols);
        }
        if (aff.Translation().HasJacobian()) {
            maxCols = std::max<int>(int(aff.Translation().Jacobian().Cols()), maxCols);
        }
    }

    if (restVertices.HasJacobian()) {
        maxCols = std::max<int>(int(restVertices.Jacobian().Cols()), maxCols);
    }

    const bool computeJacobian = (maxCols >= 0);
    SparseMatrixConstPtr<T> restVerticesJacobian = (computeJacobian && restVertices.HasJacobian()) ? restVertices.Jacobian().AsSparseMatrix() : nullptr;

    SparseMatrixBuilder<T, 3> sparseMatrixBuilder(3 * N, std::max<int>(0, maxCols));

    PROFILING_BLOCK("part 2");
    for (int vID = 0; vID < int(vertexInfluenceWeights.outerSize()); ++vID)
    {
        if (computeJacobian) {
            sparseMatrixBuilder.StartBlock(3 * vID);
        }
        Eigen::Vector3<T> result(0,0,0);
        for (typename SparseMatrix<T>::InnerIterator it(vertexInfluenceWeights, vID); it; ++it)
        {
            const int jointIndex = int(it.col());
            const T weight = it.value();
            const DiffDataMatrix<T, 3, 3>& linear = skinningMatrices[jointIndex].Linear();
            const DiffDataMatrix<T, 3, 1>& t = skinningMatrices[jointIndex].Translation();
            result += weight * (linear.Matrix() * restVertices.Matrix().col(vID) + t.Matrix());

            if (computeJacobian)
            {
                // add to jacobian
                for (int k = 0; k < 3; k++) {
                    if (linear.HasJacobian()) {
                        for (int j = 0; j < 3; j++) {
                            for (typename SparseMatrix<T>::InnerIterator jit(*linearJacobians[jointIndex], 3 * j + k); jit; ++jit) {
                                sparseMatrixBuilder.Add(3 * vID + k, int(jit.col()), weight * jit.value() * restVertices.Matrix()(j, vID));
                            }
                        }
                    }
                    if (t.HasJacobian()) {
                        for (typename SparseMatrix<T>::InnerIterator jit(*tJacobians[jointIndex], k); jit; ++jit) {
                            sparseMatrixBuilder.Add(3 * vID + k, int(jit.col()), weight * jit.value());
                        }
                    }
                    if (restVertices.HasJacobian()) {
                        for (int j = 0; j < 3; j++) {
                            for (typename SparseMatrix<T>::InnerIterator jit(*restVerticesJacobian, 3 * vID + j); jit; ++jit) {
                                sparseMatrixBuilder.Add(3 * vID + k,int(jit.col()), weight * jit.value() * linear.Matrix()(k, j));
                            }
                        }
                    }
                }
            }
        }
        outputVerticesMap.col(vID) = result;

        if (computeJacobian) {
            sparseMatrixBuilder.FinalizeBlock();
        }
    }
    PROFILING_END_BLOCK;

    PROFILING_BLOCK("part 3");
    for (int vID = int(vertexInfluenceWeights.outerSize()); vID < restVertices.Cols(); ++vID)
    {
        outputVerticesMap.col(vID) = restVertices.Matrix().col(vID);
        if (computeJacobian)
        {
            sparseMatrixBuilder.StartBlock(3 * vID);
            for (int k = 0; k < 3; k++)
            {
                for (typename SparseMatrix<T>::InnerIterator it(*restVerticesJacobian, 3 * vID + k); it; ++it)
                {
                    sparseMatrixBuilder.Add(3 * vID + k, int(it.col()), it.value());
                }
            }
            sparseMatrixBuilder.FinalizeBlock();
        }
    }
    PROFILING_END_BLOCK;

    if (computeJacobian) {
        PROFILING_BLOCK("part 4");
        SparseMatrixPtr<T> sparseMatrix = std::make_shared<SparseMatrix<T>>();
        sparseMatrixBuilder.Build(*sparseMatrix);
        Jacobian = std::make_shared<SparseJacobian<T>>(sparseMatrix);
        PROFILING_END_BLOCK;
    }

    return DiffDataMatrix<T,3,-1>(3, N, DiffData<T>(values, Jacobian));
}


template <class AffineType>
DiffDataMatrix<typename JointRig<AffineType>::T, 3, -1> JointRig<AffineType>::EvaluateGeometryDense(const std::string& geometryName,
                                                                                                    const DiffDataMatrix<T, 3, -1>& restVertices) const
{
    if (m_vertexInfluenceWeights.count(geometryName) == 0) {
        throw std::runtime_error("joint rig does not influence the geometry");
    }

    PROFILING_FUNCTION(PROFILING_COLOR_RED);

    VectorPtr<T> values;
    JacobianConstPtr<T> Jacobian;

    const int N = int(restVertices.Cols());
    values = std::make_shared<Vector<T>>(restVertices.Size());
    Eigen::Map<Eigen::Matrix<T, 3, -1>> outputVerticesMap(values->data(), restVertices.Rows(), restVertices.Cols());

    const SparseMatrix<T>& vertexInfluenceWeights = m_vertexInfluenceWeights.find(geometryName)->second;

    // first evaluate each joint independently
    PROFILING_BLOCK("part 1");
    std::vector<AffineType> skinningMatrices(m_jointsVector.size());
    for (int i = 0; i < int(m_jointsVector.size()); i++) {
        skinningMatrices[i] = m_jointsVector[i]->SkinningMatrix();
    }
    PROFILING_END_BLOCK;

    // get column size for jacobian
    int maxCols = -1;
    for (const AffineType& aff : skinningMatrices) {
        if (aff.Linear().HasJacobian()) {
            maxCols = std::max<int>(int(aff.Linear().Jacobian().Cols()), maxCols);
        }
        if (aff.Translation().HasJacobian()) {
            maxCols = std::max<int>(int(aff.Translation().Jacobian().Cols()), maxCols);
        }
    }

    if (restVertices.HasJacobian()) {
        maxCols = std::max<int>(int(restVertices.Jacobian().Cols()), maxCols);
    }

    const bool computeJacobian = (maxCols >= 0);

    PROFILING_BLOCK("part 2");
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> denseJacobian = Eigen::Matrix<T, -1, -1, Eigen::RowMajor>::Zero(3 * N, std::max<int>(0, maxCols));
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> restVerticesJacobian = restVertices.HasJacobian() ? *(restVertices.Jacobian().AsSparseMatrix()) : Eigen::Matrix<T, -1, -1, Eigen::RowMajor>();
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> jointTranslationJacobian(3 * skinningMatrices.size(), std::max<int>(0, maxCols));
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> jointRotationJacobian(9 * skinningMatrices.size(), std::max<int>(0, maxCols));
    for (int i = 0; i < int(skinningMatrices.size()); ++i) {
        const AffineType& aff = skinningMatrices[i];
        if (aff.Linear().HasJacobian()) {
            jointRotationJacobian.block(9 * i, 0, 9, aff.Linear().Jacobian().Cols()) = *(aff.Linear().Jacobian().AsSparseMatrix());
        }
        if (aff.Translation().HasJacobian()) {
            jointTranslationJacobian.block(3 * i, 0, 3, aff.Translation().Jacobian().Cols()) = *(aff.Translation().Jacobian().AsSparseMatrix());
        }
    }
    PROFILING_END_BLOCK;

    PROFILING_BLOCK("part 3");
    for (int vID = 0; vID < int(vertexInfluenceWeights.outerSize()); ++vID)
    {
        Eigen::Vector3<T> result(0,0,0);
        for (typename SparseMatrix<T>::InnerIterator it(vertexInfluenceWeights, vID); it; ++it)
        {
            const int jointIndex = int(it.col());
            const T weight = it.value();
            const DiffDataMatrix<T, 3, 3>& linear = skinningMatrices[jointIndex].Linear();
            const DiffDataMatrix<T, 3, 1>& t = skinningMatrices[jointIndex].Translation();
            result += weight * (linear.Matrix() * restVertices.Matrix().col(vID) + t.Matrix());

            if (computeJacobian)
            {
                // add to jacobian
                if (linear.HasJacobian()) {
                    for (int k = 0; k < 3; k++) {
                        for (int j = 0; j < 3; j++) {
                            denseJacobian.row(3 * vID + k) += (weight * restVertices.Matrix()(j, vID)) * jointRotationJacobian.row(9 * jointIndex + 3 * j + k);
                        }
                    }
                }
                if (t.HasJacobian()) {
                    for (int k = 0; k < 3; k++) {
                        denseJacobian.row(3 * vID + k) += weight * jointTranslationJacobian.row(3 * jointIndex + k);
                    }
                }

                if (restVertices.HasJacobian())
                {
                    // add to jacobian
                    for (int k = 0; k < 3; k++) {
                        for (int j = 0; j < 3; j++) {
                            denseJacobian.block(3 * vID + k, 0, 1, restVerticesJacobian.cols()) += (weight * linear.Matrix()(k,j)) * restVerticesJacobian.row(3 * vID + j);
                        }
                    }
                }
            }
        }
        outputVerticesMap.col(vID) = result;
    }
    PROFILING_END_BLOCK;

    PROFILING_BLOCK("part 4");
    for (int vID = int(vertexInfluenceWeights.outerSize()); vID < restVertices.Cols(); ++vID)
    {
        outputVerticesMap.col(vID) = restVertices.Matrix().col(vID);
        if (computeJacobian)
        {
            denseJacobian.block(3 * vID, 0, 3, restVerticesJacobian.cols()) = restVerticesJacobian.block(3 * vID, 0, 3, restVerticesJacobian.cols());
        }
    }
    PROFILING_END_BLOCK;

    if (computeJacobian) {
        PROFILING_BLOCK("part 5");
        Jacobian = std::make_shared<SparseJacobian<T>>(std::make_shared<SparseMatrix<T>>(denseJacobian.sparseView()));
        // jacobian = std::make_shared<SparseMatrix<T>>();
        // Dense2Sparse(denseJacobian, *jacobian);
        PROFILING_END_BLOCK;
    }

    return DiffDataMatrix<T,3,-1>(3, N, DiffData<T>(values, Jacobian));
}


template <class AffineType>
DiffData<typename JointRig<AffineType>::T> JointRig<AffineType>::EvaluateRegularization(const T rotationWeight,
                                    const T translationWeight,
                                    const std::map<std::string, Affine<T,3,3>>& restStates,
                                    const bool regularizeRoot) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_RED);

    const int numJoints = int(restStates.size()) - (regularizeRoot ? 0 : 1);
    VectorPtr<T> values = std::make_shared<Vector<T>>(numJoints * 3 * 4);
    std::vector<Eigen::Triplet<T>> triplets;

    int varIndex = 0;
    int maxCols = 0;

    for (auto&& [jointName, restState] : restStates) {
        JointPtr joint = this->GetJoint(jointName);
        if (!joint) {
            throw std::runtime_error("joint rig does not contain " + jointName);
        }
        const bool isRoot = joint->IsRoot();
        if (isRoot && !regularizeRoot) {
            // the root may freely move in the scene
            continue;
        }

        // TODO: this can be more efficient!
        const Eigen::Matrix<T,3,3> restStateRotation = restState.Linear();
        const DiffDataMatrix<T,3,3> restStateRotationInverse(restStateRotation.inverse());
        const DiffDataMatrix<T,3,1> restStateTranslation = restState.Translation();

        const DiffDataAffine<T, 3, 3>& deformedState = joint->LocalMatrix();
        const DiffDataMatrix<T, 3, 3>& deformedStateRotation = deformedState.Linear();
        const DiffDataMatrix<T, 3, 3>& deltaRotation = deformedStateRotation.Multiply(restStateRotationInverse);
        Eigen::Vector<T, 9> deltaRotationFlattened = deltaRotation.Value();
        deltaRotationFlattened[0] -= T(1.0);
        deltaRotationFlattened[4] -= T(1.0);
        deltaRotationFlattened[8] -= T(1.0);

        const DiffDataMatrix<T, 3, 1>& deformedStateTranslation = deformedState.Translation();
        const DiffDataMatrix<T, 3, 1>& deltaTranslate = deformedStateTranslation - restStateTranslation;

        values->segment(12 * varIndex, 9) = deltaRotationFlattened * rotationWeight;
        values->segment(12 * varIndex + 9, 3) = deltaTranslate.Value() * translationWeight;
        if (deltaRotation.HasJacobian()) {
            SparseMatrixConstPtr<T> deltaRotationJacobianSMat = deltaRotation.Jacobian().AsSparseMatrix();
            CARBON_ASSERT(deltaRotationJacobianSMat->rows() == 9, "delta rotation jacobian needs to have 9 rows");
            maxCols = std::max<int>(maxCols, int(deltaRotationJacobianSMat->cols()));
            for (int r = 0; r < deltaRotationJacobianSMat->rows(); r++) {
                for (typename SparseMatrix<T>::InnerIterator it(*deltaRotationJacobianSMat, r); it; ++it) {
                    triplets.push_back(Eigen::Triplet<T>(12 * varIndex + r, int(it.col()), it.value() * rotationWeight));
                }
            }
        }

        if (deltaTranslate.HasJacobian()) {
            SparseMatrixConstPtr<T> deltaTranslateJacobianSMat = deltaTranslate.Jacobian().AsSparseMatrix();
            CARBON_ASSERT(deltaTranslateJacobianSMat->rows() == 3, "delta translation jacobian needs to have 3 rows");
            maxCols = std::max<int>(maxCols, int(deltaTranslateJacobianSMat->cols()));
            for (int r= 0; r < deltaTranslateJacobianSMat->rows(); r++) {
                for (typename SparseMatrix<T>::InnerIterator it(*deltaTranslateJacobianSMat, r); it; ++it) {
                    triplets.push_back(Eigen::Triplet<T>(12 * varIndex + 9 + r, int(it.col()), it.value() * translationWeight));
                }
            }
        }
        varIndex++;
    }

    JacobianConstPtr<T> Jacobian;
    if (triplets.size() > 0) {
        SparseMatrixPtr<T> sparseMatrix = std::make_shared<SparseMatrix<T>>(numJoints * 12, maxCols);
        sparseMatrix->setFromTriplets(triplets.begin(), triplets.end());
        Jacobian = std::make_shared<SparseJacobian<T>>(sparseMatrix);
    }

    return DiffData<T>(values, Jacobian);
}

#ifdef _MSC_VER
    __pragma(warning(push)) \
    __pragma(warning(disable:4324)) // disable warning for padding (Affine<double, 3, 3> is padded in std::optional)
#endif

template Eigen::Matrix<float, 3, -1> JointRig<Affine<float, 3, 3>>::EvaluateGeometry(const std::string& geometryName, const Eigen::Matrix<float, 3, -1>& restVertices) const;
template Eigen::Matrix<double, 3, -1> JointRig<Affine<double, 3, 3>>::EvaluateGeometry(const std::string& geometryName, const Eigen::Matrix<double, 3, -1>& restVertices) const;
template Eigen::Matrix<float, 3, -1> JointRig<DiffDataAffine<float, 3, 3>>::EvaluateGeometry(const std::string& geometryName, const Eigen::Matrix<float, 3, -1>& restVertices) const;
template Eigen::Matrix<double, 3, -1> JointRig<DiffDataAffine<double, 3, 3>>::EvaluateGeometry(const std::string& geometryName, const Eigen::Matrix<double, 3, -1>& restVertices) const;

template DiffDataMatrix<float, 3, -1> JointRig<DiffDataAffine<float, 3, 3>>::EvaluateGeometry(const std::string& geometryName, const DiffDataMatrix<float, 3, -1>& restVertices) const;
template DiffDataMatrix<double, 3, -1> JointRig<DiffDataAffine<double, 3, 3>>::EvaluateGeometry(const std::string& geometryName, const DiffDataMatrix<double, 3, -1>& restVertices) const;
template DiffDataMatrix<float, 3, -1> JointRig<DiffDataAffine<float, 3, 3>>::EvaluateGeometryDense(const std::string& geometryName, const DiffDataMatrix<float, 3, -1>& restVertices) const;
template DiffDataMatrix<double, 3, -1> JointRig<DiffDataAffine<double, 3, 3>>::EvaluateGeometryDense(const std::string& geometryName, const DiffDataMatrix<double, 3, -1>& restVertices) const;

template DiffData<float> JointRig<DiffDataAffine<float, 3, 3>>::EvaluateRegularization(const float rotationWeight, const float translationWeight,
                                                                                        const std::map<std::string, Affine<float,3,3>>& restStates, const bool regularizeRoot) const;
template DiffData<double> JointRig<DiffDataAffine<double, 3, 3>>::EvaluateRegularization(const double rotationWeight, const double translationWeight,
                                                                                        const std::map<std::string, Affine<double,3,3>>& restStates, const bool regularizeRoot) const;

#ifdef _MSC_VER
    __pragma(warning(pop))
#endif

} // namespace nls
} //namespace epic
