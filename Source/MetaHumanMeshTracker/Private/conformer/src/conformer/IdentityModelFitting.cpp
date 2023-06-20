// Copyright Epic Games, Inc. All Rights Reserved.

#include <conformer/IdentityModelFitting.h>

#include <carbon/utils/Profiler.h>
#include <nls/Cost.h>
#include <nls/DiffDataMatrix.h>
#include <nls/Solver.h>
#include <nls/SolverLM.h>
#include <nls/BoundedCoordinateDescentSolver.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/utils/Configuration.h>
#include <nls/utils/ConfigurationParameter.h>
#include <nls/geometry/AffineVariable.h>
#include <nls/geometry/QuaternionVariable.h>
#include <conformer/GeometryConstraints.h>
#include <nrr/IdentityBlendModel.h>
#include <nrr/deformation_models/DeformationModelIdentity.h>
#include <nrr/deformation_models/DeformationModelRigid.h>
#include <nrr/deformation_models/DeformationModelVertex.h>
#include <nrr/CollisionConstraints.h>

namespace epic::nls {

template <class T>
struct IdentityModelFitting<T>::Private
{
    //! The source mesh (the vertices are the latest deformed state, or set by the user)
    Mesh<T> sourceMesh;

    //! Structure to calculate landmark constraints
    std::vector<std::shared_ptr<LandmarkConstraints2D<T>>> landmarkConstraints2d;

    //! Structure to calculate 3D landmark constraints
    std::vector<std::shared_ptr<LandmarkConstraints3D<T>>> landmarkConstraints3d;

    //! Structure to keep mesh landmarks
    MeshLandmarks<T> meshLandmarks;

    //! An identity model for part-based nonrigid registration
    DeformationModelIdentity<T> deformationModelIdentity;

    //! the base of the mesh
    Eigen::Matrix<T, 3, -1> sourceBase;

    //! the per-vertex offsets of the mesh
    Eigen::Matrix<T, 3, -1> sourceOffsets;

    //! Structure to calculate icp constraints
    std::vector<std::shared_ptr<GeometryConstraints<T>>> icpConstraints;

    bool alignedInputs = false;

    // prevent collision between upper and lower jaw
    CollisionConstraints<T> collisionConstraints;

    Eigen::Matrix<T, 3, -1> CurrentBase()
    {
        if (deformationModelIdentity.NumParameters() > 0) {
            return deformationModelIdentity.DeformedVertices();
        }
        else {
            return sourceBase;
        }
    }

    //! update the deformed vertices based on the current model state
    void UpdateDeformed()
    {
        if (sourceOffsets.cols() > 0) {
            sourceDeformed = CurrentBase() + sourceOffsets;
        }


        //constraintsDebugInfo.reset();
    }

    const Eigen::Matrix<T, 3, -1>& CurrentDeformed() const
    {
        return sourceDeformed;
    }

private:
    //! the current final deformed source (internal representation after evaluation)
    Eigen::Matrix<T, 3, -1> sourceDeformed;
};


template <class T>
IdentityModelFitting<T>::IdentityModelFitting() : m(std::make_unique<Private>())
{
}

template <class T> IdentityModelFitting<T>::~IdentityModelFitting() = default;
template <class T> IdentityModelFitting<T>::IdentityModelFitting(IdentityModelFitting&& other) = default;
template <class T> IdentityModelFitting<T>& IdentityModelFitting<T>::operator=(IdentityModelFitting&& other) = default;

template <class T>
void IdentityModelFitting<T>::InitIcpConstraints(int numOfObservations) {
    if (int(m->icpConstraints.size()) != numOfObservations) {
        m->icpConstraints.resize(numOfObservations);
    }

    for (int i = 0; i < numOfObservations; i++) {
        m->icpConstraints[i] = std::make_shared<GeometryConstraints<T>>();
    }
}

template <class T>
void IdentityModelFitting<T>::Init2DLandmarksConstraints(int numOfObservations) {
    if (int(m->landmarkConstraints2d.size()) != numOfObservations) {
        m->landmarkConstraints2d.resize(numOfObservations);
    }

    for (int i = 0; i < numOfObservations; i++) {
        m->landmarkConstraints2d[i] = std::make_shared<LandmarkConstraints2D<T>>();
    }
}

template <class T>
void IdentityModelFitting<T>::Init3DLandmarksConstraints(int numOfObservations) {
    if (int(m->landmarkConstraints3d.size()) != numOfObservations) {
        m->landmarkConstraints3d.resize(numOfObservations);
    }

    for (int i = 0; i < numOfObservations; i++) {
        m->landmarkConstraints3d[i] = std::make_shared<LandmarkConstraints3D<T>>();
    }
}

template <class T>
void IdentityModelFitting<T>::SetSourceMesh(const Mesh<T>& mesh)
{
    m->sourceMesh = mesh;
    m->sourceMesh.Triangulate();

    m->sourceBase = m->sourceMesh.Vertices();
    m->sourceOffsets = Eigen::Matrix<T, 3, -1>::Zero(3, m->sourceMesh.NumVertices());
    m->UpdateDeformed();

    m->deformationModelIdentity.ResetParameters();
}

template <class T>
void IdentityModelFitting<T>::LoadModel(const std::string& identityModelFile)
{
    m->deformationModelIdentity.LoadModel(identityModelFile);
    m->UpdateDeformed();
}

template <class T>
void IdentityModelFitting<T>::SetMeshLandmarks(const MeshLandmarks<T>& meshLandmarks)
{
    m->meshLandmarks = meshLandmarks;
}

template <class T>
void IdentityModelFitting<T>::SetTargetMeshes(const std::vector<std::shared_ptr<const Mesh<T>>>& targetMeshes,
                                              const std::vector<Eigen::VectorX<T>>& targetWeights, bool alignedInputs)
{
    InitIcpConstraints(int(targetMeshes.size()));
    m->alignedInputs = alignedInputs;

    for (size_t i = 0; i < targetMeshes.size(); ++i) {
        m->icpConstraints[i]->SetTargetMesh(targetMeshes[i]);
        if (targetWeights.size() > 0) {
            m->icpConstraints[i]->SetTargetWeights(targetWeights[i]);
        }
    }
}

template <class T>
void IdentityModelFitting<T>::SetTargetDepths(const std::vector<std::vector<std::shared_ptr<const DepthmapData<T>>>>& targetDepths)
{
    InitIcpConstraints(int(targetDepths.size()));

    for (size_t i = 0; i < targetDepths.size(); ++i) {
        for (size_t j = 0; j < targetDepths[i].size(); ++j) {
            m->icpConstraints[i]->AddTargetDepthAndNormals(targetDepths[i][j]);
        }
    }
}

template <class T>
void IdentityModelFitting<T>::SetTarget2DLandmarks(const std::vector<std::vector<std::pair<LandmarkInstance<T, 2>, Camera<T>>>>& landmarks)
{
    Init2DLandmarksConstraints(int(landmarks.size()));

    for (size_t i = 0; i < landmarks.size(); ++i) {
        m->landmarkConstraints2d[i]->SetMeshLandmarks(m->meshLandmarks);
        m->landmarkConstraints2d[i]->SetTargetLandmarks(landmarks[i]);
    }
}

template <class T>
void IdentityModelFitting<T>::SetTarget3DLandmarks(const std::vector<LandmarkInstance<T, 3>>& landmarks) {
    Init3DLandmarksConstraints(int(landmarks.size()));

    for (int i = 0; i < int(landmarks.size()); ++i) {
        m->landmarkConstraints3d[i]->SetMeshLandmarks(m->meshLandmarks);
        m->landmarkConstraints3d[i]->SetTargetLandmarks(landmarks[i]);
    }
}

template <class T>
void IdentityModelFitting<T>::SetCollisionMasks(const VertexWeights<T>& maskSource, const VertexWeights<T>& maskTarget)
{
    m->collisionConstraints.SetSourceTopology(m->sourceMesh, maskSource.NonzeroVertices());
    m->collisionConstraints.SetTargetTopology(m->sourceMesh, maskTarget.NonzeroVertices());
}

template <class T>
const Eigen::Matrix<T, 3, -1>& IdentityModelFitting<T>::CurrentDeformedVertices() const
{
    return m->CurrentDeformed();
}

template <class T>
Affine<T, 3, 3> IdentityModelFitting<T>::RegisterRigid(const Affine<T, 3, 3>& source2target,
    const VertexWeights<T>& searchWeights,
    int numIterations,
    int scanFrame)
{
    CARBON_PRECONDITION(!source2target.HasScaling(), "the source2target transformation cannot contain any scaling component");

    PROFILING_FUNCTION(PROFILING_COLOR_NAVY);

    DeformationModelRigid<T> deformationModelRigid;

    deformationModelRigid.SetRigidTransformation(source2target);
    deformationModelRigid.SetVertices(m->CurrentDeformed());

    UpdateIcpConfiguration(m_rigidFittingConfig);
    Update2DLandmarkConfiguration(m_rigidFittingConfig);
    Update3DLandmarkConfiguration(m_rigidFittingConfig);
    m->icpConstraints[scanFrame]->SetSourceWeights(searchWeights);
    m->icpConstraints[scanFrame]->ClearPreviousCorrespondences();

    const bool use3dLandmarks = !m->landmarkConstraints3d.empty();
    const bool use2dLandmarks = !m->landmarkConstraints2d.empty();

    if (!use3dLandmarks && !use2dLandmarks) {
        LOG_WARNING("No landmark constraints set for rigid face fitting.");
    }

    Mesh<T> currentMesh = m->sourceMesh;

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {
        Cost<T> cost;

        const DiffDataMatrix<T, 3, -1> transformedVertices = deformationModelRigid.EvaluateVertices(context);

        if (context || !m->icpConstraints[scanFrame]->HasCorrespondences()) {
            currentMesh.SetVertices(transformedVertices.Matrix());
            currentMesh.CalculateVertexNormals();
        }
        m->icpConstraints[scanFrame]->SetupCorrespondences(currentMesh, /*useTarget2Source=*/false);
        cost.Add(m->icpConstraints[scanFrame]->EvaluateICP(transformedVertices), T(1));
        if (use2dLandmarks) {
            cost.Add(m->landmarkConstraints2d[scanFrame]->EvaluateLandmarks(transformedVertices), T(1));
        }
        if (use3dLandmarks) {
            cost.Add(m->landmarkConstraints3d[scanFrame]->EvaluateLandmarks(transformedVertices), T(1));
        }

        return cost.CostToDiffData();
    };

    GaussNewtonSolver<T> solver;
    // LMSolver<T> solver;
    const T startEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    if (solver.Solve(evaluationFunction, numIterations)) {
        const T finalEnergy = evaluationFunction(nullptr).Value().squaredNorm();
        LOG_INFO("energy changed from {} to {}", startEnergy, finalEnergy);
        //m->constraintsDebugInfo.reset();
    }
    else {
        LOG_ERROR("could not solve optimization problem");
    }
    return deformationModelRigid.RigidTransformation();
}

template <class T>
std::vector<Affine<T, 3, 3>> IdentityModelFitting<T>::RegisterRigid(const std::vector<Affine<T, 3, 3>>& source2target,
    const std::vector<VertexWeights<T>>& searchWeights,
    int numIterations) {
    CARBON_ASSERT(m->icpConstraints.size() == source2target.size(), "number of targets does not match number of icp constraints");

    std::vector<Affine<T, 3, 3>> outSource2Target(source2target.size());
    if (m->alignedInputs) {
        auto src2Tgt = RegisterRigid(source2target[0], searchWeights[0], numIterations, 0);
        std::fill(outSource2Target.begin(), outSource2Target.end(), src2Tgt);
    }
    else {
        for (int frame = 0; frame < int(m->icpConstraints.size()); ++frame) {
            outSource2Target[frame] = RegisterRigid(source2target[frame], searchWeights[frame], numIterations, frame);
        }
    }

    return outSource2Target;
}

template <class T>
void IdentityModelFitting<T>::ResetNonrigid()
{
    m->deformationModelIdentity.ResetParameters();
}

template <class T>
std::vector<Affine<T, 3, 3>> IdentityModelFitting<T>::RegisterNonRigid(const std::vector<Affine<T, 3, 3>>& source2target,
                                                                       const std::vector<VertexWeights<T>>& searchWeights,
                                                                       int numIterations)
{
    CARBON_PRECONDITION(m->deformationModelIdentity.NumParameters() > 0, "no identity model - first load model before nonrigid registration");
    CARBON_ASSERT(m->icpConstraints.size() == source2target.size(), "number of targets does not match number of icp constraints");

    PROFILING_FUNCTION(PROFILING_COLOR_NAVY);

    std::vector<AffineVariable<QuaternionVariable<T>>> face2scanTransformVariables(source2target.size());
    for (int i = 0; i < int(face2scanTransformVariables.size()); ++i) {
        face2scanTransformVariables[i].SetAffine(source2target[i]);
        face2scanTransformVariables[i].MakeConstant(true, false);
    }

    Configuration config = m->deformationModelIdentity.GetConfiguration();
    config["optimizePose"].Set(false);
    config["modelRegularization"] = m_modelFittingConfig["modelRegularization"];
    m->deformationModelIdentity.SetConfiguration(config);

    Update2DLandmarkConfiguration(m_modelFittingConfig);
    Update3DLandmarkConfiguration(m_modelFittingConfig);
    UpdateIcpConfiguration(m_modelFittingConfig);
    UpdateIcpWeights(searchWeights);

    bool useInitialCorrespondences = true;
    const bool use3dLandmarks = !m->landmarkConstraints3d.empty();
    const bool use2dLandmarks = !m->landmarkConstraints2d.empty();
    Mesh<T> currentMesh = m->sourceMesh;

    if (!use3dLandmarks && !use2dLandmarks) {
        LOG_WARNING("No landmark constraints set for identity model fitting.");
    }

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {
        Cost<T> cost;

        const DiffDataMatrix<T, 3, -1> stabilizedVertices = m->deformationModelIdentity.EvaluateVertices(context);
        std::vector<DiffDataMatrix<T, 3, -1>> transformedVertices;
        DiffDataAffine<T, 3, 3> diffFace2ScanTransform;
        for (int i = 0; i < face2scanTransformVariables.size(); ++i) {
            if (m->alignedInputs && i == 0) {
                diffFace2ScanTransform = face2scanTransformVariables[0].EvaluateAffine(context);
            }
            if (!m->alignedInputs) {
                diffFace2ScanTransform = face2scanTransformVariables[i].EvaluateAffine(context);
            }
            transformedVertices.push_back(diffFace2ScanTransform.Transform(stabilizedVertices));
        }

        for (int i = 0; i < int(m->icpConstraints.size()); ++i) {
            if (context || !m->icpConstraints[i]->HasCorrespondences()) {
                if (useInitialCorrespondences) {
                    if (m->alignedInputs) {
                        currentMesh.SetVertices(face2scanTransformVariables[0].Affine().Transform(m->CurrentDeformed()));
                    }
                    else {
                        currentMesh.SetVertices(face2scanTransformVariables[i].Affine().Transform(m->CurrentDeformed()));
                    }
                }
                else {
                    currentMesh.SetVertices(transformedVertices[i].Matrix());
                }
                currentMesh.CalculateVertexNormals();
                if (context) {
                    // when we use a Jacobian then we have an update step, and then we should not use the initial correspondences
                    useInitialCorrespondences = false;
                }
                m->icpConstraints[i]->SetupCorrespondences(currentMesh, /*useTarget2Source=*/false);
            }

            cost.Add(m->icpConstraints[i]->EvaluateICP(transformedVertices[i]), T(1));
            if (use2dLandmarks) {
                cost.Add(m->landmarkConstraints2d[i]->Evaluate(transformedVertices[i], currentMesh.VertexNormals()), T(1));
            }
            if (use3dLandmarks) {
                cost.Add(m->landmarkConstraints3d[i]->EvaluateLandmarks(transformedVertices[i]), T(1));
            }
        }
        cost.Add(m->deformationModelIdentity.EvaluateModelConstraints(context), T(1));

        return cost.CostToDiffData();
    };

    GaussNewtonSolver<T> solver;
    const T startEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    if (solver.Solve(evaluationFunction, numIterations)) {
        const T finalEnergy = evaluationFunction(nullptr).Value().squaredNorm();
        LOG_INFO("energy changed from {} to {}", startEnergy, finalEnergy);
        m->UpdateDeformed();
    } else {
        LOG_ERROR("could not solve optimization problem");
    }

    std::vector<Affine<T, 3, 3>> updatedTransforms(face2scanTransformVariables.size());
    for (int i = 0; i < int(face2scanTransformVariables.size()); ++i) {
        updatedTransforms[i] = face2scanTransformVariables[i].Affine();
    }

    return updatedTransforms;
}

template <class T>
void IdentityModelFitting<T>::ResetFine()
{
    m->sourceOffsets.setZero();
    m->UpdateDeformed();
}


template <class T>
std::vector<Affine<T, 3, 3>> IdentityModelFitting<T>::RegisterFine(const std::vector<Affine<T, 3, 3>>& source2target,
    const std::vector<VertexWeights<T>>& searchWeights,
    int numIterations)
{
    CARBON_ASSERT(m->icpConstraints.size() == source2target.size(), "number of targets does not match number of icp constraints");
    PROFILING_FUNCTION(PROFILING_COLOR_NAVY);

    std::vector<AffineVariable<QuaternionVariable<T>>> face2scanTransformVariables(source2target.size());
    const bool optimizePose = m_fineFittingConfig["optimizePose"].template Value<bool>();
    for (int i = 0; i < int(face2scanTransformVariables.size()); ++i) {
        face2scanTransformVariables[i].SetAffine(source2target[i]);
        face2scanTransformVariables[i].MakeConstant(!optimizePose, !optimizePose);
    }

    DeformationModelVertex<T> deformationModelVertex;
    deformationModelVertex.SetMeshTopology(m->sourceMesh);
    deformationModelVertex.SetRestVertices(m->CurrentBase());
    deformationModelVertex.SetVertexOffsets(m->sourceOffsets);
    deformationModelVertex.SetRigidTransformation(Affine<T, 3, 3>());

    // TODO: setting the configuration parameters this way is not nice, instead we should combine all the configuration parameters into a hierarchical structure
    Configuration config = deformationModelVertex.GetConfiguration();
    config["optimizePose"].Set(false);
    config["vertexOffsetRegularization"] = m_fineFittingConfig["vertexOffsetRegularization"];
    config["projectiveStrain"] = m_fineFittingConfig["projectiveStrain"];
    config["greenStrain"] = m_fineFittingConfig["greenStrain"];
    config["quadraticBending"] = m_fineFittingConfig["quadraticBending"];
    config["dihedralBending"] = m_fineFittingConfig["dihedralBending"];
    config["vertexLaplacian"] = m_fineFittingConfig["vertexLaplacian"];
    config["vertexOffsetRegularization"] = m_fineFittingConfig["vertexOffsetRegularization"];
    deformationModelVertex.SetConfiguration(config);

    Update2DLandmarkConfiguration(m_fineFittingConfig);
    Update3DLandmarkConfiguration(m_fineFittingConfig);
    UpdateIcpConfiguration(m_fineFittingConfig);
    UpdateIcpWeights(searchWeights);

    bool useInitialCorrespondences = true;
    Mesh<T> currentMesh = m->sourceMesh;
    const bool sampleScan = m_fineFittingConfig["sampleScan"].template Value<bool>();
    const bool use3dLandmarks = !m->landmarkConstraints3d.empty();
    const bool use2dLandmarks = !m->landmarkConstraints2d.empty();
    const float collisionWeight = m_fineFittingConfig["collisionWeight"].template Value<float>();

    if (!use3dLandmarks && !use2dLandmarks) {
        LOG_WARNING("No landmark constraints set for fine id fitting.");
    }

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {
        Cost<T> cost;

        const auto [stabilizedVertices, _] = deformationModelVertex.EvaluateBothStabilizedAndTransformedVertices(context);
        std::vector<DiffDataMatrix<T, 3, -1>> transformedVertices;

        DiffDataAffine<T, 3, 3> diffFace2ScanTransform;
        for (int i = 0; i < face2scanTransformVariables.size(); ++i) {
            if (m->alignedInputs && i == 0) {
                diffFace2ScanTransform = face2scanTransformVariables[0].EvaluateAffine(context);
            }
            if (!m->alignedInputs) {
                diffFace2ScanTransform = face2scanTransformVariables[i].EvaluateAffine(context);
            }
            transformedVertices.push_back(diffFace2ScanTransform.Transform(stabilizedVertices));
        }

        for (int i = 0; i < int(m->icpConstraints.size()); ++i) {
            if (context || !m->icpConstraints[i]->HasCorrespondences()) {
                if (useInitialCorrespondences) {
                    if (m->alignedInputs) {
                        currentMesh.SetVertices(face2scanTransformVariables[0].Affine().Transform(m->CurrentDeformed()));
                    }
                    else {
                        currentMesh.SetVertices(face2scanTransformVariables[i].Affine().Transform(m->CurrentDeformed()));
                    }
                }
                else {
                    currentMesh.SetVertices(transformedVertices[i].Matrix());
                }
                currentMesh.CalculateVertexNormals();
                if (context) {
                    // when we use a Jacobian then we have an update step, and then we should not use the initial correspondences
                    useInitialCorrespondences = false;
                }
                if (collisionWeight > 0) {
                    m->collisionConstraints.CalculateCollisions(currentMesh, currentMesh);
                }
                m->icpConstraints[i]->SetupCorrespondences(currentMesh, sampleScan);
            }

            const Cost<T> icpResidual = m->icpConstraints[i]->EvaluateICP(transformedVertices[i]);
            if (icpResidual.Size() > 0) {
                // resize the "energy of the ICP constraints" to be the same whether model or scan are sampled
                cost.Add(icpResidual, T(1) / T(icpResidual.Size()) * T(96196));
            }

            if (use2dLandmarks) {
                cost.Add(m->landmarkConstraints2d[i]->Evaluate(transformedVertices[i], currentMesh.VertexNormals()), T(1));
            }
            if (use3dLandmarks) {
                cost.Add(m->landmarkConstraints3d[i]->Evaluate(transformedVertices[i], currentMesh.VertexNormals()), T(1));
            }
            if (collisionWeight > 0 && i == 0) {
                cost.Add(m->collisionConstraints.EvaluateCollisions(transformedVertices[i], transformedVertices[i]), collisionWeight);
            }
        }
        cost.Add(deformationModelVertex.EvaluateModelConstraints(context), T(1));

        return cost.CostToDiffData();
    };

    GaussNewtonSolver<T> solver;
    // LMSolver<T> solver;
    const T startEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    if (solver.Solve(evaluationFunction, numIterations)) {
        const T finalEnergy = evaluationFunction(nullptr).Value().squaredNorm();
        LOG_INFO("energy changed from {} to {}", startEnergy, finalEnergy);
        m->sourceOffsets = deformationModelVertex.VertexOffsets();
        m->UpdateDeformed();
    }
    else {
        LOG_ERROR("could not solve optimization problem");
    }

    std::vector<Affine<T, 3, 3>> updatedTransforms(face2scanTransformVariables.size());
    for (int i = 0; i < int(face2scanTransformVariables.size()); ++i) {
        updatedTransforms[i] = face2scanTransformVariables[i].Affine();
    }

    return updatedTransforms;
}

template <class T>
void IdentityModelFitting<T>::UpdateIcpConfiguration(const Configuration& targetConfig) {
    for (auto icpConstr : m->icpConstraints) {
        Configuration currentConfig = icpConstr->GetConfiguration();
        currentConfig["geometryWeight"] = targetConfig["geometryWeight"];
        currentConfig["point2point"] = targetConfig["point2point"];
        currentConfig["useDistanceThreshold"] = targetConfig["useDistanceThreshold"];
        currentConfig["minimumDistanceThreshold"] = targetConfig["minimumDistanceThreshold"];
        icpConstr->SetConfiguration(currentConfig);
    }
}

template <class T>
void IdentityModelFitting<T>::Update2DLandmarkConfiguration(const Configuration& targetConfig) {
    for (auto landmarkConstr : m->landmarkConstraints2d) {
        Configuration currentConfig = landmarkConstr->GetConfiguration();
        currentConfig["landmarksWeight"] = targetConfig["landmarksWeight"];
        currentConfig["curveResampling"] = targetConfig["curveResampling"];
        landmarkConstr->SetConfiguration(currentConfig);
    }
}

template <class T>
void IdentityModelFitting<T>::Update3DLandmarkConfiguration(const Configuration& targetConfig) {
    for (auto landmarkConstr : m->landmarkConstraints3d) {
        Configuration currentConfig = landmarkConstr->GetConfiguration();
        currentConfig["landmarksWeight"] = targetConfig["3DlandmarksWeight"];
        currentConfig["curveResampling"] = targetConfig["curveResampling"];
        landmarkConstr->SetConfiguration(currentConfig);
    }
}

template <class T>
void IdentityModelFitting<T>::UpdateIcpWeights(const std::vector<VertexWeights<T>>& weights) {
    int i = 0;
    for (auto icp : m->icpConstraints) {
        icp->SetSourceWeights(weights[i]);
        icp->ClearPreviousCorrespondences();
        ++i;
    }
}

// explicitly instantiate the face fitting classes
template class IdentityModelFitting<float>;
template class IdentityModelFitting<double>;

} //namespace epic::nls
