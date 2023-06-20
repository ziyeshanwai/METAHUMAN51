// Copyright Epic Games, Inc. All Rights Reserved.

#include <conformer/RigLogicFitting.h>

#include <carbon/utils/Profiler.h>
#include <nls/Cost.h>
#include <nls/DiffDataMatrix.h>
#include <nls/Solver.h>
#include <nls/SolverLM.h>
#include <nls/BoundedCoordinateDescentSolver.h>
#include <nls/geometry/AffineVariable.h>
#include <nls/geometry/QuaternionVariable.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/utils/Configuration.h>
#include <nls/utils/ConfigurationParameter.h>
#include <nrr/ICPConstraints.h>
#include <nrr/CollisionConstraints.h>

namespace epic::nls {

template <class T>
struct RigLogicFitting<T>::Private
{
    //! The source mesh (the vertices are the latest deformed state, or set by the user)
    Mesh<T> sourceMesh;

    //! Structure to calculate 2d landmark constraints
    std::vector<std::shared_ptr<LandmarkConstraints2D<T>>> landmarkConstraints2d;

    //! Structure to calculate 3d landmark constraints
    std::vector<std::shared_ptr<LandmarkConstraints3D<T>>> landmarkConstraints3d;

    //! Structure to keep mesh landmarks
    MeshLandmarks<T> meshLandmarks;

    //! An identity model for part-based nonrigid registration
    DeformationModelRigLogic<T> deformationModelRigLogic;

    CollisionConstraints<T> lipCollisionConstraints;

    std::vector<std::shared_ptr<ICPConstraints<T>>> icpConstraints;
};

template <class T>
RigLogicFitting<T>::RigLogicFitting() : m(std::make_unique<Private>())
{
}


template <class T> RigLogicFitting<T>::~RigLogicFitting() = default;
template <class T> RigLogicFitting<T>::RigLogicFitting(RigLogicFitting&& other) = default;
template <class T> RigLogicFitting<T>& RigLogicFitting<T>::operator=(RigLogicFitting&& other) = default;


template <class T>
void RigLogicFitting<T>::InitIcpConstraints(int numOfObservations) {
    if (int(m->icpConstraints.size()) != numOfObservations) {
        m->icpConstraints.resize(numOfObservations);
    }

    for (int i = 0; i < numOfObservations; i++) {
        m->icpConstraints[i] = std::make_shared<ICPConstraints<T>>();
    }
}

template <class T>
void RigLogicFitting<T>::Init2DLandmarksConstraints(int numOfObservations) {
    if (int(m->landmarkConstraints2d.size()) != numOfObservations) {
        m->landmarkConstraints2d.resize(numOfObservations);
    }

    for (int i = 0; i < numOfObservations; i++) {
        m->landmarkConstraints2d[i] = std::make_shared<LandmarkConstraints2D<T>>();
    }
}

template <class T>
void RigLogicFitting<T>::Init3DLandmarksConstraints(int numOfObservations) {
    if (int(m->landmarkConstraints3d.size()) != numOfObservations) {
        m->landmarkConstraints3d.resize(numOfObservations);
    }

    for (int i = 0; i < numOfObservations; i++) {
        m->landmarkConstraints3d[i] = std::make_shared<LandmarkConstraints3D<T>>();
    }
}

template <class T>
void RigLogicFitting<T>::LoadRig(dna::StreamReader* dnaRig)
{
    std::shared_ptr<Rig<T>> rig = std::make_shared<Rig<T>>();
    if (!rig->LoadRig(dnaRig)) {
        CARBON_CRITICAL("Unable to initialize rig logic from loaded dna.");
    }
    SetRig(rig);
}

template <class T>
void RigLogicFitting<T>::SetLipCollisionMasks(const VertexWeights<T>& maskUpperLip, const VertexWeights<T>& maskLowerLip)
{
    m->lipCollisionConstraints.SetSourceTopology(m->sourceMesh, maskUpperLip.NonzeroVertices());
    m->lipCollisionConstraints.SetTargetTopology(m->sourceMesh, maskLowerLip.NonzeroVertices());
}

template <class T>
void RigLogicFitting<T>::SetRig(std::shared_ptr<Rig<T>> rig)
{
    m->deformationModelRigLogic.SetRig(rig);
    m->sourceMesh = rig->GetRigGeometry()->GetMesh(0);
    m->sourceMesh.Triangulate();
    m->sourceMesh.CalculateVertexNormals();
}

template <class T>
void RigLogicFitting<T>::SetMeshLandmarks(const MeshLandmarks<T>& meshLandmarks)
{
    m->meshLandmarks = meshLandmarks;
}

template <class T>
void RigLogicFitting<T>::SetTargetDepths(const std::vector<std::vector<std::shared_ptr<const DepthmapData<T>>>>& targetDepths)
{
    InitIcpConstraints(int(targetDepths.size()));

    for (size_t i = 0; i < targetDepths.size(); ++i) {
        for (size_t j = 0; j < targetDepths[i].size(); ++j) {
            m->icpConstraints[i]->AddTargetDepthAndNormals(targetDepths[i][j]);
        }
    }
}

template <class T>
void RigLogicFitting<T>::SetTargetMeshes(const std::vector<std::shared_ptr<const Mesh<T>>>& targetMeshes,
    const std::vector<Eigen::VectorX<T>>& targetWeights)
{
    InitIcpConstraints(int(targetMeshes.size()));

    for (int i = 0; i < int(targetMeshes.size()); ++i) {
        m->icpConstraints[i]->SetTargetMesh(*targetMeshes[i]);
        if (targetWeights.size() > 0) {
            m->icpConstraints[i]->SetTargetWeights(targetWeights[i]);
        }
    }
}

template <class T>
void RigLogicFitting<T>::SetGuiControls(const Eigen::VectorX<T>& currentControls)
{
    m->deformationModelRigLogic.SetGuiControls(currentControls);
}

template <class T>
void RigLogicFitting<T>::SetTarget2DLandmarks(const std::vector<std::vector<std::pair<LandmarkInstance<T, 2>, Camera<T>>>>& landmarks)
{
    Init2DLandmarksConstraints(int(landmarks.size()));

    for (int i = 0; i < int(landmarks.size()); ++i) {
        m->landmarkConstraints2d[i]->SetMeshLandmarks(m->meshLandmarks);
        m->landmarkConstraints2d[i]->SetTargetLandmarks(landmarks[i]);
    }
}

template <class T>
void RigLogicFitting<T>::SetTarget3DLandmarks(const std::vector<LandmarkInstance<T, 3>>& landmarks) {
    Init3DLandmarksConstraints(int(landmarks.size()));

    for (int i = 0; i < int(landmarks.size()); ++i) {
        m->landmarkConstraints3d[i]->SetMeshLandmarks(m->meshLandmarks);
        m->landmarkConstraints3d[i]->SetTargetLandmarks(landmarks[i]);
    }
}

template <class T>
Eigen::VectorX<T> RigLogicFitting<T>::CurrentGuiControls() const
{
    return m->deformationModelRigLogic.GuiControls();
}

template <class T>
Eigen::Matrix<T, 3, -1> RigLogicFitting<T>::CurrentVertices()
{
    return m->deformationModelRigLogic.DeformedVertices(0);
}

template <class T>
std::vector<Affine<T, 3, 3>> RigLogicFitting<T>::RegisterRigLogic(const std::vector<Affine<T, 3, 3>>& source2target,
                                                                    const VertexWeights<T>& searchWeights,
                                                                    int numIterations)
{
    CARBON_ASSERT(m->icpConstraints.size() == source2target.size(), "number of targets does not match number of icp constraints");

    PROFILING_FUNCTION(PROFILING_COLOR_NAVY);

    const bool optimizePose = m_rigLogicFittingConfig["optimizePose"].template Value<bool>();
    std::vector<AffineVariable<QuaternionVariable<T>>> face2scanTransformVariables(source2target.size());
    for (int i = 0; i < int(face2scanTransformVariables.size()); ++i) {
        face2scanTransformVariables[i].SetAffine(source2target[i]);
        face2scanTransformVariables[i].MakeConstant(!optimizePose, !optimizePose);
    }

    Update2DLandmarkConfiguration(m_rigLogicFittingConfig);
    Update3DLandmarkConfiguration(m_rigLogicFittingConfig);
    UpdateIcpConfiguration(m_rigLogicFittingConfig);
    UpdateIcpWeights(searchWeights);

    const bool use3dLandmarks = !m->landmarkConstraints3d.empty();
    const bool use2dLandmarks = !m->landmarkConstraints2d.empty();

    if (!use3dLandmarks && !use2dLandmarks) {
        LOG_WARNING("No landmark constraints set for riglogic fitting.");
    }

    Mesh<T> currentMesh = m->sourceMesh;
    const T lipCollisionWeight = m_rigLogicFittingConfig["collisionWeight"].template Value<T>();

    typename RigGeometry<T>::State state;

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {
        Cost<T> cost;

        m->deformationModelRigLogic.EvaluateVertices(context, /*lod=*/0, /*meshIndices=*/{0}, /*withRigid=*/false, state);
        const DiffDataMatrix<T, 3, -1> stabilizedVertices = state.Vertices().front();
        std::vector<DiffDataMatrix<T, 3, -1>> transformedVertices;

        if (lipCollisionWeight > T(0)) {
            m->lipCollisionConstraints.CalculateCollisions(currentMesh, currentMesh);
        }

        for (auto& transformVar : face2scanTransformVariables) {
            DiffDataAffine<T, 3, 3> diffFace2ScanTransform = transformVar.EvaluateAffine(context);
            transformedVertices.push_back(diffFace2ScanTransform.Transform(stabilizedVertices));
        }
        for (int i = 0; i < int(m->icpConstraints.size()); ++i) {
            currentMesh.SetVertices(transformedVertices[i].Matrix());
            currentMesh.CalculateVertexNormals();
            cost.Add(m->icpConstraints[i]->EvaluateICP(transformedVertices[i], currentMesh.VertexNormals(), /*searchCorrespondences=*/bool(context)), T(1));
            if (use2dLandmarks) {
                cost.Add(m->landmarkConstraints2d[i]->Evaluate(transformedVertices[i], currentMesh.VertexNormals()), T(1));
            }
            if (use3dLandmarks) {
                cost.Add(m->landmarkConstraints3d[i]->EvaluateLandmarks(transformedVertices[i]), T(1));
            }
            if (lipCollisionWeight > T(0)) {
                cost.Add(m->lipCollisionConstraints.EvaluateCollisions(transformedVertices[i], transformedVertices[i]), lipCollisionWeight);
            }
        }

        cost.Add(m->deformationModelRigLogic.EvaluateModelConstraints(context), T(1));

        return cost.CostToDiffData();
    };

    T l1reg = m_rigLogicFittingConfig["l1regularization"].template Value<T>();

    BoundedCoordinateDescentSolver<T> solver;
    const T startEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    Context<T> context;
    if (solver.Solve(evaluationFunction, context, numIterations, m->deformationModelRigLogic.SolveControlVariable(), l1reg)) {
        const T finalEnergy = evaluationFunction(nullptr).Value().squaredNorm();
        LOG_INFO("energy changed from {} to {}", startEnergy, finalEnergy);
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
void RigLogicFitting<T>::UpdateIcpConfiguration(const Configuration& targetConfig) {
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
void RigLogicFitting<T>::Update2DLandmarkConfiguration(const Configuration& targetConfig) {
    for (auto landmarkConstr : m->landmarkConstraints2d) {
        Configuration currentConfig = landmarkConstr->GetConfiguration();
        currentConfig["landmarksWeight"] = targetConfig["landmarksWeight"];
        currentConfig["innerLipWeight"] = targetConfig["innerLipWeight"];
        currentConfig["curveResampling"] = targetConfig["curveResampling"];
        landmarkConstr->SetConfiguration(currentConfig);
    }
}

template <class T>
void RigLogicFitting<T>::Update3DLandmarkConfiguration(const Configuration& targetConfig) {
    for (auto landmarkConstr : m->landmarkConstraints3d) {
        Configuration currentConfig = landmarkConstr->GetConfiguration();
        currentConfig["landmarksWeight"] = targetConfig["3DlandmarksWeight"];
        currentConfig["innerLipWeight"] = targetConfig["innerLipWeight"];
        currentConfig["curveResampling"] = targetConfig["curveResampling"];
        landmarkConstr->SetConfiguration(currentConfig);
    }
}

template <class T>
void RigLogicFitting<T>::UpdateIcpWeights(const VertexWeights<T>& weights) {
    for (auto icp : m->icpConstraints) {
        icp->ClearPreviousCorrespondences();
        icp->SetCorrespondenceSearchVertexWeights(weights);
    }
}

// explicitly instantiate the rig logic fitting classes
template class RigLogicFitting<float>;
template class RigLogicFitting<double>;

} //namespace epic::nls
