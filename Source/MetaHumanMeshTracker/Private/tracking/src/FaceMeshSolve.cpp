// Copyright Epic Games, Inc. All Rights Reserved.

#include <tracking/FaceMeshSolve.h>

#include <nls/BoundedCoordinateDescentSolver.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/SubtractFunction.h>
#include <nrr/deformation_models/DeformationModelRigLogic.h>
#include <nrr/TrackingDescription.h>
#include <tracking/TrackingRig.h>
#include <tracking/TrackingRigState.h>

#include <vector>

#include <stdio.h>

namespace epic::nls {

struct FaceMeshSolve::Private {
    TemplateDescription templateDescription;

    //! deformation models
    std::unique_ptr<DeformationModelRigLogic<float>> defModelRigLogic;

    //! the tracking rig containing RigLogic, RigGeometry, and RigLogicSolveControls
    TrackingRig<float> trackingRig;

    //! Additional tracking rig state containing for the TrackingRig (so RigLogic and RigLogicSolveControls parameters)
    std::unique_ptr<TrackingRigState<float>> trackingRigState;
};

FaceMeshSolve::FaceMeshSolve() : m(std::make_unique<Private>()) {}
FaceMeshSolve::~FaceMeshSolve() {}

bool FaceMeshSolve::Init(const std::string& templateDescriptionFilename, const std::string& solveDefinitionFilename, const std::string& dnaFilename)
{
    if (!m->templateDescription.Load(templateDescriptionFilename)) {
        LOG_ERROR("failed to load template description");
        return false;
    }

    m->defModelRigLogic = std::make_unique<DeformationModelRigLogic<float>>();

    if (!m->trackingRig.LoadRig(dnaFilename, solveDefinitionFilename)) {
        LOG_ERROR("could not load tracking rig");
        return false;
    }
    m->trackingRigState = std::make_unique<TrackingRigState<float>>(m->trackingRig);

    if (!m->trackingRig.VerifyTopology(m->templateDescription.Topology())) {
        LOG_ERROR("the tracking rig is not compatible with the base mesh topology");
        return false;
    }

    m->defModelRigLogic->SetRig(m->trackingRig.GetRig());

    return true;
}

size_t FaceMeshSolve::NumSolveControlSets() const
{
    return m->trackingRig.GetNumSolveControlSets();
}

Eigen::Matrix<float, -1, 3, Eigen::RowMajor> FaceMeshSolve::EvaluateRig(const Eigen::VectorXf& guiControls, int meshIndex) const
{
    return m->trackingRig.GetRig()->EvaluateVertices(guiControls, /*lod=*/0, {meshIndex}).front().transpose();
}

int FaceMeshSolve::NumGuiControls() const
{
    return m->trackingRig.GetRig()->GetRigLogic()->NumGUIControls();
}

std::map<std::string, int> FaceMeshSolve::GuiControlNameToIndexMap() const
{
    std::map<std::string, int> map;
    for (int i = 0; i < m->trackingRig.GetRig()->GetRigLogic()->NumGUIControls(); ++i) {
        map[m->trackingRig.GetRig()->GetRigLogic()->GuiControlNames()[i]] = i;
    }
    return map;
}

Eigen::VectorXf FaceMeshSolve::Solve(const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& targetHeadVertices,
                                     const Eigen::VectorXf& prior,
                                     size_t solveControlSetIndex,
                                     int iterations,
                                     float l1Regularization,
                                     float wPrior,
                                     float wPoint2Point) const
{
    return SolveMultiple({{HeadMeshIndex(), targetHeadVertices}}, prior, solveControlSetIndex, iterations, l1Regularization, wPrior, wPoint2Point);
}

Eigen::VectorXf FaceMeshSolve::SolveMultiple(const std::map<int, Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>& targetVertices,
                                             const Eigen::VectorXf& prior,
                                             size_t solveControlSetIndex,
                                             int iterations,
                                             float l1Regularization,
                                             float wPrior,
                                             float wPoint2Point) const
{
    for (const auto& [meshIndex, meshTargetVertices] : targetVertices) {
        const int numVertices = int(meshTargetVertices.rows());
        const int numExpectedVertices = m->trackingRig.GetRigGeometry()->GetMesh(meshIndex).NumVertices();
        if (numVertices != numExpectedVertices) {
            CARBON_CRITICAL("number of vertices do not match for mesh {}: {} vs {}", m->trackingRig.GetRigGeometry()->GetMeshName(meshIndex), numVertices, numExpectedVertices);
        }
    }

    m->defModelRigLogic->SetRigLogicSolveControls(m->trackingRig.GetRigLogicSolveControls(solveControlSetIndex));
    m->defModelRigLogic->SetSolveControlsToOptimize(m->trackingRigState->RigSolveControlsToOptimize(solveControlSetIndex));
    if (prior.size() > 0) {
        if (prior.size() == m->trackingRig.GetDefaultGuiControlValues().size()) {
            m->defModelRigLogic->SetGuiControls(prior);
        } else {
            LOG_ERROR("prior should have size of the gui controls");
            m->defModelRigLogic->SetGuiControls(m->trackingRig.GetDefaultGuiControlValues());
        }
    } else {
        m->defModelRigLogic->SetGuiControls(m->trackingRig.GetDefaultGuiControlValues());
    }

    // m_defModelRigLogic->SetSymmetricControls(m_symmetricRigLogicGuiControls);

    auto config = m->defModelRigLogic->GetConfiguration();
    config["optimizePose"].Set(false);
    config["l2Regularization"].Set(0.0f);
    config["symmetry"].Set(100.0f);
    m->defModelRigLogic->SetConfiguration(config);

    // const int iterations = 10;
    // const float l1Regularization = 0.1f;
    // const float wPrior = 10.0f;
    // const float wPoint2Point = 1.0f;

    RigGeometry<float>::State state;

    std::vector<int> meshIndices;
    for (const auto& [meshIndex, _] : targetVertices) {
        meshIndices.push_back(meshIndex);
    }


    std::function<DiffData<float>(Context<float>*)> evaluationFunction = [&](Context<float>* context) {
        Cost<float> cost;
        m->defModelRigLogic->EvaluateVertices(context, /*lod=*/0, meshIndices, /*withRigid=*/false, state);
        for (size_t i = 0; i < meshIndices.size(); ++i) {
            const DiffDataMatrix<float, 3, -1> vertices = state.Vertices()[i];
            const DiffDataMatrix<float, 3, -1> target(targetVertices.find(meshIndices[i])->second.transpose());
            cost.Add(vertices - target, wPoint2Point);
        }
        cost.Add(m->defModelRigLogic->EvaluateModelConstraints(context));
        if (prior.size() == m->trackingRig.GetDefaultGuiControlValues().size()) {
            cost.Add(m->defModelRigLogic->EvaluateGuiControls(context) - DiffData<float>(prior), wPrior);
        }
        return cost.CostToDiffData();
    };

    BoundedCoordinateDescentSolver<float> solver;
    Context<float> context;
    if (!solver.Solve(evaluationFunction, context, iterations, m->defModelRigLogic->SolveControlVariable(), l1Regularization)) {
        LOG_ERROR("could not solve optimization problem");
    }
    return m->defModelRigLogic->GuiControls();
}

int FaceMeshSolve::HeadMeshIndex() const
{
    return m->trackingRig.GetRigGeometry()->HeadMeshIndex(/*lod=*/0);
}

int FaceMeshSolve::TeethMeshIndex() const
{
    return m->trackingRig.GetRigGeometry()->TeethMeshIndex(/*lod=*/0);
}

int FaceMeshSolve::EyeLeftMeshIndex() const
{
    return m->trackingRig.GetRigGeometry()->EyeLeftMeshIndex(/*lod=*/0);
}

int FaceMeshSolve::EyeRightMeshIndex() const
{
    return m->trackingRig.GetRigGeometry()->EyeRightMeshIndex(/*lod=*/0);
}

} // namespace epic::nls
