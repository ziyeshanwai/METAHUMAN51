// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/deformation_models/DeformationModelRigLogic.h>
#include <nrr/deformation_models/DeformationModelRigid.h>

#include <nls/BoundedVectorVariable.h>
#include <nls/geometry/AffineVariable.h>
#include <nls/geometry/QuaternionVariable.h>

namespace epic::nls {

template <class T>
struct DeformationModelRigLogic<T>::Private {

    DeformationModelRigid<T> defModelRigid;
    std::shared_ptr<const Rig<T>> rig;
    std::shared_ptr<const RigLogicSolveControls<T>> rigLogicSolveControls;

    Eigen::VectorX<T> baseGuiControls;
    std::unique_ptr<BoundedVectorVariable<T>> varSolveControls;
    std::vector<bool> controlsToOptimize;

    //! user-defined symmetric controls can be used to add a model constraint favoring symmetric activations
    std::vector<std::tuple<std::string, std::string, T>> symmetricControls;

    //! matrix representing the symmetric controls
    SparseMatrix<T> symmetricControlsMatrix;

    Configuration config = {DeformationModelRigLogic<T>::ConfigName(), {
        //!< weight on regularizing the parameter activation
        { "l2Regularization", ConfigurationParameter(T(0), T(0), T(10)) },
        //!< weight for symmetric activations: increase weight to favor symmetric activations
        { "symmetry", ConfigurationParameter(T(0), T(0), T(1000)) },
        //!< whether to optimize the pose when doing rig logic registration
        { "optimizePose", ConfigurationParameter(true) }
    }};
};

template <class T>
DeformationModelRigLogic<T>::DeformationModelRigLogic()
    : m(std::make_unique<Private>())
{}

template <class T> DeformationModelRigLogic<T>::~DeformationModelRigLogic() = default;
template <class T> DeformationModelRigLogic<T>::DeformationModelRigLogic(DeformationModelRigLogic&& other) = default;
template <class T> DeformationModelRigLogic<T>& DeformationModelRigLogic<T>::operator=(DeformationModelRigLogic&& other) = default;

template <class T>
DiffDataMatrix<T, 3, -1> DeformationModelRigLogic<T>::EvaluateVertices(Context<T>* context)
{
    return EvaluateVertices(context, /*lod*/0, /*meshIndex*/0, /*withRigid=*/true);
}

template <class T>
DiffDataMatrix<T, 3, -1> DeformationModelRigLogic<T>::EvaluateVertices(Context<T>* context, int lod, const char* meshName, bool withRigid)
{
    const int meshIndex = MeshIndex(meshName);
    if (meshIndex >= 0) {
        return EvaluateVertices(context, lod, meshIndex, withRigid);
    }
    return DiffDataMatrix<T, 3, -1>(3, 0, DiffData<T>(Vector<T>()));
}

template <class T>
DiffDataMatrix<T, 3, -1> DeformationModelRigLogic<T>::EvaluateVertices(Context<T>* context, int lod, int meshIndex, bool withRigid)
{
    return EvaluateVertices(context, lod, std::vector<int>{ meshIndex }, withRigid).front();
}

template <class T>
DiffData<T> DeformationModelRigLogic<T>::EvaluateGuiControls(Context<T>* context) const
{
    DiffData<T> solveControls = m->varSolveControls->Evaluate(context);
    if (m->rigLogicSolveControls) {
        // if we use a higher level solve control then we need to evaluate it here
        solveControls = m->rigLogicSolveControls->EvaluateGuiControls(solveControls) + DiffData<T>(m->baseGuiControls);
    }
    return solveControls;
}

template <class T>
void DeformationModelRigLogic<T>::EvaluateVertices(Context<T>* context, int lod, const std::vector<int>& meshIndices, bool withRigid, typename RigGeometry<T>::State& state)
{
    DiffData<T> solveControls = EvaluateGuiControls(context);
    DiffData<T> rawControls = m->rig->GetRigLogic()->EvaluateRawControls(solveControls);
    DiffData<T> psd = m->rig->GetRigLogic()->EvaluatePSD(rawControls);
    DiffData<T> joints = m->rig->GetRigLogic()->EvaluateJoints(psd, lod);
    DiffDataAffine<T, 3, 3> rigid;
    if (withRigid) {
        const bool optimizePose = m->config["optimizePose"].template Value<bool>();
        rigid = m->defModelRigid.EvaluateAffine(optimizePose ? context : nullptr);
    }
    m->rig->GetRigGeometry()->EvaluateRigGeometry(rigid, joints, psd, lod, meshIndices, state);
}

template <class T>
std::vector<DiffDataMatrix<T, 3, -1>> DeformationModelRigLogic<T>::EvaluateVertices(Context<T>* context, int lod, const std::vector<int>& meshIndices, bool withRigid)
{
    typename RigGeometry<T>::State state;
    EvaluateVertices(context, lod, meshIndices, withRigid, state);
    return state.Vertices();
}

template <class T>
Cost<T> DeformationModelRigLogic<T>::EvaluateModelConstraints(Context<T>* context)
{
    Cost<T> cost;

    const T l2Regularization = m->config["l2Regularization"].template Value<T>();

    if (l2Regularization > 0) {
        cost.Add(m->varSolveControls->Evaluate(context), l2Regularization, "l2Regularization");
    }

    const T symmetricRegularization = m->config["symmetry"].template Value<T>();
    if (symmetricRegularization > 0 && m->symmetricControlsMatrix.rows() > 0) {
        DiffData<T> solveControls = m->varSolveControls->Evaluate(context);
        if (m->rigLogicSolveControls) {
            // if we use a higher level solve control then we need to evaluate it here
            solveControls = m->rigLogicSolveControls->EvaluateGuiControls(solveControls) + DiffData<T>(m->baseGuiControls);
        }
        const VectorConstPtr<T> symmetricActivationResidual = std::make_shared<Vector<T>>(m->symmetricControlsMatrix * solveControls.Value());
        if (solveControls.HasJacobian()) {
            cost.Add(DiffData<T>(symmetricActivationResidual, solveControls.Jacobian().Premultiply(m->symmetricControlsMatrix)), symmetricRegularization, "rig_symmetry");
        } else {
            cost.Add(DiffData<T>(symmetricActivationResidual), symmetricRegularization, "symmetricRegularization");
        }
    }

    return cost;
}

template <class T>
const Configuration& DeformationModelRigLogic<T>::GetConfiguration() const
{
    return m->config;
}

template <class T>
void DeformationModelRigLogic<T>::SetConfiguration(const Configuration& config)
{
    m->config.Set(config);
}


template <class T>
void DeformationModelRigLogic<T>::SetRig(std::shared_ptr<const Rig<T>> rig)
{
    m->rig = rig;;
    m->defModelRigid.SetVertices(m->rig->GetRigGeometry()->GetMesh(/*meshIndex=*/0).Vertices());

    SetRigLogicSolveControls(m->rigLogicSolveControls);
}

template <class T>
void DeformationModelRigLogic<T>::SetRigLogicSolveControls(std::shared_ptr<const RigLogicSolveControls<T>> rigLogicSolveControls)
{
    if (m->rigLogicSolveControls == rigLogicSolveControls && m->varSolveControls) {
        return;
    }

    m->rigLogicSolveControls = rigLogicSolveControls;
    if (m->rigLogicSolveControls) {
        m->varSolveControls = std::make_unique<BoundedVectorVariable<T>>(m->rigLogicSolveControls->NumSolveControls());
        m->varSolveControls->SetZero();
        m->varSolveControls->SetBounds(m->rigLogicSolveControls->SolveControlRanges());
        m->varSolveControls->EnforceBounds(true);
        m->controlsToOptimize = std::vector<bool>(m->varSolveControls->Size(), true);
    } else {
        // no solve controls, so we use the rig directly
        m->varSolveControls = std::make_unique<BoundedVectorVariable<T>>(m->rig->GetRigLogic()->NumGUIControls());
        m->varSolveControls->SetZero();
        m->varSolveControls->SetBounds(m->rig->GetRigLogic()->GuiControlRanges());
        m->varSolveControls->EnforceBounds(true);
        m->controlsToOptimize = std::vector<bool>(m->varSolveControls->Size(), true);
    }
}

template <class T>
void DeformationModelRigLogic<T>::SetRigidTransformation(const Affine<T, 3, 3>& affine)
{
    m->defModelRigid.SetRigidTransformation(affine);
}

template <class T>
Affine<T, 3, 3> DeformationModelRigLogic<T>::RigidTransformation() const
{
    return m->defModelRigid.RigidTransformation();
}

template <class T>
Eigen::Matrix<T, 3, -1> DeformationModelRigLogic<T>::DeformedVertices(int meshIndex)
{
    return EvaluateVertices(nullptr, /*lod=*/0, meshIndex, /*withRigid=*/false).Matrix();
}

template <class T>
Eigen::VectorX<T> DeformationModelRigLogic<T>::GuiControls() const
{
    if (m->rigLogicSolveControls) {
        const Eigen::VectorX<T> solveControls = SolveControls();
        const Eigen::VectorX<T> levelGuiControls = m->rigLogicSolveControls->EvaluateGuiControls(DiffData<T>(solveControls)).Value();
        Eigen::VectorX<T> guiValues = m->baseGuiControls;
        for (const int guiIndex : m->rigLogicSolveControls->UsedGuiControls()) {
            guiValues[guiIndex] = levelGuiControls[guiIndex];
        }
        return guiValues;
    } else {
        return SolveControls();
    }

}


template <class T>
void DeformationModelRigLogic<T>::SetGuiControls(const Eigen::VectorX<T>& guiControls)
{
    if (m->rigLogicSolveControls) {
        m->baseGuiControls = guiControls;
        std::vector<int> inconsistentSolveControls;
        const Eigen::VectorX<T> solveControls = m->rigLogicSolveControls->SolveControlsFromGuiControls(guiControls, inconsistentSolveControls);
        SetSolveControls(solveControls);
        // set all controls that are modified by the solve control to zero so that we can add the values
        for (const int guiIndex : m->rigLogicSolveControls->UsedGuiControls()) {
            m->baseGuiControls[guiIndex] = 0;
        }
    } else {
        SetSolveControls(guiControls);
    }
}


template <class T>
const Eigen::VectorX<T>& DeformationModelRigLogic<T>::SolveControls() const
{
    if (!m->varSolveControls) {
        CARBON_CRITICAL("no rig set");
    }
    return m->varSolveControls->Value();
}


template <class T>
void DeformationModelRigLogic<T>::SetSolveControls(const Eigen::VectorX<T>& controls)
{
    if (!m->varSolveControls) {
        CARBON_CRITICAL("no rig set");
    }
    if (int(controls.size()) != m->varSolveControls->Size()) {
        CARBON_CRITICAL("invalid size for controls: {} instead of the expected {}", controls.size(), m->varSolveControls->Size());
    }
    m->varSolveControls->Set(controls);
}


template <class T>
const std::vector<std::string>& DeformationModelRigLogic<T>::SolveControlNames() const
{
    if (m->rigLogicSolveControls) {
        return m->rigLogicSolveControls->SolveControlNames();
    } else {
        return m->rig->GetRigLogic()->GuiControlNames();
    }
}

template <class T>
const Eigen::Matrix<T, 2, -1>& DeformationModelRigLogic<T>::SolveControlRanges() const
{
    if (m->rigLogicSolveControls) {
        return m->rigLogicSolveControls->SolveControlRanges();
    } else {
        return m->rig->GetRigLogic()->GuiControlRanges();
    }
}


template <class T>
const std::vector<bool>& DeformationModelRigLogic<T>::SolveControlsToOptimize() const
{
    return m->controlsToOptimize;
}


template <class T>
void DeformationModelRigLogic<T>::SetSolveControlsToOptimize(const std::vector<bool>& controlsToOptimize)
{
    if (controlsToOptimize.size() != m->controlsToOptimize.size()) {
        CARBON_CRITICAL("array for which controls to optimize does not match number of controls");
    }
    m->controlsToOptimize = controlsToOptimize;

    std::vector<int> constantIndices;
    for (int i = 0; i < int(m->controlsToOptimize.size()); ++i) {
        if (!m->controlsToOptimize[i]) {
            constantIndices.push_back(i);
        }
    }
    m->varSolveControls->MakeIndividualIndicesConstant(constantIndices);
}

template <class T>
BoundedVectorVariable<T>* DeformationModelRigLogic<T>::SolveControlVariable()
{
    return m->varSolveControls.get();
}

template <class T>
int DeformationModelRigLogic<T>::MeshIndex(const char* meshName) const
{
    for (int i = 0; i < m->rig->GetRigGeometry()->NumMeshes(); ++i) {
        if (m->rig->GetRigGeometry()->GetMeshName(i) == std::string(meshName)) {
            return i;
        }
    }
    return -1;
}

template <class T>
int DeformationModelRigLogic<T>::LeftEyeMeshIndex() const
{
    return MeshIndex("eyeLeft_lod0_mesh");
}

template <class T>
int DeformationModelRigLogic<T>::RightEyeMeshIndex() const
{
    return MeshIndex("eyeRight_lod0_mesh");
}

template<class T>
int DeformationModelRigLogic<T>::TeethMeshIndex() const
{
    return MeshIndex("teeth_lod0_mesh");
}


template <class T>
const std::vector<std::tuple<std::string, std::string, T>>& DeformationModelRigLogic<T>::SymmetricControls() const
{
    return m->symmetricControls;
}

template <class T>
void DeformationModelRigLogic<T>::SetSymmetricControls(const std::vector<std::tuple<std::string, std::string, T>>& symmetricControls)
{
    if (m->symmetricControls != symmetricControls) {
        m->symmetricControls = symmetricControls;
        const std::vector<std::string>& guiControlNames = m->rig->GetRigLogic()->GuiControlNames();

        // create the symmetric controls matrix that creates the constraint
        m->symmetricControlsMatrix.resize(symmetricControls.size(), m->rig->GetRigLogic()->NumGUIControls());
        std::vector<Eigen::Triplet<T>> triplets;
        for (int i = 0; i < static_cast<int>(symmetricControls.size()); ++i) {
            std::string name1;
            std::string name2;
            T weight;
            std::tie(name1, name2, weight) = symmetricControls[i];
            auto it1 = std::find(guiControlNames.begin(), guiControlNames.end(), name1);
            auto it2 = std::find(guiControlNames.begin(), guiControlNames.end(), name2);
            if (it1 != guiControlNames.end() && it2 != guiControlNames.end()) {
                const int id1 = static_cast<int>(std::distance(guiControlNames.begin(), it1));
                const int id2 = static_cast<int>(std::distance(guiControlNames.begin(), it2));
                triplets.push_back(Eigen::Triplet<T>(i, id1, std::sqrt(weight)));
                triplets.push_back(Eigen::Triplet<T>(i, id2, -std::sqrt(weight)));
            } else {
                if (it1 == guiControlNames.end()) {
                    LOG_WARNING("{} is not part of the rig", name1);
                }
                if (it2 == guiControlNames.end()) {
                    LOG_WARNING("{} is not part of the rig", name2);
                }
            }
        }
        m->symmetricControlsMatrix.setFromTriplets(triplets.begin(), triplets.end());
    }
}

// explicitly instantiate the DeformationModelRigLogic classes
template class DeformationModelRigLogic<float>;
template class DeformationModelRigLogic<double>;

} // namespace epic::nls
