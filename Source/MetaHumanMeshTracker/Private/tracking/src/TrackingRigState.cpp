// Copyright Epic Games, Inc. All Rights Reserved.

#include <tracking/TrackingRigState.h>

namespace epic::nls {

template <class T>
TrackingRigState<T>::TrackingRigState(const TrackingRig<T>& trackingRig)
{
    for (size_t solveControlSetIndex = 0; solveControlSetIndex < trackingRig.GetNumSolveControlSets(); ++solveControlSetIndex) {
        SolveControlSetState state;
        state.solveControlValues = trackingRig.GetDefaultSolveControlValues(solveControlSetIndex);
        state.controlsToOptimize = std::vector<bool>(trackingRig.GetNumSolveControls(solveControlSetIndex), true);
        m_solveControlSetStates.emplace_back(std::move(state));
    }
}

template <class T>
TrackingRigState<T>::~TrackingRigState()
{
}

template <class T>
const Eigen::VectorX<T>& TrackingRigState<T>::RigGuiControlValues() const
{
    return m_solveControlSetStates.back().solveControlValues;
}

template <class T>
const Eigen::VectorX<T>& TrackingRigState<T>::RigSolveControlValues(size_t solveControlSetIndex) const
{
    if (solveControlSetIndex >= m_solveControlSetStates.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }
    return m_solveControlSetStates[solveControlSetIndex].solveControlValues;
}

template <class T>
const std::vector<int>& TrackingRigState<T>::InconsistentRigSolveControls(size_t solveControlSetIndex) const
{
    if (solveControlSetIndex >= m_solveControlSetStates.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }
    return m_solveControlSetStates[solveControlSetIndex].inconsistentSolveControlValues;
}

template <class T>
const std::vector<bool>& TrackingRigState<T>::RigSolveControlsToOptimize(size_t solveControlSetIndex) const
{
    if (solveControlSetIndex >= m_solveControlSetStates.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }
    return m_solveControlSetStates[solveControlSetIndex].controlsToOptimize;
}

template <class T>
const std::vector<int>& TrackingRigState<T>::InconsistentRigSolveControlsToOptimize(size_t solveControlSetIndex) const
{
    if (solveControlSetIndex >= m_solveControlSetStates.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }
    return m_solveControlSetStates[solveControlSetIndex].inconsistentSolveControlsToOptimize;
}

template <class T>
void TrackingRigState<T>::SetRigGuiControlValues(const Eigen::VectorX<T>& values, const TrackingRig<T>& trackingRig)
{
    if (GetNumSolveControlSets() != trackingRig.GetNumSolveControlSets()) {
        CARBON_CRITICAL("number of solver states does not match tracking rig");
    }

    if (m_solveControlSetStates.back().solveControlValues.size() != values.size()) {
        CARBON_CRITICAL("gui control vector size incorrect");
    }
    m_solveControlSetStates.back().solveControlValues = values;

    // update the solve control sets based on the current Gui controls.
    for (size_t i = 0; i < m_solveControlSetStates.size() - 1; ++i) {
        auto rigLogicSolveControls = trackingRig.GetRigLogicSolveControls(i);
        CARBON_ASSERT(rigLogicSolveControls, "all besides the last solve control set need to contain rig logic solve controls");
        if (rigLogicSolveControls) {
            m_solveControlSetStates[i].solveControlValues = rigLogicSolveControls->SolveControlsFromGuiControls(values, m_solveControlSetStates[i].inconsistentSolveControlValues);
        }
    }
}

template <class T>
void TrackingRigState<T>::SetRigSolveControlValues(size_t solveControlSetIndex, const Eigen::VectorX<T>& values, const TrackingRig<T>& trackingRig)
{
    if (solveControlSetIndex >= m_solveControlSetStates.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }

    if (GetNumSolveControlSets() != trackingRig.GetNumSolveControlSets()) {
        CARBON_CRITICAL("number of solver states does not match tracking rig");
    }

    if (m_solveControlSetStates[solveControlSetIndex].solveControlValues.size() != values.size()) {
        CARBON_CRITICAL("vector size does not match the number of controls");
    }

    if (IsGuiControls(solveControlSetIndex)) {
        SetRigGuiControlValues(values, trackingRig);
    } else {
        m_solveControlSetStates[solveControlSetIndex].solveControlValues = values;
        auto rigLogicSolveControls = trackingRig.GetRigLogicSolveControls(solveControlSetIndex);
        CARBON_ASSERT(rigLogicSolveControls, "all besides the last solve control set need to contain rig logic solve controls");
        if (rigLogicSolveControls) {
            // only evaluate the controls of this set and then set Gui controls
            const Eigen::VectorX<T> perSetGuiControls = rigLogicSolveControls->EvaluateGuiControls(values).Value();
            Eigen::VectorX<T> newGuiControls = RigGuiControlValues();
            for (int i : rigLogicSolveControls->UsedGuiControls()) {
                newGuiControls[i] = perSetGuiControls[i];
            }
            SetRigGuiControlValues(newGuiControls, trackingRig);
        }
    }
}


template <class T>
void TrackingRigState<T>::ResetRigSolveControlValues(size_t solveControlSetIndex, bool resetNextSets, const TrackingRig<T>& trackingRig)
{
    if (solveControlSetIndex >= m_solveControlSetStates.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }

    if (GetNumSolveControlSets() != trackingRig.GetNumSolveControlSets()) {
        CARBON_CRITICAL("number of solver states does not match tracking rig");
    }

    if (IsGuiControls(solveControlSetIndex)) {
        // if we reset the full rig then we just set all controls to default
        SetRigGuiControlValues(trackingRig.GetDefaultGuiControlValues(), trackingRig);
        return;
    }

    if (resetNextSets) {
        Eigen::VectorX<T> newGuiControls = trackingRig.GetDefaultGuiControlValues();
        // evaluate the prior controls but not including this solve control set
        for (size_t setIndex = 0; setIndex < solveControlSetIndex; ++setIndex) {
            auto rigLogicSolveControls = trackingRig.GetRigLogicSolveControls(setIndex);
            CARBON_ASSERT(rigLogicSolveControls, "all besides the last solve control set need to contain rig logic solve controls");
            const Eigen::VectorX<T> perSetGuiControls = rigLogicSolveControls->EvaluateGuiControls(RigSolveControlValues(setIndex)).Value();
            for (int i : rigLogicSolveControls->UsedGuiControls()) {
                newGuiControls[i] = perSetGuiControls[i];
            }
        }
        SetRigGuiControlValues(newGuiControls, trackingRig);
    } else {
        // reset only the solve controls for set @p solveControlSetIndex
        SetRigSolveControlValues(solveControlSetIndex, trackingRig.GetDefaultSolveControlValues(solveControlSetIndex), trackingRig);
    }
}


template <class T>
void TrackingRigState<T>::SetRigSolveControlsToOptimize(size_t solveControlSetIndex, const std::vector<bool>& controlsToOptimize, const TrackingRig<T>& trackingRig)
{
    const size_t numControlsToOptimize = static_cast<int>(controlsToOptimize.size());

    if (solveControlSetIndex >= m_solveControlSetStates.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }

    if (numControlsToOptimize != size_t(m_solveControlSetStates[solveControlSetIndex].solveControlValues.size())) {
        CARBON_CRITICAL("invalid vector size for controls to optimize");
    }

    if (m_solveControlSetStates.size() != trackingRig.GetNumSolveControlSets()) {
        CARBON_CRITICAL("tracking rig state does not match tracking rig");
    }

    if (numControlsToOptimize != size_t(trackingRig.GetDefaultSolveControlValues(solveControlSetIndex).size())) {
        CARBON_CRITICAL("number of controls to optimize does not match the number of controls in the tracking rig");
    }

    if (m_solveControlSetStates.size() != trackingRig.GetNumSolveControlSets()) {
        CARBON_CRITICAL("number of solver states does not match tracking rig");
    }

    m_solveControlSetStates[solveControlSetIndex].controlsToOptimize = controlsToOptimize;
    if (IsGuiControls(solveControlSetIndex)) {
        // update the solver control sets based on the optimization flags of the gui controls
        for (size_t setIndex = 0; setIndex < m_solveControlSetStates.size() - 1; ++setIndex) {
            m_solveControlSetStates[setIndex].inconsistentSolveControlsToOptimize.clear();
            for (size_t i = 0; i < m_solveControlSetStates[setIndex].controlsToOptimize.size(); ++i) {
                const size_t numAffectedGuiControls = trackingRig.GetAffectedGuiControls(setIndex)[i].size();
                size_t count = 0;
                for (size_t j = 0; j < numAffectedGuiControls; ++j) {
                    if (controlsToOptimize[trackingRig.GetAffectedGuiControls(setIndex)[i][j]]) {
                        count++;
                    }
                }
                if (count == 0) {
                    m_solveControlSetStates[setIndex].controlsToOptimize[i] = false;
                } else if (count == numAffectedGuiControls) {
                    m_solveControlSetStates[setIndex].controlsToOptimize[i] = true;
                } else {
                    // don't change whether the solve control set is optimized as it is inconsistent anyhow
                    m_solveControlSetStates[setIndex].inconsistentSolveControlsToOptimize.push_back(static_cast<int>(i));
                }
            }
       }
    } else {
        // update all Gui controls that are affected by this control set, and then call this method for the Gui controls
        std::vector<bool> guiControlsToOptimize = m_solveControlSetStates.back().controlsToOptimize;
        for (size_t i = 0; i < numControlsToOptimize; ++i) {
            for (size_t j = 0; j < trackingRig.GetAffectedGuiControls(solveControlSetIndex)[i].size(); ++j) {
                guiControlsToOptimize[trackingRig.GetAffectedGuiControls(solveControlSetIndex)[i][j]] = controlsToOptimize[i];
            }
        }
        SetRigSolveControlsToOptimize(m_solveControlSetStates.size() - 1, guiControlsToOptimize, trackingRig);
    }
}

// explicitly instantiate the TrackingRigState classes
template class TrackingRigState<float>;
template class TrackingRigState<double>;

} // namespace epic::nls
