// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <tracking/TrackingRig.h>

#include <vector>

namespace epic::nls {

/**
 * Defines the state for the instance of a TrackingRig.
 */
template <class T>
class TrackingRigState
{
public:
    explicit TrackingRigState(const TrackingRig<T>&);
    ~TrackingRigState();

    //! @returns the number of solve control sets (at least one, the last one is all gui controls)
    size_t GetNumSolveControlSets() const { return m_solveControlSetStates.size(); }

    const Eigen::VectorX<T>& RigGuiControlValues() const;
    const Eigen::VectorX<T>& RigSolveControlValues(size_t solveControlSetIndex) const;
    const std::vector<int>& InconsistentRigSolveControls(size_t solveControlSetIndex) const;
    const std::vector<bool>& RigSolveControlsToOptimize(size_t solveControlSetIndex) const;
    const std::vector<int>& InconsistentRigSolveControlsToOptimize(size_t solveControlSetIndex) const;

    /**
     * Sets the gui controls of the TrackingRigState and updates for each solve control set whether there are inconsistencies.
     * @see SolveControlSetState::inconsistentSolveControlValues
     */
    void SetRigGuiControlValues(const Eigen::VectorX<T>& values, const TrackingRig<T>& trackingRig);

    void SetRigSolveControlValues(size_t solveControlSetIndex, const Eigen::VectorX<T>& values, const TrackingRig<T>& trackingRig);

    void ResetRigSolveControlValues(size_t solveControlSetIndex, bool resetNextSets, const TrackingRig<T>& trackingRig);

    /**
     * Set which controls to optimize for set @p solveControlSetIndex. Updates the full Gui controls (last set) based on the input, and then
     * updates all solver sets based on the Gui control set. Also updates whether there are any inconsistent solve controls to optimize.
     * @see SolveControlSetState::inconsistentSolveControlsToOptimize
     * @throw std::runtime_error for invalid solveControlSetIndex or if controlsToOptimize does match the size of the number of controls.
     */
    void SetRigSolveControlsToOptimize(size_t solveControlSetIndex, const std::vector<bool>& controlsToOptimize, const TrackingRig<T>& trackingRig);

    //! The last control set is the Gui controls
    bool IsGuiControls(size_t solveControlSetIndex) const { return (solveControlSetIndex == m_solveControlSetStates.size() - 1); }

private:
    struct SolveControlSetState {
        //! the current solve control values
        Eigen::VectorX<T> solveControlValues;

        //! indices of solve controls that are inconsistent with the GUI controls
        std::vector<int> inconsistentSolveControlValues;

        //! flag per solve control whether it should be optimized
        std::vector<bool> controlsToOptimize;

        /**
         * Indices of solve controls to optimize that are not consistent with the GUI optimization
         */
        std::vector<int> inconsistentSolveControlsToOptimize;
    };
    //! rig logic solve control sets
    std::vector<SolveControlSetState> m_solveControlSetStates;
};

} // namespace epic::nls
