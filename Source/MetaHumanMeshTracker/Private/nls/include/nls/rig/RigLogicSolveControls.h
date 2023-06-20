// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <carbon/io/JsonIO.h>
#include <nls/DiffData.h>
#include <nls/math/Math.h>
#include <nls/rig/RigLogic.h>

namespace epic::nls {

/**
 * Higher level solve controls that map to GUI controls of RigLogic.
 */
template <class T>
class RigLogicSolveControls
{
public:
	RigLogicSolveControls();
	~RigLogicSolveControls();
    RigLogicSolveControls(RigLogicSolveControls&&);
    RigLogicSolveControls(RigLogicSolveControls&) = delete;
    RigLogicSolveControls& operator=(RigLogicSolveControls&&);
    RigLogicSolveControls& operator=(const RigLogicSolveControls&) = delete;

    //! Initializes RigLogicSolveControls with the rig logic reference and a json description of the solve controls
    bool Init(const RigLogic<T>& rigLogicReference, const carbon::JsonElement& rigLogicSolveControlJson);

    //! @returns the name of the riglogic solve control set
    const std::string& Name() const;

    //! @returns the number of solve controls of the rig
    int NumSolveControls() const;

    /**
     * Evaluate the GUI controls given the solve controls.
     */
    DiffData<T> EvaluateGuiControls(DiffData<T> solveControls) const;

    /**
     * @returns the names of the Solve controls.
     */
    const std::vector<std::string>& SolveControlNames() const;

    /**
     * @returns the range of the Solve controls
     */
    const Eigen::Matrix<T, 2, -1>& SolveControlRanges() const;

    /**
     * Calculate solve controls from Gui Controls.
     * @param[out] inconsistentSolveControls  Marks all solve controls that are not consistent in the mapping.
     */
    Eigen::VectorX<T> SolveControlsFromGuiControls(const Eigen::VectorX<T>& guiControls, std::vector<int>& inconsistentSolveControls) const;

    //! @returns all gui controls that are triggered by the solve controls
    const std::vector<int>& UsedGuiControls() const;

    //! @returns per solve control which gui control is affected
    std::vector<std::vector<int>> UsedGuiControlsPerSolveControl() const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
