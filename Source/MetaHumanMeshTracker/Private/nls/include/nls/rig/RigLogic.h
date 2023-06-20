// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/Context.h>
#include <nls/DiffData.h>
#include <nls/math/Math.h>

#include <vector>

namespace dna {
class StreamReader;
}

namespace epic::nls {

/**
 * RigLogic implements rig logic including Jacobian.
 *
 * Opens:
 *  - TODO: animated map clamping needs to be fixed, once clamping in RigLogic has been fixed
 *  - TODO: the clamping scheme needs to be investigated: when using clamps the Jacobians would become 0, which would hurt any optimization. currently
 *    Jacobians are not supported when clamping is enabled
*/
template <class T>
class RigLogic
{
public:
	RigLogic();
    ~RigLogic();
    RigLogic(RigLogic&&);
    RigLogic(RigLogic&) = delete;
    RigLogic& operator=(RigLogic&&);
    RigLogic& operator=(const RigLogic&) = delete;

    std::shared_ptr<RigLogic> Clone() const;

    //! Initializes RigLogic with the data from the dna::StreamReader
    bool Init(dna::StreamReader* reader, bool withJointScaling = false);

    //! @return the number of gui controls of the rig
    int NumGUIControls() const;

    //! @return the number of raw controls of the rig
    int NumRawControls() const;

    //! @return the number of psd controls of the rig
    int NumPsdControls() const;

    //! @return the number of total output controls of the rig i.e. sum of raw and psd controls, and the side of the data that is outpuut from EvaluatePSD()
    int NumTotalControls() const;

    /**
     * Evaluate the raw controls given the input gui controls. Throws an expection is the size is incorrect.
     * There is no clamping involved, any gui controls exceeding their range will extrapolate the values.
     */
    DiffData<T> EvaluateRawControls(const DiffData<T>& guiControls) const;

    /**
     * Evaluate the PSD matrix from raw controls. Throws an expection is the size is incorrect.
     * You would typically call it with the output from EvaluateRawControls().
     */
    DiffData<T> EvaluatePSD(const DiffData<T>& rawControls) const;

    // /**
    //  * Evaluate the blendshape coefficients from the psd values. Throws an expection is the size is incorrect.
    //  * You would typically call it with the output from EvaluatePSD().
    //  *
    //  * @note currently disabled as RigGeometry evaluates blendshapes based on psdControls directly.
    //  */
    // DiffData<T> EvaluateBlendshapes(const DiffData<T>& psdControls, int lod) const;

    /**
     * Evaluate the joint values from the psd values. Throws an expection is the size is incorrect.
     * You would typically call it with the output from EvaluatePSD().
     */
    DiffData<T> EvaluateJoints(const DiffData<T>& psdControls, int lod) const;

    /**
     * Evaluate the animated map coefficients from the psd values. Throws an expection is the size is incorrect.
     * You would typically call it with the output from EvaluatePSD().
     */
    DiffData<T> EvaluateAnimatedMaps(const DiffData<T>& psdControls, int lod) const;

    /**
     * @return the names of the GUI controls.
     */
    const std::vector<std::string>& GuiControlNames() const;

    /**
     * @return the names of the raw controls.
     */
    const std::vector<std::string>& RawControlNames() const;

    /**
     * @return the range of the GUI controls
     */
    const Eigen::Matrix<T, 2, -1>& GuiControlRanges() const;

    /**
     * @return the number of LODs in the rig.
     */
    int NumLODs() const;

    /**
     * @return the matrix mapping psd controls to raw controls     *
     */
    const SparseMatrix<T>& PsdToRawMap() const;

    //! Remove all LODs besides the highest.
    void ReduceToLOD0Only();

    //! @return whether joint scaling is output by RigLogic
    bool WithJointScaling() const;

    /**
     * The PSD values for all expressions (raw controls, and each corretive expression) of the rig sorted by the number of raw
     * controls that affect the expression.
     * @return vector with tuples containing the number of raw controls affecting the expression, the index into the PSD output (EvaluatePSD())
     * for the expression, and the psd control activations.
     */
    std::vector<std::tuple<int, int, Eigen::VectorX<T>>> GetAllExpressions() const;

    /**
     * @brief Remove joints from RigLogic. This is being together with simplification in RigGeometry.
     */
    void RemoveJoints(const std::vector<int>& newToOldJointMapping);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
