// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/rig/RigLogic.h>

#include <nls/VectorVariable.h>
#include <nls/utils/FileIO.h>
#include <carbon/io/JsonIO.h>
#include <carbon/utils/Profiler.h>

#include <riglogic/RigLogic.h>

#include <string>
#include <vector>

namespace epic::nls {

template <class T>
struct RigLogic<T>::Private
{
    int numLODs = 0;

    int guiControlCount = 0;
    int rawControlCount = 0;
    int psdControlCount = 0;
    int totalControlCount = 0;

    std::vector<std::string> guiControlNames;
    std::vector<std::string> rawControlNames;

    // ### GUI to Raw mapping ###
    // mapping from input (rowIndex=0) to output (rowIndex=1) for GUI controls (columns)
    Eigen::Matrix<int, 2, -1> guiToRawControlMapping;
    // from(rowIndex=0), to(rowIndex=1), slope(rowIndex=2), cut(rowIndex=3), for each GUI control (columns)
    Eigen::Matrix<T, 4, -1> guiToRawControlValues;
    // the ranges for each gui control
    Eigen::Matrix<T, 2, -1> guiControlRanges;

    // ### psd matrix ###
    SparseMatrix<T> psdToRawMap;

    // ### blendshapes ###
    Eigen::VectorX<int> blendshapesPerLOD;
    Eigen::Matrix<int, 2, -1> blendshapeMapping;

    // ### joints ###
    int numJoints = 0;
    bool withJointScaling = false;
    std::vector<SparseMatrix<T>> jointMatrixPerLOD;

    // ### aniated maps ###
    int numAnimatedMaps = 0;
    Eigen::VectorX<int> animatedMapsPerLOD;
    // mapping from input (rowIndex=0) to output (rowIndex=1) for each animated map (columns)
    Eigen::Matrix<int, 2, -1> animatedMapsMapping;
    // from(rowIndex=0), to(rowIndex=1), slope(rowIndex=2), cut(rowIndex=3), for each animated map (columns)
    Eigen::Matrix<T, 4, -1> animatedMapsValues;
};


template <class T>
RigLogic<T>::RigLogic() : m(std::make_unique<Private>())
{
}


template <class T> RigLogic<T>::~RigLogic() = default;
template <class T> RigLogic<T>::RigLogic(RigLogic&&) = default;
template <class T> RigLogic<T>& RigLogic<T>::operator=(RigLogic&&) = default;

template <class T>
std::shared_ptr<RigLogic<T>> RigLogic<T>::Clone() const
{
    std::shared_ptr<RigLogic<T>> clone = std::make_shared<RigLogic>();
    *clone->m = *m;
    return clone;
}

template <class T>
bool RigLogic<T>::WithJointScaling() const
{
    return m->withJointScaling;
}

template <class T>
bool RigLogic<T>::Init(dna::StreamReader* reader, bool withJointScaling)
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    m->numLODs = reader->getLODCount();
    m->withJointScaling = withJointScaling;

    m->guiControlCount = reader->getGUIControlCount();
    m->rawControlCount = reader->getRawControlCount();
    m->psdControlCount = reader->getPSDCount();
    m->totalControlCount = m->rawControlCount + m->psdControlCount;

    m->guiControlNames.clear();
    for (int i = 0; i < m->guiControlCount; ++i) {
        m->guiControlNames.push_back(reader->getGUIControlName(std::uint16_t(i)).c_str());
    }

    m->rawControlNames.clear();
    for (int i = 0; i < m->rawControlCount; ++i) {
        m->rawControlNames.push_back(reader->getRawControlName(std::uint16_t(i)).c_str());
    }

    // setup gui to raw calculation
    const int numGuiToRawAssignments = int(reader->getGUIToRawInputIndices().size());
    m->guiToRawControlMapping.resize(2, numGuiToRawAssignments);
    m->guiToRawControlValues.resize(4, numGuiToRawAssignments);
    m->guiControlRanges = Eigen::Matrix<T, 2, -1>(2, m->guiControlCount);
    m->guiControlRanges.row(0).setConstant(1e6f);
    m->guiControlRanges.row(1).setConstant(-1e6f);

    std::vector<bool> guiControlUsed(m->guiControlCount, false);
    std::vector<bool> rawControlUsed(m->rawControlCount, false);

    for (int i = 0; i < numGuiToRawAssignments; i++) {
        const int inputIndex = reader->getGUIToRawInputIndices()[i];
        const int outputIndex = reader->getGUIToRawOutputIndices()[i];
        if (inputIndex < 0 || inputIndex >= m->guiControlCount) {
            CARBON_CRITICAL("gui control input index is invalid");
        }
        if (outputIndex < 0 || outputIndex >= m->rawControlCount) {
            CARBON_CRITICAL("gui control output index is invalid");
        }
        m->guiToRawControlMapping(0, i) = inputIndex;
        m->guiToRawControlMapping(1, i) = outputIndex;
        T from = reader->getGUIToRawFromValues()[i];
        T to = reader->getGUIToRawToValues()[i];
        if (from > to) std::swap(from, to);
        m->guiToRawControlValues(0, i) = from;
        m->guiToRawControlValues(1, i) = to;
        m->guiToRawControlValues(2, i) = reader->getGUIToRawSlopeValues()[i];
        m->guiToRawControlValues(3, i) = reader->getGUIToRawCutValues()[i];
        // printf("gui mapping %s %d to %d: [%f %f] [%f %f]\n", m->guiControlNames[inputIndex].c_str(), inputIndex, outputIndex, m->guiToRawControlValues(0, i), m->guiToRawControlValues(1, i),
        //     m->guiToRawControlValues(2, i), m->guiToRawControlValues(3, i));
        m->guiControlRanges(0, inputIndex) = std::min<T>(m->guiControlRanges(0, inputIndex), from);
        m->guiControlRanges(1, inputIndex) = std::max<T>(m->guiControlRanges(1, inputIndex), to);
        guiControlUsed[inputIndex] = true;
        rawControlUsed[outputIndex] = true;
    }
    // for (int i = 0; i < m->guiControlCount; i++) {
    //     LOG_INFO("range {}: [{}, {}]", m->guiControlNames[i], m->guiControlRanges(0, i), m->guiControlRanges(1, i));
    // }

    for (int i = 0; i < m->guiControlCount; ++i) {
        if (!guiControlUsed[i]) {
            CARBON_CRITICAL("not all gui controls are being used");
        }
    }
    if (m->guiControlCount > 0) {
        for (int i = 0; i < m->rawControlCount; ++i) {
            if (!rawControlUsed[i]) {
                LOG_WARNING("raw control {} {} is not mapped by gui controls", RawControlNames()[i], i);
            }
        }
    }

    // setup psd calculation
    m->psdToRawMap = SparseMatrix<T>(m->psdControlCount, m->rawControlCount);

    std::vector<Eigen::Triplet<T>> psdToRawMapTriplets;
    for (int j = 0; j < int(reader->getPSDColumnIndices().size()); j++) {
        int row = reader->getPSDRowIndices()[j] - m->rawControlCount;
        if (row < 0 || row >= m->psdControlCount) {
            CARBON_CRITICAL("psd control mapping invalid");
        }
        int col = reader->getPSDColumnIndices()[j];
        if (col < 0 || col >= m->rawControlCount) {
            CARBON_CRITICAL("psd control mapping invalid");
        }

        psdToRawMapTriplets.push_back(Eigen::Triplet<T>(row, col, reader->getPSDValues()[j]));
    }
    m->psdToRawMap.setFromTriplets(psdToRawMapTriplets.begin(), psdToRawMapTriplets.end());


    // setup blendshape mapping
    m->blendshapesPerLOD.resize(reader->getLODCount());
    for (int lod = 0; lod < reader->getLODCount(); lod++) {
        m->blendshapesPerLOD[lod] = reader->getBlendShapeChannelLODs()[lod];
    }
    m->blendshapeMapping.resize(2, reader->getBlendShapeChannelCount());
    for (int i = 0; i < reader->getBlendShapeChannelCount(); i++) {
        m->blendshapeMapping(0, i) = reader->getBlendShapeChannelInputIndices()[i];
        m->blendshapeMapping(1, i) = reader->getBlendShapeChannelOutputIndices()[i];
    }

    // setup joints
    m->numJoints = reader->getJointCount();
    if (reader->getJointCount() * 9 != reader->getJointRowCount()) {
        CARBON_CRITICAL("number of joints and joint rows not matching");
    }
    if (m->numJoints > 0 && m->totalControlCount != reader->getJointColumnCount()) {
        CARBON_CRITICAL("number of total controls and joint columns not matching");
    }

    m->jointMatrixPerLOD.resize(m->numLODs);
    for (int lod = 0; lod < m->numLODs; lod++) {
        std::vector<Eigen::Triplet<T>> jointMatrixPerLODTriplets;
        for (std::uint16_t i = 0; i < reader->getJointGroupCount(); i++) {
            rl4::ConstArrayView<std::uint16_t> rowsPerLOD = reader->getJointGroupLODs(i);
            rl4::ConstArrayView<std::uint16_t> jointGroupInputIndices = reader->getJointGroupInputIndices(i);
            rl4::ConstArrayView<std::uint16_t> jointGroupOutputIndices = reader->getJointGroupOutputIndices(i);
            rl4::ConstArrayView<float> jointGroupValues = reader->getJointGroupValues(i);
            for (int j = 0; j < rowsPerLOD[lod]; j++) {
                std::uint16_t jointIndexAndDof = jointGroupOutputIndices[j];
                const std::uint16_t jointIndex = jointIndexAndDof / 9;
                const std::uint16_t dof = jointIndexAndDof % 9;
                if (!m->withJointScaling) {
                    if (dof >= 6) {
                        LOG_INFO("discarding scaling {} for joint {}", dof - 6, reader->getJointName(jointIndex));
                        continue;
                    }
                    jointIndexAndDof = 6 * jointIndex + dof;
                }
                constexpr T deg2rad = T(CARBON_PI / 180.0);
                const T scaling = (dof >= 3 && dof < 6) ? deg2rad : T(1);
                for (int k = 0; k < int(jointGroupInputIndices.size()); k++) {
                    const int valueIndex = j * int(jointGroupInputIndices.size()) + k;
                    const T value = scaling * T(jointGroupValues[valueIndex]);
                    if (fabs(value) > 1e-20) {
                        jointMatrixPerLODTriplets.push_back(Eigen::Triplet<T>(jointIndexAndDof, jointGroupInputIndices[k], value));
                    }
                }
            }
        }

        m->jointMatrixPerLOD[lod].resize(m->numJoints * (m->withJointScaling ? 9 : 6), m->totalControlCount);
        m->jointMatrixPerLOD[lod].setFromTriplets(jointMatrixPerLODTriplets.begin(), jointMatrixPerLODTriplets.end());
    }

    // setup animated maps
    m->numAnimatedMaps = reader->getAnimatedMapCount();

    m->animatedMapsPerLOD = Eigen::VectorX<int>::Zero(m->numLODs);
    if (m->numAnimatedMaps > 0) {
        rl4::ConstArrayView<std::uint16_t> animatedMapLODs = reader->getAnimatedMapLODs();
        if (int(animatedMapLODs.size()) != int(m->animatedMapsPerLOD.size())) {
            CARBON_CRITICAL("animated map lods incorrect");
        }
        for (int i = 0; i < m->numLODs; i++) {
            m->animatedMapsPerLOD[i] = animatedMapLODs[i];
        }
        const int numAnimatedMapAssignments = int(reader->getAnimatedMapInputIndices().size());
        m->animatedMapsMapping.resize(2, numAnimatedMapAssignments);
        m->animatedMapsValues.resize(4, numAnimatedMapAssignments);

        for (int i = 0; i < numAnimatedMapAssignments; i++) {
            const int inputIndex = reader->getAnimatedMapInputIndices()[i];
            const int outputIndex = reader->getAnimatedMapOutputIndices()[i];
            if (inputIndex < 0 || inputIndex >= m->totalControlCount) {
                CARBON_CRITICAL("animated map input index is invalid");
            }
            if (outputIndex < 0 || outputIndex >= m->numAnimatedMaps) {
                CARBON_CRITICAL("animated map output index is invalid");
            }
            m->animatedMapsMapping(0, i) = inputIndex;
            m->animatedMapsMapping(1, i) = outputIndex;
            m->animatedMapsValues(0, i) = reader->getAnimatedMapFromValues()[i];
            m->animatedMapsValues(1, i) = reader->getAnimatedMapToValues()[i];
            m->animatedMapsValues(2, i) = reader->getAnimatedMapSlopeValues()[i];
            m->animatedMapsValues(3, i) = reader->getAnimatedMapCutValues()[i];
            if (m->animatedMapsValues(0, i) > m->animatedMapsValues(1, i)) {
                CARBON_CRITICAL("animated maps mapping needs to have smaller from-value than to-value");
            }
        }
    }

    return true;
}

template <class T>
int RigLogic<T>::NumGUIControls() const
{
    return m->guiControlCount;
}

template <class T>
int RigLogic<T>::NumRawControls() const
{
    return m->rawControlCount;
}

template <class T>
int RigLogic<T>::NumPsdControls() const
{
    return m->psdControlCount;
}

template <class T>
int RigLogic<T>::NumTotalControls() const
{
    return m->totalControlCount;
}


template <class T>
const std::vector<std::string>& RigLogic<T>::GuiControlNames() const
{
    return m->guiControlNames;
}

template <class T>
const std::vector<std::string>& RigLogic<T>::RawControlNames() const
{
    return m->rawControlNames;
}

template <class T>
const Eigen::Matrix<T, 2, -1>& RigLogic<T>::GuiControlRanges() const
{
    return m->guiControlRanges;
}

template <class T>
DiffData<T> RigLogic<T>::EvaluateRawControls(const DiffData<T>& guiControls) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    if (guiControls.Size() != m->guiControlCount) {
        CARBON_CRITICAL("RigLogic::EvaluateRawControls(): guiControls control count incorrect");
    }

    VectorPtr<T> output = std::make_shared<Vector<T>>(m->rawControlCount);
    output->setZero();

    // evaluate GUI controls
    for (int i = 0; i < int(m->guiToRawControlMapping.cols()); i++) {
        const int inputIndex = m->guiToRawControlMapping(0, i);
        const int outputIndex = m->guiToRawControlMapping(1, i);
        const T rangeStart = m->guiControlRanges(0, inputIndex);
        const T rangeEnd = m->guiControlRanges(1, inputIndex);
        const T from = m->guiToRawControlValues(0, i);
        const T to = m->guiToRawControlValues(1, i);
        const T slope = m->guiToRawControlValues(2, i);
        const T cut = m->guiToRawControlValues(3, i);
        const T value = guiControls.Value()[inputIndex];
        const bool belowRange = (from == rangeStart && value < from);
        const bool aboveRange = (to == rangeEnd && value >= to);
        if (from <= value && value < to) {
            // note that the evaluation here is slightly different compared to the original RigLogic implementation:
            // there the condition is (from < value && value <= to). The reason
            // to use this condition here is so that the base analytical Jacobian at "from" is matching forward differentiation
            (*output)[outputIndex] += slope * value + cut;
        } else if (belowRange) {
            (*output)[outputIndex] += slope * from + cut; // clamp to minimum range value
        } else if (aboveRange) {
            (*output)[outputIndex] += slope * to + cut; // clamp to maximum range value
        }
    }

    JacobianConstPtr<T> Jacobian;

    if (guiControls.HasJacobian()) {
        PROFILING_BLOCK("jacobian");
        SparseMatrix<T> localJacobian(m->rawControlCount, guiControls.Size());
        std::vector<Eigen::Triplet<T>> triplets;
        for (int i = 0; i < int(m->guiToRawControlMapping.cols()); i++) {
            const int inputIndex = m->guiToRawControlMapping(0, i);
            const int outputIndex = m->guiToRawControlMapping(1, i);
            const T rangeStart = m->guiControlRanges(0, inputIndex);
            const T rangeEnd = m->guiControlRanges(1, inputIndex);
            const T from = m->guiToRawControlValues(0, i);
            const T to = m->guiToRawControlValues(1, i);
            const T slope = m->guiToRawControlValues(2, i);
            const T value = guiControls.Value()[inputIndex];
            const bool belowRange = (from == rangeStart && value < from);
            const bool aboveRange = (to == rangeEnd && value >= to);
            if (from <= value && value < to) {
                if (slope != 0) {
                    triplets.push_back(Eigen::Triplet<T>(outputIndex, inputIndex, slope));
                }
            } else if (belowRange || aboveRange) {
                // When the GUI control is out of bounds then the raw control is clamped and technically
                // the Jacobian would be zero. However this would mean that any optimization that uses
                // the control would not have an "incentive" to move the control back inside the bounds.
                // Therefore we keep the Jacobian for these bounds, but any optimization needs
                // to enforce that the GUI controls stay within the bounds.
                if (slope != 0) {
                    triplets.push_back(Eigen::Triplet<T>(outputIndex, inputIndex, slope));
                }
            }
        }
        localJacobian.setFromTriplets(triplets.begin(), triplets.end());
        Jacobian = guiControls.Jacobian().Premultiply(localJacobian);
        PROFILING_END_BLOCK;
    }

    return DiffData<T>(output, Jacobian);
}


template <class T>
DiffData<T> RigLogic<T>::EvaluatePSD(const DiffData<T>& rawControls) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    if (rawControls.Size() != m->rawControlCount) {
        CARBON_CRITICAL("raw control count incorrect {} instead of {}", rawControls.Size(), m->rawControlCount);
    }

    if (m->psdControlCount == 0) return rawControls;

    VectorPtr<T> output = std::make_shared<Vector<T>>(m->totalControlCount);
    JacobianConstPtr<T> Jacobian;

    // clamp the input raw controls
    for (int i = 0; i < m->rawControlCount; i++) {
        (*output)[i] = clamp<T>(rawControls.Value()[i], 0, 1);
    }

    for (int k = 0; k < int(m->psdToRawMap.outerSize()); ++k) {
        T weight = T(1);
        for (typename SparseMatrix<T>::InnerIterator it(m->psdToRawMap, k); it; ++it)
        {
            weight *= (*output)[it.col()] * it.value(); // this is guaranteed to be an index into the raw controls that are copied and clamped above
        }
        (*output)[m->rawControlCount + k] = weight;
    }

    if (rawControls.HasJacobian()) {
        PROFILING_BLOCK("jacobian");
        SparseMatrix<T> localJacobian;
        std::vector<Eigen::Triplet<T>> triplets;
        for (int i = 0; i < m->rawControlCount; i++) {
            triplets.push_back(Eigen::Triplet(i, i, T(1)));
        }
        for (int k = 0; k < int(m->psdToRawMap.outerSize()); ++k) {
            const T weight = (*output)[m->rawControlCount + k];
            CARBON_ASSERT(weight >= 0, "raw control outputs can only be 0 or positive, never negative");
            if (weight > 1) {
                // If we are outside of bounds then correctives do not have an impact on the Jacobian
                // as a tiny delta on any of the values will not move the corrective output to within bounds.
                // For the value of 1 we keep the Jacobian valid as a tiny negative delta will change the corrective
                continue;
            }
            // Note that even if the corrective value is zero, the Jacobian can be valid. For example a corrective
            // corr(A, B) = A * B. If A is 1, and B is 0, then the derivative of corr(A, B) with respect to B is A.
            for (typename SparseMatrix<T>::InnerIterator it(m->psdToRawMap, k); it; ++it)
            {
                T accValue = T(1);
                for (typename SparseMatrix<T>::InnerIterator it2(m->psdToRawMap, k); it2; ++it2)
                {
                    if (it.col() != it2.col()) {
                        accValue *= (*output)[it2.col()] * it2.value();
                    } else {
                        accValue *= it2.value();
                    }
                }
                if (accValue > 0) { // we can discard 0 values as it is a sparse matrix, and accValue is never negative as all controls are >= 0
                    triplets.push_back(Eigen::Triplet<T>(m->rawControlCount + k, int(it.col()), accValue));
                }
            }
        }
        localJacobian.resize(m->totalControlCount, m->rawControlCount);
        localJacobian.setFromTriplets(triplets.begin(), triplets.end());
        Jacobian = rawControls.Jacobian().Premultiply(localJacobian);
        PROFILING_END_BLOCK;
    }

    // clamp correctives
    for (int k = 0; k < int(m->psdToRawMap.outerSize()); ++k) {
        (*output)[m->rawControlCount + k] = clamp<T>((*output)[m->rawControlCount + k], 0, 1);
    }

    return DiffData<T>(output, Jacobian);
}


// template <class T>
// DiffData<T> RigLogic<T>::EvaluateBlendshapes(const DiffData<T>& psdControls, int lod) const
// {
//     PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

//     if (psdControls.Size() != m->totalControlCount) {
//         CARBON_CRITICAL("RigLogic::EvaluateBlendshapes(): psd control count incorrect");
//     }
//     if (lod < 0 || lod >= m->numLODs) {
//         CARBON_CRITICAL("RigLogic::EvaluateBlendshapes(): invalid lod");
//     }

//     VectorPtr<T> output = std::make_shared<Vector<T>>(m->blendshapeMapping.cols());
//     output->setZero();

//     for (int i = 0; i < m->blendshapesPerLOD[lod]; i++) {
//         (*output)[m->blendshapeMapping(1, i)] = psdControls.Value()[m->blendshapeMapping(0, i)];
//     }

//     JacobianConstPtr<T> Jacobian;
//     if (psdControls.HasJacobian()) {
//         PROFILING_BLOCK("jacobian");
//         SparseMatrix<T> localJacobian(m->blendshapeMapping.cols(), psdControls.Size());
//         std::vector<Eigen::Triplet<T>> triplets;
//         for (int i = 0; i < m->blendshapesPerLOD[lod]; i++) {
//             triplets.push_back(Eigen::Triplet<T>(m->blendshapeMapping(1, i), m->blendshapeMapping(0, i), T(1)));
//         }
//         localJacobian.setFromTriplets(triplets.begin(), triplets.end());
//         Jacobian = psdControls.Jacobian().Premultiply(localJacobian);
//         PROFILING_END_BLOCK;
//     }

//     return DiffData<T>(output, Jacobian);
// }


template <class T>
DiffData<T> RigLogic<T>::EvaluateJoints(const DiffData<T>& psdControls, int lod) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    if (psdControls.Size() != m->totalControlCount) {
        CARBON_CRITICAL("RigLogic::EvaluateJoints(): psd control count incorrect");
    }
    if (lod < 0 || lod >= m->numLODs) {
        CARBON_CRITICAL("RigLogic::EvaluateJoints(): invalid lod");
    }

    VectorPtr<T> output = std::make_shared<Vector<T>>(m->jointMatrixPerLOD[lod].rows());
    output->noalias() = m->jointMatrixPerLOD[lod] * psdControls.Value();

    JacobianConstPtr<T> Jacobian;
    if (psdControls.HasJacobian()) {
        PROFILING_BLOCK("jacobian");
        Jacobian = psdControls.Jacobian().Premultiply(m->jointMatrixPerLOD[lod]);
        PROFILING_END_BLOCK;
    }

    return DiffData<T>(output, Jacobian);
}


template <class T>
DiffData<T> RigLogic<T>::EvaluateAnimatedMaps(const DiffData<T>& psdControls, int lod) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    if (psdControls.Size() != m->totalControlCount) {
        CARBON_CRITICAL("RigLogic::EvaluateAnimatedMaps(): psd control count incorrect");
    }
    if (lod < 0 || lod >= m->numLODs) {
        CARBON_CRITICAL("RigLogic::EvaluateAnimatedMaps(): invalid lod");
    }

    VectorPtr<T> output = std::make_shared<Vector<T>>(m->numAnimatedMaps);
    output->setZero();

    // evaluate animated maps
    for (int i = 0; i < m->animatedMapsPerLOD[lod]; i++) {
        const int inputIndex = m->animatedMapsMapping(0, i);
        const int outputIndex = m->animatedMapsMapping(1, i);
        const T from = m->animatedMapsValues(0, i);
        const T to = m->animatedMapsValues(1, i);
        const T slope = m->animatedMapsValues(2, i);
        const T cut = m->animatedMapsValues(3, i);
        const T value = psdControls.Value()[inputIndex];
        if (from < value && value <= to) {
            (*output)[outputIndex] = (*output)[outputIndex] + slope * value + cut;
        }
    }
    // replace the clamp within the loop above once it has been fixed in RigLogic
    for (int i = 0; i < m->numAnimatedMaps; i++) {
        (*output)[i] = clamp<T>((*output)[i], T(0), T(1));
    }

    JacobianConstPtr<T> Jacobian;
    if (psdControls.HasJacobian()) {
        PROFILING_BLOCK("jacobian");
        SparseMatrix<T> localJacobian(m->numAnimatedMaps, psdControls.Size());
        std::vector<Eigen::Triplet<T>> triplets;
        for (int i = 0; i < m->animatedMapsPerLOD[lod]; i++) {
            const int inputIndex = m->animatedMapsMapping(0, i);
            const int outputIndex = m->animatedMapsMapping(1, i);
            const T from = m->animatedMapsValues(0, i);
            const T to = m->animatedMapsValues(1, i);
            const T slope = m->animatedMapsValues(2, i);
            const T value = psdControls.Value()[inputIndex];
            if (from < value && value <= to) {
                // note that at this point the value may be clamped and therefore the Jacobian is techincally 0. However this would
                // mean that the value could never be pulled back within bounds
                // const T outputValue = (*output)[outputIndex];
                // if (outputValue >= 0 && outputValue < 1) {
                    triplets.push_back(Eigen::Triplet<T>(outputIndex, inputIndex, slope));
                // }
            }
        }
        localJacobian.setFromTriplets(triplets.begin(), triplets.end());
        Jacobian = psdControls.Jacobian().Premultiply(localJacobian);
        PROFILING_END_BLOCK;
    }

    return DiffData<T>(output, Jacobian);
}


template <class T>
int RigLogic<T>::NumLODs() const
{
    return m->numLODs;
}

template <class T>
const SparseMatrix<T>& RigLogic<T>::PsdToRawMap() const
{
    return m->psdToRawMap;
}

template <class T>
void RigLogic<T>::ReduceToLOD0Only()
{
    m->numLODs = 1;
    m->blendshapesPerLOD.conservativeResize(1);
    m->jointMatrixPerLOD.resize(1);
    m->animatedMapsPerLOD.conservativeResize(1);
}

template <class T>
std::vector<std::tuple<int, int, Eigen::VectorX<T>>> RigLogic<T>::GetAllExpressions() const
{
    std::vector<std::tuple<int, int, Eigen::VectorX<T>>> psds;
    Eigen::VectorX<T> psdValues = Eigen::VectorX<T>::Zero(m->totalControlCount);

    // push each raw control
    for (int k = 0; k < m->rawControlCount; ++k) {
        psdValues[k] = T(1);
        psds.push_back({1, k, psdValues});
        psdValues[k] = 0;
    }

    // push all combinations
    for (int k = 0; k < int(m->psdToRawMap.outerSize()); ++k) {
        Eigen::VectorX<T> rawControls = Eigen::Vector<T, -1>::Zero(m->rawControlCount);
        int controlCount = 0;
        for (typename SparseMatrix<T>::InnerIterator it(m->psdToRawMap, k); it; ++it)
        {
            rawControls[it.col()] = T(1) / it.value();
            controlCount++;
        }
        psds.push_back({controlCount, m->rawControlCount + k, EvaluatePSD(DiffData<T>(rawControls)).Value()});
    }

    // sort the expressions by the number of raw controls that are affecting the expression
    std::sort(psds.begin(), psds.end(), [](const auto& a, const auto& b) { return (std::get<0>(a) < std::get<0>(b)) || (std::get<0>(a) == std::get<0>(b) && (std::get<1>(a) < std::get<1>(b))); });

    return psds;
}

template <class T>
void RigLogic<T>::RemoveJoints(const std::vector<int>& newToOldJointMapping)
{
    m->numJoints = int(newToOldJointMapping.size());
    const int dofPerJoint = (m->withJointScaling ? 9 : 6);
    for (auto& smat : m->jointMatrixPerLOD) {
        std::vector<Eigen::Triplet<T>> triplets;
        for (int newIdx = 0; newIdx < int(newToOldJointMapping.size()); ++newIdx) {
            const int oldIdx = newToOldJointMapping[newIdx];
            for (int k = 0; k < dofPerJoint; ++k) {
                for (typename SparseMatrix<T>::InnerIterator it(smat, oldIdx * dofPerJoint + k); it; ++it) {
                    triplets.push_back(Eigen::Triplet<T>(newIdx * dofPerJoint + k, int(it.col()), it.value()));
                }
            }
        }
        smat.resize(m->numJoints * dofPerJoint, smat.cols());
        smat.setFromTriplets(triplets.begin(), triplets.end());
    }
}

// explicitly instantiate the rig logic classes
template class RigLogic<float>;
template class RigLogic<double>;

} // namespace epic::nls
