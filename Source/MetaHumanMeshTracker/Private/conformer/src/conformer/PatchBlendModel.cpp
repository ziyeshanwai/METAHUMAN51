// Copyright Epic Games, Inc. All Rights Reserved.

#include <conformer/PatchBlendModel.h>

#include <carbon/utils/TaskThreadPool.h>
#include <nls/VectorVariable.h>
#include <nls/functions/ColwiseAddFunction.h>
#include <nls/functions/ConcatenateFunction.h>
#include <nls/geometry/AffineVariable.h>
#include <nls/geometry/QuaternionVariable.h>
#include <nls/serialization/EigenSerialization.h>
#include <nls/utils/FileIO.h>
#include <tracking/rt/LinearVertexModel.h>
#include <nls/geometry/QRigidMotion.h>

#include <carbon/io/JsonIO.h>

#include <algorithm>
#include <vector>

namespace epic::nls {

template <class T, int R, int C>
Eigen::Ref<Eigen::Vector<T, -1>> Flatten(Eigen::Matrix<T, R, C>& matrix)
{
    return Eigen::Map<Eigen::Vector<T, -1>>(matrix.data(), matrix.rows() * matrix.cols());
}

template <class T, int R, int C>
Eigen::Ref<const Eigen::Vector<T, -1>> Flatten(const Eigen::Matrix<T, R, C>& matrix)
{
    return Eigen::Map<const Eigen::Vector<T, -1>>(matrix.data(), matrix.rows() * matrix.cols());
}


template<class T>
struct PatchBlendModel<T>::Private {

    //! variable containing all variables: coefficients for region PCA models, rotation, translation, and scale
    VectorVariable<T> allVariables = VectorVariable<T>(0);

    //! the offset for each region into allVariables
    std::vector<int> regionVariableOffsets;

    //! number of vertices
    int numVertices;

    //! which region should stay "fixed"
    size_t fixRegion = 0;

    //! the rotation and translation per region
    std::vector<Eigen::Quaternion<T>> regionRotations;

    //! the linear model per region: dR * (1 + dscale) * (base + modes * params) + dt
    std::vector<rt::LinearVertexModel<T>> regionModels;

    /**
     * The linear model after rotation and scale have been applied.
     * rotated = (dR * R) * (scale + dscale) * (base + modes * params) + (T + dt)
     */
    std::vector<rt::LinearVertexModel<T>> transformedRegionModels;

    // ! gravity center of each region
    std::vector<Eigen::Vector3<T>> centerOfGravityPerRegion;

    // ! global blend matrix combining the individual patches
    SparseMatrix<T> globalBlendMatrix;

    //! for each "concatenated" region vertex as used in globalBlendMatrix, this points to the region and vertex index in the region.
    std::vector<std::vector<std::tuple<int, int, T>>> globalBlendMatrix2;

    // ! global smoothness matrix ensuring that neighboring patches evaluate to similar output vertices
    SparseMatrixConstPtr<T> globalSmoothnessMatrix;

    //! cache size of sparse matrices
    int numNonZerosVertexJacobian = 0;
    int numNonZerosSmoothnessJacobian = 0;

    //! thread pool for parallelization
    std::shared_ptr<epic::carbon::TaskThreadPool> globalThreadPool = epic::carbon::TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/ true);

    Configuration config = {std::string("Patch-blend Identity Deformation Model Configuration"), {
                            // ! whether to optimize the scale of the model
                            {"optimizeScale", ConfigurationParameter(false)},
                            // ! regularization on PCA coefficients of patches
                            {"modelRegularization", ConfigurationParameter(T(100), T(0), T(1000))},
                            // ! weight on smoothness between patches
                            {"patchSmoothness", ConfigurationParameter(T(1), T(0), T(10))}
                            }};
};

template<class T>
PatchBlendModel<T>::PatchBlendModel()
    : m(std::make_unique<Private>()) {
}

template <class T> PatchBlendModel<T>::~PatchBlendModel() = default;
template <class T> PatchBlendModel<T>::PatchBlendModel(PatchBlendModel&& other) = default;
template <class T> PatchBlendModel<T>& PatchBlendModel<T>::operator=(PatchBlendModel&& other) = default;

template<class T>
std::pair<DiffDataMatrix<T, 3, -1>, Cost<T>> PatchBlendModel<T>::EvaluateVerticesAndConstraints(Context<T>* context)
{
    // evaluate so that the variables are being used
    m->allVariables.Evaluate(context);

    // update the vertices per region (and the modes if context is used)
    UpdateRegionModels(/*withModes=*/bool(context));

    epic::carbon::TaskFutures taskFutures;

    DiffDataMatrix<T, 3, -1> diffGlobalVertices(3, 0, DiffData<T>(nullptr, 0));
    taskFutures.Add(m->globalThreadPool->AddTask([&](){
        diffGlobalVertices = EvaluateVertices(context);
    }));

    // evaluate model regularization
    const T modelRegularization = m->config["modelRegularization"].template Value<T>();
    DiffData<T> regDiffData(nullptr, 0);
    if  (modelRegularization > 0) {
        taskFutures.Add(m->globalThreadPool->AddTask([&](){
            regDiffData = EvaluateRegularization(context);
        }));
    }

    // evaluate patch regularization
    const T patchRegularization = m->config["patchSmoothness"].template Value<T>();
    DiffData<T> patchDiffData(nullptr, 0);
    if (patchRegularization > 0) {
        taskFutures.Add(m->globalThreadPool->AddTask([&](){
            patchDiffData = EvaluatePatchSmoothness(context);
        }));
    }

    taskFutures.Wait();

    Cost<T> cost;
    if (patchRegularization > 0) {
        cost.Add(patchDiffData, patchRegularization);
    }
    if (modelRegularization > 0) {
        cost.Add(regDiffData, modelRegularization);
    }

    return {diffGlobalVertices, cost};
}

template<class T>
DiffDataMatrix<T, 3, -1> PatchBlendModel<T>::EvaluateVertices(Context<T>* context)
{
    Eigen::Matrix<T, 3, -1> output = Eigen::Matrix<T, 3, -1>(3, m->numVertices);
    for (int vID = 0; vID < m->numVertices; ++vID) {
        Eigen::Vector3<T> v = Eigen::Vector3<T>::Zero();
        for (const auto& [regionIndex, regionVID, weight] : m->globalBlendMatrix2[vID]) {
            v += m->transformedRegionModels[regionIndex].Base().col(regionVID) * weight;
        }
        output.col(vID) = v;
    }

    // set up the jacobian for the vertex evaluation
    SparseJacobianConstPtr<T> jacobian;
    if (context) {
        SparseMatrixPtr<T> smat = std::make_shared<SparseMatrix<T>>(3 * m->numVertices, NumParameters());
        smat->reserve(m->numNonZerosVertexJacobian);

        for (int vID = 0; vID < m->numVertices; ++vID) {
            for (int k = 0; k < 3; ++k) {
                smat->startVec(3 * vID + k);
                for (const auto& [regionIndex, regionVID, weight] : m->globalBlendMatrix2[vID]) {
                    const int offset = m->regionVariableOffsets[regionIndex];
                    for (Eigen::Index c = 0; c < m->transformedRegionModels[regionIndex].MutableModes().cols(); ++c) {
                        smat->insertBackByOuterInnerUnordered(3 * vID + k, offset + c) = weight * m->transformedRegionModels[regionIndex].MutableModes()(3 * regionVID + k, c);
                    }
                }
                smat->finalize();
            }
        }
        m->numNonZerosVertexJacobian = int(smat->nonZeros());

        jacobian = std::make_shared<SparseJacobian<T>>(smat);
    }

    return DiffDataMatrix<T, 3, -1>(output, jacobian);
}

template<class T>
DiffData<T> PatchBlendModel<T>::EvaluateRegularization(Context<T>* context)
{
    int numValues = 0;
    for (int k = 0; k < NumPatches(); ++k) {
        numValues += m->regionModels[k].NumPCAModes();
    }
    Eigen::VectorX<T> regValues(numValues);

    SparseJacobianConstPtr<T> regJacobian;
    std::vector<Eigen::Triplet<T>> triplets;

    int offset = 0;
    for (int k = 0; k < NumPatches(); ++k) {
        const int numModes = m->regionModels[k].NumPCAModes();
        const Eigen::VectorX<T> coeffs = m->allVariables.Value().segment(m->regionVariableOffsets[k], numModes);
        regValues.segment(offset, numModes) = coeffs;
        if (context) {
            for (int j = 0; j < numModes; ++j) {
                triplets.push_back(Eigen::Triplet<T>(offset + j, m->regionVariableOffsets[k] + j, T(1)));
            }
        }
        offset += numModes;
    }

    if (context) {
        SparseMatrixPtr<T> smat = std::make_shared<SparseMatrix<T>>(numValues, NumParameters());
        smat->setFromTriplets(triplets.begin(), triplets.end());
        regJacobian = std::make_shared<SparseJacobian<T>>(smat);
    }

    return DiffData<T>(regValues, regJacobian);
}

template<class T>
DiffData<T> PatchBlendModel<T>::EvaluatePatchSmoothness(Context<T>* context)
{
    std::vector<Eigen::Vector3<T>> regCosts;
    for (int vID = 0; vID < m->numVertices; ++vID) {
        for (size_t j1 = 0; j1 < m->globalBlendMatrix2[vID].size(); ++j1) {
            for (size_t j2 = j1 + 1; j2 < m->globalBlendMatrix2[vID].size(); ++j2) {
                const int regionIndex1 = std::get<0>(m->globalBlendMatrix2[vID][j1]);
                const int regionvID1 = std::get<1>(m->globalBlendMatrix2[vID][j1]);
                const int regionIndex2 = std::get<0>(m->globalBlendMatrix2[vID][j2]);
                const int regionvID2 = std::get<1>(m->globalBlendMatrix2[vID][j2]);
                const Eigen::Vector3<T> vertexDiff = m->transformedRegionModels[regionIndex1].Base().col(regionvID1) - m->transformedRegionModels[regionIndex2].Base().col(regionvID2);
                regCosts.push_back(vertexDiff);
            }
        }
    }

    Eigen::VectorX<T> regCostsVec = Eigen::Map<const Eigen::VectorX<T>>((const T*)regCosts.data(), regCosts.size() * 3);

    // evaluate smoothness of patches i.e. the same vertex for two different patches
    // should evaluate to a similar position
    JacobianConstPtr<T> patchJacobian;
    if (context) {
        SparseMatrixPtr<T> smat = std::make_shared<SparseMatrix<T>>(regCostsVec.size(), NumParameters());
        smat->reserve(m->numNonZerosSmoothnessJacobian);

        int rowIndex = 0;
        for (int vID = 0; vID < m->numVertices; ++vID) {
            for (size_t j1 = 0; j1 < m->globalBlendMatrix2[vID].size(); ++j1) {
                for (size_t j2 = j1 + 1; j2 < m->globalBlendMatrix2[vID].size(); ++j2) {
                    const int regionIndex1 = std::get<0>(m->globalBlendMatrix2[vID][j1]);
                    const int regionVID1 = std::get<1>(m->globalBlendMatrix2[vID][j1]);
                    const int regionIndex2 = std::get<0>(m->globalBlendMatrix2[vID][j2]);
                    const int regionVID2 = std::get<1>(m->globalBlendMatrix2[vID][j2]);
                    const int offset1 = m->regionVariableOffsets[regionIndex1];
                    const int offset2 = m->regionVariableOffsets[regionIndex2];
                    for (int k = 0; k < 3; ++k) {
                        smat->startVec(rowIndex + k);
                        for (Eigen::Index c = 0; c < m->transformedRegionModels[regionIndex1].MutableModes().cols(); ++c) {
                            smat->insertBackByOuterInnerUnordered(rowIndex + k, offset1 + c) = m->transformedRegionModels[regionIndex1].MutableModes()(3 * regionVID1 + k, c);
                        }
                        for (Eigen::Index c = 0; c < m->transformedRegionModels[regionIndex2].MutableModes().cols(); ++c) {
                            smat->insertBackByOuterInnerUnordered(rowIndex + k, offset2 + c) = - m->transformedRegionModels[regionIndex2].MutableModes()(3 * regionVID2 + k, c);
                        }
                        smat->finalize();
                    }
                    rowIndex += 3;
                }
            }
        }

        m->numNonZerosSmoothnessJacobian = int(smat->nonZeros());
        patchJacobian = std::make_shared<SparseJacobian<T>>(smat);
    }

    return DiffData<T>(std::make_shared<Vector<T>>(regCostsVec), patchJacobian);
}

template<class T>
const Configuration& PatchBlendModel<T>::GetConfiguration() const {
    return m->config;
}

template<class T>
void PatchBlendModel<T>::SetConfiguration(const Configuration& config) {
    m->config.Set(config);
}

template<class T>
void PatchBlendModel<T>::LoadModel(const std::string& identityBlendModelFile) {
    // see format in IdentityBlendModel.h
    const carbon::JsonElement json = carbon::ReadJson(ReadFile(identityBlendModelFile));

    // m->regionParameters.clear();
    // m->regionMeans.clear();
    m->centerOfGravityPerRegion.clear();
    // m->regionBlendMatrices.clear();
    // m->regionRotations.clear();

    m->regionRotations.clear();
    m->regionModels.clear();
    m->regionVariableOffsets.clear();

    Eigen::Matrix<T, 3, -1> mean;
    FromJson(json["mean"], mean);
    const int numVertices = int(mean.cols());
    m->numVertices = numVertices;
    m->globalBlendMatrix2 = std::vector<std::vector<std::tuple<int,int, T>>>(numVertices);

    std::vector<T> maxWeight(numVertices, 0);

    std::vector<Eigen::Triplet<T> > globalBlendTriplets;
    int globalRowIndex = 0;
    int accumulatedParameters = 0;

    const carbon::JsonElement& jRegions = json["regions"];

    int regionIndex = 0;
    for (auto&& [regionName, regionData] : jRegions.Map()) {
        Eigen::VectorXi vertexIDs;
        Eigen::VectorX<T> weights;
        Eigen::Matrix<T, -1, -1> modes;
        FromJson(regionData["vertex_ids"], vertexIDs);
        FromJson(regionData["weights"], weights);
        FromJson(regionData["modes"], modes);
        // const int numRegionModes = int(modes.cols());
        const int numRegionVertices = int(modes.rows() / 3);
        if (int(vertexIDs.size()) != numRegionVertices) {
            CARBON_CRITICAL("vertex_ids and modes matrix for region {} do not match", regionName);
        }
        if (int(weights.size()) != numRegionVertices) {
            CARBON_CRITICAL("weights and modes matrix for region {} do not match", regionName);
        }

        // region blend matrix is just the modes matrix
        // m->regionParameters.push_back(std::move(VectorVariable<T>(numRegionModes)));
        // m->regionBlendMatrices.push_back(std::make_shared<SparseMatrix<T> >(modes.sparseView()));

        Eigen::VectorX<T> regionMean(3 * numRegionVertices);
        for (int j = 0; j < numRegionVertices; ++j) {
            for (int k = 0; k < 3; ++k) {
                regionMean[3 * j + k] = mean(k, vertexIDs[j]);
            }
        }
        auto regionMeanVertices = Eigen::Map<Eigen::Matrix<T, 3, -1>>(regionMean.data(), 3, numRegionVertices);

        Eigen::Vector3<T> centerOfGravityOfRegion = regionMeanVertices.rowwise().mean();
        regionMeanVertices.colwise() -= centerOfGravityOfRegion;
        m->centerOfGravityPerRegion.push_back(centerOfGravityOfRegion);

        // m->regionMeans.push_back(regionMean);
        // m->regionRotations.push_back(std::move(AffineVariable<QuaternionVariable<T>>()));

        m->regionRotations.push_back(Eigen::Quaternion<T>::Identity());
        m->regionModels.push_back(rt::LinearVertexModel<T>(regionMeanVertices, modes));

        for (int j = 0; j < numRegionVertices; ++j) {
            const int vID = vertexIDs[j];
            for (int k = 0; k < 3; ++k) {
                globalBlendTriplets.push_back(Eigen::Triplet<T>(3 * vID + k, globalRowIndex++, weights[j]));
                maxWeight[vID] = std::max<T>(maxWeight[vID], weights[j]);
            }
            m->globalBlendMatrix2[vID].push_back({regionIndex, j, weights[j]});
        }

        if (regionName == "nose") {
            // make the nose the fix region i.e. no rigid transformation
            m->fixRegion = regionIndex;
        }

        regionIndex++;
        m->regionVariableOffsets.push_back(accumulatedParameters);
        accumulatedParameters += int(modes.cols()) + 7 /*rigid and scale*/;
    }

    SparseMatrix<T> globalBlendMatrix(3 * numVertices, globalRowIndex);
    globalBlendMatrix.setFromTriplets(globalBlendTriplets.begin(), globalBlendTriplets.end());
    m->globalBlendMatrix = std::move(globalBlendMatrix);

    std::vector<Eigen::Triplet<T> > globalSmoothnessTriplets;
    int smoothnessRowIndex = 0;
    for (int r = 0; r < int(globalBlendMatrix.rows()); ++r) {
        for (typename SparseMatrix<T>::InnerIterator it1(globalBlendMatrix, r); it1; ++it1) {
            typename SparseMatrix<T>::InnerIterator it2 = it1;
            ++it2;
            for (; it2; ++it2) {
                const int c1 = static_cast<int>(it1.col());
                const int c2 = static_cast<int>(it2.col());
                globalSmoothnessTriplets.push_back(Eigen::Triplet<T>(smoothnessRowIndex, c1, T(1)));
                globalSmoothnessTriplets.push_back(Eigen::Triplet<T>(smoothnessRowIndex, c2, T(-1)));
                smoothnessRowIndex++;
            }
        }
    }
    SparseMatrixPtr<T> globalSmoothMatrix = std::make_shared<SparseMatrix<T> >(smoothnessRowIndex, globalRowIndex);
    globalSmoothMatrix->setFromTriplets(globalSmoothnessTriplets.begin(), globalSmoothnessTriplets.end());
    m->globalSmoothnessMatrix = globalSmoothMatrix;

    m->transformedRegionModels.clear();
    for (const auto& regionModel : m->regionModels) m->transformedRegionModels.push_back(regionModel);

    m->allVariables = VectorVariable<T>(accumulatedParameters);
    m->allVariables.SetZero();

    ResetParameters();
}

template<class T>
int PatchBlendModel<T>::NumParameters() const {
    return m->allVariables.Size();
}

template<class T>
int PatchBlendModel<T>::NumPatches() const {
    return int(m->regionRotations.size());
}

template<class T>
void PatchBlendModel<T>::ResetParameters() {
    if (m->allVariables.Size() == 0) return;

    Eigen::VectorX<T> values = m->allVariables.Value();
    values.setZero();

    // set scale to 1 and translation to origin center of gravity per region
    for (size_t k = 0; k < m->regionModels.size(); ++k) {
        const int rigidOffset = m->regionVariableOffsets[k] + m->regionModels[k].NumPCAModes();
        values[rigidOffset + 6] = T(1);
        values.segment(rigidOffset + 3, 3) = m->centerOfGravityPerRegion[k];
    }

    m->allVariables.Set(values);

    for (size_t i = 0; i < m->regionRotations.size(); ++i) {
        m->regionRotations[i] = Eigen::Quaternion<T>::Identity();
    }
}

template<class T>
Eigen::Matrix<T, 3, -1> PatchBlendModel<T>::DeformedVertices() {

    UpdateRegionModels(/*withModes=*/false);

    Eigen::Matrix<T, 3, -1> output = Eigen::Matrix<T, 3, -1>(3, m->numVertices);
    for (int vID = 0; vID < m->numVertices; ++vID) {
        Eigen::Vector3<T> v = Eigen::Vector3<T>::Zero();
        for (const auto& [regionIndex, regionVID, weight] : m->globalBlendMatrix2[vID]) {
            v += m->transformedRegionModels[regionIndex].Base().col(regionVID) * weight;
        }
        output.col(vID) = v;
    }

    return output;
}

template<class T>
void PatchBlendModel<T>::TransformPatches(const Affine<T, 3, 3>& aff)
{
    // transform the patches
    Eigen::VectorX<T> values = m->allVariables.Value();

    QRigidMotion<T> deltaTransform(aff.Matrix());
    for (size_t k = 0; k < m->regionRotations.size(); ++k) {
        const int offset = m->regionVariableOffsets[k];
        const int numModes = m->regionModels[k].NumPCAModes();
        const int rigidOffset = offset + numModes;

        const Eigen::Vector3<T> dR = values.segment(rigidOffset + 0, 3);
        const Eigen::Vector3<T> dt = values.segment(rigidOffset + 3, 3);

        QRigidMotion<T> currTransform;
        currTransform.q = (Eigen::Quaternion<T>(T(1), dR[0], dR[1], dR[2]) * m->regionRotations[k]).normalized();
        currTransform.t = dt;
        QRigidMotion<T> newTransform = deltaTransform * currTransform;

        m->regionRotations[k] = newTransform.q;

        values.segment(rigidOffset + 0, 3).setZero();
        values.segment(rigidOffset + 3, 3) = newTransform.t;
    }

    m->allVariables.Set(values);
}

template<class T>
void PatchBlendModel<T>::UpdateRegionModels(bool withModes)
{
    const bool optimizeScale = m->config["optimizeScale"].template Value<bool>();

    auto updateRegionModel = [&](int start, int end) {
        for (int k = start; k < end; ++k) {
            // base + modes * params
            const int offset = m->regionVariableOffsets[k];
            const int numModes = m->regionModels[k].NumPCAModes();
            const int numRegionVertices = m->regionModels[k].NumVertices();

            m->regionModels[k].EvaluateLinearized(m->allVariables.Value().segment(offset, numModes), rt::LinearVertexModel<T>::EvaluationMode::STATIC, m->transformedRegionModels[k]);

            // rotated = (scale + dscale) * (dR * R) * (base + modes * params) + (T + dt)

            const int rigidOffset = offset + numModes;

            const Eigen::Vector3<T> dR = m->allVariables.Value().segment(rigidOffset + 0, 3);
            const Eigen::Vector3<T> dt = m->allVariables.Value().segment(rigidOffset + 3, 3);
            const T scale = m->allVariables.Value()[rigidOffset + 6];

            const Eigen::Matrix<T, 3, 3> R = (Eigen::Quaternion<T>(T(1), dR[0], dR[1], dR[2]) * m->regionRotations[k]).normalized().toRotationMatrix();
            const Eigen::Vector3<T> t = dt;

            // R * (base + modes * params)
            m->transformedRegionModels[k].MutableBase() = (R * m->transformedRegionModels[k].Base()).eval();

            if (withModes) {
                if (optimizeScale) {
                    // scale mode is the rotated vertices
                    m->transformedRegionModels[k].SetScaleMode(m->transformedRegionModels[k].Base());
                } else {
                    // no scale, so set mode to zero
                    m->transformedRegionModels[k].MutableModes().col(numModes + 6).setZero();
                }
            }

            // scale * R * (base + modes * params)
            m->transformedRegionModels[k].MutableBase() *= scale;

            if (withModes) {
                if (k == int(m->fixRegion)) {
                    m->transformedRegionModels[k].MutableModes().block(0, numModes, numRegionVertices * 3, 3).setZero();
                } else {
                    // rotation mode depends on rotated and scaled vertices (no translation)
                    m->transformedRegionModels[k].SetRotationModes(m->transformedRegionModels[k].Base());
                }
            }

            // add translation: R * scale * (base + modes * params) + T
            m->transformedRegionModels[k].MutableBase().colwise() += t;

            if (withModes) {
                // translation mode is always identity (i.e. no change) besides the first region which is fixed.
                if (k == int(m->fixRegion)) {
                    m->transformedRegionModels[k].MutableModes().block(0, numModes + 3, numRegionVertices * 3, 3).setZero();
                } else {
                    m->transformedRegionModels[k].SetTranslationModes();
                }
            }

            if (withModes) {
                // scale and rotate the modes
                for (int j = 0; j < m->regionModels[k].NumPCAModes(); ++j) {
                    Eigen::VectorX<T> mode = m->regionModels[k].Modes(rt::LinearVertexModel<T>::EvaluationMode::STATIC).col(j);
                    Eigen::Matrix<T, 3, -1> scaledRotatedMode = (scale * R) * Eigen::Map<const Eigen::Matrix<T, 3, -1>>(mode.data(), 3, mode.size() / 3);
                    m->transformedRegionModels[k].MutableModes().col(j).noalias() = Flatten(scaledRotatedMode);
                }
            }
        }
    };
    m->globalThreadPool->AddTaskRangeAndWait(int(m->regionModels.size()), updateRegionModel);
}

template<class T>
void PatchBlendModel<T>::BakeRotationLinearization()
{
    Eigen::VectorX<T> values = m->allVariables.Value();

    // copy rigid transforms
    for (size_t regionIndex = 0; regionIndex < m->regionVariableOffsets.size(); ++regionIndex) {
        const int rigidOffset = m->regionVariableOffsets[regionIndex] + m->regionModels[regionIndex].NumPCAModes();
        const Eigen::Vector3<T> dR = m->allVariables.Value().segment(rigidOffset + 0, 3);
        m->regionRotations[regionIndex] = (Eigen::Quaternion<T>(T(1), dR[0], dR[1], dR[2]) * m->regionRotations[regionIndex]).normalized();
        // set linearized rotation to zero
        values.segment(rigidOffset, 3).setZero();
    }

    m->allVariables.Set(values);
}


// explicitly instantiate the PatchBlendModel classes
template class PatchBlendModel<float>;
template class PatchBlendModel<double>;

}  // namespace epic::nls
