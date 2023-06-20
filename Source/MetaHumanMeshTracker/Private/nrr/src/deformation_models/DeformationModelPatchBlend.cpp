// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/deformation_models/DeformationModelPatchBlend.h>
#include <nrr/deformation_models/DeformationModelRigid.h>

#include <nls/VectorVariable.h>
#include <nls/functions/ConcatenateFunction.h>
#include <nls/serialization/EigenSerialization.h>
#include <nls/utils/FileIO.h>

#include <carbon/io/JsonIO.h>

#include <vector>

namespace epic::nls {

template<class T>
struct DeformationModelPatchBlend<T>::Private {
    DeformationModelRigid<T> defModelRigid;

    // ! parameters per patch
    std::vector<VectorVariable<T> > regionParameters;

    // ! global scale parameter
    VectorVariable<T> scaleVariable = VectorVariable<T>(1);

    // ! gravity center of mean
    Eigen::Vector3<T> centerOfGravity;

    // ! mean per patch
    std::vector<Eigen::Vector<T, -1> > regionMeans;

    // ! vertex evaluation per patch
    std::vector<SparseMatrixConstPtr<T> > regionBlendMatrices;

    // ! rigid transformation per patch
    std::vector<DeformationModelRigid<T> > regionTransforms;

    // ! global blend matrix combining the individual patches
    SparseMatrixConstPtr<T> globalBlendMatrix;

    // ! global smoothness matrix ensuring that neighboring patches evaluate to similar output vertices
    SparseMatrixConstPtr<T> globalSmoothnessMatrix;

    Configuration config = {std::string("Patch-blend Identity Deformation Model Configuration"), {
                                // ! whether to optimize the pose of the model
                                {"optimizePose", ConfigurationParameter(true)},
                                // ! whether to fix rotation while optimizing pose
                                {"fixRotation", ConfigurationParameter(false)},
                                // ! whether to fix translation while optimizing pose
                                {"fixTranslation", ConfigurationParameter(false)},
                                // ! whether to optimize the scale of the model
                                {"optimizeScale", ConfigurationParameter(false)},
                                // ! regularization on PCA coefficients of patches
                                {"modelRegularization", ConfigurationParameter(T(100), T(0), T(1000))},
                                // ! whether to optimize the transformation per patch
                                {"patchTransformation", ConfigurationParameter(false)},
                                // ! weight on smoothness between patches
                                {"patchSmoothness", ConfigurationParameter(T(1), T(0), T(10))}
                            }};
};

template<class T>
DeformationModelPatchBlend<T>::DeformationModelPatchBlend()
    : m(std::make_unique<Private>()) {
}

template <class T> DeformationModelPatchBlend<T>::~DeformationModelPatchBlend() = default;
template <class T> DeformationModelPatchBlend<T>::DeformationModelPatchBlend(DeformationModelPatchBlend&& other) = default;
template <class T> DeformationModelPatchBlend<T>& DeformationModelPatchBlend<T>::operator=(DeformationModelPatchBlend&& other) = default;

template<class T>
DiffDataMatrix<T, 3, -1> DeformationModelPatchBlend<T>::EvaluateVertices(Context<T>* context) {
    return EvaluateVertices(context,  /*withPose=*/ true);
}

template<class T>
Cost<T> DeformationModelPatchBlend<T>::EvaluateModelConstraints(Context<T>* context) {
    Cost<T> cost;

    const T modelRegularization = m->config["modelRegularization"].template Value<T>();
    if  (modelRegularization > 0) {
        for (size_t i = 0; i < m->regionParameters.size(); ++i) {
            cost.Add(m->regionParameters[i].Evaluate(context), modelRegularization);
        }
    }

    const T patchRegularization = m->config["patchSmoothness"].template Value<T>();
    const bool optimizePatchTransform = m->config["patchTransformation"].template Value<bool>();
    if (patchRegularization > 0) {
        // evaluate local regions
        std::vector<DiffData<T> > patchEvaluations;
        std::vector<T> unusedWeight;
        for (size_t i = 0; i < m->regionParameters.size(); ++i) {
            const DiffData<T> regionParams = m->regionParameters[i].Evaluate(context);
            const SparseMatrix<T>& regionBlendMatrix = *m->regionBlendMatrices[i];
            const Vector<T> regionVertices = regionBlendMatrix * regionParams.Value() + m->regionMeans[i];
            const JacobianConstPtr<T> jacobian = context ? regionParams.Jacobian().Premultiply(regionBlendMatrix) : nullptr;
            const DiffData<T> diffRegionVertices(regionVertices, jacobian);
            const int numRegionVertices = static_cast<int>(regionVertices.size()) / 3;
            const DiffDataMatrix<T, 3, -1> diffRegionVerticesMatrix(3, numRegionVertices, diffRegionVertices);
            Context<T>* transformContext = optimizePatchTransform ? context : nullptr;
            const DiffDataAffine<T, 3, 3> diffAffine = m->regionTransforms[i].EvaluateAffine(transformContext);
            patchEvaluations.push_back(diffAffine.Transform(diffRegionVerticesMatrix));
            unusedWeight.push_back(T(1));
        }

        // concatenate all patches
        const DiffData<T> concatenatedPatchEvaluations = ConcatenateDiffData<T>(patchEvaluations, unusedWeight);

        // evaluate smoothness of patches i.e. the same vertex for two different patches
        // should evaluate to a similar position
        JacobianConstPtr<T> jacobian;
        if (concatenatedPatchEvaluations.HasJacobian()) {
            jacobian = concatenatedPatchEvaluations.Jacobian().Premultiply(*m->globalSmoothnessMatrix);
        }
        const Vector<T> smoothnessTerm = (*m->globalSmoothnessMatrix) * concatenatedPatchEvaluations.Value();
        cost.Add(DiffData<T>(std::make_shared<Vector<T> >(smoothnessTerm), jacobian), patchRegularization);
    }

    return cost;
}

template<class T>
const Configuration& DeformationModelPatchBlend<T>::GetConfiguration() const {
    return m->config;
}

template<class T>
void DeformationModelPatchBlend<T>::SetConfiguration(const Configuration& config) {
    m->config.Set(config);
}

template<class T>
void DeformationModelPatchBlend<T>::LoadModel(const std::string& identityBlendModelFile) {
    //// see format in IdentityBlendModel.h
    const carbon::JsonElement json = carbon::ReadJson(ReadFile(identityBlendModelFile));

    m->regionParameters.clear();
    m->regionMeans.clear();
    m->regionBlendMatrices.clear();
    m->regionTransforms.clear();

    Eigen::Matrix<T, 3, -1> mean;
    FromJson(json["mean"], mean);
    const int numVertices = int(mean.cols());

    m->centerOfGravity = mean.rowwise().mean();

    std::vector<Eigen::Triplet<T> > globalBlendTriplets;
    int globalRowIndex = 0;

    const carbon::JsonElement& jRegions = json["regions"];

    for (auto&& [regionName, regionData] : jRegions.Map()) {
        Eigen::VectorXi vertexIDs;
        Eigen::VectorX<T> weights;
        Eigen::Matrix<T, -1, -1> modes;
        FromJson(regionData["vertex_ids"], vertexIDs);
        FromJson(regionData["weights"], weights);
        FromJson(regionData["modes"], modes);
        const int numRegionModes = int(modes.cols());
        const int numRegionVertices = int(modes.rows() / 3);
        if (int(vertexIDs.size()) != numRegionVertices) {
            CARBON_CRITICAL("vertex_ids and modes matrix for region {} do not match", regionName);
        }
        if (int(weights.size()) != numRegionVertices) {
            CARBON_CRITICAL("weights and modes matrix for region {} do not match", regionName);
        }

        // region blend matrix is just the modes matrix
        m->regionParameters.push_back(std::move(VectorVariable<T>(numRegionModes)));
        m->regionBlendMatrices.push_back(std::make_shared<SparseMatrix<T> >(modes.sparseView()));

        Eigen::VectorX<T> regionMean(3 * numRegionVertices);
        for (int j = 0; j < numRegionVertices; ++j) {
            for (int k = 0; k < 3; ++k) {
                regionMean[3 * j + k] = mean(k, vertexIDs[j]);
            }
        }
        m->regionMeans.push_back(regionMean);
        m->regionTransforms.push_back(std::move(DeformationModelRigid<T>()));
        m->regionTransforms.back().SetVertices(Eigen::Map<const Eigen::Matrix<T, 3, -1>>(regionMean.data(), 3, numRegionVertices));

        for (int j = 0; j < numRegionVertices; ++j) {
            const int vID = vertexIDs[j];
            for (int k = 0; k < 3; ++k) {
                globalBlendTriplets.push_back(Eigen::Triplet<T>(3 * vID + k, globalRowIndex++, weights[j]));
            }
        }
    }

    SparseMatrixPtr<T> globalBlendMatrix = std::make_shared<SparseMatrix<T> >(3 * numVertices, globalRowIndex);
    globalBlendMatrix->setFromTriplets(globalBlendTriplets.begin(), globalBlendTriplets.end());
    m->globalBlendMatrix = globalBlendMatrix;

    std::vector<Eigen::Triplet<T> > globalSmoothnessTriplets;
    int smoothnessRowIndex = 0;
    for (int r = 0; r < int(globalBlendMatrix->rows()); ++r) {
        for (typename SparseMatrix<T>::InnerIterator it1(*globalBlendMatrix, r); it1; ++it1) {
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

    ResetParameters();
    // it is important to set the base vertices for the rigid model so that the center of gravity is taken into account in the
    // rigid transformation
    m->defModelRigid.SetVertices(DeformedVertices());
}

template<class T>
int DeformationModelPatchBlend<T>::NumParameters() const {
    // TODO: should we add the affine transformation as well?
    int numParameters = 0;
    for (const auto& regionVar : m->regionParameters) {
        numParameters += regionVar.Size();
    }
    return numParameters;
}

template<class T>
int DeformationModelPatchBlend<T>::NumPatches() const {
    return int(m->regionParameters.size());
}


template<class T>
void DeformationModelPatchBlend<T>::ResetParameters() {
    for (auto& regionVar : m->regionParameters) {
        regionVar.SetZero();
    }
    for (auto& regionTransformation : m->regionTransforms) {
        regionTransformation.SetRigidTransformation(Affine<T, 3, 3>());
    }

    m->scaleVariable.Set(Eigen::Vector<T, 1>(T(1)));
}

template<class T>
Eigen::Matrix<T, 3, -1> DeformationModelPatchBlend<T>::DeformedVertices() {
    return EvaluateVertices(nullptr,  /*withPose=*/ false).Matrix();
}

template<class T>
DiffDataMatrix<T, 3, -1> DeformationModelPatchBlend<T>::EvaluateVertices(Context<T>* context, bool withPose) {
    if  (m->regionParameters.size() == 0) {
        throw std::runtime_error("no patch-blend identity model has been loaded");
    }

    Configuration rigidConfig = m->defModelRigid.GetConfiguration();
    rigidConfig["fixRotation"] = m->config["fixRotation"];
    rigidConfig["fixTranslation"] = m->config["fixTranslation"];
    m->defModelRigid.SetConfiguration(rigidConfig);
    rigidConfig["fixRotation"].Set(true);
    rigidConfig["fixTranslation"].Set(true);
    m->regionTransforms[0].SetConfiguration(rigidConfig);

    const bool optimizePatchTransform = m->config["patchTransformation"].template Value<bool>();
    const bool optimizePose = m->config["optimizePose"].template Value<bool>() && !optimizePatchTransform;
    const bool optimizeScale = m->config["optimizeScale"].template Value<bool>();

    // evaluate local regions
    std::vector<DiffData<T> > patchEvaluations;
    std::vector<T> unusedWeight;
    for (size_t i = 0; i < m->regionParameters.size(); ++i) {
        const DiffData<T> regionParams = m->regionParameters[i].Evaluate(context);
        const SparseMatrix<T>& regionBlendMatrix = *m->regionBlendMatrices[i];
        const Vector<T> regionVertices = regionBlendMatrix * regionParams.Value() + m->regionMeans[i];
        const JacobianConstPtr<T> jacobian = context ? regionParams.Jacobian().Premultiply(regionBlendMatrix) : nullptr;
        const DiffData<T> diffRegionVertices(regionVertices, jacobian);
        const int numRegionVertices = static_cast<int>(regionVertices.size()) / 3;
        const DiffDataMatrix<T, 3, -1> diffRegionVerticesMatrix(3, numRegionVertices, diffRegionVertices);
        Context<T>* transformContext = optimizePatchTransform ? context : nullptr;
        const DiffDataAffine<T, 3, 3> diffAffine = m->regionTransforms[i].EvaluateAffine(transformContext);
        patchEvaluations.push_back(diffAffine.Transform(diffRegionVerticesMatrix));
        unusedWeight.push_back(T(1));
    }

    // concatenate all patches
    const DiffData<T> concatenatedPatchEvaluations = ConcatenateDiffData<T>(patchEvaluations, unusedWeight);

    // blend patches into all vertices
    JacobianConstPtr<T> jacobian;
    if (concatenatedPatchEvaluations.HasJacobian()) {
        jacobian = concatenatedPatchEvaluations.Jacobian().Premultiply(*m->globalBlendMatrix);
    }
    const int numVertices = int(m->globalBlendMatrix->rows() / 3);
    const Vector<T> globalVertices = (*m->globalBlendMatrix) * concatenatedPatchEvaluations.Value();
    DiffDataMatrix<T, 3, -1> diffGlobalVertices(3, numVertices, DiffData<T>(globalVertices, jacobian));

    // scale the vertices
    DiffData<T> diffScale = m->scaleVariable.Evaluate(optimizeScale ? context : nullptr);
    if (optimizeScale || diffScale.Value()[0] != T(1)) {
        diffGlobalVertices = ColwiseAddFunction<T>().colwiseAddFunction(diffGlobalVertices, DiffDataMatrix<T, 3, 1>(-m->centerOfGravity));
        DiffDataMatrix<T, 1, 1> scaleMatrix(1, 1, m->scaleVariable.Evaluate(optimizeScale ? context : nullptr));
        DiffDataMatrix<T, 1, -1> flattenedVertices(1, numVertices * 3, diffGlobalVertices);
        DiffDataMatrix<T, 1, -1> scaledFlattenedVertices = MatrixMultiplyFunction<T>::DenseMatrixMatrixMultiply(scaleMatrix, flattenedVertices);
        diffGlobalVertices = DiffDataMatrix<T, 3, -1>(3, numVertices, scaledFlattenedVertices);
        diffGlobalVertices = ColwiseAddFunction<T>().colwiseAddFunction(diffGlobalVertices, DiffDataMatrix<T, 3, 1>(m->centerOfGravity));
    }

    if (withPose) {
        return m->defModelRigid.EvaluateAffine(optimizePose ? context : nullptr).Transform(diffGlobalVertices);
    } else {
        return diffGlobalVertices;
    }
}

template<class T>
void DeformationModelPatchBlend<T>::SetRigidTransformation(const Affine<T, 3, 3>& affine) {
    m->defModelRigid.SetRigidTransformation(affine);
}

template<class T>
Affine<T, 3, 3> DeformationModelPatchBlend<T>::RigidTransformation() const {
    return m->defModelRigid.RigidTransformation();
}

// explicitly instantiate the DeformationModelPatchBlend classes
template class DeformationModelPatchBlend<float>;
template class DeformationModelPatchBlend<double>;

}  // namespace epic::nls
