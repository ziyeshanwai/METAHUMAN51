// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/deformation_models/DeformationModelRegionBlend.h>

#include <carbon/io/JsonIO.h>
#include <nls/utils/FileIO.h>
#include <nrr/deformation_models/DeformationModelRigid.h>
#include <nrr/RegionBlendModel.h>
#include <nrr/DMTNormalizationConstraint.h>
#include <nrr/DMTSymmetryConstraint.h>
#include <nrr/serialization/RegionBlendModelSerialization.h>

#include <vector>

namespace epic::nls {

template<class T>
struct DeformationModelRegionBlend<T>::Private {
    DeformationModelRigid<T> defModelRigid;

    RegionBlendModel<T> regionBlendModel;
    BoundedVectorVariable<T> regionBlendParameters = BoundedVectorVariable<T>(0);

    // ! global scale parameter
    VectorVariable<T> scaleVariable = VectorVariable<T>(1);

    // ! gravity center of the default shape
    Eigen::Vector3<T> centerOfGravity;

    Configuration config = {std::string("Region Blend Deformation Model Configuration"), {
                                // !< whether to optimize the pose when doing fine registration
                                {"optimizePose", ConfigurationParameter(true)},
                                // ! whether to fix rotation while optimizing pose
                                {"fixRotation", ConfigurationParameter(false)},
                                // ! whether to fix translation while optimizing pose
                                {"fixTranslation", ConfigurationParameter(false)},
                                // ! whether to optimize the scale of the model
                                {"optimizeScale", ConfigurationParameter(false)},
                                // !< projective strain weight (stable, but incorrect Jacobian)
                                {"modelRegularization", ConfigurationParameter(T(100), T(0), T(1000))},
                                // !< regions normalization
                                {"normalization", ConfigurationParameter(T(10000), T(0), T(10000))},
                                // !< symmetry constraint
                                {"symmetryRegularization", ConfigurationParameter(T(0), T(0), T(2000))},
                            }};
};

template<class T>
DeformationModelRegionBlend<T>::DeformationModelRegionBlend()
    : m(std::make_unique<Private>()) {
        m->scaleVariable.Set(Eigen::Vector<T, 1>(T(1)));
}

template <class T> DeformationModelRegionBlend<T>::~DeformationModelRegionBlend() = default;
template <class T> DeformationModelRegionBlend<T>::DeformationModelRegionBlend(DeformationModelRegionBlend&& other) = default;
template <class T> DeformationModelRegionBlend<T>& DeformationModelRegionBlend<T>::operator=(DeformationModelRegionBlend&& other) = default;

template<class T>
DiffDataMatrix<T, 3, -1> DeformationModelRegionBlend<T>::EvaluateVertices(Context<T>* context) {
    if  (m->regionBlendModel.NumParameters() == 0) {
        throw std::runtime_error("no region blend model has been loaded");
    }

    const DiffData<T> params = m->regionBlendParameters.Evaluate(context);
    DiffDataMatrix<T, 3, -1> vertices = m->regionBlendModel.Evaluate(params);

    // scale the vertices
    const bool optimizeScale = m->config["optimizeScale"].template Value<bool>();
    DiffData<T> diffScale = m->scaleVariable.Evaluate(optimizeScale ? context : nullptr);
    if (optimizeScale || diffScale.Value()[0] != T(1)) {
        const int numVertices = vertices.Cols();
        vertices = ColwiseAddFunction<T>().colwiseAddFunction(vertices, DiffDataMatrix<T, 3, 1>(-m->centerOfGravity));
        DiffDataMatrix<T, 1, 1> scaleMatrix(1, 1, m->scaleVariable.Evaluate(optimizeScale ? context : nullptr));
        DiffDataMatrix<T, 1, -1> flattenedVertices(1, numVertices * 3, vertices);
        DiffDataMatrix<T, 1, -1> scaledFlattenedVertices = MatrixMultiplyFunction<T>::DenseMatrixMatrixMultiply(scaleMatrix, flattenedVertices);
        vertices = DiffDataMatrix<T, 3, -1>(3, numVertices, scaledFlattenedVertices);
        vertices = ColwiseAddFunction<T>().colwiseAddFunction(vertices, DiffDataMatrix<T, 3, 1>(m->centerOfGravity));
    }

    Configuration rigidConfig = m->defModelRigid.GetConfiguration();
    rigidConfig["fixRotation"] = m->config["fixRotation"];
    rigidConfig["fixTranslation"] = m->config["fixTranslation"];
    m->defModelRigid.SetConfiguration(rigidConfig);

    const bool optimizePose = m->config["optimizePose"].template Value<bool>();
    return m->defModelRigid.EvaluateAffine(optimizePose ? context : nullptr).Transform(vertices);
}

template<class T>
Cost<T> DeformationModelRegionBlend<T>::EvaluateModelConstraints(Context<T>* context) {
    Cost<T> cost;

    const T modelRegularization = m->config["modelRegularization"].template Value<T>();
    const T normRegularization = m->config["normalization"].template Value<T>();
    const T symmetryWeight = m->config["symmetryRegularization"].template Value<T>();

    const DiffData<T> params = m->regionBlendParameters.Evaluate(context);

    // regularization
    if (modelRegularization > T(0)) {
        cost.Add(m->regionBlendModel.EvaluateRegularization(params), modelRegularization);
    }
    // normalization
    if (normRegularization > T(0)) {
        DMTNormalizationConstraint<T> normConstraint;
        const DiffData<T>& regionsDiff = normConstraint.EvaluateRegionsSumEquals(params, m->regionBlendModel.NumRegions());
        cost.Add(regionsDiff, normRegularization);
    }
    // symmetry
    if (symmetryWeight > T(0)) {
        DMTSymmetryConstraint<T> symmetryConstraint;
        const DiffData<T> symmetry = symmetryConstraint.EvaluateSymmetry(params,
                                                                            m->regionBlendModel.NumRegions(),
                                                                            m->regionBlendModel.GetSymmetricRegions());
        cost.Add(symmetry, symmetryWeight);
    }

    return cost;
}

template<class T>
const Configuration& DeformationModelRegionBlend<T>::GetConfiguration() const {
    return m->config;
}

template<class T>
void DeformationModelRegionBlend<T>::SetConfiguration(const Configuration& config) {
    m->config.Set(config);
}

template<class T>
void DeformationModelRegionBlend<T>::LoadModel(const std::string& regionBlendModelFile) {
    const carbon::JsonElement j = carbon::ReadJson(ReadFile(regionBlendModelFile));
    RegionBlendModelFromJson(j, m->regionBlendModel);
    ResetParameters();
    // it is important to set the base vertices for the rigid model so that the center of gravity is taken into account in the
    // rigid transformation
    m->defModelRigid.SetVertices(DeformedVertices());

    m->centerOfGravity = m->regionBlendModel.Evaluate(m->regionBlendModel.DefaultParameters()).rowwise().mean();
}

template<class T>
int DeformationModelRegionBlend<T>::NumParameters() const
{
    return m->regionBlendParameters.Size();
}

template<class T>
int DeformationModelRegionBlend<T>::NumVertices() const
{
    return m->regionBlendModel.NumVertices();
}

template<class T>
void DeformationModelRegionBlend<T>::ResetParameters()
{
    m->regionBlendParameters = BoundedVectorVariable<T>(m->regionBlendModel.DefaultParameters());
    m->regionBlendParameters.SetZero();
    for (int i = 0; i < m->regionBlendParameters.Size(); ++i) {
        // values need to be between 0 and 1
        m->regionBlendParameters.SetBounds(i, 0, 1);
    }
    m->scaleVariable.Set(Eigen::Vector<T, 1>(T(1.0)));
}

template<class T>
const Vector<T>& DeformationModelRegionBlend<T>::ModelParameters() const {
    return m->regionBlendParameters.Value();
}

template<class T>
void DeformationModelRegionBlend<T>::SetModelParameters(const Vector<T>& params) {
    if (params.size() == m->regionBlendModel.NumParameters()) {
        m->regionBlendParameters.Set(params);
    } else {
        throw std::runtime_error("incorrect number of model parameters");
    }
}

template<class T>
Eigen::Matrix<T, 3, -1> DeformationModelRegionBlend<T>::DeformedVertices() const {
    return m->regionBlendModel.Evaluate(m->regionBlendParameters.Value());
}

template<class T>
void DeformationModelRegionBlend<T>::SetRigidTransformation(const Affine<T, 3, 3>& affine) {
    m->defModelRigid.SetRigidTransformation(affine);
}

template<class T>
Affine<T, 3, 3> DeformationModelRegionBlend<T>::RigidTransformation() const {
    return m->defModelRigid.RigidTransformation();
}

template<class T>
BoundedVectorVariable<T>* DeformationModelRegionBlend<T>::Variable()
{
    return &(m->regionBlendParameters);
}

template<class T>
T DeformationModelRegionBlend<T>::Scale() const
{
    return m->scaleVariable.Value()[0];
}

template<class T>
Eigen::Vector3<T> DeformationModelRegionBlend<T>::ScalingPivot() const
{
    return m->centerOfGravity;
}

template<class T>
const std::vector<std::string>& DeformationModelRegionBlend<T>::RegionNames() const
{
    return m->regionBlendModel.RegionNames();
}

template<class T>
void DeformationModelRegionBlend<T>::SetRegionNames(const std::vector<std::string>& regionNames)
{
    m->regionBlendModel.SetRegionNames(regionNames);
}

template<class T>
void DeformationModelRegionBlend<T>::SetRegion(const std::string& regionName, const Vector<T>& regionData)
{
    m->regionBlendModel.SetRegion(regionName, regionData);
}

template<class T>
void DeformationModelRegionBlend<T>::SetRegions(const std::map<std::string, Vector<T>>& regions)
{
    m->regionBlendModel.SetRegions(regions);
}

template<class T>
const std::vector<std::string>& DeformationModelRegionBlend<T>::CharacterNames() const
{
    return m->regionBlendModel.CharacterNames();
}

template<class T>
void DeformationModelRegionBlend<T>::SetCharacterNames(const std::vector<std::string>& charNames)
{
    m->regionBlendModel.SetCharacterNames(charNames);
}

template<class T>
void DeformationModelRegionBlend<T>::SetCharacter(const std::string& charName, const Eigen::Matrix<T, 3, -1>& charData)
{
    m->regionBlendModel.SetCharacter(charName, charData);
}

template<class T>
void DeformationModelRegionBlend<T>::SetCharacters(const std::map<std::string, Eigen::Matrix<T, 3, -1>>& characters)
{
    m->regionBlendModel.SetCharacters(characters);
}

template<class T>
void DeformationModelRegionBlend<T>::SetArchetype(const Eigen::Matrix<T, 3, -1>& archetype)
{
    m->regionBlendModel.SetArchetype(archetype);
}

template<class T>
void DeformationModelRegionBlend<T>::SetSymmetricRegions(const std::vector<std::pair<std::string, std::string>>& symmetricRegions)
{
    m->regionBlendModel.SetSymmetricRegions(symmetricRegions);
}

template<class T>
void DeformationModelRegionBlend<T>::GenerateModel()
{
    m->regionBlendModel.Generate();
    ResetParameters();
    // it is important to set the base vertices for the rigid model so that the center of gravity is taken into account in the
    // rigid transformation
    m->defModelRigid.SetVertices(DeformedVertices());

    m->centerOfGravity = m->regionBlendModel.Evaluate(m->regionBlendModel.DefaultParameters()).rowwise().mean();
}

// explicitly instantiate the DeformationModelRegionBlend classes
template class DeformationModelRegionBlend<float>;
template class DeformationModelRegionBlend<double>;

}  // namespace epic::nls
