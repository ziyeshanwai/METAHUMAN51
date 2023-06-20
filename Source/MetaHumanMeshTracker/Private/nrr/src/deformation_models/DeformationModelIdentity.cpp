// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/deformation_models/DeformationModelIdentity.h>
#include <nrr/deformation_models/DeformationModelRigid.h>
#include <nrr/IdentityBlendModel.h>

#include <vector>

namespace epic::nls {

template<class T>
struct DeformationModelIdentity<T>::Private {
    DeformationModelRigid<T> defModelRigid;

    IdentityBlendModel<T> identityBlendModel;
    VectorVariable<T> identityParameters = VectorVariable<T>(0);

    Configuration config = {std::string("Identity Deformation Model Configuration"), {
                                // !< whether to optimize the pose when doing fine registration
                                {"optimizePose", ConfigurationParameter(true)},
                                // ! whether to fix rotation while optimizing pose
                                {"fixRotation", ConfigurationParameter(false)},
                                // ! whether to fix translation while optimizing pose
                                {"fixTranslation", ConfigurationParameter(false)},
                                // !< projective strain weight (stable, but incorrect Jacobian)
                                {"modelRegularization", ConfigurationParameter(T(100), T(0), T(1000))}
                            }};
};

template<class T>
DeformationModelIdentity<T>::DeformationModelIdentity()
    : m(std::make_unique<Private>()) {
}

template <class T> DeformationModelIdentity<T>::~DeformationModelIdentity() = default;
template <class T> DeformationModelIdentity<T>::DeformationModelIdentity(DeformationModelIdentity&& other) = default;
template <class T> DeformationModelIdentity<T>& DeformationModelIdentity<T>::operator=(DeformationModelIdentity&& other) = default;

template<class T>
DiffDataMatrix<T, 3, -1> DeformationModelIdentity<T>::EvaluateVertices(Context<T>* context) {
    if  (m->identityBlendModel.NumParameters() == 0) {
        CARBON_CRITICAL("no identity model has been loaded");
    }

    Configuration rigidConfig = m->defModelRigid.GetConfiguration();
    rigidConfig["fixRotation"] = m->config["fixRotation"];
    rigidConfig["fixTranslation"] = m->config["fixTranslation"];
    m->defModelRigid.SetConfiguration(rigidConfig);

    const DiffDataMatrix<T, 3, -1> vertices = m->identityBlendModel.Evaluate(m->identityParameters.Evaluate(context));
    const bool optimizePose = m->config["optimizePose"].template Value<bool>();
    return m->defModelRigid.EvaluateAffine(optimizePose ? context : nullptr).Transform(vertices);
}

template<class T>
Cost<T> DeformationModelIdentity<T>::EvaluateModelConstraints(Context<T>* context) {

    Cost<T> cost;

    const T modelRegularization = m->config["modelRegularization"].template Value<T>();
    if  (modelRegularization > 0) {
        cost.Add(m->identityBlendModel.EvaluateRegularization(m->identityParameters.Evaluate(context)), modelRegularization);
    }

    return cost;
}

template<class T>
const Configuration& DeformationModelIdentity<T>::GetConfiguration() const {
    return m->config;
}

template<class T>
void DeformationModelIdentity<T>::SetConfiguration(const Configuration& config) {
    m->config.Set(config);
}

template<class T>
void DeformationModelIdentity<T>::LoadModel(const std::string& identityBlendModelFile) {
    m->identityBlendModel.LoadModel(identityBlendModelFile);
    ResetParameters();
    // it is important to set the base vertices for the rigid model so that the center of gravity is taken into account in the rigid transformation
    m->defModelRigid.SetVertices(DeformedVertices());
}

template<class T>
int DeformationModelIdentity<T>::NumParameters() const
{
    return m->identityBlendModel.NumParameters();
}

template<class T>
void DeformationModelIdentity<T>::ResetParameters() {
    m->identityParameters = VectorVariable<T>(m->identityBlendModel.DefaultParameters());
    m->identityParameters.SetZero();
}

template<class T>
const Vector<T>& DeformationModelIdentity<T>::ModelParameters() const {
    return m->identityParameters.Value();
}

template<class T>
void DeformationModelIdentity<T>::SetModelParameters(const Vector<T>& params) {
    if (params.size() == m->identityBlendModel.NumParameters()) {
        m->identityParameters.Set(params);
    } else {
        throw std::runtime_error("incorrect number of model parameters");
    }
}

template<class T>
Eigen::Matrix<T, 3, -1> DeformationModelIdentity<T>::DeformedVertices() const {
    return m->identityBlendModel.Evaluate(m->identityParameters.Value());
}

template<class T>
void DeformationModelIdentity<T>::SetRigidTransformation(const Affine<T, 3, 3>& affine) {
    m->defModelRigid.SetRigidTransformation(affine);
}

template<class T>
Affine<T, 3, 3> DeformationModelIdentity<T>::RigidTransformation() const {
    return m->defModelRigid.RigidTransformation();
}

// explicitly instantiate the DeformationModelIdentity classes
template class DeformationModelIdentity<float>;
template class DeformationModelIdentity<double>;

}  // namespace epic::nls
