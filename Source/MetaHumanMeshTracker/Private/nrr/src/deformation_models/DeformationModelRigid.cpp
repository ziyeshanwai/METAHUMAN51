// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/deformation_models/DeformationModelRigid.h>

#include <nls/geometry/AffineVariable.h>
#include <nls/geometry/QuaternionVariable.h>

namespace epic::nls {

template <class T>
struct DeformationModelRigid<T>::Private {

    AffineVariable<QuaternionVariable<T>> varAffine;
    Eigen::Matrix<T, 3, -1> vertices;
    Eigen::Vector3<T> centerOfGravity = Eigen::Vector3<T>::Zero();
    Configuration config = {DeformationModelRigid<T>::ConfigName(), {
                                // ! whether to make translation part constant
                                {"fixTranslation", ConfigurationParameter(false)},
                                // ! whether to make rotation part constant
                                {"fixRotation", ConfigurationParameter(false)}
                            }};
};

template <class T>
DeformationModelRigid<T>::DeformationModelRigid()
    : m(std::make_unique<Private>())
{}

template <class T> DeformationModelRigid<T>::~DeformationModelRigid() = default;
template <class T> DeformationModelRigid<T>::DeformationModelRigid(DeformationModelRigid&& other) = default;
template <class T> DeformationModelRigid<T>& DeformationModelRigid<T>::operator=(DeformationModelRigid&& other) = default;

template <class T>
DiffDataAffine<T, 3, 3> DeformationModelRigid<T>::EvaluateAffine(Context<T>* context)
{
    const bool fixRotation = m->config["fixRotation"].template Value<bool>();
    const bool fixTranslation = m->config["fixTranslation"].template Value<bool>();
    m->varAffine.MakeConstant(fixRotation, fixTranslation);

    DiffDataAffine<T, 3, 3> diffToCenterOfGravity(Affine<T, 3, 3>::FromTranslation(-m->centerOfGravity));
    DiffDataAffine<T, 3, 3> diffFromCenterOfGravity(Affine<T, 3, 3>::FromTranslation(m->centerOfGravity));
    DiffDataAffine<T, 3, 3> diffAffine = m->varAffine.EvaluateAffine(context);
    return diffFromCenterOfGravity.Multiply(diffAffine.Multiply(diffToCenterOfGravity));
}

template <class T>
DiffDataMatrix<T, 3, -1> DeformationModelRigid<T>::EvaluateVertices(Context<T>* context)
{
    return EvaluateAffine(context).Transform(m->vertices);
}

template <class T>
Cost<T> DeformationModelRigid<T>::EvaluateModelConstraints(Context<T>* /*context*/)
{
    // no constraints on the rigid transformation
    return Cost<T>();
}

template <class T>
const Configuration& DeformationModelRigid<T>::GetConfiguration() const
{
    return m->config;
}

template <class T>
void DeformationModelRigid<T>::SetConfiguration(const Configuration& config)
{
    m->config.Set(config);
}


template <class T>
void DeformationModelRigid<T>::SetVertices(const Eigen::Matrix<T, 3, -1>& vertices)
{
    m->vertices = vertices;

    Affine<T, 3, 3> currRigid = RigidTransformation();
    m->centerOfGravity = vertices.rowwise().mean();
    // we don't want to change the rigid transformation just because the center of gravity has changed, so we compensate for it
    SetRigidTransformation(currRigid);
}

template <class T>
void DeformationModelRigid<T>::SetRigidTransformation(const Affine<T, 3, 3>& affine)
{
    m->varAffine.SetAffine(Affine<T, 3, 3>::FromTranslation(-m->centerOfGravity) * affine * Affine<T, 3, 3>::FromTranslation(m->centerOfGravity));
}

template <class T>
Affine<T, 3, 3> DeformationModelRigid<T>::RigidTransformation() const
{
    return Affine<T, 3, 3>::FromTranslation(m->centerOfGravity) * m->varAffine.Affine() * Affine<T, 3, 3>::FromTranslation(-m->centerOfGravity);
}

// explicitly instantiate the DeformationModelRigid classes
template class DeformationModelRigid<float>;
template class DeformationModelRigid<double>;

} // namespace epic::nls
