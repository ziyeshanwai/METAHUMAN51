// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/deformation_models/DeformationModelRigidScale.h>

#include <nls/functions/MatrixMultiplyFunction.h>
#include <nls/geometry/AffineVariable.h>
#include <nls/geometry/QuaternionVariable.h>


namespace epic::nls {

template <class T>
struct DeformationModelRigidScale<T>::Private {

    AffineVariable<QuaternionVariable<T>> varAffine;
    VectorVariable<T> varScale = VectorVariable<T>(1);
    Eigen::Matrix<T, 3, -1> vertices;
    Eigen::Vector3<T> centerOfGravity = Eigen::Vector3<T>::Zero();
    Configuration config = {std::string("Rigid/Scale Deformation Model Configuration") , {}};
};

template <class T>
DeformationModelRigidScale<T>::DeformationModelRigidScale()
    : m(std::make_unique<Private>())
{
    m->varScale.Set(Eigen::Vector<T, 1>(1));
}

template <class T> DeformationModelRigidScale<T>::~DeformationModelRigidScale() = default;
template <class T> DeformationModelRigidScale<T>::DeformationModelRigidScale(DeformationModelRigidScale&& other) = default;
template <class T> DeformationModelRigidScale<T>& DeformationModelRigidScale<T>::operator=(DeformationModelRigidScale&& other) = default;

template <class T>
DiffDataMatrix<T, 3, -1> DeformationModelRigidScale<T>::EvaluateVertices(Context<T>* context)
{
    // center vertices
    const Eigen::Matrix<T, 3, -1> centeredVertices = m->vertices.colwise() - m->centerOfGravity;

    // apply scale
    DiffData<T> scale = m->varScale.Evaluate(context);
    DiffDataMatrix<T, 1, 1> scaleMatrix(scale);
    DiffDataMatrix<T, 1, -1> flattenedVertices(1, static_cast<int>(centeredVertices.size()), DiffData<T>(centeredVertices));
    flattenedVertices = MatrixMultiplyFunction<T>::DenseMatrixMatrixMultiply(scaleMatrix, flattenedVertices);

    DiffDataMatrix<T, 3, -1> scaledAndCenteredVertices(3, static_cast<int>(centeredVertices.cols()), flattenedVertices);

    /// apply affine transformation and move back to center
    DiffDataAffine<T, 3, 3> diffFromCenterOfGravity(Affine<T, 3, 3>::FromTranslation(m->centerOfGravity));
    DiffDataAffine<T, 3, 3> diffAffine = m->varAffine.EvaluateAffine(context);
    diffAffine = diffFromCenterOfGravity.Multiply(diffAffine);

    return diffAffine.Transform(scaledAndCenteredVertices);
}

template <class T>
Cost<T> DeformationModelRigidScale<T>::EvaluateModelConstraints(Context<T>* /*context*/)
{
    // no constraints on the rigid transformation
    return Cost<T>();
}

template <class T>
const Configuration& DeformationModelRigidScale<T>::GetConfiguration() const
{
    return m->config;
}

template <class T>
void DeformationModelRigidScale<T>::SetConfiguration(const Configuration& config)
{
    m->config.Set(config);
}


template <class T>
void DeformationModelRigidScale<T>::SetVertices(const Eigen::Matrix<T, 3, -1>& vertices)
{
    m->vertices = vertices;

    Affine<T, 3, 3> currRigid = RigidTransformation(/*preScale=*/true);
    m->centerOfGravity = vertices.rowwise().mean();
    // we don't want to change the rigid transformation just because the center of gravity has changed, so we compensate for it
    SetRigidTransformation(currRigid, /*preScale=*/true);
}

template <class T>
Affine<T, 3, 3> DeformationModelRigidScale<T>::RigidTransformation(bool preScale) const
{
    // in case of prescale we need to scale the subtraction of the center of gravity as that was not taken into account in EvaluateVertices()
    Affine<T, 3, 3> affine = Affine<T, 3, 3>::FromTranslation(m->centerOfGravity) * m->varAffine.Affine() * Affine<T, 3, 3>::FromTranslation(Scale() * (-m->centerOfGravity));
    if (!preScale) {
        // in case we use postscale we need to apply scale at the end
        // prescale:  v' = R * (s * v) + t = s * R * v + t
        // postscale: v' = s * (R * v + t') = s * R * v + s * t'
        // t' = invS * t
        affine.SetTranslation(affine.Translation() * (T(1) / Scale()));
    }
    return affine;
}

template <class T>
void DeformationModelRigidScale<T>::SetRigidTransformation(const Affine<T, 3, 3>& affine, bool preScale)
{
    // invert logic from RigidTransformation()
    Affine<T, 3, 3> newAffine = affine;
    if (!preScale) {
        newAffine.SetTranslation(newAffine.Translation() * Scale());
    }
    m->varAffine.SetAffine(Affine<T, 3, 3>::FromTranslation(-m->centerOfGravity) * newAffine * Affine<T, 3, 3>::FromTranslation(Scale() * m->centerOfGravity));
}

template <class T>
T DeformationModelRigidScale<T>::Scale() const
{
    return m->varScale.Value()[0];
}

// explicitly instantiate the DeformationModelRigid classes
template class DeformationModelRigidScale<float>;
template class DeformationModelRigidScale<double>;

} // namespace epic::nls
