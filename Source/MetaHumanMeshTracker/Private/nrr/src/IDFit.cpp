// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/IDFit.h>

#include <nls/BoundedCoordinateDescentSolver.h>
#include <nls/Cost.h>
#include <nls/DiffDataMatrix.h>
#include <nls/Solver.h>
#include <nls/utils/ConfigurationParameter.h>
#include <nrr/ICPConstraints.h>
#include <nrr/MarkerConstraints.h>
#include <nrr/deformation_models/DeformationModelRegionBlend.h>
#include <nrr/deformation_models/DeformationModelRigid.h>
#include <nrr/deformation_models/DeformationModelVertex.h>
#include <nrr/landmarks/LandmarkConstraints2D.h>

namespace epic::nls {

template <class T>
struct IDFit<T>::Private {
    //! The source mesh (the vertices are the latest deformed state, or set by the user)
    Mesh<T> sourceMesh;

    //! Weighting mask defining the weight per search point
    VertexWeights<T> correspondenceSearchWeights;

    //! Structure to calculate landmark constraints
    LandmarkConstraints2D<T> landmarkConstraints;

    //! A blend model for region based nonrigid registration
    DeformationModelRegionBlend<T> deformationModelRegionBlend;

    //! Set to true when blend model is generated
    bool generatedModel{false};

    //! the base of the mesh
    Eigen::Matrix<T, 3, -1> sourceBase;

    //! the per-vertex offsets of the mesh
    Eigen::Matrix<T, 3, -1> sourceOffsets;

    //! the current affine transformation of the source mesh
    Affine<T, 3, 3> sourceAffine;
    //! the current affine transformation of the target mesh
    Affine<T, 3, 3> targetAffine;

    DeformationModelRigid<T> deformationModelRigid;
    DeformationModelVertex<T> deformationModelVertex;

    ICPConstraints<T> icpConstraints;

    int completedIterationsNNLS{0};

    MarkerConstraints<T> markerConstraints;

    Configuration rigidConfig = { std::string("Rigid Registration Configuration") , {
        //!< number of iterations to use for rigid registration
        { "numIterations", ConfigurationParameter(int(5), int(1), int(20)) }
    } };

    Configuration nonRigidConfigGN = { std::string("Non-Rigid Registration Configuration (Gauss-Newton)") , {
        { "numIterations", ConfigurationParameter(int(10), int(1), int(20))},
        //!< how much weight to use on geometry constraint
        { "geometryWeight", ConfigurationParameter(T(1), T(0), T(1)) },
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< regularization of model parameters
        { "modelRegularization", ConfigurationParameter(T(0), T(0), T(5)) },
        //!< regions normalization
        { "normalization", ConfigurationParameter(T(2000), T(0), T(10000)) },
        // !< symmetry constraint
        { "symmetryRegularization", ConfigurationParameter(T(0), T(0), T(2000)) },
        //!< projective strain weight (stable, but incorrect Jacobian)
        { "projectiveStrain", ConfigurationParameter(T(0.5), T(0), T(1)) },
        //!< green strain (unstable???, correct Jacobian)
        { "greenStrain", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< quadratic bending (stable, but incorrect Jacobian, and also has strain component)
        { "quadraticBending", ConfigurationParameter(T(0.2), T(0), T(1)) },
        //!< dihedral bending (unstable???, correct Jacobian)
        { "dihedralBending", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< weight on regularizing the per-vertex offset
        { "vertexLaplacian", ConfigurationParameter(T(0), T(0), T(1)) },
        // ! whether to optimize the scale of the model
        { "optimizeScale", ConfigurationParameter(false) }
    } };

    Configuration nonRigidConfigNNLS = { std::string("Non-Rigid Registration Configuration (NNLS)") , {
        { "numIterations", ConfigurationParameter(int(15), int(1), int(20))},
        //!< how much weight to use on geometry constraint
        { "geometryWeight", ConfigurationParameter(T(1), T(0), T(1)) },
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< regularization of model parameters
        { "modelRegularization", ConfigurationParameter(T(0), T(0), T(5)) },
        //!< regions normalization
        { "normalization", ConfigurationParameter(T(2000), T(0), T(10000)) },
        // !< symmetry constraint
        { "symmetryRegularization", ConfigurationParameter(T(0), T(0), T(2000)) },
        //!< projective strain weight (stable, but incorrect Jacobian)
        { "projectiveStrain", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< green strain (unstable???, correct Jacobian)
        { "greenStrain", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< quadratic bending (stable, but incorrect Jacobian, and also has strain component)
        { "quadraticBending", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< dihedral bending (unstable???, correct Jacobian)
        { "dihedralBending", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< weight on regularizing the per-vertex offset
        { "vertexLaplacian", ConfigurationParameter(T(0), T(0), T(1)) },
        // ! whether to optimize the scale of the model
        { "optimizeScale", ConfigurationParameter(false) }
    } };

    Configuration fineFittingConfig = { std::string("Fine Fitting Configuration") , {
        { "numIterations", ConfigurationParameter(int(10), int(1), int(20))},
        //!< whether to optimize the pose when doing fine registration
        { "optimizePose", ConfigurationParameter(false) },
        //!< projective strain weight (stable, but incorrect Jacobian)
        { "projectiveStrain", ConfigurationParameter(T(0.5), T(0), T(1)) },
        //!< green strain (unstable???, correct Jacobian)
        { "greenStrain", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< quadratic bending (stable, but incorrect Jacobian, and also has strain component)
        { "quadraticBending", ConfigurationParameter(T(0.2), T(0), T(1)) },
        //!< dihedral bending (unstable???, correct Jacobian)
        { "dihedralBending", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< weight on regularizing the per-vertex offset
        { "vertexOffsetRegularization", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< how much weight to use on geometry constraint
        { "geometryWeight", ConfigurationParameter(T(1), T(0), T(1)) },
        //!< weight on regularizing the per-vertex offset
        { "vertexLaplacian", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(0.1), T(0), T(1)) },
        //!< how much weight to use on landmark constraints
        { "landmarksWeight", ConfigurationParameter(T(0.01), T(0), T(0.1)) },
        //!< how much weight to use on inner lip constraints
        { "innerLipWeight", ConfigurationParameter(T(0.01), T(0), T(0.1)) },
        //!< whether to lock vertices excluded with the vertex weights mask
        { "lockVertices", ConfigurationParameter(false) }
    } };

    Configuration markerConfig = { std::string("Marker Configuration") , {
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< how much to increase the marker influence on the optimization (0 - same as vertices, 1 - x10, 2 - x100, etc.)
        { "globalWeight", ConfigurationParameter(int(2), int(0), int(10)) },
        //!< a scalar less than 1 to use as a base for non-linear decrease of the marker influence with each iteration. w_i = w * (powBase ^ (i/maxIterations))
        { "powBase", ConfigurationParameter(T(0.001), T(0), T(1)) }
    } };

    //! update the deformed vertices based on the current model state
    void UpdateDeformed()
    {
        if (generatedModel) {
            sourceDeformed = deformationModelRegionBlend.DeformedVertices() + sourceOffsets;
        }
        else {
            sourceDeformed = sourceBase + sourceOffsets;
        }
    }

    Eigen::Matrix<T, 3, -1> CurrentBase() const
    {
        if (generatedModel) {
            return deformationModelRegionBlend.DeformedVertices();
        } else {
            return sourceBase;
        }
    }

    const Eigen::Matrix<T, 3, -1>& CurrentDeformed() const
    {
        return sourceDeformed;
    }

private:
    //! the current final deformed source (internal representation after evaluation)
    Eigen::Matrix<T, 3, -1> sourceDeformed;
};

template <class T>
IDFit<T>::IDFit() : m(std::make_unique<Private>())
{
}

template <class T> IDFit<T>::~IDFit() = default;
template <class T> IDFit<T>::IDFit(IDFit&& other) = default;
template <class T> IDFit<T>& IDFit<T>::operator=(IDFit&& other) = default;

template <class T>
std::map<std::string, std::string> IDFit<T>::RigidRegistrationConfiguration() const
{
    return m->rigidConfig.StringConfiguration();
}

template <class T>
void IDFit<T>::SetRigidRegistrationConfiguration(const std::map<std::string, std::string>& config)
{
    m->rigidConfig.SetStringConfiguration(config);
}

template <class T>
std::map<std::string, std::string> IDFit<T>::NonRigidRegistrationConfigurationGN() const
{
    return m->nonRigidConfigGN.StringConfiguration();
}

template <class T>
void IDFit<T>::SetNonRigidRegistrationConfigurationGN(const std::map<std::string, std::string>& config)
{
    m->nonRigidConfigGN.SetStringConfiguration(config);
}

template <class T>
std::map<std::string, std::string> IDFit<T>::NonRigidRegistrationConfigurationNNLS() const
{
    return m->nonRigidConfigNNLS.StringConfiguration();
}

template <class T>
void IDFit<T>::SetNonRigidRegistrationConfigurationNNLS(const std::map<std::string, std::string>& config)
{
    m->nonRigidConfigNNLS.SetStringConfiguration(config);
}

template <class T>
std::map<std::string, std::string> IDFit<T>::FineRegistrationConfiguration() const
{
    return m->fineFittingConfig.StringConfiguration();
}

template <class T>
void IDFit<T>::SetFineRegistrationConfiguration(const std::map<std::string, std::string>& config)
{
    m->fineFittingConfig.SetStringConfiguration(config);
}

template <class T>
std::map<std::string, std::string> IDFit<T>::MarkerConfiguration() const
{
    return m->markerConfig.StringConfiguration();
}

template <class T>
void IDFit<T>::SetMarkerConfiguration(const std::map<std::string, std::string>& config)
{
    m->markerConfig.SetStringConfiguration(config);
}

template <class T>
void IDFit<T>::SetSourceMesh(const Eigen::VectorXi& polygons, const Eigen::VectorXi& vIDs, const Eigen::Matrix<T, 3, -1>& vertices)
{
    Mesh<T> srcMesh;
    srcMesh.SetTopology(polygons, vIDs);
    srcMesh.SetVertices(vertices);
    m->sourceMesh = srcMesh;
    m->sourceMesh.Triangulate();

    m->sourceBase = m->sourceMesh.Vertices();
    m->sourceOffsets = Eigen::Matrix<T, 3, -1>::Zero(3, m->sourceMesh.NumVertices());
    m->UpdateDeformed();

    m->sourceAffine.SetIdentity();
    m->targetAffine.SetIdentity();

    if (m->generatedModel) {
        m->deformationModelRegionBlend.ResetParameters();
    }
    if (m->correspondenceSearchWeights.NumVertices() != int(m->sourceBase.cols())) {
        m->correspondenceSearchWeights = VertexWeights<T>(int(m->sourceBase.cols()), T(1));
    }
}

template <class T>
void IDFit<T>::SetCorrespondenceSearchVertexWeights(const Eigen::VectorX<T>& vertexWeights)
{
    if (vertexWeights.size() != int(m->sourceBase.cols())) {
        throw std::runtime_error("nonrigid mask does not have the same size as the mesh");
    }
    m->correspondenceSearchWeights = vertexWeights;
    m->icpConstraints.SetCorrespondenceSearchVertexWeights(vertexWeights);
}

template <class T>
void IDFit<T>::SetTargetMesh(const Eigen::VectorXi& polygons, const Eigen::VectorXi& vIDs, const Eigen::Matrix<T, 3, -1>& vertices)
{
    Mesh<T> targetMesh;
    targetMesh.SetTopology(polygons, vIDs);
    targetMesh.SetVertices(vertices);
    m->icpConstraints.SetTargetMesh(targetMesh);
}


template <class T>
void IDFit<T>::SetModelRegionNames(const std::vector<std::string>& regionNames)
{
    m->deformationModelRegionBlend.SetRegionNames(regionNames);
}

template <class T>
void IDFit<T>::SetModelRegion(const std::string& regionName, const Vector<T>& regionData)
{
    m->deformationModelRegionBlend.SetRegion(regionName, regionData);
}


template <class T>
void IDFit<T>::SetModelRegions(const std::map<std::string, Vector<T>>& regions)
{
    m->deformationModelRegionBlend.SetRegions(regions);
}

template <class T>
void IDFit<T>::SetModelCharacterNames(const std::vector<std::string>& charNames)
{
    m->deformationModelRegionBlend.SetCharacterNames(charNames);
}

template <class T>
void IDFit<T>::SetModelCharacter(const std::string& charName, const Eigen::Matrix<T, 3, -1>& charData)
{
    m->deformationModelRegionBlend.SetCharacter(charName, charData);
}

template <class T>
void IDFit<T>::SetModelCharacters(const std::map<std::string, Eigen::Matrix<T, 3, -1>>& characters)
{
    m->deformationModelRegionBlend.SetCharacters(characters);
}

template <class T>
void IDFit<T>::SetModelArchetype(const Eigen::Matrix<T, 3, -1>& archetype)
{
    m->deformationModelRegionBlend.SetArchetype(archetype);
}

template <class T>
void IDFit<T>::SetModelSymmetricRegions(const std::vector<std::pair<std::string, std::string>>& symmetricRegions)
{
    m->deformationModelRegionBlend.SetSymmetricRegions(symmetricRegions);
}

template <class T>
void IDFit<T>::GenerateModel()
{
    m->deformationModelRegionBlend.GenerateModel();
    m->generatedModel = true;

    if (m->sourceOffsets.size() == 0) {
        m->sourceOffsets = Eigen::Matrix<T, 3, -1>::Zero(3, m->deformationModelRegionBlend.NumVertices());
    }

    m->UpdateDeformed();
}

template <class T>
const Eigen::Matrix<T, 3 ,-1>& IDFit<T>::CurrentDeformedVertices() const
{
    return m->CurrentDeformed();
}

template <class T>
const Eigen::Matrix<T, 4, 4>& IDFit<T>::CurrentAlignment() const
{
    return m->sourceAffine.Matrix();
}

template <class T>
void IDFit<T>::RegisterRigid(const Eigen::Matrix<T, 4, 4>& sourceAffineMatrix,
                                 const Eigen::Matrix<T, 4, 4>& targetAffineMatrix,
                                 const Eigen::Matrix<T, 3, -1>& initialCorrespondenceVertices)
{
    Affine<T, 3, 3> sourceAffine = sourceAffineMatrix;
    Affine<T, 3, 3> targetAffine = targetAffineMatrix;

    if (initialCorrespondenceVertices.cols() != 0 && int(initialCorrespondenceVertices.cols()) != m->sourceMesh.NumVertices()) {
        throw std::runtime_error("initial correspondence vertices do not match mesh size");
    }
    if (sourceAffine.HasScaling()) {
        throw std::runtime_error("the source affine transformation cannot contain any scaling component");
    }
    if (targetAffine.HasScaling()) {
        throw std::runtime_error("the target affine transformation cannot contain any scaling component");
    }

    // current transformation from source to target:
    const Affine<T, 3, 3> source2target = targetAffine.Inverse() * sourceAffine;
    m->deformationModelRigid.SetRigidTransformation(source2target);
    m->deformationModelRigid.SetVertices(m->CurrentDeformed());

    Configuration config = m->icpConstraints.GetConfiguration();
    config["geometryWeight"].Set(T(1));
    config["point2point"].Set(T(0));
    config["useDistanceThreshold"].Set(false);
    m->icpConstraints.SetConfiguration(config);
    m->icpConstraints.SetCorrespondenceSearchVertexWeights(VertexWeights<T>());
    m->icpConstraints.ClearPreviousCorrespondences();

    config = m->markerConstraints.GetConfiguration();
    config["point2point"] = m->markerConfig["point2point"];
    config["globalWeight"] = m->markerConfig["globalWeight"];
    config["powBase"] = m->markerConfig["powBase"];
    m->markerConstraints.SetConfiguration(config, m->sourceMesh.NumVertices());

    bool useInitialCorrespondences = (int(initialCorrespondenceVertices.cols()) == m->sourceMesh.NumVertices());
    Mesh<T> currentMesh = m->sourceMesh;

    const int numIterations = m->rigidConfig["numIterations"].template Value<int>();
    int currentIteration = 0;

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {

        Cost<T> cost;

        const DiffDataMatrix<T, 3, -1> transformedVertices = m->deformationModelRigid.EvaluateVertices(context);

        if (context || !m->icpConstraints.HasCorrespondences()) {
            if (useInitialCorrespondences) {
                currentMesh.SetVertices(source2target.Transform(initialCorrespondenceVertices));
            } else {
                currentMesh.SetVertices(transformedVertices.Matrix());
            }
            currentMesh.CalculateVertexNormals();
            m->icpConstraints.SetupCorrespondences(currentMesh.Vertices(), currentMesh.VertexNormals());

            if (context) {
                // when we use a Jacobian then we have an update step, and then we should not use the initial correspondences
                useInitialCorrespondences = false;
            }
        }

        // just point-to-surface
        cost.Add(m->icpConstraints.EvaluateICP(transformedVertices), T(1));
        cost.Add(m->markerConstraints.Evaluate(transformedVertices, numIterations, currentIteration), T(1));

        if (context) {
            currentIteration++;
        }

        return cost.CostToDiffData();
    };

    GaussNewtonSolver<T> solver;
    const T startEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    if (!solver.Solve(evaluationFunction, numIterations)) {
        printf("could not solve optimization problem\n");
        return;
    }
    const T finalEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    printf("energy changed from %f to %f\n", startEnergy, finalEnergy);

    m->sourceAffine = targetAffine * m->deformationModelRigid.RigidTransformation();
    m->targetAffine = targetAffine;
}

template <class T>
void IDFit<T>::ResetNonRigid()
{
    m->deformationModelRegionBlend.ResetParameters();
    ResetFine();
}

template <class T>
void IDFit<T>::RegisterNonRigidGN(const Eigen::Matrix<T, 3, -1>& initialCorrespondenceVertices)
{
    if (initialCorrespondenceVertices.cols() != 0 && int(initialCorrespondenceVertices.cols()) != m->sourceMesh.NumVertices()) {
        throw std::runtime_error("initial correspondence vertices do not match mesh size");
    }
    if (m->sourceAffine.HasScaling()) {
        throw std::runtime_error("the source affine transformation cannot contain any scaling component");
    }
    if (m->targetAffine.HasScaling()) {
        throw std::runtime_error("the target affine transformation cannot contain any scaling component");
    }
    if (!m->generatedModel) {
        throw std::runtime_error("no blend model - first generate model before nonrigid registration");
    }

    Configuration config = m->icpConstraints.GetConfiguration();
    config["geometryWeight"] = m->nonRigidConfigGN["geometryWeight"];
    config["point2point"] = m->nonRigidConfigGN["point2point"];
    config["useDistanceThreshold"].Set(false);
    m->icpConstraints.SetConfiguration(config);
    m->icpConstraints.SetCorrespondenceSearchVertexWeights(m->correspondenceSearchWeights);
    m->icpConstraints.ClearPreviousCorrespondences();

    // current transformation from source to target:
    const Affine<T, 3, 3> source2target = m->targetAffine.Inverse() * m->sourceAffine;
    m->deformationModelRegionBlend.SetRigidTransformation(source2target);

    config = m->deformationModelRegionBlend.GetConfiguration();
    config["modelRegularization"] = m->nonRigidConfigGN["modelRegularization"];
    config["normalization"] = m->nonRigidConfigGN["normalization"];
    config["symmetryRegularization"] = m->nonRigidConfigGN["symmetryRegularization"];
    config["optimizeScale"] = m->nonRigidConfigGN["optimizeScale"];
    m->deformationModelRegionBlend.SetConfiguration(config);
    // do not use bounds for GN - the parameters do not have to be between 0 and 1
    m->deformationModelRegionBlend.Variable()->EnforceBounds(false);

    m->deformationModelVertex.SetRigidTransformation(source2target);
    m->deformationModelVertex.SetMeshTopology(m->sourceMesh);
    m->deformationModelVertex.SetRestVertices(m->CurrentBase());
    m->deformationModelVertex.SetVertexOffsets(m->sourceOffsets);
    config = m->deformationModelVertex.GetConfiguration();
    config["optimizePose"].Set(false);
    config["vertexOffsetRegularization"].Set(T(0));
    config["projectiveStrain"] = m->nonRigidConfigGN["projectiveStrain"];
    config["greenStrain"] = m->nonRigidConfigGN["greenStrain"];
    config["quadraticBending"] = m->nonRigidConfigGN["quadraticBending"];
    config["dihedralBending"] = m->nonRigidConfigGN["dihedralBending"];
    config["vertexLaplacian"] = m->nonRigidConfigGN["vertexLaplacian"];
    m->deformationModelVertex.SetConfiguration(config);

    config = m->markerConstraints.GetConfiguration();
    config["point2point"] = m->markerConfig["point2point"];
    config["globalWeight"] = m->markerConfig["globalWeight"];
    config["powBase"] = m->markerConfig["powBase"];
    m->markerConstraints.SetConfiguration(config, m->sourceMesh.NumVertices());

    Eigen::Matrix<T, 3, -1> initialCorrespondences = (int(initialCorrespondenceVertices.cols()) > 0) ? initialCorrespondenceVertices : m->CurrentDeformed();
    bool useInitialCorrespondences = true;
    Mesh<T> currentMesh = m->sourceMesh;

    const int numIterations = m->nonRigidConfigGN["numIterations"].template Value<int>();
    int currentIteration = 0;

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {

        Cost<T> cost;

        const DiffDataMatrix<T, 3, -1> transformedVertices = m->deformationModelRegionBlend.EvaluateVertices(context);

        if (context || !m->icpConstraints.HasCorrespondences()) {
            if (useInitialCorrespondences) {
                currentMesh.SetVertices(source2target.Transform(initialCorrespondences));
            } else {
                currentMesh.SetVertices(transformedVertices.Matrix());
            }
            currentMesh.CalculateVertexNormals();
            m->icpConstraints.SetupCorrespondences(currentMesh.Vertices(), currentMesh.VertexNormals());

            if (context) {
                // when we use a Jacobian then we have an update step, and then we should not use the initial correspondences
                useInitialCorrespondences = false;
            }
        }

        cost.Add(m->icpConstraints.EvaluateICP(transformedVertices), T(1));
        cost.Add(m->deformationModelRegionBlend.EvaluateModelConstraints(context), T(1));
        cost.Add(m->deformationModelVertex.EvaluateModelConstraints(nullptr), T(1));
        cost.Add(m->markerConstraints.Evaluate(transformedVertices, numIterations, currentIteration), T(1));

        if (context) {
            currentIteration++;
        }

        return cost.CostToDiffData();
    };

    GaussNewtonSolver<T> solver;

    const T startEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    if (!solver.Solve(evaluationFunction, numIterations)) {
        printf("could not solve optimization problem\n");
        return;
    }

    const T finalEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    printf("energy changed from %f to %f\n", startEnergy, finalEnergy);

    m->sourceAffine = m->targetAffine * m->deformationModelRegionBlend.RigidTransformation();
    m->sourceOffsets.setZero(); // discard per-vertex offsets

    m->UpdateDeformed();
}

template <class T>
void IDFit<T>::RegisterNonRigidNNLS(const Eigen::Matrix<T, 3, -1>& initialCorrespondenceVertices)
{
    if (initialCorrespondenceVertices.cols() != 0 && int(initialCorrespondenceVertices.cols()) != m->sourceMesh.NumVertices()) {
        throw std::runtime_error("initial correspondence vertices do not match mesh size");
    }
    if (m->sourceAffine.HasScaling()) {
        throw std::runtime_error("the source affine transformation cannot contain any scaling component");
    }
    if (m->targetAffine.HasScaling()) {
        throw std::runtime_error("the target affine transformation cannot contain any scaling component");
    }
    if (!m->generatedModel) {
        throw std::runtime_error("no blend model - first generate model before nonrigid registration");
    }

    Configuration config = m->icpConstraints.GetConfiguration();
    config["geometryWeight"] = m->nonRigidConfigNNLS["geometryWeight"];
    config["point2point"] = m->nonRigidConfigNNLS["point2point"];
    config["useDistanceThreshold"].Set(false);
    m->icpConstraints.SetConfiguration(config);
    m->icpConstraints.SetCorrespondenceSearchVertexWeights(m->correspondenceSearchWeights);
    m->icpConstraints.ClearPreviousCorrespondences();

    config = m->deformationModelRegionBlend.GetConfiguration();
    config["modelRegularization"] = m->nonRigidConfigNNLS["modelRegularization"];
    config["normalization"] = m->nonRigidConfigNNLS["normalization"];
    config["symmetryRegularization"] = m->nonRigidConfigNNLS["symmetryRegularization"];
    config["optimizeScale"] = m->nonRigidConfigNNLS["optimizeScale"];
    m->deformationModelRegionBlend.SetConfiguration(config);

    config = m->deformationModelVertex.GetConfiguration();
    config["optimizePose"].Set(false);
    config["vertexOffsetRegularization"].Set(T(0));
    config["projectiveStrain"] = m->nonRigidConfigNNLS["projectiveStrain"];
    config["greenStrain"] = m->nonRigidConfigNNLS["greenStrain"];
    config["quadraticBending"] = m->nonRigidConfigNNLS["quadraticBending"];
    config["dihedralBending"] = m->nonRigidConfigNNLS["dihedralBending"];
    config["vertexLaplacian"] = m->nonRigidConfigNNLS["vertexLaplacian"];
    m->deformationModelVertex.SetConfiguration(config);

    config = m->markerConstraints.GetConfiguration();
    config["point2point"] = m->markerConfig["point2point"];
    config["globalWeight"] = m->markerConfig["globalWeight"];
    config["powBase"] = m->markerConfig["powBase"];
    m->markerConstraints.SetConfiguration(config, m->sourceMesh.NumVertices());

    const int numIterations = m->nonRigidConfigNNLS["numIterations"].template Value<int>();
    const int bcdIterations = 10;
    int currentIteration = 0;

    m->completedIterationsNNLS = 0;

    for (int i = 0; i < numIterations; ++i) {
        if (i > 0) {
            m->deformationModelRegionBlend.ResetParameters();
        }
        // use bounds for NNLS - the parameters have to be between 0 and 1
        m->deformationModelRegionBlend.Variable()->EnforceBounds(true);

        // current transformation from source to target:
        const Affine<T, 3, 3> source2target = m->targetAffine.Inverse() * m->sourceAffine;

        m->deformationModelRegionBlend.SetRigidTransformation(source2target);
        m->deformationModelVertex.SetRigidTransformation(source2target);

        m->deformationModelVertex.SetMeshTopology(m->sourceMesh);
        m->deformationModelVertex.SetRestVertices(m->CurrentBase());
        m->deformationModelVertex.SetVertexOffsets(m->sourceOffsets);

        Eigen::Matrix<T, 3, -1> initialCorrespondences = (int(initialCorrespondenceVertices.cols()) > 0) ? initialCorrespondenceVertices : m->CurrentDeformed();
        bool useInitialCorrespondences = true;
        Mesh<T> currentMesh = m->sourceMesh;

        std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {

            Cost<T> cost;

            const DiffDataMatrix<T, 3, -1> transformedVertices = m->deformationModelRegionBlend.EvaluateVertices(context);

            if (context || !m->icpConstraints.HasCorrespondences()) {
                if (useInitialCorrespondences) {
                    currentMesh.SetVertices(source2target.Transform(initialCorrespondences));
                }
                else {
                    currentMesh.SetVertices(transformedVertices.Matrix());
                }
                currentMesh.CalculateVertexNormals();
                m->icpConstraints.SetupCorrespondences(currentMesh.Vertices(), currentMesh.VertexNormals());

                if (context) {
                    // when we use a Jacobian then we have an update step, and then we should not use the initial correspondences
                    useInitialCorrespondences = false;
                }
            }

            cost.Add(m->icpConstraints.EvaluateICP(transformedVertices), T(1));
            cost.Add(m->deformationModelRegionBlend.EvaluateModelConstraints(context), T(1));
            cost.Add(m->deformationModelVertex.EvaluateModelConstraints(nullptr), T(1));
            cost.Add(m->markerConstraints.Evaluate(transformedVertices, numIterations * bcdIterations, currentIteration), T(1));

            if (context) {
                currentIteration++;
            }

            return cost.CostToDiffData();
        };

        BoundedCoordinateDescentSolver<T> solver;

        const T startEnergy = evaluationFunction(nullptr).Value().squaredNorm();
        Context<T> context;
        if (!solver.Solve(evaluationFunction, context, bcdIterations, m->deformationModelRegionBlend.Variable(), T(0))) {
            printf("could not solve optimization problem\n");
            return;
        }

        const T finalEnergy = evaluationFunction(nullptr).Value().squaredNorm();
        printf("energy changed from %f to %f\n", startEnergy, finalEnergy);

        m->sourceAffine = m->targetAffine * m->deformationModelRegionBlend.RigidTransformation();
        m->sourceOffsets.setZero(); // discard per-vertex offsets

        m->UpdateDeformed();
        m->completedIterationsNNLS++;
    }
}

template <class T>
void IDFit<T>::ResetFine()
{
    m->sourceOffsets.setZero();
    m->UpdateDeformed();
}

template <class T>
void IDFit<T>::RegisterFine(const Eigen::Matrix<T, 3, -1>& initialCorrespondenceVertices)
{
    if (initialCorrespondenceVertices.cols() != 0 && int(initialCorrespondenceVertices.cols()) != m->sourceMesh.NumVertices()) {
        throw std::runtime_error("initial correspondence vertices do not match mesh size");
    }
    if (m->sourceAffine.HasScaling()) {
        throw std::runtime_error("the source affine transformation cannot contain any scaling component");
    }
    if (m->targetAffine.HasScaling()) {
        throw std::runtime_error("the target affine transformation cannot contain any scaling component");
    }

    Configuration config = m->icpConstraints.GetConfiguration();
    config["geometryWeight"] = m->fineFittingConfig["geometryWeight"];
    config["point2point"] = m->fineFittingConfig["point2point"];
    config["useDistanceThreshold"].Set(false);
    m->icpConstraints.SetConfiguration(config);
    m->icpConstraints.SetCorrespondenceSearchVertexWeights(m->correspondenceSearchWeights);
    m->icpConstraints.ClearPreviousCorrespondences();

    config = m->landmarkConstraints.GetConfiguration();
    config["landmarksWeight"] = m->fineFittingConfig["landmarksWeight"];
    config["innerLipWeight"] = m->fineFittingConfig["innerLipWeight"];
    m->landmarkConstraints.SetConfiguration(config);

    // current transformation from source to target:
    const Affine<T, 3, 3> source2target = m->targetAffine.Inverse() * m->sourceAffine;

    m->deformationModelVertex.SetRigidTransformation(source2target);
    m->deformationModelVertex.SetMeshTopology(m->sourceMesh);
    m->deformationModelVertex.SetRestVertices(m->CurrentBase());
    m->deformationModelVertex.SetVertexOffsets(m->sourceOffsets);

    const bool lockVertices = m->fineFittingConfig["lockVertices"].template Value<bool>();

    if (lockVertices) {
        std::vector<int> constantVertices;
        for (int vID = 0; vID < m->correspondenceSearchWeights.Weights().size(); ++vID) {
            if (m->correspondenceSearchWeights.Weights()[vID] < 0.001) {
                constantVertices.push_back(vID);
            }
        }
        m->deformationModelVertex.MakeVerticesConstant(constantVertices);
    } else {
        m->deformationModelVertex.MakeVerticesMutable();
    }

    config = m->deformationModelVertex.GetConfiguration();
    config["optimizePose"] = m->fineFittingConfig["optimizePose"];
    config["vertexOffsetRegularization"] = m->fineFittingConfig["vertexOffsetRegularization"];
    config["projectiveStrain"] = m->fineFittingConfig["projectiveStrain"];
    config["greenStrain"] = m->fineFittingConfig["greenStrain"];
    config["quadraticBending"] = m->fineFittingConfig["quadraticBending"];
    config["dihedralBending"] = m->fineFittingConfig["dihedralBending"];
    config["vertexLaplacian"] = m->fineFittingConfig["vertexLaplacian"];
    m->deformationModelVertex.SetConfiguration(config);

    config = m->markerConstraints.GetConfiguration();
    config["point2point"] = m->markerConfig["point2point"];
    config["globalWeight"] = m->markerConfig["globalWeight"];
    config["powBase"] = m->markerConfig["powBase"];
    m->markerConstraints.SetConfiguration(config, m->sourceMesh.NumVertices());

    Eigen::Matrix<T, 3, -1> initialCorrespondences = (int(initialCorrespondenceVertices.cols()) > 0) ? initialCorrespondenceVertices : m->CurrentDeformed();
    bool useInitialCorrespondences = true;
    Mesh<T> currentMesh = m->sourceMesh;

    const int numIterations = m->fineFittingConfig["numIterations"].template Value<int>();
    int currentIteration = 0;

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {

        Cost<T> cost;

        const DiffDataMatrix<T, 3, -1> transformedVertices = m->deformationModelVertex.EvaluateVertices(context);

        if (context || !m->icpConstraints.HasCorrespondences()) {
            if (useInitialCorrespondences) {
                currentMesh.SetVertices(source2target.Transform(initialCorrespondences));
            } else {
                currentMesh.SetVertices(transformedVertices.Matrix());
            }
            currentMesh.CalculateVertexNormals();
            m->icpConstraints.SetupCorrespondences(currentMesh.Vertices(), currentMesh.VertexNormals());

            if (context) {
                // when we use a Jacobian then we have an update step, and then we should not use the initial correspondences
                useInitialCorrespondences = false;
            }
        }

        cost.Add(m->icpConstraints.EvaluateICP(transformedVertices), T(1));
        cost.Add(m->landmarkConstraints.Evaluate(transformedVertices, currentMesh.VertexNormals()), T(1));
        cost.Add(m->deformationModelVertex.EvaluateModelConstraints(context), T(1));
        cost.Add(m->markerConstraints.Evaluate(transformedVertices, numIterations, currentIteration), T(1));

        if (context) {
            currentIteration++;
        }

        return cost.CostToDiffData();
    };

    GaussNewtonSolver<T> solver;
    const T startEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    if (!solver.Solve(evaluationFunction, numIterations)) {
        printf("could not solve optimization problem\n");
        return;
    }
    const T finalEnergy = evaluationFunction(nullptr).Value().squaredNorm();
    printf("energy changed from %f to %f\n", startEnergy, finalEnergy);

    m->sourceAffine = m->targetAffine * m->deformationModelVertex.RigidTransformation();
    m->sourceOffsets = m->deformationModelVertex.VertexOffsets();

    m->UpdateDeformed();
}

template <class T>
Eigen::VectorX<T> IDFit<T>::ModelParameters() const
{
    return m->deformationModelRegionBlend.ModelParameters();
}

template <class T>
Eigen::Matrix<T, 3, -1> IDFit<T>::FineFittingDeltas() const
{
    return m->sourceOffsets;
}

template <class T>
int IDFit<T>::CompletedIterationsNNLS() const
{
    return m->completedIterationsNNLS;
}

template <class T>
void IDFit<T>::UpdateSourceAffine(const Eigen::Matrix<T, 4, 4>& sourceAffine)
{
    m->sourceAffine = sourceAffine;
}

template <class T>
void IDFit<T>::UpdateTargetAffine(const Eigen::Matrix<T, 4, 4>& targetAffine)
{
    m->targetAffine = targetAffine;
}

template <class T>
void IDFit<T>::UpdateSourceMarker(int markerID, const BarycentricCoordinates<T>& coordinates)
{
    m->markerConstraints.UpdateSourceMarker(markerID, coordinates);
}

template <class T>
void IDFit<T>::UpdateTargetMarker(int markerID, const Eigen::Vector3<T>& position, const Eigen::Vector3<T>& normal)
{
    m->markerConstraints.UpdateTargetMarker(markerID, position, normal);
}

template <class T>
void IDFit<T>::SetMarkers(const std::vector<BarycentricCoordinates<T>>& sourceMarkers,
                              const Eigen::Matrix<T, 3, -1>& targetMarkers,
                              const Eigen::Matrix<T, 3, -1>& targetNormals,
                              const Eigen::Vector<T, -1>& markerWeights)
{
    m->markerConstraints.SetMarkers(sourceMarkers, targetMarkers, targetNormals, markerWeights);
}

template <class T>
void IDFit<T>::RemoveMarkers()
{
    m->markerConstraints.RemoveMarkers();
}

template <class T>
void IDFit<T>::UpdateIndividualMarkerWeight(int markerID, T markerWeight)
{
    m->markerConstraints.UpdateIndividualMarkerWeight(markerID, markerWeight);
}

// explicitly instantiate the IDFit classes
template class IDFit<float>;
template class IDFit<double>;

} // namespace epic::nls
