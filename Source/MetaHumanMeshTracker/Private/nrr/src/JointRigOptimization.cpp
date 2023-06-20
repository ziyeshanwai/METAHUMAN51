// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/JointRigOptimization.h>

#include <nls/Cost.h>
#include <nls/Solver.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/SubtractFunction.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/TriangleBending.h>
#include <nls/geometry/TriangleStrain.h>
#include <nls/rig/JointRig.h>
#include <nls/rig/JointRigVariable.h>
#include <nls/serialization/GeometrySerialization.h>
#include <nls/serialization/MeshSerialization.h>
#include <nls/serialization/JointRigSerialization.h>
#include <carbon/io/JsonIO.h>
#include <carbon/utils/Profiler.h>


namespace epic::nls {


template <class T>
struct MeshSmoothness
{
    Mesh<T> mesh;
    // evaluates strain against the rest state of the rig
    TriangleStrain<T> triangleStrainRest;
    // evaluates bending against the rest state of the rig
    TriangleBending<T> triangleBendingRest;
    // evaluates strain against the target shape
    TriangleStrain<T> triangleStrainTarget;
    // evaluate bending against the target shape
    TriangleBending<T> triangleBendingTarget;
};

template <class T>
struct JointRigOptimization<T>::Private
{
    JointRigVariable<T> jointRig;
    std::map<std::string, Eigen::Matrix<T, 3, -1>> undeformedGeometry;
    std::map<std::string, Affine<T, 3, 3>> restStates;
    std::map<std::string, std::shared_ptr<MeshSmoothness<T>>> meshSmoothness;
};


template <class T>
JointRigOptimization<T>::JointRigOptimization() : m(std::make_unique<Private>())
{
}

template <class T> JointRigOptimization<T>::~JointRigOptimization() = default;
template <class T> JointRigOptimization<T>::JointRigOptimization(JointRigOptimization&& other) = default;
template <class T> JointRigOptimization<T>& JointRigOptimization<T>::operator=(JointRigOptimization&& other) = default;

template <class T>
void JointRigOptimization<T>::SetJointRigJsonBased(const std::string& jointRigDefinitionJson)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

    const carbon::JsonElement jJointRig = carbon::ReadJson(jointRigDefinitionJson);

    // deserialize joint rig
    m->jointRig.Clear();
    JointRigFromJson(jJointRig, m->jointRig);

    // deserialize the undeformed geometry
    m->undeformedGeometry.clear();
    MultiGeometryFromJson(jJointRig, m->undeformedGeometry);

    // verify that all geometry is mapped
    const std::vector<std::string>& names = GetGeometryNames();
    for (auto&& [name,_] : m->undeformedGeometry) {
        if (std::find(names.begin(), names.end(), name) == names.end()) {
            CARBON_CRITICAL("undeformed geometry contains {}, but it is not part of the rig", name);
        }
    }
    for (const std::string& name : names) {
        if (m->undeformedGeometry.count(name) == 0) {
            CARBON_CRITICAL("rig contains {}, but it is not part of the undeformed geometry", name);
        }
    }

    // create the variables for the joint rig
    m->jointRig.CreateVariables();
    m->restStates = m->jointRig.GetState();

    LOG_INFO("num joints: {}", int(m->jointRig.GetJoints().size()));

    // verify the validity of the rig
    m->jointRig.CheckValidity();
}


template <class T>
void JointRigOptimization<T>::SetMeshJsonBased(const std::string& meshDefinitionJson)
{
    m->meshSmoothness.clear();

    const carbon::JsonElement j = carbon::ReadJson(meshDefinitionJson);
    for (const auto& [geometryName, meshDict] : j.Map()) {
        std::shared_ptr<MeshSmoothness<T>> meshSmoothnessData = std::make_shared<MeshSmoothness<T>>();
        MeshFromJson(meshDict, meshSmoothnessData->mesh);
        meshSmoothnessData->mesh.Triangulate();
        meshSmoothnessData->triangleStrainRest.SetTopology(meshSmoothnessData->mesh.Triangles());
        meshSmoothnessData->triangleBendingRest.SetTopology(meshSmoothnessData->mesh.Triangles());
        meshSmoothnessData->triangleStrainTarget.SetTopology(meshSmoothnessData->mesh.Triangles());
        meshSmoothnessData->triangleBendingTarget.SetTopology(meshSmoothnessData->mesh.Triangles());
        m->meshSmoothness[geometryName] = meshSmoothnessData;
    }

    // verify that all geometry is mapped
    const std::vector<std::string>& names = GetGeometryNames();
    for (auto&& [name,_] : m->meshSmoothness) {
        if (std::find(names.begin(), names.end(), name) == names.end()) {
            CARBON_CRITICAL("mesh data contains {}, but it is not part of the rig", name);
        }
    }
    for (const std::string& name : names) {
        if (m->meshSmoothness.count(name) == 0) {
            CARBON_CRITICAL("rig contains {}, but it is not part of the mesh data", name);
        }
    }
}


template <class T>
std::string JointRigOptimization<T>::OptimizeJointsJsonBased(const std::string& deformedGeometryJson,
                                                            const Vector<T>& splitMap,
                                                            T rotationRegularization,
                                                            T translationRegularization,
                                                            T strainWeight,
                                                            T bendingWeight,
                                                            int numIterations,
                                                            bool regularizeRoot)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

    // deserialize the undeformed geometry
    const carbon::JsonElement jDeformed = carbon::ReadJson(deformedGeometryJson);

    std::map<std::string, Eigen::Matrix<T, 3, -1>> deformedGeometry;
    MultiGeometryFromJson(jDeformed, deformedGeometry);

    // verify that all geometry is mapped
    const std::vector<std::string>& names = GetGeometryNames();
    for (auto&& [name,_] : deformedGeometry) {
        if (std::find(names.begin(), names.end(), name) == names.end()) {
            CARBON_CRITICAL("deformed geometry contains {}, but it is not part of the rig", name);
        }
    }
    for (const std::string& name : names) {
        if (deformedGeometry.count(name) == 0) {
            CARBON_CRITICAL("rig contains {}, but it is not part of the deformed geometry", name);
        }
    }

    PROFILING_BLOCK("solve setup");

    for (auto&& [name, meshSmoothnessData] : m->meshSmoothness) {
        meshSmoothnessData->triangleStrainRest.SetRestPose(m->undeformedGeometry[name]);
        meshSmoothnessData->triangleBendingRest.SetRestPose(m->undeformedGeometry[name]);
        meshSmoothnessData->triangleStrainTarget.SetRestPose(deformedGeometry[name]);
        meshSmoothnessData->triangleBendingTarget.SetRestPose(deformedGeometry[name]);
    }

    std::function<DiffData<T>(Context<T>*)> evaluationFunction = [&](Context<T>* context) {
        m->jointRig.EvaluateVariables(context);
        Cost<T> cost;
        for (auto&& [name, undeformedVerticesMatrix] : m->undeformedGeometry) {
            DiffDataMatrix<T,3,-1> undeformedVertices = undeformedVerticesMatrix;
            Eigen::Matrix<T, 3, -1> deformedVertices = deformedGeometry[name];
            DiffDataMatrix<T,3,-1> calculatedDeformedVertices = m->jointRig.EvaluateGeometry(name, undeformedVertices);
            if (splitMap.size() != calculatedDeformedVertices.Cols()) {
                CARBON_CRITICAL("split map size does not match number of vertices");
            }
            DiffData<T> residual = PointPointConstraintFunction<T, 3>::Evaluate(calculatedDeformedVertices, deformedVertices, splitMap, T(1));
            cost.Add(residual, T(1));

            auto it = m->meshSmoothness.find(name);
            if (it!= m->meshSmoothness.end()) {
                std::shared_ptr<MeshSmoothness<T>> meshSmoothnessData = it->second;
                T projectiveStrainRest = T(0);
                T greenStrainRest = T(0);
                T quadraticBendingRest = T(0);
                T dihedralBendingRest = T(0);

                T projectiveStrainTarget = T(0);
                T greenStrainTarget = strainWeight;
                T quadraticBendingTarget = T(0);
                T dihedralBendingTarget = bendingWeight;

                if (projectiveStrainRest > 0) {
                    cost.Add(meshSmoothnessData->triangleStrainRest.EvaluateProjectiveStrain(calculatedDeformedVertices, projectiveStrainRest), T(1));
                }
                if (greenStrainRest > 0) {
                    cost.Add(meshSmoothnessData->triangleStrainRest.EvaluateGreenStrain(calculatedDeformedVertices, greenStrainRest), T(1));
                }
                if (quadraticBendingRest > 0) {
                    cost.Add(meshSmoothnessData->triangleBendingRest.EvaluateQuadratic(calculatedDeformedVertices, quadraticBendingRest), T(1));
                }
                if (dihedralBendingRest > 0) {
                    cost.Add(meshSmoothnessData->triangleBendingRest.EvaluateDihedral(calculatedDeformedVertices, dihedralBendingRest), T(1));
                }

                if (projectiveStrainTarget > 0) {
                    cost.Add(meshSmoothnessData->triangleStrainTarget.EvaluateProjectiveStrain(calculatedDeformedVertices, projectiveStrainTarget), T(1));
                }
                if (greenStrainTarget > 0) {
                    cost.Add(meshSmoothnessData->triangleStrainTarget.EvaluateGreenStrain(calculatedDeformedVertices, greenStrainTarget), T(1));
                }
                if (quadraticBendingTarget > 0) {
                    cost.Add(meshSmoothnessData->triangleBendingTarget.EvaluateQuadratic(calculatedDeformedVertices, quadraticBendingTarget), T(1));
                }
                if (dihedralBendingTarget > 0) {
                    cost.Add(meshSmoothnessData->triangleBendingTarget.EvaluateDihedral(calculatedDeformedVertices, dihedralBendingTarget), T(1));
                }
            }
        }
        if (rotationRegularization > 0 || translationRegularization > 0) {
            DiffData<T> reg = m->jointRig.EvaluateRegularization(rotationRegularization, translationRegularization, m->restStates, regularizeRoot);
            cost.Add(reg, T(1));
        }
        return cost.CostToDiffData();

    };
    PROFILING_END_BLOCK;

    PROFILING_BLOCK("solve");
    GaussNewtonSolver<T> solver;
    const T startEnergy = evaluationFunction(nullptr).Value().norm();
    if (!solver.Solve(evaluationFunction, numIterations)) {
        printf("could not solve optimization problem\n");
        return "";
    }
    const T finalEnergy = evaluationFunction(nullptr).Value().norm();
    printf("energy reduced from %f to %f\n", double(startEnergy), double(finalEnergy));
    PROFILING_END_BLOCK;

    return RigStateJsonBased();
}


template <class T>
std::string JointRigOptimization<T>::RigStateJsonBased()
{
    return carbon::WriteJson(JointRigStateToJson(m->jointRig));
}


template <class T>
void JointRigOptimization<T>::SetRigStateJsonBased(const std::string& rigStateJson)
{
    JointRigStateFromJson(carbon::ReadJson(rigStateJson), m->jointRig);
}


template <class T>
void JointRigOptimization<T>::RestoreRigState()
{
    m->jointRig.SetState(m->restStates);
}


template <class T>
void JointRigOptimization<T>::SetJointDegreesOfFreedom(const std::string& jointName, bool rotation, bool translation)
{
    m->jointRig.SetDegreesOfFreedom(jointName, rotation, translation);
}


template <class T>
std::vector<std::string> JointRigOptimization<T>::GetJointNames() const
{
    return m->jointRig.GetJointNames();
}


template <class T>
std::vector<std::string> JointRigOptimization<T>::GetGeometryNames() const
{
    return m->jointRig.GetGeometryNames();
}

// explicitly instantiate the joint rig optimization classes
template class JointRigOptimization<float>;
template class JointRigOptimization<double>;

} // namespace epic::nls
