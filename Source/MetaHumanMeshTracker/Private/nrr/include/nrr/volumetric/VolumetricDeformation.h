// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Context.h>
#include <nls/Cost.h>
#include <nls/MatrixVariable.h>
#include <nls/Solver.h>
#include <nls/functions/BarycentricCoordinatesFunction.h>
#include <nls/functions/GatherFunction.h>
#include <nls/functions/SubtractFunction.h>
#include <nls/geometry/BarycentricEmbedding.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/TetMesh.h>
#include <nls/geometry/TetConstraints.h>
#include <nls/geometry/VertexLaplacian.h>
#include <nls/math/Math.h>
#include <nls/serialization/ObjFileFormat.h>
#include <nls/utils/FileIO.h>
#include <nrr/GridDeformation.h>
#include <nrr/VertexWeights.h>
#include <nrr/volumetric/VolumetricFaceModel.h>

#include <string>

namespace epic::nls {

/**
 * Morphs the volumetric model using a grid deformation model. As boundary counstraints the grid deformation uses
 * the skin mesh @p newSkinMesh and optionally also the teeth mesh @p newTeethMesh.
 * @param[in] inputVolModel  The input volumetric model that is morphed.
 * @param[in] newSkinMesh    The skin mesh that acts as target for the volumetric morph.
 * @param[in] newTeethMesh   The teeth mesh that acts as target for the volumetric morph.
 * @returns the morphed volumetric model were each mesh has been morphed, and skin and possible teeth mesh are replaced by the input meshes.
 * @warning the method does *NOT* update the tet mesh.
 */
template<class T>
VolumetricFaceModel<T> MorphVolumetricFaceModelUsingGrid(const VolumetricFaceModel<T>& inputVolModel,
                                                            const Mesh<T>& newSkinMesh,
                                                            const Mesh<T>& newTeethMesh = Mesh<T>(),
                                                            int gridSize = 32) {
    if (inputVolModel.GetSkinMesh().NumVertices() != newSkinMesh.NumVertices()) {
        CARBON_CRITICAL("skin mesh does not match skin mesh of volumetric face model");
    }

    const bool hasTeethMesh = (newTeethMesh.NumVertices() > 0);
    if (hasTeethMesh && (inputVolModel.GetTeethMesh().NumVertices() != newTeethMesh.NumVertices())) {
        CARBON_CRITICAL("teeth mesh does not match teeth mesh of volumetric face model");
    }

    std::vector<Eigen::Vector3<T> > srcVertices;
    std::vector<Eigen::Vector3<T> > targetVertices;
    for (const auto& [skinIndex, _] : inputVolModel.SkinFleshMapping()) {
        srcVertices.push_back(inputVolModel.GetSkinMesh().Vertices().col(skinIndex));
        targetVertices.push_back(newSkinMesh.Vertices().col(skinIndex));
    }
    if (newTeethMesh.NumVertices() > 0) {
        for (int k = 0; k < inputVolModel.GetTeethMesh().NumVertices(); ++k) {
            srcVertices.push_back(inputVolModel.GetTeethMesh().Vertices().col(k));
            targetVertices.push_back(newTeethMesh.Vertices().col(k));
        }
    }

    Eigen::Map<const Eigen::Matrix<T, 3, -1> > mappedSrcVertices(&srcVertices[0][0], 3, srcVertices.size());
    Eigen::Map<const Eigen::Matrix<T, 3, -1> > mappedTargetVertices(&targetVertices[0][0], 3, targetVertices.size());

    GridDeformation<T> gridDeformation(gridSize, gridSize, gridSize);
    gridDeformation.Init(inputVolModel.GetSkinMesh().Vertices());
    gridDeformation.Solve(mappedSrcVertices, mappedTargetVertices, 1.0);

    auto deformMesh = [&](const Mesh<T>& mesh) {
            Eigen::Matrix<T, 3, -1> vertices = mesh.Vertices();
            for (int i = 0; i < int(vertices.cols()); ++i) {
                vertices.col(i) += gridDeformation.EvaluateGridPosition(vertices.col(i));
            }
            Mesh<T> output = mesh;
            output.SetVertices(vertices);
            return output;
        };

    VolumetricFaceModel<T> volModel = inputVolModel;
    volModel.SetSkinMeshVertices(newSkinMesh.Vertices());
    if (hasTeethMesh) {
        volModel.SetTeethMeshVertices(newTeethMesh.Vertices());
    } else {
        volModel.SetTeethMeshVertices(deformMesh(volModel.GetTeethMesh()).Vertices());
    }
    volModel.SetFleshMeshVertices(deformMesh(volModel.GetFleshMesh()).Vertices());
    volModel.SetCraniumMeshVertices(deformMesh(volModel.GetCraniumMesh()).Vertices());
    volModel.SetMandibleMeshVertices(deformMesh(volModel.GetMandibleMesh()).Vertices());

    return volModel;
}

/**
 * Deforms the flesh mesh using the cranium, mandible, and skin as boundary conditions. This
 * method only deforms the flesh mesh and the tet mesh is not modified. Smoothness of the flesh mesh is
 * based on the @p referenceFleshMesh. The vertices of the flesh mesh that coincide with vertices
 * of cranium, mandible, and skin are directly set to the corresponding vertices of these meshes
 * to have perfect alignment.
 * This method is intended to be used for identities where there is no knowledge on the volume of the flesh.
 * To optimize the mesh of a known identity see @DeformTetsAndFleshMeshUsingCraniumMandibleAndSkinAsBoundaryConditions.
 * @param[in] inputVolModel  The input volumetric model for which the flesh mesh is motidifed.
 * @param[in] referenceFleshMesh   The reference flesh mesh that is used for smoothness constraints.
 * @returns the volumetric model with the flesh mesh being smoothly deformed.
 * @warning the method does *NOT* update the tet mesh. @see UpdateTetsUsingFleshMesh().
 */
template<class T>
VolumetricFaceModel<T> DeformFleshMeshUsingCraniumMandibleAndSkinAsBoundaryConditions(
    const VolumetricFaceModel<T>& inputVolModel,
    const Mesh<T>& referenceFleshMesh) {
    if (inputVolModel.GetFleshMesh().NumVertices() != referenceFleshMesh.NumVertices()) {
        CARBON_CRITICAL("number of vertices in flesh mesh do not match");
    }

    std::vector<int> fleshVertexIndices;
    std::vector<Eigen::Vector3<T> > targetPositions;
    for (const auto& [skinIndex, fleshIndex] : inputVolModel.SkinFleshMapping()) {
        targetPositions.push_back(inputVolModel.GetSkinMesh().Vertices().col(skinIndex));
        fleshVertexIndices.push_back(fleshIndex);
    }
    for (const auto& [craniumIndex, fleshIndex] : inputVolModel.CraniumFleshMapping()) {
        targetPositions.push_back(inputVolModel.GetCraniumMesh().Vertices().col(craniumIndex));
        fleshVertexIndices.push_back(fleshIndex);
    }
    for (const auto& [mandibleIndex, fleshIndex] : inputVolModel.MandibleFleshMapping()) {
        targetPositions.push_back(inputVolModel.GetMandibleMesh().Vertices().col(mandibleIndex));
        fleshVertexIndices.push_back(fleshIndex);
    }

    MatrixVariable<T, 3, -1> fleshOffsetVariable(3, referenceFleshMesh.NumVertices());
    fleshOffsetVariable.SetZero();
    DiffDataMatrix<T, 3, -1> fleshVertices(referenceFleshMesh.Vertices());

    DiffDataMatrix<T, 3,
                    -1> diffTargets(Eigen::Map<const Eigen::Matrix<T, 3, -1> >(&targetPositions[0][0], 3,
                                                                                targetPositions.size()));

    VertexLaplacian<T> vertexLaplacian;
    vertexLaplacian.SetRestPose(referenceFleshMesh);

    std::function<DiffData<T>(Context<T>* context)> evaluationFunction = [&](Context<T>* context) -> DiffData<T> {

            Cost<T> cost;

            DiffDataMatrix<T, 3, -1> offset = fleshOffsetVariable.EvaluateMatrix(context);
            DiffDataMatrix<T, 3, -1> deformedFleshVertices = fleshVertices + offset;

            DiffDataMatrix<T, 3, -1> interfaceVertices = GatherFunction<T>::template GatherColumns<3, -1, -1>(
                deformedFleshVertices,
                fleshVertexIndices);
            cost.Add(interfaceVertices - diffTargets, T(10));
            cost.Add(vertexLaplacian.EvaluateLaplacianOnOffsets(offset, 1.0), T(0.1));

            return cost.CostToDiffData();
        };

    GaussNewtonSolver<T> solver;
    LOG_INFO("start energy: {}", evaluationFunction(nullptr).Value().squaredNorm());
    if (!solver.Solve(evaluationFunction, 10)) {
        CARBON_CRITICAL("could not solve optimization problem");
    }
    LOG_INFO("final energy: {}", evaluationFunction(nullptr).Value().squaredNorm());

    Eigen::Matrix<T, 3, -1> deformedFleshVertices = referenceFleshMesh.Vertices() + fleshOffsetVariable.Matrix();

    VolumetricFaceModel<T> volModel = inputVolModel;
    volModel.SetFleshMeshVertices(deformedFleshVertices);
    volModel.UpdateFleshMeshVerticesFromSkinCraniumAndMandible();

    return volModel;
}


/**
 * Deform the tets based on the flesh mesh.
 * @param[inout] volModel  The volumetric face model or which the tests are updated.
 * @param[in] referenceVolModel  The reference volumetric model that defines the rest state of the tets
 */
template<class T>
void UpdateTetsUsingFleshMesh(VolumetricFaceModel<T>& volModel, const VolumetricFaceModel<T>& referenceVolModel) {
    MatrixVariable<T, 3, -1> deformedTetVerticesVariable(3, volModel.GetTetMesh().NumVertices());
    deformedTetVerticesVariable.SetMatrix(volModel.GetTetMesh().Vertices());

    std::vector<BarycentricCoordinates<T, 4> > barycentricCoordinates = volModel.Embedding().GetBarycentricCoordinates();

    TetConstraints<T> tetConstraints;
    tetConstraints.SetTopology(referenceVolModel.GetTetMesh().Tets());
    tetConstraints.SetRestPose(referenceVolModel.GetTetMesh().Vertices(), /*allowInvertedTets=*/ true);

    const T strainWeight = 1.0;
    const Eigen::Matrix<T, 3, -1>& targetVertices = volModel.GetFleshMesh().Vertices();
    DiffData<T> diffTargetVertices(targetVertices);

    std::function<DiffData<T>(Context<T>* context)> evaluationFunction = [&](Context<T>* context) -> DiffData<T> {
            Cost<T> cost;

            DiffDataMatrix<T, 3, -1> deformedTetVertices = deformedTetVerticesVariable.EvaluateMatrix(context);
            DiffDataMatrix<T, 3, -1> deformedSurfaceVertices = BarycentricCoordinatesFunction<T, 3, 4>::Evaluate(
                deformedTetVertices,
                barycentricCoordinates);

            cost.Add(deformedSurfaceVertices - diffTargetVertices, T(1));
            cost.Add(tetConstraints.EvaluateStrain(deformedTetVertices, strainWeight), T(1));

            return cost.CostToDiffData();
        };

    T minVol, avgVol, maxVol;
    volModel.GetTetMesh().TetVolumeStatistics(minVol, avgVol, maxVol, false);
    LOG_INFO("pre volume statistics: {} {} {}", minVol, avgVol, maxVol);

    GaussNewtonSolver<T> solver;
    LOG_INFO("start energy: {}", evaluationFunction(nullptr).Value().squaredNorm());
    if (!solver.Solve(evaluationFunction, 10)) {
        CARBON_CRITICAL("could not solve optimization problem\n");
    }
    LOG_INFO("final energy: {}", evaluationFunction(nullptr).Value().squaredNorm());

    volModel.SetTetMeshVertices(deformedTetVerticesVariable.Matrix());

    volModel.GetTetMesh().TetVolumeStatistics(minVol, avgVol, maxVol, false);
    LOG_INFO("pre volume statistics: {} {} {}", minVol, avgVol, maxVol);
}


/**
 * TODO: comment
 * @param[inout] volModel  The volumetric model with deformed mandible and skin (cranium should never change). The flesh and tet mesh are updated.
 * @param[in] referenceVolModel  The reference model. Typically the same "identity" as @p volModel but in Neutral position.
 */
template<class T>
void DeformTetsAndFleshMeshUsingCraniumMandibleAndSkinAsBoundaryConditions(
    VolumetricFaceModel<T>& volModel,
    const VolumetricFaceModel<T>& referenceVolModel) {

    MatrixVariable<T, 3, -1> deformedTetVerticesVariable(3, volModel.GetTetMesh().NumVertices());
    deformedTetVerticesVariable.SetMatrix(volModel.GetTetMesh().Vertices());

    std::vector<BarycentricCoordinates<T, 4> > barycentricCoordinates;
    std::vector<Eigen::Vector<T, 3>> targetVertices;
    for (const auto& [skinIndex, fleshIndex] : volModel.SkinFleshMapping()) {
        targetVertices.push_back(volModel.GetSkinMesh().Vertices().col(skinIndex));
        barycentricCoordinates.push_back(volModel.Embedding().GetBarycentricCoordinates()[fleshIndex]);
    }
    for (const auto& [craniumIndex, fleshIndex] : volModel.CraniumFleshMapping()) {
        targetVertices.push_back(volModel.GetCraniumMesh().Vertices().col(craniumIndex));
        barycentricCoordinates.push_back(volModel.Embedding().GetBarycentricCoordinates()[fleshIndex]);
    }
    for (const auto& [mandibleIndex, fleshIndex] : volModel.MandibleFleshMapping()) {
        targetVertices.push_back(volModel.GetMandibleMesh().Vertices().col(mandibleIndex));
        barycentricCoordinates.push_back(volModel.Embedding().GetBarycentricCoordinates()[fleshIndex]);
    }

    TetConstraints<T> tetConstraints;
    tetConstraints.SetTopology(referenceVolModel.GetTetMesh().Tets());
    tetConstraints.SetRestPose(referenceVolModel.GetTetMesh().Vertices());

    const T strainWeight = 1.0;
    Eigen::Matrix<T, 3, -1> targetVerticesMatrix = Eigen::Map<const Eigen::Matrix<T, 3, -1>>(&targetVertices[0][0], 3, static_cast<int>(targetVertices.size()));
    DiffDataMatrix<T, 3, -1> diffTargetVertices(targetVerticesMatrix);

    std::function<DiffData<T>(Context<T>* context)> evaluationFunction = [&](Context<T>* context) -> DiffData<T> {
        Cost<T> cost;

        DiffDataMatrix<T, 3, -1> deformedTetVertices = deformedTetVerticesVariable.EvaluateMatrix(context);
        DiffDataMatrix<T, 3, -1> deformedVertices = BarycentricCoordinatesFunction<T, 3, 4>::Evaluate(deformedTetVertices, barycentricCoordinates);

        cost.Add(deformedVertices - diffTargetVertices, T(10));
        cost.Add(tetConstraints.EvaluateStrainLinearProjective(deformedTetVertices, strainWeight, TetConstraints<T>::ElasticityModel::Linear), T(1)); //Corotated), T(1));

        return cost.CostToDiffData();
    };

    T minVol, avgVol, maxVol;
    volModel.GetTetMesh().TetVolumeStatistics(minVol, avgVol, maxVol, false);
    LOG_INFO("pre volume statistics: {} {} {}", minVol, avgVol, maxVol);

    GaussNewtonSolver<T> solver;
    LOG_INFO("start energy: {}", evaluationFunction(nullptr).Value().squaredNorm());
    if (!solver.Solve(evaluationFunction, 10)) {
        CARBON_CRITICAL("could not solve optimization problem\n");
    }
    LOG_INFO("final energy: {}", evaluationFunction(nullptr).Value().squaredNorm());

    volModel.SetTetMeshVertices(deformedTetVerticesVariable.Matrix());

    volModel.GetTetMesh().TetVolumeStatistics(minVol, avgVol, maxVol, false);
    LOG_INFO("pre volume statistics: {} {} {}", minVol, avgVol, maxVol);

    Eigen::Matrix<T, 3, -1> deformedFleshVertices = BarycentricCoordinatesFunction<T, 3, 4>::Evaluate(deformedTetVerticesVariable.EvaluateMatrix(nullptr), volModel.Embedding().GetBarycentricCoordinates()).Matrix();

    volModel.SetFleshMeshVertices(deformedFleshVertices);
    // volModel.UpdateFleshMeshVerticesFromSkinCraniumAndMandible();
}

}  // namespace epic::nls
