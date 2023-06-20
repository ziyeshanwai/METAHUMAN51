// Copyright Epic Games, Inc. All Rights Reserved.

#include <rigposebasedsolver/RigPoseBasedSolverTrainer.h>
#include <rigposebasedsolver/RigPoseBasedSolver.h>
#include <rigposebasedsolver/RigGenerateTrainingData.h>
#include <rigposebasedsolver/FramePoseData.h>
#include <algorithm>

namespace
{
    using P = dlib::vector<double, 3>;
    void ConvertTrainingData(const std::vector<epic::rigposebasedsolver::FramePoseData>& trainingData,
        std::vector<std::vector<std::map<std::string, P>>>& framesShapesTracking,
        std::vector<std::map<std::string, double>>& framesControls)
    {
        framesShapesTracking.resize(trainingData.size());
        framesControls.resize(trainingData.size());

        for (size_t i = 0; i < trainingData.size(); i++)
        {
            const auto& curFramePose = trainingData[i];
            framesControls[i] = curFramePose.controlValues;
            framesShapesTracking[i].resize(curFramePose.shapes.size());
            for (size_t j = 0; j < curFramePose.shapes.size(); j++)
            {
                for (const auto& shape : curFramePose.shapes[j])
                {
                    dlib::vector<double, 3> curPoint;
                    curPoint.x() = shape.second(0);
                    curPoint.y() = shape.second(1);
                    curPoint.z() = shape.second(2);
                    framesShapesTracking[i][j][shape.first] = curPoint;
                }
            }
        }
    }

    bool TrainingDataConsistent(const std::map<std::string, cm::modular_solver<double, 3>::control_range>& solverControls, 
        const std::vector<std::pair<cm::modular_solver<double, 3>::alignmentD_type, std::vector<std::vector<std::string>>>>& shapesAlignmentViewsPointsIds,
        const std::vector<epic::rigposebasedsolver::FramePoseData>& trainingData,
        std::string& errorMessage)
    {
        // check shapesAlignmentViewsPointsIds contain unique ids within each shape 
        for (const auto& shapeAlignmentViewsPointsIds : shapesAlignmentViewsPointsIds)
        {
            // there should be only one view for each shape
            if (shapeAlignmentViewsPointsIds.second.size() != 1)
            {
                errorMessage = "Expecting only a single view for the RigPoseBasedSolver";
                return false;
            }

            std::set<std::string> shapePointIdsSet(shapeAlignmentViewsPointsIds.second[0].begin(), shapeAlignmentViewsPointsIds.second[0].end());
            if (shapePointIdsSet.size() != shapeAlignmentViewsPointsIds.second[0].size())
            {
                errorMessage = "shapesAlignmentViewsPointsIds contain duplicate id(s) within a single shape";
                return false;
            }
        }

        // check that all solver controls are present in each frame, that all point ids are present in each frame, and that the same number of shapes, views and point ids
        // are present for each frame
        auto sortedShapesAlignmentViewsPointsIds = shapesAlignmentViewsPointsIds;
        for (auto& shapeAlignmentViewsPointsIds : sortedShapesAlignmentViewsPointsIds)
        {
            if (shapeAlignmentViewsPointsIds.second.size() != 1)
            {
                errorMessage = "more than one view is not supported for RigPoseBasedSolvers";
                return false;
            }

            std::sort(shapeAlignmentViewsPointsIds.second[0].begin(), shapeAlignmentViewsPointsIds.second[0].end());
        }
        unsigned frame = 0;
        for (const auto& frameData : trainingData)
        {
            for (const auto& control : solverControls)
            {
                if (frameData.controlValues.find(control.first) == frameData.controlValues.end())
                {
                    errorMessage = "control " + control.first + " is missing from training frame  " + std::to_string(frame);
                    return false;
                }
            }
            
            std::vector< std::vector< std::string > > curShapesPointIds(frameData.shapes.size());
            unsigned i = 0;
            for (const auto& shape : frameData.shapes)
            {
                for (const auto& pointId : shape)
                {
                    curShapesPointIds[i].emplace_back(pointId.first);
                }
                i++;
            }

            for (unsigned j = 0; j < sortedShapesAlignmentViewsPointsIds.size(); j++)
            {
                if (curShapesPointIds[j] != sortedShapesAlignmentViewsPointsIds[j].second[0])
                {
                    errorMessage = "one or more required point ids is missing from training frame  " + std::to_string(frame);
                    return false;
                }
            }
            frame++;
        }
        return true;
    }
}

namespace epic
{
    namespace rigposebasedsolver
    {

        void RigPoseBasedSolverTrainer::TrainSolver(const cm::modular_solver<double, 3>::training_params& solverTrainingParameters,
            const std::map<std::string, cm::modular_solver<double, 3>::control_range>& solverControls, 
            const std::vector<ControlMapper>& controlMappers,
            const std::vector<std::pair<cm::modular_solver<double, 3>::alignmentD_type, std::vector<std::vector<std::string>>>>& shapesAlignmentViewsPointsIds,
            const std::map< std::string, RigGenerateTrainingData::MeshVertexIdData> & rigMeshVertexIdData,
            const std::vector<FramePoseData>& trainingData, unsigned numShapes,
            std::vector<cm::modular_solver_trainer<double, 3>::training_delta>& trainingDeltas, RigPoseBasedSolver & solver)
        {
            std::string errString = "";
            CARBON_ASSERT(TrainingDataIsConsistent(solverTrainingParameters, solverControls, controlMappers, shapesAlignmentViewsPointsIds, rigMeshVertexIdData, trainingData, numShapes, errString), "Training data generated is not consistent with the solver specification");
            std::vector<std::vector<std::map<std::string, P>>> framesShapesTracking;
            std::vector<std::map<std::string, double>> framesControls;
            ConvertTrainingData(trainingData, framesShapesTracking, framesControls);
            solver.SetSolverData(cm::modular_solver_trainer<double, 3>::train(framesShapesTracking, framesControls, solverControls,
                shapesAlignmentViewsPointsIds, solverTrainingParameters, trainingDeltas), rigMeshVertexIdData, controlMappers);
        }

        void RigPoseBasedSolverTrainer::TrainSolverFromPreviousBestParameters(const RigPoseBasedSolver& existingSolver,
            const std::vector<FramePoseData>& trainingData, std::vector<cm::modular_solver_trainer<double, 3>::training_delta>& trainingDeltas, RigPoseBasedSolver& solver)
        {
            std::string errString = "";
            CARBON_ASSERT(TrainingDataIsConsistent(existingSolver, trainingData, errString), "Training data generated is not consistent with the solver specification");
            std::vector<std::vector<std::map<std::string, P>>> framesViewsTracking;
            std::vector<std::map<std::string, double>> framesControls;
            ConvertTrainingData(trainingData, framesViewsTracking, framesControls);
            std::vector<std::pair<typename cm::modular_solver<double, 3>::alignmentD_type, std::vector<std::vector<std::string>>>> shapesPointIds(existingSolver.ModularSolver().get_n_shapes());
            for (unsigned shape = 0; shape < existingSolver.ModularSolver().get_n_shapes(); shape++)
            {
                std::vector<std::vector<std::string>> currentShapePointIds(existingSolver.ModularSolver().get_n_views());
                for (unsigned view = 0; view < existingSolver.ModularSolver().get_n_views(); view++)
                {
                    currentShapePointIds[view] = existingSolver.ModularSolver().get_shape_view_point_ids(shape, view);
                }
                shapesPointIds[shape] = std::make_pair(existingSolver.ModularSolver().get_shape_alignment(shape), currentShapePointIds);
            }
            solver.SetSolverData(cm::modular_solver_trainer<double, 3>::train(framesViewsTracking, framesControls,
                existingSolver.ModularSolver().get_control_ranges(), shapesPointIds, existingSolver.ModularSolver().get_final_trained_params(), trainingDeltas),
                existingSolver.RigMeshVertexIdData(), existingSolver.ControlMappers());
        }

        bool RigPoseBasedSolverTrainer::TrainingDataIsConsistent(const cm::modular_solver<double, 3>::training_params& solverTrainingParameters,
            const std::map<std::string, cm::modular_solver<double, 3>::control_range>& solverControls,
            const std::vector<ControlMapper>& controlMappers,
            const std::vector<std::pair<cm::modular_solver<double, 3>::alignmentD_type, std::vector<std::vector<std::string>>>>& shapesAlignmentViewsPointsIds,
            const std::map< std::string, RigGenerateTrainingData::MeshVertexIdData>& rigMeshVertexIdData,
            const std::vector<FramePoseData>& trainingData, unsigned numShapes, std::string& errorMessage)
        {
            errorMessage = "";
            // first check the training parameters are valid
            try
            {
                cm::modular_solver_trainer<double, 3>::verify_params(solverTrainingParameters);
            }
            catch (const cm::modular_solver_trainer<double, 3>::modular_solver_error& err)
            {
                errorMessage = err.what();
                return false;
            }

            // check that control mapper to name is present in the list of solver controls
            for (const auto& controlMapper : controlMappers)
            {
                if (solverControls.find(controlMapper.toName) == solverControls.end())
                {
                    errorMessage = "Cannot find one the control for control mapper " + controlMapper.toName + " in the list of (mapped) solver controls";
                    return false;
                }
            }

            // check that the rigMeshVertexIdData is consistent with shapesAlignmentViewsPointsIds 
            for (const auto& item : rigMeshVertexIdData)
            {
                // check meshname is valid
                if (std::find(RigGenerateTrainingData::MeshNames().begin(), RigGenerateTrainingData::MeshNames().end(), item.first) == RigGenerateTrainingData::MeshNames().end())
                {
                    errorMessage = "Meshname: " + item.first + " in the rigMeshVertexIdData is invalid";
                    return false;
                }
            }

            // check vertex ids in shapesAlignmentViewsPointsIds can be found in rigMeshVertexIdData
            for (const auto& shapeAlignmentViewsPointsIds : shapesAlignmentViewsPointsIds)
            {
                if (shapeAlignmentViewsPointsIds.second.size() != 1)
                {
                    errorMessage = "Expecting only a single view for the RigPoseBasedSolver";
                    return false;
                }

                for (const auto& idStr : shapeAlignmentViewsPointsIds.second[0])
                {
                    bool found = false;
                    for (const auto& item : rigMeshVertexIdData)
                    {
                        for (auto id : item.second.vertexIdsAndShapeIndices)
                        {
                            std::string offsetIdStr = std::to_string(id.first + item.second.vertexIdOutputOffset);
                            if (offsetIdStr == idStr)
                            {
                                found = true;
                                break;
                            }
                        }
                    }

                    if (!found)
                    {
                        errorMessage = "Failed to find required vertex id:  " + idStr + " referenced in the rigMeshVertexIdData";
                        return false;
                    }
                }
            }

            // check that all shape indices are in range
            for (const auto& item : rigMeshVertexIdData)
            {
                for (auto id : item.second.vertexIdsAndShapeIndices)
                {
                    for (auto shapeIndex : id.second)
                    {
                        if (shapeIndex >= numShapes)
                        {
                            errorMessage = "Out of range shape index for vertex id:  " + std::to_string(id.first) + " referenced in the rigMeshVertexIdData for mesh: " + item.first;
                            return false;
                        }
                    }
                }
            }
  
            // next check the training data
            return TrainingDataConsistent(solverControls, shapesAlignmentViewsPointsIds, trainingData, errorMessage);
        }

        bool RigPoseBasedSolverTrainer::TrainingDataIsConsistent(const RigPoseBasedSolver& existingSolver, const std::vector<FramePoseData>& trainingData, std::string& errorMessage)
        {
            std::vector<std::pair<typename cm::modular_solver<double, 3>::alignmentD_type, std::vector<std::vector<std::string>>>> shapesPointIds(existingSolver.ModularSolver().get_n_shapes());
            for (unsigned shape = 0; shape < existingSolver.ModularSolver().get_n_shapes(); shape++)
            {
                std::vector<std::vector<std::string>> currentShapePointIds(existingSolver.ModularSolver().get_n_views());
                for (unsigned view = 0; view < existingSolver.ModularSolver().get_n_views(); view++)
                {
                    currentShapePointIds[view] = existingSolver.ModularSolver().get_shape_view_point_ids(shape, view);
                }
                shapesPointIds[shape] = std::make_pair(existingSolver.ModularSolver().get_shape_alignment(shape), currentShapePointIds);
            }

            return TrainingDataConsistent(existingSolver.ModularSolver().get_control_ranges(), shapesPointIds, trainingData, errorMessage);
        }
    }
}
