// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include <rigposebasedsolver/RigGenerateTrainingData.h>
#include <posebasedsolver/modular_solver_trainer.h>
#include <map>
#include <vector>
#include <string>

namespace epic
{
    namespace rigposebasedsolver
    {

		struct FramePoseData;
		class RigPoseBasedSolver;

		/**
		 * RigPoseBasedSolverTrainer is essentially a class / set of functions which are used to train a pose-based solver (modular_solver)
		 * which works with data generated from a 3D face rig.
		 */
		class RigPoseBasedSolverTrainer
		{
		public:
			/*
			* Train a RigPoseBasedSolver from scratch from a set of training data and various solver configuration/parameters, calculating the solver itself
			* and training diagnostics. During the solver training process, feature selection and meta-parameter
			* optimization is performed using a cross-validation metric to choose the best values.
			* Requires TrainingDataIsConsistent(solverTrainingParameters, solverControls, viewsPointIds, rigMeshVertexIdData, trainingData) == true
			* @param[in] solverTrainingParameters: a struct containing the parameters to be tried during solver training.
			* @param[in] solverControls: a map of control name to control range for each control in the solver.
			* @param[in] controlMappers: a vector of mapper objects which map from ui controls to raw controls and back.
			* @param[in] shapesAlignmentViewsPointsIds: for each shape used in the solver, a pair defining the alignment used for that shape and a vector of point id string for each view 
			* for 3D there will be a single view.
			* @param[in] rigMeshOutputVertexIdData: a map of rig mesh name to MeshVertexIdData (defining vertex indices of the rig to use) for each rig mesh to use
			* @param[in] trainingData: a vector of pairs of (input) control data and (output) rig vertices for each training example
			* @param[out] trainingDeltas: a vector of the fractional error (as a fraction of control range) for each control in the training set, measured by applying
			* the trained solver to the training set input data.
			* @param[in] numShapes: the number of shapes in the solver
			* @params[out] solver: the trained solver.
			*/
			static void TrainSolver(const cm::modular_solver<double, 3>::training_params& solverTrainingParameters,
				const std::map<std::string, cm::modular_solver<double, 3>::control_range>& solverControls,
				const std::vector<ControlMapper>& controlMappers,
				const std::vector<std::pair<cm::modular_solver<double, 3>::alignmentD_type, std::vector<std::vector<std::string>>>>& shapesAlignmentViewsPointsIds,
				const std::map< std::string, RigGenerateTrainingData::MeshVertexIdData>& rigMeshVertexIdData,
				const std::vector<FramePoseData>& trainingData, unsigned numShapes,
				std::vector<cm::modular_solver_trainer<double, 3>::training_delta>& trainingDeltas, RigPoseBasedSolver& solver);

			/*
			* Train a RigPoseBasedSolver from a set of training data using the best features / meta-parameters from the supplied existing solver, calculating the solver itself
			* and training diagnostics.  During the solver training process, feature selection and meta-parameter
			* optimization is performed using a cross-validation metric to choose the best values.
			* @param[in] existingSolver: an existing pre-trained solver
			* @param[in] trainingData: the training data for the solver
			* @param[out] trainingDeltas: a vector of the fractional error (as a fraction of control range) for each control in the training set, measured by applying
			* @params[out] solver: the trained solver.
			*/
			static void TrainSolverFromPreviousBestParameters(const RigPoseBasedSolver& existingSolver,
				const std::vector<FramePoseData>& trainingData, std::vector<cm::modular_solver_trainer<double, 3>::training_delta>& trainingDeltas, RigPoseBasedSolver& solver);

			/*
			* @returns true if the supplied solver training data is consistent with the supplied solver training parameters, controls and point ids, rig mesh vertex id data, false otherwise.
			* @param[in] solverTrainingParameters: a struct containing the parameters to be tried during solver training.
			* @param[in] solverControls: a map of control name to control range for each control in the solver.
			* @param[in] controlMappers: a vector of mapper objects which map from ui controls to raw controls and back.
			* @param[in] shapesAlignmentViewsPointsIds: for each shape used in the solver, a pair defining the alignment used for that shape and a vector of point id string for each view
			* for 3D there will be a single view.
			* @param[in] rigMeshOutputVertexIdData: a map of rig mesh name to MeshVertexIdData (defining vertex indices of the rig to use) for each rig mesh to use
			* @param[in] trainingData: a vector of pairs of (input) control data and (output) rig vertices for each training example
			* @param[in] numShapes: the number of shapes in the solver
			* @param[out] errorMessage: on return contains an error message describing what was wrong
			*/
			static bool TrainingDataIsConsistent(const cm::modular_solver<double, 3>::training_params& solverTrainingParameters,
				const std::map<std::string, cm::modular_solver<double, 3>::control_range>& solverControls,
				const std::vector<ControlMapper>& controlMappers,
				const std::vector<std::pair<cm::modular_solver<double, 3>::alignmentD_type, std::vector<std::vector<std::string>>>>& shapesAlignmentViewsPointsIds,
				const std::map< std::string, RigGenerateTrainingData::MeshVertexIdData>& rigMeshVertexIdData,
				const std::vector<FramePoseData>& trainingData, unsigned numShapes, std::string& errorMessage);

			/*
			* @returns true if the supplied solver training data is consistent with the supplied existing solver, false otherwise.
			* @param[in] existingSolver: an existing pre-trained solver
			* @param[in] trainingData: the training data for the solver
			* @param[out] errorMessage: on return contains an error message describing what was wrong
			*/
			static bool TrainingDataIsConsistent(const RigPoseBasedSolver& existingSolver, const std::vector<FramePoseData>& trainingData, std::string& errorMessage);
		};
    }
}

