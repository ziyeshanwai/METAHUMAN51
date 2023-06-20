// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once
#include <rigposebasedsolver/RigPoseBasedSolver.h>
#include <rigposebasedsolver/RigGenerateTrainingData.h>
#include <posebasedsolver/modular_solver.h>
#include <carbon/common/EigenDenseBackwardsCompatible.h>
#include <vector>
#include <string>

namespace epic
{
    namespace rigposebasedsolver
    {
        /**
         * A simple struct containing a pose-based solver and all the parameters and training data needed to retrain it from a rig
        */
        struct RigPoseBasedSolverData
        {
            /*
            * default constructor
            */
            RigPoseBasedSolverData()
            {};

            RigPoseBasedSolver existingSolver;
            RigGenerateTrainingData::PoseBasedSolverTrainingDataParameters poseBasedSolverTrainingDataParameters;
            cm::modular_solver<double, 3>::training_params poseBasedSolverTrainingParameters;
            std::vector< std::map<std::string, std::map<int, double>> > animationData;
            std::vector<std::pair<std::string, std::vector<unsigned>>> animationFilenamesAndFrameNumbers;
            std::map<std::string, cm::modular_solver<double, 3>::control_range> solverControls;

            /*
                friend function for serialization
            */
            friend void serialize(const RigPoseBasedSolverData& item, std::ostream& out);

            /*
                friend function for deserialization
            */
            friend void deserialize(RigPoseBasedSolverData& item, std::istream& in);
        };
    }
}

