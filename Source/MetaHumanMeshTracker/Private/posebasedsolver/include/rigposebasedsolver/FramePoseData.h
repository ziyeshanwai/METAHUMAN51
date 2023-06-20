// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include <carbon/common/EigenDenseBackwardsCompatible.h>
#include <map>
#include <string>
#include <vector>

namespace epic
{
    namespace rigposebasedsolver
    {
        /**
         * A simple struct containing the training data associated with a frame of training data for the pose-based solver
        */
        struct FramePoseData
        {
            /*
            * default constructor
            */
            FramePoseData()
            {};

            // a map of rig UI control name to corresponding control values
            std::map< std::string, double> controlValues;

            // a vector with an element for each solver 'shape' (or locally aligned set of vertices).
            // for each shape, there is a map of (string converted) vertex id (which will have an offset applied for each rig mesh as specified in the solver
            // config) to corresponding vertex position; note that vertices from all rig meshes are combined into a single map, using the
            // vertex id offset to differentiate the data for different meshes
            std::vector< std::map< std::string, Eigen::Vector<float, 3>>> shapes;

            /*
                friend function for serialization
            */
            friend void serialize(const FramePoseData& item, std::ostream& out);

            /*
                friend function for deserialization
            */
            friend void deserialize(FramePoseData& item, std::istream& in);

            static const unsigned version;
        };
    }
}

