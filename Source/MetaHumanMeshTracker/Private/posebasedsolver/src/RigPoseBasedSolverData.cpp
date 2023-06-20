// Copyright Epic Games, Inc. All Rights Reserved.

#include <rigposebasedsolver/RigPoseBasedSolverData.h>
#include <posebasedsolver/serialization_utils.h>

namespace epic
{
    namespace rigposebasedsolver
    {
		void serialize(const RigPoseBasedSolverData& item, std::ostream& out)
		{
			// Note that we are not versioning this struct, as the data
			// saved can also be used in an old tool by just deserializing the first item
			// and adding an extra version field would prevent that
			serialize(item.existingSolver, out);
			serialize(item.poseBasedSolverTrainingDataParameters, out);
			serialize(item.poseBasedSolverTrainingParameters, out);
			dlib::serialize(item.animationData, out);
			dlib::serialize(item.animationFilenamesAndFrameNumbers, out);
			dlib::serialize(item.solverControls, out);
		}

		void deserialize(RigPoseBasedSolverData& item, std::istream& in)
		{
			deserialize(item.existingSolver, in);
			deserialize(item.poseBasedSolverTrainingDataParameters, in);
			deserialize(item.poseBasedSolverTrainingParameters, in);
			dlib::deserialize(item.animationData, in);
			dlib::deserialize(item.animationFilenamesAndFrameNumbers, in);
			dlib::deserialize(item.solverControls, in);
		}

    }
}
