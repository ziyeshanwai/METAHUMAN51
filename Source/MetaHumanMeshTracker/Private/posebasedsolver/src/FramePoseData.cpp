// Copyright Epic Games, Inc. All Rights Reserved.

#include <rigposebasedsolver/FramePoseData.h>

namespace dlib
{
	// helper functions for serializing and deserializing Eigen::Vector<float, 3>
	// declaration needs to be before including serialization_utils.h so that the
	// template functions there can use these Eigen serialization functions.
	void serialize(const Eigen::Vector<float, 3>& vec3, std::ostream& out);
	void deserialize(Eigen::Vector<float, 3>& vec3, std::istream& in);
}

#include <posebasedsolver/serialization_utils.h>

namespace dlib
{
	// helper functions for serializing and deserializing Eigen::Vector<float, 3>
	void serialize(const Eigen::Vector<float, 3>& vec3, std::ostream& out)
	{
		serialize(vec3.x(), out);
		serialize(vec3.y(), out);
		serialize(vec3.z(), out);
	}
	void deserialize(Eigen::Vector<float, 3>& vec3, std::istream& in)
	{
		deserialize(vec3.x(), in);
		deserialize(vec3.y(), in);
		deserialize(vec3.z(), in);
	}
}

namespace epic
{
    namespace rigposebasedsolver
    {
		void serialize(const FramePoseData& item, std::ostream& out)
		{
			dlib::serialize(item.version, out);
			dlib::serialize(item.controlValues, out);
			dlib::serialize(item.shapes, out);
		}

		void deserialize(FramePoseData& item, std::istream& in)
		{
			// current version deserialization
			// IMPORTANT: please try and make class serialization / deserialization back-compatible
			// by supporting multiple versions if possible, and if not, function should throw an exception
			unsigned curVersion;
			dlib::deserialize(curVersion, in);
			if (curVersion == 1u)
			{
				using dlib::deserialize;
				dlib::deserialize(item.controlValues, in);
				dlib::deserialize(item.shapes, in);
			}
			else
			{
				throw cm::serialization_error("FramePoseData is not compatible with version : " + std::to_string(curVersion));
			}
		}

		const unsigned FramePoseData::version = 1;

    }
}
