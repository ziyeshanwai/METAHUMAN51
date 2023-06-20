// Copyright Epic Games, Inc. All Rights Reserved.

#include <rigposebasedsolver/ControlMapper.h>
#include <posebasedsolver/serialization_utils.h>


namespace epic
{
    namespace rigposebasedsolver
    {

		void serialize(const ControlMapper::ControlMapRange& item, std::ostream& out)
		{
			dlib::serialize(item.version, out);
			dlib::serialize(item.controlName, out);
			dlib::serialize(item.inputVal1Val2, out);
			dlib::serialize(item.outputVal1Val2, out);
		}

		void deserialize(ControlMapper::ControlMapRange& item, std::istream& in)
		{
			// current version deserialization
			// IMPORTANT: please try and make class serialization / deserialization back-compatible
			// by supporting multiple versions if possible, and if not, function should throw an exception
			unsigned curVersion;
			dlib::deserialize(curVersion, in);
			if (curVersion == 1u)
			{
				dlib::deserialize(item.controlName, in);
				dlib::deserialize(item.inputVal1Val2, in);
				dlib::deserialize(item.outputVal1Val2, in);
			}
			else
			{
				throw cm::serialization_error("ControlMapper::ControlMapRange is not compatible with version : " + std::to_string(curVersion));
			}
		}

		const unsigned ControlMapper::ControlMapRange::version = 1;

		void serialize(const ControlMapper& item, std::ostream& out)
		{
			dlib::serialize(item.version, out);
			serialize(item.fromControlMapRange1, out);
			serialize(item.fromControlMapRange2, out);
			dlib::serialize(item.toName, out);
			dlib::serialize(item.toDefault, out);
		}

		void deserialize(ControlMapper& item, std::istream& in)
		{
			// current version deserialization
			// IMPORTANT: please try and make class serialization / deserialization back-compatible
			// by supporting multiple versions if possible, and if not, function should throw an exception
			unsigned curVersion;
			dlib::deserialize(curVersion, in);
			if (curVersion == 1u)
			{
				deserialize(item.fromControlMapRange1, in);
				deserialize(item.fromControlMapRange2, in);
				dlib::deserialize(item.toName, in);
				dlib::deserialize(item.toDefault, in);
			}
			else
			{
				throw cm::serialization_error("ControlMapper is not compatible with version : " + std::to_string(curVersion));
			}
		}

		const unsigned ControlMapper::version = 1;


    }
}
