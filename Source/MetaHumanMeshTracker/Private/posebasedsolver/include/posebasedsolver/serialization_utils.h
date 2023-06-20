// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/serialize.h>
#include <dlib/image_transforms/interpolation.h>
POSEBASEDSOLVER_RENABLE_WARNINGS

namespace cm
{
    using namespace dlib;

	/*!
	    Exception for errors when deserialising chunks of data.
        The exception stores the expected and actual versions in addition to an error message
	*/
	struct serialization_version_error : public serialization_error
	{
		explicit serialization_version_error(const std::string  &object_name, const int code_version, const int data_version) :
			serialization_error("Could not deserialise object of type " + object_name + ". Expected Version = " + std::to_string(code_version) + ", Deserializing Version = " + std::to_string(data_version)),
			object_name(object_name),
			code_version(code_version),
			data_version(data_version)
		{
		}
		std::string object_name;
		int code_version = 0;
		int data_version = 0;
	};

	/*!
		Compares two version numbers and throws a serialization_version_error if they are not the same.
	*/
	inline static void check_serialization_versioning(const std::string & object_name, const int code_version, const int data_version)
	{
		if (code_version != data_version)
		{
			throw serialization_version_error(object_name, code_version, data_version);
		}
	}
}
