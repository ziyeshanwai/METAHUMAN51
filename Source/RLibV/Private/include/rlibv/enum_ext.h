// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <string>
#include <sstream>
#include <iostream>

#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/error.h>
#include <dlib/string.h>
#include <dlib/matrix.h>
RLIBV_RENABLE_WARNINGS

namespace rlibv
{
	/**
	* @brief Trims a string
	* @details Internal function used in the macro below
	* @param s String
	*/
	inline std::string trim_enum_string(const std::string& s)
	{
		std::string::const_iterator it = s.begin();
		while (it != s.end() && isspace(static_cast<unsigned char>(*it)))
		{
			++it;
		}
		std::string::const_reverse_iterator rit = s.rbegin();
		while (rit.base() != it && isspace(static_cast<unsigned char>(*rit)))
		{
			++rit;
		}

		return std::string(it, rit.base());
	}

	/**
	* @brief Splits a string into parts based on a comma.
	* @details Internal function used in the macro below
	* @param args
	* @param array String
	* @param max_elements
	*/
	inline void split_enum_args(const char* args, std::string array[], int max_elements)
	{

		std::stringstream ss(args);
		std::string sub_str;
		int nIdx = 0;
		while (ss.good() && (nIdx < max_elements))
		{
			getline(ss, sub_str, ',');

			auto parts = dlib::split(sub_str, "=");

			if (parts.size() != 1)
			{
				nIdx = dlib::string_cast<int>(parts[1]);
			}

			array[nIdx] = trim_enum_string(parts[0]);
			nIdx++;
		}
	}
	
	/*
	 *	The following is a MACRO that declares the following functions in addition to the enum declaration itself
	 *
	 *	std::string dlib::to_string(T enum)
	 *	Converts the enumeration to a string
	 *
	 *	std::string dlib::enum_type_as_string(T enum)
	 *	Given an example of an enumeration value a string of the enumeration type is returned
	 *
	 *	int dlib::enum_count(T enum)
	 *	returns the number of elements in a enumeration
	 *
	 *	serialize(T e, std::ostream&)
	 *	writes the enumeration value to the stream
	 *
	 *	deserialize(T e ,std::istream &)
	 *	reads the enum valie from stream
	 *
	 *	throw_enum_error(T e, std::string msg)
	 *	utility function for throwing exceptions adding enum information to the message
	 *
	 *	For usage examples see the unit tests in tests_enums_ext.cpp
	 *
	 */
#define DECLARE_ENUM(ename, ...)															\
	enum class ename { __VA_ARGS__, MAX_NUMBER_OF_##ename };								\
	inline std::string to_string(ename e)													\
	{																						\
		const auto MAX_NUMBER_OF_##ename = static_cast<int>(ename::MAX_NUMBER_OF_##ename);	\
		static std::string ename##Strings[MAX_NUMBER_OF_##ename];							\
		if (ename##Strings[0].empty())														\
		{																					\
			split_enum_args(#__VA_ARGS__, ename##Strings, MAX_NUMBER_OF_##ename);			\
		}																					\
		auto asInt = static_cast<int>(e);													\
		return ename##Strings[asInt];														\
	}																						\
	inline std::string enum_type_as_string(ename /*e*/)											\
	{																						\
		return	#ename;																	    \
	}																						\
	inline int enum_count(ename /*e*/)															\
	{																						\
		const auto count = static_cast<int>(ename::MAX_NUMBER_OF_##ename);					\
		return count;																		\
	}																						\
	inline void serialize(ename item, std::ostream& out)									\
	{																						\
		const auto val = static_cast<int>(item);											\
		dlib::serialize(val, out);															\
	}																						\
	inline void deserialize(ename& item, std::istream& in)									\
	{																						\
		auto val(0);																		\
		dlib::deserialize(val, in);															\
		item = static_cast<ename>(val);														\
	}																						\
	inline void throw_enum_error(ename item, const std::string & msg)						\
	{																						\
		std::string full_msg = msg + " (" + enum_type_as_string(item) + ")";				\
		throw enum_error(full_msg);																\
	}

	 /**
	 * @brief Exception converting to/from enums
	 * @param message
	 */
	struct enum_error : public dlib::error
	{
		explicit enum_error(const std::string& message) :
			dlib::error(message) {}
	};

	/**
	* @brief Converts a string to an enum of the specified type
	* @tparam T Must be either float or double
	* @param as_string String
	* @returns Enum value corresponding to the given string.
	* @throws Enum_error exception if the string wasn't convertible to a enum type.
	*/
	template<typename T>
	T to_enum(const std::string& as_string)
	{
		static_assert(std::is_enum<T>::value, "class must be an enumerated type");

		for (auto i = 0; i < enum_count(T()); ++i)
		{
			auto result = static_cast<T>(i);
			auto cur_enum_as_str = to_string(result);
			if (as_string == cur_enum_as_str)
			{
				return result;
			}
		}

		throw_enum_error(T{}, std::string("Could not convert " + as_string + " to enumeration."));
		return T{};

	}
}