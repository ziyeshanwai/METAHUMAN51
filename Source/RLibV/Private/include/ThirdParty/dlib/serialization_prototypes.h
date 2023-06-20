// Copyright Epic Games, Inc. All Rights Reserved.

// serialize for std::array is not prototyped in dlib/serialize.h, prototype it here to avoid error with /permissive-

#include <array>
#include <ostream>

namespace dlib
{
	template <
		typename T,
		size_t N
	>
		void serialize(
			const std::array<T, N>& array,
			std::ostream& out
		);

	template <
		typename T,
		size_t N
	>
		inline void deserialize(
			std::array<T, N>& array,
			std::istream& in
		);
}
