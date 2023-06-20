// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "anchored_curve.h"

namespace rlibv
{
	using dlib::serialize;
	using dlib::deserialize;

	/**
	 * @brief Represents an anchor connection point between two named curves. The corresponding anchor index on each
	 *        curve are marked as connected.
	 */
	struct curve_anchor_connection
	{
		std::pair<std::string, int> a;
		std::pair<std::string, int> b;
	};

	/**
	 * @brief Comparison operator
	 * @param lhs
	 * @param rhs
	 * @return True if connections are functionally equivalent.
	 * @note Connections are directionless so a connection with a and b interchanged is considered to
	 *       be exactly the same connection.
	 */
	inline bool operator==(const curve_anchor_connection& lhs, const curve_anchor_connection& rhs)
	{
		return (lhs.a == rhs.a && lhs.b == rhs.b) || (lhs.b == rhs.a && lhs.a == rhs.b);
	}

	/**
	 * @brief Not Equal comparison operator
	 * @param lhs
	 * @param rhs
	 * @return True if connections are NOT functionally equivalent.
	 * @note Connections are directionless so a connection with a and b interchanged is considered to
	 *       be exactly the same connection.
	 */
	inline bool operator!=(const curve_anchor_connection& lhs, const curve_anchor_connection& rhs)
	{
		return !((lhs.a == rhs.a && lhs.b == rhs.b) || (lhs.b == rhs.a && lhs.a == rhs.b));
	}

	/**
	 * @brief Supports serialization of curve_connection structures.
	 * @param item
	 * @param out
	 */
	inline void serialize(const curve_anchor_connection& item, std::ostream& out)
	{
		serialize(item.a, out);
		serialize(item.b, out);
	}

	/**
	 * @brief Supports deserialization of curve_connection structures.
	 * @param item
	 * @param in
	 */
	inline void deserialize(curve_anchor_connection& item, std::istream& in)
	{
		deserialize(item.a, in);
		deserialize(item.b, in);
	}
}
