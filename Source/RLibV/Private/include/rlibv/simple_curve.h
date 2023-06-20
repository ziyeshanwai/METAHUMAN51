// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

namespace rlibv
{
	/**
	* Represents a curve by a set of indices and flag indicating whether it's closed or not.
	*/
	struct simple_curve
	{
		std::vector<int> indices;
		bool closed = false;
	};

	/**
	 * Serialize a simple curve.
	 *
	 * @param item
	 * @param out
	 */
	inline void serialize(const simple_curve& item, std::ostream& out)
	{
		serialize(item.indices, out);
		serialize(item.closed, out);
	}

	/**
	 * Deserialize a simple curve.
	 *
	 * @param item
	 * @param in
	 */
	inline void deserialize(simple_curve& item, std::istream& in)
	{
		deserialize(item.indices, in);
		deserialize(item.closed, in);
	}
}
