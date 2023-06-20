// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "anchored_curve.h"
#include "curve_anchor_connection.h"

namespace rlibv
{
	using dlib::serialize;
	using dlib::deserialize;

	template<typename U>
	inline void serialize(const image_annotation<U>& item, std::ostream& out)
	{
		serialize(item.curves, out);
		serialize(item.anchor_connections, out);
	}

	template<typename U>
	inline void deserialize(image_annotation<U>& item, std::istream& in)
	{
		deserialize(item.curves, in);
		deserialize(item.anchor_connections, in);
	}

}
