// Copyright Epic Games, Inc. All Rights Reserved.

#include "../include/rlibv/keypoint_curve.h"

namespace rlibv
{
	void serialize(const keypoint_curve& item, std::ostream& out)
	{
		serialize(item.start_keypoint_name, out);
		serialize(item.end_keypoint_name, out);
		serialize(item.internal_points, out);
		serialize(item.color, out);
	}

	void deserialize(keypoint_curve& item, std::istream& in)
	{
		deserialize(item.start_keypoint_name, in);
		deserialize(item.end_keypoint_name, in);
		deserialize(item.internal_points, in);
		deserialize(item.color, in);
	}
}


