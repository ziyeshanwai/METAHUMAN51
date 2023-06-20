// Copyright Epic Games, Inc. All Rights Reserved.

#include "../include/rlibv/keypoint.h"

namespace rlibv
{
	void serialize(const keypoint& item, std::ostream& out)
	{
		serialize(item.style, out);
		serialize(item.pos, out);
	}

	void deserialize(keypoint& item, std::istream& in)
	{
		deserialize(item.style, in);
		deserialize(item.pos, in);
	}
}
	



