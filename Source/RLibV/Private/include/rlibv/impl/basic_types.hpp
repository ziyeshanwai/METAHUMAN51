// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "enum_ext.h"

#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include <dlib/matrix.h>
RLIBV_RENABLE_WARNINGS


namespace rlibv
{
	using dlib::serialize;
	using dlib::deserialize;


	template<typename U>
	void serialize(const multiview_shape2d<U>& item, std::ostream& out)
	{
		serialize(item.shapes, out);
		serialize(item.visible, out);
	}

	template<typename U>
	void deserialize(multiview_shape2d<U>& item, std::istream& in)
	{
		deserialize(item.shapes, in);
		deserialize(item.visible, in);	
	}

	template<typename U>
	void deserialize_old_mvshapes2d(std::vector<multiview_shape2d<U>>& item, std::istream& in)
	{
		unsigned long size;
		deserialize(size, in);
		item.resize(static_cast<size_t>(size));
		for (unsigned long i = 0; i < size; ++i)
		{
			deserialize(item[i].shapes, in);
		}

		for (auto& subitem : item)
		{
			subitem.visible.resize(subitem.shapes.size());
			for (int i = 0; i < static_cast<int>(subitem.shapes.size()); i++)
			{
				subitem.visible[i] = std::vector<bool>(true, subitem.shapes[i].size());
			}
		}

	}
}
