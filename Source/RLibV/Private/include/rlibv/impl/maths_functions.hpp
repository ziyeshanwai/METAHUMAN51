// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

namespace rlibv
{

	template<typename T>
	T wrap_angle(T angle)
	{
		angle = fmod(angle, static_cast<T>(dlib::pi * 2.0f));
		if (angle < 0.0f)
		{
			angle += static_cast<T>(dlib::pi * 2.0f);
		}
		return angle;
	}

	template<typename T>
	T legendre(unsigned int n, T x)
	{
		static_assert(dlib::is_float_type<T>::value, "x must be a float or double type");

		if (n == 0U)
		{
			return 1;
		}
		if (n == 1U)
		{
			return x;
		}
		return (static_cast<T>(2U * n - 1U) * x * legendre(n - 1U, x) - static_cast<T>(n - 1U) * legendre(n - 2U, x)) / static_cast<T>(n);
	}

	template<typename T>
	T halton(int index, int base)
	{
		T value = static_cast<T>(0);
		T reciprocal = static_cast<T>(1) / base;
		T f = reciprocal;
		while (index > 0)
		{
			value += (index % base) * f;
			index /= base;
			f *= reciprocal;
		}
		return value;
	}

}