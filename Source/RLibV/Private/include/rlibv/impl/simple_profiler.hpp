// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <chrono>

namespace rlibv
{
	template<typename T>
	simple_profiler<T>::simple_profiler(const std::string& description)
	{
		description_ = description;
		start_time_ = std::chrono::high_resolution_clock::now();
	}

	template<typename T>
	simple_profiler<T>::simple_profiler()
	{
		description_ = "anonymous";
		start_time_ = std::chrono::high_resolution_clock::now();
	}

	template<typename T>
	simple_profiler<T>::~simple_profiler()
	{
		std::string units;
		if (std::is_same<T, std::chrono::microseconds>::value)
		{
			units = std::string(u8"\u03BC") + "s";
		}
		else if (std::is_same<T, std::chrono::milliseconds>::value)
		{
			units = "ms";
		}
		else if (std::is_same<T, std::chrono::nanoseconds>::value)
		{
			units = "ns";
		}
		else if (std::is_same<T, std::chrono::seconds>::value)
		{
			units = "s";
		}
		else if (std::is_same<T, std::chrono::minutes>::value)
		{
			units = "m";
		}
		else if (std::is_same<T, std::chrono::hours>::value)
		{
			units = "h";
		}

		auto end_time_ = std::chrono::high_resolution_clock::now();
		std::cout << "Operation " << description_ << " took " << std::chrono::duration_cast<T>(end_time_ - start_time_).count() << " " << units << std::endl;
	}
}

