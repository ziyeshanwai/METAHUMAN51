// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <chrono>

namespace rlibv
{
	/**
	* \defgroup Utils Utilities
	* @{
	*/

	/**
	 * @brief A class for simple profiling
	 * @tparam T T must be a std::chrono duration type
	 */
	template<typename T>
	class simple_profiler
	{
		static_assert(
			std::is_same<T, std::chrono::nanoseconds>::value ||
			std::is_same<T, std::chrono::microseconds>::value ||
			std::is_same<T, std::chrono::milliseconds>::value ||
			std::is_same<T, std::chrono::seconds>::value ||
			std::is_same<T, std::chrono::minutes>::value ||
			std::is_same<T, std::chrono::hours>::value,
			"T must be a std::chrono duration type");
	public: 
		simple_profiler();
		~simple_profiler();
		
		/**
		 * @brief Construct the profiler with a descriptive label.
		 * @param description The description
		 */
		simple_profiler(const std::string& description);

	private:
		std::string description_;
		std::chrono::high_resolution_clock::time_point start_time_;
	};


	/**@}*/
}

#include "impl/simple_profiler.hpp"