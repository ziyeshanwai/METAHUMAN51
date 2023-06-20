// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <stdexcept>

namespace rlibv
{
	/**
    * \defgroup Errors Errors
    * @{
    */

	/**
	 * @brief A struct representing an error in randomized SVD
	 * @tparam T T must be either float or double
	 */
	struct randomized_svd_error : public std::runtime_error
	{
		/**
		 * @brief Error constructor
		 * @param message User-defined error message
		 */
		randomized_svd_error(const std::string& message)
				:std::runtime_error(message)
		{
		}
	};
	/**@}*/
}
