// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <string>
#include <vector>

namespace rlibv
{

	/**
	 * @brief Collects sorted numbered filenames of image sequences from a vector of filenames and the actual numbers
	 * @details This function gathers the best candidate sequence of files that look like they
	 *			correspond to an image sequence. It seeks the largest set of files which have:
	 *				a root name in common
	 *				are numbered in some way e.g.  frame_0, frame_1, or frame_001, frame_002, etc.
	*				an extension in common
	 * @param candidate_filenames The vector of possible filenames, typically a list of files in a directory
	 * @return A new vector containing pairs of filenames and numbers of frames in ascending numerical order
	 */
	std::vector<std::pair<std::string,int>> collect_image_sequence(const std::vector<std::string>& candidate_filenames);

	/**
	 * @brief Replaces all occurrence of a string with another string
	 * @param subject The original string
	 * @param search The string to replace
	 * @param replace The string with which to replace the found instances
	 * @return A new string with the replacements made
	 */
    std::string replace_string(const std::string& subject, const std::string& search, const std::string& replace);

	/**
	 * @brief Check if a string ends with another string
	 * @param str The original string
	 * @param cmp The substring to check
	 * @return True if string ends with the other string
	 */
	bool string_ends_with(const std::string& str, const std::string& cmp);

	/**
	 * @brief Splits a string into a vector of strings according to a delimiter string.
	 * @param input The original string.
	 * @param delimiter The delimiter.
	 * @return A vector of the parts, not including the delimiters.
	 * @remark If no instances of the delimiter are found, the vector will contain just the original string.
	 */
	std::vector<std::string> split_by_string(const std::string& input, const std::string& delimiter);

    /**
     * @brief Removes any of the strings in strip_these from the subject
     * @param subject The original string.
     * @param strip_these All the strings to remove any instances of.
     * @return The new string without the unwanted strings.
     */
    std::string strip_string(const std::string& subject, const std::vector<std::string>& strip_these);

     /**
      * @brief Removes any whitespace characters from the string
      * @param subject The original string.
      * @return The new string without whitespace.
      * @remark Whitespace characters are any of: " ","\t","\n","\v","\f","\r"
      */
    std::string strip_whitespace(const std::string& subject);
 
}