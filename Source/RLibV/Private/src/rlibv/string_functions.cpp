// Copyright Epic Games, Inc. All Rights Reserved.

#include "../include/rlibv/string_functions.h"

#include <algorithm>
#include <filesystem>
#include <map>

namespace rlibv
{

	std::vector<std::pair<std::string,int>> collect_image_sequence(const std::vector<std::string>& candidate_filenames)
	{
		//Split filenames into path+name, number, extension
		std::vector<std::tuple<std::string, int, std::string>> numbers_extracted;

		for (const auto& filename : candidate_filenames)
		{
			std::filesystem::path p(filename);
			auto ext = p.extension().string();
			auto name = p.replace_extension("").string();
			size_t last_index = name.find_last_not_of("0123456789");
			std::string number_str = name.substr(last_index + 1);
			std::string stripped_str = name.substr(0, last_index + 1);
			if (!number_str.empty())
			{
				int number = std::stoi(number_str);
				numbers_extracted.emplace_back(std::tuple<std::string, int, std::string>{stripped_str + ext, number, filename });
			}
		}

		//Find most common naming and extensions
		std::map<std::string, int> root_counts;
		for (const auto& [name_and_ext, number, filename] : numbers_extracted)
		{
			if (root_counts.find(name_and_ext) == root_counts.end())
			{
				root_counts[name_and_ext] = 1;
			}
			else
			{
				root_counts[name_and_ext]++;
			}
		}

		int max_count = 0;
		std::string best_template;
		for (const auto& item : root_counts)
		{
			if (item.second > max_count)
			{
				max_count = item.second;
				best_template = item.first;
			}
		}

		numbers_extracted.erase(std::remove_if(numbers_extracted.begin(), numbers_extracted.end(), [&best_template](const std::tuple<std::string, int, std::string>& a)->bool { return (std::get<0>(a) != best_template); }), numbers_extracted.end());

		//Sort on the frame number
		std::sort(numbers_extracted.begin(), numbers_extracted.end(), [](std::tuple<std::string, int, std::string>& a, std::tuple<std::string, int, std::string>& b)->bool { return std::get<1>(a) < std::get<1>(b); });

		std::vector<std::pair<std::string, int>> result;
		for (const auto& item : numbers_extracted)
		{
			result.emplace_back(std::pair<std::string, int>{ std::get<2>(item),std::get<1>(item) });
		}

		return result;
	}

	std::string replace_string(const std::string& subject, const std::string& search, const std::string& replace)
	{
		auto output(subject);
		size_t pos = 0;
		while ((pos = output.find(search, pos)) != std::string::npos)
		{
			output.replace(pos, search.length(), replace);
			pos += replace.length();
		}
		return output;
	}

	bool string_ends_with(const std::string& str, const std::string& cmp)
	{
		if (str.size() >= cmp.size())
		{
			return (str.compare(str.size() - cmp.size(), cmp.size(), cmp) == 0);
		}
		return false;
	}

	std::vector<std::string> split_by_string(const std::string& input, const std::string& delimiter)
	{
		std::vector<std::string> result;
		
		size_t begin_pos = 0, end_pos = 0, delimiter_len = delimiter.length();

		while ((end_pos = input.find(delimiter, begin_pos)) != std::string::npos)
		{
			result.emplace_back(input.substr(begin_pos, end_pos - begin_pos));

			begin_pos = end_pos + delimiter_len;
		}

		if (begin_pos < input.size())
		{
			result.emplace_back(input.substr(begin_pos));
		}

		return result;
	}

	std::string strip_string(const std::string& subject, const std::vector<std::string>& strip_these)
	{
		auto stripped(subject);
		for (const auto& item : strip_these)
		{
			stripped = replace_string(stripped, item, "");
		}
		return stripped;
	}

	std::string strip_whitespace(const std::string& subject)
	{
		return strip_string(subject, { " ","\t","\n","\v","\f","\r" });
	}
}
