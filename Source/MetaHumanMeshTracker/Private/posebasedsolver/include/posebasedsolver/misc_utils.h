// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <posebasedsolver/data_utils.h>
#include "disable_dlib_warnings.h"
POSEBASEDSOLVER_DISABLE_DLIB_WARNINGS
#include <dlib/config_reader.h>
#include <dlib/matrix.h>
POSEBASEDSOLVER_RENABLE_WARNINGS
#include <filesystem>

using namespace dlib;

namespace cm
{
	struct config_parsing_error : public error
	{
		/**
		*   cascade_parsing_error error constructor
		*/
		explicit config_parsing_error(const std::string& message) : error(message) {}
	};


	/**
	*    - REQUIREMENTS ON T
    *        - T must be either 'double', 'long double' or 'float'.
    *          In other words COMPILE_TIME_ASSERT(is_float_type<T>::value) must equal true.
    *
	*    - Ensures:
	*        - Similar to Matlab's linspace, populates a vector from first to last with n equally spaced elements
	*/
	template<typename T>
	std::vector<T> linspace_vector(T first, T last, unsigned int n)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);
		std::vector<T> result(n);
		if (0 == n)
		{
			return result;
		}
		T step = (last - first) / (n - 1);
		T val = first;
		auto it = result.begin();
		while (it != result.end())
		{
			*it = val;
			val += step;
			++it;
		}
		return result;
	}


	/**
    *    - Ensures:
    *        - Returns a new string which is copy of 'subject' with the 'search' string replaced by the 'replace' string.
    *        - If no instances of the search string are found, the original string is returned.
    */
    inline std::string replace_string(const std::string& subject, const std::string& search, const std::string& replace)
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


    /**
    *   - Ensures:
    *      - Returns a new string which is copy of 'subject' with all occurences of strings in 'strip_these' removed.
    */
    inline std::string strip_string(const std::string& subject, const std::vector<std::string>& strip_these)
    {
        auto stripped(subject);
        for (const auto& item : strip_these)
        {
            stripped = replace_string(stripped, item, "");
        }
        return stripped;
    }

    /**
     *   - Ensures:
     *       - Returns a new string which is copy of 'subject' with all whitespace removed.
     *       - Whitespace characters are any of: " ","\t","\n","\v","\f","\r"
     */
    inline std::string strip_whitespace(const std::string& subject)
    {
        return strip_string(subject, { " ","\t","\n","\v","\f","\r" });
    }


    inline std::string get_item(const config_reader& cr, const std::string& key)
    {
        std::string result;
        try
        {
            result = strip_whitespace(cr[key]);
        }
        catch (std::exception& e)
        {
            throw config_parsing_error("Problem reading " + result + " from config file.\n" + std::string(e.what()));
        }
        return result;
    }

    /**
     *    Interpolate linearly between keyframes
     *
     *    - REQUIREMENTS ON T
     *        - T must be either 'double', 'long double' or 'float'.
     *          In other words COMPILE_TIME_ASSERT(is_float_type<T>::value) must equal true.
     *
     *    - Requires
     *        - keyframes must have at least two elements
     *
     *    - Ensures
     *        - returns a vector of values which linearly interpolates between the given keyframes
     *        - the vector will include the first and last keyframes
     */
    template<typename T>
    inline std::vector<T> uniform_linear_interpolate(std::map<int, T> keyframes)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);

        auto key_times = list_all_keys(keyframes);
        int n_elements = key_times.back() - key_times.front() + 1;
        std::vector<T> result(n_elements);
        auto prev_key_index = 0;
        auto next_key_index = prev_key_index + 1;
        for (auto t = 0; t < n_elements; t++)
        {
            if (t > key_times[next_key_index])
            {
                prev_key_index = next_key_index;
                next_key_index = prev_key_index + 1;
            }

            T prev_time = key_times[prev_key_index];
            T next_time = key_times[next_key_index];

            T alpha = static_cast<T>(t - prev_time) / static_cast<T>(next_time - prev_time);
            result[t] = keyframes[static_cast<int>(prev_time)] * (1 - alpha) + alpha * keyframes[static_cast<int>(next_time)];
        }

        return result;
    }

    /**
     *    Create a keyframe-reduced set of keyframes from a vector of data.
     *
     *    - REQUIREMENTS ON T
     *        - T must be either 'double', 'long double' or 'float'.
     *          In other words COMPILE_TIME_ASSERT(is_float_type<T>::value) must equal true.
     *
     *    - Ensures
     *        - Takes a vector of data and returns a keyframe (time-value) curve which would
     *          reconstruct the original data to tolerance given by 'tolerance' if it were
     *          reconstructed using simple linear interpolation
     */
    template<typename T>
    std::map<int, T> dense_data_to_key_frames(const std::vector<T>& data, T tolerance)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);

        std::map<int, T> result;

        if (tolerance <= 0)
        {
            for (unsigned f = 0; f < data.size(); ++f)
            {
                result[f] = data[f];
            }
        }
        else
        {
            result[0] = data[0];
            const unsigned n_times = static_cast<unsigned>(data.size());
            result[n_times - 1] = data[n_times - 1];
            auto max_error = 999999999.9;
            while (max_error > tolerance)
            {
                auto predicted = uniform_linear_interpolate(result);
                auto worst_frame = 0;
                T this_max_error = -1.0f;
                for (unsigned f = 0; f < n_times; ++f)
                {
                    auto delta = std::abs(data[f] - predicted[f]);
                    if (delta > this_max_error)
                    {
                        worst_frame = f;
                        this_max_error = delta;
                    }
                }
                max_error = this_max_error;
                result[worst_frame] = data[worst_frame];
            }
        }

        return result;
    }

    /**
     *    Stores path, name, and exten
     *
     */
    struct file_parts
    {
        std::string path;
        std::string name;
        std::string ext;
    };

    /**
    *    - Ensures:
    *        - Returns a file_parts structure containing the elements of a full path
    *          split into the three parts, path, name, ext
    *
    */
    inline file_parts get_file_parts(std::string filename)
    {
        auto idx0 = filename.rfind("/");

        if (idx0 == std::string::npos)
            idx0 = filename.rfind("\\");
        const auto idx1 = filename.rfind(".");

        file_parts fp;
        if (idx0 != std::string::npos)
        {
            fp.path = filename.substr(0, idx0 + 1);
        }
        if (idx0 != std::string::npos && idx1 != std::string::npos)
        {
            fp.name = filename.substr(idx0 + 1, idx1 - idx0 - 1);
            fp.ext = filename.substr(idx1);
        }
        if (idx0 == std::string::npos && idx1 != std::string::npos)
        {
            fp.name = filename.substr(0, idx1);
            fp.ext = filename.substr(idx1);
        }
        return fp;
    }


    /*!
        - Requires:
            - if the directory associated to dirname exists, a temporary file named tmp.tmp must not exist in such directory.

        - Ensures:
            - returns true if the directory associated to dirname exists, and if a file is writable in that directory.
            - returns false if the directory associated to the dirname does not exist.
            - returns false if the directory associated to the dirname exists and a ofstream could not be opened succesfully to a (new) file named tmp.tmp in it.
    */
    inline bool is_directory_writable(const std::string& dirname)
    {
        using path = std::filesystem::path;
        const auto dir_path = path(dirname);

        if(!exists(dir_path))
        {
            return false;
        }

        const path tmp_filename("tmp.tmp");
        const path tmp_file_path = dir_path / tmp_filename;

        try
        {
            {
                std::ofstream out(tmp_file_path);
                if (!out.is_open())
                {
                    return false;
                }
            }
            remove(tmp_file_path);
            return true;

        } catch (std::ofstream::failure&)
        {
            return false;
        }
    }



    /*!
        Confirms that a string is matlab-style range input
        - Ensures
            - returns true if string is of the form a:b:c where a, b, and c are integers
    */
    inline bool is_matlab_style_range(const std::string& range_str)
    {
        auto elements = split_by_string(range_str, ":");
        if(elements.empty() || elements.size() > 3)
        {
            return false;
        }

        try
        {
            for(const auto& elem : elements)
            {
				int i = std::stoi(elem);
				if (i == 0 && elem != "0") // note this doesn't catch all possible failure cases, as (for example) "00" or "000" would fail here
				{
					return false;
				}
			}
        }
        catch(...)
        {
            return false;
        }
        return true;
    }

    /*!
        Creates a vector of strings (representing ints) from matlab-style range input
        - Requires
            - string must be of the form a:b:c where a, b, and c are integers
              i.e. is_matlab_style_range(range_str) == true;
        - Ensures
            - returns a vector of strings (representing ints) from matlab-style range input
    */
    inline std::vector<std::string> parse_matlab_style_range(const std::string& range_str)
    {
        DLIB_ASSERT(is_matlab_style_range(range_str) == true,
            "\t  parse_matlab_style_range(const std::string& range_str)"
            << "\n\t Invalid inputs were given to this function "
            << "\n\t input: " << range_str);

        std::vector<std::string> output;

        auto elems = split_by_string(range_str, ":");
        if (elems.size()>2)
        {
            const auto first = stoi(elems[0]);
            const auto step = stoi(elems[1]);
            const auto last = stoi(elems[2]);
            for (auto i = first; i <= last; i += step)
            {
                output.emplace_back(std::to_string(i));
            }
        }
        else if (elems.size()>1)
        {
            const auto first = stoi(elems[0]);
            const auto last = stoi(elems[1]);
            for (auto i = first; i <= last; ++i)
            {
                output.emplace_back(std::to_string(i));
            }
        }
        else
        {
            output.emplace_back(elems[0]);
        }

        return output;
    }



    /**
    *   Error loading a csv file.
    */
    struct load_csv_error : public dlib::error
    {
        explicit load_csv_error(const std::string& message) : error(message) {}
    };

    /**
    *   Loads simple comma-separated data from a text file without headings.
    *    - Ensures
    *       - Returns a std::vector<std::vector<std::string>> where each element
    *         is a vector of elements for each row.
    *    - Throws
    *       - load_csv_error if file couldn't found or parsed correctly
    */
    inline std::vector<std::vector<std::string>> load_raw_csv_as_strings(const std::string& filename)
    {
        try
        {
            std::vector<std::vector<std::string>> results;
            std::ifstream file;
            file.open(filename);
            if (!file.is_open())
            {
                throw load_csv_error("Error loading csv data - couldn't open file " + filename + ".\n");
            }
            std::string line;
            while (std::getline(file, line))
            {
                results.push_back(split_by_string(line, ","));
            }
            file.close();
            return results;
        }
        catch (std::exception & ex)
        {
            throw load_csv_error("Problem reading file: " + filename + ".\n" + ex.what());
        }
        catch (...)
        {
            throw load_csv_error("Problem reading file: " + filename + ".\n");
        }
    }

	/**
	*   Loads simple comma-separated data from a text file.
	*    - Requires
	*		- Input is a *headerless* csv
	*		- Each line is a comma-separated list of value
	*		- Values can be parseable as type T
	*    - Ensures
	*       - Returns a std::vector of std::vector<std::T>>
	*       - The elements of the outer vector correspond to the lines of the file
	*       - The elements of each inner vector contain the elements of each column in the csv file
	*    - Throws
	*       - load_csv_error if file couldn't be found or parsed correctly
	*/
	template<typename T>
	std::vector<std::vector<T>> load_csv(const std::string& filename)
    {
		try
		{
			std::vector<std::vector<T>> results;
			std::ifstream file;
			file.open(filename);
			if (!file.is_open())
			{
				throw load_csv_error("Error loading csv data - couldn't open file " + filename + ".\n");
			}
			std::string line;
			while (std::getline(file, line))
			{
				std::vector<std::string> cells = split_by_string(line, ",");
				std::vector<T> values;
				for(const auto& val : cells)
				{
					values.push_back(stod(val));
				}
				results.emplace_back(values);
			}
			file.close();
			return results;
		}
		catch (std::exception & ex)
		{
			throw load_csv_error("Problem reading file: " + filename + ".\n" + ex.what());
		}
		catch (...)
		{
			throw load_csv_error("Problem reading file: " + filename + ".\n");
		}
    }

    /**
    *   Loads simple comma-separated data from a text file.
    *    - Ensures
    *       - Returns a std::map of std::vector<std::string>>
    *       - The keys of the map are the elements in the first line of the file
    *       - The elements of the map contain the elements of each column in the csv file
    *         excluding the elements in the first line (which are considered to be headings).
    *    - Throws
    *       - load_csv_error if file couldn't found or parsed correctly
    */
    inline std::map<std::string, std::vector<std::string>> load_csv_as_strings(const std::string& filename)
    {
        try
        {
            std::map<std::string, std::vector<std::string>> results;
            std::ifstream file;
            file.open(filename);
            if (!file.is_open())
            {
                throw load_csv_error("Error loading csv data - couldn't open file " + filename + ".\n");
            }
            std::string line;
            std::getline(file, line);
            auto keys = split_by_string(line, ",");
			//Remove whitespace from keys
			for (auto& key : keys)
			{
				key = strip_whitespace(key);
			}
            for (const auto& key : keys)
            {
                results[key] = std::vector<std::string>();
            }

            auto line_num = 0;
            std::vector<std::string> lines;
            while (std::getline(file, line))
            {
                auto count = 0;
                auto parts = split_by_string(line, ",");
                if (keys.size() > parts.size())
                {

                    throw load_csv_error("Error loading csv data from file:" + filename + ". Incorrect number of values on line " + std::to_string(line_num) + ".");
                }
                for (const auto& key : keys)
                {
                    results[key].push_back(parts[count++]);
                }
                line_num++;
            }
            file.close();
            return results;
        }
        catch (std::exception & ex)
        {
            throw load_csv_error("Problem reading file: " + filename + ".\n" + ex.what());
        }
        catch (...)
        {
            throw load_csv_error("Problem reading file: " + filename + ".\n");
        }
    }

    /**
    *   Error loading a qpt file.
    */
    struct load_qpt_error : public dlib::error
    {
        explicit load_qpt_error(const std::string& message) : error(message) {}
    };

    /**
    *   Load a set of 2d points from a qpt file, where points are identified by integers.
    *
    *    - REQUIREMENTS ON T
    *	       - Must be either float, double, or long double, ie:
    *		     is_float_type<T>::value == true
    *
    *    - Ensures
    *       - Returns a map of shapes, keyed by frame number, where each shape is map of 2d points,
    *         with integer keys
    */
    template<typename T>
    std::map<int,std::map<int,point2<T>>> load_int_keyed_qpt(const std::string& filename)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);
        try
        {
            std::map<int, std::map<int, point2<T>>> result;
            auto data = load_csv_as_strings(filename);

            const auto n_csv_rows = data["point_id"].size();
            for (int i = 0; i < static_cast<int>(n_csv_rows); ++i)
            {
                auto frame_number = stoi(data["frame_number"][i]);
                if (result.find(frame_number) == result.end())
                {
                    result[frame_number] = std::map<int, point2<T>>();
                }
                const auto id = stoi(data["point_id"][i]);
                if (is_same_type<T, float>::value)
                {
                    result[frame_number][id] = point2<float>(stof(data["x"][i]), stof(data["y"][i]));
                }
                else if (is_same_type<T, double>::value)
                {
                    result[frame_number][id] = point2<double>(stod(data["x"][i]), stod(data["y"][i]));
                }
                else if (is_same_type<T, long double>::value)
                {
                    result[frame_number][id] = point2<long double>(stold(data["x"][i]), stold(data["y"][i]));
                }
            }

            return result;
        }
        catch (std::exception& e)
        {
            throw load_qpt_error("Error loading qpt filename: " + filename + "\n" + e.what());
        }
    }

    /**
    *   Load a set of 2d points from a qpt file, where points are identified by strings.
    *
    *    - REQUIREMENTS ON T
    *	       - Must be either float, double, or long double, ie:
    *		     is_float_type<T>::value == true
    *
    *    - Ensures
    *       - Returns a map of shapes, keyed by frame number, where each shape is map of 2d points,
    *         with string keys
    */
    template<typename T>
    std::map<int, std::map<std::string, point2<T>>> load_string_keyed_qpt(const std::string& filename)
    {
        COMPILE_TIME_ASSERT(is_float_type<T>::value);
        try
        {
            std::map<int, std::map<std::string, point2<T>>> result;
            auto data = load_csv_as_strings(filename);

            const auto n_csv_rows = data["point_id"].size();
            for (int i = 0; i < static_cast<int>(n_csv_rows); ++i)
            {
                auto frame_number = stoi(data["frame_number"][i]);
                if (result.find(frame_number) == result.end())
                {
                    result[frame_number] = std::map<std::string, point2<T>>();
                }
                const auto id = data["point_id"][i];
                if constexpr (is_same_type<T, float>::value)
                {
                    result[frame_number][id] = point2<float>(stof(data["x"][i]), stof(data["y"][i]));
                }
                else if constexpr (is_same_type<T, double>::value)
                {
                    result[frame_number][id] = point2<double>(stod(data["x"][i]), stod(data["y"][i]));
                }
                else if constexpr (is_same_type<T, long double>::value)
                {
                    result[frame_number][id] = point2<long double>(stold(data["x"][i]), stold(data["y"][i]));
                }
            }
            return result;
        }
        catch (std::exception& e)
        {
            throw load_qpt_error("Error loading qpt filename: " + filename + "\n" + e.what());
        }
    }

    /**
    *   Error converting a string to a Boolean value.
    */
    struct string_to_bool_error : public error
    {
        explicit string_to_bool_error(const std::string& message) : error(message) {}
    };


    /**
    *   Error loading a qsa file.
    */
    struct load_qsa_error : public dlib::error
    {
        explicit load_qsa_error(const std::string& message) : error(message) {}
    };

    inline std::map<std::string, std::map<int, double>> load_qsa(const std::string& filename)
    {
        try
        {
            auto data = load_csv_as_strings(filename);
            std::map<std::string, std::map<int, double>> numbered_results;

            std::vector<int> all_frame_numbers;
            std::vector<std::string> all_control_names;
            std::vector<double> all_control_values;
            const auto n_csv_rows = data["control_name"].size();
            for (auto i = 0; i < static_cast<int>(n_csv_rows); ++i)
            {
                all_frame_numbers.push_back(stoi(data["frame_number"][i]));
                all_control_names.push_back(data["control_name"][i]);
                all_control_values.push_back(stod(data["control_value"][i]));
            }

            for (auto i = 0; i < static_cast<int>(n_csv_rows); ++i)
            {
                const auto control_name = all_control_names[i];
                if (numbered_results.find(control_name) == numbered_results.end())
                {
                    numbered_results[control_name] = std::map<int, double>();
                }
                auto frame_number = all_frame_numbers[i];
                numbered_results[control_name][frame_number] = all_control_values[i];
            }
            return numbered_results;
        }
        catch (std::exception& e)
        {
            throw load_qsa_error("Error loading qsa, filename: " + filename + "\n" + e.what());
        }
    }

    inline std::vector<int> frame_number_string_to_ints(const std::string& str)
    {
        std::vector<int> result;
        auto first_level = split_by_string(strip_whitespace(str), ",");
        for (const auto& item : first_level)
        {
            const auto next_to_split(item);
            auto second_level = split_by_string(next_to_split, ":");
            if (second_level.size() == 1)
            {

               result.push_back(stoi(second_level[0]));
            }
            else if (second_level.size() == 2)
            {
                const auto first = stoi(second_level[0]);
                const auto last = stoi(second_level[1]);
                for (auto i = first; i <= last; ++i)
                {

                    result.push_back(i);

                }
            }
            else if (second_level.size() == 3)
            {
                const auto first = stoi(second_level[0]);
                const auto step = stoi(second_level[1]);
                const auto last = stoi(second_level[2]);
                for (auto i = first; i <= last; i += step)
                {
                    result.push_back(i);
                }
            }
        }
        return result;
    }


    template<typename T>
    struct to_string_helper
    {
        static std::string get(const T& v) { return std::to_string(v); }
    };

    template<>
    struct to_string_helper<std::string>
    {
        static std::string get(const std::string& s) { return s; }
    };

    /*!
        Concatenates the elements in the container into a single string. Each element is separated by the separator string passed as second parameter.

        - Requirements on Container:
            - Must be iterable
            - Must support std::begin(container) and std::end(container)
            - Must support container.size()

        - Requirements on T:
            - Must be an arithmetic type or a string: std::is_arithmetic<T>::value == true or std::is_same<T, std::string>::value == true
    */
    template<template <typename, typename...> class Container, typename T, class... Alloc>
    inline std::string elements_to_string(const Container<T, Alloc...>& container, const std::string& separator = ", ")
    {
        COMPILE_TIME_ASSERT((std::is_arithmetic<T>::value || std::is_same<T, std::string>::value));

        std::string v;

        if (container.size() > 0)
        {
            auto start = std::begin(container);
            auto stop = std::end(container);
            --stop;
            std::for_each(start, stop, [&v, &separator](const auto& e) { v += to_string_helper<T>::get(e) + separator; });
            v += to_string_helper<T>::get(container.back());
        }
        return v;
    }

	inline std::pair<int, std::string> parse_int_with_f_prefix(const std::string& str, bool allow_f_prefix)
	{
		std::string stripped = str;
		if (allow_f_prefix && str[0] == 'f')
		{
			stripped = str.substr(1);
		}

		return std::make_pair(stoi(stripped), str);
	}

	inline std::vector<std::string> matlab_style_range_to_vector(const std::vector<std::string>& css, bool allow_f_prefix = false)
	{
		std::vector<std::string> result;
		for (const auto& str : css)
		{
			auto elems = split_by_string(str, ":");
			if (elems.size() > 2)
			{
				const auto first = parse_int_with_f_prefix(elems[0], allow_f_prefix);
				const auto step = stoi(elems[1]);
				const auto last = parse_int_with_f_prefix(elems[2], allow_f_prefix);

				if (step > 0)
				{
					result.emplace_back(first.second);
					int i;
					for (i = first.first + step; i < last.first; i += step)
					{
						result.emplace_back(std::to_string(i));
					}
					if (i == last.first)
						result.emplace_back(last.second);
				}
				else
				{
					result.emplace_back(first.second);
					int i;
					for (i = first.first + step; i > last.first; i += step)
					{
						result.emplace_back(std::to_string(i));
					}
					if (i == last.first)
						result.emplace_back(last.second);
				}
			}
			else if (elems.size() > 1)
			{
				const auto first = parse_int_with_f_prefix(elems[0], allow_f_prefix);
				const auto last = parse_int_with_f_prefix(elems[1], allow_f_prefix);
				result.emplace_back(first.second);
				for (auto i = first.first + 1; i < last.first; ++i)
				{
					result.emplace_back(std::to_string(i));
				}
				result.emplace_back(last.second);
			}
			else
			{
				result.emplace_back(elems[0]);
			}
		}

		return result;
	}

	inline bool validated_matlab_range_to_int_vector(const std::vector<std::string>& css, std::vector<int>& result)
	{
		result.clear();
		try {
			for (const auto& str : matlab_style_range_to_vector(css))
			{
				result.emplace_back(stoi(str));
			}
			return true;
		}
		catch (...)
		{
			return false;
		}
	}

	inline std::vector<int> matlab_ranges_to_int_vector(const std::vector<std::string>& css)
	{
		std::vector<int> result;
		for (const auto& str : css)
		{
			auto elems = split_by_string(str, ":");
			if (elems.size() > 2)
			{
				const auto first = stoi(elems[0]);
				const auto step = stoi(elems[1]);
				const auto last = stoi(elems[2]);
				if (step > 0)
				{
					for (auto i = first; i <= last; i += step)
					{
						result.emplace_back(i);
					}
				}
				else
				{
					for (auto i = first; i >= last; i += step)
					{
						result.emplace_back(i);
					}
				}
			}
			else if (elems.size() > 1)
			{
				const auto first = stoi(elems[0]);
				const auto last = stoi(elems[1]);
				for (auto i = first; i <= last; ++i)
				{
					result.emplace_back(i);
				}
			}
			else
			{
				result.emplace_back(stoi(elems[0]));
			}
		}

		return result;
	}


    template <typename T>
    T get_range_checked_item(const config_reader& cr, const std::string& key, T min, T max)
    {
        try
        {
            std::stringstream ss(strip_whitespace(cr[key]));
            T result;

            if (!(ss >> result))
            {
                throw config_parsing_error("String conversion error reading " + key + " from config file.\n");
            }

            if (result<min || result>max)
            {
                throw config_parsing_error("Out-of-range problem reading " + key + " from config file.\n");
            }
            return result;
        }
        catch (std::exception& e)
        {
            throw config_parsing_error("Problem reading " + key + " from config file.\n" + std::string(e.what()));
        }
    }

	template <typename T>
	T get_range_checked_item(const config_reader& cr, const std::string& block_name, const std::string& key, T min, T max)
	{
		try
		{
			std::stringstream ss(strip_whitespace(cr.block(block_name)[key]));
			T result;

			if (!(ss >> result))
			{
				throw config_parsing_error("String conversion error reading " + key + " from config file.\n");
			}

			if (result<min || result>max)
			{
				throw config_parsing_error("Out-of-range problem reading " + key + " from config file.\n");
			}
			return result;
		}
		catch (std::exception& e)
		{
			throw config_parsing_error("Problem reading " + key + " from config file.\n" + std::string(e.what()));
		}
	}

	template <typename T>
	T get_range_checked_item(const config_reader& cr, const std::string& block_1_name, const std::string& block_2_name, const std::string& key, T min, T max)
	{
		try
		{
			std::stringstream ss(strip_whitespace(cr.block(block_1_name).block(block_2_name)[key]));
			T result;

			if (!(ss >> result))
			{
				throw config_parsing_error("String conversion error reading " + key + " from config file.\n");
			}

			if (result<min || result>max)
			{
				throw config_parsing_error("Out-of-range problem reading " + key + " from config file.\n");
			}
			return result;
		}
		catch (std::exception& e)
		{
			throw config_parsing_error("Problem reading " + key + " from config file.\n" + std::string(e.what()));
		}
	}

    inline double get_range_checked_double_in_block_and_location(
        const config_reader& cr,
        const std::string& block,
        const std::string& key,
        int location,
        double min,
        double max)
    {
        try
        {
            const auto result = stod(split_by_string(strip_whitespace(cr.block(block)[key]), ",")[location]);
            if (result<min || result>max)
            {
                throw config_parsing_error("Out-of-range problem reading " + key + " from config file.\n");
            }
            return result;
        }
        catch (std::exception& e)
        {
            throw config_parsing_error("Problem reading " + key + " from config file.\n" + std::string(e.what()));
        }
    }

    inline std::string get_nested_config_item(const config_reader& cr, const std::vector<std::string>& key_levels, const std::map<std::string, std::string>& aliases)
    {
        DLIB_ASSERT(key_levels.size() < 5);
        std::string output;
        if (key_levels.size() == 1)
        {
            output = cr[key_levels.at(0)];
        }
        else if (key_levels.size() == 2)
        {
            output = cr.block(key_levels[0])[key_levels[1]];
        }
        else if (key_levels.size() == 3)
        {
            output = cr.block(key_levels[0]).block(key_levels[1])[key_levels[2]];
        }
        else if (key_levels.size() == 4)
        {
            output = cr.block(key_levels[0]).block(key_levels[1]).block(key_levels[2])[key_levels[3]];
        }
        for (const auto& item : aliases)
        {
            output = replace_string(strip_whitespace(output), item.first, item.second);
        }
        return output;
    }

    template <typename T>
    std::vector<T> get_comma_separated_vector(const std::string & str, int min_length, int max_length)
    {
        try
        {
            std::vector<T> result;
            auto parts = split_by_string(strip_whitespace(str), ",");
            for (const auto& part : parts)
            {
                std::stringstream ss(part);
                T val;
                if (!(ss >> val))
                {
                    throw config_parsing_error("String conversion error parsing " + str + "\n");
                }
                result.push_back(val);
            }
            if (static_cast<int>(result.size()) < min_length || static_cast<int>(result.size()) > max_length)
            {
                throw config_parsing_error("Vector length error parsing " + str + "\n");
            }
            return result;
        }
        catch (std::exception& e)
        {
            throw config_parsing_error("Problem paring " + str + "\n" + std::string(e.what()));
        }
    }

    template <typename T>
    std::vector<T> get_comma_separated_vector(const config_reader& cr, const std::string& key, int min_length, int max_length)
    {
        try
        {
            const std::string str = get_item(cr, key);
            return get_comma_separated_vector<T>(str, min_length, max_length);
        }
        catch (std::exception& e)
        {
            throw config_parsing_error("Problem reading " + key + " from config file.\n" + std::string(e.what()));
        }
    }

    template <typename T>
    std::vector<T> get_ampersand_separated_vector(const config_reader& cr, const std::string& key, int min_length, int max_length)
    {
        try
        {
            std::vector<T> result;
            auto parts = split_by_string(strip_whitespace(cr[key]), "&");
            for (const auto& part : parts)
            {
                std::stringstream ss(part);
                T val;
                if (!(ss >> val))
                {
                    throw config_parsing_error("String conversion error reading " + key + " from config file.\n");
                }
                result.push_back(val);
            }
            if (static_cast<int>(result.size()) < min_length || static_cast<int>(result.size()) > max_length)
            {
                throw config_parsing_error("Vector length error reading " + key + " from config file.\n");
            }
            return result;
        }
        catch (std::exception& e)
        {
            throw config_parsing_error("Problem reading " + key + " from config file.\n" + std::string(e.what()));
        }
    }

    template <typename T>
    std::vector<T> get_comma_separated_vector(const config_reader& cr, const std::vector<std::string>& nested_key)
    {
        try
        {
            std::vector<T> result;
            auto parts = split_by_string(strip_whitespace(get_nested_config_item(cr,nested_key,{})), ",");
            for (const auto& part : parts)
            {
                std::stringstream ss(part);
                T val;
                if (!(ss >> val))
                {
                    throw config_parsing_error("String conversion error reading " + nested_key[0] + " from config file.\n");
                }
                result.push_back(val);
            }
            return result;
        }
        catch (std::exception& e)
        {
            throw config_parsing_error("Problem reading " + nested_key[0] + " from config file.\n" + std::string(e.what()));
        }
    }

    template <typename T>
    T get_enum_item(const config_reader& cr, const std::string& key)
    {
        try
        {
            T result = to_enum<T>(strip_whitespace(cr[key]));
            return result;
        }
        catch (std::exception& e)
        {
            throw config_parsing_error("Problem reading " + key + " from config file.\n" + std::string(e.what()));
        }
    }


	/**
    *   Load a set of 3d points from a qp3 file, where points are identified by strings.
    *
    *    - REQUIREMENTS ON T
    *	       - Must be either float, double, or long double, ie:
    *		     is_float_type<T>::value == true
    *
    *    - Ensures
    *       - Returns a map of shapes, keyed by frame number, where each shape is map of 3d points,
    *         with string keys
    */
	template<typename T>
	std::map<int, std::map<std::string, point3<T>>> load_string_keyed_qp3(const std::string& filename)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);
		try
		{
			std::map<int, std::map<std::string, point3<T>>> result;
			auto data = load_csv_as_strings(filename);

			const auto n_csv_rows = data["point_id"].size();
			for (unsigned  i = 0; i < n_csv_rows; ++i)
			{
				auto frame_number = stoi(data["frame_number"][i]);
				if (result.find(frame_number) == result.end())
				{
					result[frame_number] = std::map<std::string, point3<T>>();
				}
				const auto id = data["point_id"][i];
				if (is_same_type<T, float>::value)
				{
					result[frame_number][id] = point3<float>(stof(data["x"][i]), stof(data["y"][i]), stof(data["z"][i]));
				}
				else if (is_same_type<T, double>::value)
				{
					result[frame_number][id] = point3<double>(stod(data["x"][i]), stod(data["y"][i]), stod(data["z"][i]));
				}
				else if (is_same_type<T, long double>::value)
				{
					result[frame_number][id] = point3<long double>(stold(data["x"][i]), stold(data["y"][i]), stold(data["z"][i]));
				}
			}
			return result;
		}
		catch (std::exception& e)
		{
			throw load_qpt_error("Error loading qp3 filename: " + filename + "\n" + e.what());
		}
	}

	/**
	 *   Creates a vector of strings (representing ints) from a comma-separated list of matlab-style range inputs
	 *   - Requires
	 *		- string must be a comma-separated list of items, whereby:
	 *		- each item is parseable by parse_matlab_style_range, i.e:
	 *			- a string of the form a:b:c where a, b, and c are integers, OR
	 *			- a string that represents a single integer
	 *
	 *   - Ensures
	 *       - returns a vector of strings (representing ints)
	 */
	inline std::vector<std::string> parse_point_id_string(const std::string& filter_string)
	{
		std::vector<std::string> result;
		auto filter_sets = split_by_string(filter_string, ",");
		for (const auto& filter_set : filter_sets)
		{
			auto ids = parse_matlab_style_range(filter_set);
			for (const auto& id : ids)
			{
				result.push_back(id);
			}
		}
		return result;
	}


	/**
   *   Load a vector of 2d points from a qpt file, where points are identified by strings.
   *
   *    - REQUIREMENTS ON T
   *	       - Must be either float, double, or long double, ie:
   *		     is_float_type<T>::value == true
   *
   *    - Ensures
   *       - Returns a vector of shapes of size equal to the frame number, where each shape is map of 2d points,
   *         with string keys
   */
	template<typename T>
	std::vector<std::map<std::string, point2<T>>> load_qpt_basic(const std::string& filename)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);
		try
		{
			std::map<int, std::map<std::string, point2<T>>> numbered_results = load_string_keyed_qpt<T>(filename);
			std::vector<std::map<std::string, point2<T>>> results;

			auto valid_frame_numbers = list_all_keys(numbered_results);
			int max_frame = *max_element(valid_frame_numbers.begin(), valid_frame_numbers.end());

			for (auto f : zero_based_int_range(static_cast<unsigned>(max_frame + 1)))
			{
				if (numbered_results.find(f) != numbered_results.end())
				{
					results.emplace_back(numbered_results[f]);
				}
				else
				{
					results.emplace_back(std::map<std::string, point2<T>>());
					std::cout << "WARNING: missing data at frame " << f << std::endl;
				}
			}

			return results;
		}
		catch (std::exception& e)
		{
			throw load_qpt_error("Error loading qpt filename: " + filename + "\n" + e.what());
		}
	}

	/**
	*   Load a vector of 3d points from a qp3 file, where points are identified by strings.
	*
	*    - REQUIREMENTS ON T
	*	       - Must be either float, double, or long double, ie:
	*		     is_float_type<T>::value == true
	*
	*    - Ensures
	*       - Returns a vector of shapes of size equal to the frame number, where each shape is map of 3d points,
	*         with string keys
	*/
	template<typename T>
	std::vector<std::map<std::string, point3<T>>> load_qp3(const std::string& filename)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);
		try
		{
			auto data = load_csv_as_strings(filename);
			std::map<int, std::map<std::string, point3<T>>> numbered_results;
			std::vector<std::map<std::string, point3<T>>> results;

			const auto n_csv_rows = data["point_id"].size();
			for (auto i = 0; i < static_cast<int>(n_csv_rows); ++i)
			{
				auto frame_number = stoi(data["frame_number"][i]);
				if (numbered_results.find(frame_number) == numbered_results.end())
				{
					numbered_results[frame_number] = std::map<std::string, point3<T>>();
				}
				const auto id = data["point_id"][i];
				if constexpr (is_same_type<T, float>::value)
				{
					numbered_results[frame_number][id] = point3<float>(stof(data["x"][i]), stof(data["y"][i]), stof(data["z"][i]));
				}
				else if constexpr (is_same_type<T, double>::value)
				{
					numbered_results[frame_number][id] = point3<double>(stod(data["x"][i]), stod(data["y"][i]), stof(data["z"][i]));
				}
				else if constexpr (is_same_type<T, long double>::value)
				{
					numbered_results[frame_number][id] = point3<long double>(stold(data["x"][i]), stold(data["y"][i]), stof(data["z"][i]));
				}
			}

			auto valid_frame_numbers = list_all_keys(numbered_results);
			int max_frame = *max_element(valid_frame_numbers.begin(), valid_frame_numbers.end());

			for (auto f : zero_based_int_range(static_cast<unsigned>(max_frame + 1)))
			{
				if (numbered_results.find(f) != numbered_results.end())
				{
					results.emplace_back(numbered_results[f]);
				}
				else
				{
					results.emplace_back(std::map<std::string, point3<T>>());
					std::cout << "WARNING: missing data at frame " << f << std::endl;
				}
			}

			return results;
		}
		catch (...)
		{
			throw std::runtime_error("Problem reading file.");
		}
	}

    /*!
    Saves a vector to a csv file.

    - Ensures:
        - Saves all vector's elements into a csv file.
    */
    template<typename T>
    void save_vector_to_file(const std::string& filename, const std::vector<std::vector<T>>& vec)
    {
        std::ofstream csv_file;

        csv_file.open(filename);
        for (auto fr = 0; fr < vec.size(); ++fr)
        {
            for (auto const& el : vec[fr])
            {
                csv_file << el << ",";
            }
            csv_file << "\n";
        }
        csv_file.close();
    }

	//Saves a .qp3 file
	/**
	*   Saves 3d points into a qp3 file.
	*
	*    - REQUIREMENTS ON T
	*	       - Must be either float, double, or long double, ie:
	*		     is_float_type<T>::value == true
	*
	*    - Requires
	*       - points3d.size() == point_ids.size()
	*    - Ensures
	*       - Stores all 3d points in a .qp3 file to filename qp3_filename using the supplied point ids
	*/
	template<typename T>
	void save_qp3_file(const std::string& qp3_filename, const std::vector<point3_vector<T>>& points3d, const std::vector<std::string>& point_ids)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);

		for(unsigned f = 0; f < (unsigned)points3d.size (); ++f)
		{
			DLIB_ASSERT (points3d[f].size () == point_ids.size (),
				"\t  save_qp3_file: points3d.size() != point_ids.size()");
		}

		std::ofstream qp3_file;
		qp3_file.open(qp3_filename);
		qp3_file << "frame_number,point_id,x,y,z\n";

		for (unsigned f = 0; f < (unsigned)points3d.size(); ++f)
		{
			auto frame_id = std::to_string(f);

			for (unsigned idx = 0; idx < (unsigned)point_ids.size(); idx++)
			{
				qp3_file << frame_id << "," << point_ids[idx] << "," << points3d[f][idx].x() << "," << points3d[f][idx].y() << "," << points3d[f][idx].z() << "\n";
			}
		}
		qp3_file.close();
	}

	//Saves a .qp3 file
	/**
	*   Saves 3d points into a qp3 file.
	*
	*    - REQUIREMENTS ON T
	*	       - Must be either float, double, or long double, ie:
	*		     is_float_type<T>::value == true
	*
	*    - Requires
	*       - points3d.size() == point_ids.size()
	*    - Ensures
	*       - Stores all 3d points in a .qp3 file to filename qp3_filename using the supplied point ids
	*/
	template<typename T>
	void save_qp3_file(const std::string& qpt_filename, const std::map<int, std::map<std::string, point3<T>>>& qp3_data)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);

		std::ofstream qpt_file;
		qpt_file.open(qpt_filename);
		qpt_file << "frame_number,point_id,x,y,z\n";

		for (typename std::map<int, std::map<std::string, point3<T>>>::const_iterator frame_it = qp3_data.begin(); frame_it != qp3_data.end(); ++frame_it)
		{
			for (typename std::map<std::string, point3<T>>::const_iterator point_it = frame_it->second.begin(); point_it != frame_it->second.end(); ++point_it)
			{
				qpt_file << frame_it->first << "," << point_it->first << "," << point_it->second.x() << "," << point_it->second.y() << "," << point_it->second.z() << "\n";
			}
		}
	}

	//Saves a .qpt file
	 /**
	 *   Saves 2d points into a qpt file.
	 *
	 *    - REQUIREMENTS ON T
	 *	       - Must be either float, double, or long double, ie:
	 *		     is_float_type<T>::value == true
	 *
	 *    - Ensures
	 *       - Stores all 2d points in a .qpt file, using the supplied map keys as point ids and scale factors to scale the results
	 */
	template<typename T>
	void save_qpt_file(const std::string& qpt_filename, const std::vector<point2_map<T>>& points2d, double x_scaling = 1, double y_scaling = 1)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);


		std::ofstream qpt_file;
		qpt_file.open(qpt_filename);
		qpt_file << "frame_number,point_id,x,y\n";

		for (unsigned f = 0; f < (unsigned)points2d.size(); ++f)
		{
			auto frame_id = std::to_string(f);
			for (const auto & pt : points2d[f])
			{
				qpt_file << frame_id << "," << pt.first << "," << pt.second.x() * x_scaling << "," << pt.second.y() * y_scaling << "\n";
			}
		}
		qpt_file.close();
	}

	//Saves a .qpt file
	/**
	*   Saves 2d points into a qpt file.
	*
	*    - REQUIREMENTS ON T
	*	       - Must be either float, double, or long double, ie:
	*		     is_float_type<T>::value == true
	*
	*    - Requires
	*       - points2d.size() == point_ids.size()
	*    - Ensures
	*       - Stores all 2d points in a .qpt file, using the supplied point ids and scale factors
	*/
	template<typename T>
	void save_qpt_file(const std::string& qpt_filename, const std::vector<point2_vector<T>>& points2d, const std::vector<std::string>& point_ids, double x_scaling = 1, double y_scaling = 1)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);

		DLIB_ASSERT(points2d.size() == point_ids.size(),
			"\t  save_qpt_file: points2d.size() != point_ids.size()");


		std::ofstream qpt_file;
		qpt_file.open(qpt_filename);
		qpt_file << "frame_number,point_id,x,y\n";

		for (unsigned f = 0; f < (unsigned)points2d.size(); ++f)
		{
			auto frame_id = std::to_string(f);
			for (unsigned idx = 0; idx < (unsigned)point_ids.size(); idx++)
			{
				qpt_file << frame_id << "," << point_ids[idx] << "," << points2d[f][idx].x() * x_scaling << "," << points2d[f][idx].y() * y_scaling << "\n";
			}
		}
		qpt_file.close();
	}

	//Saves a .qpt file
	/**
	*   Saves 2d points into a qpt file.
	*
	*    - REQUIREMENTS ON T
	*	       - Must be either float, double, or long double, ie:
	*		     is_float_type<T>::value == true
	*
	*    - Requires
	*       - points2d.size() == point_ids.size()
	*    - Ensures
	*       - Stores all 2d points in a .qpt file, using the supplied frame numbers and point ids
	*/
	template<typename T>
	void save_qpt_file(const std::string& qpt_filename, const std::map<int, std::map<std::string, point2<T>>>& qpt_data)
	{
		COMPILE_TIME_ASSERT(is_float_type<T>::value);

		std::ofstream qpt_file;
		qpt_file.open(qpt_filename);
		qpt_file << "frame_number,point_id,x,y\n";

		for (typename std::map<int, std::map<std::string, point2<T>>>::const_iterator frame_it = qpt_data.begin(); frame_it != qpt_data.end(); ++frame_it)
		{
			for (typename std::map<std::string, point2<T>>::const_iterator point_it = frame_it->second.begin(); point_it != frame_it->second.end(); ++point_it)
			{
				qpt_file << frame_it->first << "," << point_it->first << "," << point_it->second.x() << "<" << point_it->second.y() << "\n";
			}
		}
	}

	/*
	 *	Converts a single frame from a qpt to a vector of point2's
	 *
	 *	- Requires
	 *		- qp2_frame is a map of point id's (as strings) to point3's
	 *		- points_filter is a vector of point id's (as strings), which constitute a subset of the keys in qp2_frame
	 *	- Ensures
	 *		- v is populated with the coordinates of the points from qp2_frame whose id's are in points_filter
	 */
	inline void qpt_frame_filter_to_vector(const std::map<std::string, point2<double>>& qpt_frame, const std::vector<std::string>& points_filter, std::vector<point2<double>>& v)
	{
		const auto n_points = points_filter.size();
		v.resize(n_points);

		for (size_t i = 0; i < n_points; i++)
		{
			v[i] = qpt_frame.at(points_filter[i]);
		}
	}

	/*
	 *	Converts a single frame from a qp3 to a vector matrix of point3's
	 *
	 *	- Requires
	 *		- qp3_frame is a map of point id's (as strings) to point3's
	 *		- points_filter is a vector of point id's (as strings), which constitute a subset of the keys in qp3_frame
	 *	- Ensures
	 *		- v is populated with the coordinates of the points from qp3_frame whose id's are in points_filter
	 */
	inline void qp3_frame_filter_to_vector(const std::map<std::string, point3<double>>& qp3_frame, const std::vector<std::string>& points_filter, std::vector<point3<double>>& v)
	{
		const auto n_points = points_filter.size();
		v.resize(n_points);

		for (size_t i = 0; i < n_points; i++)
		{
			v[i] = qp3_frame.at(points_filter[i]);
		}
	}


    inline unsigned char saturate_cast(int v) { return (unsigned char)((unsigned)v <= UCHAR_MAX ? v : v > 0 ? UCHAR_MAX : 0); }
    inline unsigned char saturate_cast(double v) { int iv = static_cast<int>(std::round(v)); return saturate_cast(iv); }



    /*!
        Clamp value into the range 0, max
    */
    template <class T>
    T clamp0(T val, T max)
    {
        val = std::min(val, max);
        return val - (val < 0) * val;
    }
}
