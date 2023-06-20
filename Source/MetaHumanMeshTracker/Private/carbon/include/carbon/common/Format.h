// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#if defined(USE_FMT_FORMAT)
    #include <fmt/format.h>
#else
    #include <iomanip>
    #include <limits>
    #include <sstream>
    #include <vector>
#endif

/**
 * Extremely basic implementation of c++20/libfmt formatting. Only supports placeholders as {}.
 * The purpose of this implementation is to minimize 3rd party dependencies.
 * If we can use libfmt, or if c++20 format support is available, then i would recommend using that.
 */

namespace epic {
namespace carbon {
namespace fmt {

// ! Trivial to_string implementation. Compared to std::to_string this prints out floating point in full precision.
template<typename T>
inline std::string to_string(const T& value) {
    std::ostringstream s;
    if constexpr (std::is_floating_point<T>::value) {
        // set maximum precision for floating point so that subsequent parsing gives the exact same result
        s << std::setprecision(std::numeric_limits<T>::max_digits10) << std::noshowpoint;
        s << value;
    } else if constexpr (std::is_enum<T>::value) {
        s << int(value);
    } else {
        s << value;
    }
    return s.str();
}

// ! Specialization for bool type
template<> inline std::string to_string<bool>(const bool& value) {
    return value ? "true" : "false";
}

#if defined(USE_FMT_FORMAT)  // use fmt implementation of format() and print()

    template<typename ... Types>
    std::string format(Types&& ... args) {
        return ::fmt::format(args ...);
    }

    template<typename ... Types>
    void print(Types&& ... args) {
        ::fmt::print(args ...);
    }

#else // use internal basic implementation of format() and print() [only supports simple {} placeholders]

    template<typename ... Types>
    std::string BasicFormat(const std::string& str, Types&& ... args) {
        std::vector<std::string> items = {to_string(args)...};

        std::string output;
        size_t pos = 0;
        size_t itemIndex = 0;
        while (true) {
            size_t nextItem = str.find("{}", pos);
            if (nextItem != std::string::npos) {
                output.append(str.substr(pos, nextItem - pos));
                pos = nextItem + 2; // skip {}
                if (itemIndex < items.size()) {
                    output.append(items[itemIndex]);
                    itemIndex++;
                } else {
                    throw std::logic_error("BasicFormat: missing argument");
                }
            } else {
                if (itemIndex != items.size()) {
                    throw std::logic_error("BasicFormat: too many arguments");
                }
                output.append(str.substr(pos, str.length() - pos));
                return output;
            }
        }
        return output;
    }

    template<typename ... Types>
    std::string format(Types&& ... args) {
        return BasicFormat(args ...);
    }

    template<typename ... Types>
    void print(Types&& ... args) {
        printf("%s", format(args ...).c_str());
    }

#endif

}  // namespace fmt
}  // namespace carbon
}  // namespace epic
