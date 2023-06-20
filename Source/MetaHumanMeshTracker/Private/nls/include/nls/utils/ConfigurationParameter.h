// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/utils/Configuration.h>

#include <string>
#include <stdexcept>
#include <typeinfo>
#include <variant>

namespace epic::nls {

template<class T> struct always_false : std::false_type {};

namespace detail {
    typedef std::variant<bool, int, float, double, std::string, Configuration> ConfigurationParameterVariant;

    template <typename T> inline bool IsType(const ConfigurationParameterVariant& value) { return std::holds_alternative<T>(value); }

    template <typename T> inline T& Value(ConfigurationParameterVariant& value) {
        if (std::holds_alternative<T>(value)) { return std::get<T>(value); }
        else CARBON_CRITICAL("Not a config parameter of type {}", typeid(T).name());
    }

    template <typename T> inline const T& Value(const ConfigurationParameterVariant& value) {
        if (std::holds_alternative<T>(value)) { return std::get<T>(value); }
        else CARBON_CRITICAL("Not a config parameter of type {}", typeid(T).name());
    }

    inline std::string AsString(const ConfigurationParameterVariant& value) {
        return std::visit([](auto&& arg) {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, int>)
                return std::to_string(arg);
            else if constexpr (std::is_same_v<T, float>)
                return std::to_string(arg);
            else if constexpr (std::is_same_v<T, double>)
                return std::to_string(arg);
            else if constexpr (std::is_same_v<T, bool>)
                return std::string(arg ? "True" : "False");
            else if constexpr (std::is_same_v<T, std::string>)
                return arg;
            else if constexpr (std::is_same_v<T, Configuration>) {
                static_assert("cannot retrieve sub configuration as string");
                return std::string();
            }
            else {
                static_assert(always_false<T>::value, "non-exhaustive visitor!");
                return std::string();
            }
        }, value);
    }
}

class ConfigurationParameter {
public:
    ConfigurationParameter() : m_value(int(0)), m_rangeStart(int(0)), m_rangeEnd(int(0))
    {
        CARBON_CRITICAL("Creating config parameter of invalid type");
    }

    explicit ConfigurationParameter(bool value) : m_value(value), m_rangeStart(int(0)), m_rangeEnd(int(0))
    {
    }

    explicit ConfigurationParameter(int value, int rangeStart = 0, int rangeEnd = 1) : m_value(value), m_rangeStart(rangeStart), m_rangeEnd(rangeEnd)
    {
    }

    explicit ConfigurationParameter(float value, float rangeStart = 0.0f, float rangeEnd = 1.0f) : m_value(value), m_rangeStart(rangeStart), m_rangeEnd(rangeEnd)
    {
    }

    explicit ConfigurationParameter(double value, double rangeStart = 0.0, double rangeEnd = 1.0) : m_value(value), m_rangeStart(rangeStart), m_rangeEnd(rangeEnd)
    {
    }

    explicit ConfigurationParameter(const std::string& value) : m_value(value), m_rangeStart(int(0)), m_rangeEnd(int(0))
    {
    }

    explicit ConfigurationParameter(const char* value) : ConfigurationParameter(std::string(value))
    {
    }

    explicit ConfigurationParameter(Configuration value) : m_value(value), m_rangeStart(int(0)), m_rangeEnd(int(0))
    {
    }

    void Set(const std::string& value)
    {
        if (std::holds_alternative<bool>(m_value)) {
            if (value == "True" || value == "true" || value == "Yes" || value == "yes") {
                m_value = true;
            } else if (value == "False" || value == "false" || value == "No" || value == "no") {
                m_value = false;
            } else {
                CARBON_CRITICAL("Value cannot be converted to bool");
            }
        } else if (std::holds_alternative<int>(m_value)) {
            // conversion - throws exception if it is not valid
            m_value = std::stoi(value);
        } else if (std::holds_alternative<double>(m_value)) {
            // conversion - throws exception if it is not valid
            m_value = std::stod(value);
        } else if (std::holds_alternative<std::string>(m_value)) {
            m_value = value;
        }
    }

    void Set(float value)
    {
        if (std::holds_alternative<float>(m_value)) { m_value = float(value); }
        else CARBON_CRITICAL("Not a config parameter of type float");
    }

    void Set(double value)
    {
        if (std::holds_alternative<double>(m_value)) { m_value = value; }
        else CARBON_CRITICAL("Not a config parameter of type double");

    }

    void Set(int value)
    {
        if (std::holds_alternative<int>(m_value)) { m_value = value; }
        else CARBON_CRITICAL("Not a config parameter of type integer");
    }

    void Set(bool value)
    {
        if (std::holds_alternative<bool>(m_value)) { m_value = value; }
        else CARBON_CRITICAL("Not a config parameter of type bool");
    }

    template <class T>
    /* */ T& Value() /* */ { return detail::Value<T>(m_value); }
    template <class T>
    const T& Value() const { return detail::Value<T>(m_value); }

    template <class T>
    operator /* */ T&() /* */ { return detail::Value<T>(m_value); }
    template <class T>
    operator const T&() const { return detail::Value<T>(m_value); }

#if !defined(__APPLE__) && defined(__unix__)
    template <class T>
    operator T() const { return detail::Value<T>(m_value); }
#endif

    template <class T>
    ConfigurationParameter& operator=(const T& v) { Set(v); return *this; }

    //! @returns String representation of the underlying value. It does not support the Configuration type.
    std::string AsString() const { return detail::AsString(m_value); }

    template <class T>
    std::pair<T, T> Range() const { return std::pair<T, T>(std::get<T>(m_rangeStart), std::get<T>(m_rangeEnd)); }

    template <class T>
    bool IsType() const { return detail::IsType<T>(m_value); }

    ConfigurationParameter& operator[](const std::string& name)
    {
        if (std::holds_alternative<Configuration>(m_value)) { return std::get<Configuration>(m_value)[name]; }
        else CARBON_CRITICAL("Not a config parameter of type Configuration - cannot access an element with name \"{}\".", name);
    }

    const ConfigurationParameter& operator[] (const std::string& name) const
    {
        if (std::holds_alternative<Configuration>(m_value)) { return std::get<Configuration>(m_value)[name]; }
        else CARBON_CRITICAL("Not a config parameter of type Configuration - cannot access an element with name \"{}\".", name);
    }

    ConfigurationParameter& operator[](const char* name)
    {
        if (std::holds_alternative<Configuration>(m_value)) { return std::get<Configuration>(m_value)[name]; }
        else CARBON_CRITICAL("Not a config parameter of type Configuration - cannot access an element with name \"{}\".", name);
    }

    const ConfigurationParameter& operator[] (const char* name) const
    {
        if (std::holds_alternative<Configuration>(m_value)) { return std::get<Configuration>(m_value)[name]; }
        else CARBON_CRITICAL("Not a config parameter of type Configuration - cannot access an element with name \"{}\".", name);
    }

private:
    //! the parameter name
    std::string m_name;

    //! the parameter value
    detail::ConfigurationParameterVariant m_value;

    //! the range of the parameter value in [m_rangeStart, m_rangeEnd]. Note that the end of the range is included.
    std::variant<int, float, double> m_rangeStart;
    std::variant<int, float, double> m_rangeEnd;
};

} // namespace epic::nls
