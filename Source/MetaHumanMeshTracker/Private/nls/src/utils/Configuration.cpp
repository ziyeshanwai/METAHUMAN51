// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/utils/Configuration.h>
#include <nls/utils/ConfigurationParameter.h>

#include <algorithm>

namespace epic::nls {

Configuration::Configuration(const std::string& configName, const std::vector<std::pair<std::string, ConfigurationParameter>>& configParameters)
    : m_configName(configName)
    , m_configParameters(configParameters.begin(), configParameters.end())
{
    std::transform(configParameters.begin(), configParameters.end(), std::back_inserter(m_paramOrder), [](const auto& pair) -> std::string { return pair.first; });
}

void Configuration::Set(const Configuration& other, bool ignoreUnknownKeys)
{
    if (other.m_configName != m_configName) {
        CARBON_CRITICAL("configuration names do not match: {} vs {}", m_configName, other.m_configName);
    }
    for (auto&& [key, value] : other.m_configParameters) {
        if (m_configParameters.find(key) != m_configParameters.end()) {
            m_configParameters[key] = value;
        } else {
            if (!ignoreUnknownKeys) {
                CARBON_CRITICAL("key {} is not part of configuration {}", key, m_configName);
            } else {
                LOG_WARNING("key {} is not part of configuration {}", key, m_configName);
            }
        }
    }
    for (auto&& [key, value] : m_configParameters) {
        if (other.m_configParameters.find(key) == other.m_configParameters.end()) {
            LOG_WARNING("key {} is not set by input for configuration {}", key, m_configName);
        }
    }
}

std::map<std::string, std::string> Configuration::StringConfiguration() const
{
    std::map<std::string, std::string> dict;
    for (auto && [key, value] : m_configParameters) {
        dict[key] = value.AsString();
    }
    return dict;
}

void Configuration::SetStringConfiguration(const std::map<std::string, std::string>& config, bool ignoreUnknownKeys)
{
    for (auto&& [key, value] : config) {
        if (m_configParameters.find(key) != m_configParameters.end()) {
            m_configParameters[key].Set(value);
        } else if (!ignoreUnknownKeys) {
            CARBON_CRITICAL("key {} is not part of configuration {}", key, m_configName);
        }
    }
}

bool Configuration::HasParameter(const std::string& name) const { return (m_configParameters.find(name) != m_configParameters.end()); }

const ConfigurationParameter& Configuration::operator[](const std::string& name) const
{
    auto it = m_configParameters.find(name);
    if (it != m_configParameters.end()) {
        return it->second;
    }
    CARBON_CRITICAL("{} is not part of the configuration {}", name, m_configName);
}

ConfigurationParameter& Configuration::operator[](const std::string& name)
{
    auto it = m_configParameters.find(name);
    if (it != m_configParameters.end()) {
        return it->second;
    }
    CARBON_CRITICAL("{} is not part of the configuration {}", name, m_configName);
}

const std::map<std::string, ConfigurationParameter>& Configuration::Parameters() const { return m_configParameters; }
std::map<std::string, ConfigurationParameter>& Configuration::Parameters() { return m_configParameters; }

const std::string& Configuration::Name() const { return m_configName; }

void Configuration::AddParameter(const std::string& key, ConfigurationParameter&& value)
{
    if (HasParameter(key)) {
        CARBON_CRITICAL("config already contains a parameter with key '{}'", key);
    }
    m_configParameters.emplace(key, value);
    m_paramOrder.emplace_back(key);
}

void Configuration::AddConfiguration(const std::string& key, const Configuration& config)
{
    if (HasParameter(key))
    {
        CARBON_CRITICAL("config already contains a parameter with key '{}'", key);
    }
    m_configParameters.emplace(key, ConfigurationParameter(config));
    m_paramOrder.emplace_back(key);
}

void Configuration::AddConfiguration(const Configuration& config)
{
    AddConfiguration(config.Name(), config);
}

void Configuration::AddConfigurations(const std::vector<Configuration>& configs)
{
    for (const auto& config : configs) {
        AddConfiguration(config);
    }
}

carbon::JsonElement Configuration::ToJson() const {
    carbon::JsonElement json(carbon::JsonElement::JsonType::Object);

    // set the json type based on the configuration parameter type
    auto toJsonElement = [](const ConfigurationParameter& param) {
        if (param.IsType<bool>()) {
            return carbon::JsonElement(param.Value<bool>());
        } else if (param.IsType<int>()) {
            return carbon::JsonElement(param.Value<int>());
        } else if (param.IsType<double>()) {
            return carbon::JsonElement(param.Value<double>());
        } else if (param.IsType<float>()) {
            return carbon::JsonElement(param.Value<float>());
        } else if (param.IsType<Configuration>()) {
            return param.Value<Configuration>().ToJson();
        } else {
            return carbon::JsonElement(param.AsString());
        }
    };

    for (const auto& [paramName, param] : Parameters()) {
        json.Insert(paramName, toJsonElement(param));
    }

    return json;
}

void Configuration::FromJson(const carbon::JsonElement& json, std::vector<std::string>& unspecifiedKeys, std::vector<std::string>& unknownKeys)
{
    FromJson(json, unspecifiedKeys, unknownKeys, "");
}

void Configuration::FromJson(const carbon::JsonElement& json, std::vector<std::string>& unspecifiedKeys, std::vector<std::string>& unknownKeys, const std::string& prefix)
{
    for (auto&& [key, value] : json.Object()) {
        auto it = Parameters().find(key);
        if (it != Parameters().end()) {
            ConfigurationParameter& param = it->second;
            switch (value.Type()) {
                case carbon::JsonElement::JsonType::True: param.Set(true); break;
                case carbon::JsonElement::JsonType::False: param.Set(false); break;
                case carbon::JsonElement::JsonType::String: param.Set(value.String()); break;
                case carbon::JsonElement::JsonType::Int: {
                    if (param.IsType<double>()) {
                        param.Set(value.Value<double>());
                    }
                    else if (param.IsType<float>()) {
                        param.Set(value.Value<float>());
                    }
                    else {
                        param.Set(value.Value<int>());
                    }
                } break;
                case carbon::JsonElement::JsonType::Double: {
                    if (param.IsType<double>()) {
                        param.Set(value.Value<double>());
                    }
                    else if (param.IsType<float>()) {
                        param.Set(value.Value<float>());
                    } else {
                        CARBON_CRITICAL("json double can only be assigned to float/double configuration parameters");
                    }
                } break;
                case carbon::JsonElement::JsonType::Object: {
                    if (param.IsType<Configuration>()) {
                        param.Value<Configuration>().FromJson(value, unspecifiedKeys, unknownKeys, prefix + ":" + key);
                    } else {
                        CARBON_CRITICAL("json objects can only be assigned to configurations");
                    }
                } break;
                case carbon::JsonElement::JsonType::Null:
                case carbon::JsonElement::JsonType::Array: {
                    CARBON_CRITICAL("configuration parameter does not support json type {}", value.Type());
                }
            }
        } else {
            unknownKeys.push_back(prefix + ":" + key);
        }
    }

    for (auto&& [key, value] : Parameters()) {
        auto it = json.Object().find(key);
        if (it == json.Object().end()) {
            unspecifiedKeys.push_back(prefix + ":" + key);
        }
    }
}

} // namespace epic::nls
