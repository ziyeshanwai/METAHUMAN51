// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>

#include <map>
#include <string>
#include <vector>

namespace epic::nls {

class ConfigurationParameter;

class Configuration {
public:
    Configuration(const std::string& configName, const std::vector<std::pair<std::string, ConfigurationParameter>>& configParameters);

    void Set(const Configuration& other, bool ignoreUnknownKeys = false);

    std::map<std::string, std::string> StringConfiguration() const;
    void SetStringConfiguration(const std::map<std::string, std::string>& config, bool ignoreUnknownKeys = false);

    bool HasParameter(const std::string& name) const;

    const ConfigurationParameter& operator[](const std::string& name) const;
    ConfigurationParameter& operator[](const std::string& name);

    const std::map<std::string, ConfigurationParameter>& Parameters() const;
    std::map<std::string, ConfigurationParameter>& Parameters();

    const std::string& Name() const;

    void AddParameter(const std::string& key, ConfigurationParameter&& value);
    void AddConfiguration(const std::string& key, const Configuration& config);
    void AddConfiguration(const Configuration& config);
    void AddConfigurations(const std::vector<Configuration>& configs);

    carbon::JsonElement ToJson() const;

    void FromJson(const carbon::JsonElement& json, std::vector<std::string>& unspecifiedKeys, std::vector<std::string>& unknownKeys);

    //! @returns the order in which the parameters were added to the configuration
    const std::vector<std::string>& ParameterOrder() { return m_paramOrder; }

private:
    void FromJson(const carbon::JsonElement& json, std::vector<std::string>& unspecifiedKeys, std::vector<std::string>& unknownKeys, const std::string& prefix);

private:
    std::string m_configName;
    std::map<std::string, ConfigurationParameter> m_configParameters;
    //! keep the order of the keys as they have been added
    std::vector<std::string> m_paramOrder;
};

} // namespace epic::nls
