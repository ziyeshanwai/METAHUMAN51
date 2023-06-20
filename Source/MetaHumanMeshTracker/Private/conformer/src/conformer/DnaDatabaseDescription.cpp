// Copyright Epic Games, Inc. All Rights Reserved.

#include <conformer/DnaDatabaseDescription.h>

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>
#include <nls/utils/FileIO.h>

#include <filesystem>

using namespace epic;

bool DnaDatabaseDescription::Load(const std::string& filename)
{
    try {
        const std::string filedata = nls::ReadFile(filename);
        const carbon::JsonElement j = carbon::ReadJson(filedata);

        const std::string descriptionDirectory = std::filesystem::absolute(std::filesystem::path(filename)).parent_path().string();

        auto makeAbsolute = [&](const std::string& filename) {
            if (std::filesystem::path(filename).is_relative()) {
                return descriptionDirectory + "/" + filename;
            }
            else {
                return filename;
            }
        };

        if (j.Contains("pca_identity_model")) {
            m_pcaIdentityModelFilename = makeAbsolute(j["pca_identity_model"].String());
        }
        else {
            LOG_WARNING("DNA Database description does not containt pca_identity model.");
        }
        if (j.Contains("blend_identity_model")) {
            m_regionIdentityModelFilename = makeAbsolute(j["blend_identity_model"].String());
        }
        else {
            LOG_WARNING("DNA Database description does not contain blend_identity model.");
        }
        if (j.Contains("dna_database_folder")) {
            m_dnaDatabaseFolder = makeAbsolute(j["dna_database_folder"].String());
        }
        else {
            LOG_WARNING("DNA Database description does not contain DNA database folder.");
        }
        if (j.Contains("archetype_dna")) {
            m_archetypeDna = makeAbsolute(j["archetype_dna"].String());
        }
        else {
            LOG_WARNING("DNA Database description does not contain archetype DNA.");
        }
        if (j.Contains("region_affiliation_file")) {
            m_rafFile = makeAbsolute(j["region_affiliation_file"].String());
        }
        if (j.Contains("assets_identity_models")) {
            for (const auto& [key, value] : j["assets_identity_models"].Map()) {
                m_assetsIdentityModels[key] = makeAbsolute(value.String());
            }
        }
        else {
            LOG_WARNING("DNA Database description does not contain identity models for assets.");
        }
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("failure to load template description {}: {}", filename, e.what());
        return false;
    }

    return true;
}

const std::string& DnaDatabaseDescription::AssetIdentityModel(const std::string& assetName) const
{
    auto it = m_assetsIdentityModels.find(assetName);
    if (it != m_assetsIdentityModels.end()) {
        return it->second;
    }
    else {
        CARBON_CRITICAL("no data description for expression {}", assetName);
    }
}
