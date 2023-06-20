// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <string>
#include <map>

class DnaDatabaseDescription
{
public:
    DnaDatabaseDescription() = default;

    bool Load(const std::string& filename);

    const std::string& PcaIdentityModelFilename() const { return m_pcaIdentityModelFilename; }
    const std::string& RegionBlendIdentityModelFilename() const { return m_regionIdentityModelFilename; }
    const std::string& DnaDatabaseFolder() const { return m_dnaDatabaseFolder; }
    const std::string& ArchetypeDnaFilename() const { return m_archetypeDna; }
    const std::string& RafFilename() const { return m_rafFile; }
    const std::string& AssetIdentityModel(const std::string& assetName) const;

private:

    // PCA identity model
    std::string m_pcaIdentityModelFilename;

    // region blend identity model
    std::string m_regionIdentityModelFilename;

    // DNA database folder
    std::string m_dnaDatabaseFolder;

    // DNA archetype file
    std::string m_archetypeDna;

    // Region affiliation filename
    std::string m_rafFile;

    // Assets PCA identity models
    std::map<std::string, std::string> m_assetsIdentityModels;

};
