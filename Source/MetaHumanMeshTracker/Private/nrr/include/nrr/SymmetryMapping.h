// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>
#include <nls/math/Math.h>
#include <nls/utils/FileIO.h>
#include <nls/serialization/EigenSerialization.h>

#include <string>

namespace epic {
namespace nls {


/**
 * Simple helper class returning symmetry information
 */
class SymmetryMapping
{
public:
    SymmetryMapping() = default;

    bool Load(const std::string& filename)
    {
        const std::string symmetryData = ReadFile(filename);
        const carbon::JsonElement jSymmetry = carbon::ReadJson(symmetryData);
        Eigen::VectorXi symmetries;
        if (jSymmetry.Contains("symmetry")) {
            FromJson(jSymmetry["symmetry"], symmetries);
        } else {
            CARBON_CRITICAL("no symmetry data in {}", filename);
        }

        for (int i = 0; i < int(symmetries.size()); ++i) {
            int other = symmetries[i];
            if (other < 0 || other >= int(symmetries.size()) || symmetries[other] != i) {
                CARBON_CRITICAL("invalid symmetry information in {}", filename);
            }
        }

        m_symmetries = symmetries;

        return true;
    }

    int NumSymmetries() const { return int(m_symmetries.size()); }

    int Map(int vID) const { return m_symmetries[vID]; }

    bool IsSelfSymmetric(int vID) const { return m_symmetries[vID] == vID; }

private:
    Eigen::VectorXi m_symmetries;
};


} // namespace nls
} //namespace epic
