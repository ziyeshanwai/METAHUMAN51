// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>
#include <nls/utils/FileIO.h>

#include <string>
#include <vector>

namespace epic {
namespace nls {

/*
{
    "stereo": ["tri_075_mv", "tri_077_mv"],
    or for multiple pairs:
    "stereo": [["tri_075_mv", "tri_077_mv"], [...]]
    "range": [0, 2]
}
*/
class ReconstructionDescription {
public:
    ReconstructionDescription() = default;

    bool Load(const std::string& filename)
    {
        const std::string filedata = ReadFile(filename);
        const carbon::JsonElement j = carbon::ReadJson(filedata);

        // parse stereo
        if (!j.Contains("stereo")) {
            LOG_ERROR("no stereo in json file {}", filename);
            return false;
        }
        if (!j["stereo"].IsArray()) {
            LOG_ERROR("stereo key needs to point to a list of stereo pairs or a single stereo pair");
            return false;
        }
        if (j["stereo"].Size() == 0) {
            LOG_ERROR("stereo key needs to contain at least one stereo pair");
            return false;
        }
        bool ok = true;
        m_stereoPairs.clear();
        auto checkAndExtractStereoPair = [&](const carbon::JsonElement& j) {
            if (j.IsArray() && j.Size() == 2 && j[0].IsString() && j[1].IsString()) {
                m_stereoPairs.push_back(j.Get<std::pair<std::string, std::string>>());
                return true;
            }
            return false;
        };
        if (j["stereo"][0].IsArray()) {
            for (size_t k = 0; k < j["stereo"].Size(); ++k) {
                ok &= checkAndExtractStereoPair(j["stereo"][k]);
            }
        } else {
            ok &= checkAndExtractStereoPair(j["stereo"]);
        }
        if (!ok) {
            LOG_ERROR("stereo key does not point to a valid list of stereo pairs or a single stereo pair");
            return false;
        }

        if (!j.Contains("range")) {
            LOG_ERROR("no range in json file %s\n", filename);
            return false;
        }

        m_reconstructionRange.first = j["range"][0].Get<int>();
        m_reconstructionRange.second = j["range"][1].Get<int>();

        return true;
    }

    const size_t NumStereoPairs() const { return m_stereoPairs.size(); }

    const std::pair<std::string, std::string> StereoPair(size_t index = 0) const { return m_stereoPairs[index]; }

    const std::pair<int, int> ReconstructionRange() const { return m_reconstructionRange; }

private:
    std::vector<std::pair<std::string, std::string>> m_stereoPairs;
    std::pair<int, int> m_reconstructionRange;


};


} // namespace nls
} //namespace epic
