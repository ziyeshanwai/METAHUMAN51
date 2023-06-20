// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/landmarks/LandmarkSequence.h>

#include <nls/utils/FileIO.h>

#include <carbon/io/JsonIO.h>

namespace epic::nls {


template <class T>
LandmarkSequence<T>::LandmarkSequence()
{
}

template <class T>
bool LandmarkSequence<T>::Load(const std::string& filename, int frameOffset)
{
    PROFILING_FUNCTION(PROFILING_COLOR_NAVY);

    const std::string filedata = ReadFile(filename);
    carbon::JsonElement json = carbon::ReadJson(filedata);

    if (!json.IsObject() || !json.Contains("frames")) {
        CARBON_CRITICAL("frames are missing in the landmarks file");
    }

    const int numFrames = int(json["frames"].Size());

    m_landmarkInstances.clear();
    for (int i = 0; i < numFrames; i++) {
        std::shared_ptr<LandmarkInstance<T, 2>> landmarkInstance = std::make_shared<LandmarkInstance<T, 2>>();
        if (!landmarkInstance->LoadFromJson(json["frames"][i])) {
            LOG_ERROR("failed to load landmarks from json file");
            return false;
        }
        if (frameOffset >= 0) {
            m_landmarkInstances[frameOffset + i] = landmarkInstance;
            if (landmarkInstance->FrameNumber() != frameOffset + i) {
                LOG_WARNING("frame number does not match: {} vs {}", frameOffset + i, landmarkInstance->FrameNumber());
            }
        } else {
            m_landmarkInstances[landmarkInstance->FrameNumber()] = landmarkInstance;
        }
    }

    return true;
}

template <class T>
void LandmarkSequence<T>::Save(const std::string& filename) const
{
    carbon::JsonElement frames(carbon::JsonElement::JsonType::Array);
    for (const auto& [_, landmarkInstance] : m_landmarkInstances) {
        frames.Append(landmarkInstance->SaveToJson());
    }
    carbon::JsonElement meta(carbon::JsonElement::JsonType::Object);
    meta.Insert("type", carbon::JsonElement("points"));
    meta.Insert("version", carbon::JsonElement(1));

    carbon::JsonElement json(carbon::JsonElement::JsonType::Object);
    json.Insert("metadata", std::move(meta));
    json.Insert("frames", std::move(frames));

    WriteFile(filename, carbon::WriteJson(json, 1));
}

template <class T>
bool LandmarkSequence<T>::HasLandmarks(int frame) const {
    return (m_landmarkInstances.find(frame) != m_landmarkInstances.end());
}

template <class T>
const LandmarkInstance<T, 2>& LandmarkSequence<T>::Landmarks(int frame) const {
    auto it = m_landmarkInstances.find(frame);
    if (it != m_landmarkInstances.end()) {
        return *(it->second);
    } else {
        throw std::runtime_error("no landmarks for frame " + std::to_string(frame));
    }
}

template <class T>
std::shared_ptr<const LandmarkInstance<T, 2>> LandmarkSequence<T>::LandmarksPtr(int frame) const {
    auto it = m_landmarkInstances.find(frame);
    if (it != m_landmarkInstances.end()) {
        return it->second;
    } else {
        return nullptr;
    }
}

template <class T>
void LandmarkSequence<T>::Undistort(const MetaShapeCamera<T>& camera)
{
    for (auto && [_, landmarkInstance] : m_landmarkInstances) {
        for (int i = 0; i < landmarkInstance->NumLandmarks(); ++i) {
            const T confidence = landmarkInstance->Confidence()[i];
            const Eigen::Vector2<T> pix = camera.Undistort(landmarkInstance->Points().col(i));
            landmarkInstance->SetLandmark(i, pix, confidence);
        }
    }
}

template <class T>
bool LandmarkSequence<T>::MergeCurves(const std::vector<std::string>& curveNames, const std::string& newCurveName, bool ignoreMissingCurves, bool removeMergedCurves)
{
    bool ok = true;
    for (const auto& [_, landmarkInstance] : m_landmarkInstances) {
        ok &= landmarkInstance->MergeCurves(curveNames, newCurveName, ignoreMissingCurves, removeMergedCurves);
    }
    return ok;
}

template <class T>
void LandmarkSequence<T>::MergeSequences(const LandmarkSequence& otherSequence)
{
    if (otherSequence.m_landmarkInstances.size() != m_landmarkInstances.size()) {
        CARBON_CRITICAL("landmark sequences do not have the same size");
    }

    for (const auto& [frame, _] : otherSequence.m_landmarkInstances) {
        if (m_landmarkInstances.find(frame) == m_landmarkInstances.end()) {
            CARBON_CRITICAL("landmarks sequences do not have the same frame numbers");
        }
    }

    // merge instances
    for (const auto& [frame, landmarkInstance] : otherSequence.m_landmarkInstances) {
        auto it = m_landmarkInstances.find(frame);
        it->second->MergeInstance(*landmarkInstance);
    }
}

template <class T>
std::map<int, std::shared_ptr<const LandmarkInstance<T, 2>>> LandmarkSequence<T>::LandmarkInstances() const
{
    std::map<int, std::shared_ptr<const LandmarkInstance<T, 2>>> landmarkInstances;
    for (const auto& [key, value] : m_landmarkInstances) {
        landmarkInstances.emplace(key, value);
    }
    return landmarkInstances;
}

template <class T>
void LandmarkSequence<T>::SetLandmarkInstances(const std::map<int, std::shared_ptr<LandmarkInstance<T, 2>>>& landmarkInstances)
{
    m_landmarkInstances.clear();
    for (const auto& [key, value] : landmarkInstances) {
        m_landmarkInstances.emplace(key, value);
    }
}

template <class T>
void LandmarkSequence<T>::SetLandmarkInstance(int frame, std::shared_ptr<LandmarkInstance<T, 2>>& landmarkInstance)
{
    m_landmarkInstances[frame] = landmarkInstance;
}


// explicitly instantiate the landmark sequence classes
template class LandmarkSequence<float>;
template class LandmarkSequence<double>;

} // namespace epic::nls
