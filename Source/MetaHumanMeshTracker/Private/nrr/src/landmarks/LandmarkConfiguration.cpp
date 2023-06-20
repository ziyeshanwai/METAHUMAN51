// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/landmarks/LandmarkConfiguration.h>

#include <carbon/Algorithm.h>
#include <carbon/utils/Profiler.h>

#include <algorithm>
#include <numeric>

namespace epic::nls {

bool LandmarkConfiguration::LoadConfiguration(const carbon::JsonElement& j)
{
    PROFILING_FUNCTION(PROFILING_COLOR_NAVY);

    if (!j.IsObject() || !j.Contains("frames")) {
        throw std::runtime_error("frames are missing in the landmarks file");
    }

    const int numFrames = int(j["frames"].Size());
    if (numFrames == 0) {
        throw std::runtime_error("no frames in landmarks file");
    }

    const bool newFormat = j["frames"][0].Contains("points");
    const carbon::JsonElement& jFrame = newFormat ? j["frames"][0]["points"] : j["frames"][0];

    m_landmarkMapping.clear();
    m_curvesMapping.clear();
    m_numPoints = 0;
    for (const auto& [landmarkOrCurveName, jLandmarks] : jFrame.Map()) {
        const int numPointsForLandmark = static_cast<int>(jLandmarks.Size());
        if (numPointsForLandmark == 1) {
            m_landmarkMapping[landmarkOrCurveName] = m_numPoints;
        } else {
            std::vector<int> indices(numPointsForLandmark);
            for (int i = 0; i < numPointsForLandmark; ++i) {
                indices[i] = m_numPoints + i;
            }
            m_curvesMapping[landmarkOrCurveName] = indices;
        }
        m_numPoints += numPointsForLandmark;
    }

    return true;
}

void LandmarkConfiguration::AddLandmark(const std::string& landmarkName)
{
    if (HasLandmark(landmarkName)) {
        CARBON_CRITICAL("landmark configuration already contains landmark {}", landmarkName);
    }
    m_landmarkMapping[landmarkName] = m_numPoints;
    m_numPoints++;
}

void LandmarkConfiguration::AddCurve(const std::string& curveName, int numPoints)
{
    if (HasCurve(curveName)) {
        CARBON_CRITICAL("landmark configuration already contains curve {}", curveName);
    }
    std::vector<int> indices(numPoints);
    std::iota(indices.begin(), indices.end(), m_numPoints);
    m_curvesMapping[curveName] = indices;
    m_numPoints += numPoints;
}

void LandmarkConfiguration::RemoveLandmark(const std::string& landmarkName)
{
    if (!HasLandmark(landmarkName)) {
        CARBON_CRITICAL("landmark configuration does not contain landmark {}", landmarkName);
    }
    m_landmarkMapping.erase(m_landmarkMapping.find(landmarkName));
}

void LandmarkConfiguration::RemoveCurve(const std::string& curveName)
{
    if (!HasCurve(curveName)) {
        CARBON_CRITICAL("landmark configuration does not contain curve {}", curveName);
    }
    m_curvesMapping.erase(m_curvesMapping.find(curveName));
}

int LandmarkConfiguration::IndexForLandmark(const std::string& landmarkName) const
{
    auto it = m_landmarkMapping.find(landmarkName);
    if (it != m_landmarkMapping.end()) {
        return it->second;
    } else {
        throw std::runtime_error("no landmark of name " + landmarkName + " in configuration");
    }
}


const std::vector<int>& LandmarkConfiguration::IndicesForCurve(const std::string& curveName) const
{
    auto it = m_curvesMapping.find(curveName);
    if (it != m_curvesMapping.end()) {
        return it->second;
    } else {
        throw std::runtime_error("no curve of name " + curveName + " in configuration");
    }
}

template <class T, int D>
std::map<int, int> LandmarkConfiguration::FindDuplicatesAndCreateMap(const Eigen::Matrix<T, D, -1>& pts) const
{
    if (int(pts.cols()) != m_numPoints) {
        CARBON_CRITICAL("numter of points does not match the number of points of the landmark configuration");
    }
    // find matching incides for all curves, then merge curves in the configuration by mapping original curve indices to get rid of duplicates
    std::map<std::pair<T, T>, int> pt2index;
    for (const auto& [_, index] : LandmarkMapping()) {
        pt2index[std::pair<T, T>(pts(0, index), pts(1, index))] = index; // this will remove identical pts
    }
    for (const auto& [_, indices] : CurvesMapping()) {
        for (const int index : indices) {
            pt2index[std::pair<T, T>(pts(0, index), pts(1, index))] = index; // this will remove identical pts
        }
    }
    std::map<int, int> index2index;
    for (const auto& [_, index] : LandmarkMapping()) {
        index2index[index] = pt2index[std::pair<T, T>(pts(0, index), pts(1, index))];
    }
    for (const auto& [_, indices] : CurvesMapping()) {
        for (const int index : indices) {
            index2index[index] = pt2index[std::pair<T, T>(pts(0, index), pts(1, index))];
        }
    }
    return index2index;
}

/**
 * Concatenates two vectors with undown direction along one matching end point (removing the duplicate).
 * Fails if there is no matching end point or if either vector is empty.
 */
template <class T, int D>
bool ConcatenateVectorsWithMatchingEndPointsAndUnknownDirection(
    const std::vector<int>& vector1,
    const std::vector<int>& vector2,
    const Eigen::Matrix<T, D, -1>& pts,
    std::vector<int>& mergedVector)
{
    if (vector1.empty() || vector2.empty()) {
        // no matching of vectors if any is empty
        return false;
    }

    std::vector<int> newMergedVector;
    newMergedVector.reserve(vector1.size() + vector2.size() - 1);
    bool success = false;

    static constexpr T eps = T(1e-9);
    if ((pts.col(vector2.front()) - pts.col(vector1.front())).squaredNorm() < eps) {
        newMergedVector.insert(newMergedVector.begin(), vector1.rbegin(), vector1.rend());
        newMergedVector.insert(newMergedVector.end(), vector2.begin() + 1, vector2.end());
        success = true;
    } else if ((pts.col(vector2.front()) - pts.col(vector1.back())).squaredNorm() < eps) {
        newMergedVector.insert(newMergedVector.begin(), vector1.begin(), vector1.end());
        newMergedVector.insert(newMergedVector.end(), vector2.begin() + 1, vector2.end());
        success = true;
    } else if ((pts.col(vector2.back()) - pts.col(vector1.front())).squaredNorm() < eps) {
        newMergedVector.insert(newMergedVector.begin(), vector2.begin(), vector2.end());
        newMergedVector.insert(newMergedVector.end(), vector1.begin() + 1, vector1.end());
        success = true;
    } else if ((pts.col(vector2.back()) - pts.col(vector1.back())).squaredNorm() < eps) {
        newMergedVector.insert(newMergedVector.begin(), vector2.begin(), vector2.end());
        newMergedVector.insert(newMergedVector.end(), vector1.rbegin() + 1, vector1.rend());
        success = true;
    }

    if (success) {
        mergedVector.swap(newMergedVector);
    }
    return success;
}

template <class T, int D>
bool LandmarkConfiguration::MergeCurves(const std::vector<std::string>& curveNames, const std::string& newCurveName, const Eigen::Matrix<T, D, -1>& pts, bool ignoreMissingCurves, bool removeMergedCurves)
{
    std::vector<std::string> curvesToMerge;
    for (const std::string& curveName : curveNames) {
        if (HasCurve(curveName)) {
            curvesToMerge.push_back(curveName);
        } else {
            if (!ignoreMissingCurves) {
                throw std::runtime_error("cannot merge curves as curve " + curveName + " does not exist");
            }
        }
    }

    if (HasCurve(newCurveName)) {
        throw std::runtime_error("there is a prior curve with name " + newCurveName);
    }

    if (curvesToMerge.size() < 1) {
        // if there is nothing to merge, then return false
        return false;
    }

    if (curvesToMerge.size() == 1) {
        // a single curve, just add it with the new name
        m_curvesMapping[newCurveName] = m_curvesMapping[curvesToMerge.front()];
        if (removeMergedCurves) {
            RemoveCurve(curvesToMerge.front());
        }
        return true;
    }

    // take the first curve
    std::vector<int> newCurve = m_curvesMapping[curvesToMerge.front()];

    // merge all other curves
    std::set<std::string> toProcess;
    toProcess.insert(curvesToMerge.begin() + 1, curvesToMerge.end());

    while (toProcess.size() > 0) {
        bool mergeOk = false;
        for (const std::string& candidate : toProcess) {
            // it should be possible merge at least one curve, otherwise the curves are not connected
            if (ConcatenateVectorsWithMatchingEndPointsAndUnknownDirection<T>(newCurve, m_curvesMapping[candidate], pts, newCurve)) {
                toProcess.erase(toProcess.find(candidate));
                mergeOk = true;
                break;
            }
        }
        if (!mergeOk) {
            CARBON_CRITICAL("failure to merge curves - no matching indices");
        }
    }

    m_curvesMapping[newCurveName] = newCurve;

    if (removeMergedCurves) {
        for (const std::string& mergeCurveName : curvesToMerge) {
            RemoveCurve(mergeCurveName);
        }
    }

    return true;
}


void LandmarkConfiguration::MergeConfiguration(const LandmarkConfiguration& otherConfiguration)
{
    // check that landmark and curve names are unique
    for (const auto& [landmarkName, _] : otherConfiguration.m_landmarkMapping) {
        if (HasLandmark(landmarkName) || HasCurve(landmarkName)) {
            CARBON_CRITICAL("landmark or curve with name {} already exists", landmarkName);
        }
    }

    for (const auto& [curveName, _] : otherConfiguration.m_curvesMapping) {
        if (HasLandmark(curveName) || HasCurve(curveName)) {
            CARBON_CRITICAL("landmark or curve with name {} already exists", curveName);
        }
    }

    // add mapping
    int numPointsOtherConfiguration = otherConfiguration.NumPoints();
    for (const auto& [landmarkName, landmarkIndex] : otherConfiguration.m_landmarkMapping) {
        m_landmarkMapping[landmarkName] = landmarkIndex + m_numPoints;
    }

    for (const auto& [curveName, curveIndices] : otherConfiguration.m_curvesMapping) {
        std::vector<int> newCurveIndices = curveIndices;
        std::transform(curveIndices.begin(), curveIndices.end(), newCurveIndices.begin(), [&](int id) { return id + m_numPoints; });
        m_curvesMapping[curveName] = newCurveIndices;
    }

    m_numPoints += numPointsOtherConfiguration;
}

std::vector<int> LandmarkConfiguration::RemapLandmarksAndCurvesAndCompress(const std::map<int, int>& map)
{
    std::vector<bool> used(NumPoints(), false);
    for (const auto& [_, index] : map) {
        used[index] = true;
    }
    std::vector<int> newIndex(NumPoints(), -1);
    int newSize = 0;
    for (int i = 0; i < NumPoints(); ++i) {
        if (used[i]) {
            newIndex[i] = newSize++;
        }
    }

    std::map<std::string, int> newLandmarkMapping;
    std::map<std::string, std::vector<int>> newCurvesMapping;

    for (auto && [landmarkName, index] : m_landmarkMapping) {
        auto it = map.find(index);
        if (it != map.end()) {
            newLandmarkMapping[landmarkName] = newIndex[it->second];
        }
    }

    for (auto && [curveName, indices] : m_curvesMapping) {
        std::vector<int> newCurveIndices;
        for (int& index : indices) {
            auto it = map.find(index);
            if (it != map.end()) {
                newCurveIndices.push_back(newIndex[map.find(index)->second]);
            }
        }
        if (newCurveIndices.size() > 1) {
            newCurvesMapping[curveName] = newCurveIndices;
        }
    }

    std::vector<int> newLandmarksToOldLandmarksMap(newSize);
    for (const auto& [_, index] : map) {
        newLandmarksToOldLandmarksMap[newIndex[index]] = index;
    }

    m_numPoints = newSize;
    m_landmarkMapping = std::move(newLandmarkMapping);
    m_curvesMapping = std::move(newCurvesMapping);
    return newLandmarksToOldLandmarksMap;
}

template std::map<int, int> LandmarkConfiguration::FindDuplicatesAndCreateMap<float, 2>(const Eigen::Matrix<float, 2, -1>&) const;
template std::map<int, int> LandmarkConfiguration::FindDuplicatesAndCreateMap<double, 2>(const Eigen::Matrix<double, 2, -1>&) const;
template std::map<int, int> LandmarkConfiguration::FindDuplicatesAndCreateMap<float, 3>(const Eigen::Matrix<float, 3, -1>&) const;
template std::map<int, int> LandmarkConfiguration::FindDuplicatesAndCreateMap<double, 3>(const Eigen::Matrix<double, 3, -1>&) const;

template bool LandmarkConfiguration::MergeCurves<float, 2>(const std::vector<std::string>&, const std::string&, const Eigen::Matrix<float, 2, -1>&, bool, bool);
template bool LandmarkConfiguration::MergeCurves<double, 2>(const std::vector<std::string>&, const std::string&, const Eigen::Matrix<double, 2, -1>&, bool, bool);
template bool LandmarkConfiguration::MergeCurves<float, 3>(const std::vector<std::string>&, const std::string&, const Eigen::Matrix<float, 3, -1>&, bool, bool);
template bool LandmarkConfiguration::MergeCurves<double, 3>(const std::vector<std::string>&, const std::string&, const Eigen::Matrix<double, 3, -1>&, bool, bool);

} // namespace epic::nls
