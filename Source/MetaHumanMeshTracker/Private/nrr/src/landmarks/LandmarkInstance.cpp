// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/landmarks/LandmarkInstance.h>
#include <nls/geometry/CatmullRom.h>
#include <set>

namespace epic::nls {

template <class T, int D>
Eigen::Matrix<T, D, -1> CalculateSimplifiedCurve(const Eigen::Matrix<T, D, -1>& curve, const std::vector<int>& indices, T eps) {
    std::set<int> selectedIndices(indices.begin(), indices.end());
    selectedIndices.insert(0);
    selectedIndices.insert(static_cast<int>(curve.cols()) - 1);

    while (true) {
        Eigen::Matrix<T, D, -1> newControlPoints(D, selectedIndices.size());
        int count = 0;
        for (int index : selectedIndices) {
            newControlPoints.col(count++) = curve.col(index);
        }
        CatmullRom<T, D> catmull(newControlPoints, 5,  /*closed=*/ false);
        if (catmull.SampledPoints().NumControlPoints() > 1) {
            int nextIndex = -1;
            T maxDist = 0;
            for (int k = 0; k < curve.cols(); ++k) {
                int closestSegment;
                T lambda;
                const T dist = catmull.SampledPoints().ClosestPoint(curve.col(k), closestSegment, lambda);
                if (dist > maxDist) {
                    nextIndex = k;
                    maxDist = dist;
                }
            }
            if ((maxDist > eps) && (nextIndex >= 0) && (selectedIndices.find(nextIndex) == selectedIndices.end())) {
                selectedIndices.insert(nextIndex);
            }
            else {
                break;
            }
        }
        else {
            int nextIndex = -1;
            T maxDist = 0;
            for (int k = 0; k < curve.cols(); ++k) {
                const T dist = (curve.col(k) - catmull.SampledPoints().ControlPoints().col(0)).squaredNorm();
                if (dist > maxDist) {
                    nextIndex = k;
                }
            }
            if ((nextIndex >= 0) && (selectedIndices.find(nextIndex) == selectedIndices.end())) {
                selectedIndices.insert(nextIndex);
            }
            else {
                break;
            }
        }
    }

    Eigen::Matrix<T, D, -1> newControlPoints(D, selectedIndices.size());
    int count = 0;
    for (int index : selectedIndices) {
        newControlPoints.col(count++) = curve.col(index);
    }
    return newControlPoints;
}

template <class T, int D>
Eigen::Matrix<T, D, -1> LandmarkInstance<T, D>::Points(const std::vector<int>& indices) const
{
    Eigen::Matrix<T, D, -1> pts(D, indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        pts.col(i) = m_points.col(indices[i]);
    }
    return pts;
}

template <class T, int D>
Eigen::Vector<T, -1> LandmarkInstance<T, D>::Confidences(const std::vector<int>& indices) const
{
    Eigen::Vector<T, 1> conf(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        conf[i] = m_confidence[indices[i]];
    }
    return conf;
}


template <class T, int D>
std::shared_ptr<const LandmarkInstance<T, D>> LandmarkInstance<T, D>::CreateLandmarkInstance(const std::map<std::string, Eigen::Matrix<T, D, -1>>& namedLandmarkData,
                                                                                       const std::map<std::string, std::vector<std::string>>& curvesToMerge)
{
    std::shared_ptr<LandmarkConfiguration> landmarkConfiguration = std::make_shared<LandmarkConfiguration>();
    for (const auto& [landmarkOrCurveName, landmarkData] : namedLandmarkData) {
        if (landmarkData.cols() == 1) {
            landmarkConfiguration->AddLandmark(landmarkOrCurveName);
        } else if (landmarkData.cols() > 1) {
            landmarkConfiguration->AddCurve(landmarkOrCurveName, static_cast<int>(landmarkData.cols()));
        } else {
            CARBON_CRITICAL("at least one point per landmark/curve required");
        }
    }

    Eigen::Matrix<T, D, -1> landmarks(D, landmarkConfiguration->NumPoints());
    Eigen::Vector<T, -1> confidence(landmarkConfiguration->NumPoints());
    for (const auto& [landmarkOrCurveName, landmarkData] : namedLandmarkData) {
        if (landmarkData.cols() == 1) {
            int index = landmarkConfiguration->IndexForLandmark(landmarkOrCurveName);
            landmarks.col(index) = landmarkData.col(0);
            confidence[index] = T(1);
        } else {
            const std::vector<int>& indices = landmarkConfiguration->IndicesForCurve(landmarkOrCurveName);
            for (int32_t i = 0; i < landmarkData.cols(); ++i) {
                int index = indices[i];
                landmarks.col(index) = landmarkData.col(i);
                confidence[index] = T(1);
            }
        }
    }
    for (const auto& [mergedCurve, listOfCurves] : curvesToMerge) {
        landmarkConfiguration->MergeCurves<T,D>(listOfCurves, mergedCurve, landmarks, /*ignoreMissingCurves=*/true);
    }

    std::shared_ptr<LandmarkInstance<T, D>> landmarkInstance = std::make_shared<LandmarkInstance<T, D>>(landmarks, confidence);
    landmarkInstance->SetLandmarkConfiguration(landmarkConfiguration);
    return landmarkInstance;
}

template <class T, int D>
void LandmarkInstance<T, D>::MergeInstance(const LandmarkInstance& other)
{
    auto newLandmarkConfiguration = std::make_shared<LandmarkConfiguration>(*m_landmarkConfiguration);
    newLandmarkConfiguration->MergeConfiguration(*other.GetLandmarkConfiguration());
    m_landmarkConfiguration = newLandmarkConfiguration;

    Eigen::Matrix<T, D, -1> newPoints(D, m_points.cols() + other.m_points.cols());
    newPoints << m_points, other.m_points;
    Eigen::Vector<T, -1> newConfidences(m_confidence.size() + other.m_confidence.size());
    newConfidences << m_confidence, other.m_confidence;
    std::swap(m_points, newPoints);
    std::swap(m_confidence, newConfidences);
}

template <class T, int D>
bool LandmarkInstance<T, D>::MergeCurves(const std::vector<std::string>& curveNames, const std::string& newCurveName, bool ignoreMissingCurves, bool removeMergedCurves)
{
    auto newLandmarkConfiguration = std::make_shared<LandmarkConfiguration>(*m_landmarkConfiguration);
    bool ok = newLandmarkConfiguration->template MergeCurves<T,D>(curveNames, newCurveName, m_points, ignoreMissingCurves, removeMergedCurves);
    m_landmarkConfiguration = newLandmarkConfiguration;
    return ok;
}

template <class T, int D>
void LandmarkInstance<T, D>::SimplifyCurve(const std::string& curveName, T eps) {
    auto newLandmarkConfiguration = std::make_shared<LandmarkConfiguration>(*m_landmarkConfiguration);

    const auto& globalIndices = newLandmarkConfiguration->IndicesForCurve(curveName);
    std::vector<int> fixedIndices;
    for (int localIndex = 0; localIndex < int(globalIndices.size()); ++localIndex) {
        bool indexFound = false;
        for (const auto& [_, landmarkIndex] : m_landmarkConfiguration->LandmarkMapping())
        {
            if (landmarkIndex == globalIndices[localIndex]) {
                fixedIndices.push_back(localIndex);
                indexFound = true;
                break;
            }
        }

        if (indexFound) {
            continue;
        }

        for (const auto& [currentCurveName, landmarkIndices] : m_landmarkConfiguration->CurvesMapping())
        {
            if (curveName == currentCurveName) {
                continue;
            }
            for (const auto curveLandmarkIndex : landmarkIndices) {
                if (curveLandmarkIndex == globalIndices[localIndex]) {
                    fixedIndices.push_back(localIndex);
                    indexFound = true;
                    break;
                }
            }
            if (indexFound) {
                continue;
            }
        }
    }

    const auto& updatedPixs = epic::nls::CalculateSimplifiedCurve<T>(Points(globalIndices), fixedIndices, eps);

    // remove current curve
    newLandmarkConfiguration->RemoveCurve(curveName);
    SetLandmarkConfiguration(newLandmarkConfiguration);
    RemoveDuplicatesAndCompress();

    // add points and setup new, simplified curve
    newLandmarkConfiguration = std::make_shared<LandmarkConfiguration>(*m_landmarkConfiguration);
    newLandmarkConfiguration->AddCurve(curveName, static_cast<int>(updatedPixs.cols()));
    Eigen::Matrix<T, D, -1> currPts(D, newLandmarkConfiguration->NumPoints());
    Eigen::VectorX<T> currConfidence(newLandmarkConfiguration->NumPoints());
    currPts.block(0, 0, D, newLandmarkConfiguration->NumPoints() - updatedPixs.cols()) = m_points;
    currConfidence.head(newLandmarkConfiguration->NumPoints() - updatedPixs.cols()) = m_confidence;
    currPts.block(0, newLandmarkConfiguration->NumPoints() - updatedPixs.cols(), D, updatedPixs.cols()) = updatedPixs;
    currConfidence.segment(newLandmarkConfiguration->NumPoints() - updatedPixs.cols(), updatedPixs.cols()).setConstant(T(1));

    m_points = currPts;
    m_confidence = currConfidence;
    m_landmarkConfiguration = newLandmarkConfiguration;

    RemoveDuplicatesAndCompress();
}

template <class T, int D>
void LandmarkInstance<T, D>::SimplifyCurves(T eps) {

    for (const auto& [curveName, _] : m_landmarkConfiguration->CurvesMapping()) {
        SimplifyCurve(curveName, eps);
    }
}

template <class T, int D>
void LandmarkInstance<T, D>::RemoveDuplicatesAndCompress()
{
    if (!m_landmarkConfiguration) {
        CARBON_CRITICAL("no landmark configuration set");
    }

    std::shared_ptr<LandmarkConfiguration> config = std::make_shared<LandmarkConfiguration>(*m_landmarkConfiguration);
    const std::map<int, int> map = config->FindDuplicatesAndCreateMap<T,D>(Points());
    std::vector<int> newLandmarksToOldLandmarksMap = config->RemapLandmarksAndCurvesAndCompress(map);
    Eigen::Matrix<T, D, -1> newPoints(D, newLandmarksToOldLandmarksMap.size());
    Eigen::VectorX<T> newConfidence(newLandmarksToOldLandmarksMap.size());
    for (size_t i = 0; i < newLandmarksToOldLandmarksMap.size(); ++i) {
        newPoints.col(i) = m_points.col(newLandmarksToOldLandmarksMap[i]);
        newConfidence(i) = m_confidence[newLandmarksToOldLandmarksMap[i]];
    }
    m_points = newPoints;
    m_confidence = newConfidence;
    m_landmarkConfiguration = config;
}

template <class T, int D>
void LandmarkInstance<T, D>::RemoveLandmarkOrCurvePoint(int landmark)
{
    if (!m_landmarkConfiguration) {
        CARBON_CRITICAL("no landmark configuration set");
    }

    std::shared_ptr<LandmarkConfiguration> config = std::make_shared<LandmarkConfiguration>(*m_landmarkConfiguration);
    std::map<int, int> remappedPoints;
    for (int i = 0; i < NumLandmarks(); ++i) {
        if (i < landmark) remappedPoints[i] = i;
        else if (i > landmark) remappedPoints[i] = i;
    }
    std::vector<int> newLandmarksToOldLandmarksMap = config->RemapLandmarksAndCurvesAndCompress(remappedPoints);
    Eigen::Matrix<T, D, -1> newPoints(D, newLandmarksToOldLandmarksMap.size());
    Eigen::VectorX<T> newConfidence(newLandmarksToOldLandmarksMap.size());
    for (size_t i = 0; i < newLandmarksToOldLandmarksMap.size(); ++i) {
        newPoints.col(i) = m_points.col(newLandmarksToOldLandmarksMap[i]);
        newConfidence(i) = m_confidence[newLandmarksToOldLandmarksMap[i]];
    }
    m_points = newPoints;
    m_confidence = newConfidence;
    m_landmarkConfiguration = config;
}

template <class T, int D>
void LandmarkInstance<T, D>::AddPointToCurve(const std::string& curveName, int segment, const Eigen::Vector<T, D>& pos, T confidence)
{
    if (!m_landmarkConfiguration) {
        CARBON_CRITICAL("no landmark configuration set");
    }
    if (!m_landmarkConfiguration->HasCurve(curveName)) {
        CARBON_CRITICAL("no curve {} is part of the landmark instance", curveName);
    }

    const int numPoints = NumLandmarks();
    std::map<std::string, int> newLandmarkMapping = m_landmarkConfiguration->LandmarkMapping();
    std::map<std::string, std::vector<int>> newCurvesMapping = m_landmarkConfiguration->CurvesMapping();
    std::vector<int> indices = newCurvesMapping[curveName];
    indices.insert(indices.begin() + segment + 1, numPoints);
    newCurvesMapping[curveName] = indices;

    m_points.conservativeResize(D, numPoints + 1);
    m_confidence.conservativeResize(numPoints + 1);
    m_points.col(numPoints) = pos;
    m_confidence[numPoints] = confidence;
    m_landmarkConfiguration = std::make_shared<LandmarkConfiguration>(numPoints + 1, newLandmarkMapping, newCurvesMapping);
}

template <class T, int D>
bool LandmarkInstance<T, D>::LoadFromJson(const carbon::JsonElement& json)
{
    int version = 0;
    int frameNumber = 0;
    if (json.Contains("metadata")) {
        const auto& jMeta = json["metadata"];
        if (jMeta.Contains("version")) {
            version = jMeta["version"].Get<int>();
        }
        if (jMeta.Contains("frame_number")) {
            frameNumber = jMeta["frame_number"].Get<int>();
        }
    }

    if (version <= 0) {
        const bool newFormat = json.Contains("points");
        const carbon::JsonElement& jFrame = newFormat ? json["points"] : json;
        int numPoints = 0;
        std::vector<T> points;
        std::vector<T> confidence;
        std::map<std::string, int> landmarkMapping;
        std::map<std::string, std::vector<int>> curvesMapping;

        for (const auto& [landmarkOrCurveName, jLandmarks] : jFrame.Map()) {
            const int numPointsForLandmark = int(jLandmarks.Size());
            if (numPointsForLandmark == 1) {
                // landmark
                landmarkMapping.emplace(landmarkOrCurveName, numPoints);
                for (int d = 0; d < D; ++d) {
                    points.push_back(jLandmarks[0][d].template Value<T>());
                }
                confidence.push_back(T(1));
            }
            else {
                std::vector<int> indices(numPointsForLandmark);
                for (int i = 0; i < numPointsForLandmark; ++i) {
                    indices[i] = numPoints + i;
                    for (int d = 0; d < D; ++d) {
                        points.push_back(jLandmarks[i][d].template Value<T>());
                    }
                    confidence.push_back(T(1));
                }
                curvesMapping.emplace(landmarkOrCurveName, std::move(indices));
            }
            numPoints += numPointsForLandmark;
        }
        m_landmarkConfiguration = std::make_shared<LandmarkConfiguration>(numPoints, landmarkMapping, curvesMapping);
        m_frameNumber = frameNumber;
        m_points = Eigen::Map<const Eigen::Matrix<T, D, -1>>(points.data(), D, numPoints);
        m_confidence = Eigen::Map<const Eigen::VectorX<T>>(confidence.data(), numPoints);
    } else if (version == 1) {
        std::map<std::string, int> landmarkMapping = json["landmarks"].Get<std::map<std::string, int>>();
        std::map<std::string, std::vector<int>> curvesMapping = json["curves"].Get<std::map<std::string, std::vector<int>>>();
        int numPoints;
        if constexpr (D == 3) {
            std::vector<std::tuple<T, T, T>> points = json["points"].Get<std::vector<std::tuple<T, T, T>>>();
            numPoints = static_cast<int>(points.size());
            m_points = Eigen::Map<const Eigen::Matrix<T, D, -1>>((const T*)points.data(), D, numPoints);
        }
        else if constexpr (D == 2) {
            std::vector<std::pair<T, T>> points = json["points"].Get<std::vector<std::pair<T, T>>>();
            numPoints = static_cast<int>(points.size());
            m_points = Eigen::Map<const Eigen::Matrix<T, D, -1>>((const T*)points.data(), D, numPoints);
        } else {
            return false;
        }

        std::vector<T> confidence(numPoints, T(1));
        if (json.Contains("confidences")) {
            confidence = json["confidences"].Get<std::vector<T>>();
        } else if (json.Contains("confidence")) {
            confidence = json["confidence"].Get<std::vector<T>>();
        }

        m_landmarkConfiguration = std::make_shared<LandmarkConfiguration>(numPoints, std::move(landmarkMapping), std::move(curvesMapping));
        m_frameNumber = frameNumber;
        m_confidence = Eigen::Map<const Eigen::VectorX<T>>(confidence.data(), numPoints);


    } else {
        return false;
    }

    return true;
}

template <class T, int D>
carbon::JsonElement LandmarkInstance<T, D>::SaveToJson() const
{
    carbon::JsonElement json(carbon::JsonElement::JsonType::Object);

    carbon::JsonElement meta(carbon::JsonElement::JsonType::Object);
    meta.Insert("type", carbon::JsonElement("frame"));
    meta.Insert("version", carbon::JsonElement(1));
    meta.Insert("frame_number", carbon::JsonElement(m_frameNumber));
    json.Insert("metadata", std::move(meta));

    carbon::JsonElement points(carbon::JsonElement::JsonType::Array);
    carbon::JsonElement confidences(carbon::JsonElement::JsonType::Array);
    for (int i = 0; i < NumLandmarks(); ++i) {
        if constexpr (D == 3) {
            points.Append(carbon::JsonElement(std::tuple<T, T, T>(m_points(0, i), m_points(1, i), m_points(2, i))));
        } else if constexpr (D == 2) {
            points.Append(carbon::JsonElement(std::pair<T, T>(m_points(0, i), m_points(1, i))));
        } else {
            CARBON_CRITICAL("landmark dimension not supported.");
        }
        confidences.Append(carbon::JsonElement(m_confidence[i]));
    }
    json.Insert("points", std::move(points));
    json.Insert("confidences", std::move(confidences));

    if (m_landmarkConfiguration) {
        json.Insert("landmarks", carbon::JsonElement(m_landmarkConfiguration->LandmarkMapping()));
        json.Insert("curves", carbon::JsonElement(m_landmarkConfiguration->CurvesMapping()));
    }

    return json;
}

// explicitly instantiate the landmark instance classes
template class LandmarkInstance<float, 2>;
template class LandmarkInstance<double, 2>;
template class LandmarkInstance<float, 3>;
template class LandmarkInstance<double, 3>;


} // namespace epic::nls
