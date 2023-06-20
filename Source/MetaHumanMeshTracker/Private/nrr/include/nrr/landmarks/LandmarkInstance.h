// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>
#include <nls/math/Math.h>
#include <nrr/landmarks/LandmarkConfiguration.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace epic {
namespace nls {

template <class T, int D>
class LandmarkInstance
{
public:
    LandmarkInstance() = default;
    LandmarkInstance(const Eigen::Matrix<T, D, -1> points, const Eigen::Vector<T, -1>& confidence)
    {
        Set(points, confidence);
    }

    void Clear(int numLandmarks)
    {
        m_points = Eigen::Matrix<T, D, -1>::Zero(D, numLandmarks);
        m_confidence = Eigen::VectorX<T>::Zero(numLandmarks);
    }

    void SetLandmark(int landmarkIndex, const Eigen::Vector<T,D>& pos, T confidence)
    {
        m_points.col(landmarkIndex) = pos;
        m_confidence[landmarkIndex] = confidence;
    }

    void SetLandmarkConfiguration(const std::shared_ptr<const LandmarkConfiguration>& landmarkConfiguration)
    {
        if (landmarkConfiguration && landmarkConfiguration->NumPoints() != NumLandmarks()) {
            CARBON_CRITICAL("landmark configuration is not valid for landmark instance");
        }
        m_landmarkConfiguration = landmarkConfiguration;
    }
    const std::shared_ptr<const LandmarkConfiguration>& GetLandmarkConfiguration() const { return m_landmarkConfiguration; }

    void Set(const Eigen::Matrix<T, D, -1> points, const Eigen::Vector<T, -1>& confidence) {
        if (points.cols() != confidence.size()) {
            CARBON_CRITICAL("size of points and confidence does not match");
        }
        m_points = points;
        m_confidence = confidence;
    }

    int NumLandmarks() const { return int(m_points.cols()); }

    int FrameNumber() const { return m_frameNumber; }
    void SetFrameNumber(int frameNumber) { m_frameNumber = frameNumber; }

    const Eigen::Matrix<T, D, -1>& Points() const { return m_points; }
    const Eigen::Vector<T, -1>& Confidence() const { return m_confidence; }

    Eigen::Vector<T, D> Point(int index) const { return m_points.col(index); }
    T Confidence(int index) const { return m_confidence[index]; }

    Eigen::Matrix<T, D, -1> Points(const std::vector<int>& indices) const;
    Eigen::Vector<T, -1> Confidences(const std::vector<int>& indices) const;

    /**
     * Merges curves @curveNames into a single @p newCurveName concatenating points along matching points. See LandmarkConfiguration::MergeCurves.
     * If @p ignoreMissingCurves is True, then missing curves are simply ignored.
     * @return True if the new curve was generated, False if nothing could be merged.
     */
    bool MergeCurves(const std::vector<std::string>& curveNames, const std::string& newCurveName, bool ignoreMissingCurves, bool removeMergedCurves = false);

    void MergeInstance(const LandmarkInstance& other);

    /**
     * Method to reduce the number of points along the curve in order to have better control while manually modifying.
     * @param[in] curveName    name of the curve to simplify
     */
    void SimplifyCurve(const std::string& curveName, T eps = T(1));

    /**
     * Method to reduce the number of points along the curve in order to have better control while manually modifying.
     * Applies to all curves in landmark configuration.
     */
    void SimplifyCurves(T eps = T(1));

    /**
     *  Convenience function to create a landmark instance and a landmark configuration from a
     *  set of landmark data and optionally a description on what curves to merge.
     * @param[in] namedLandmarkData   Landmark data mapping the name to the landmark/curve.
     * @param[in] curvesToMerge       Description on what curves to merge for the instance.
     * @return The created landmakr instance including landmark configuration.
     */
    static std::shared_ptr<const LandmarkInstance<T,D>> CreateLandmarkInstance(const std::map<std::string, Eigen::Matrix<T, D, -1>>& namedLandmarkData,
                                                                                      const std::map<std::string, std::vector<std::string>>& curvesToMerge);

    /**
     * Remove duplicates based on the exact equality of the landmarks and curves points and compresses the remaining points into one
     * continuous chunk.
     */
    void RemoveDuplicatesAndCompress();

    //! Remove landmark/curve points with index @p landmark from the landmark instance.
    void RemoveLandmarkOrCurvePoint(int landmark);

    /**
     * Add a point to an existing curve.
     * @param[in] curveName  Name of the curve that is modified.
     * @param[in] segment    The index of the segment after which the point is added.
     * @param[in] pos        The position of the new curve point.
     * @param[in] confidence The confidence of the new curve point.
     */
    void AddPointToCurve(const std::string& curveName, int segment, const Eigen::Vector<T, D>& pos, T confidence);

    //! Load the landmark instance from a json structure.
    bool LoadFromJson(const carbon::JsonElement& json);

    //! Save the landmark instance to json
    carbon::JsonElement SaveToJson() const;

private:
    Eigen::Matrix<T, D, -1> m_points;
    Eigen::Vector<T, -1> m_confidence;
    int m_frameNumber = -1;
    std::shared_ptr<const LandmarkConfiguration> m_landmarkConfiguration;
};

} // namespace nls
} // namespace epic
