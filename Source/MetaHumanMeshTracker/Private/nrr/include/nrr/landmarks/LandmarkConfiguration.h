// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>
#include <nls/math/Math.h>

#include <map>
#include <string>

namespace epic::nls {

class LandmarkConfiguration
{
public:
    LandmarkConfiguration() = default;
    LandmarkConfiguration(int numPoints, const std::map<std::string, int>& landmarkMapping, const std::map<std::string, std::vector<int>>& curvesMapping)
        : m_numPoints(numPoints)
        , m_landmarkMapping(landmarkMapping)
        , m_curvesMapping(curvesMapping)
    {}
    LandmarkConfiguration(int numPoints, std::map<std::string, int>&& landmarkMapping, std::map<std::string, std::vector<int>>&& curvesMapping)
        : m_numPoints(numPoints)
        , m_landmarkMapping(landmarkMapping)
        , m_curvesMapping(curvesMapping)
    {}

    bool LoadConfiguration(const epic::carbon::JsonElement& j);

    void AddLandmark(const std::string& landmarkName);
    void AddCurve(const std::string& curveName, int numPoints);
    void RemoveLandmark(const std::string& landmarkName);
    void RemoveCurve(const std::string& curveName);

    int NumPoints() const { return m_numPoints; }

    bool HasLandmark(const std::string& landmarkName) const { return m_landmarkMapping.find(landmarkName) != m_landmarkMapping.end(); }

    //! @return the global index of the landmark
    int IndexForLandmark(const std::string& landmarkName) const;

    bool HasCurve(const std::string& curveName) const { return m_curvesMapping.find(curveName) != m_curvesMapping.end(); }

    /**
     * @return the indices for the curve @p curveName.
     * @pre the curve needs to exists. See HasCurve()
     */
    const std::vector<int>& IndicesForCurve(const std::string& curveName) const;

    const std::map<std::string, int>& LandmarkMapping() const { return m_landmarkMapping; }
    const std::map<std::string, std::vector<int>>& CurvesMapping() const { return m_curvesMapping; }

    /**
     * Merges curves @p curveNames into a single @p newCurveName concatenating points along matching indices.
     * Landmarks and curves have potentially duplicates, so the caller needs to pass in a set of landmarks @p pts and duplicated landmarks are found
     * using the landmark positions (@see FindDuplicatesAndCreateMap). This way the curves can be merged.
     * If @p ignoreMissingCurves is True, then all curves that are not part of the landmark configuration are ignored.
     * @return True if the new curve was generated, False if nothing could be merged.
     *
     * Ideally we can replace it in the future if we can guarantee that the loaded landmarks never have duplicates, and instead curves are stored as vector of indices
     * in the json files.
     */
    template <class T, int D>
    bool MergeCurves(const std::vector<std::string>& curveNames, const std::string& newCurveName, const Eigen::Matrix<T, D, -1>& pts, bool ignoreMissingCurves, bool removeMergedCurves = false);

    //! Combine two landmark configurations
    void MergeConfiguration(const LandmarkConfiguration& otherConfiguration);

    //! Finds duplicated landmarks and returns a map to a single unique landmark
    template <class T, int D>
    std::map<int, int> FindDuplicatesAndCreateMap(const Eigen::Matrix<T, D, -1>& pts) const;

    /**
     * Remap landmarks and curves based on the input @p map, removing all unused points, and moving all points into one continuous chunk.
     * @param[in] map  The map specifying which landmarks/curve indices map to what unique landmark index (e.g. as created
     *                 by FindDuplicatesAndCreateMap)
     * @return A vector the size of the new number of landmarks, where each entry maps to the unique landmark before compression.
     */
    std::vector<int> RemapLandmarksAndCurvesAndCompress(const std::map<int, int>& map);

private:
    //! total number of points in points matrix (see LandmarkInstance, may have duplicate points)
    int m_numPoints;

    //! map from landmark name to index in points matrix (see LandmarkInstance)
    std::map<std::string, int> m_landmarkMapping;

    //! map from curve name to indices in points matrix (see LandmarkInstance)
    std::map<std::string, std::vector<int>> m_curvesMapping;
};


} // namespace epic::nls
