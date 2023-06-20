// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/MetaShapeCamera.h>
#include <nls/math/Math.h>
#include <nls/utils/FileIO.h>

#include <nrr/landmarks/LandmarkConfiguration.h>
#include <nrr/landmarks/LandmarkInstance.h>

#include <map>
#include <memory>
#include <string>

namespace epic {
namespace nls {


template <class T>
class LandmarkSequence
{
public:
    LandmarkSequence();

    /**
     * Loads the landmarks from @p filename.
     * If @p frameOffset >= 0, then landmarks are loaded sequentially from the landmark file and the internal frame number of
     * the landmarks is ignored, instead it will be frameOffset + index. If @p frameOffset < 0, then the frame number of the landmarks
     * are used.
     */
    bool Load(const std::string& filename, int frameOffset);
    void Save(const std::string& filename) const;

    bool HasLandmarks(int frame) const;

    const LandmarkInstance<T, 2>& Landmarks(int frame) const;

    std::shared_ptr<const LandmarkInstance<T, 2>> LandmarksPtr(int frame) const;

    /**
     * Undistort all landmarks and curves in the sequence.
     * Precondition: landmarks and curves have not yet been undistorted.
     */
    void Undistort(const MetaShapeCamera<T>& camera);

    /**
     * Merges curves @curveNames into a single @p newCurveName concatenating points along matching points. See LandmarkConfiguration::MergeCurves.
     * If @p ignoreMissingCurves is True, then missing curves are simply ignored.
     * @return True if the new curve was generated, False if nothing could be merged.
     */
    bool MergeCurves(const std::vector<std::string>& curveNames, const std::string& newCurveName, bool ignoreMissingCurves, bool removeMergedCurves = false);

    //! Combine two landmark sequences
    void MergeSequences(const LandmarkSequence& otherSequence);

    //! convenience function to get access to all landmark instances
    std::map<int, std::shared_ptr<const LandmarkInstance<T, 2>>> LandmarkInstances() const;

    // set the landmark sequences
    void SetLandmarkInstances(const std::map<int, std::shared_ptr<LandmarkInstance<T, 2>>>& landmarkInstances);

    void SetLandmarkInstance(int frame, std::shared_ptr<LandmarkInstance<T, 2>>& landmarkInstance);

private:
    std::map<int, std::shared_ptr<LandmarkInstance<T, 2>>> m_landmarkInstances;
};

} // namespace nls
} // namespace epic
