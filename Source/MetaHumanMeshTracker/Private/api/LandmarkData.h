// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Defs.h"

#include <map>
#include <string>
#include <vector>

namespace titan {
namespace api {

/**
 * Simple class to pass landmark data to FaceTrackingAPI and ActCreationAPI.
 */
class TITAN_API FaceTrackingLandmarkData
{
public:

    FaceTrackingLandmarkData();
    ~FaceTrackingLandmarkData();
    FaceTrackingLandmarkData(const FaceTrackingLandmarkData& other);
    FaceTrackingLandmarkData& operator=(const FaceTrackingLandmarkData& other);
    void swap(FaceTrackingLandmarkData&);
    FaceTrackingLandmarkData(FaceTrackingLandmarkData&&);
    FaceTrackingLandmarkData& operator=(FaceTrackingLandmarkData&&);

    const std::vector<float> & ConfidenceData() const;
    const std::vector<float>& PointsData() const;

    int32_t NumPoints() const;
    int32_t PointsDimension() const;

    //! Create FaceTrackingLandmarkData from a float array and optional confidence data.
    static FaceTrackingLandmarkData Create(const float* PointsData, const float* ConfidenceData, int32_t NumPoints, int32_t Dim);

private:
    struct Private;
    Private* m{};
};

} // namespace api
} // namespace titan
