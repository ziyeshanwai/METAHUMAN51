// Copyright Epic Games, Inc. All Rights Reserved.

#include "LandmarkData.h"
#include "Common.h"

#include <cstring>

using namespace epic::carbon;
using namespace pma;

namespace titan::api {
    struct FaceTrackingLandmarkData::Private {
        std::vector<float> PointsData;
        std::vector<float> ConfidenceData;
        int32_t Dim;
    };


    FaceTrackingLandmarkData::FaceTrackingLandmarkData() :
        m(new FaceTrackingLandmarkData::Private()) {
    }

    FaceTrackingLandmarkData::~FaceTrackingLandmarkData() {
        delete m;
    }

    FaceTrackingLandmarkData::FaceTrackingLandmarkData(const FaceTrackingLandmarkData& other) :
        m(new FaceTrackingLandmarkData::Private()) {
        m->PointsData = other.m->PointsData;
        m->ConfidenceData = other.m->ConfidenceData;
        m->Dim = other.m->Dim;
    }

    void FaceTrackingLandmarkData::swap(FaceTrackingLandmarkData& other) {
        std::swap(m, other.m);
    }

    FaceTrackingLandmarkData& FaceTrackingLandmarkData::operator=(const FaceTrackingLandmarkData& other) {
        return *this = FaceTrackingLandmarkData(other);
    }

    FaceTrackingLandmarkData::FaceTrackingLandmarkData(FaceTrackingLandmarkData&& other) {
        swap(other);
    }

    FaceTrackingLandmarkData& FaceTrackingLandmarkData::operator=(FaceTrackingLandmarkData&& other) {
        swap(other);
        return *this;
    }

    const std::vector<float>& FaceTrackingLandmarkData::ConfidenceData() const {
        return m->ConfidenceData;
    }

    const std::vector<float>& FaceTrackingLandmarkData::PointsData() const {
        return m->PointsData;
    }

    int32_t FaceTrackingLandmarkData::NumPoints() const {
        return int32_t(m->ConfidenceData.size());
    }

    int32_t FaceTrackingLandmarkData::PointsDimension() const {
        return int32_t(m->Dim);
    }

    FaceTrackingLandmarkData FaceTrackingLandmarkData::Create(const float* PointsData,
                                                              const float* ConfidenceData,
                                                              int32_t NumPoints,
                                                              int32_t Dim) {

        FaceTrackingLandmarkData Out;
		const int32_t totalPoints = NumPoints * Dim;
        Out.m->PointsData.resize(totalPoints);
        Out.m->Dim = Dim;
        memcpy(Out.m->PointsData.data(), PointsData, sizeof(float) * totalPoints);
        if (ConfidenceData) {
            Out.m->ConfidenceData.resize(NumPoints);
            memcpy(Out.m->ConfidenceData.data(), ConfidenceData, sizeof(float) * NumPoints);
        } else {
            Out.m->ConfidenceData = std::vector<float>(NumPoints, 1.0f);
        }
        return Out;
    }
}  // namespace titan::api
