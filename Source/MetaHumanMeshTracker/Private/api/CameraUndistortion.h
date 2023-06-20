// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Defs.h"
#include "OpenCVCamera.h"

namespace titan {
namespace api {

class TITAN_API CameraUndistortion {
public:
    CameraUndistortion();
    ~CameraUndistortion();
    CameraUndistortion(CameraUndistortion&&) = delete;
    CameraUndistortion(const CameraUndistortion&) = delete;
    CameraUndistortion& operator=(CameraUndistortion&&) = delete;
    CameraUndistortion& operator=(const CameraUndistortion&) = delete;

    /**
     * Initialize camera undistortion.
     * @returns True if initialization is successful, False otherwise.
     */
    bool Init();

    /**
     * Remove all cameras.
     */
    void Reset();

    /**
     * Add a camera with name @p InCameraName and camera parameters @p InCameraParameters
     */
    void AddCamera(const char* InCameraName, const OpenCVCamera& InCameraParameters);

    /**
     * Undistorts the image @p InImageData using the parameters of camera @p InCameraName and saves it to @p OutImageData.
     * The images need have format RGBA uint8, though each channel is interpolated independently including the alpha channel.
     * @returns True if image was undistorted, False if @p InCameraName was not added via AddCamera().
     */
    bool UndistortImage(const char* InCameraName,
                        const unsigned char* InImageData,
                        unsigned char* OutImageData);

    /**
     * Undistort the @p InNumPoints number of points @p InPointsData using the parameters of camera @p InCameraName and outputs it to @p OutPointsData
     * @returns True if the points were undistorted, False if @p InCameraName was not added via AddCamera().
     */
    bool UndistortPoints(const char* InCameraName,
                         int32_t InNumPoints,
                         const float* InPointsData,
                         float* OutPointsData) const;

private:
    struct Private;
    Private* m;
};

} // namespace api
} // namespace titan
