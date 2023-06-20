// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Defs.h"

#include <stdint.h>

namespace titan {
namespace api {

class TITAN_API StereoReconstruction {
public:
    StereoReconstruction();
    ~StereoReconstruction();
    StereoReconstruction(StereoReconstruction&&) = delete;
    StereoReconstruction(const StereoReconstruction&) = delete;
    StereoReconstruction& operator=(StereoReconstruction&&) = delete;
    StereoReconstruction& operator=(const StereoReconstruction&) = delete;

    /**
     * Initialize stereo reconstruction.
     * @returns True if initialization is successful, False otherwise.
     */
    bool Init();

    /**
     * Load the camera calibration from @p Filename
     * @returns True if successful, False if there was a problem loading the calibration from file.
     */
    bool LoadCalibrationFromFile(const char* Filename);

    /**
     * Retrieves the image size for camera @p CameraName
     * @returns False if the camera does not exist.
     */
    bool GetImageSize(const char* CameraName, int32_t& WidthOut, int32_t& HeightOut) const;

    /**
     * Set the depth range for camera @p CameraName.
     */
    bool SetCameraRange(const char* CameraName, float MinDepth, float MaxDepth);

    /**
     * Reconstructs the point cloud using the stereo pair @p CameraNameLeft and @p CameraNameRight and images @p ImageDataLeft and @p ImageDataRight.
     * The images need have format RGBA uint8. Check the image for a camera using GetImageSize().
     * @returns The number of reconstructed points. -1 in case of an error.
     */
    int32_t ReconstructPointCloud(const char* CameraNameLeft,
        const char* CameraNameRight,
        const unsigned char* ImageDataLeft,
        const unsigned char* ImageDataRight);

    /**
     * Retrieve the point cloud as reconstructed by @p ReconstructPointCloud(). Copies a maximum of @p MaximumNumberOfPoints.
     * It is the callers responsibility that @p PointCloudDataOut points to memory of at least size MaximumNumberOfPoints * 3 * sizeof(float).
     * @returns The number of points. -1 in case of an error.
     */
    int32_t RetrievePointCloud(float* PointCloudDataOut, int32_t MaximumNumberOfPoints) const;

    /**
     * Get the number of vertices and triangles for the reconstructed point cloud.
     */
    void GetReconstructedTriangleMeshSize(int32_t& NumberOfPoints, int32_t& NumberOfTriangles) const;

    //! Reconstruct a triangle mesh from the stereo reconstruction.
    bool RetrieveReconstructedTriangleMesh(float* VerticesDataOut, int32_t* TriangleIndicesOut) const;

private:
    struct Private;
    Private* m;
};

} // namespace api
} // namespace titan
