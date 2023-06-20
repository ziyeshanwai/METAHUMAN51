// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/utils/Timecode.h>
#include <carbon/io/JsonIO.h>
#include <nls/serialization/CameraSerialization.h>
#include <nls/serialization/MetaShapeSerialization.h>
#include <nls/utils/FileIO.h>

#include <filesystem>
#include <map>
#include <string>
#include <vector>
#include <optional>

namespace epic::nls {

class DataDescription {
public:

    DataDescription() = default;

    bool Load(const std::string& filename);

    int NumCameras() const { return static_cast<int>(m_cameras.size()); }

    const std::vector<std::string>& CameraNames() const { return m_cameras; }

    std::string CalibrationCameraLabel(const std::string& cameraName) const;

    std::vector<MetaShapeCamera<float>> LoadCameras() const;

    bool HasCameraRanges() const { return bool(m_cameraDepthRange); }

    std::map<std::string, std::pair<float, float>> CameraRanges() const;

    std::string ImageFilename(const std::string& cameraName, int imageIndex) const;

    //! checks that there is a pattern and label for each camera, and optionally if every file exists
    bool CheckValidity(bool checkFilesExist);

    int ImageStart() const { return m_imageStart; }
    int ImageEnd() const { return m_imageEnd; }

    bool HasVideo(const std::string& cameraName) const;

    std::string CameraVideo(const std::string& cameraName) const;

    bool HasLandmarks(const std::string& cameraName) const;

    std::string Landmarks(const std::string& cameraName) const;

    const std::map<std::string, std::string>& Landmarks() const { return m_landmarks; }

    const std::map<std::string, std::string>& Markers() const { return m_markers; }

    bool LandmarksNeedToBeUndistorted() const { return m_landmarksNeedToBeUndistorted; }
    bool LandmarksFrameNumbersAreCorrect() const { return m_landmarksFrameNumbersAreCorrect; }

    bool IsSRGBInput() const { return m_srgbInput; }

    bool HasScans() const { return !m_scanBase.empty(); }

    bool HasScanTextures() const { return !m_scanTexturePattern.empty(); }

    bool HasScanUpVector() const { return m_scanUpVector.has_value(); }

    const Eigen::Vector3f& ScanUpVector() const { return m_scanUpVector.value(); }

    bool HasScanFrontVector() const { return m_scanFrontVector.has_value(); }

    const Eigen::Vector3f& ScanFrontVector() const { return m_scanFrontVector.value(); }

    std::string ScanMeshFilename(int index) const;

    std::string ScanTextureFilename(int index) const;

    //! @return True if the data description has orientation information for the camera.
    bool HasCameraOrientation(const std::string& cameraName) const;

    /**
     * The natural orientation per camera. Defines how many degrees the camera image needs to be rotated
     * clock-wise for its natural orienation i.e. the face is upright.
     * @throw runtime error if no orientation for camera @p cameraName is specified. Use HasCameraOrientation() to check.
     */
    int CameraOrientation(const std::string& cameraName) const;

    //! @returns True if the data description contains depthmap data for the camera.
    int NumCamerasWithDepthmaps() const { return static_cast<int>(m_depthmapPatternMapping.size()); }
    bool HasDepthmaps(const std::string& cameraName) const;
    std::string DepthmapFilename(const std::string& cameraName, int imageIndex) const;

    //! @returns True if the data description contains flow data for the camera.
    bool HasFlow(const std::string& cameraName) const;
    std::string FlowFilename(const std::string& cameraName, int srcImageIndex, int targetImageIndex) const;

    double GetFps() const { return m_timecode.GetFps(); }
    const carbon::Timecode& GetTimecode() const { return m_timecode; }

private:
    std::vector<std::string> m_cameras;

    std::string m_calibrationFile;

    //! mapping from camera names to the camera label in the calibration file
    std::map<std::string, std::string> m_cameraCalibrationLabelMapping;

    //! optional information on the valid depth range for each camera
    std::optional<std::pair<float, float>> m_cameraDepthRange;

    int m_imageStart;
    int m_imageEnd;
    std::map<std::string, std::string> m_cameraPatternMapping;

    std::map<std::string, std::string> m_depthmapPatternMapping;
    std::map<std::string, std::string> m_flowPatternMapping;

    /**
     * The natural orientation per camera. Defines how many degrees the camera image needs to be rotated
     * clock-wise for its natural orienation i.e. the face is upright.
     */
    std::map<std::string, int> m_cameraOrientations;

    std::map<std::string, std::string> m_cameraVideos;

    std::map<std::string, std::string> m_landmarks;
    std::map<std::string, std::string> m_markers;
    bool m_landmarksNeedToBeUndistorted = true;
    bool m_landmarksFrameNumbersAreCorrect = false;

    //! Flag describing if the data input is in srgb color space. Default is linear color space.
    bool m_srgbInput = false;

    //! description of scan meshes
    std::string m_scanBase;
    std::string m_scanMeshPattern;
    //! Texture for scans. May be empty if no texture is available for the scan.
    std::string m_scanTexturePattern;
    //! The index of the scan mesh may be offset relative to the images (mesh_5.obj corresponds to the image index 3 if m_scanIndexOffset=2)
    int m_scanIndexOffset = 0;
    //! Optional information on the up vector of a scan.
    std::optional<Eigen::Vector3f> m_scanUpVector;
    //! Optional information on the front vector of a scan (the direction in which the nose points).
    std::optional<Eigen::Vector3f> m_scanFrontVector;

    carbon::Timecode m_timecode = carbon::Timecode(0, 0, 0, 0, 60.0);
};


} //namespace epic::nls
