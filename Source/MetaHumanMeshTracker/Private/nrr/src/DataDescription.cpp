// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/DataDescription.h>

namespace epic::nls {

template<typename ... Args>
std::string StringFormat(const std::string& format, Args ... args)
{
    const size_t size = snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
    if (size <= 0) {
        CARBON_CRITICAL("Error during formatting.");
    }
    std::unique_ptr<char[]> buf(new char[size]);
    std::snprintf(buf.get(), size, format.c_str(), args ...);
    return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

std::string ReplaceSubstring(const std::string& input, const std::string& pattern, const std::string& replacement)
{
    std::string newString = input;
    while (true) {
        auto pos = newString.find(pattern);
        if (pos == std::string::npos) return newString;
        newString.erase(pos, pattern.size());
        newString.insert(pos, replacement);
    }
    return newString;
}

bool DataDescription::Load(const std::string& filename)
{
    try {
        const std::string filedata = ReadFile(filename);
        const carbon::JsonElement j = carbon::ReadJson(filedata);

        const std::string dataDescriptionDirectory = std::filesystem::absolute(std::filesystem::path(filename)).parent_path().string();

        const std::string CAMERAVAR = "${CAMERA}";
        std::map<std::string, std::string> variables;

        if (j.Contains("variables")) {
            variables = j["variables"].Get<std::map<std::string, std::string>>();
        }

        auto replaceVariables = [&](const std::string& filename, const std::string& additionalKey = "", const std::string& additionalValue = "") {
            std::string filenameWithVarSubstitution = filename;
            for (const auto& [varname, varvalue]  : variables) {
                const std::string varnameFull = "${" + varname + "}";
                filenameWithVarSubstitution = ReplaceSubstring(filenameWithVarSubstitution, varnameFull, varvalue);
            }
            if (!additionalKey.empty()) {
                const std::string additionalKeyFull = "${" + additionalKey + "}";
                filenameWithVarSubstitution = ReplaceSubstring(filenameWithVarSubstitution, additionalKeyFull, additionalValue);
            }
            if (filenameWithVarSubstitution.find("${") != std::string::npos) {
                LOG_ERROR("failed to fully substitute variables in string \"{}\"", filenameWithVarSubstitution);
            }
            return filenameWithVarSubstitution;
        };

        auto makeAbsolute = [&](const std::string& filename, const std::string& additionalKey = "", const std::string& additionalValue = "") {
            const std::string filenameWithVarSubstitution = replaceVariables(filename, additionalKey, additionalValue);
            if (std::filesystem::path(filenameWithVarSubstitution).is_relative()) {
                return dataDescriptionDirectory + "/" + filenameWithVarSubstitution;
            } else {
                return filenameWithVarSubstitution;
            }
        };

        // parse cameras - We will survive absense of the camera, but then only scans needs to be present in the file
        // Likewise, if cameras are present, all other data (images or videos, landmarks also needs to be available)
        bool hasCameras = true;
        if (!j.Contains("cameras")) {
            LOG_INFO("no camera in json file {}", filename);
            hasCameras = false;
        }
        if (hasCameras && !j["cameras"].IsArray())
        {
            LOG_ERROR("cameras in {} need to point to an array of strings", filename);
            return false;
        }
        const std::vector<std::string> cameras = hasCameras ? j["cameras"].Get<std::vector<std::string>>() : std::vector<std::string>(0);

        // parse calibration
        if (hasCameras ^ j.Contains("calibration"))
        {
            LOG_ERROR("Invalid calibration data in json file {}", filename);
            return false;
        }

        const std::string calibrationFile = hasCameras ? makeAbsolute(j["calibration"]["file"].String()) : "";
        std::map<std::string, std::string> cameraCalibrationLabelMapping;

        if (hasCameras)
        {
            if (j["calibration"].Contains("label"))
            {
                cameraCalibrationLabelMapping = j["calibration"]["label"].Get<std::map<std::string, std::string>>();
            }
            else
            {
                // camera name is the same between calibration file and internal name
                for (const auto& cam : cameras)
                {
                    cameraCalibrationLabelMapping[cam] = cam;
                }
            }

            if (j["calibration"].Contains("depth_range"))
            {
                // load optional depth range data for the cameras
                m_cameraDepthRange = j["calibration"]["depth_range"].Get<std::pair<float, float>>();
            }
        }

        // parse image info
        if (hasCameras ^ (j.Contains("images") || j.Contains("videos")))
        {
            LOG_ERROR("Invalid image or video info in json file {}", filename);
            return false;
        }

        std::map<std::string, std::string> cameraPatternMapping;
        std::string imageBase;
        int imageStart = -1;
        int imageEnd = -1;

        if (j.Contains("range")) {
            std::tie(imageStart, imageEnd) = j["range"].Get<std::pair<int, int>>();
        }

        if (j.Contains("images")) {
            imageBase = j["images"]["base"].String();
            if (j["images"].Contains("range"))
            {
                LOG_WARNING("Placing range inside images segment is deprecated. It should reside in the root section of config file.");
                std::tie(imageStart, imageEnd) = j["images"]["range"].Get<std::pair<int, int>>();
            }

            const auto& cameraPatternMap = j["images"]["pattern"].Map();
            for (const auto& [key, value] : cameraPatternMap) {
                if (key != CAMERAVAR) {
                    if (std::find(cameras.begin(), cameras.end(), key) == cameras.end()) {
                        LOG_WARNING("images are specified for an unknown camera {}", key);
                    }
                    cameraPatternMapping[key] = makeAbsolute(imageBase + "/" + value.String(), "CAMERA", key);
                }

            }
            auto it = cameraPatternMap.find(CAMERAVAR);
            if (it != cameraPatternMap.end()) {
                const std::string filenamePattern = it->second.String();
                for (const std::string& camera : cameras) {
                    if (cameraPatternMap.find(camera) == cameraPatternMap.end()) {
                        const std::string autoPath = makeAbsolute(imageBase + "/" + filenamePattern, "CAMERA", camera);
                        if (std::filesystem::exists(StringFormat(autoPath, imageStart))) {
                            cameraPatternMapping[camera] = autoPath;
                            LOG_INFO("automatically found images for camera {}: {}", camera, cameraPatternMapping[camera]);
                        }
                    }
                }
            }

            if (j["images"].Contains("srgb_format")) {
                m_srgbInput = j["images"]["srgb_format"].Boolean();
            } else {
                LOG_WARNING("no key \"srgb_format\" within \"images\", assuming data is in linear color space i.e. \"srgb_format\": false");
            }
        }

        std::map<std::string, std::string> cameraVideos;
        if (j.Contains("videos")) {
            const carbon::JsonElement& jVideos = j["videos"];
            const std::string videoBase = jVideos.Contains("base") ? jVideos["base"].String() : "";
            if (jVideos.Contains("paths")) {
                const auto& cameraPathsMap = jVideos["paths"].Map();
                for (const auto& [key, value] : cameraPathsMap) {
                    if (key != CAMERAVAR) {
                        if (std::find(cameras.begin(), cameras.end(), key) == cameras.end()) {
                            LOG_WARNING("videos are specified for an unknown camera {}", key);
                        }
                        if (videoBase.empty()) {
                            cameraVideos[key] = makeAbsolute(value.String(), "CAMERA", key);
                        } else {
                            cameraVideos[key] = makeAbsolute(videoBase + "/" + value.String(), "CAMERA", key);
                        }
                    }
                }
                auto it = cameraPathsMap.find(CAMERAVAR);
                if (it != cameraPathsMap.end()) {
                    const std::string filenamePattern = it->second.String();
                    for (const std::string& camera : cameras) {
                        if (cameraVideos.find(camera) == cameraVideos.end()) {
                            std::string autoVideoPath;
                            if (videoBase.empty()) {
                                autoVideoPath = makeAbsolute(filenamePattern, "CAMERA", camera);
                            } else {
                                autoVideoPath = makeAbsolute(videoBase + "/" + filenamePattern, "CAMERA", camera);
                            }
                            if (std::filesystem::exists(autoVideoPath)) {
                                cameraVideos[camera] = autoVideoPath;
                                LOG_INFO("automatically found video {} for camera {}", cameraVideos[camera], camera);
                            }
                        }
                    }
                }

            } else {
                for (const auto& [key, value] : j["videos"].Map()) {
                    cameraVideos[key] = makeAbsolute(value.String(), "CAMERA", key);
                }
            }
            if (jVideos.Contains("range"))
            {
                LOG_WARNING("Placing range inside videos segment is deprecated. It should reside in the root section of config file.");
                std::tie(imageStart, imageEnd) = jVideos["range"].Get<std::pair<int, int>>();
            }

            if (jVideos.Contains("srgb_format")) {
                m_srgbInput = jVideos["srgb_format"].Boolean();
            } else {
                LOG_WARNING("no key \"srgb_format\" within \"videos\", assuming data is in linear color space i.e. \"srgb_format\": false");
            }
        }

         // parse range info
        if (imageStart < 0 || imageEnd < 0) {
            LOG_ERROR("data range is missing in json file {}", filename);
            return false;
        }

        for (const std::string& camera : cameras) {
            if (cameraPatternMapping.find(camera) == cameraPatternMapping.end() &&
                cameraVideos.find(camera) == cameraVideos.end()) {
                LOG_ERROR("no images or videos defined for camera {}", camera);
            }
        }

        // parse landmarks
        std::map<std::string, std::string> landmarks;
        if (hasCameras)
        {
            if (j.Contains("landmarks"))
            {
                const auto& landmarksMap = j["landmarks"].Map();
                for (const auto& [key, value] : landmarksMap)
                {
                    if (key != CAMERAVAR)
                    {
                        if (std::find(cameras.begin(), cameras.end(), key) == cameras.end())
                        {
                            LOG_WARNING("landmarks are specified for an unknown camera {}", key);
                        }
                        landmarks[key] = makeAbsolute(value.String());
                    }
                }
                auto it = landmarksMap.find(CAMERAVAR);
                if (it != landmarksMap.end())
                {
                    const std::string filenamePattern = it->second.String();
                    for (const std::string& camera : cameras)
                    {
                        if (landmarks.find(camera) == landmarks.end())
                        {
                            const std::string landmarksFilename = makeAbsolute(ReplaceSubstring(filenamePattern, CAMERAVAR, camera));
                            if (std::filesystem::exists(landmarksFilename))
                            {
                                landmarks[camera] = landmarksFilename;
                                LOG_INFO("automatically found landmarks for camera {}: {}", camera, landmarksFilename);
                            }
                        }
                    }
                }
            }
        }

        std::map<std::string, std::string> markers;
        if (hasCameras)
        {
            if (j.Contains("markers"))
            {
                for (const auto& [key, value] : j["markers"].Map())
                {
                    markers[key] = makeAbsolute(value.String());
                }
            }

            if (j.Contains("undistort_landmarks"))
            {
                m_landmarksNeedToBeUndistorted = j["undistort_landmarks"].Boolean();
            }
            else if (j.Contains("landmarks") || j.Contains("markers"))
            {
                LOG_WARNING("Data description should contain information on whether landmarks need to be undistorted i.e. \"undistort_landmarks\": true. All new data should have landmarks that are mapping to the raw distorted images. If landmarks are already undistorted, then this points to a potential processing pipeline issue.");
            }

            if (j.Contains("landmark_frames_are_correct"))
            {
                m_landmarksFrameNumbersAreCorrect = j["landmark_frames_are_correct"].Boolean();
            }
            else if (j.Contains("landmarks") || j.Contains("markers"))
            {
                LOG_WARNING("Data description should contain information on whether landmarks are saved with the correct frame index in the json files i.e. \"landmark_frames_are_correct\": true. If false, which is the default if the key is missing, then landmarks are loaded sequentially and map to [ImageStart, ImageEnd]. All new landmark data should have the correct frame numbers, otherwise this points to a potential processing pipeline issue.");
            }
        }
        // parse scans
        if (j.Contains("scans")) {
            m_scanBase = makeAbsolute(j["scans"]["base"].String());
            m_scanMeshPattern = replaceVariables(j["scans"]["mesh"].String());
            if (j["scans"].Contains("texture")) {
                m_scanTexturePattern = replaceVariables(j["scans"]["texture"].String());
            }
            if (j["scans"].Contains("offset")) {
                m_scanIndexOffset = j["scans"]["offset"].Get<int>();
            }
            auto JsonToVec3 = [](const carbon::JsonElement& json) -> Eigen::Vector3f {
                const std::vector<float> vecData = json.Get<std::vector<float>>();
                if (vecData.size() == 3) {
                    return Eigen::Map<const Eigen::Vector3f>(vecData.data());
                } else {
                    CARBON_CRITICAL("vector needs to be an array of size 3");
                }
            };
            if (j["scans"].Contains("up")) {
                m_scanUpVector = JsonToVec3(j["scans"]["up"]);
            }
            if (j["scans"].Contains("front")) {
                m_scanFrontVector = JsonToVec3(j["scans"]["front"]);
            }
        }

        std::map<std::string, std::string> depthmapPatternMapping {};
        std::map<std::string, std::string> flowPatternMapping {};

        if (hasCameras)
        {
            // parse orientations
            if (j.Contains("orientations"))
            {
                for (const auto& [cameraName, orientationJson] : j["orientations"].Map())
                {
                    m_cameraOrientations[cameraName] = orientationJson.Get<int>();
                }
            }

            if (j.Contains("depthmaps"))
            {
                for (const auto& [key, value] : j["depthmaps"].Map())
                {
                    depthmapPatternMapping[key] = makeAbsolute(value.String(), "CAMERA", key);
                }
            }

            if (j.Contains("flow"))
            {
                for (const auto& [key, value] : j["flow"].Map())
                {
                    flowPatternMapping[key] = makeAbsolute(value.String(), "CAMERA", key);
                }
            }

            if (j.Contains("timecode"))
            {
                m_timecode = carbon::Timecode(j["timecode"]["hour"].Get<int>(),
                    j["timecode"]["min"].Get<int>(),
                    j["timecode"]["sec"].Get<int>(),
                    j["timecode"]["frame"].Get<int>(),
                    j["timecode"]["fps"].Get<double>());
            }
            else if (j.Contains("fps"))
            {
                m_timecode = carbon::Timecode(0, 0, 0, 0, j["fps"].Get<double>());
            }
        }

        m_cameras = cameras;
        m_calibrationFile = calibrationFile;
        m_cameraCalibrationLabelMapping = cameraCalibrationLabelMapping;
        m_imageStart = imageStart;
        m_imageEnd = imageEnd;
        m_cameraPatternMapping = cameraPatternMapping;
        m_cameraVideos = cameraVideos;
        m_landmarks = landmarks;
        m_markers = markers;
        m_depthmapPatternMapping = depthmapPatternMapping;
        m_flowPatternMapping = flowPatternMapping;
    }
    catch(std::exception& e)
    {
        LOG_ERROR("failure to load data description: {}", e.what());
        return false;
    }

    return true;
}

std::string DataDescription::CalibrationCameraLabel(const std::string& cameraName) const
{
    auto it = m_cameraCalibrationLabelMapping.find(cameraName);
    if (it != m_cameraCalibrationLabelMapping.end()) {
        return it->second;
    } else {
        CARBON_CRITICAL("no calibration label for camera {}", cameraName);
    }
}

std::vector<MetaShapeCamera<float>> DataDescription::LoadCameras() const
{
    // load camera calibration and rename
    std::vector<MetaShapeCamera<float>> cameras;
    const std::string& calibrationFile = m_calibrationFile;
    if (NumCameras() == 0)
    {
        return std::vector<MetaShapeCamera<float>>(0);
    }
    const std::string& suffix = (calibrationFile.size() >= 4) ? calibrationFile.substr(calibrationFile.size() - 4) : "";
    if (suffix == "json") {
        if (!ReadMetaShapeCamerasFromJsonFile(m_calibrationFile, cameras)) {
            CARBON_CRITICAL("failed to read cameras from {}", m_calibrationFile);
        }
    } else if (suffix == ".xml") {
        if (!ReadMetaShapeCameras(m_calibrationFile, cameras, Affine<float, 3, 3>(), 1.0f)) {
            CARBON_CRITICAL("failed to read cameras from {}", m_calibrationFile);
        }
    } else {
        CARBON_CRITICAL("cannot read calibration file {}", calibrationFile);
    }

    // rename cameras based on data description
    std::vector<bool> camerasUsed(cameras.size(), false);
    std::vector<bool> camerasInCalibration(m_cameras.size(), false);
    std::vector<MetaShapeCamera<float>> outputCameras;
    for (size_t j = 0; j < m_cameras.size(); ++j) {
        const std::string& cameraName = m_cameras[j];
        const std::string& cameraNameInCalibrationFile = CalibrationCameraLabel(cameraName);
        for (size_t i = 0; i < cameras.size(); ++i) {
            if (cameras[i].Label() == cameraNameInCalibrationFile) {
                camerasUsed[i] = true;
                camerasInCalibration[j] = true;
                MetaShapeCamera<float> outputCamera = cameras[i];
                outputCamera.SetLabel(cameraName);
                outputCameras.emplace_back(outputCamera);
                break;
            }
        }
    }
    int numUnusedCameras = 0;
    for (size_t i = 0; i < cameras.size(); ++i) {
        if (!camerasUsed[i]) {
            numUnusedCameras++;
        }
    }
    if (numUnusedCameras > 0) {
        LOG_WARNING("{} cameras in calibration file are not being used by the data description", numUnusedCameras);
    }
    for (size_t j = 0; j < camerasInCalibration.size(); ++j) {
        if (!camerasInCalibration[j]) {
            LOG_WARNING("no calibration for camera {}", m_cameras[j]);
        }
    }
    return outputCameras;
}

std::map<std::string, std::pair<float, float>> DataDescription::CameraRanges() const
{
    std::map<std::string, std::pair<float, float>> cameraRanges;
    for (const std::string& cameraName : m_cameras) {
        cameraRanges[cameraName] = m_cameraDepthRange.value();
    }
    return cameraRanges;
}

std::string DataDescription::ImageFilename(const std::string& cameraName, int imageIndex) const
{
    if (imageIndex < m_imageStart || imageIndex >= m_imageEnd) {
        CARBON_CRITICAL("index out of bounds: {} not in [{}, {})", imageIndex, m_imageStart, m_imageEnd);
    }
    auto it = m_cameraPatternMapping.find(cameraName);
    if (it != m_cameraPatternMapping.end()) {
        const std::string fullfilename = it->second;
        return StringFormat(fullfilename, imageIndex);
    } else {
        CARBON_CRITICAL("no images for camera {}", cameraName);
    }
}

//! checks that there is a pattern and label for each camera, and optionally if every file exists
bool DataDescription::CheckValidity(bool checkFilesExist)
{
    for (int i = 0; i < int(m_cameras.size()); i++) {
        const std::string cameraName = m_cameras[i];
        if (m_cameraCalibrationLabelMapping.find(cameraName) == m_cameraCalibrationLabelMapping.end()) {
            LOG_ERROR("camera {} has no file calibration label associated with it", cameraName);
            return false;
        }
        if (m_cameraPatternMapping.find(cameraName) == m_cameraPatternMapping.end() &&
            m_cameraVideos.find(cameraName) == m_cameraVideos.end()) {
            LOG_ERROR("camera {} has no image or video pattern associated with it", cameraName);
            return false;
        }

        if (checkFilesExist) {
            for (int k = m_imageStart; k < m_imageEnd; k++) {
                const std::string filename = ImageFilename(cameraName, k);
                if (!std::filesystem::exists(filename)) {
                    LOG_ERROR("file {} does not exist", filename);
                    return false;
                }
            }
        }
    }

    return true;
}

bool DataDescription::HasVideo(const std::string& cameraName) const
{
    return m_cameraVideos.find(cameraName) != m_cameraVideos.end();
}

std::string DataDescription::CameraVideo(const std::string& cameraName) const
{
    auto it = m_cameraVideos.find(cameraName);
    if (it != m_cameraVideos.end()) {
        return it->second;
    } else {
        CARBON_CRITICAL("no video for camera {}", cameraName);
    }
}

bool DataDescription::HasLandmarks(const std::string& cameraName) const
{
    return m_landmarks.find(cameraName) != m_landmarks.end();
}

std::string DataDescription::Landmarks(const std::string& cameraName) const
{
    auto it = m_landmarks.find(cameraName);
    if (it != m_landmarks.end()) {
        return it->second;
    } else {
        CARBON_CRITICAL("no landmarks for camera {}", cameraName);
    }
}

std::string DataDescription::ScanMeshFilename(int index) const
{
    if ((NumCameras() != 0) && (index < m_imageStart || index >= m_imageEnd))
    {
        CARBON_CRITICAL("index out of bounds: {} not in [{}, {})", index, m_imageStart, m_imageEnd);
    }

    return StringFormat(m_scanBase + m_scanMeshPattern, index + m_scanIndexOffset);
}

std::string DataDescription::ScanTextureFilename(int index) const
{
    if ((NumCameras() != 0) && (index < m_imageStart || index >= m_imageEnd))
    {
        CARBON_CRITICAL("index out of bounds: {} not in [{}, {})", index, m_imageStart, m_imageEnd);
    }

    return StringFormat(m_scanBase + m_scanTexturePattern, index + m_scanIndexOffset);
}

bool DataDescription::HasCameraOrientation(const std::string& cameraName) const
{
    return (m_cameraOrientations.find(cameraName) != m_cameraOrientations.end());
}

int DataDescription::CameraOrientation(const std::string& cameraName) const
{
    auto it = m_cameraOrientations.find(cameraName);
    if (it != m_cameraOrientations.end()) {
        return it->second;
    } else {
        CARBON_CRITICAL("no orientation for camera {}", cameraName);
    }
}

bool DataDescription::HasDepthmaps(const std::string& cameraName) const
{
    return m_depthmapPatternMapping.find(cameraName) != m_depthmapPatternMapping.end();
}

std::string DataDescription::DepthmapFilename(const std::string& cameraName, int imageIndex) const
{
    if (imageIndex < m_imageStart || imageIndex >= m_imageEnd) {
        CARBON_CRITICAL("index out of bounds: {} not in [{}, {})", imageIndex, m_imageStart, m_imageEnd);
    }
    auto it = m_depthmapPatternMapping.find(cameraName);
    if (it != m_depthmapPatternMapping.end()) {
        const std::string pattern = it->second;
        return StringFormat(pattern, imageIndex);
    } else {
        CARBON_CRITICAL("no depthmap images for camera {}", cameraName);
    }
}


bool DataDescription::HasFlow(const std::string& cameraName) const
{
    return m_flowPatternMapping.find(cameraName) != m_flowPatternMapping.end();
}

std::string DataDescription::FlowFilename(const std::string& cameraName, int srcImageIndex, int targetImageIndex) const
{
    if (srcImageIndex < m_imageStart || srcImageIndex >= m_imageEnd) {
        CARBON_CRITICAL("index out of bounds: {} not in [{}, {})", srcImageIndex, m_imageStart, m_imageEnd);
    }
    if (targetImageIndex < m_imageStart || targetImageIndex >= m_imageEnd) {
        CARBON_CRITICAL("index out of bounds: {} not in [{}, {})", targetImageIndex, m_imageStart, m_imageEnd);
    }
    auto it = m_flowPatternMapping.find(cameraName);
    if (it != m_flowPatternMapping.end()) {
        const std::string pattern = it->second;
        return StringFormat(pattern, srcImageIndex, targetImageIndex);
    } else {
        CARBON_CRITICAL("no flow images for camera {}", cameraName);
    }
}

} //namespace epic::nls
