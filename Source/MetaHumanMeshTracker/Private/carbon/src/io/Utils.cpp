// Copyright Epic Games, Inc. All Rights Reserved.

#include "Utils.h"

#include <cstdlib>

#include <ctime>
#include <chrono>

namespace epic {
namespace carbon {

std::string ReadFile(const std::string& filename) {
    FILE* pFile = nullptr;
    #ifdef _MSC_VER
        if (fopen_s(&pFile, filename.c_str(), "rb") == 0 && pFile != nullptr) {
    #else
        pFile = fopen(filename.c_str(), "rb");
        if (pFile) {
            #endif
            fseek(pFile, 0, SEEK_END);
            const long pos = ftell(pFile);
            std::string data;
            data.resize(pos);
            fseek(pFile, 0, SEEK_SET);
            if (fread(&data[0], 1, pos, pFile) != size_t(pos)) {
                CARBON_CRITICAL("failed to read all data of file {}", filename);
            }
            fclose(pFile);
            return data;
        }
        CARBON_CRITICAL("no file {}", filename);
}

void WriteFile(const std::string& filename, const std::string& data) {
    std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary);
    if (out) {
        out.write(data.c_str(), data.size());
        if (!out) {
            CARBON_CRITICAL("Cannot write to file at \"{}\"", filename);
        }
        out.close();
        return;
    }
    CARBON_CRITICAL("Cannot write to file at \"{}\"", filename);
}

std::string GetUsername() {
    std::string key;
    #if defined(_WIN32)
        key = "USERNAME";
    #else
        key = "USER";
    #endif

    #ifdef _MSC_VER
        #pragma warning(push)
        #pragma warning(disable:4996)
    #endif
    return std::getenv(key.c_str());
    #ifdef _MSC_VER
        #pragma warning(pop)
    #endif
}

JsonElement WriteFileinfo() {
    std::string username, currentTime;
    try {
        auto ctime = std::chrono::system_clock::now();
        std::time_t timet = std::chrono::system_clock::to_time_t(ctime);
        #ifdef _MSC_VER
            #pragma warning(push)
            #pragma warning(disable:4996)
        #endif
        currentTime = std::ctime(&timet);
        #ifdef _MSC_VER
            #pragma warning(pop)
        #endif
        currentTime.pop_back();
        username = GetUsername();
    } catch (std::exception& e) {
        std::string errmsg = "Failure to write camera data to json. Cannot Get username and/or current time. Error: " +
            std::string(e.what());
        throw std::runtime_error(errmsg);
    }

    JsonElement fileinfo(carbon::JsonElement::JsonType::Object);
    fileinfo.Insert("type", carbon::JsonElement("file"));
    fileinfo.Insert("created", carbon::JsonElement(currentTime));
    fileinfo.Insert("user", carbon::JsonElement(username));
    // TODO: Implement calibration metadata structure
    // fileinfo["metadata"]["metric_unit"]
    // fileinfo["mean_squared_error"]

    return fileinfo;
}

template<class T>
void CreateAndAddElementXml(XMLElement& parent, const char* name, T value) {
    XMLElement& p = parent.AddChild(name);
    p.SetText(carbon::fmt::to_string(value));
}

double GetDoubleValueFromElementIfExists(XMLElement* node, const std::string& name) {
    double value = 0;
    XMLElement* child = node->UniqueChild<  /*failIfNotUnique=*/ false>(name);
    if (child) {
        value = std::stod(child->Text());
    }
    return value;
}

void ReadCameraMetadataJson(const JsonElement& json, std::string& label, std::string& model, int& width, int& height) {
    if (!json.Contains("image_size_x") || !json.Contains("image_size_y") || !json.Contains("metadata")) {
        throw std::runtime_error("Camera calibration is missing camera information (name, size, width, height).");
    }

    label = json["metadata"]["name"].Get<std::string>();
    model = json["metadata"]["camera"].Get<std::string>();
    width = json["image_size_x"].Get<int>();
    height = json["image_size_y"].Get<int>();
}

void ReadSensorMetadataXml(XMLElement* xmlSensor, int& width, int& height, int& id) {
    id = std::stoi(xmlSensor->Attribute("id"));
    XMLElement* xmlResolution = xmlSensor->UniqueChild("resolution");
    width = std::stoi(xmlResolution->Attribute("width"));
    height = std::stoi(xmlResolution->Attribute("height"));
}

void WriteCameraMetadataJson(JsonElement& json, int width, int height, const std::string& label, const std::string& model) {
    JsonElement cameraInfo(carbon::JsonElement::JsonType::Object);

    cameraInfo.Insert("type", carbon::JsonElement("camera"));
    cameraInfo.Insert("version", carbon::JsonElement("0"));
    cameraInfo.Insert("name", carbon::JsonElement(label));
    cameraInfo.Insert("camera", carbon::JsonElement(model));
    json.Insert("image_size_x", carbon::JsonElement(width));
    json.Insert("image_size_y", carbon::JsonElement(height));
    json.Insert("metadata", std::move(cameraInfo));
}

void WriteSensorMetadataXml(XMLElement& xmlSensor, int width, int height, int id) {
    xmlSensor.AddAttribute("id", carbon::fmt::to_string(id));
    XMLElement& xmlResolution = xmlSensor.AddChild("resolution");
    xmlResolution.AddAttribute("width", carbon::fmt::to_string(width));
    xmlResolution.AddAttribute("height", carbon::fmt::to_string(height));
}

void WriteCameraMetadataXml(XMLElement& xmlCamera, const std::string& label, int id) {
    xmlCamera.AddAttribute("sensor_id", carbon::fmt::to_string(id));
    xmlCamera.AddAttribute("label", label);
    xmlCamera.AddAttribute("id", carbon::fmt::to_string(id));
}

template<class T>
void WriteIntrinsicsJson(JsonElement& json, const Eigen::Matrix3<T>& intrinsics) {
    json.Insert("fx", carbon::JsonElement(intrinsics(0, 0)));
    json.Insert("fy", carbon::JsonElement(intrinsics(1, 1)));
    json.Insert("cx", carbon::JsonElement(intrinsics(0, 2)));
    json.Insert("cy", carbon::JsonElement(intrinsics(1, 2)));
}

template<class T>
void WriteIntrinsicsXml(XMLElement& xmlCalibration, const Eigen::Matrix3<T>& intrinsics, int width, int height) {
    CreateAndAddElementXml<T>(xmlCalibration, "f", intrinsics(1, 1));
    CreateAndAddElementXml<T>(xmlCalibration, "cx", intrinsics(0, 2) - width / T(2));
    CreateAndAddElementXml<T>(xmlCalibration, "cy", intrinsics(1, 2) - height / T(2));
    CreateAndAddElementXml<T>(xmlCalibration, "b1", intrinsics(0, 0) - intrinsics(1, 1));
    CreateAndAddElementXml<T>(xmlCalibration, "b2", intrinsics(0, 1));
}

template<class T>
void ReadIntrinsicsJson(const JsonElement& json, Eigen::Matrix3<T>& intrinsics) {
    intrinsics = Eigen::Matrix<T, 3, 3>::Identity();

    if (!json.Contains("fx") || !json.Contains("fy") || !json.Contains("cx") || !json.Contains("cy")) {
        throw std::runtime_error("Camera calibration is missing one of fx, fy, cx, cy");
    }

    intrinsics(0, 0) = json["fx"].Get<T>();
    intrinsics(1, 1) = json["fy"].Get<T>();
    intrinsics(0, 2) = json["cx"].Get<T>();
    intrinsics(1, 2) = json["cy"].Get<T>();
}

template<class T>
void ReadIntrinsicsXml(XMLElement* xmlCalibration, Eigen::Matrix3<T>& intrinsics, int width, int height) {
    intrinsics = Eigen::Matrix<T, 3, 3>::Identity();

    const double f = GetDoubleValueFromElementIfExists(xmlCalibration, "f");
    const double cx = GetDoubleValueFromElementIfExists(xmlCalibration, "cx");
    const double cy = GetDoubleValueFromElementIfExists(xmlCalibration, "cy");
    const double b1 = GetDoubleValueFromElementIfExists(xmlCalibration, "b1");
    const double b2 = GetDoubleValueFromElementIfExists(xmlCalibration, "b2");

    intrinsics(0, 0) = T(f + b1);
    intrinsics(1, 1) = T(f);
    intrinsics(0, 1) = T(b2);
    intrinsics(0, 2) = T(width) / T(2) + T(cx);
    intrinsics(1, 2) = T(height) / T(2) + T(cy);
}

template<class T>
JsonElement WriteExtrinsicsJson(const Eigen::Matrix<T, 3, 4>& transform) {
    JsonElement cameraTransform(carbon::JsonElement::JsonType::Array);
    Eigen::Matrix<T, 4, 4> aff = Eigen::Matrix<T, 4, 4>::Identity();
    aff.topLeftCorner(3, 4) = transform;

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            cameraTransform.Append(carbon::JsonElement(aff(r, c)));
        }
    }
    return cameraTransform;
}

template<class T>
void WriteExtrinsicsXml(XMLElement& xmlCamera, const Eigen::Matrix<T, 3, 4>& transform) {
    XMLElement& xmlTransform = xmlCamera.AddChild("transform");
    Eigen::Matrix4d aff = Eigen::Matrix4d::Identity();
    auto cExtr = transform.template cast<double>();
    aff.topLeftCorner(3, 4) = cExtr;
    auto affInverse = aff.inverse();

    std::stringstream ss;
    for (int j = 0; j < 4; ++j) {
        for (int k = 0; k < 4; ++k) {
            ss << std::scientific << std::setprecision(12) << affInverse(j, k) << " ";
        }
    }
    xmlTransform.SetText(ss.str());
}

template<class T>
void ReadExtrinsicsJson(const JsonElement& json, Eigen::Matrix<T, 3, 4>& transform) {
    if (!json.Contains("transform")) {
        throw std::runtime_error("Camera calibration is missing extrinsic matrix.");
    }
    for (size_t c = 0; c < 4; ++c) {
        for (size_t r = 0; r < 3; ++r) {
            transform(r, c) = json["transform"][4 * r + c].Get<T>();
        }
    }
}

template<class T>
void ReadGlobalTransformXml(XMLElement* xmlChunk, Eigen::Matrix4d& transform, T& scale) {
    scale = T(1);
    transform = Eigen::Matrix4d::Identity();

    XMLElement* transformElement = xmlChunk->UniqueChild<  /*failIfNotUnique=*/ false>("transform");
    if (transformElement) {
        XMLElement* rotationElement = transformElement->UniqueChild<  /*failIfNotUnique=*/ false>("rotation");
        if (rotationElement) {
            std::stringstream ss;
            ss << rotationElement->Text();
            for (int j = 0; j < 3; j++) {
                for (int i = 0; i < 3; i++) {
                    double t;
                    ss >> t;
                    transform(j, i) = t;
                }
            }
        }
        XMLElement* translationElement =
            transformElement->UniqueChild<  /*failIfNotUnique=*/ false>("translation");
        if (translationElement) {
            std::stringstream ss;
            ss << translationElement->Text();
            Eigen::Vector3d translation;
            for (int j = 0; j < 3; j++) {
                double t;
                ss >> t;
                transform(j, 3) = t;
            }
        }
        XMLElement* scaleElement = transformElement->UniqueChild<  /*failIfNotUnique=*/ false>("scale");
        if (scaleElement) {
            scale = T(std::stod(scaleElement->Text()));
        }
    }
}

template<class T>
void ReadExtrinsicsXml(XMLElement* xmlCamera,
                       const Eigen::Matrix4d& globalTransform,
                       Eigen::Matrix<T, 3, 4>& cameraExtrinsics,
                       T scale) {
    XMLElement* cameraTransformElement = xmlCamera->UniqueChild<  /*failIfNotUnique=*/ false>("transform");
    if (cameraTransformElement) {
        std::stringstream ss;
        ss << cameraTransformElement->Text();
        Eigen::Matrix4d transform;
        for (int j = 0; j < 4; j++) {
            for (int i = 0; i < 4; i++) {
                double t;
                ss >> t;
                transform(j, i) = t;
            }
        }

        transform(0, 3) *= scale;
        transform(1, 3) *= scale;
        transform(2, 3) *= scale;
        transform = globalTransform * transform;

        // the camera is the matrix going from world to camera, hence we need an inverse
        cameraExtrinsics = transform.inverse().topLeftCorner(3, 4).cast<T>();
    }
}

template<class T>
void WriteDistortionOpenCVJson(JsonElement& json, const Eigen::Vector<T, 5>& distortion) {
    json.Insert("distortion_model", carbon::JsonElement("opencv"));
    json.Insert("k1", carbon::JsonElement(distortion[0]));
    json.Insert("k2", carbon::JsonElement(distortion[1]));
    json.Insert("p1", carbon::JsonElement(distortion[2]));
    json.Insert("p2", carbon::JsonElement(distortion[3]));
    json.Insert("k3", carbon::JsonElement(distortion[4]));
}

template<class T>
void WriteDistortionOpenCVXml(XMLElement& xmlCalibration, const Eigen::Vector<T, 5>& distortion) {
    CreateAndAddElementXml<T>(xmlCalibration, "k1", distortion(0));
    CreateAndAddElementXml<T>(xmlCalibration, "k2", distortion(1));
    CreateAndAddElementXml<T>(xmlCalibration, "k3", distortion(4));
    CreateAndAddElementXml<T>(xmlCalibration, "p1", distortion(2));
    CreateAndAddElementXml<T>(xmlCalibration, "p2", distortion(3));
}

template<class T>
void ReadDistortionOpenCVJson(const JsonElement& json, Eigen::Vector<T, 5>& distortion) {
    if (!json.Contains("distortion_model")) {
        throw std::runtime_error("Camera calibration does not contain distortion model.");
    }

    if (json["distortion_model"].Get<std::string>() == "opencv") {
        auto valueOrNull = [](const JsonElement& j, const char* label) {
                return j.Contains(label) ? j[label].Get<T>() : T(0);
            };
        const T k1 = valueOrNull(json, "k1");
        const T k2 = valueOrNull(json, "k2");
        const T k3 = valueOrNull(json, "k3");
        const T k4 = valueOrNull(json, "k4");
        const T k5 = valueOrNull(json, "k5");
        const T k6 = valueOrNull(json, "k6");
        const T p1 = valueOrNull(json, "p1");
        const T p2 = valueOrNull(json, "p2");
        if ((k4 != T(0)) || (k5 != T(0)) || (k6 != T(0))) {
            throw std::runtime_error("Currently, camera model does not support k4, k5, and k6 parameter.");
        }
        distortion[0] = k1;
        distortion[1] = k2;
        distortion[2] = p1;
        distortion[3] = p2;
        distortion[4] = k3;
    } else {
        throw std::runtime_error("Distortion model is not OpenCV.");
    }
}

template<class T>
void ReadDistortionOpenCvXml(XMLElement* xmlCalibration, Eigen::Vector<T, 5>& distortion) {
    const double k1 = GetDoubleValueFromElementIfExists(xmlCalibration, "k1");
    const double k2 = GetDoubleValueFromElementIfExists(xmlCalibration, "k2");
    const double k3 = GetDoubleValueFromElementIfExists(xmlCalibration, "k3");
    const double p1 = GetDoubleValueFromElementIfExists(xmlCalibration, "p1");
    const double p2 = GetDoubleValueFromElementIfExists(xmlCalibration, "p2");

    distortion[0] = T(k1);
    distortion[1] = T(k2);
    distortion[2] = T(p1);
    distortion[3] = T(p2);
    distortion[4] = T(k3);
}

template void WriteIntrinsicsJson(JsonElement& json, const Eigen::Matrix3<float>& intrinsics);
template void WriteIntrinsicsJson(JsonElement& json, const Eigen::Matrix3<double>& intrinsics);

template void WriteIntrinsicsXml(XMLElement& xmlCalibration, const Eigen::Matrix3<float>& intrinsics, int width, int height);
template void WriteIntrinsicsXml(XMLElement& xmlCalibration, const Eigen::Matrix3<double>& intrinsics, int width, int height);

template void ReadIntrinsicsJson(const JsonElement& json, Eigen::Matrix3<float>& intrinsics);
template void ReadIntrinsicsJson(const JsonElement& json, Eigen::Matrix3<double>& intrinsics);

template void ReadIntrinsicsXml(XMLElement* xml, Eigen::Matrix3<float>& intrinsics, int width, int height);
template void ReadIntrinsicsXml(XMLElement* xml, Eigen::Matrix3<double>& intrinsics, int width, int height);

template JsonElement WriteExtrinsicsJson(const Eigen::Matrix<float, 3, 4>& intrinsics);
template JsonElement WriteExtrinsicsJson(const Eigen::Matrix<double, 3, 4>& intrinsics);

template void WriteExtrinsicsXml(XMLElement& xmlCamera, const Eigen::Matrix<float, 3, 4>& transform);
template void WriteExtrinsicsXml(XMLElement& xmlCamera, const Eigen::Matrix<double, 3, 4>& transform);

template void ReadExtrinsicsJson(const JsonElement& json, Eigen::Matrix<float, 3, 4>& transform);
template void ReadExtrinsicsJson(const JsonElement& json, Eigen::Matrix<double, 3, 4>& transform);

template void ReadGlobalTransformXml(XMLElement* xmlChunk, Eigen::Matrix4d& transform, float& scale);
template void ReadGlobalTransformXml(XMLElement* xmlChunk, Eigen::Matrix4d& transform, double& scale);

template void ReadExtrinsicsXml(XMLElement* xmlCamera,
                                const Eigen::Matrix4d& globalTransform,
                                Eigen::Matrix<float, 3,
                                              4>& cameraExtrinsics,
                                float scale);
template void ReadExtrinsicsXml(XMLElement* xmlCamera,
                                const Eigen::Matrix4d& globalTransform,
                                Eigen::Matrix<double,
                                              3,
                                              4>& cameraExtrinsics,
                                double scale);

template void WriteDistortionOpenCVJson(JsonElement& json, const Eigen::Vector<float, 5>& distortion);
template void WriteDistortionOpenCVJson(JsonElement& json, const Eigen::Vector<double, 5>& distortion);

template void WriteDistortionOpenCVXml(XMLElement& json, const Eigen::Vector<float, 5>& distortion);
template void WriteDistortionOpenCVXml(XMLElement& json, const Eigen::Vector<double, 5>& distortion);

template void ReadDistortionOpenCVJson(const JsonElement& json, Eigen::Vector<float, 5>& distortion);
template void ReadDistortionOpenCVJson(const JsonElement& json, Eigen::Vector<double, 5>& distortion);

template void ReadDistortionOpenCvXml(XMLElement* xmlCalibration, Eigen::Vector<float, 5>& distortion);
template void ReadDistortionOpenCvXml(XMLElement* xmlCalibration, Eigen::Vector<double, 5>& distortion);
}
}
