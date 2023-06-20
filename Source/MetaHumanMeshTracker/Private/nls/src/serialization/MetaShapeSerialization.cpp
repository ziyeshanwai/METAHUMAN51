// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/serialization/MetaShapeSerialization.h>

#include <carbon/io/XmlIO.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/EulerAngles.h>
#include <nls/geometry/MetaShapeCamera.h>
#include <nls/utils/FileIO.h>

#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>

namespace epic {
namespace nls {


template <class T>
bool ReadStabilizationFile(const std::string& filename, Affine<T, 3, 3>& transform, T& scale)
{
    std::ifstream stream(filename);
    if (!stream.is_open()) {
        return false;
    }

    T tx, ty, tz;
    T rx, ry, rz;
    T sx, sy, sz;
    stream >> tx >> ty >> tz >> rx >> ry >> rz >> sx >> sy >> sz;
    if (stream.bad()) {
        return false;
    }

    constexpr T deg2rad = T(CARBON_PI / 180.0);
    transform.SetLinear(EulerXYZ<T>(deg2rad * rx, deg2rad * ry, deg2rad * rz));
    transform.SetTranslation(Eigen::Vector3<T>(tx, ty, tz));
    scale = sx;
    if (fabs(sx - sy) > T(1e-6) || fabs(sx - sz) > T(1e-6)) {
        CARBON_CRITICAL("assymmetric scaling is not supported");
    }

    return true;
}


template <class T>
bool ReadMetaShapeCameras(const std::string& filename,
                          std::vector<MetaShapeCamera<T>>& cameras,
                          const Affine<T, 3, 3>& metashapeToWorldTransform,
                          const T metashapeToWorldScale)
{
    using namespace epic::carbon::xml;

    cameras.clear();

    std::map<int, MetaShapeCamera<T>> sensors;
    {
        XMLElement element = ReadXML(ReadFile(filename));
        if (element.Name() != "document") {
            CARBON_CRITICAL("xml file does not start with root node \"document\", but {}", element.Name());
        }
        XMLElement* xmlChunk = element.UniqueChild("chunk");
        XMLElement* xmlSensors = xmlChunk->UniqueChild("sensors");
        std::vector<XMLElement*> allXmlSensor = xmlSensors->ChildrenWithName("sensor");

        for (auto xmlSensor : allXmlSensor) {
            const int id = std::stoi(xmlSensor->Attribute("id"));
            XMLElement* xmlResolution = xmlSensor->UniqueChild("resolution");
            const int width = std::stoi(xmlResolution->Attribute("width"));
            const int height = std::stoi(xmlResolution->Attribute("height"));
            XMLElement* xmlCalibration = xmlSensor->UniqueChild("calibration");
            auto GetDoubleValueFromElementIfExists = [](XMLElement* node, const std::string& name) {
                double value = 0;
                XMLElement* child = node->UniqueChild</*failIfNotUnique=*/false>(name);
                if (child) {
                    value = std::stod(child->Text());
                }
                return value;
            };
            const double f = GetDoubleValueFromElementIfExists(xmlCalibration, "f");
            const double cx = GetDoubleValueFromElementIfExists(xmlCalibration, "cx");
            const double cy = GetDoubleValueFromElementIfExists(xmlCalibration, "cy");
            const double b1 = GetDoubleValueFromElementIfExists(xmlCalibration, "b1");
            const double b2 = GetDoubleValueFromElementIfExists(xmlCalibration, "b2");
            const double k1 = GetDoubleValueFromElementIfExists(xmlCalibration, "k1");
            const double k2 = GetDoubleValueFromElementIfExists(xmlCalibration, "k2");
            const double k3 = GetDoubleValueFromElementIfExists(xmlCalibration, "k3");
            const double k4 = GetDoubleValueFromElementIfExists(xmlCalibration, "k4");
            const double p1 = GetDoubleValueFromElementIfExists(xmlCalibration, "p1");
            const double p2 = GetDoubleValueFromElementIfExists(xmlCalibration, "p2");
            const double p3 = GetDoubleValueFromElementIfExists(xmlCalibration, "p3");
            const double p4 = GetDoubleValueFromElementIfExists(xmlCalibration, "p4");

            MetaShapeCamera<T> sensor;
            sensor.SetWidth(width);
            sensor.SetHeight(height);
            Eigen::Matrix<T, 3, 3> intrinsics = Eigen::Matrix<T, 3, 3>::Identity();
            intrinsics(0,0) = T(f);
            intrinsics(1,1) = T(f);
            intrinsics(0,2) = T(width)/T(2) + T(cx);
            intrinsics(1,2) = T(height)/T(2) +T(cy);
            sensor.SetIntrinsics(intrinsics);
            sensor.SetRadialDistortion(Eigen::Vector<T, 4>(T(k1), T(k2), T(k3), T(k4)));
            sensor.SetTangentialDistortion(Eigen::Vector<T, 4>(T(p1), T(p2), T(p3), T(p4)));
            sensor.SetSkew(Eigen::Vector<T,2>(T(b1), T(b2)));
            sensor.SetSensorID(id);

            sensors[id] = sensor;
        }

        // get transformation from reconstruction space to export space
        double scale = 1.0;
        Eigen::Matrix4d globalTransform = Eigen::Matrix4d::Identity();

        XMLElement* transformElement = xmlChunk->UniqueChild</*failIfNotUnique=*/false>("transform");
        if (transformElement) {
            XMLElement* rotationElement = transformElement->UniqueChild</*failIfNotUnique=*/false>("rotation");
            if (rotationElement) {
                std::stringstream ss;
                ss << rotationElement->Text();
                for (int j = 0; j < 3; j++) {
                    for (int i = 0; i < 3; i++) {
                        double t;
                        ss >> t;
                        globalTransform(j, i) = t;
                    }
                }
            }
            XMLElement* translationElement = transformElement->UniqueChild</*failIfNotUnique=*/false>("translation");
            if (translationElement) {
                std::stringstream ss;
                ss << translationElement->Text();
                Eigen::Vector3d translation;
                for (int j = 0; j < 3; j++) {
                    double t;
                    ss >> t;
                    globalTransform(j, 3) = t;
                }
            }
            XMLElement* scaleElement = transformElement->UniqueChild</*failIfNotUnique=*/false>("scale");
            if (scaleElement) {
                scale = std::stod(scaleElement->Text());
            }
        }

        XMLElement* xmlCameras = xmlChunk->UniqueChild</*failIfNotUnique=*/false>("cameras");
        std::vector<XMLElement*> allXmlCamera = xmlCameras->ChildrenWithName("camera");
        {
            // support cameras that are under a group element
            std::vector<XMLElement*> allXmlGroup = xmlCameras->ChildrenWithName("group");
            for (XMLElement* xmlGroup : allXmlGroup) {
                std::vector<XMLElement*> allGroupXmlCamera = xmlGroup->ChildrenWithName("camera");
                allXmlCamera.insert(allXmlCamera.end(), allGroupXmlCamera.begin(), allGroupXmlCamera.end());
            }
        }

        for (XMLElement* xmlCamera : allXmlCamera) {
            const int sensorId = std::stoi(xmlCamera->Attribute("sensor_id"));
            if (sensors.find(sensorId) == sensors.end()) {
                CARBON_CRITICAL("sensor id {} does not exist", sensorId);
            }

            MetaShapeCamera<T> camera = sensors[sensorId];

            camera.SetLabel(xmlCamera->Attribute("label"));
            const int cameraId = std::stoi(xmlCamera->Attribute("id"));
            camera.SetCameraID(cameraId);

            XMLElement* cameraTransformElement = xmlCamera->UniqueChild</*failIfNotUnique=*/false>("transform");
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

                // transform is from camera to reconstruction space
                // to go from camera to export space we apply the scale and then the export affine transformation
                transform(0,3) *= scale;
                transform(1,3) *= scale;
                transform(2,3) *= scale;
                transform = globalTransform * transform;
                // there may be an additional user provided transformation from metashape/export to world coordinates
                // again we first apply the scale, then the export space to world space affine transformation
                transform(0,3) *= double(metashapeToWorldScale);
                transform(1,3) *= double(metashapeToWorldScale);
                transform(2,3) *= double(metashapeToWorldScale);
                transform = metashapeToWorldTransform.Matrix().template cast<double>() * transform;

                // the camera is the matrix going from world to camera, hence we need an inverse
                Affine<T, 3, 3> aff;
                aff.SetMatrix(transform.inverse().cast<T>());
                camera.SetExtrinsics(aff);
            }
            cameras.push_back(camera);
        }
    }
    return true;
}

template <class T>
bool WriteMetaShapeCameras(const std::string& filename, const std::vector<MetaShapeCamera<T>>& cameras)
{
    using namespace epic::carbon::xml;

    XMLElement xmlRoot("document");
    XMLElement& xmlChunk = xmlRoot.AddChild("chunk");
    XMLElement& xmlSensors = xmlChunk.AddChild("sensors");
    XMLElement& xmlCameras = xmlChunk.AddChild("cameras");

    auto CreateAndAddElement = [&](XMLElement& parent, const char* name, T value) {
        XMLElement& p = parent.AddChild(name);
        p.SetText(carbon::fmt::to_string(value));
    };

    for (int i = 0; i < int(cameras.size()); ++i) {
        const MetaShapeCamera<T>& camera = cameras[i];
        XMLElement& xmlSensor = xmlSensors.AddChild("sensor");
        xmlSensor.AddAttribute("id", carbon::fmt::to_string(i));
        XMLElement& xmlResolution = xmlSensor.AddChild("resolution");
        xmlResolution.AddAttribute("width", carbon::fmt::to_string(camera.Width()));
        xmlResolution.AddAttribute("height", carbon::fmt::to_string(camera.Height()));
        XMLElement& xmlCalibration = xmlSensor.AddChild("calibration");
        if (camera.Intrinsics()(0,0) != camera.Intrinsics()(1,1)) {
            CARBON_CRITICAL("metashape camera only supports the same focal length for fx and fy");
        }
        CreateAndAddElement(xmlCalibration, "f", camera.Intrinsics()(0,0));
        CreateAndAddElement(xmlCalibration, "cx", camera.Intrinsics()(0,2) - camera.Width() / T(2));
        CreateAndAddElement(xmlCalibration, "cy", camera.Intrinsics()(1,2) - camera.Height() / T(2));
        CreateAndAddElement(xmlCalibration, "k1", camera.RadialDistortion()(0));
        CreateAndAddElement(xmlCalibration, "k2", camera.RadialDistortion()(1));
        CreateAndAddElement(xmlCalibration, "k3", camera.RadialDistortion()(2));
        CreateAndAddElement(xmlCalibration, "k4", camera.RadialDistortion()(3));
        CreateAndAddElement(xmlCalibration, "p1", camera.TangentialDistortion()(0));
        CreateAndAddElement(xmlCalibration, "p2", camera.TangentialDistortion()(1));
        CreateAndAddElement(xmlCalibration, "p3", camera.TangentialDistortion()(2));
        CreateAndAddElement(xmlCalibration, "p4", camera.TangentialDistortion()(3));
        CreateAndAddElement(xmlCalibration, "b1", camera.Skew()(0));
        CreateAndAddElement(xmlCalibration, "b2", camera.Skew()(1));

        XMLElement& xmlCamera = xmlCameras.AddChild("camera");
        xmlCamera.AddAttribute("sensor_id", carbon::fmt::to_string(i));
        xmlCamera.AddAttribute("label", camera.Label());
        xmlCamera.AddAttribute("id", carbon::fmt::to_string(camera.CameraID()));
        XMLElement& xmlTransform = xmlCamera.AddChild("transform");
        Eigen::Matrix<T, 4, 4> aff = camera.Extrinsics().Matrix().inverse();
        std::stringstream ss;
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                ss << aff(j, k)  << " ";
            }
        }
        xmlTransform.SetText(ss.str());
    }

    WriteFile(filename, WriteXML(xmlRoot));

    return true;
}



// explicitly instantiate the methods for float and double
template bool ReadStabilizationFile(const std::string& filename, Affine<float, 3, 3>& transform, float& scale);
template bool ReadStabilizationFile(const std::string& filename, Affine<double, 3, 3>& transform, double& scale);

template bool ReadMetaShapeCameras(const std::string&, std::vector<MetaShapeCamera<float>>&, const Affine<float, 3, 3>&, const float);
template bool ReadMetaShapeCameras(const std::string&, std::vector<MetaShapeCamera<double>>&, const Affine<double, 3, 3>&, const double);

template bool WriteMetaShapeCameras(const std::string&, const std::vector<MetaShapeCamera<float>>&);
template bool WriteMetaShapeCameras(const std::string&, const std::vector<MetaShapeCamera<double>>&);

} // namespace nls
} //namespace epic
