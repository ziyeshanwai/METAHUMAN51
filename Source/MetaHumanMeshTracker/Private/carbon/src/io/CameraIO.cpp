// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/io/CameraIO.h>
#include "Utils.h"

namespace epic {
namespace carbon {
namespace camera {
template<class T>
std::vector<CameraModelOpenCV<T> > ReadOpenCvModelJson(const std::string& filename) {
    std::string data;
    try {
        data = ReadFile(filename.c_str());
    } catch (std::exception& e) {
        std::string msg = "Failed reading JSON file at given path. Error: {}";
        CARBON_CRITICAL(msg, e.what());
    }
    JsonElement json = ReadJson(data);
    if (!json.IsArray()) {
        CARBON_CRITICAL("Json camera file should contains an array as top level.");
    }

    std::vector<CameraModelOpenCV<T> > cameras;

    int frameWidth, frameHeight;
    std::string label, model;
    Eigen::Matrix3<T> intrinsics;
    Eigen::Matrix<T, 3, 4> extrinsics;
    Eigen::Vector<T, 5> distortionOpenCV;

    for (const auto& obj : json.Array()) {
        if (!obj.Contains("metadata")) {
            CARBON_CRITICAL("Json camera file should contain an array of objects that have a metadata field.");
        }
        if (obj["metadata"]["type"].Get<std::string>() == "camera") {
            CameraModelOpenCV<T> camera;

            ReadCameraMetadataJson(obj, label, model, frameWidth, frameHeight);

            camera.SetLabel(label);
            camera.SetModel(model);
            camera.SetWidth(frameWidth);
            camera.SetHeight(frameHeight);

            ReadIntrinsicsJson<T>(obj, intrinsics);
            camera.SetIntrinsics(intrinsics);

            ReadExtrinsicsJson<T>(obj, extrinsics);
            camera.SetExtrinsics(extrinsics);

            ReadDistortionOpenCVJson<T>(obj, distortionOpenCV);
            camera.SetDistortionParams(distortionOpenCV);

            cameras.push_back(camera);
        }
    }

    return cameras;
}

template<class T>
std::vector<CameraModelPinhole<T> > ReadPinholeModelJson(const std::string& filename) {
    std::string data;
    try {
        data = ReadFile(filename.c_str());
    } catch (std::exception& e) {
        std::string msg = "Failed reading JSON file at given path. Error: {}";
        CARBON_CRITICAL(msg, e.what());
    }
    JsonElement json = ReadJson(data);
    if (!json.IsArray()) {
        CARBON_CRITICAL("Json camera file should contains an array as top level.");
    }

    std::vector<CameraModelPinhole<T> > cameras;

    int frameWidth, frameHeight;
    std::string label, model;
    Eigen::Matrix3<T> intrinsics;
    Eigen::Matrix<T, 3, 4> extrinsics;

    for (const auto& obj : json.Array()) {
        if (!obj.Contains("metadata")) {
            CARBON_CRITICAL("Json camera file should contain an array of objects that have a metadata field.");
        }
        if (obj["metadata"]["type"].Get<std::string>() == "camera") {
            CameraModelOpenCV<T> camera;

            ReadCameraMetadataJson(obj, label, model, frameWidth, frameHeight);

            camera.SetLabel(label);
            camera.SetModel(model);
            camera.SetWidth(frameWidth);
            camera.SetHeight(frameHeight);

            ReadIntrinsicsJson<T>(obj, intrinsics);
            camera.SetIntrinsics(intrinsics);

            ReadExtrinsicsJson<T>(obj, extrinsics);
            camera.SetExtrinsics(extrinsics);

            cameras.push_back(camera);
        }
    }

    return cameras;
}

template<class T>
void WriteOpenCvModelJson(const std::string& filename, const std::vector<CameraModelOpenCV<T> >& cameras) {
    JsonElement j(JsonElement::JsonType::Array);

    JsonElement metadataObj(JsonElement::JsonType::Object);
    JsonElement fileinfo = WriteFileinfo();
    metadataObj.Insert("metadata", std::move(fileinfo));

    j.Append(std::move(metadataObj));

    CameraModelOpenCV<T> camera;
    Eigen::Matrix<T, 3, 3> K;
    Eigen::Matrix<T, 3, 4> transform;
    Eigen::Vector<T, 5> D;
    std::string label, model;
    int frameWidth, frameHeight;

    for (size_t i = 0; i < cameras.size(); i++) {
        JsonElement cameraObj(JsonElement::JsonType::Object);
        camera = cameras[i];
        try {
            K = camera.GetIntrinsics();
            D = camera.GetDistortionParams();
            transform = camera.GetExtrinsics();
            label = camera.GetLabel();
            model = camera.GetModel();
            frameWidth = camera.GetWidth();
            frameHeight = camera.GetHeight();
        } catch (std::exception& e) {
            std::string msg = "Failure to write camera data to json. Cannot load camera data. Error: {}";
            CARBON_CRITICAL(msg, e.what());
        }

        WriteCameraMetadataJson(cameraObj, frameWidth, frameHeight, label, model);
        WriteIntrinsicsJson<T>(cameraObj, K);
        WriteDistortionOpenCVJson<T>(cameraObj, D);
        JsonElement cameraExtrinsics = WriteExtrinsicsJson<T>(transform);
        cameraObj.Insert("transform", std::move(cameraExtrinsics));

        j.Append(std::move(cameraObj));
    }

    WriteFile(filename, WriteJson(j,  /*tabs=*/ 1));
}

template<class T>
void WritePinholeModelJson(const std::string& filename, const std::vector<CameraModelPinhole<T> >& cameras) {
    JsonElement j(JsonElement::JsonType::Array);

    JsonElement metadataObj(JsonElement::JsonType::Object);
    JsonElement fileinfo = WriteFileinfo();
    metadataObj.Insert("metadata", std::move(fileinfo));

    j.Append(std::move(metadataObj));

    CameraModelPinhole<T> camera;
    Eigen::Matrix<T, 3, 3> K;
    Eigen::Matrix<T, 3, 4> transform;
    Eigen::Vector<T, 5> D;
    std::string label, model;
    int frameWidth, frameHeight;

    for (size_t i = 0; i < cameras.size(); i++) {
        JsonElement cameraObj(JsonElement::JsonType::Object);
        camera = cameras[i];
        try {
            K = camera.GetIntrinsics();
            transform = camera.GetExtrinsics();
            label = camera.GetLabel();
            model = camera.GetModel();
            frameWidth = camera.GetWidth();
            frameHeight = camera.GetHeight();
        } catch (std::exception& e) {
            std::string msg = "Failure to write camera data to json. Cannot load camera data. Error: {}";
            CARBON_CRITICAL(msg, e.what());
        }

        WriteCameraMetadataJson(cameraObj, frameWidth, frameHeight, label, model);
        WriteIntrinsicsJson<T>(cameraObj, K);
        JsonElement cameraExtrinsics = WriteExtrinsicsJson<T>(transform);
        cameraObj.Insert("transform", std::move(cameraExtrinsics));

        j.Append(std::move(cameraObj));
    }

    WriteFile(filename, WriteJson(j,  /*tabs=*/ 1));
}

template<class T>
std::vector<CameraModelOpenCV<T> > ReadOpenCvModelXml(const std::string& filename) {
    std::vector<CameraModelOpenCV<T> > cameras;
    std::map<int, CameraModelOpenCV<T> > sensors;
    {
        XMLElement element = ReadXML(ReadFile(filename.c_str()));
        if (element.Name() != "document") {
            CARBON_CRITICAL("xml file does not start with root node \"document\", but {}", element.Name());
        }
        XMLElement* xmlChunk = element.UniqueChild("chunk");
        XMLElement* xmlSensors = xmlChunk->UniqueChild("sensors");
        std::vector<XMLElement*> allXmlSensor = xmlSensors->ChildrenWithName("sensor");

        for (auto xmlSensor : allXmlSensor) {
            int id, width, height;
            Eigen::Matrix3<T> intrinsics;
            Eigen::Vector<T, 5> distortion;
            Eigen::Matrix<T, 3, 4> extrinsics;
            ReadSensorMetadataXml(xmlSensor, width, height, id);
            XMLElement* xmlCalibration = xmlSensor->UniqueChild("calibration");

            ReadIntrinsicsXml<T>(xmlCalibration, intrinsics, width, height);
            ReadDistortionOpenCvXml<T>(xmlCalibration, distortion);

            CameraModelOpenCV<T> sensor;
            sensor.SetWidth(width);
            sensor.SetHeight(height);
            sensor.SetIntrinsics(intrinsics);
            sensor.SetDistortionParams(distortion);

            sensors[id] = sensor;
        }

        T scale;
        Eigen::Matrix4d globalTransform;
        Eigen::Matrix<T, 3, 4> transform;
        ReadGlobalTransformXml<T>(xmlChunk, globalTransform, scale);

        XMLElement* xmlCameras = xmlChunk->UniqueChild<  /*failIfNotUnique=*/ false>("cameras");
        std::vector<XMLElement*> allXmlCamera = xmlCameras->ChildrenWithName("camera");

        for (XMLElement* xmlCamera : allXmlCamera) {
            const int sensorId = std::stoi(xmlCamera->Attribute("sensor_id"));
            if (sensors.find(sensorId) == sensors.end()) {
                CARBON_CRITICAL("sensor id {} does not exist", sensorId);
            }

            CameraModelOpenCV<T> camera = sensors[sensorId];
            camera.SetLabel(xmlCamera->Attribute("label"));

            ReadExtrinsicsXml<T>(xmlCamera, globalTransform, transform, scale);
            camera.SetExtrinsics(transform);

            cameras.push_back(camera);
        }
    }
    return cameras;
}

template<class T>
std::vector<CameraModelPinhole<T> > ReadPinholeModelXml(const std::string& filename) {
    std::vector<CameraModelPinhole<T> > cameras;
    std::map<int, CameraModelPinhole<T> > sensors;
    {
        XMLElement element = ReadXML(ReadFile(filename.c_str()));
        if (element.Name() != "document") {
            CARBON_CRITICAL("xml file does not start with root node \"document\", but {}", element.Name());
        }
        XMLElement* xmlChunk = element.UniqueChild("chunk");
        XMLElement* xmlSensors = xmlChunk->UniqueChild("sensors");
        std::vector<XMLElement*> allXmlSensor = xmlSensors->ChildrenWithName("sensor");

        for (auto xmlSensor : allXmlSensor) {
            int id, width, height;
            Eigen::Matrix3<T> intrinsics;
            Eigen::Vector<T, 5> distortion;
            Eigen::Matrix<T, 3, 4> extrinsics;
            ReadSensorMetadataXml(xmlSensor, width, height, id);
            XMLElement* xmlCalibration = xmlSensor->UniqueChild("calibration");

            ReadIntrinsicsXml<T>(xmlCalibration, intrinsics, width, height);

            CameraModelPinhole<T> sensor;
            sensor.SetWidth(width);
            sensor.SetHeight(height);
            sensor.SetIntrinsics(intrinsics);

            sensors[id] = sensor;
        }

        T scale;
        Eigen::Matrix4d globalTransform;
        Eigen::Matrix<T, 3, 4> transform;
        ReadGlobalTransformXml<T>(xmlChunk, globalTransform, scale);

        XMLElement* xmlCameras = xmlChunk->UniqueChild<  /*failIfNotUnique=*/ false>("cameras");
        std::vector<XMLElement*> allXmlCamera = xmlCameras->ChildrenWithName("camera");

        for (XMLElement* xmlCamera : allXmlCamera) {
            const int sensorId = std::stoi(xmlCamera->Attribute("sensor_id"));
            if (sensors.find(sensorId) == sensors.end()) {
                CARBON_CRITICAL("sensor id {} does not exist", sensorId);
            }

            CameraModelPinhole<T> camera = sensors[sensorId];
            camera.SetLabel(xmlCamera->Attribute("label"));

            ReadExtrinsicsXml<T>(xmlCamera, globalTransform, transform, scale);
            camera.SetExtrinsics(transform);

            cameras.push_back(camera);
        }
    }
    return cameras;
}

template<class T>
void WriteOpenCvModelXml(const std::string& filename, const std::vector<CameraModelOpenCV<T> >& cameras) {
    XMLElement xmlRoot("document");
    XMLElement& xmlChunk = xmlRoot.AddChild("chunk");
    XMLElement& xmlSensors = xmlChunk.AddChild("sensors");
    XMLElement& xmlCameras = xmlChunk.AddChild("cameras");

    for (int i = 0; i < int(cameras.size()); ++i) {
        const CameraModelOpenCV<T>& camera = cameras[i];
        XMLElement& xmlSensor = xmlSensors.AddChild("sensor");
        WriteSensorMetadataXml(xmlSensor, camera.GetWidth(), camera.GetHeight(), i);

        XMLElement& xmlCalibration = xmlSensor.AddChild("calibration");

        WriteIntrinsicsXml<T>(xmlCalibration, camera.GetIntrinsics(), camera.GetWidth(), camera.GetHeight());
        WriteDistortionOpenCVXml<T>(xmlCalibration, camera.GetDistortionParams());

        XMLElement& xmlCamera = xmlCameras.AddChild("camera");
        WriteCameraMetadataXml(xmlCamera, camera.GetLabel(), i);
        WriteExtrinsicsXml<T>(xmlCamera, camera.GetExtrinsics());
    }

    WriteFile(filename, WriteXML(xmlRoot));
}

template<class T>
void WritePinholeModelXml(const std::string& filename, const std::vector<CameraModelPinhole<T> >& cameras) {
    XMLElement xmlRoot("document");
    XMLElement& xmlChunk = xmlRoot.AddChild("chunk");
    XMLElement& xmlSensors = xmlChunk.AddChild("sensors");
    XMLElement& xmlCameras = xmlChunk.AddChild("cameras");

    for (int i = 0; i < int(cameras.size()); ++i) {
        const CameraModelPinhole<T>& camera = cameras[i];
        XMLElement& xmlSensor = xmlSensors.AddChild("sensor");
        WriteSensorMetadataXml(xmlSensor, camera.GetWidth(), camera.GetHeight(), i);

        XMLElement& xmlCalibration = xmlSensor.AddChild("calibration");

        WriteIntrinsicsXml<T>(xmlCalibration, camera.GetIntrinsics(), camera.GetWidth(), camera.GetHeight());

        XMLElement& xmlCamera = xmlCameras.AddChild("camera");
        WriteCameraMetadataXml(xmlCamera, camera.GetLabel(), i);
        WriteExtrinsicsXml<T>(xmlCamera, camera.GetExtrinsics());
    }

    WriteFile(filename, WriteXML(xmlRoot));
}

template std::vector<CameraModelOpenCV<float> > ReadOpenCvModelJson(const std::string& filename);
template std::vector<CameraModelOpenCV<double> > ReadOpenCvModelJson(const std::string& filename);
template std::vector<CameraModelPinhole<float> > ReadPinholeModelJson(const std::string& filename);
template std::vector<CameraModelPinhole<double> > ReadPinholeModelJson(const std::string& filename);

template std::vector<CameraModelOpenCV<float> > ReadOpenCvModelXml(const std::string& filename);
template std::vector<CameraModelOpenCV<double> > ReadOpenCvModelXml(const std::string& filename);
template std::vector<CameraModelPinhole<float> > ReadPinholeModelXml(const std::string& filename);
template std::vector<CameraModelPinhole<double> > ReadPinholeModelXml(const std::string& filename);

template void WriteOpenCvModelJson(const std::string& filename, const std::vector<CameraModelOpenCV<float> >& cameras);
template void WriteOpenCvModelJson(const std::string& filename, const std::vector<CameraModelOpenCV<double> >& cameras);
template void WritePinholeModelJson(const std::string& filename, const std::vector<CameraModelPinhole<float> >& cameras);
template void WritePinholeModelJson(const std::string& filename, const std::vector<CameraModelPinhole<double> >& cameras);

template void WriteOpenCvModelXml(const std::string& filename, const std::vector<CameraModelOpenCV<float> >& cameras);
template void WriteOpenCvModelXml(const std::string& filename, const std::vector<CameraModelOpenCV<double> >& cameras);
template void WritePinholeModelXml(const std::string& filename, const std::vector<CameraModelPinhole<float> >& cameras);
template void WritePinholeModelXml(const std::string& filename, const std::vector<CameraModelPinhole<double> >& cameras);
}
}
}
