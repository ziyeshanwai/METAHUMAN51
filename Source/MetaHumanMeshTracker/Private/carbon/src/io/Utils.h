// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/common/Common.h>
#include <carbon/common/EigenDenseBackwardsCompatible.h>
#include <carbon/io/JsonIO.h>
#include <carbon/io/Utils.h>
#include <carbon/io/XmlIO.h>

#include <fstream>
#include <iomanip>

using namespace epic::carbon::xml;

namespace epic {
namespace carbon {

JsonElement WriteFileinfo();

void ReadCameraMetadataJson(const JsonElement& json, std::string& label, std::string& model, int& width, int& height);
void ReadSensorMetadataXml(XMLElement* xmlSensor, int& width, int& height, int& id);

void WriteCameraMetadataJson(JsonElement& json, int width, int height, const std::string& label, const std::string& model);
void WriteSensorMetadataXml(XMLElement& xmlSensor, int width, int height, int id);
void WriteCameraMetadataXml(XMLElement& xmlCamera, const std::string& label, int id);

template<class T>
void WriteIntrinsicsJson(JsonElement& json, const Eigen::Matrix3<T>& intrinsics);

template<class T>
void WriteIntrinsicsXml(XMLElement& xmlCalibration, const Eigen::Matrix3<T>& intrinsics, int width, int height);

template<class T>
void ReadIntrinsicsJson(const JsonElement& json, Eigen::Matrix3<T>& intrinsics);

template<class T>
void ReadIntrinsicsXml(XMLElement* xmlCalibration, Eigen::Matrix3<T>& intrinsics, int width, int height);

template<class T>
JsonElement WriteExtrinsicsJson(const Eigen::Matrix<T, 3, 4>& transform);

template<class T>
void WriteExtrinsicsXml(XMLElement& xmlCamera, const Eigen::Matrix<T, 3, 4>& transform);

template<class T>
void ReadExtrinsicsJson(const JsonElement& json, Eigen::Matrix<T, 3, 4>& transform);

template<class T>
void ReadGlobalTransformXml(XMLElement* xmlChunk, Eigen::Matrix4d& transform, T& scale);

template<class T>
void ReadExtrinsicsXml(XMLElement* xmlCamera,
                       const Eigen::Matrix4d& globalTransform,
                       Eigen::Matrix<T, 3, 4>& cameraExtrinsics,
                       T scale);

template<class T>
void WriteDistortionOpenCVJson(JsonElement& json, const Eigen::Vector<T, 5>& distortion);

template<class T>
void WriteDistortionOpenCVXml(XMLElement& json, const Eigen::Vector<T, 5>& distortion);

template<class T>
void ReadDistortionOpenCVJson(const JsonElement& json, Eigen::Vector<T, 5>& distortion);

template<class T>
void ReadDistortionOpenCvXml(XMLElement* xmlCalibration, Eigen::Vector<T, 5>& distortion);
}
}
