// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/io/JsonIO.h>
#include <nls/geometry/MetaShapeCamera.h>
#include <nls/utils/FileIO.h>

namespace epic::nls {

/**
 * Reads camera calibration information from our own camera json format and converts it to MetaShapeCameras.
 */
template <class T>
bool ReadMetaShapeCamerasFromJsonFile(const std::string& filename, std::vector<MetaShapeCamera<T>>& cameras)
{
    carbon::JsonElement json = carbon::ReadJson(ReadFile(filename));
    if (!json.IsArray()) {
        LOG_ERROR("json camera file should contains an array as top level");
        return false;
    }

    std::vector<MetaShapeCamera<T>> newCameras;

    for (int i = 0; i < static_cast<int>(json.Size()); ++i) {
        if (!json[i].Contains("metadata")) {
            LOG_ERROR("json camera file should contain an array of objects that have a metadata field");
            return false;
        }

        if (json[i]["metadata"]["type"].Get<std::string>() == "camera") {
            // we have a camera type
            MetaShapeCamera<T> camera;
            camera.SetLabel(json[i]["metadata"]["name"].Get<std::string>());
            camera.SetWidth(json[i]["image_size_x"].Get<int>());
            camera.SetHeight(json[i]["image_size_y"].Get<int>());
            Eigen::Matrix<T, 3, 3> intrinsics = Eigen::Matrix<T, 3, 3>::Identity();
            if (json[i].Contains("fx") && json[i].Contains("fy") && json[i].Contains("cx") && json[i].Contains("cy")) {
                intrinsics(0, 0) = json[i]["fx"].Get<T>();
                intrinsics(1, 1) = json[i]["fy"].Get<T>();
                intrinsics(0, 2) = json[i]["cx"].Get<T>();
                intrinsics(1, 2) = json[i]["cy"].Get<T>();
                camera.SetIntrinsics(intrinsics);
            } else {
                LOG_ERROR("camera calibration is missing one of fx, fy, cx, cy");
                return false;
            }

            Eigen::Matrix<T, 4, 4> extrinsics;
            for (int c = 0; c < 4; ++c) {
                for (int r = 0; r < 4; ++r) {
                    extrinsics(r, c) = json[i]["transform"][4 * r + c].Get<T>();
                }
            }

            camera.SetExtrinsics(extrinsics);

            if (json[i]["distortion_model"].Get<std::string>() == "opencv") {
                auto valueOrNull = [](const carbon::JsonElement& j, const char* label) { return j.Contains(label) ? j[label].Get<T>() : T(0); };
                const T k1 = valueOrNull(json[i], "k1");
                const T k2 = valueOrNull(json[i], "k2");
                const T k3 = valueOrNull(json[i], "k3");
                const T k4 = valueOrNull(json[i], "k4");
                const T k5 = valueOrNull(json[i], "k5");
                const T k6 = valueOrNull(json[i], "k6");
                const T p1 = valueOrNull(json[i], "p1");
                const T p2 = valueOrNull(json[i], "p2");
                if (k4 != T(0) || k5 != T(0) || k6 != T(0)) {
                    LOG_ERROR("metashape camera does not support k4, k5, and k6 parameter");
                    return false;
                }
                camera.SetRadialDistortion(Eigen::Vector<T, 4>(k1, k2, k3, T(0)));
                camera.SetTangentialDistortion(Eigen::Vector<T, 4>(p2, p1, T(0), T(0))); // note that metashape camera has swapped tangential distortion compared to opencv
            } else {
                LOG_ERROR("no valid distortion model defined");
                return false;
            }

            newCameras.push_back(camera);
        }
    }

    cameras.swap(newCameras);
    return true;
}


/**
 * Writes camera calibration information into our own camera json format
 */
template <class T>
bool WriteMetaShapeCamerasToJsonFile(const std::string& filename, const std::vector<MetaShapeCamera<T>>& cameras)
{
    carbon::JsonElement json(carbon::JsonElement::JsonType::Array);
    for (const MetaShapeCamera<T>& camera : cameras) {

        carbon::JsonElement jsonMeta(carbon::JsonElement::JsonType::Object);
        jsonMeta.Insert("type", carbon::JsonElement("camera"));
        jsonMeta.Insert("version", carbon::JsonElement(0));
        jsonMeta.Insert("name", carbon::JsonElement(camera.Label()));
        jsonMeta.Insert("camera", carbon::JsonElement(camera.Label()));

        carbon::JsonElement jsonCamera(carbon::JsonElement::JsonType::Object);
        jsonCamera.Insert("metadata", std::move(jsonMeta));

        jsonCamera.Insert("image_size_x", carbon::JsonElement(camera.Width()));
        jsonCamera.Insert("image_size_y", carbon::JsonElement(camera.Height()));
        jsonCamera.Insert("fx", carbon::JsonElement(camera.Intrinsics()(0,0)));
        jsonCamera.Insert("fy", carbon::JsonElement(camera.Intrinsics()(1,1)));
        jsonCamera.Insert("cx", carbon::JsonElement(camera.Intrinsics()(0,2)));
        jsonCamera.Insert("cy", carbon::JsonElement(camera.Intrinsics()(1,2)));

        if (camera.Intrinsics()(0, 1) != T(0)) {
            LOG_ERROR("failed to write camera parameters as intrinsics skew is not supported");
            return false;
        }

        jsonCamera.Insert("distortion_model", carbon::JsonElement("opencv"));
        jsonCamera.Insert("k1", carbon::JsonElement(camera.RadialDistortion()[0]));
        jsonCamera.Insert("k2", carbon::JsonElement(camera.RadialDistortion()[1]));
        jsonCamera.Insert("k3", carbon::JsonElement(camera.RadialDistortion()[2]));

        if (camera.RadialDistortion()[3] != T(0)) {
            LOG_ERROR("failed to write camera parameters as k4 distortion is not supported");
            return false;
        }


        jsonCamera.Insert("p1", carbon::JsonElement(camera.TangentialDistortion()[1])); // note swapped tangential distortion between opencv and metashape
        jsonCamera.Insert("p2", carbon::JsonElement(camera.TangentialDistortion()[0]));
        if (camera.TangentialDistortion()[2] != T(0) || camera.TangentialDistortion()[3] != T(0)) {
            LOG_ERROR("failed to write camera parameters as extended tangential distortion is not supported");
            return false;
        }
        if (camera.Skew().squaredNorm() > 0) {
            LOG_ERROR("failed to write camera parameters as extended skew is not supported");
            return false;
        }

        carbon::JsonElement jsonTransform(carbon::JsonElement::JsonType::Array);
        const Eigen::Matrix<T, 4, 4> transform = camera.Extrinsics().Matrix();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                jsonTransform.Append(carbon::JsonElement(transform(r, c)));
            }
        }
        jsonCamera.Insert("transform", std::move(jsonTransform));
        json.Append(std::move(jsonCamera));
    }

    WriteFile(filename, carbon::WriteJson(json, /*tabs=*/1));

    return true;
}

} // epic::nls