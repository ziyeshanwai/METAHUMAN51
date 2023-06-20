// Copyright Epic Games, Inc. All Rights Reserved.

#include "CameraUndistortion.h"
#include "Common.h"

#include "Internals/OpenCVCamera2MetaShapeCamera.h"

#include <nls/geometry/MetaShapeCamera.h>

#include <cstring>
#include <map>

using namespace epic::nls;


namespace titan {
    namespace api {
        struct CameraUndistortion::Private {
            std::map<std::string, MetaShapeCamera<float> > cameras;
            std::map<std::string, Eigen::Matrix<Eigen::Vector2f, -1, -1> > undistortionMaps;
        };

        CameraUndistortion::CameraUndistortion()
            : m(new Private) {
        }

        CameraUndistortion::~CameraUndistortion() {
            if (m) {
                delete m;
                m = nullptr;
            }
        }

        bool CameraUndistortion::Init() {
            return true;
        }

        void CameraUndistortion::Reset() {
            m->cameras.clear();
            m->undistortionMaps.clear();
        }

        void CameraUndistortion::AddCamera(const char* InCameraName, const OpenCVCamera& InCameraParameters) {
            RESET_ERROR;
            CHECK_OR_RETURN(InCameraName, , "invalid filename");
            MetaShapeCamera<float> camera = OpenCVCamera2MetaShapeCamera<float>(InCameraName, InCameraParameters);
            m->cameras.emplace(camera.Label(), std::move(camera));

            auto undistortionMapIt = m->undistortionMaps.find(InCameraName);
            if (undistortionMapIt != m->undistortionMaps.end()) {
                m->undistortionMaps.erase(undistortionMapIt);
            }
        }

        bool CameraUndistortion::UndistortImage(const char* InCameraName,
                                                const unsigned char* ImageDataIn,
                                                unsigned char* ImageDataOut) {
            RESET_ERROR;
            CHECK_OR_RETURN(InCameraName, false, "invalid camera name");
            CHECK_OR_RETURN(ImageDataIn, false, "invalid image data");
            CHECK_OR_RETURN(ImageDataOut, false, "invalid image data");

            auto cameraIt = m->cameras.find(InCameraName);
            CHECK_OR_RETURN(cameraIt != m->cameras.end(), false, "camera {} does not exist", InCameraName);

            const MetaShapeCamera<float>& camera = cameraIt->second;

            auto undistortionMapIt = m->undistortionMaps.find(InCameraName);
            if (undistortionMapIt == m->undistortionMaps.end()) {
                Eigen::Matrix<Eigen::Vector2f, -1, -1> undistortionMap(camera.Width(), camera.Height());
                for (int y = 0; y < camera.Height(); ++y) {
                    for (int x = 0; x < camera.Width(); ++x) {
                        Eigen::Vector2f distortedPixelPosition =
                            camera.Distort(Eigen::Vector2f(x + 0.5f, y + 0.5f)) - Eigen::Vector2f(0.5f, 0.5f);
                        undistortionMap(x, y) = distortedPixelPosition;
                        undistortionMap(x, y)[0] = std::clamp<float>(undistortionMap(x, y)[0], 0, camera.Width() - 1.001f);
                        undistortionMap(x, y)[1] = std::clamp<float>(undistortionMap(x, y)[1], 0, camera.Height() - 1.001f);
                    }
                }

                undistortionMapIt = m->undistortionMaps.emplace(camera.Label(), std::move(undistortionMap)).first;
            }

            const Eigen::Matrix<Eigen::Vector2f, -1, -1>& undistortionMap = undistortionMapIt->second;
            const int numChannels = 4;

            for (int y = 0; y < camera.Height(); ++y) {
                for (int x = 0; x < camera.Width(); ++x) {
                    const Eigen::Vector2f pix = undistortionMap(x, y);
                    const int tx = int(pix[0]);
                    const int ty = int(pix[1]);
                    const float wx1 = pix[0] - float(tx);
                    const float wy1 = pix[1] - float(ty);
                    const float wx0 = float(1) - wx1;
                    const float wy0 = float(1) - wy1;
                    for (int k = 0; k < 4; ++k) {
                        float v00 = float(ImageDataIn[((ty + 0) * camera.Width() + tx + 0) * numChannels + k]);
                        float v01 = float(ImageDataIn[((ty + 0) * camera.Width() + tx + 1) * numChannels + k]);
                        float v10 = float(ImageDataIn[((ty + 1) * camera.Width() + tx + 0) * numChannels + k]);
                        float v11 = float(ImageDataIn[((ty + 1) * camera.Width() + tx + 1) * numChannels + k]);
                        float output = wx0 * wy0 * v00 + wx1 * wy0 * v01 + wx0 * wy1 * v10 + wx1 * wy1 * v11;
                        ImageDataOut[(y * camera.Width() + x) * numChannels + k] = uint8_t(output);
                    }
                }
            }

            return true;
        }

        bool CameraUndistortion::UndistortPoints(const char* InCameraName,
                                                 int32_t InNumPoints,
                                                 const float* InPointsData,
                                                 float* OutPointsData) const {
            RESET_ERROR;
            CHECK_OR_RETURN(InCameraName, false, "invalid camera name");
            CHECK_OR_RETURN(InNumPoints >= 0, false, "invalid number of points");
            CHECK_OR_RETURN(InPointsData, false, "invalid points data");
            CHECK_OR_RETURN(OutPointsData, false, "invalid points data");

            auto cameraIt = m->cameras.find(InCameraName);
            CHECK_OR_RETURN(cameraIt != m->cameras.end(), false, "camera {} does not exist", InCameraName);

            const MetaShapeCamera<float>& camera = cameraIt->second;
            Eigen::Map<const Eigen::Matrix<float, 2, -1> > inPoints(InPointsData, 2, InNumPoints);
            Eigen::Map<Eigen::Matrix<float, 2, -1> > outPoints(OutPointsData, 2, InNumPoints);

            for (int32_t i = 0; i < InNumPoints; ++i) {
                outPoints.col(i) = camera.Undistort(inPoints.col(i));
            }

            return true;
        }
    }  // namespace api
}  // namespace titan
