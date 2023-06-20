// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/MetaShapeCamera.h>

#include <map>

namespace epic::nls {

    /**
     * Class that contains a full camera setup including depth ranges.
     */
    template <class T>
    class MultiCameraSetup {
        public:
            MultiCameraSetup() = default;

            void Init(const std::vector<MetaShapeCamera<float> >& cameras)
            {
                m_cameraMap.clear();
                m_cameraNames.clear();
                for (const MetaShapeCamera<float>& camera : cameras) {
                    m_cameraNames.push_back(camera.Label());
                    m_cameraMap[camera.Label()] = camera;
                }
            }

            void SetCameraRanges(const std::map<std::string, std::pair<T, T> >& cameraRanges)
            {
                for (const auto& [cameraName, range] : cameraRanges) {
                    SetCameraRange(cameraName, range);
                }
            }

            void SetCameraRange(const std::string& cameraName, const std::pair<T, T>& cameraRange)
            {
                if (!HasCamera(cameraName)) {
                    CARBON_CRITICAL("cannot set camera range as there is no camera {}", cameraName);
                }
                m_cameraRanges[cameraName] = cameraRange;
            }

            const std::vector<std::string>& GetCameraNames() const {
                return m_cameraNames;
            }

            std::vector<MetaShapeCamera<T> > GetCamerasAsVector() const {
                std::vector<MetaShapeCamera<float> > cameras;
                for (const auto& [_, camera] : m_cameraMap) {
                    cameras.push_back(camera);
                }
                return cameras;
            }

            bool HasCamera(const std::string& name) const { return (m_cameraMap.find(name) != m_cameraMap.end()); }

            const MetaShapeCamera<T>& GetCamera(const std::string& name) const {
                auto it = m_cameraMap.find(name);
                if (it != m_cameraMap.end()) {
                    return it->second;
                } else {
                    CARBON_CRITICAL("No camera {} in multi camera setup", name);
                }
            }

            const std::map<std::string, MetaShapeCamera<T> >& GetCameras() const { return m_cameraMap; }

            bool HasCameraRange(const std::string& name) const { return (m_cameraRanges.find(name) != m_cameraRanges.end()); }

            const std::pair<T, T>& GetCameraRange(const std::string& name) const {
                auto it = m_cameraRanges.find(name);
                if (it != m_cameraRanges.end()) {
                    return it->second;
                } else {
                    CARBON_CRITICAL("No range for camera {} in multi camera setup", name);
                }
            }

            const std::map<std::string, std::pair<T, T> >& GetCameraRanges() const { return m_cameraRanges; }

            void ScaleCameras(T scale)
            {
                m_originalCamerasToCurrentCamerasScale *= scale;

                // scale camera configuration (which just means that we need to scale the translation)
                for (auto&& [_, camera] : m_cameraMap) {
                    Affine<float, 3, 3> extrinsics = camera.Extrinsics();
                    extrinsics.SetTranslation(scale * extrinsics.Translation());
                    camera.SetExtrinsics(extrinsics);
                }

                // scale camera ranges
                for (auto&& [_, cameraRange] : m_cameraRanges) {
                    cameraRange.first *= scale;
                    cameraRange.second *= scale;
                }
            }

            T GetScale() const { return m_originalCamerasToCurrentCamerasScale; }

        private:
            std::vector<std::string> m_cameraNames;
            std::map<std::string, MetaShapeCamera<T> > m_cameraMap;
            std::map<std::string, std::pair<T, T> > m_cameraRanges;
            // any scaling that was applied to the cameras after setting up the the camera setup
            T m_originalCamerasToCurrentCamerasScale = T(1);

    };
}  // namespace epic::nls
