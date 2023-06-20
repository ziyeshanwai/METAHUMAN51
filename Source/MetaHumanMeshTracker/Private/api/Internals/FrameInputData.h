// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nrr/landmarks/LandmarkInstance.h>
#include <nls/geometry/DepthmapData.h>
#include <nls/geometry/Mesh.h>

#include <map>
#include <string>
#include <vector>

namespace titan {
namespace api {

enum class InputDataType {
    DEPTHS,
    SCAN,
    NONE
};


class FrameInputData {
public:
    FrameInputData(const std::map<std::string, std::shared_ptr<const epic::nls::LandmarkInstance<float, 2> > >& landmarks,
        const std::map<std::string, std::shared_ptr<const epic::nls::DepthmapData<float> > >& depthmaps);

    FrameInputData(const std::map<std::string, std::shared_ptr<const epic::nls::LandmarkInstance<float, 2> > >& landmarks2d,
                   const std::shared_ptr<const epic::nls::LandmarkInstance<float, 3> >& landmarks3d,
                   const std::shared_ptr<const epic::nls::Mesh<float> >& scan);

    ~FrameInputData() = default;

    const std::map<std::string, std::shared_ptr<const epic::nls::LandmarkInstance<float, 2> > >& LandmarksPerCamera() const;

    const std::shared_ptr<const epic::nls::LandmarkInstance<float, 3> >& Landmarks3D() const;

    const std::shared_ptr<const epic::nls::Mesh<float> >& Scan() const;

    const std::map<std::string, std::shared_ptr<const epic::nls::DepthmapData<float> > >& Depthmaps() const;

    void Clear();

private:
    std::map<std::string, std::shared_ptr<const epic::nls::LandmarkInstance<float, 2> > > m_perCameraLandmarkData;
    std::shared_ptr<const epic::nls::LandmarkInstance<float, 3>> m_landmark3Ddata;
    std::shared_ptr<const epic::nls::Mesh<float> > m_scan;
    std::map<std::string, std::shared_ptr<const epic::nls::DepthmapData<float> > > m_depthmaps;
};

} // namespace api
} // namespace titan
