// Copyright Epic Games, Inc. All Rights Reserved.

#include "FrameInputData.h"
#include <carbon/Common.h>

#include <cstring>
#include <filesystem>
#include <map>

using namespace epic::nls;

namespace titan::api {

FrameInputData::FrameInputData(const std::map<std::string, std::shared_ptr<const LandmarkInstance<float, 2> > >& landmarks,
                const std::map<std::string, std::shared_ptr<const DepthmapData<float> > >& depthmaps)
    : m_perCameraLandmarkData(landmarks),
      m_depthmaps(depthmaps) {
}

FrameInputData::FrameInputData(const std::map<std::string, std::shared_ptr<const epic::nls::LandmarkInstance<float, 2> > >& landmarks2d,
                               const std::shared_ptr<const epic::nls::LandmarkInstance<float, 3> >& landmarks3d,
                               const std::shared_ptr<const epic::nls::Mesh<float> >& scan)
    :m_perCameraLandmarkData(landmarks2d),
     m_landmark3Ddata(landmarks3d),
     m_scan(scan) {
}


const std::map<std::string, std::shared_ptr<const LandmarkInstance<float, 2> > >& FrameInputData::LandmarksPerCamera() const {
    return m_perCameraLandmarkData;
}

const std::shared_ptr<const epic::nls::LandmarkInstance<float, 3> >& FrameInputData::Landmarks3D() const {
    return m_landmark3Ddata;
}

const std::shared_ptr<const Mesh<float> >& FrameInputData::Scan() const {
    CARBON_PRECONDITION(m_scan, "no scan loaded");
    return m_scan;
}

const std::map<std::string, std::shared_ptr<const DepthmapData<float> > >& FrameInputData::Depthmaps() const {
    CARBON_PRECONDITION(!m_depthmaps.empty(), "no depthmaps loaded");
    return m_depthmaps;
}

void FrameInputData::Clear() {
    m_perCameraLandmarkData.clear();
    m_depthmaps.clear();
    m_landmark3Ddata.reset();
    m_scan.reset();
}

} // namespace titan::api
