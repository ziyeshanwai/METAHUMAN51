// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/data/CameraModelOpenCV.h>

namespace epic {
namespace carbon {

template<class T>
CameraModelOpenCV<T>::CameraModelOpenCV() noexcept {
    m_distortion = Eigen::Vector<T, 5>::Zero();
}

template<class T> CameraModelOpenCV<T>::~CameraModelOpenCV() noexcept = default;

template<class T>
const Eigen::Vector<T, 5>& CameraModelOpenCV<T>::GetDistortionParams() const noexcept {
    return m_distortion;
}

template<class T>
void CameraModelOpenCV<T>::SetDistortionParams(const Eigen::Vector<T, 5>& distortion) noexcept {
    m_distortion = distortion;
}

template class CameraModelOpenCV<float>;
template class CameraModelOpenCV<double>;

}
}
