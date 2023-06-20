// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/data/CameraModelPinhole.h>

namespace epic {
namespace carbon {

template<class T>
CameraModelPinhole<T>::CameraModelPinhole()  noexcept {
    m_intrinsics = Eigen::Matrix<T, 3, 3>::Identity();
    m_extrinsics = Eigen::Matrix<T, 3, 4>::Identity();
    m_width = 0;
    m_height = 0;
}

template<class T> CameraModelPinhole<T>::~CameraModelPinhole() noexcept = default;

template<class T>
const Eigen::Matrix<T, 3, 3>& CameraModelPinhole<T>::GetIntrinsics() const noexcept {
    return m_intrinsics;
}

template<class T>
void CameraModelPinhole<T>::SetIntrinsics(const Eigen::Matrix<T, 3, 3>& intrinsics) {
    if ((intrinsics(1, 0) != 0) || (intrinsics(2, 0) != 0) || (intrinsics(2, 1) != 0) || (intrinsics(2, 2) != 1)) {
        throw std::runtime_error("Invalid intrinsics - needs to be an upper triangular matrix with K(2,2) == 1");
    }
    m_intrinsics = intrinsics;
}

template<class T>
const Eigen::Matrix<T, 3, 4>& CameraModelPinhole<T>::GetExtrinsics() const noexcept {
    return m_extrinsics;
}

template<class T>
void CameraModelPinhole<T>::SetExtrinsics(const Eigen::Matrix<T, 3, 4>& extrinsics) {
    if (((extrinsics.topLeftCorner(3,
                                   3) *
          extrinsics.topLeftCorner(3, 3).transpose()) - Eigen::Matrix3<T>::Identity()).norm() > T(1e-3)) {
        throw std::runtime_error("Invalid extrinsics - Rotation matrix is not orthogonal.");
    }
    m_extrinsics = extrinsics;
}

template<class T>
int CameraModelPinhole<T>::GetWidth() const noexcept {
    return m_width;
}

template<class T>
void CameraModelPinhole<T>::SetWidth(const int width) noexcept {
    m_width = width;
}

template<class T>
int CameraModelPinhole<T>::GetHeight() const noexcept {
    return m_height;
}

template<class T>
void CameraModelPinhole<T>::SetHeight(const int height) noexcept {
    m_height = height;
}

template<class T>
std::string CameraModelPinhole<T>::GetLabel() const noexcept {
    return m_label;
}

template<class T>
void CameraModelPinhole<T>::SetLabel(const std::string label) noexcept {
    m_label = label;
}

template<class T>
std::string CameraModelPinhole<T>::GetModel() const noexcept {
    return m_model;
}

template<class T>
void CameraModelPinhole<T>::SetModel(const std::string model) noexcept {
    m_model = model;
}

template class CameraModelPinhole<float>;
template class CameraModelPinhole<double>;

}
}
