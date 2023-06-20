// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/data/CameraModelOpenCV.h>

namespace epic {
namespace carbon {
namespace camera {
/**
    @brief Functions for reading and writing camera parameters for specified formats.
*/

/**
    @brief Read Digital Humans Json camera file.
    Read Digital Humans Json camera file format with OpenCV distortion as distortion
    model (k1, k2, p1, p2, k3).
    @param filename
        String with file path.
*/
template<class T>
std::vector<CameraModelOpenCV<T> > ReadOpenCvModelJson(const std::string& filename);

/**
    @brief Read Digital Humans Json camera file.
    Read Digital Humans Json camera file format with no distortion model - pinhole cameras
    (in the case of working with undistorted images, for example).
    @param filename
        String with file path.
*/
template<class T>
std::vector<CameraModelPinhole<T> > ReadPinholeModelJson(const std::string& filename);

/**
    @brief Write Digital Humans Json camera file.
    Write Digital Humans Json camera file format with OpenCV distortion as distortion
    model (k1, k2, p1, p2, k3).
    @param cameras
        Vector of cameras with OpenCV distortion model.
*/
template<class T>
void WriteOpenCvModelJson(const std::string& filename, const std::vector<CameraModelOpenCV<T> >& cameras);

/**
    @brief Write Digital Humans Json camera file.
    Write Digital Humans Json camera file format with pinhole camera model.
    @param cameras
        Vector of cameras with pinhole camera model.
*/
template<class T>
void WritePinholeModelJson(const std::string& filename, const std::vector<CameraModelPinhole<T> >& cameras);

/**
    @brief Read Agisoft Metashape xml camera file.
    Read Agisoft Metashape xml camera file format with OpenCV distortion as distortion
    model (k1, k2, p1, p2, k3).
    @param filename
        String with file path.
*/
template<class T>
std::vector<CameraModelOpenCV<T> > ReadOpenCvModelXml(const std::string& filename);

/**
    @brief Read Agisoft Metashape xml camera file.
    Read Agisoft Metashape xml camera file format with as Pinhole camera model.
    @param filename
        String with file path.
*/
template<class T>
std::vector<CameraModelPinhole<T> > ReadPinholeModelXml(const std::string& filename);

/**
    @brief Write Agisoft Metashape xml camera file.
    Write Agisoft Metashape xml camera file format with OpenCV distortion as distortion
    model (k1, k2, p1, p2, k3).
    @param cameras
        Vector of cameras with OpenCV camera model.
    @param filename
        String with file path.
*/

template<class T>
void WriteOpenCvModelXml(const std::string& filename, const std::vector<CameraModelOpenCV<T> >& cameras);

/**
    @brief Write Agisoft Metashape xml camera file.
    Write Agisoft Metashape xml camera file without distortion parameters (pinhole).
    @param cameras
        Vector of cameras with Pinhole camera model.
    @param filename
        String with file path.
*/

template<class T>
void WritePinholeModelXml(const std::string& filename, const std::vector<CameraModelPinhole<T> >& cameras);
}
}
}
