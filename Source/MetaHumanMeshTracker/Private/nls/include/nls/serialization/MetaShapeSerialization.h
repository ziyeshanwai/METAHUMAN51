// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/geometry/MetaShapeCamera.h>


namespace epic {
namespace nls {

/**
 * Reads the metashape cameras from a cameras.xml file as exported from Agisoft Metashape.
 * The user can provide an additional transformation and scaling converting from metashape space to the user defined world space.
 * The resulting metashape cameras will all be in world space i.e. the translation component of each metashape camera will be in world coordinates.
 */
template <class T>
bool ReadMetaShapeCameras(const std::string& filename, std::vector<MetaShapeCamera<T>>& cameras, const Affine<T, 3, 3>& metashapeToWorldTransform, const T metashapeToWorldScale);

/**
 * Writes the metashape cameras to a cameras.xml file in Agisoft Metashape format.
 * Writes out one sensor per camera even if two or more cameras have the same intrinsics and distortion parameters.
 */
template <class T>
bool WriteMetaShapeCameras(const std::string& filename, const std::vector<MetaShapeCamera<T>>& cameras);

/**
 * Reads our custom formst stabilization file that describes the transformation from metashape to world coordinates.
 * The stabilisation file has the info as tx, ty, tz, rx, ry, rz, sx, sy, sz in Maya convention (rotations in degrees, xyz order, scale should be uniform).
 * The transformation from metashape to world is therefore:
 * P_world = EulerZ(rx) * EulerY(ry) * EulerX(rx) * sx * P_metashape + (tx, ty, tz)
 */
template <class T>
bool ReadStabilizationFile(const std::string& filename, Affine<T, 3, 3>& transform, T& scale);

}
}
