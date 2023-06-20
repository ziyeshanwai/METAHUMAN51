// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/io/JsonIO.h>

#include <nls/geometry/Affine.h>
#include <nls/geometry/DiffDataAffine.h>
#include <nls/serialization/EigenSerialization.h>


namespace epic {
namespace nls {

//! affine serialization as homogenous 4x4 matrix (see Eigen json serialization)
template <class T, int R, int C>
carbon::JsonElement ToJson2(const Affine<T, R, C>& aff) {
    return ToJson2(aff.Matrix());
}

//! deserializes an affine matrix from json format
template <class T, int R, int C>
void FromJson(const carbon::JsonElement& j, Affine<T, R, C>& aff) {
    aff.SetMatrix(FromJson<Eigen::Matrix<T, R + 1, C + 1>>(j));
}

}
}
