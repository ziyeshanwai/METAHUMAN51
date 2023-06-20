// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {

template <class T>
T srgb2linear(T srgb)
{
  const T offset = T(0.055);
  if (srgb < T(0.04045)) {
    return srgb / T(12.92);
  } else {
    return T(std::pow((srgb + offset)/(1.0 + offset), 2.4));
  }
}

template <class T>
T linear2srgb(T linear)
{
  if (linear < T(0.0031308)) {
    return T(12.92) * linear;
  } else {
    return T(1.055) * std::pow(linear, T(1.0)/T(2.4)) - T(0.055);
  }
}


} // namespace nls
} //namespace epic
