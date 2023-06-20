// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/utils/Timer.h>

#include <fstream>
#include <stdio.h>

namespace epic {
namespace nls {

//! read the entire file as a string (basic iostream implementation is way slower if buffer of fstream is not modified)
inline std::string ReadFile(const std::string& filename)
{
  FILE* pFile = nullptr;
#ifdef _MSC_VER
  if (fopen_s(&pFile, filename.c_str(), "rb") == 0 && pFile) {
#else
  pFile = fopen(filename.c_str(), "rb");
  if (pFile) {
#endif
    fseek(pFile, 0, SEEK_END);
    const long pos = ftell(pFile);
    std::string data;
    data.resize(pos);
    fseek(pFile, 0, SEEK_SET);
    if (fread(&data[0], 1, pos, pFile) != size_t(pos)) {
      throw std::runtime_error("failed to read all data of file " + filename);
    }
    fclose(pFile);
    return data;
  }
  throw std::runtime_error("no file " + filename);
}


//! write the entire file as a string
inline void WriteFile(const std::string& filename, const std::string& data)
{
  std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary);
  if (out) {
    out.write(data.c_str(), data.size());
    out.close();
    return;
  }
  throw std::runtime_error("no file");
}

} // namespace nls
} //namespace epic
