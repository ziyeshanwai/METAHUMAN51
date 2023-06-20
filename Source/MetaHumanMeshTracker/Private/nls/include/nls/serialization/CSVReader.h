// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace epic {
namespace nls {

//! Inefficient yet simple CSV reader. Does not support escaped or quoated commas and newlines
inline std::vector<std::vector<std::string>> ReadCSVFile(const std::string& filename)
{
    std::ifstream in(filename);
    if (!in) {
        throw std::runtime_error("failed to read file " + filename);
    }

    std::vector<std::vector<std::string>> csvTokens;

    std::string line;
    std::string token;
    while (std::getline(in, line)) {
        std::stringstream lineIn(line);
        std::vector<std::string> csvLineTokens;
        while(std::getline(lineIn, token, ',')) {
            csvLineTokens.push_back(token);
        }
        csvTokens.push_back(csvLineTokens);
    }

    in.close();
    return csvTokens;
}


}
}
