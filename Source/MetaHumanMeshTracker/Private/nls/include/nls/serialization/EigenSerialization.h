// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>
#include <carbon/io/JsonIO.h>

#include <stdio.h>

namespace epic {
namespace nls {

/**
 * Serializes an Eigen matrix mat to json format in column major format.
 * Vectors are serialized to an array [values...]
 * Matrices are serialized to {
 *  "rows" : rows
 *  "cols" : cols
 *  "data" : [values ...] (in column major layout)
 * }
 */
template <class T, int R, int C>
carbon::JsonElement ToJson2(const Eigen::Matrix<T, R, C>& mat)
{
    if constexpr (R == 1 || C == 1) {
        // vector type is directly written as a single number array
        carbon::JsonElement j(carbon::JsonElement::JsonType::Array);
        for (int i = 0; i < int(mat.size()); i++) { j.Append(carbon::JsonElement(*(mat.data() + i))); }
        return j;
    } else {
        carbon::JsonElement matObject(carbon::JsonElement::JsonType::Array);
        for (int c = 0; c < int(mat.cols()); c++) {
            for (int r = 0; r < int(mat.rows()); r++) {
                matObject.Append(carbon::JsonElement(mat(r,c)));
            }
        }
        carbon::JsonElement j(carbon::JsonElement::JsonType::Object);
        j.Insert("rows", carbon::JsonElement(int(mat.rows())));
        j.Insert("cols", carbon::JsonElement(int(mat.cols())));
        j.Insert("data", std::move(matObject));
        return j;
    }
}

//! deserializes a matrix from json format (see @ToJson)
template <class T, int R, int C>
void FromJson(const carbon::JsonElement& j, Eigen::Matrix<T, R, C>& mat) {
    if (j.IsArray()) {
        const int size = int(j.Size());

        // we only row and colum vectors to be read directly from an array
        const bool isColumnVector = (C == 1 && (R < 0 || R == size));
        const bool isRowVector = (R == 1 && (C < 0 || C == size));
        if (isColumnVector) {
            mat.resize(size,1);
        } else if (isRowVector) {
            mat.resize(1,size);
        } else {
            CARBON_CRITICAL("invalid size {} for matrix of size {}x{}", size, R, C);
        }
        for (int i = 0; i < size; i++) {
            *(mat.data() + i) = j[i].template Get<T>();
        }
    }
    else {
        int rows = j["rows"].Get<int>();
        int cols = j["cols"].Get<int>();
        if (rows < 0 || cols < 0) {
            CARBON_CRITICAL("invalid number of rows and/or columns: {}x{}", rows, cols);
        }
        if constexpr (R >= 0) assert (R == rows);
        if constexpr (C >= 0) assert (C == cols);
        mat.resize(rows, cols);

        const carbon::JsonElement& jsonObject = j["data"];
        if (!jsonObject.IsArray() || (static_cast<int>(jsonObject.Size()) != rows * cols)) {
            CARBON_CRITICAL("object is not an array or not of the right size");
        }
        for (int c = 0; c < cols; c++) {
            for (int r = 0; r < rows; r++) {
                mat(r,c) = jsonObject[c * rows + r].template Get<T>();
            }
        }
    }
}

//! Serializes a matrix to json format.
template <class MatrixType>
MatrixType FromJson(const carbon::JsonElement& j)
{
    MatrixType mat;
    FromJson(j, mat);
    return mat;
}

//! Serializes a column-major matrix to binary format
template <class T, int R, int C>
bool ToBinaryFile(FILE* pFile, const Eigen::Matrix<T, R, C>& mat)
{
    bool success = true;
    int r = static_cast<int>(mat.rows());
    int c = static_cast<int>(mat.cols());
    success &= (fwrite(&r, sizeof(r), 1, pFile) == 1);
    success &= (fwrite(&c, sizeof(c), 1, pFile) == 1);
    if (mat.size() > 0) {
        success &= (fwrite(mat.data(), sizeof(T) * r * c, 1, pFile) == 1);
    }
    return success;
}

//! Deserializes a column-major matrix from binary format
template <class T, int R, int C>
bool FromBinaryFile(FILE* pFile, Eigen::Matrix<T, R, C>& mat)
{
    bool success = true;
    int r = 0;
    int c = 0;
    success &= (fread(&r, sizeof(r), 1, pFile) == 1);
    success &= (fread(&c, sizeof(c), 1, pFile) == 1);
    if constexpr (R >= 0) {
        if (r != R) return false;
    }
    if constexpr (C >= 0) {
        if (c != C) return false;
    }
    mat.resize(r, c);
    if (mat.size() > 0) {
        success &= (fread(mat.data(), sizeof(T) * r * c, 1, pFile) == 1);
    }
    return success;
}

}
}
