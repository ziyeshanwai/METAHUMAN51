// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

//! from irlcpp code

#include <carbon/utils/TaskThreadPool.h>
#include <nls/math/Math.h>

#include <string>
#include <stdio.h>
#include <vector>

namespace epic::nls {

template <class T>
class Mesh;


template <class T>
class ObjFileReader {
public:
    static bool readObj(std::string fileName, Mesh<T>& mesh, const std::shared_ptr<epic::carbon::TaskThreadPool>& taskThreadPool = epic::carbon::TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/false));

    enum class ErrorType {
        NONE, LEXER, SYNTAX, FACES_WITHOUT_UVS, UNEXPECTED_TOK, UNEXPECTED_FACE_SIZE
    };

    static const char* ErrorTypeToString(ErrorType errorType);
};


template <class T>
class ObjFileWriter {
public:
    ObjFileWriter();
    ~ObjFileWriter();

    void writeObj(const Mesh<T>& mesh, std::string filename, bool withTexture = true);

private:
    void openStream(std::string filename);
    void closeStream();
    void writeHeader();
    void writeVertices();
    void writeUV();
    void writeNormals();
    void writeFaces(bool withTexture);
    void error(std::string msg);

    FILE* m_stream;
    const Mesh<T>* m_mesh;
};

} // namespace epic::nls
