// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/data/Mesh.h>

#include <fstream>

namespace epic {
namespace carbon {

enum TokenType {
    NONE_TOK, VERTEX_TOK, UV_TOK, NORMAL_TOK, NUMBER_TOK,
    FACE_TOK, OBJ_TOK, SLASH_TOK, NEW_LINE_TOK, EOF_TOK, COMMENT_TOK
};


class ReadingError : public std::runtime_error {
    public:
        ReadingError(const char* msg);
        ~ReadingError();
};

template<class T>
class ObjReader {
    public:
        std::ifstream file;
        ObjReader();
        ObjReader(const std::string& path);
        ~ObjReader();

        /// Open the ply file at given path, and prepare it for parsing.
        bool Open(const std::string& path);

        int GetChar();
        void PutBackChar(int c);
        void EatWhiteSpace();
        TokenType GetToken();
        void PutBackToken(TokenType tok);
        TokenType GetTokenFromStream();
        // parse whole file (implementation for parsing)
        void Parse(MeshData<T>& mesh);

        // parse v flag
        void ParseVertex(std::vector<T>& data);

        // parse vn flag
        void ParseNormal(std::vector<T>& data);

        // parse vt flag
        void ParseUV(std::vector<T>& data);

        // parse f flag
        void ParseFace(std::vector<unsigned int>& fvcount,
                       std::vector<unsigned int>& fids,
                       std::vector<unsigned int>& nids,
                       std::vector<unsigned int>& tids);

        void ExpectToken(TokenType expectedToken);

        int returnedChar;
        bool hasReturnedChar;
        TokenType returnedToken;
        bool hasReturnedToken;
        double numberValue;
        std::vector<char> buffer;
        std::string errorMessage;
};

class WriterError : public std::runtime_error {
    public:
        WriterError(const char* msg);
        ~WriterError();
};

template<class T>
class ObjWriter {
    public:
        std::ofstream file;
        ObjWriter();
        ObjWriter(const std::string& path);
        ~ObjWriter();

        bool Open(const std::string& path);
        bool Close();

        void Flush(MeshData<T>& mesh);

    protected:
        void WriteHeader(size_t vertexCount, size_t faceCount);
        void WriteVertices(size_t vertexCount, const T* vertexData);
        void WriteUV(size_t uvCount, const T* uvData);
        void WriteNormals(size_t normalCount, const T* normalData);
        void WriteFaces(size_t faceCount,
                        const size_t* faceVertexCountData,
                        const size_t* faceVertexData,
                        const size_t* faceUvData,
                        const size_t* faceNormalData);
};
}
}
