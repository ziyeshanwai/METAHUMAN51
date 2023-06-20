// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/io/MeshIO.h>

#include <iomanip>

namespace epic {
namespace carbon {

    //Reader
    ReadingError::ReadingError(const char* msg) : std::runtime_error(msg) {
    }

    ReadingError::~ReadingError() {
    }

    template<class T>
    ObjReader<T>::ObjReader() {
        returnedChar = 0;
        hasReturnedChar = false;
        numberValue = 0.0;
        hasReturnedToken = false;
        returnedToken = NONE_TOK;
    }

    template<class T>
    ObjReader<T>::ObjReader(const std::string& path) : ObjReader() {
        Open(path);
    }

    template<class T>
    ObjReader<T>::~ObjReader() {
    }

    template<class T>
    bool ObjReader<T>::Open(const std::string& path) {
        file.open(path);
        return file.is_open();
    }

    template<class T>
    int ObjReader<T>::GetChar() {
        //assert(file);
        if (hasReturnedChar) {
            hasReturnedChar = false;
            return returnedChar;
        }
        return file.get();
    }

    template<class T>
    void ObjReader<T>::PutBackChar(int c) {
        assert(!hasReturnedChar);
        returnedChar = c;
        hasReturnedChar = true;
    }

    template<class T>
    void ObjReader<T>::EatWhiteSpace() {
        int c = GetChar();
        while (c == ' ' || c == '\t') {
            c = GetChar();
        }
        PutBackChar(c);
    }

    template<class T>
    TokenType ObjReader<T>::GetToken() {
        if (hasReturnedToken) {
            hasReturnedToken = false;
            return returnedToken;
        }
        return GetTokenFromStream();
    }

    template<class T>
    void ObjReader<T>::PutBackToken(TokenType tok) {
        assert(!hasReturnedToken);
        hasReturnedToken = true;
        returnedToken = tok;
    }

    template<class T>
    TokenType ObjReader<T>::GetTokenFromStream() {
        EatWhiteSpace();
        int c = GetChar();
        if (c == EOF) {
            return EOF_TOK;
        }
        if ((c == '-') || (c == '+') || ((c >= '0') && (c <= '9')) || (c == '.')) {
            buffer.clear();
            while (c == '-' || c == '+' || (c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E') {
                buffer.push_back((char)c);
                c = GetChar();
            }
            PutBackChar(c);
            buffer.push_back(0);
            numberValue = atof(&buffer[0]);
            return NUMBER_TOK;
        }
        if ((c >= 'a') && (c <= 'z')) {
            buffer.clear();
            while (c >= 'a' && c <= 'z') {
                buffer.push_back((char)c);
                c = GetChar();
            }
            PutBackChar(c);
            buffer.push_back(0);
            if (strcmp("v", &buffer[0]) == 0) {
                return VERTEX_TOK;
            }
            if (strcmp("vt", &buffer[0]) == 0) {
                return UV_TOK;
            }
            if (strcmp("vn", &buffer[0]) == 0) {
                return NORMAL_TOK;
            }
            if (strcmp("f", &buffer[0]) == 0) {
                return FACE_TOK;
            }
            // Treat all other commands as a comment.
            c = GetChar();
            while (c != '\n' && c != '\r') {
                c = GetChar();
            }
            PutBackChar(c);
            return COMMENT_TOK;
        }
        if (c == '#') {
            for (;;) {
                c = GetChar();
                if ((c == '\n') || (c == '\r') || (c == EOF)) {
                    PutBackChar(c);
                    return COMMENT_TOK;
                }
            }
        }
        if (c == '/') {
            return SLASH_TOK;
        }
        if ((c == '\r') || (c == '\n')) {
            return NEW_LINE_TOK;
        }
        if (c == 0) {
            return EOF_TOK;
        }
        throw ReadingError("syntax error in lexer");
    }

    template<class T>
    void ObjReader<T>::Parse(MeshData<T>& data) {

        std::vector<T> vts;
        std::vector<T> nrm;
        std::vector<T> uvs;
        std::vector<unsigned int> fc;
        std::vector<unsigned int> fids;
        std::vector<unsigned int> nids;
        std::vector<unsigned int> tids;

        while (true) {
            TokenType tok = GetToken();

            switch (tok) {
            case NONE_TOK:
                break;
            case VERTEX_TOK:
                ParseVertex(vts);
                break;
            case UV_TOK:
                ParseUV(uvs);
                break;
            case NORMAL_TOK:
                ParseNormal(nrm);
                break;
            case FACE_TOK:
                ParseFace(fc, fids, nids, tids);
                break;
            case OBJ_TOK:
                throw ReadingError("command object is not supported.");
                break;
            case NEW_LINE_TOK:
                break;
            case COMMENT_TOK:
                break;
            case EOF_TOK:
                goto end_reading;
            default:
                throw ReadingError("Syntax Error");
                break;
            }
        }

    end_reading:

        auto vertexCount = vts.size() / 3;
        auto uvCount = uvs.size() / 2;
        auto normalCount = nrm.size() / 3;

        MeshDataBuilder<T> builder;

        // setup vertices
        if (!vts.empty()) {
            builder.SetVertices(vertexCount, reinterpret_cast<const T*>(vts.data()));
        }
        // setup uvs
        if (!uvs.empty()) {
            builder.AddUvSet(uvCount, reinterpret_cast<const T*>(uvs.data()));
        }

        // setup normals
        if (!nrm.empty()) {
            builder.SetNormals(normalCount, reinterpret_cast<const T*>(nrm.data()));
        }

        // setup faces
        builder.SetFaces(fc, fids, tids, nids);

        data = builder.Build();

        // setup face vertex index lists
        if (!vts.empty()) {
            data.GenerateMeshFaceVertexIndices();
        }
        // setup face uv coordinate index lists
        if (!uvs.empty()) {
            data.GenerateMeshFaceUvIndices();
        }

        // setup face normal index lists
        if (!nrm.empty()) {
            data.GenerateMeshFaceNormalIndices();
        }
    }

    template<class T>
    void ObjReader<T>::ParseVertex(std::vector<T>& data) {
        ExpectToken(NUMBER_TOK);
        T x = static_cast<T>(numberValue);
        ExpectToken(NUMBER_TOK);
        T y = static_cast<T>(numberValue);
        ExpectToken(NUMBER_TOK);
        T z = static_cast<T>(numberValue);
        ExpectToken(NEW_LINE_TOK);

        data.push_back(x);
        data.push_back(y);
        data.push_back(z);
    }

    template<class T>
    void ObjReader<T>::ParseNormal(std::vector<T>& data) {
        ExpectToken(NUMBER_TOK);
        T x = static_cast<T>(numberValue);
        ExpectToken(NUMBER_TOK);
        T y = static_cast<T>(numberValue);
        ExpectToken(NUMBER_TOK);
        T z = static_cast<T>(numberValue);
        ExpectToken(NEW_LINE_TOK);

        data.push_back(x);
        data.push_back(y);
        data.push_back(z);
    }

    template<class T>
    void ObjReader<T>::ParseUV(std::vector<T>& data) {
        ExpectToken(NUMBER_TOK);
        T u = static_cast<T>(numberValue);
        ExpectToken(NUMBER_TOK);
        T v = static_cast<T>(numberValue);

        data.push_back(u);
        data.push_back(v);

        // skip all further tokens until the newline.
        // Added due to hodini way of storing texture coordinates - UVW, instead of UV.
        while (true) {
            if (GetToken() == NEW_LINE_TOK) {
                break;
            }
        }
    }

    template<class T>
    void ObjReader<T>::ParseFace(std::vector<unsigned int>& fvcount,
        std::vector<unsigned int>& fids,
        std::vector<unsigned int>& nids,
        std::vector<unsigned int>& tids) {
        unsigned int fvc = 0;
        for (;;) {
            TokenType tok = GetToken();
            if (tok == NEW_LINE_TOK) {
                break;
            }
            if (tok != NUMBER_TOK) {
                throw ReadingError("Unexpected token");
            }

            fvc++;
            fids.push_back(static_cast<unsigned int>(numberValue) - 1);

            tok = GetToken();
            if (tok == SLASH_TOK) {
                tok = GetToken();
                if (tok == NUMBER_TOK) {
                    tids.push_back(static_cast<unsigned int>(numberValue) - 1);
                    tok = GetToken();
                }
                if (tok == SLASH_TOK) {
                    ExpectToken(NUMBER_TOK);
                    nids.push_back(static_cast<unsigned int>(numberValue) - 1);
                }
                else {
                    PutBackToken(tok);
                }
            }
            else {
                PutBackToken(tok);
            }
        }

        if (fvc < 3) {
            throw ReadingError("Face data malformed - should be at least 3 vertices in a face but got less.");
        }

        fvcount.push_back(fvc);
    }

    template<class T>
    void ObjReader<T>::ExpectToken(TokenType expectedToken) {
        TokenType currentToken = GetToken();
        if (currentToken != expectedToken) {
            throw ReadingError("Unexpected token");
        }
    }

    // Writer
    WriterError::WriterError(const char* msg) : std::runtime_error(msg) {
    }

    WriterError::~WriterError() {
    }

    template<class T>
    ObjWriter<T>::ObjWriter() {
    }

    template<class T>
    ObjWriter<T>::ObjWriter(const std::string& path) {
        Open(path);
    }

    template<class T>
    ObjWriter<T>::~ObjWriter() {
    }

    template<class T>
    bool ObjWriter<T>::Open(const std::string& path) {
        file.open(path);

        // not sure about this
        file << std::setprecision(std::numeric_limits<long double>::digits10 - 7);
        return file.is_open();
    }

    template<class T>
    bool ObjWriter<T>::Close() {
        file.close();
        return !file.is_open();
    }

    template<class T>
    void ObjWriter<T>::Flush(MeshData<T>& data) {
        if (!file.is_open()) {
            throw WriterError("File is not opened for writing.");
        }

        size_t vertexCount = data.NumVertices();
        size_t faceCount = data.NumFaces();
        size_t normalCount = data.NumNormals();
        size_t uvCount = data.NumUvSets();
        if (uvCount > 1) {
            throw WriterError("OBJ format does not support multiple UV sets.");
        }

        T* vertexData = reinterpret_cast<T*>(data.GetVertexDataPtr());
        T* normalData = reinterpret_cast<T*>(data.GetNormalDataPtr());
        const size_t* faceVertexCountData = reinterpret_cast<const size_t*>(data.GetFaceVertexCountsDataPtr());
        const size_t* faceVertexIndexData = reinterpret_cast<const size_t*>(data.GetFaceVertexIndicesDataPtr());
        const size_t* faceUvIndexData = reinterpret_cast<const size_t*>(data.GetFaceUvIndicesDataPtr());
        const size_t* faceNormalIndexData = reinterpret_cast<const size_t*>(data.GetFaceNormalIndicesDataPtr());

        std::vector<std::vector<size_t>> faceData = data.GetFaceVertexIndexLists();

        WriteHeader(vertexCount, faceCount);
        WriteVertices(vertexCount, vertexData);

        if (uvCount) {
            for (size_t i = 0; i < uvCount; i++) {
                size_t setSize = data.NumUvSetCoords(i);
                T* uvData = reinterpret_cast<T*>(data.GetUvDataPtr(i));

                WriteUV(setSize, uvData);
            }
        }

        if (normalData) {
            WriteNormals(normalCount, normalData);
        }

        WriteFaces(faceCount, faceVertexCountData, faceVertexIndexData, faceUvIndexData, faceNormalIndexData);
    }

    template<class T>
    void ObjWriter<T>::WriteHeader(size_t vertexCount, size_t faceCount) {
        file << "# This file is generated by Epic Games Carbon OBJ Writer.\n";
        file << "# number of vertices: " << std::to_string(vertexCount) << "\n";
        file << "# number of faces: " << std::to_string(faceCount) << "\n";
    }

    template<class T>
    void WriteVerticesImpl(size_t count, T* data, std::ofstream& file) {
        for (size_t i = 0; i < count; i++) {
            const T* v = data + i;
            file << "v " << v[0] << " " << v[count] << " " << v[2* count] << "\n";
        }
    }

    template<class T>
    void WriteUVImpl(size_t count, const T* data, std::ofstream& file) {
        for (size_t i = 0; i < count; i++) {
            const T* v = data + i;
            file << "vt " << v[0] << " " << v[count] << "\n";
        }
    }

    template<class T>
    void WriteNormalsImpl(size_t count, const T* data, std::ofstream& file) {
        for (size_t i = 0; i < count; i++) {
            const T* v = data + i;
            file << "vn " << v[0] << " " << v[count] << " " << v[2* count] << "\n";
        }
    }

    template<class T>
    void ObjWriter<T>::WriteVertices(size_t vertexCount, const T* vertexData) {
        WriteVerticesImpl(vertexCount, reinterpret_cast<const T*>(vertexData), file);
    }

    template<class T>
    void ObjWriter<T>::WriteUV(size_t uvCount, const T* uvData) {
        WriteUVImpl(uvCount, reinterpret_cast<const T*>(uvData), file);
    }

    template<class T>
    void ObjWriter<T>::WriteNormals(size_t normalCount, const T* normalData) {
        WriteNormalsImpl(normalCount, reinterpret_cast<const T*>(normalData), file);
    }

    template<class T>
    void WriteFacesVerticeAndUV(size_t faceCount,
        const T* faceVertexCountData,
        const size_t* faceVertexData,
        const size_t* faceUvData,
        std::ofstream& file) {
        for (size_t i = 0, k = 0; i < faceCount; i++) {
            int fvCount = static_cast<int>(faceVertexCountData[i]);
            file << "f ";
            for (int j = 0; j < fvCount; j++, k++) {
                file<< "v " + std::to_string(faceVertexData[k] + 1) + "/" + std::to_string(faceUvData[k] + 1) << " ";
            }
            file << "\n";
        }
    }

    template<class T>
    void WriteFacesVerticeAndNormals(size_t faceCount,
        const T* faceVertexCountData,
        const size_t* faceVertexData,
        const size_t* faceNormalData,
        std::ofstream& file) {
        for (size_t i = 0, k = 0; i < faceCount; i++) {
            int fvCount = static_cast<int>(faceVertexCountData[i]);
            file << "f ";
            for (int j = 0; j < fvCount; j++, k++) {
                file << std::to_string(faceVertexData[k] + 1) << "/" << std::to_string(faceNormalData[k] + 1) << " ";
            }
            file << "\n";
        }
    }

    template<class T>
    void WriteFacesFull(size_t faceCount,
        const T* faceVertexCountData,
        const size_t* faceVertexData,
        const size_t* faceUvData,
        const size_t* faceNormalData,
        std::ofstream& file) {
        for (size_t i = 0, k = 0; i < faceCount; i++) {
            int fvCount = static_cast<int>(faceVertexCountData[i]);

            file << "f ";
            for (int j = 0; j < fvCount; j++, k++) {
                file <<std::to_string(faceVertexData[k] + 1) << "/" << std::to_string(faceUvData[k] + 1) << "/" << std::to_string(faceNormalData[k] + 1) << " ";
            }
            file << "\n";
        }
    }

    void WriteFacesVertices(size_t faceCount, const size_t* faceVertexCountData, const size_t* faceVertexData, std::ofstream& file) {
        size_t cv = 0;
        for (size_t i = 0, k = 0; i < faceCount; i++) {
            int fvCount = static_cast<int>(faceVertexCountData[i]);
            file << "f ";
            for (int j = 0; j < fvCount; j++, k++) {
                file << std::to_string(faceVertexData[cv] + 1) << "/";
                cv++;
            }
            file << "\n";
        }
    }

    template<class T>
    void ObjWriter<T>::WriteFaces(size_t faceCount,
        const size_t* faceVertexCountData,
        const size_t* faceVertexData,
        const size_t* faceUvData,
        const size_t* faceNormalData) {

        const size_t* fcd = faceVertexCountData;
        auto fvd = reinterpret_cast<const size_t*>(faceVertexData);
        auto fuvd = reinterpret_cast<const size_t*>(faceUvData);
        auto fnd = reinterpret_cast<const size_t*>(faceNormalData);
        auto fvcd = reinterpret_cast<const size_t*>(faceVertexCountData);

        if (!fvd) {
            throw WriterError("Mesh does not contain face-vertex indices.");
        }

        if (fuvd && fnd) {
            WriteFacesFull(faceCount, fcd, fvd, fuvd, fnd, file);
        }
        else if (!fuvd && fnd) {
            WriteFacesVerticeAndNormals(faceCount, fcd, fvd, fnd, file);
        }
        else if (fuvd && !fnd) {
            WriteFacesVerticeAndUV(faceCount, fcd, fvd, fuvd, file);
        }
        else {
            WriteFacesVertices(faceCount, fvcd, fvd, file);
        }
    }

    template class ObjReader<float>;
    template class ObjReader<double>;
    template class ObjWriter<float>;
    template class ObjWriter<double>;
}
}
