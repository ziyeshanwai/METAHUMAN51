// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/serialization/ObjFileFormat.h>
#include <nls/geometry/Mesh.h>
#include <carbon/io/Utils.h>
#include <carbon/utils/TaskThreadPool.h>

#include <stdlib.h>
#include <string.h>
#include <thread>

#if defined(__clang__)
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
__pragma(warning(disable:4996)) // fopen deprecation warning, use secure fopen_s
#endif

using namespace std;

namespace epic {
namespace nls {

template <class T>
class ObjParseHelper {
public:
    void parse(const char* data, size_t size);

    bool hasError() const { return (m_error != ObjFileReader<T>::ErrorType::NONE); }
    typename ObjFileReader<T>::ErrorType getError() const { return m_error; }

    size_t NumVertices() const { return m_vertices.size(); }
    size_t NumColors() const { return m_colors.size(); }
    size_t NumNormals() const { return m_normals.size(); }
    size_t NumTexcoords() const { return m_uvs.size(); }
    size_t NumTris() const { return m_tris.size(); }
    size_t NumQuads() const { return m_quads.size(); }
    size_t NumTexTris() const { return m_tex_tris.size(); }
    size_t NumTexQuads() const { return m_tex_quads.size(); }

    Eigen::Map<const Eigen::Matrix<T, 3, -1>> Vertices() const { return Eigen::Map<const Eigen::Matrix<T, 3, -1>>((const T*)m_vertices.data(), 3, m_vertices.size()); }
    Eigen::Map<const Eigen::Matrix<T, 3, -1>> Colors() const { return Eigen::Map<const Eigen::Matrix<T, 3, -1>>((const T*)m_colors.data(), 3, m_colors.size()); }
    Eigen::Map<const Eigen::Matrix<T, 3, -1>> Normals() const { return Eigen::Map<const Eigen::Matrix<T, 3, -1>>((const T*)m_normals.data(), 3, m_normals.size()); }
    Eigen::Map<const Eigen::Matrix<T, 2, -1>> Texcoords() const { return Eigen::Map<const Eigen::Matrix<T, 2, -1>>((const T*)m_uvs.data(), 2, m_uvs.size()); }
    Eigen::Map<const Eigen::Matrix<int, 3, -1>> Tris() const { return Eigen::Map<const Eigen::Matrix<int, 3, -1>>((const int*)m_tris.data(), 3, m_tris.size()); }
    Eigen::Map<const Eigen::Matrix<int, 4, -1>> Quads() const { return Eigen::Map<const Eigen::Matrix<int, 4, -1>>((const int*)m_quads.data(), 4, m_quads.size()); }
    Eigen::Map<const Eigen::Matrix<int, 3, -1>> TexTris() const { return Eigen::Map<const Eigen::Matrix<int, 3, -1>>((const int*)m_tex_tris.data(), 3, m_tex_tris.size()); }
    Eigen::Map<const Eigen::Matrix<int, 4, -1>> TexQuads() const { return Eigen::Map<const Eigen::Matrix<int, 4, -1>>((const int*)m_tex_quads.data(), 4, m_tex_quads.size()); }

private:
    enum TokenType {
        NONE_TOK, VERTEX_TOK, UV_TOK, NORMAL_TOK, NUMBER_TOK,
        FACE_TOK, SLASH_TOK, NEW_LINE_TOK, EOF_TOK, COMMENT_TOK
    };

    int getChar();
    void putBackChar();
    void eatWhiteSpace();
    void expectEndOfLineOrFile();
    bool isAtEndOfLineOrFile();
    bool checkSlash();
    TokenType getTokenFromStream();
    void parseVertex();
    void parseNormal();
    void parseUV();
    void parseFace();
    T getValueFromStream();
    int getIndexFromStream();

private:
    const char* m_data;
    size_t m_size;
    size_t m_pos;
    std::vector<char> m_buffer;
    typename ObjFileReader<T>::ErrorType m_error;

    std::vector<Eigen::Vector3<T>> m_vertices;
    std::vector<Eigen::Vector3<T>> m_colors;
    std::vector<Eigen::Vector3<T>> m_normals;
    std::vector<Eigen::Vector2<T>> m_uvs;

    std::vector<Eigen::Vector3i> m_tris;
    std::vector<Eigen::Vector4i> m_quads;
    std::vector<Eigen::Vector3i> m_tex_tris;
    std::vector<Eigen::Vector4i> m_tex_quads;
};

template <class T>
void ObjParseHelper<T>::parse(const char* data, size_t size)
{
    m_vertices.clear();
    m_colors.clear();
    m_normals.clear();
    m_uvs.clear();
    m_tris.clear();
    m_quads.clear();
    m_tex_tris.clear();
    m_tex_quads.clear();

    m_data = data;
    m_size = size;
    m_pos = 0;
    m_error = ObjFileReader<T>::ErrorType::NONE;

    while (!hasError()) {
        TokenType tok = getTokenFromStream();
        // LOG_INFO("getting token {} at pos {} of {}", tok, pos, streamData.size());

        switch (tok) {
            case NONE_TOK:
                break;
            case VERTEX_TOK:
                parseVertex();
                break;
            case UV_TOK:
                parseUV();
                break;
            case NORMAL_TOK:
                parseNormal();
                break;
            case FACE_TOK:
                parseFace();
                break;
            case NEW_LINE_TOK:
                break;
            case COMMENT_TOK:
                break;
            case EOF_TOK:
                return;
            default:
                m_error = ObjFileReader<T>::ErrorType::SYNTAX;
                break;
        }
    }
}


template <class T>
int ObjParseHelper<T>::getChar() {
    if (m_pos == m_size) return EOF;
    return m_data[m_pos++];
}

template <class T>
void ObjParseHelper<T>::putBackChar() {
    --m_pos;
}

template <class T>
void ObjParseHelper<T>::eatWhiteSpace() {
    int c = getChar();
    while (c == ' ' || c == '\t') {
        c = getChar();
    }
    if (c != EOF) {
        putBackChar();
    }
}

template <class T>
bool ObjParseHelper<T>::isAtEndOfLineOrFile()
{
    eatWhiteSpace();
    int c = getChar();
    if (c == '\n' || c == '\r' || c == EOF) {
        return true;
    }
    putBackChar();
    return false;
}

template <class T>
void ObjParseHelper<T>::expectEndOfLineOrFile()
{
    if (!isAtEndOfLineOrFile()) {
        m_error = ObjFileReader<T>::ErrorType::UNEXPECTED_TOK;
    }
}

template <class T>
bool ObjParseHelper<T>::checkSlash()
{
    eatWhiteSpace();
    if (m_pos < m_size) {
        if (m_data[m_pos] == '/') {
            m_pos++;
            return true;
        }
        return false;
    }
    return false;
}

template <class T>
typename ObjParseHelper<T>::TokenType ObjParseHelper<T>::getTokenFromStream() {
    eatWhiteSpace();
    int c = getChar();
    if (c == EOF) {
        return EOF_TOK;
    }

    if ((c >= 'a') && (c <= 'z')) {
        if (c == 'v') {
            c = getChar();
            if (c == 't') {
                return UV_TOK;
            } else if (c == 'n') {
                return NORMAL_TOK;
            } else {
                return VERTEX_TOK;
            }
        } else if (c == 'f') {
            return FACE_TOK;
        }
        // Treat all other commands as a comment.
        c = getChar();
        while (c != '\n' && c != '\r' && c != EOF) {
            c = getChar();
        }
        if (c != EOF) {
            putBackChar();
        }
        return COMMENT_TOK;
    }
    if (c == '#') {
        for (;;) {
            c = getChar();
            if ((c == '\n') || (c == '\r') || (c == EOF)) {
                if (c != EOF) {
                    putBackChar();
                }
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
    m_error = ObjFileReader<T>::ErrorType::LEXER;
    // error("syntax error in lexer");
    return NONE_TOK;
}

template <class T>
T ObjParseHelper<T>::getValueFromStream()
{
    eatWhiteSpace();
    int c = getChar();
    if ((c == '-') || (c == '+') || ((c >= '0') && (c <= '9')) || (c == '.')) {
        m_buffer.clear();
        while (c == '-' || c == '+' || (c >= '0' && c <= '9') || c == '.'  || c == 'e' || c == 'E') {
            m_buffer.push_back((char)c);
            c = getChar();
        }
        if (c != EOF) {
            putBackChar();
        }
        m_buffer.push_back(0);
        T value = T(atof(&m_buffer[0]));
        return value;
    } else {
        m_error = ObjFileReader<T>::ErrorType::UNEXPECTED_TOK;
        return 0;
    }
}

template <class T>
int ObjParseHelper<T>::getIndexFromStream()
{
    eatWhiteSpace();
    int c = getChar();
    if ((c >= '0') && (c <= '9')) {
        int value = 0;
        while (c >= '0' && c <= '9') {
            value = value * 10 + (c - '0');
            c = getChar();
        }
        if (c != EOF) {
            putBackChar();
        }
        return value;
    } else {
        m_error = ObjFileReader<T>::ErrorType::UNEXPECTED_TOK;
        return -1;
    }
}

template <class T>
void ObjParseHelper<T>::parseVertex() {
    T x = getValueFromStream();
    T y = getValueFromStream();
    T z = getValueFromStream();
    // currently not supporting a w coordinate
    if (hasError()) return;

    m_vertices.push_back(Eigen::Vector3<T>(x, y, z));

    if (isAtEndOfLineOrFile()) {
        return;
    }

    // supporting color values per vertex
    T r = getValueFromStream();
    T g = getValueFromStream();
    T b = getValueFromStream();
    expectEndOfLineOrFile();
    if (hasError()) return;

    m_colors.push_back(Eigen::Vector3<T>(r, g, b));
}

template <class T>
void ObjParseHelper<T>::parseNormal() {
    T x = getValueFromStream();
    T y = getValueFromStream();
    T z = getValueFromStream();
    expectEndOfLineOrFile();
    if (hasError()) return;

    m_normals.push_back(Eigen::Vector3<T>(x, y, z));
}

template <class T>
void ObjParseHelper<T>::parseUV() {
    T u = getValueFromStream();
    T v = getValueFromStream();
    // currently not supporting a 3rd w coordinate
    expectEndOfLineOrFile();
    if (hasError()) return;

    m_uvs.push_back(Eigen::Vector2<T>(u, T(1) - v));
}

template <class T>
void ObjParseHelper<T>::parseFace() {
    std::vector<Eigen::Vector3i> face;
    for (;!hasError();) {
        if (isAtEndOfLineOrFile()) {
            break;
        }

        const int vID = getIndexFromStream();
        Eigen::Vector3i vertex(vID - 1, -1, -1);
        if (checkSlash()) {
            if (checkSlash()) {
                // format num//num
                vertex[2] = getIndexFromStream() - 1;
            } else {
                // format num/num
                vertex[1] = getIndexFromStream() - 1;
                if (checkSlash()) {
                    // format num/num/num
                    vertex[2] = getIndexFromStream() - 1;
                }
            }
        }

        face.push_back(vertex);
    }

    if (face.size() == 3) {
        m_tris.push_back(Eigen::Vector3i(face[0][0], face[1][0], face[2][0]));
        m_tex_tris.push_back(Eigen::Vector3i(face[0][1], face[1][1], face[2][1]));
    } else if (face.size() == 4) {
        m_quads.push_back(Eigen::Vector4i(face[0][0], face[1][0], face[2][0], face[3][0]));
        m_tex_quads.push_back(Eigen::Vector4i(face[0][1], face[1][1], face[2][1], face[3][1]));
    } else {
        m_error = ObjFileReader<T>::ErrorType::UNEXPECTED_FACE_SIZE;
    }
}

static std::vector<size_t> splitStringEvenlyBasedOnWhitespace(const std::string& data, size_t numParts)
{
    if (numParts <= 1) {
        return {0, data.size()};
    }
    std::vector<size_t> splitPositions = {0};
    const size_t totalSize = data.size();
    const size_t partSize = totalSize / numParts;
    for (size_t k = 1; k < numParts; ++k) {
        size_t proposedSplit = k * partSize;
        while (proposedSplit < totalSize && data[proposedSplit] != '\n' && data[proposedSplit] != '\r') {
            proposedSplit++;
        }
        splitPositions.push_back(proposedSplit);
    }
    splitPositions.push_back(totalSize);
    return splitPositions;
}

template <class T>
bool ObjFileReader<T>::readObj(std::string filename, Mesh<T>& mesh, const std::shared_ptr<epic::carbon::TaskThreadPool>& taskThreadPool) {
    const std::string& streamData = carbon::ReadFile(filename);

    const size_t numParts = taskThreadPool ? taskThreadPool->NumThreads() : 1U;
    const std::vector<size_t> splits = splitStringEvenlyBasedOnWhitespace(streamData, numParts);

    std::vector<ObjParseHelper<T>> multipleHelpers(splits.size() - 1);
    auto parseFunc = [&](size_t i){ multipleHelpers[i].parse(streamData.data() + splits[i], splits[i + 1] - splits[i]); };

    epic::carbon::TaskFutures taskFutures;
    for (size_t k = 0; k < multipleHelpers.size(); ++k) {
        if (taskThreadPool) {
            taskFutures.Add(taskThreadPool->AddTask(std::bind(parseFunc, k)));
        } else {
            parseFunc(k);
        }
    }
    if (taskThreadPool) {
        taskFutures.Wait();
    }
    for (size_t k = 0; k < multipleHelpers.size(); ++k) {
        if (multipleHelpers[k].hasError()) {
            LOG_ERROR("error reading file {}: {} ({})", filename, ErrorTypeToString(multipleHelpers[k].getError()), multipleHelpers[k].getError());
            return false;
        }
    }

    // merge all the parsed data
    Mesh<T> newMesh;
    size_t numVertices = 0;
    size_t numTriangles = 0;
    size_t numQuads = 0;
    size_t numTexcoords = 0;
    size_t numTexTris = 0;
    size_t numTexQuads = 0;
    for (size_t k = 0; k < multipleHelpers.size(); ++k) {
        numVertices += multipleHelpers[k].NumVertices();
        numTriangles += multipleHelpers[k].NumTris();
        numQuads += multipleHelpers[k].NumQuads();
        numTexcoords += multipleHelpers[k].NumTexcoords();
        numTexTris += multipleHelpers[k].NumTexTris();
        numTexQuads += multipleHelpers[k].NumTexQuads();
    }

    Eigen::Matrix<T, 3, -1> vertices(3, numVertices);
    Eigen::Matrix<int, 3, -1> tris(3, numTriangles);
    Eigen::Matrix<int, 4, -1> quads(4, numQuads);
    Eigen::Matrix<T, 2, -1> texcoords(2, numTexcoords);
    Eigen::Matrix<int, 3, -1> tex_tris(3, numTexTris);
    Eigen::Matrix<int, 4, -1> tex_quads(4, numTexQuads);

    size_t indexVertices = 0;
    size_t indexTriangles = 0;
    size_t indexQuads = 0;
    size_t indexTexcoords = 0;
    size_t indexTexTris = 0;
    size_t indexTexQuads = 0;
    for (size_t k = 0; k < multipleHelpers.size(); ++k) {
        vertices.block(0, indexVertices, 3, multipleHelpers[k].NumVertices()) = multipleHelpers[k].Vertices();
        indexVertices += multipleHelpers[k].NumVertices();
        tris.block(0, indexTriangles, 3, multipleHelpers[k].NumTris()) = multipleHelpers[k].Tris();
        indexTriangles += multipleHelpers[k].NumTris();
        quads.block(0, indexQuads, 4, multipleHelpers[k].NumQuads()) = multipleHelpers[k].Quads();
        indexQuads += multipleHelpers[k].NumQuads();
        texcoords.block(0, indexTexcoords, 2, multipleHelpers[k].NumTexcoords()) = multipleHelpers[k].Texcoords();
        indexTexcoords += multipleHelpers[k].NumTexcoords();
        tex_tris.block(0, indexTexTris, 3, multipleHelpers[k].NumTexTris()) = multipleHelpers[k].TexTris();
        indexTexTris += multipleHelpers[k].NumTexTris();
        tex_quads.block(0, indexTexQuads, 4, multipleHelpers[k].NumTexQuads()) = multipleHelpers[k].TexQuads();
        indexTexQuads += multipleHelpers[k].NumTexQuads();
    }

    newMesh.SetVertices(vertices);
    newMesh.SetTriangles(tris);
    newMesh.SetQuads(quads);
    if (texcoords.cols() > 0) {
        newMesh.SetTexcoords(texcoords);
        newMesh.SetTexTriangles(tex_tris);
        newMesh.SetTexQuads(tex_quads);
    }

    if (newMesh.HasTexcoords()) {
        if (newMesh.TexTriangles().cols() != newMesh.Triangles().cols()
         || newMesh.TexQuads().cols() != newMesh.Quads().cols()) {
            LOG_ERROR("error reading file {}: {} ({})", filename, ErrorTypeToString(ErrorType::FACES_WITHOUT_UVS), ErrorType::FACES_WITHOUT_UVS);
            return false;
        }
    }
    std::swap(mesh, newMesh);

    return  true;
}


template <class T>
const char* ObjFileReader<T>::ErrorTypeToString(ErrorType errorType)
{
    switch (errorType) {
        case ErrorType::NONE: return "no error";
        case ErrorType::LEXER: return "lexer error";
        case ErrorType::SYNTAX: return "syntax error";
        case ErrorType::FACES_WITHOUT_UVS: return "faces are missing uvs";
        case ErrorType::UNEXPECTED_TOK: return "unexpected token";
        case ErrorType::UNEXPECTED_FACE_SIZE: return "unsupported face size";
        default: return "unknown error";
    }
}


template <class T>
ObjFileWriter<T>::ObjFileWriter() {
    m_stream = 0;
    m_mesh = 0;
}

template <class T>
ObjFileWriter<T>::~ObjFileWriter() {
    closeStream();
}

template <class T>
void ObjFileWriter<T>::writeObj(const Mesh<T>& mesh, std::string fileName, bool withTexture) {
    this->m_mesh = &mesh;
    openStream(fileName.c_str());
    writeHeader();
    writeVertices();
    if (withTexture) {
        writeUV();
    }
    writeNormals();
    writeFaces(withTexture);
    closeStream();
}

template <class T>
void ObjFileWriter<T>::openStream(std::string filename) {
    closeStream();
    m_stream = fopen(filename.c_str(), "wt");
    if (m_stream == 0) {
        error("Unable to open File " + filename);
    }
}

template <class T>
void ObjFileWriter<T>::closeStream() {
    if (m_stream) {
        fclose(m_stream);
        m_stream = 0;
    }
}

template <class T>
void ObjFileWriter<T>::writeHeader() {
    fprintf(m_stream, "# This file is generated by ObjFileWriter.\n");
    fprintf(m_stream, "# number of vertices:%d\n", m_mesh->NumVertices());
    fprintf(m_stream, "# number of tris:%d\n", m_mesh->NumTriangles());
    fprintf(m_stream, "# number of quads:%d\n", m_mesh->NumQuads());
}

template <class T>
void ObjFileWriter<T>::writeVertices() {
    CARBON_PRECONDITION(m_stream, "stream must be valid");

    for (int i = 0; i < m_mesh->NumVertices(); i++) {
        fprintf(m_stream, "v %.12lf %.12lf %.12lf\n", static_cast<float>(m_mesh->Vertices()(0, i)), static_cast<float>(m_mesh->Vertices()(1, i)), static_cast<float>(m_mesh->Vertices()(2, i)));
    }
    // TODO: write colors
}

template <class T>
void ObjFileWriter<T>::writeUV() {
    CARBON_PRECONDITION(m_stream, "stream must be valid");
    for (int i = 0; i < m_mesh->Texcoords().cols(); i++) {
        // vertical flip when outputting the UVs as the UVs are stored with pixel in bottom left corner
        fprintf(m_stream, "vt %.12lf %.12lf\n", static_cast<float>(m_mesh->Texcoords()(0, i)), static_cast<float>(T(1) - m_mesh->Texcoords()(1, i)));
    }
}

template <class T>
void ObjFileWriter<T>::writeNormals() {
    CARBON_PRECONDITION(m_stream, "stream must be valid");
    // TODO: write Normals
}

template <class T>
void ObjFileWriter<T>::writeFaces(bool withTexture) {
    CARBON_PRECONDITION(m_stream, "stream must be valid");
    for (int i = 0; i < m_mesh->NumTriangles(); i++) {
        if (withTexture && m_mesh->Triangles().cols() == m_mesh->TexTriangles().cols()) {
            fprintf(m_stream, "f %d/%d %d/%d %d/%d\n",
                    m_mesh->Triangles()(0, i) + 1, m_mesh->TexTriangles()(0, i) + 1,
                    m_mesh->Triangles()(1, i) + 1, m_mesh->TexTriangles()(1, i) + 1,
                    m_mesh->Triangles()(2, i) + 1, m_mesh->TexTriangles()(2, i) + 1);
        } else {
            fprintf(m_stream, "f %d %d %d\n", m_mesh->Triangles()(0, i) + 1, m_mesh->Triangles()(1, i) + 1, m_mesh->Triangles()(2, i) + 1);
        }
    }
    for (int i = 0; i < m_mesh->NumQuads(); i++) {
        if (withTexture && m_mesh->Quads().cols() == m_mesh->TexQuads().cols()) {
            fprintf(m_stream, "f %d/%d %d/%d %d/%d %d/%d\n",
                    m_mesh->Quads()(0, i) + 1, m_mesh->TexQuads()(0, i) + 1,
                    m_mesh->Quads()(1, i) + 1, m_mesh->TexQuads()(1, i) + 1,
                    m_mesh->Quads()(2, i) + 1, m_mesh->TexQuads()(2, i) + 1,
                    m_mesh->Quads()(3, i) + 1, m_mesh->TexQuads()(3, i) + 1);
        } else {
            fprintf(m_stream, "f %d %d %d %d\n", m_mesh->Quads()(0, i) + 1, m_mesh->Quads()(1, i) + 1, m_mesh->Quads()(2, i) + 1, m_mesh->Quads()(3, i) + 1);
        }
    }
    // TODO: write uv and normal face indices
}

template <class T>
void ObjFileWriter<T>::error(std::string msg) {
    throw std::runtime_error(msg.c_str());
}

template class ObjFileReader<float>;
template class ObjFileReader<double>;
template class ObjFileWriter<float>;
template class ObjFileWriter<double>;

}
}
