// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/Mesh.h>

#include <nls/geometry/Triangle.h>
#include <carbon/utils/Profiler.h>

#include <set>
#include <vector>

namespace epic::nls {


template <class T>
bool Mesh<T>::Validate(bool checkVertexUsage) const
{
    Eigen::Matrix<int, -1, 1> usage = Eigen::Matrix<int, -1, 1>::Zero(NumVertices());

    for (int i = 0; i < NumTriangles(); i++) {
        for (int k = 0; k < 3; k++) {
            if (Triangles()(k,i) < 0 || Triangles()(k,i) >= NumVertices()) {
                return false;
            }
            if (checkVertexUsage) {
                usage[Triangles()(k,i)]++;
            }
        }
    }
    for (int i = 0; i < NumQuads(); i++) {
        for (int k = 0; k < 4; k++) {
            if (Quads()(k,i) < 0 || Quads()(k,i) >= NumVertices()) {
                return false;
            }
            if (checkVertexUsage) {
                usage[Quads()(k,i)]++;
            }
        }
    }

    if (checkVertexUsage) {
        for (int i = 0; i < int(usage.size()); i++) {
            if (usage[i] == 0) {
                return false;
            }
        }
    }

    // check texture indices - if we have texcoords then we also need to have texture coordinates for every triangle and quad
    if (m_texcoords.size() > 0 && m_tex_tris.cols() != m_tris.cols()) {
        return false;
    }
    if (m_texcoords.size() > 0 && m_tex_quads.cols() != m_quads.cols()) {
        return false;
    }

    return true;
}


template <class T>
void Mesh<T>::SetTopology(const Eigen::VectorXi& polygons, const Eigen::VectorXi& vIDs)
{
    if (m_tris.size() > 0 || m_quads.size() || m_tex_tris.size() > 0 || m_tex_quads.size() > 0) {
        CARBON_CRITICAL("Setting mesh topology only supported for empty meshes");
    }
    int numTris = 0;
    int numQuads = 0;
    for (int i = 0; i < int(polygons.size()); i++) {
        if (polygons[i] == 3) {
            numTris++;
        } else if (polygons[i] == 4) {
            numQuads++;
        } else {
            CARBON_CRITICAL("Mesh only supports triangles and quads.");
        }
    }
    Eigen::Matrix<int, 3, -1> tris(3, numTris);
    Eigen::Matrix<int, 4, -1> quads(4, numQuads);
    int vtxIDcounter = 0;
    int triIndex = 0;
    int quadIndex = 0;
    for (int i = 0; i < int(polygons.size()); i++) {
        if (polygons[i] == 3) {
            for (int k = 0; k < 3; k++) {
                tris(k, triIndex) = vIDs[vtxIDcounter++];
            }
            triIndex++;
        } else if (polygons[i] == 4) {
            for (int k = 0; k < 4; k++) {
                quads(k, quadIndex) = vIDs[vtxIDcounter++];
            }
            quadIndex++;
        }
    }
    m_tris = tris;
    m_quads = quads;
}


template <class T>
void Mesh<T>::Triangulate()
{
    if (NumQuads() == 0) return;

    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    const int newNumTriangles = NumQuads() * 2 + NumTriangles();
    Eigen::Matrix<int, 3, -1> newTriangles(3, newNumTriangles);
    newTriangles.leftCols(NumTriangles()) = m_tris;

    Eigen::Matrix<int, 3, -1> newTexTriangles;
    if (HasTexcoords()) {
        const int newNumTexTriangles = NumQuads() * 2 + NumTriangles();
        newTexTriangles.resize(3, newNumTexTriangles);
        newTexTriangles.leftCols(NumTriangles()) = m_tex_tris;
    }

    for (int i = 0; i < NumQuads(); i++) {
        const int tIdx1 = NumTriangles() + 2 * i + 0;
        const int tIdx2 = NumTriangles() + 2 * i + 1;

        // options:
        // 1)                2)
        // v4 --- v3         v4 --- v3
        // |    /  |         |  \    |
        // |  /    |    or   |    \  |
        // v1 --- v2         v1 --- v2
        //
        const Eigen::Vector3<T> v1 = m_vertices.col(m_quads(0,i));
        const Eigen::Vector3<T> v2 = m_vertices.col(m_quads(1,i));
        const Eigen::Vector3<T> v3 = m_vertices.col(m_quads(2,i));
        const Eigen::Vector3<T> v4 = m_vertices.col(m_quads(3,i));

        const Eigen::Vector3<T> n1_option1 = (v2 - v1).cross(v3-v1);
        const Eigen::Vector3<T> n2_option1 = (v3 - v1).cross(v4-v1);
        const bool option1_flip = n1_option1.dot(n2_option1) < 0;

        const Eigen::Vector3<T> n1_option2 = (v2 - v1).cross(v4-v1);
        const Eigen::Vector3<T> n2_option2 = (v3 - v2).cross(v4-v2);
        const bool option2_flip = n1_option2.dot(n2_option2) < 0;

        bool useOption1 = true;
        if (option1_flip == option2_flip) {
            // use better triangle quality if none flips or both
            const T triangleQuality1_option1 = TriangleQuality<T>(v1, v2, v3);
            const T triangleQuality2_option1 = TriangleQuality<T>(v1, v3, v4);
            const T triangleQuality1_option2 = TriangleQuality<T>(v1, v2, v4);
            const T triangleQuality2_option2 = TriangleQuality<T>(v2, v3, v4);

            if (std::max(triangleQuality1_option1, triangleQuality2_option1) >
                std::max(triangleQuality1_option2, triangleQuality2_option2)) {
                    // option 2 has better quality as the maximum quality value (the higher the worse)
                    useOption1 = false;
            } else {
                // use option 1
            }
        } else if (option1_flip) {
            // use option 2
            useOption1 = false;
        } else {
            // use option 1 as option2 is flipping
        }

        if (useOption1) {
            newTriangles(0, tIdx1) = m_quads(0, i);
            newTriangles(1, tIdx1) = m_quads(1, i);
            newTriangles(2, tIdx1) = m_quads(2, i);
            newTriangles(0, tIdx2) = m_quads(0, i);
            newTriangles(1, tIdx2) = m_quads(2, i);
            newTriangles(2, tIdx2) = m_quads(3, i);
            if (HasTexcoords()) {
                newTexTriangles(0, tIdx1) = m_tex_quads(0, i);
                newTexTriangles(1, tIdx1) = m_tex_quads(1, i);
                newTexTriangles(2, tIdx1) = m_tex_quads(2, i);
                newTexTriangles(0, tIdx2) = m_tex_quads(0, i);
                newTexTriangles(1, tIdx2) = m_tex_quads(2, i);
                newTexTriangles(2, tIdx2) = m_tex_quads(3, i);
            }
        } else {
            newTriangles(0, tIdx1) = m_quads(0, i);
            newTriangles(1, tIdx1) = m_quads(1, i);
            newTriangles(2, tIdx1) = m_quads(3, i);
            newTriangles(0, tIdx2) = m_quads(1, i);
            newTriangles(1, tIdx2) = m_quads(2, i);
            newTriangles(2, tIdx2) = m_quads(3, i);
            if (HasTexcoords()) {
                newTexTriangles(0, tIdx1) = m_tex_quads(0, i);
                newTexTriangles(1, tIdx1) = m_tex_quads(1, i);
                newTexTriangles(2, tIdx1) = m_tex_quads(3, i);
                newTexTriangles(0, tIdx2) = m_tex_quads(1, i);
                newTexTriangles(1, tIdx2) = m_tex_quads(2, i);
                newTexTriangles(2, tIdx2) = m_tex_quads(3, i);
            }
        }
    }
    m_tris = newTriangles;
    Eigen::Matrix<int, 4, -1> newQuads(4, 0);
    m_quads.swap(newQuads);

    if (HasTexcoords()) {
        m_tex_tris = newTexTriangles;
        m_tex_quads.swap(newQuads);
    }
}


template <class T>
Eigen::Vector<T, -1> Mesh<T>::TriangleAreas() const
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    if (NumQuads() > 0) {
        CARBON_CRITICAL("triangle area can only be calculated for pure triangle meshes");
    }

    Eigen::Vector<T, -1> triangleAreas = Eigen::Vector<T, -1>::Zero(NumTriangles());
    for (int i = 0; i < NumTriangles(); i++) {
        const int vID0 = m_tris(0, i);
        const int vID1 = m_tris(1, i);
        const int vID2 = m_tris(2, i);
        const Eigen::Vector3<T> v0 = m_vertices.col(vID0);
        const Eigen::Vector3<T> v1 = m_vertices.col(vID1);
        const Eigen::Vector3<T> v2 = m_vertices.col(vID2);
        const Eigen::Vector3<T> e01 = v1 - v0;
        const Eigen::Vector3<T> e12 = v2 - v1;
        triangleAreas[i] = e01.cross(e12).norm() * T(0.5);
    }

    return triangleAreas;
}


template <class T>
Eigen::Vector<T, -1> Mesh<T>::VertexAreas() const
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    if (NumQuads() > 0) {
        CARBON_CRITICAL("vertex area can only be calculated for pure triangle meshes");
    }

    Eigen::Vector<T, -1> vertexAreas = Eigen::Vector<T, -1>::Zero(NumVertices());
    for (int i = 0; i < NumTriangles(); i++) {
        const int vID0 = m_tris(0, i);
        const int vID1 = m_tris(1, i);
        const int vID2 = m_tris(2, i);
        const Eigen::Vector3<T> v0 = m_vertices.col(vID0);
        const Eigen::Vector3<T> v1 = m_vertices.col(vID1);
        const Eigen::Vector3<T> v2 = m_vertices.col(vID2);
        const Eigen::Vector3<T> e01 = v1 - v0;
        const Eigen::Vector3<T> e12 = v2 - v1;
        const Eigen::Vector3<T> e20 = v0 - v2;
        const T cot0 = cotangent<T>(e01, -e20);
        const T cot1 = cotangent<T>(e12, -e01);
        const T cot2 = cotangent<T>(e20, -e12);
        if (cot0 <= 0 || cot1 <= 0 || cot2 <= 0) {
            const T triangleArea = e01.cross(e12).norm() * T(0.5);
            // obtuse
            if (cot0 <= 0) {
                vertexAreas[vID0] += triangleArea / T(2);
                vertexAreas[vID1] += triangleArea / T(4);
                vertexAreas[vID2] += triangleArea / T(4);
            } else if (cot1 <= 0) {
                vertexAreas[vID0] += triangleArea / T(4);
                vertexAreas[vID1] += triangleArea / T(2);
                vertexAreas[vID2] += triangleArea / T(4);
            } else if (cot2 <= 0) {
                vertexAreas[vID0] += triangleArea / T(4);
                vertexAreas[vID1] += triangleArea / T(4);
                vertexAreas[vID2] += triangleArea / T(2);
            }
        } else {
            // non-obtuse
            vertexAreas[vID0] += T(0.125) * cot1 * e20.squaredNorm();
            vertexAreas[vID0] += T(0.125) * cot2 * e01.squaredNorm();
            vertexAreas[vID1] += T(0.125) * cot0 * e12.squaredNorm();
            vertexAreas[vID1] += T(0.125) * cot2 * e01.squaredNorm();
            vertexAreas[vID2] += T(0.125) * cot0 * e12.squaredNorm();
            vertexAreas[vID2] += T(0.125) * cot1 * e20.squaredNorm();
        }
    }

    return vertexAreas;
}


template <class T>
Eigen::Matrix<T, 3, -1> Mesh<T>::VertexMeanCurvatureNormals() const
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    if (NumQuads() > 0) {
        CARBON_CRITICAL("mean cuverature normals can only be calculated for pure triangle meshes");
    }

    Eigen::Matrix<T, 3, -1> meanCurvatureNormals = Eigen::Matrix<T, 3, -1>::Zero(3, NumVertices());
    for (int i = 0; i < NumTriangles(); i++) {
        const int vID0 = m_tris(0, i);
        const int vID1 = m_tris(1, i);
        const int vID2 = m_tris(2, i);
        const Eigen::Vector3<T> v0 = m_vertices.col(vID0);
        const Eigen::Vector3<T> v1 = m_vertices.col(vID1);
        const Eigen::Vector3<T> v2 = m_vertices.col(vID2);
        const Eigen::Vector3<T> e01 = v1 - v0;
        const Eigen::Vector3<T> e12 = v2 - v1;
        const Eigen::Vector3<T> e20 = v0 - v2;
        const T cot0 = cotangent<T>(e01, -e20);
        const T cot1 = cotangent<T>(e12, -e01);
        const T cot2 = cotangent<T>(e20, -e12);

        meanCurvatureNormals.col(vID0) +=   cot1 * e20;
        meanCurvatureNormals.col(vID0) += - cot2 * e01;

        meanCurvatureNormals.col(vID1) += - cot0 * e12;
        meanCurvatureNormals.col(vID1) +=   cot2 * e01;

        meanCurvatureNormals.col(vID2) +=   cot0 * e12;
        meanCurvatureNormals.col(vID2) += - cot1 * e20;
    }

    Eigen::Vector<T, -1> vertexAreas = VertexAreas();
    for (int i = 0; i < int(meanCurvatureNormals.cols()); i++) {
        if (vertexAreas[i] > 0) {
            meanCurvatureNormals.col(i) /= (T(2) * vertexAreas[i]);
        } else {
            meanCurvatureNormals.col(i).setZero();
        }
    }

    return meanCurvatureNormals;
}


template <class T>
const Eigen::Matrix<T, 3, -1>& Mesh<T>::CalculateVertexNormals(bool recompute)
{
    if (m_normals.cols() == m_vertices.cols() && !recompute) {
        return m_normals;
    }

    CalculateVertexNormals(m_vertices, m_normals, VertexNormalComputationType::VoronoiAreaWeighted, /*stableNormalize=*/true);
    return m_normals;
}

template<class T> T EpsilonForNormalization();
template<> inline float EpsilonForNormalization<float>() { return 1e-9f; }
template<> inline double EpsilonForNormalization<double>() { return 1e-12; }

template <class T>
void Mesh<T>::CalculateVertexNormals(const Eigen::Matrix<T, 3, -1>& vertices,
                                     Eigen::Matrix<T, 3, -1>& normals,
                                     VertexNormalComputationType vertexNormalComputationType,
                                     bool stableNormalize) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    if (NumQuads() > 0) {
        CARBON_CRITICAL("vertex normals can only be calculated for pure triangle meshes");
    }

    if (vertices.cols() != m_vertices.cols()) {
        CARBON_CRITICAL("incorrect number of vertices");
    }

    normals.resize(3, NumVertices());
    normals.setZero();

    switch (vertexNormalComputationType) {
        case VertexNormalComputationType::VoronoiAreaWeighted: {
            // accurate version: weighted by vertex area
            for (int i = 0; i < NumTriangles(); i++) {
                const int vID0 = m_tris(0, i);
                const int vID1 = m_tris(1, i);
                const int vID2 = m_tris(2, i);

                const Eigen::Vector3<T> v0 = vertices.col(vID0);
                const Eigen::Vector3<T> v1 = vertices.col(vID1);
                const Eigen::Vector3<T> v2 = vertices.col(vID2);

                const Eigen::Vector3<T> e01 = v1 - v0;
                const Eigen::Vector3<T> e12 = v2 - v1;
                const Eigen::Vector3<T> e20 = v0 - v2;

                const Eigen::Vector3<T> unnormalizedNormal = e01.cross(-e20);
                const T n_sqr = unnormalizedNormal.squaredNorm();

                if (n_sqr > T(10) * std::numeric_limits<T>::min()) {

                    // const T doubleTriangleArea = std::sqrt(n_sqr);
                    // const Eigen::Vector3<T> normalizedNormal = normal / doubleTriangleArea;
                    const T doubleTriangleAreaSquared = n_sqr;

                    const T l0sq = e12.squaredNorm();
                    const T l1sq = e20.squaredNorm();
                    const T l2sq = e01.squaredNorm();

                    // numerically stable cotangent calculation using side length a, b, and c:
                    // cot(alpha) = cos(alpha)/sin(alpha)
                    // Area = 0.5 * b * c * sin(alpha) => sin(alpha) = 2 * Area / (b * c)
                    // Law of cosines: cos(alpha) = (a^2 - b^2 - c^2) / (- 2 * b * c)
                    // cot(alpha) = cos(alpha)/sin(alpha) = (a^2 - b^2 - c^2) * (b * c) / (2 * Area * (- 2 * b * c))
                    //                                    = - (a^2 - b^2 - c^2) / (4 * Area)
                    //                                    = (b^2 + c^2 - a^2) / (4 * Area)

                    // we do not divide by doubleTriangleArea as further below we delete again by doubleTriangleArea, and in
                    // this case we can divide by the squared value and we do not need to calculate the sqrt
                    const T cot0 = (l1sq + l2sq - l0sq) / (T(2));// * doubleTriangleArea);
                    const T cot1 = (l0sq + l2sq - l1sq) / (T(2));// * doubleTriangleArea);
                    const T cot2 = (l0sq + l1sq - l2sq) / (T(2));// * doubleTriangleArea);

                    T vertexArea0 = 0.0;
                    T vertexArea1 = 0.0;
                    T vertexArea2 = 0.0;

                    if (cot0 <= 0 || cot1 <= 0 || cot2 <= 0) {
                        // obtuse
                        if (cot0 <= 0) {
                            vertexArea0 = T(1) / T(4);
                            vertexArea1 = T(1) / T(8);
                            vertexArea2 = T(1) / T(8);
                        } else if (cot1 <= 0) {
                            vertexArea0 = T(1) / T(8);
                            vertexArea1 = T(1) / T(4);
                            vertexArea2 = T(1) / T(8);
                        } else if (cot2 <= 0) {
                            vertexArea0 = T(1) / T(8);
                            vertexArea1 = T(1) / T(8);
                            vertexArea2 = T(1) / T(4);
                        }
                        normals.col(vID0) += vertexArea0 * unnormalizedNormal; // note that the normal is already scaled by the area of the face
                        normals.col(vID1) += vertexArea1 * unnormalizedNormal;
                        normals.col(vID2) += vertexArea2 * unnormalizedNormal;

                    } else {
                        // non-obtuse
                        vertexArea0 += T(0.125) * cot1 * e20.squaredNorm();
                        vertexArea0 += T(0.125) * cot2 * e01.squaredNorm();
                        vertexArea1 += T(0.125) * cot0 * e12.squaredNorm();
                        vertexArea1 += T(0.125) * cot2 * e01.squaredNorm();
                        vertexArea2 += T(0.125) * cot0 * e12.squaredNorm();
                        vertexArea2 += T(0.125) * cot1 * e20.squaredNorm();

                        normals.col(vID0) += vertexArea0 * unnormalizedNormal / doubleTriangleAreaSquared;
                        normals.col(vID1) += vertexArea1 * unnormalizedNormal / doubleTriangleAreaSquared;
                        normals.col(vID2) += vertexArea2 * unnormalizedNormal / doubleTriangleAreaSquared;
                    }

                }
            }
        } break;
        case VertexNormalComputationType::AreaWeighted: {
            for (int i = 0; i < NumTriangles(); i++) {
                const auto vID0 = m_tris(0, i);
                const auto vID1 = m_tris(1, i);
                const auto vID2 = m_tris(2, i);

                const auto v0 = vertices.col(vID0);
                const auto v1 = vertices.col(vID1);
                const auto v2 = vertices.col(vID2);

                const Eigen::Vector3<T> fn = (v1 - v0).cross(v2 - v0);

                normals.col(vID0) += fn;
                normals.col(vID1) += fn;
                normals.col(vID2) += fn;
            }
        } break;
    }

    if (stableNormalize) {
        for (auto k = 0; k < normals.cols(); ++k) {
            normals.col(k).stableNormalize();
        }
    } else {
        for (auto k = 0; k < normals.cols(); ++k) {
            const T z = normals.col(k).norm();
            if (z > EpsilonForNormalization<T>()) {
                normals.col(k) /= z;
            }
        }
    }
}

/**
 * Helper class to count the number of edges
 */
class EdgeCounting {
public:
    EdgeCounting(const int numVertices, const Eigen::Matrix<int, 3, Eigen::Dynamic>& tris, const Eigen::Matrix<int, 4, Eigen::Dynamic>& quads);

    /**
     * Calls @p edgeCall for each edge with signature void(int ID0, int vID1, int count).
     * Note that using a template parameter is more efficient than using std::function.
     */
    template <typename F>
    void ProcessEdges(const F& edgeCall) const;

    int NumEdges() const { return m_numEdges; }

private:
    inline void AddEdge(int vID0, int vID1);

private:
    int m_numVertices;
    int m_numEdgesPerVertex;
    int m_lineSize;
    int m_nextFreeLine;
    int m_numEdges;
    std::vector<int> m_edgeCountingArray;
};

EdgeCounting::EdgeCounting(const int numVertices, const Eigen::Matrix<int, 3, Eigen::Dynamic>& tris, const Eigen::Matrix<int, 4, Eigen::Dynamic>& quads)
    : m_numVertices(numVertices)
{
    // maximum number of edges is when each face is separate
    const int maxNumEdges = int(tris.cols()) * 3 + int(quads.cols()) * 4;

    // use reasonable number of edges per vertex
    m_numEdgesPerVertex = maxNumEdges / std::max<int>(numVertices, 1);

    // the edge counting array counts for each vertex to which other vertices it is connected
    // in the format [target_vID1, count1, target_vID2, count2, ...], and the last entry of the line points to the next line
    // in case a vertex points to more vertices and therefore another line in the edgeCountingArray is used.
    m_lineSize = (m_numEdgesPerVertex * 2 + 1);
    m_edgeCountingArray = std::vector<int>(static_cast<int>(numVertices * 1.5) * m_lineSize, -1);
    m_nextFreeLine = numVertices;
    m_numEdges = 0;

    for (int i = 0; i < int(tris.cols()); ++i) {
        const int vID0 = tris.coeff(0, i);
        const int vID1 = tris.coeff(1, i);
        const int vID2 = tris.coeff(2, i);
        AddEdge(vID0, vID1);
        AddEdge(vID1, vID2);
        AddEdge(vID2, vID0);
    }
    for (int i = 0; i < int(quads.cols()); ++i) {
        const int vID0 = quads.coeff(0, i);
        const int vID1 = quads.coeff(1, i);
        const int vID2 = quads.coeff(2, i);
        const int vID3 = quads.coeff(3, i);
        AddEdge(vID0, vID1);
        AddEdge(vID1, vID2);
        AddEdge(vID2, vID3);
        AddEdge(vID3, vID0);
    }
}

void EdgeCounting::AddEdge(int vID0, int vID1)
{
    if (vID0 > vID1) std::swap(vID0, vID1);
    bool found = false;
    int offset = vID0 * m_lineSize;
    while (!found) {
        for (int k = 0; k < m_numEdgesPerVertex; ++k) {
            const int savedvID = m_edgeCountingArray[offset + 2 * k + 0];
            if (savedvID < 0) {
                m_edgeCountingArray[offset + 2 * k + 0] = vID1;
                m_edgeCountingArray[offset + 2 * k + 1] = 1;
                m_numEdges++;
                found = true;
                break;
            } else if (savedvID == vID1) {
                m_edgeCountingArray[offset + 2 * k + 1] = 2;
                found = true;
                break;
            }
        }
        if (!found) {
            int nextOffset = m_edgeCountingArray[offset + m_lineSize - 1];
            if (nextOffset < 0) {
                nextOffset = m_nextFreeLine * m_lineSize;
                if (nextOffset == static_cast<int>(m_edgeCountingArray.size())) {
                    // we should normally not get here, but we want to be safe against the case if this should ever happen
                    m_edgeCountingArray.resize(nextOffset + 100 * m_lineSize, -1);
                }
                m_edgeCountingArray[offset + m_lineSize - 1] = nextOffset;
                m_nextFreeLine++;
            }
            offset = nextOffset;
        }
    }
}

template <typename F>
void EdgeCounting::ProcessEdges(const F& edgeCall) const
{
    for (int vID = 0; vID < m_numVertices; ++vID) {
        bool found = false;
        int offset = vID * m_lineSize;
        while (!found) {
            for (int k = 0; k < m_numEdgesPerVertex; ++k) {
                const int savedvID = m_edgeCountingArray[offset + 2 * k + 0];
                if (savedvID < 0) {
                    found = true;
                    break;
                } else {
                    const int count = m_edgeCountingArray[offset + 2 * k + 1];
                    edgeCall(vID, savedvID, count);
                }
            }
            if (!found) {
                offset = m_edgeCountingArray[offset + m_lineSize - 1];
                if (offset < 0) {
                    found = true;
                }
            }
        }
    }
}

template <class T>
std::vector<int> Mesh<T>::CalculateBorderVertices() const
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    EdgeCounting edgeCounting(NumVertices(), m_tris, m_quads);
    std::vector<bool> isBorder(NumVertices(), false);

    edgeCounting.ProcessEdges([&](int vID0, int vID1, int count){
        if (count == 1) {
            isBorder[vID0] = true;
            isBorder[vID1] = true;
        }
    });

    std::vector<int> borderIndices;
    for (size_t i = 0; i < isBorder.size(); ++i) {
        if (isBorder[i]) {
            borderIndices.emplace_back(static_cast<int>(i));
        }
    }

    return borderIndices;
}


template <class T>
std::vector<std::pair<int, int>> Mesh<T>::GetEdges(const std::vector<int>& vIDs) const
{
    if (NumQuads() == 0 && NumTriangles() == 0) return {};

    EdgeCounting edgeCounting(NumVertices(), m_tris, m_quads);
    std::vector<std::pair<int, int>> edges;
    edges.reserve(edgeCounting.NumEdges());

    if (vIDs.size() > 0) {
        std::vector<bool> isUsed(NumVertices(), false);
        for (int vID : vIDs) isUsed[vID] = true;
        edgeCounting.ProcessEdges([&](int vID0, int vID1, int){
            if (isUsed[vID0] && isUsed[vID1]) {
                edges.push_back({vID0, vID1});
            }
        });
    } else {
        edgeCounting.ProcessEdges([&](int vID0, int vID1, int){
            edges.push_back({vID0, vID1});
        });
    }

    return edges;
}


template <class T, int R>
Eigen::Matrix<T, R, -1> RemapFaces(const Eigen::Matrix<T, R, -1>& faces, const std::vector<int>& oldToNewMap)
{
    Eigen::Matrix<T, R, -1> newFaces = faces;

    int currFace = 0;
    for (int id = 0; id < int(faces.cols()); ++id) {
        Eigen::Vector<int, R> ids = faces.col(id);
        for (int k = 0; k < R; ++k) {
            ids[k] = oldToNewMap[ids[k]];
        }
        if (ids.minCoeff() >= 0) {
            newFaces.col(currFace++) = ids;
        }
    }
    newFaces.resize(R, currFace);

    return newFaces;
}

template <class T>
Eigen::Matrix<T, 3, -1> SelectColumns(const Eigen::Matrix<T, 3, -1>& matrixIn, const std::vector<int>& newToOldMap)
{
    Eigen::Matrix<T, 3, -1> matrixOut(3, newToOldMap.size());
    for (int i = 0; i < int(newToOldMap.size()); ++i) {
        matrixOut.col(i) = matrixIn.col(newToOldMap[i]);
    }
    return matrixOut;
}

template <class T>
void Mesh<T>::Resample(const std::vector<int>& newToOldMap)
{
    std::vector<int> oldToNewMap(NumVertices(), -1);
    for (int i = 0; i < int(newToOldMap.size()); ++i) {
        oldToNewMap[newToOldMap[i]] = i;
    }

    // remap vertices
    //m_vertices = m_vertices(Eigen::all, newToOldMap).eval();
    m_vertices = SelectColumns(m_vertices, newToOldMap);

    // remap normals
    if (m_normals.size() > 0) {
        // m_normals = m_normals(Eigen::all, newToOldMap).eval();
        m_normals = SelectColumns(m_normals, newToOldMap);
    }

    m_tris = RemapFaces(m_tris, oldToNewMap);
    m_quads = RemapFaces(m_quads, oldToNewMap);

    // for now discard textures
    m_texcoords.resize(2, 0);
    m_tex_tris.resize(3, 0);
    m_tex_quads.resize(4, 0);
}

template class Mesh<float>;
template class Mesh<double>;

} // namespace epic::nls
