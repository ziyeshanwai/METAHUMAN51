// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/rig/RigGeometry.h>

#include <carbon/utils/TaskThreadPool.h>
#include <carbon/utils/Timer.h>
#include <nls/functions/GatherFunction.h>
#include <nls/geometry/EulerAngles.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/Jacobians.h>
#include <nls/rig/JointRig2.h>
#include <carbon/utils/Profiler.h>

#include <riglogic/RigLogic.h>

NLS_DISABLE_EIGEN_WARNINGS
#include <Eigen/Geometry>
NLS_RENABLE_WARNINGS

#include <algorithm>
#include <iostream>
#include <set>
#include <thread>
#include <vector>

#include <immintrin.h>

namespace epic::nls {

template <class T>
class DiffDataAffineArray {
public:
    void resize(int size)
    {
        m_affineTransforms.resize(size);
    }

    const DiffDataAffine<T, 3, 3>& operator[](std::ptrdiff_t index) const { return m_affineTransforms[index]; }
    /* */ DiffDataAffine<T, 3, 3>& operator[](std::ptrdiff_t index) /* */ { return m_affineTransforms[index]; }

    std::ptrdiff_t size() const { return static_cast<std::ptrdiff_t>(m_affineTransforms.size()); }

private:
    std::vector<DiffDataAffine<T, 3, 3>> m_affineTransforms;
};

template <class T>
struct MeshData
{
    std::string meshName;

    Mesh<T> mesh;

    //! selection of which blendshapes are used by this mesh
    Eigen::VectorXi blendshapeControlsToMeshBlendshapeControls;

    // we keep a dense blendshape matrix for evaluation as the blendshape matrix only has 50% zeros, so sparse matrix
    // multiplication is significantly more expensive
    Eigen::Matrix<T, -1, -1> blendshapeMatrixDense;
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> blendshapeMatrixDenseRM;
};

template <class T>
struct JacobianData {
    //! blendshapes as they were evaluated
    Eigen::Matrix<T, 3, -1> blendshapeVertices;
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> blendshapeJacobianRM;
    bool isBlendshapeJacobianValid = false;

    //! final vertices after applying the joint evaluation
    Eigen::Matrix<T, 3, -1> finalVertices;
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> finalJacobianRM;
    bool isFinalJacobianValid = false;
};

template <class T>
struct RigGeometry<T>::State::Private {
    DiffData<T> jointDeltas = DiffData<T>(Eigen::VectorX<T>());
    DiffDataAffine<T, 3, 3> rigid;

    DiffDataAffineArray<T> jointRig2StateLocalMatrices;
    DiffDataAffineArray<T> jointRig2StateWorldMatrices;
    DiffDataAffineArray<T> jointRig2StateSkinningMatrices;

    std::vector<DiffDataMatrix<T, 3, -1>> vertices;
    std::vector<int> meshIndices;

    //! flag specifying whether the state has been set including Jacobians
    bool withJacobians = false;

    std::vector<Eigen::Transform<T, 3, Eigen::Affine>> localMatrices;
    std::vector<Eigen::Transform<T, 3, Eigen::Affine>> worldMatrices;
    std::vector<Eigen::Transform<T, 3, Eigen::Affine>> skinningMatrices;

    // structure containing temporary data for jacobian calculations

    std::vector<JacobianData<T>> meshJacobianData;


    void SetupForMesh(int meshIndex) {
        if (meshIndex >= int(meshJacobianData.size())) {
            meshJacobianData.resize(meshIndex + 1);
        }
    }
};

template <class T> RigGeometry<T>::State::State() : m(std::make_unique<Private>()) {}
template <class T> RigGeometry<T>::State::~State() = default;
template <class T> RigGeometry<T>::State::State(State&&) = default;
template <class T> typename RigGeometry<T>::State& RigGeometry<T>::State::operator=(State&&) = default;

template <class T>
const std::vector<DiffDataMatrix<T, 3, -1>>& RigGeometry<T>::State::Vertices() const
{
    return m->vertices;
}

template <class T>
const std::vector<int>& RigGeometry<T>::State::MeshIndices() const
{
    return m->meshIndices;
}

template <class T>
Eigen::Matrix<T, 4, 4> RigGeometry<T>::State::GetWorldMatrix(int jointIndex) const
{
    return m->worldMatrices[jointIndex].matrix();
}

template <class T>
struct RigGeometry<T>::Private
{
    std::vector<MeshData<T>> meshData;
    std::vector<std::vector<int>> meshIndicesForLOD;
    std::vector<std::vector<int>> jointIndicesForLOD;
    Eigen::Matrix<T, 3, -1> jointRestPose;
    std::vector<Eigen::Matrix<T, 3, 3>> jointRestOrientation;
    bool withJointScaling = false;

    JointRig2<T> jointRig2;
    std::vector<std::vector<int>> jointIndicesPerHierarchyLevel;
    std::vector<Eigen::Transform<T, 3, Eigen::Affine>> jointRig2StateBindPoses;
    std::vector<Eigen::Transform<T, 3, Eigen::Affine>> jointRig2StateInverseBindPoses;

    std::shared_ptr<epic::carbon::TaskThreadPool> taskThreadPool = epic::carbon::TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/true);
};


template <class T>
RigGeometry<T>::RigGeometry() : m(std::make_unique<Private>())
{
}

template <class T> RigGeometry<T>::~RigGeometry() = default;
template <class T> RigGeometry<T>::RigGeometry(RigGeometry&&) = default;
template <class T> RigGeometry<T>& RigGeometry<T>::operator=(RigGeometry&&) = default;

template <class T>
std::shared_ptr<RigGeometry<T>> RigGeometry<T>::Clone() const
{
    std::shared_ptr<RigGeometry> clone = std::make_shared<RigGeometry>();
    *clone->m = *m;
    return clone;
}

template <class T>
Eigen::Matrix<T, 4, 4> RigGeometry<T>::GetBindMatrix(int jointIndex) const
{
    return m->jointRig2StateBindPoses[jointIndex].matrix();
}

template <class T>
const std::vector<int>& RigGeometry<T>::JointIndicesForLOD(int lod) const
{
    return m->jointIndicesForLOD[lod];
}

template <class T>
const JointRig2<T>& RigGeometry<T>::GetJointRig() const
{
    return m->jointRig2;
}

template <class T>
const Mesh<T>& RigGeometry<T>::GetMesh(int meshIndex) const
{
    if (meshIndex < 0 || meshIndex >= int(m->meshData.size())) {
        CARBON_CRITICAL("mesh index {} out of bounds", meshIndex);
    }
    return m->meshData[meshIndex].mesh;
}

template <class T>
void RigGeometry<T>::SetMesh(int meshIndex, const Eigen::Matrix<T, 3, -1>& vertices)
{
    if (meshIndex < 0 || meshIndex >= int(m->meshData.size())) {
        CARBON_CRITICAL("mesh index {} out of bounds", meshIndex);
    }
    if (int(vertices.cols()) != m->meshData[meshIndex].mesh.NumVertices()) {
        CARBON_CRITICAL("mesh {} has a different number of vertices: {} vs {}", meshIndex, vertices.cols(), m->meshData[meshIndex].mesh.NumVertices());
    }
    m->meshData[meshIndex].mesh.SetVertices(vertices);
}


template <class T>
const std::string& RigGeometry<T>::GetMeshName(int meshIndex) const
{
    if (meshIndex < 0 || meshIndex >= int(m->meshData.size())) {
        CARBON_CRITICAL("mesh index {} out of bounds", meshIndex);
    }
    return m->meshData[meshIndex].meshName;
}

template <class T>
int RigGeometry<T>::GetMeshIndex(const std::string& meshName) const
{
    for (size_t i = 0; i < m->meshData.size(); ++i) {
        if (m->meshData[i].meshName == meshName) {
            return int(i);
        }
    }
    return -1;
}

template <class T>
int RigGeometry<T>::NumMeshes() const
{
    return int(m->meshData.size());
}


template <class T>
const std::vector<int>& RigGeometry<T>::GetMeshIndicesForLOD(int lod) const
{
    return m->meshIndicesForLOD[lod];
}

template <class T>
Mesh<T> RigGeometry<T>::ReadMesh(dna::StreamReader* reader, int meshIndex)
{
    Mesh<T> mesh;

    Eigen::Matrix<T, 3, -1> vertices(3, reader->getVertexPositionCount(std::uint16_t(meshIndex)));
    Eigen::Matrix<T, 2, -1> texcoords(2, reader->getVertexTextureCoordinateCount(std::uint16_t(meshIndex)));

    for (std::uint32_t j = 0; j < reader->getVertexPositionCount(std::uint16_t(meshIndex)); j++) {
        vertices(0, j) = reader->getVertexPosition(std::uint16_t(meshIndex), j).x;
        vertices(1, j) = reader->getVertexPosition(std::uint16_t(meshIndex), j).y;
        vertices(2, j) = reader->getVertexPosition(std::uint16_t(meshIndex), j).z;
    }
    mesh.SetVertices(vertices);

    for (std::uint32_t j = 0; j < reader->getVertexTextureCoordinateCount(std::uint16_t(meshIndex)); j++) {
        texcoords(0, j) = reader->getVertexTextureCoordinate(std::uint16_t(meshIndex), j).u;
        // texture coordinates are stored with origin in bottom left corner, but images are stored with origin in top left corner
        // and hence we flip the coordinate here
        texcoords(1, j) = T(1) -  reader->getVertexTextureCoordinate(std::uint16_t(meshIndex), j).v;
    }
    mesh.SetTexcoords(texcoords);

    const int numFaces = reader->getFaceCount(std::uint16_t(meshIndex));
    int numQuads = 0;
    int numTris = 0;
    std::map<size_t, int> numOthers;
    for (int faceIndex = 0; faceIndex < numFaces; faceIndex++) {
        rl4::ConstArrayView<std::uint32_t> faceLayoutIndices = reader->getFaceVertexLayoutIndices(std::uint16_t(meshIndex), faceIndex);
        if (faceLayoutIndices.size() == 3) {
            numTris++;
        } else if (faceLayoutIndices.size() == 4) {
            numQuads++;
        } else {
            numOthers[faceLayoutIndices.size()]++;
        }
    }
    for (const auto& [vertexCount, numFacesWithThatCount] : numOthers) {
        LOG_WARNING("mesh {} contains {} faces with {} vertices, but we only support triangles and quads", reader->getMeshName(std::uint16_t(meshIndex)).c_str(), numFacesWithThatCount, vertexCount);
    }
    rl4::ConstArrayView<std::uint32_t> vertexLayoutPositions = reader->getVertexLayoutPositionIndices(std::uint16_t(meshIndex));
    rl4::ConstArrayView<std::uint32_t> texLayoutPositons = reader->getVertexLayoutTextureCoordinateIndices(std::uint16_t(meshIndex));
    Eigen::Matrix<int, 4, -1> quads(4, numQuads);
    Eigen::Matrix<int, 3, -1> tris(3, numTris);
    Eigen::Matrix<int, 4, -1> texQuads(4, numQuads);
    Eigen::Matrix<int, 3, -1> texTris(3, numTris);

    int quadsIter = 0;
    int trisIter = 0;
    for (int faceIndex = 0; faceIndex < numFaces; faceIndex++) {
        rl4::ConstArrayView<std::uint32_t> faceLayoutIndices = reader->getFaceVertexLayoutIndices(std::uint16_t(meshIndex), faceIndex);
        if (faceLayoutIndices.size() == 3) {
            tris(0, trisIter) = vertexLayoutPositions[faceLayoutIndices[0]];
            tris(1, trisIter) = vertexLayoutPositions[faceLayoutIndices[1]];
            tris(2, trisIter) = vertexLayoutPositions[faceLayoutIndices[2]];

            texTris(0, trisIter) = texLayoutPositons[faceLayoutIndices[0]];
            texTris(1, trisIter) = texLayoutPositons[faceLayoutIndices[1]];
            texTris(2, trisIter) = texLayoutPositons[faceLayoutIndices[2]];

            trisIter++;
        } else if (faceLayoutIndices.size() == 4) {
            quads(0, quadsIter) = vertexLayoutPositions[faceLayoutIndices[0]];
            quads(1, quadsIter) = vertexLayoutPositions[faceLayoutIndices[1]];
            quads(2, quadsIter) = vertexLayoutPositions[faceLayoutIndices[2]];
            quads(3, quadsIter) = vertexLayoutPositions[faceLayoutIndices[3]];

            texQuads(0, quadsIter) = texLayoutPositons[faceLayoutIndices[0]];
            texQuads(1, quadsIter) = texLayoutPositons[faceLayoutIndices[1]];
            texQuads(2, quadsIter) = texLayoutPositons[faceLayoutIndices[2]];
            texQuads(3, quadsIter) = texLayoutPositons[faceLayoutIndices[3]];
            quadsIter++;
        }
    }
    mesh.SetTriangles(tris);
    mesh.SetQuads(quads);
    mesh.SetTexQuads(texQuads);
    mesh.SetTexTriangles(texTris);

    mesh.Validate(true);

    return mesh;
}


template <class T>
bool RigGeometry<T>::Init(dna::StreamReader* reader, bool withJointScaling)
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    m->withJointScaling = withJointScaling;

    const std::uint16_t numMeshes = reader->getMeshCount();
    m->meshData.resize(numMeshes);

    // read mesh and blendshape data
    for (std::uint16_t meshIndex = 0; meshIndex < numMeshes; meshIndex++) {

        // read mesh geometry
        m->meshData[meshIndex].mesh = ReadMesh(reader, meshIndex);
        m->meshData[meshIndex].meshName = reader->getMeshName(meshIndex).c_str();

        // read blendshape data and put into sparse matrix
        // const std::uint16_t numTotalBlendshapes = reader->getBlendShapeChannelCount();
        const std::uint16_t numBlendshapeTargets = reader->getBlendShapeTargetCount(meshIndex);
        m->meshData[meshIndex].blendshapeMatrixDense = Eigen::Matrix<T, -1, -1>::Zero(3 * m->meshData[meshIndex].mesh.NumVertices(), numBlendshapeTargets);
        m->meshData[meshIndex].blendshapeControlsToMeshBlendshapeControls.resize(numBlendshapeTargets);
        for (std::uint16_t blendShapeTargetIndex = 0; blendShapeTargetIndex < numBlendshapeTargets; blendShapeTargetIndex++) {
            const std::uint16_t channelIndex = reader->getBlendShapeChannelIndex(meshIndex, blendShapeTargetIndex);
            const int psdIndex = reader->getBlendShapeChannelInputIndices()[channelIndex];
            m->meshData[meshIndex].blendshapeControlsToMeshBlendshapeControls[blendShapeTargetIndex] = static_cast<int>(psdIndex);
            const std::uint32_t numDeltas = reader->getBlendShapeTargetDeltaCount(meshIndex, blendShapeTargetIndex);
            if (numDeltas == 0) continue;
            rl4::ConstArrayView<std::uint32_t> vertexIndices = reader->getBlendShapeTargetVertexIndices(meshIndex, blendShapeTargetIndex);
            for (int deltaIndex = 0; deltaIndex < int(numDeltas); deltaIndex++) {
                const dna::Delta delta = reader->getBlendShapeTargetDelta(meshIndex, blendShapeTargetIndex, deltaIndex);
                m->meshData[meshIndex].blendshapeMatrixDense(3 * vertexIndices[deltaIndex] + 0, blendShapeTargetIndex) = delta.x;
                m->meshData[meshIndex].blendshapeMatrixDense(3 * vertexIndices[deltaIndex] + 1, blendShapeTargetIndex) = delta.y;
                m->meshData[meshIndex].blendshapeMatrixDense(3 * vertexIndices[deltaIndex] + 2, blendShapeTargetIndex) = delta.z;
            }
            if (m->meshData[meshIndex].blendshapeMatrixDense.col(blendShapeTargetIndex).norm() == T(0)) {
                LOG_WARNING("blendshape {} ({}, psd {}) does not have any data, but {} deltas", reader->getBlendShapeChannelName(channelIndex).c_str(), channelIndex, psdIndex, numDeltas);
            }
        }
        m->meshData[meshIndex].blendshapeMatrixDenseRM = m->meshData[meshIndex].blendshapeMatrixDense;
    }

    const std::uint16_t numLODs = reader->getLODCount();

    m->meshIndicesForLOD.clear();
    m->meshIndicesForLOD.resize(numLODs);
    for (std::uint16_t lod = 0; lod < numLODs; lod++) {
        rl4::ConstArrayView<std::uint16_t> meshIndicesForLOD = reader->getMeshIndicesForLOD(lod);
        for (std::uint16_t meshIndex : meshIndicesForLOD) {
            m->meshIndicesForLOD[lod].push_back(meshIndex);
        }
    }

    // read joints data
    const std::uint16_t numJoints = reader->getJointCount();
    m->jointRig2.Clear();

    if (numJoints > 0) {
        for (std::uint16_t jointIndex = 0; jointIndex < numJoints; jointIndex++) {
            const std::string jointName = reader->getJointName(jointIndex).c_str();
            m->jointRig2.AddJoint(jointName);
        }

        // setup hierarchy
        for (std::uint16_t jointIndex = 0; jointIndex < numJoints; jointIndex++) {
            std::uint16_t parentIndex = reader->getJointParentIndex(jointIndex);
            if (parentIndex != jointIndex) {
                const std::string childJointName = reader->getJointName(jointIndex).c_str();
                const std::string parentJointName = reader->getJointName(parentIndex).c_str();
                m->jointRig2.AttachJointToParent(childJointName, parentJointName);
            }
        }

        m->jointIndicesPerHierarchyLevel = m->jointRig2.GetJointsPerHierarchyLevel();
        m->jointRig2StateBindPoses.resize(numJoints);
        m->jointRig2StateInverseBindPoses.resize(numJoints);

        // temporary state to calculate the bind poses
        State state;

        state.m->jointRig2StateLocalMatrices.resize(numJoints);
        state.m->jointRig2StateWorldMatrices.resize(numJoints);
        state.m->jointRig2StateSkinningMatrices.resize(numJoints);

        // setup bind pose
        //    a) set the local matrix of the joint
        m->jointRestPose = Eigen::Matrix<T, 3, -1>(3, numJoints);
        m->jointRestOrientation.resize(numJoints);
        for (std::uint16_t jointIndex = 0; jointIndex < numJoints; jointIndex++) {
            rl4::Vector3 t = reader->getNeutralJointTranslation(jointIndex);
            constexpr float deg2rad = float(CARBON_PI / 180.0);
            rl4::Vector3 rot = reader->getNeutralJointRotation(jointIndex) * deg2rad;
            m->jointRestPose(0, jointIndex) = t.x;
            m->jointRestPose(1, jointIndex) = t.y;
            m->jointRestPose(2, jointIndex) = t.z;
            Eigen::Matrix<T, 3, 3> R = EulerXYZ<T>(rot.x, rot.y, rot.z);
            m->jointRestOrientation[jointIndex] = R;
             const Eigen::Vector3<T> translation(t.x, t.y, t.z);
            state.m->jointRig2StateLocalMatrices[jointIndex] = DiffDataAffine<T, 3, 3>(R, translation);
        }

        //    b) get the world matrix and set as the bind pose
        for (size_t hierarchyLevel = 0; hierarchyLevel < m->jointIndicesPerHierarchyLevel.size(); ++hierarchyLevel) {
            const std::vector<int>& jointsPerLevel = m->jointIndicesPerHierarchyLevel[hierarchyLevel];
            for (int jointIndex : jointsPerLevel) {
                const int parentIndex = m->jointRig2.GetParentIndex(jointIndex);
                if (parentIndex >= 0) {
                    state.m->jointRig2StateWorldMatrices[jointIndex] = state.m->jointRig2StateWorldMatrices[parentIndex] * state.m->jointRig2StateLocalMatrices[jointIndex];
                } else {
                    state.m->jointRig2StateWorldMatrices[jointIndex] = state.m->jointRig2StateLocalMatrices[jointIndex];
                }
                m->jointRig2StateBindPoses[jointIndex] = state.m->jointRig2StateWorldMatrices[jointIndex].Matrix();
                m->jointRig2StateInverseBindPoses[jointIndex] = m->jointRig2StateBindPoses[jointIndex].inverse();
            }
        }

        // get the joints that are nedded for an lod
        m->jointIndicesForLOD.clear();
        m->jointIndicesForLOD.resize(numLODs);
        for (std::uint16_t lod = 0; lod < numLODs; lod++) {
            const rl4::ConstArrayView<std::uint16_t> jointIndices = reader->getJointIndicesForLOD(lod);
            std::vector<bool> used(m->jointRig2.NumJoints(), false);
            for (const std::uint16_t& jointIndex : jointIndices) {
                used[jointIndex] = true;
                // add parents
                int currIdx = m->jointRig2.GetParentIndex(jointIndex);
                while (currIdx >= 0) {
                    used[currIdx] = true;
                    currIdx = m->jointRig2.GetParentIndex(currIdx);
                }
            }
            for (int i = 0; i < m->jointRig2.NumJoints(); ++i) {
                if (used[i]) {
                    m->jointIndicesForLOD[lod].push_back(i);
                }
            }
        }

        // setup skinning weights
        for (std::uint16_t meshIndex = 0; meshIndex < numMeshes; meshIndex++) {
            std::vector<std::vector<std::pair<int, T>>> jointInfluences(numJoints);
            for (int vertexIndex = 0; vertexIndex < m->meshData[meshIndex].mesh.NumVertices(); vertexIndex++) {
                rl4::ConstArrayView<float> influenceWeights = reader->getSkinWeightsValues(meshIndex, vertexIndex);
                rl4::ConstArrayView<std::uint16_t> jointIndices = reader->getSkinWeightsJointIndices(meshIndex, vertexIndex);
                for (int k = 0; k < int(influenceWeights.size()); k++) {
                    jointInfluences[jointIndices[k]].push_back(std::pair<int, T>(vertexIndex, influenceWeights[k]));
                }
            }
            std::map<std::string, InfluenceWeights<T>> allInfluenceWeights;
            for (int jointIndex = 0; jointIndex < numJoints; jointIndex++) {
                InfluenceWeights<T> influenceWeights;
                const int numInfluences = int(jointInfluences[jointIndex].size());
                if (numInfluences > 0) {
                    influenceWeights.indices.resize(numInfluences);
                    influenceWeights.weights.resize(numInfluences);
                    for (int k = 0; k < numInfluences; k++) {
                        influenceWeights.indices[k] = jointInfluences[jointIndex][k].first;
                        influenceWeights.weights[k] = jointInfluences[jointIndex][k].second;
                    }
                    const std::string jointName = m->jointRig2.GetJointNames()[jointIndex];
                    allInfluenceWeights[jointName] = influenceWeights;
                }
            }
            const std::string& meshName = m->meshData[meshIndex].meshName; //std::to_string(meshIndex)
            m->jointRig2.AddInfluenceWeights(meshName, allInfluenceWeights, m->meshData[meshIndex].mesh.NumVertices());
        }

        // validate the rig
        m->jointRig2.CheckValidity();
    }

    return true;
}

template <class T, int R1, int C1, int R2, int C2>
static void DenseMatrixMatrixMultiply(DiffDataMatrix<T, R1, C2>& result, const DiffDataMatrix<T, R1, C1>& matA, const DiffDataMatrix<T, R2, C2>& matB)
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    CARBON_PRECONDITION(matA.Cols() == matB.Rows(), "for matrix multiplication the number of columns of A needs to match the number of rows of B");
    if constexpr (C1 == C2) {
        if (result.ValuePtr() == matA.ValuePtr()) {
            CARBON_CRITICAL("no aliasing allowed for dense matrix multiply");
        }
    }
    if constexpr (R1 == R2) {
        if (result.ValuePtr() == matB.ValuePtr()) {
            CARBON_CRITICAL("no aliasing allowed for dense matrix multiply");
        }
    }

    const int rows = matA.Rows();
    const int innerDim = matA.Cols();
    const int cols = matB.Cols();

    // C = A * B
    Eigen::Map<Eigen::MatrixX<T>>(result.MutableValue().data(), rows, cols).noalias() = matA.Matrix() * matB.Matrix();

    JacobianConstPtr<T> mergedJacobian;

    // dC/dx = dC/dA dA/dx + dC/dB dB/dx
    if (matB.HasJacobian() && matB.Jacobian().NonZeros() > 0) {
        PROFILING_BLOCK("jacobian b1");
        // dC/dB dB/dx
        // create dC/dB
        SparseMatrix<T> dCdB(rows * cols, matB.Rows() * matB.Cols());
        dCdB.reserve(rows * cols * innerDim);
        for (int j = 0; j < cols; j++) {
            for (int i = 0; i < rows; i++) {
                dCdB.startVec(rows * j + i);
                for (int k = 0; k < innerDim; k++) {
                    // c(i,j) += a(i,k) * b(k,j)
                    dCdB.insertBackByOuterInnerUnordered(rows * j + i, innerDim * j + k) = matA.Matrix().coeff(i,k);
                }
            }
        }
        dCdB.finalize();
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("jacobian b2");
        JacobianConstPtr<T> dCdx = matB.Jacobian().Premultiply(dCdB);
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("jacobian b3");
        mergedJacobian = dCdx;
        PROFILING_END_BLOCK;
    }

    if (matA.HasJacobian() && matA.Jacobian().NonZeros() > 0) {
        PROFILING_BLOCK("jacobian a1");
        // dC/dA dA/dx
        // create dC/dA
        SparseMatrix<T> dCdA(rows * cols, matA.Rows() * matA.Cols());
        dCdA.reserve(rows * cols * innerDim);
        for (int j = 0; j < cols; j++) {
            for (int i = 0; i < rows; i++) {
                dCdA.startVec(rows * j + i);
                for (int k = 0; k < innerDim; k++) {
                    // c(i,j) += a(i,k) * b(k,j)
                    dCdA.insertBackByOuterInnerUnordered(rows * j + i, rows * k + i) = matB.Matrix().coeff(k, j);
                }
            }
        }
        dCdA.finalize();
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("jacobian a2");
        JacobianConstPtr<T> dCdx = matA.Jacobian().Premultiply(dCdA);
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("jacobian a3");
        // add dCdx to the Jacobian matrix
        if (!mergedJacobian) {
            mergedJacobian = dCdx;
        } else {
            mergedJacobian = mergedJacobian->Add(dCdx);
        }
        PROFILING_END_BLOCK;
    }

    result.SetJacobianPtr(mergedJacobian);
}

template <class T>
void AdditionAssignment(DiffData<T>& a, const DiffData<T>& b)
{
    CARBON_PRECONDITION(a.Value().size() == b.Value().size(), "dimensions need to match for DiffData addition");

    a.MutableValue() += b.Value();

    if (b.HasJacobian()) {
        if (a.HasJacobian()) {
            // merge a and b jacobians i.e. add the two jacobians
            a.SetJacobianPtr(a.Jacobian().Add(b.JacobianPtr()));
        } else {
            a.SetJacobianPtr(b.JacobianPtr());
        }
    }
}


//! Multiplies two affine transformations A1 (A2 x + b2) + b1 = (A1 A2) x + (A1 b2 + b2)
template <class T, int R, int C, typename DISCARD = typename std::enable_if<(R > 0 && C > 0), void>::type>
void Multiply(DiffDataAffine<T, R, C> & result, const DiffDataAffine<T, R, C>& other1, const DiffDataAffine<T, R, C>& other2)
{
    DenseMatrixMatrixMultiply(result.Linear(), other1.Linear(), other2.Linear());
    DenseMatrixMatrixMultiply(result.Translation(), other1.Linear(), other2.Translation());
    AdditionAssignment(result.Translation(), other1.Translation());
}

template <class T>
void RigGeometry<T>::EvaluateJointDeltas(const DiffDataAffine<T, 3, 3>& diffRigid, const DiffData<T>& diffJoints, State& state) const
{
    if (state.m->jointDeltas.ValuePtr() == diffJoints.ValuePtr() && state.m->jointDeltas.JacobianPtr() == diffJoints.JacobianPtr() &&
        state.m->rigid.Linear().Value() == diffRigid.Linear().Value() && state.m->rigid.Linear().JacobianPtr() == diffRigid.Linear().JacobianPtr() &&
        state.m->rigid.Translation().Value() == diffRigid.Translation().Value() && state.m->rigid.Translation().JacobianPtr() == diffRigid.Translation().JacobianPtr()) {
        // joint deltas and rigid have not changed, therefore we return the previous result
        return;
    }

    // Timer timer;

    state.m->jointDeltas = diffJoints;
    state.m->rigid = diffRigid;
    Eigen::Ref<const Eigen::VectorX<T>> jointState = diffJoints.Value();

    state.m->withJacobians = true;

    const int numJoints = m->jointRig2.NumJoints();
    state.m->jointRig2StateLocalMatrices.resize(numJoints);
    state.m->jointRig2StateWorldMatrices.resize(numJoints);
    state.m->jointRig2StateSkinningMatrices.resize(numJoints);
    state.m->worldMatrices.resize(numJoints);
    state.m->skinningMatrices.resize(numJoints);

    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);
    PROFILING_BLOCK("joint deltas");
    {
        auto updateLocalMatrices = [&](int start, int end) {
            const int dofPerJoint = m->withJointScaling ? 9 : 6;
            for (int jointIndex = start; jointIndex < end; ++jointIndex) {
                const int txIdx = dofPerJoint * jointIndex + 0;
                const int tyIdx = dofPerJoint * jointIndex + 1;
                const int tzIdx = dofPerJoint * jointIndex + 2;
                const int rxIdx = dofPerJoint * jointIndex + 3;
                const int ryIdx = dofPerJoint * jointIndex + 4;
                const int rzIdx = dofPerJoint * jointIndex + 5;
                const T& dtx = jointState[txIdx];
                const T& dty = jointState[tyIdx];
                const T& dtz = jointState[tzIdx];
                const T& drx = jointState[rxIdx];
                const T& dry = jointState[ryIdx];
                const T& drz = jointState[rzIdx];

                const T tx = m->jointRestPose(0, jointIndex) + dtx;
                const T ty = m->jointRestPose(1, jointIndex) + dty;
                const T tz = m->jointRestPose(2, jointIndex) + dtz;

                JacobianConstPtr<T> rotationJacobian;
                JacobianConstPtr<T> translationJacobian;

                Eigen::Matrix<T, 3, 3> R;

                if (m->withJointScaling && jointState.segment(dofPerJoint * jointIndex + 6, 3).squaredNorm() > 0) {
                    const int sxIdx = dofPerJoint * jointIndex + 6;
                    const int syIdx = dofPerJoint * jointIndex + 7;
                    const int szIdx = dofPerJoint * jointIndex + 8;
                    const T& dsx = jointState[sxIdx];
                    const T& dsy = jointState[syIdx];
                    const T& dsz = jointState[szIdx];

                    R = m->jointRestOrientation[jointIndex] * EulerXYZAndScale(drx, dry, drz, T(1) + dsx, T(1) + dsy, T(1) + dsz);

                    if (diffJoints.HasJacobian()) {
                        // gather jacobian of drx, dry, drz, dsx, dsy, dsz, and combine with euler jacobian and scale jacobian
                        SparseMatrix<T> jacobianOfPreMultiply = JacobianOfPremultipliedMatrix<T, 3, 3, 3>(m->jointRestOrientation[jointIndex]);
                        SparseMatrix<T> eulerAndScaleJacobian = EulerXYZAndScaleJacobian<T>(drx, dry, drz, T(1) + dsx, T(1) + dsy, T(1) + dsz);
                        Eigen::Vector<int, 6> indices;
                        indices << rxIdx, ryIdx, rzIdx, sxIdx, syIdx, szIdx;
                        JacobianConstPtr<T> jacobianOfDeltas = diffJoints.Jacobian().RowGather(indices);
                        rotationJacobian = jacobianOfDeltas->Premultiply(jacobianOfPreMultiply * eulerAndScaleJacobian);
                    }
                } else {
                    R = m->jointRestOrientation[jointIndex] * EulerXYZ(drx, dry, drz);

                    if (diffJoints.HasJacobian()) {
                        // gather jacobian of drx, dry, drz and combine with euler jacobian
                        SparseMatrix<T> jacobianOfPreMultiply = JacobianOfPremultipliedMatrix<T, 3, 3, 3>(m->jointRestOrientation[jointIndex]);
                        SparseMatrix<T> eulerJacobian = EulerXYZJacobian<T>(drx, dry, drz);
                        JacobianConstPtr<T> jacobianOfDeltas = diffJoints.Jacobian().RowGather(Eigen::Vector3i(rxIdx, ryIdx, rzIdx));
                        rotationJacobian = jacobianOfDeltas->Premultiply(jacobianOfPreMultiply * eulerJacobian);
                    }
                }

                if (diffJoints.HasJacobian()) {
                    // translation jacobian is just the copy of the respective rows of the joint jacobians
                    translationJacobian = diffJoints.Jacobian().RowGather(Eigen::Vector3i(txIdx, tyIdx, tzIdx));
                }

                state.m->jointRig2StateLocalMatrices[jointIndex].Linear().MutableMatrix() = R;
                state.m->jointRig2StateLocalMatrices[jointIndex].Linear().SetJacobianPtr(rotationJacobian);
                state.m->jointRig2StateLocalMatrices[jointIndex].Translation().MutableValue() = Eigen::Vector3<T>(tx, ty, tz);
                state.m->jointRig2StateLocalMatrices[jointIndex].Translation().SetJacobianPtr(translationJacobian);
            }
        };
        const int numTasks = int(m->jointRestPose.cols());
        if (diffJoints.HasJacobian()) {
            m->taskThreadPool->AddTaskRangeAndWait(numTasks, updateLocalMatrices);
        } else {
            updateLocalMatrices(0, numTasks);
        }
    }
    // LOG_INFO("local: {}", timer.Current()); timer.Restart();
    PROFILING_END_BLOCK;

    PROFILING_BLOCK("world matrix");
    // update world and skinning matrices
    for (size_t hierarchyLevel = 0; hierarchyLevel < m->jointIndicesPerHierarchyLevel.size(); ++hierarchyLevel) {
        const std::vector<int>& jointsPerLevel = m->jointIndicesPerHierarchyLevel[hierarchyLevel];
        const int numTasks = static_cast<int>(jointsPerLevel.size());
        auto updateWorldAndSkinningMatrices = [&](int start, int end) {
            for (int taskId = start; taskId < end; ++taskId) {
                const int jointIndex = jointsPerLevel[taskId];
                const int parentIndex = m->jointRig2.GetParentIndex(jointIndex);
                if (parentIndex >= 0) {
                    Multiply(state.m->jointRig2StateWorldMatrices[jointIndex], state.m->jointRig2StateWorldMatrices[parentIndex], state.m->jointRig2StateLocalMatrices[jointIndex]);
                } else {
                    // root node
                    Multiply(state.m->jointRig2StateWorldMatrices[jointIndex], diffRigid, state.m->jointRig2StateLocalMatrices[jointIndex]);
                }
                state.m->worldMatrices[jointIndex].matrix() = state.m->jointRig2StateWorldMatrices[jointIndex].Matrix();
            }
        };
        if (diffJoints.HasJacobian()) {
            m->taskThreadPool->AddTaskRangeAndWait(numTasks, updateWorldAndSkinningMatrices);
        } else {
            updateWorldAndSkinningMatrices(0, numTasks);
        }
    }
    // LOG_INFO("world: {}", timer.Current()); timer.Restart();
    PROFILING_END_BLOCK;

    PROFILING_BLOCK("skinning matrix");
    {
        const int numTasks = static_cast<int>(state.m->jointRig2StateWorldMatrices.size());
        auto updateWorldAndSkinningMatrices = [&](int start, int end) {
            for (int taskId = start; taskId < end; ++taskId) {
                const int jointIndex = taskId;
                Multiply(state.m->jointRig2StateSkinningMatrices[jointIndex], state.m->jointRig2StateWorldMatrices[jointIndex], DiffDataAffine<T, 3, 3>(m->jointRig2StateInverseBindPoses[jointIndex].matrix()));
                state.m->skinningMatrices[jointIndex].matrix() = state.m->jointRig2StateSkinningMatrices[jointIndex].Matrix();
            }
        };
        if (diffJoints.HasJacobian()) {
            m->taskThreadPool->AddTaskRangeAndWait(numTasks, updateWorldAndSkinningMatrices);
        } else {
            updateWorldAndSkinningMatrices(0, numTasks);
        }
    }

    // LOG_INFO("skinning: {}", timer.Current()); timer.Restart();
    PROFILING_END_BLOCK;
}

template <class T>
void RigGeometry<T>::EvaluateJointDeltasWithoutJacobians(const DiffDataAffine<T, 3, 3>& diffRigid, const DiffData<T>& diffJoints, State& state) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    state.m->jointDeltas = diffJoints;
    state.m->rigid = diffRigid;
    Eigen::Ref<const Eigen::VectorX<T>> jointState = diffJoints.Value();

    const int numJoints = m->jointRig2.NumJoints();
    state.m->withJacobians = false;
    state.m->localMatrices.resize(numJoints);
    state.m->worldMatrices.resize(numJoints);
    state.m->skinningMatrices.resize(numJoints);

    // calculate local matrices
    const int dofPerJoint = m->withJointScaling ? 9 : 6;
    for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
        const T& drx = jointState[dofPerJoint * jointIndex + 3];
        const T& dry = jointState[dofPerJoint * jointIndex + 4];
        const T& drz = jointState[dofPerJoint * jointIndex + 5];

        Eigen::Matrix<T, 3, 3> R;

        if (m->withJointScaling && jointState.segment(dofPerJoint * jointIndex + 6, 3).squaredNorm() > 0) {
            const T& dsx = jointState[dofPerJoint * jointIndex + 6];
            const T& dsy = jointState[dofPerJoint * jointIndex + 7];
            const T& dsz = jointState[dofPerJoint * jointIndex + 8];

            R = m->jointRestOrientation[jointIndex] * EulerXYZAndScale(drx, dry, drz, T(1) + dsx, T(1) + dsy, T(1) + dsz);
        } else {
            R = m->jointRestOrientation[jointIndex] * EulerXYZ(drx, dry, drz);
        }

        state.m->localMatrices[jointIndex].linear() = R;
        state.m->localMatrices[jointIndex].translation() = jointState.segment(dofPerJoint * jointIndex + 0, 3) + m->jointRestPose.col(jointIndex);
    }

    // update world matrices
    for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
        const int parentIndex = m->jointRig2.GetParentIndex(jointIndex);
        if (parentIndex >= 0) {
            state.m->worldMatrices[jointIndex] = state.m->worldMatrices[parentIndex] * state.m->localMatrices[jointIndex];
        } else {
            // root node
            state.m->worldMatrices[jointIndex] = Eigen::Transform<T, 3, Eigen::Affine>(diffRigid.Matrix()) * state.m->localMatrices[jointIndex];
        }
    }

    // update skinning matrices
    for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
        state.m->skinningMatrices[jointIndex] = state.m->worldMatrices[jointIndex] * m->jointRig2StateInverseBindPoses[jointIndex];
    }
}

template <class T, int R, int C>
Eigen::Ref<Eigen::Vector<T, -1>> Flatten(Eigen::Matrix<T, R, C>& matrix)
{
    return Eigen::Map<Eigen::Vector<T, -1>>(matrix.data(), matrix.rows() * matrix.cols());
}

template <class T, int R, int C>
Eigen::Ref<const Eigen::Vector<T, -1>> Flatten(const Eigen::Matrix<T, R, C>& matrix)
{
    return Eigen::Map<const Eigen::Vector<T, -1>>(matrix.data(), matrix.rows() * matrix.cols());
}


template <class T>
void RigGeometry<T>::EvaluateBlendshapes(const DiffData<T>& diffPsd, int lod, int meshIndex, State& state) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    if (lod < 0 || lod >= int(m->meshIndicesForLOD.size())) {
        CARBON_CRITICAL("RigGeometry does not contain LOD {}", lod);
    }
    bool lodHasMeshIndex = false;
    for (int j = 0; j < int(m->meshIndicesForLOD[lod].size()); j++) {
        lodHasMeshIndex = lodHasMeshIndex || (m->meshIndicesForLOD[lod][j] == meshIndex);
    }
    if (!lodHasMeshIndex) {
        CARBON_CRITICAL("RigGeometry: LOD {} does not contain mesh index {}", lod, meshIndex);
    }

    state.m->SetupForMesh(meshIndex);

    // copy neutral
    state.m->meshJacobianData[meshIndex].blendshapeVertices = m->meshData[meshIndex].mesh.Vertices();
    state.m->meshJacobianData[meshIndex].isBlendshapeJacobianValid = false;
    auto blendshapeVerticesFlattened = Flatten(state.m->meshJacobianData[meshIndex].blendshapeVertices);

    // no blendshapes, then return neutral
    if (m->meshData[meshIndex].blendshapeControlsToMeshBlendshapeControls.size() == 0) {
        return;
    }

    PROFILING_BLOCK("blendshape selection");
    // get the blendshape activations for this mesh
    const DiffData<T> diffMeshBlendshapes = GatherFunction<T>::Gather(diffPsd, m->meshData[meshIndex].blendshapeControlsToMeshBlendshapeControls);
    PROFILING_END_BLOCK;

    PROFILING_BLOCK("blendshape evaluation");
    // evaluate blendshapes
#ifdef EIGEN_USE_BLAS
    blendshapeVerticesFlattened.noalias() += m->meshData[meshIndex].blendshapeMatrixDenseRM * diffMeshBlendshapes.Value();
#else
    // if we don't use MKL, then for large matrices (head) we parallelize the matrix vector product
    const int numVertices = int(state.m->meshJacobianData[meshIndex].blendshapeVertices.cols());
    if (numVertices > 1000) {
        auto parallelMatrixMultiply = [&](int start, int end) {
            blendshapeVerticesFlattened.segment(start, end - start).noalias() += m->meshData[meshIndex].blendshapeMatrixDenseRM.block(start, 0, end - start, diffMeshBlendshapes.Size()) * diffMeshBlendshapes.Value();
        };
        m->taskThreadPool->AddTaskRangeAndWait(3 * numVertices, parallelMatrixMultiply);
    } else {
        blendshapeVerticesFlattened.noalias() += m->meshData[meshIndex].blendshapeMatrixDenseRM * diffMeshBlendshapes.Value();
    }
#endif
    // LOG_INFO("time for blendshape evaluation {}: {}", m->meshData[meshIndex].meshName, timer.Current()); timer.Restart();
    PROFILING_END_BLOCK;

    if (diffMeshBlendshapes.HasJacobian()) {
        PROFILING_BLOCK("blendshape jacobian");
        SparseMatrix<T> diffBlendshapesSparseMatrixTransposed = diffMeshBlendshapes.Jacobian().AsSparseMatrix()->transpose();
        state.m->meshJacobianData[meshIndex].blendshapeJacobianRM.resize(m->meshData[meshIndex].blendshapeMatrixDense.rows(), diffMeshBlendshapes.Jacobian().Cols());

        auto calculate_dVertex_dBlendshapes_rm = [&](int start, int end) {
            for (int r = start; r < end; ++r) {
                for (int ctrl = 0; ctrl < int(diffBlendshapesSparseMatrixTransposed.rows()); ++ctrl) {
                    T acc = 0;
                    for (typename SparseMatrix<T>::InnerIterator it(diffBlendshapesSparseMatrixTransposed, ctrl); it; ++it)
                    {
                        acc += it.value() * m->meshData[meshIndex].blendshapeMatrixDenseRM(r, it.col());
                    }
                    state.m->meshJacobianData[meshIndex].blendshapeJacobianRM(r, ctrl) = acc;
                }
            }
        };
        m->taskThreadPool->AddTaskRangeAndWait(int(state.m->meshJacobianData[meshIndex].blendshapeVertices.size()), calculate_dVertex_dBlendshapes_rm);
        // calculate_dVertex_dBlendshapes_rm(0, int(state.m->meshJacobianData[meshIndex].blendshapeVertices.size()));

        state.m->meshJacobianData[meshIndex].isBlendshapeJacobianValid = true;
        PROFILING_END_BLOCK;
    }
}

template <class T>
DiffDataMatrix<T, 3, -1> CreateDiffDataMatrix(const Eigen::Matrix<T, 3, -1>& matrix, const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>& denseJacobian)
{
    if (denseJacobian.size() > 0) {
        return DiffDataMatrix<T, 3, -1>(matrix, std::make_shared<SparseJacobian<T>>(std::make_shared<SparseMatrix<T>>(denseJacobian.sparseView())));
    } else {
        return DiffDataMatrix<T, 3, -1>(matrix);
    }
}

template <class T>
typename RigGeometry<T>::State& RigGeometry<T>::EvaluateRigGeometry(const DiffDataAffine<T, 3, 3>& diffRigid, const DiffData<T>& diffJoints, const DiffData<T>& diffPsd, int lod, const std::vector<int>& meshIndices, State& state) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    const bool requiresJacobians = diffRigid.HasJacobian() || diffJoints.HasJacobian() || diffPsd.HasJacobian();

    const bool hasRigid = diffRigid.HasJacobian() || ((diffRigid.Matrix() - Eigen::Matrix<T, 4, 4>::Identity()).norm() > T(1e-10));

    // Timer timer;

    PROFILING_BLOCK("Evaluate - EvaluateBlendshapes()");
    for (const int meshIndex : meshIndices) {
        if (meshIndex >= 0) {
            EvaluateBlendshapes(diffPsd, lod, meshIndex, state);
            // LOG_INFO("time for blendshapes for mesh {}: {} (with jacobian: {})", meshIndex, timer.Current(), requiresJacobians ? "true" : "false"); timer.Restart();
        }
    }
    PROFILING_END_BLOCK;

    std::vector<DiffDataMatrix<T, 3, -1>> deformedVertices;
    DiffDataMatrix<T, 3, -1> emptyData(3, 0, DiffData<T>(Vector<T>()));

    if (m->jointRig2.NumJoints() > 0) {
        PROFILING_BLOCK("Evaluate - EvaluateJointDeltas()");
        if (requiresJacobians) EvaluateJointDeltas(diffRigid, diffJoints, state);
        else EvaluateJointDeltasWithoutJacobians(diffRigid, diffJoints, state);
        PROFILING_END_BLOCK;
    } else {
        if (diffJoints.Size() > 0) {
            LOG_ERROR("RigGeometry does not contain joints, but RigGeometry is called with deltas on joints");
        }
    }

    PROFILING_BLOCK("Evaluate - EvaluateGeometryDense()");
    for (size_t i = 0; i < meshIndices.size(); ++i) {
        const int meshIndex = meshIndices[i];
        if (meshIndex >= 0) {
            if (m->jointRig2.HasVertexInfluenceWeights(m->meshData[meshIndex].meshName)) {
                if (requiresJacobians) EvaluateSkinningWithJacobians(meshIndex, state);
                else EvaluateSkinningWithoutJacobians(meshIndex, state);
                if (state.m->meshJacobianData[meshIndices[i]].isFinalJacobianValid) {
                    deformedVertices.emplace_back(CreateDiffDataMatrix(state.m->meshJacobianData[meshIndex].finalVertices, state.m->meshJacobianData[meshIndex].finalJacobianRM));
                } else {
                    deformedVertices.emplace_back(DiffDataMatrix<T, 3, -1>(state.m->meshJacobianData[meshIndex].finalVertices));
                }
            } else {
                // joints do not influence geometry, just use rigid
                if (state.m->meshJacobianData[meshIndices[i]].isBlendshapeJacobianValid) {
                    DiffDataMatrix<T, 3, -1> diffDataMatrix = CreateDiffDataMatrix(state.m->meshJacobianData[meshIndex].blendshapeVertices, state.m->meshJacobianData[meshIndex].blendshapeJacobianRM);
                    deformedVertices.emplace_back(diffDataMatrix);
                } else {
                    deformedVertices.emplace_back(DiffDataMatrix<T, 3, -1>(state.m->meshJacobianData[meshIndex].blendshapeVertices));
                }
                if (hasRigid) {
                    // TODO: this is particularly slow
                    deformedVertices[i] = diffRigid.Transform(deformedVertices[i]);
                }
            }
        } else {
            deformedVertices.emplace_back(emptyData);
        }
    }
    PROFILING_END_BLOCK;

    state.m->vertices = std::move(deformedVertices);
    state.m->meshIndices = meshIndices;

    return state;
}

template <class T>
typename RigGeometry<T>::State RigGeometry<T>::EvaluateRigGeometry(const DiffDataAffine<T, 3, 3>& rigid, const DiffData<T>& joints, const DiffData<T>& blendshapes, int lod, const std::vector<int>& meshIndices) const
{
    State state;
    EvaluateRigGeometry(rigid, joints, blendshapes, lod, meshIndices, state);
    return state;
}

template <class T>
void RigGeometry<T>::EvaluateSkinningWithJacobians(int meshIndex, State& state) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_RED);

    JacobianData<T>& stateData = state.m->meshJacobianData[meshIndex];

    // rest vertices are the vertices after blendshape evaluation
    const Eigen::Matrix<T, 3, -1>& restVertices = stateData.blendshapeVertices;
    Eigen::Matrix<T, 3, -1>& deformedVertices = stateData.finalVertices;
    deformedVertices.resize(3, restVertices.cols());

    const SparseMatrix<T>& vertexInfluenceWeights = m->jointRig2.GetVertexInfluenceWeights(m->meshData[meshIndex].meshName);
    const int numVertices = int(vertexInfluenceWeights.outerSize());

    if (int(vertexInfluenceWeights.outerSize()) != int(restVertices.cols())) {
        CARBON_CRITICAL("all vertices need to be influenced by a node");
    }

    // get column size for jacobian
    int maxCols = -1;
    for (int jointIndex = 0; jointIndex <  state.m->jointRig2StateSkinningMatrices.size(); ++jointIndex) {
        const DiffDataAffine<T, 3, 3>& aff = state.m->jointRig2StateSkinningMatrices[jointIndex];
        if (aff.Linear().HasJacobian()) {
            maxCols = std::max<int>(int(aff.Linear().Jacobian().Cols()), maxCols);
        }
        if (aff.Translation().HasJacobian()) {
            maxCols = std::max<int>(int(aff.Translation().Jacobian().Cols()), maxCols);
        }
    }

    if (stateData.isBlendshapeJacobianValid) {
        maxCols = std::max<int>(int(stateData.blendshapeJacobianRM.cols()), maxCols);
    }

    Eigen::Matrix<T, -1, -1, Eigen::RowMajor>& denseJacobian = stateData.finalJacobianRM;

    PROFILING_BLOCK("part 1");
    denseJacobian.resize(3 * numVertices, maxCols);
    denseJacobian.setZero();

    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> jointTranslationJacobian = Eigen::Matrix<T, -1, -1, Eigen::RowMajor>::Zero(3 * state.m->jointRig2StateSkinningMatrices.size(), std::max<int>(0, maxCols));
    Eigen::Matrix<T, -1, -1, Eigen::RowMajor> jointRotationJacobian = Eigen::Matrix<T, -1, -1, Eigen::RowMajor>::Zero(9 * state.m->jointRig2StateSkinningMatrices.size(), std::max<int>(0, maxCols));
    for (int i = 0; i < state.m->jointRig2StateSkinningMatrices.size(); ++i) {
        const DiffDataAffine<T, 3, 3>& aff = state.m->jointRig2StateSkinningMatrices[i];
        if (aff.Linear().HasJacobian()) {
            jointRotationJacobian.block(9 * i, 0, 9, aff.Linear().Jacobian().Cols()) = *(aff.Linear().Jacobian().AsSparseMatrix());
        }
        if (aff.Translation().HasJacobian()) {
            jointTranslationJacobian.block(3 * i, 0, 3, aff.Translation().Jacobian().Cols()) = *(aff.Translation().Jacobian().AsSparseMatrix());
        }
    }
    PROFILING_END_BLOCK;

    PROFILING_BLOCK("part 2");
    auto evaluateVertexSkinning = [&](int start, int end) {
        for (int vID = start; vID < end; ++vID) {
            Eigen::Vector3<T> result(0,0,0);
            for (typename SparseMatrix<T>::InnerIterator it(vertexInfluenceWeights, vID); it; ++it)
            {
                const int64_t& jointIndex = it.col();
                const T& weight = it.value();
                result += weight * (state.m->skinningMatrices[jointIndex] * restVertices.col(vID));

                const DiffDataMatrix<T, 3, 3>& linear = state.m->jointRig2StateSkinningMatrices[jointIndex].Linear();
                const DiffDataMatrix<T, 3, 1>& t = state.m->jointRig2StateSkinningMatrices[jointIndex].Translation();
                // add to jacobian
                if (linear.HasJacobian()) {
                    for (int j = 0; j < 3; j++) {
                        denseJacobian.block(3 * vID, 0, 3, jointRotationJacobian.cols()) += (weight * restVertices(j, vID)) * jointRotationJacobian.block(9 * jointIndex + 3 * j, 0, 3, jointRotationJacobian.cols());
                    }
                }
                if (t.HasJacobian()) {
                    denseJacobian.block(3 * vID, 0, 3, jointTranslationJacobian.cols()) += weight * jointTranslationJacobian.block(3 * jointIndex, 0, 3, jointTranslationJacobian.cols());
                }

                if (stateData.isBlendshapeJacobianValid) {
                    denseJacobian.block(3 * vID, 0, 3, stateData.blendshapeJacobianRM.cols()) += (weight * linear.Matrix()) * stateData.blendshapeJacobianRM.block(3 * vID, 0, 3, stateData.blendshapeJacobianRM.cols());
                }
            }
            deformedVertices.col(vID) = result;
        }
    };

    m->taskThreadPool->AddTaskRangeAndWait(int(vertexInfluenceWeights.outerSize()), evaluateVertexSkinning);

    stateData.isFinalJacobianValid = (maxCols > 0);

    // LOG_INFO("time for skinning: {}", timer.Current());
    PROFILING_END_BLOCK;
}

template <class T>
void RigGeometry<T>::EvaluateSkinningWithoutJacobians(int meshIndex, State& state) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_RED);

    JacobianData<T>& stateData = state.m->meshJacobianData[meshIndex];

    const Eigen::Matrix<T, 3, -1>& restVertices = stateData.blendshapeVertices;
    Eigen::Matrix<T, 3, -1>& deformedVertices = stateData.finalVertices;
    deformedVertices.resize(3, restVertices.cols());

    const SparseMatrix<T>& vertexInfluenceWeights = m->jointRig2.GetVertexInfluenceWeights(m->meshData[meshIndex].meshName);
    const int numVertices = int(vertexInfluenceWeights.outerSize());

    for (int vID = 0; vID < numVertices; ++vID) {
        Eigen::Vector3<T> result(0,0,0);
        for (typename SparseMatrix<T>::InnerIterator it(vertexInfluenceWeights, vID); it; ++it)
        {
            result += it.value() * (state.m->skinningMatrices[it.col()] * restVertices.col(vID));
        }
        deformedVertices.col(vID) = result;
    }

    stateData.isFinalJacobianValid = false;
}

template <class T>
void RigGeometry<T>::MakeBlendshapeOnly(const RigLogic<T>& rigLogic)
{
    const int numRawControls = rigLogic.NumRawControls();
    const int totalControls = rigLogic.NumTotalControls();

    // do not convert eye geometry to blendshapes
    std::set<int> meshesToExclude;
    for (int lod = 0; lod < rigLogic.NumLODs(); ++lod) {
        meshesToExclude.insert(EyeLeftMeshIndex(lod));
        meshesToExclude.insert(EyeRightMeshIndex(lod));
    }

    Eigen::Vector<T, -1> rawControls = Eigen::Vector<T, -1>::Zero(numRawControls);

    // evaluate each raw control and create the blendshape
    std::vector<std::vector<Eigen::Matrix<T, 3, -1>>> allExpressions(m->meshData.size());
    for (auto& expressions : allExpressions) {
        expressions.resize(totalControls);
    }

    State state;

    // get all expressions
    const std::vector<std::tuple<int, int, Eigen::VectorX<T>>> allExpressionPSDs = rigLogic.GetAllExpressions();
    if (int(allExpressionPSDs.size()) != totalControls) {
        CARBON_CRITICAL("number of expressions should match the total number of controls (raw + psd)");
    }

    Timer timer;
    // evaluate vertices for all expressions
    for (const auto& [numAffected, psdIndex, psdControls] : allExpressionPSDs) {
        const DiffData<T> psdValues(psdControls);

        for (int lod = 0; lod < rigLogic.NumLODs(); ++lod) {
            const DiffData<T> diffJoints = rigLogic.EvaluateJoints(psdValues, lod);
            EvaluateRigGeometry(DiffDataAffine<T, 3, 3>(), diffJoints, psdValues, lod, GetMeshIndicesForLOD(lod), state);
            for (size_t j = 0; j < GetMeshIndicesForLOD(lod).size(); ++j) {
                allExpressions[GetMeshIndicesForLOD(lod)[j]][psdIndex] = state.Vertices()[j].Matrix();
            }
        }
    }
    LOG_INFO("time to evalute expressions: {}", timer.Current()); timer.Restart();

    // create blendshapes
    std::vector<MeshData<T>> newMeshData = m->meshData;
    for (int meshIndex = 0; meshIndex < NumMeshes(); ++meshIndex) {
        if (meshesToExclude.find(meshIndex) != meshesToExclude.end()) continue;

        MeshData<T>& meshData = newMeshData[meshIndex];
        meshData.blendshapeMatrixDense = Eigen::Matrix<T, -1, -1>::Zero(meshData.blendshapeMatrixDense.rows(), totalControls);
        meshData.blendshapeControlsToMeshBlendshapeControls.resize(totalControls);
    }

    for (const auto& [numAffected, psdControlIndex, psdControls] : allExpressionPSDs) {

        for (int meshIndex = 0; meshIndex < NumMeshes(); ++meshIndex) {
            if (meshesToExclude.find(meshIndex) != meshesToExclude.end()) continue;

            Eigen::Matrix<T, 3, -1> existingExpression = m->meshData[meshIndex].mesh.Vertices();
            for (int psdIndex = 0; psdIndex < int(psdControls.size()); ++psdIndex) {
                if (psdControls[psdIndex] > 0 && psdIndex != psdControlIndex) {
                    existingExpression += psdControls[psdIndex] * newMeshData[meshIndex].blendshapeMatrixDense.col(psdIndex);
                }
            }
            Eigen::Map<Eigen::Matrix<T, 3, -1>>(newMeshData[meshIndex].blendshapeMatrixDense.col(psdControlIndex).data(), 3, newMeshData[meshIndex].mesh.NumVertices()) = allExpressions[meshIndex][psdControlIndex] - existingExpression;
            newMeshData[meshIndex].blendshapeControlsToMeshBlendshapeControls[psdControlIndex] = psdControlIndex;
        }
    }
    LOG_INFO("time to create blendshapes: {}", timer.Current()); timer.Restart();

    for (int meshIndex = 0; meshIndex < NumMeshes(); ++meshIndex) {
        if (meshesToExclude.find(meshIndex) != meshesToExclude.end()) continue;

        // remove geometry from joints
        m->jointRig2.RemoveVertexInfluenceWeights(newMeshData[meshIndex].meshName);

        int currControl = 0;
        const T eps = T(0.05);
        for (int k = 0; k < totalControls; ++k) {
            const T blendshapeLength = newMeshData[meshIndex].blendshapeMatrixDense.col(k).norm();
            if (blendshapeLength > eps) {
                newMeshData[meshIndex].blendshapeMatrixDense.col(currControl) = newMeshData[meshIndex].blendshapeMatrixDense.col(k);
                newMeshData[meshIndex].blendshapeControlsToMeshBlendshapeControls[currControl] = k;
                currControl++;
            }
        }
        newMeshData[meshIndex].blendshapeMatrixDense.conservativeResize(newMeshData[meshIndex].blendshapeMatrixDense.rows(), currControl);
        newMeshData[meshIndex].blendshapeMatrixDenseRM = newMeshData[meshIndex].blendshapeMatrixDense;
        newMeshData[meshIndex].blendshapeControlsToMeshBlendshapeControls.conservativeResize(currControl);
        LOG_INFO("{} blendshapes for mesh {}", currControl, newMeshData[meshIndex].meshName);
    }
    LOG_INFO("time to reduce blendshapes: {}", timer.Current()); timer.Restart();

    m->meshData = newMeshData;

    // // uncomment below to verify blendshapes are correct
    // const DiffData<T> noJoints = DiffData<T>(Vector<T>());
    // const T errorEps = T(1e-3);

    // for (int k = 0; k < numRawControls; ++k) {
    //     const int totalControl = k;
    //     rawControls = Eigen::Vector<T, -1>::Zero(numRawControls);
    //     rawControls[k] = T(1);
    //     const DiffData<T> psdValues = rigLogic.EvaluatePSD(DiffData<T>(rawControls));
    //     for (int lod = 0; lod < rigLogic.NumLODs(); ++lod) {
    //         EvaluateRigGeometry(DiffDataAffine<T, 3, 3>(), noJoints, psdValues, lod, GetMeshIndicesForLOD(lod), state);
    //         for (size_t j = 0; j < GetMeshIndicesForLOD(lod).size(); ++j) {
    //             const int meshIndex = GetMeshIndicesForLOD(lod)[j];
    //             const T error = (allExpressions[meshIndex][totalControl] - state.Vertices()[j].Matrix()).norm();
    //             if (error > errorEps) {
    //                 LOG_ERROR("expression error for raw control {}: {}", totalControl, error);
    //             }
    //         }
    //     }
    // }
    // LOG_INFO("time to verify base expressions: {}", timer.Current()); timer.Restart();

    // const SparseMatrix<T> psdToRawMap = rigLogic.PsdToRawMap();
    // for (int k = 0; k < int(psdToRawMap.outerSize()); ++k) {
    //     const int totalControl = k + numRawControls;
    //     rawControls = Eigen::Vector<T, -1>::Zero(numRawControls);
    //     for (typename SparseMatrix<T>::InnerIterator it(psdToRawMap, k); it; ++it)
    //     {
    //         rawControls[it.col()] = T(1) / it.value();
    //     }
    //     const DiffData<T> psdValues = rigLogic.EvaluatePSD(DiffData<T>(rawControls));
    //     for (int lod = 0; lod < rigLogic.NumLODs(); ++lod) {
    //         EvaluateRigGeometry(DiffDataAffine<T, 3, 3>(), noJoints, psdValues, lod, GetMeshIndicesForLOD(lod), state);
    //         for (size_t j = 0; j < GetMeshIndicesForLOD(lod).size(); ++j) {
    //             const int meshIndex = GetMeshIndicesForLOD(lod)[j];
    //             const DiffDataMatrix<T, 3, -1> vertices = state.Vertices()[j];
    //             const T error = (allExpressions[meshIndex][totalControl] - vertices.Matrix()).norm();
    //             if (error > errorEps) {
    //                 LOG_ERROR("expression error for corrective control {}: {}", k, error);
    //             }
    //         }
    //     }
    // }
    // LOG_INFO("time to verify correctives: {}", timer.Current()); timer.Restart();
}

template <class T>
void RigGeometry<T>::RemoveUnusedJoints(RigLogic<T>& rigLogic)
{
    const int numJointsBefore = m->jointRig2.NumJoints();
    const std::vector<int> newToOldJointMapping = m->jointRig2.RemoveUnusedJoints();
    const int numJointsAfter = int(newToOldJointMapping.size());
    std::vector<int> oldToNew(numJointsBefore, -1);
    for (int newIdx = 0; newIdx < int(newToOldJointMapping.size()); ++newIdx) {
        const int oldIdx = newToOldJointMapping[newIdx];
        oldToNew[oldIdx] = newIdx;
    }

    std::vector<std::vector<int>> newJointIndicesForLOD;
    for (const auto& jointIndices : m->jointIndicesForLOD) {
        std::vector<int> newJointIndices;
        for (const auto& jointIdx : jointIndices) {
            if (oldToNew[jointIdx] >= 0) {
                newJointIndices.push_back(oldToNew[jointIdx]);
            }
        }
        newJointIndicesForLOD.push_back(newJointIndices);
    }
    m->jointIndicesForLOD = newJointIndicesForLOD;

    Eigen::Matrix<T, 3, -1> jointRestPose(3, numJointsAfter);
    std::vector<Eigen::Matrix<T, 3, 3>> jointRestOrientation(numJointsAfter);
    std::vector<Eigen::Transform<T, 3, Eigen::Affine>> jointRig2StateBindPoses(numJointsAfter);
    std::vector<Eigen::Transform<T, 3, Eigen::Affine>> jointRig2StateInverseBindPoses(numJointsAfter);
    for (int newIdx = 0; newIdx < numJointsAfter; ++newIdx) {
        const int oldIdx = newToOldJointMapping[newIdx];
        jointRestPose.col(newIdx) = m->jointRestPose.col(oldIdx);
        jointRestOrientation[newIdx] = m->jointRestOrientation[oldIdx];
        jointRig2StateBindPoses[newIdx] = m->jointRig2StateBindPoses[oldIdx];
        jointRig2StateInverseBindPoses[newIdx] = m->jointRig2StateInverseBindPoses[oldIdx];
    }
    m->jointRestPose = jointRestPose;
    m->jointRestOrientation = jointRestOrientation;
    m->jointRig2StateBindPoses = jointRig2StateBindPoses;
    m->jointRig2StateInverseBindPoses = jointRig2StateInverseBindPoses;

    m->jointIndicesPerHierarchyLevel = m->jointRig2.GetJointsPerHierarchyLevel();

    // update joints for riglogic
    rigLogic.RemoveJoints(newToOldJointMapping);
}

template <class T>
void RigGeometry<T>::ReduceToLOD0Only()
{
    m->meshIndicesForLOD.resize(1);
    m->jointIndicesForLOD.resize(1);

    std::vector<MeshData<T>> newMeshData;
    for (size_t j = 0; j < m->meshIndicesForLOD.front().size(); ++j) {
        const int meshIndex = m->meshIndicesForLOD.front()[j];
        newMeshData.push_back(m->meshData[meshIndex]);
        m->meshIndicesForLOD.front()[j] = static_cast<int>(j);
    }
    m->meshData = newMeshData;
}

template <class T>
void RigGeometry<T>::Resample(int meshIndex, const std::vector<int>& newToOldMap)
{
    if (meshIndex < 0 || meshIndex >= int(m->meshData.size())) {
        CARBON_CRITICAL("mesh index {} out of bounds", meshIndex);
    }
    m->jointRig2.Resample(m->meshData[meshIndex].meshName, newToOldMap);
    m->meshData[meshIndex].mesh.Resample(newToOldMap);
    const int numBlendshapes = int(m->meshData[meshIndex].blendshapeMatrixDense.cols());
    Eigen::Matrix<T, -1, -1> blendshapeMatrix(newToOldMap.size() * 3, numBlendshapes);
    for (int i = 0; i < int(newToOldMap.size()); ++i) {
        blendshapeMatrix.block(3 * i, 0, 3, numBlendshapes) = m->meshData[meshIndex].blendshapeMatrixDense.block(3 * newToOldMap[i], 0, 3, numBlendshapes);
    }
    m->meshData[meshIndex].blendshapeMatrixDense = blendshapeMatrix;
    m->meshData[meshIndex].blendshapeMatrixDenseRM = m->meshData[meshIndex].blendshapeMatrixDense;
}

template <class T>
int RigGeometry<T>::HeadMeshIndex(int lod) const
{
    const std::string name = "head_lod" + std::to_string(lod) + "_mesh";
    return GetMeshIndex(name);
}

template <class T>
int RigGeometry<T>::TeethMeshIndex(int lod) const
{
    const std::string name = "teeth_lod" + std::to_string(lod) + "_mesh";
    return GetMeshIndex(name);
}

template <class T>
int RigGeometry<T>::EyeLeftMeshIndex(int lod) const
{
    const std::string name = "eyeLeft_lod" + std::to_string(lod) + "_mesh";
    return GetMeshIndex(name);
}

template <class T>
int RigGeometry<T>::EyeRightMeshIndex(int lod) const
{
    const std::string name = "eyeRight_lod" + std::to_string(lod) + "_mesh";
    return GetMeshIndex(name);
}

// explicitly instantiate the rig geometry classes
template class RigGeometry<float>;
template class RigGeometry<double>;

} // namespace epic::nls
