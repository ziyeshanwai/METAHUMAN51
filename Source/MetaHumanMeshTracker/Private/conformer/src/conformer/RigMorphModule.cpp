// Copyright Epic Games, Inc. All Rights Reserved.

#include <conformer/RigMorphModule.h>

#include <carbon/geometry/AABBTree.h>
#include <nls/geometry/BarycentricCoordinates.h>
#include <nls/geometry/EulerAngles.h>
#include <nls/rendering/Rasterizer.h>
#include <nls/rig/RigGeometry.h>
#include <nrr/GridDeformation.h>
#include <pma/TypeDefs.h>

#include <iostream>

const int eyeLeftId = 3;
const int eyeRightId = 4;
const int teethId = 1;
const int headId = 0;
const int salivaId = 2;

// mesh with triangles expected
Eigen::Matrix3Xf UpdateLowerLODVerticesRaycasting(const Mesh<float>& lod0Asset, const Mesh<float>& asset) {
    int lod0UvCount = static_cast<int>(lod0Asset.Texcoords().cols());
    int assetVtxCount = asset.NumVertices();

    Eigen::Matrix3Xf outputDeltas = Eigen::Matrix3Xf::Zero(3, assetVtxCount);
    Eigen::Matrix3Xf texcoords3d = Eigen::Matrix3Xf::Zero(3, lod0UvCount);
    for (int i = 0; i < lod0UvCount; ++i) {
        texcoords3d(0, i) = lod0Asset.Texcoords()(0, i);
        texcoords3d(1, i) = lod0Asset.Texcoords()(1, i);
    }

    epic::carbon::AABBTree<float> aabbTree(texcoords3d.transpose(), lod0Asset.TexTriangles().transpose());

    // Find intersection for each asset vertex
    for (int face = 0; face < asset.NumTriangles(); ++face) {
        for (int vtx = 0; vtx < 3; ++vtx) {
            Eigen::Vector2f uv = asset.Texcoords().col(asset.TexTriangles()(vtx, face));
            Eigen::Vector3f query = Eigen::Vector3f(uv[0], uv[1], 0.0f);

            const auto [triangleIndex, closestBarycentric, squaredDistance] = aabbTree.getClosestPoint(
                query.transpose(),
                std::numeric_limits<float>::max());
            if (triangleIndex == -1) {
                outputDeltas.col(asset.Triangles()(vtx, face)) = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
                continue;
            }

            Eigen::Matrix3Xf vertices = Eigen::Matrix3Xf(3, 3);
            epic::nls::BarycentricCoordinates<float> bcOut(Eigen::Vector3i(0, 1, 2), closestBarycentric);

            vertices.col(0) = lod0Asset.Vertices().col(lod0Asset.Triangles()(0, triangleIndex));
            vertices.col(1) = lod0Asset.Vertices().col(lod0Asset.Triangles()(1, triangleIndex));
            vertices.col(2) = lod0Asset.Vertices().col(lod0Asset.Triangles()(2, triangleIndex));

            Eigen::Vector3f newVertexPosition = bcOut.Evaluate<3>(vertices);

            outputDeltas.col(asset.Triangles()(vtx,
                                               face)) = newVertexPosition - asset.Vertices().col(asset.Triangles()(vtx, face));
        }
    }

    return outputDeltas;
}

Eigen::Matrix3Xf UpdateLowerLODVerticesRasterizer(const Mesh<float>& lod0Asset, const Mesh<float>& asset) {
    int width = 2048;
    int height = 2048;

    Eigen::Matrix3Xf outputDeltas = Eigen::Matrix3Xf::Zero(3, asset.Vertices().cols());

    // Initialize raster matrices
    Eigen::MatrixXi lod0TriIndex = Eigen::MatrixXi::Zero(width, height);
    Eigen::MatrixXf lod0BcX = Eigen::MatrixXf::Zero(width, height);
    Eigen::MatrixXf lod0BcY = Eigen::MatrixXf::Zero(width, height);
    Eigen::MatrixXf lod0BcZ = Eigen::MatrixXf::Zero(width, height);

    Eigen::Matrix<float, 4, 3> bcs = Eigen::Matrix<float, 4, 3>::Zero();
    bcs.col(0) = Eigen::Vector4f(1., 0., 0., 1.);
    bcs.col(1) = Eigen::Vector4f(0., 1., 0., 1.);
    bcs.col(2) = Eigen::Vector4f(0., 0., 1., 1.);

    // Rasterize each texture triangle in LOD0 mesh
    for (int tri = 0; tri < lod0Asset.TexTriangles().cols(); tri++) {
        std::function<void(int x, int y, float depth,
                           const Eigen::Vector3<float>& bc)> uvFunction = [&](int x, int y, float /*depth*/,
                                                                              const Eigen::Vector3<float>& bc) {
                lod0TriIndex(x, y) = tri;
                lod0BcX(x, y) = bc[0];
                lod0BcY(x, y) = bc[1];
                lod0BcZ(x, y) = bc[2];
            };
        Eigen::Matrix3Xf projectedVertices = Eigen::Matrix3Xf(3, 3);
        projectedVertices.col(2) = Eigen::Vector3f(width * lod0Asset.Texcoords().col(lod0Asset.TexTriangles()(0, tri))[0],
                                                   height * lod0Asset.Texcoords().col(lod0Asset.TexTriangles()(0, tri))[1],
                                                   0.5f);
        projectedVertices.col(1) = Eigen::Vector3f(width * lod0Asset.Texcoords().col(lod0Asset.TexTriangles()(1, tri))[0],
                                                   height * lod0Asset.Texcoords().col(lod0Asset.TexTriangles()(1, tri))[1],
                                                   0.5f);
        projectedVertices.col(0) = Eigen::Vector3f(width * lod0Asset.Texcoords().col(lod0Asset.TexTriangles()(2, tri))[0],
                                                   height * lod0Asset.Texcoords().col(lod0Asset.TexTriangles()(2, tri))[1],
                                                   0.5f);

        RasterizeTriangleInsideOut<float>(projectedVertices, bcs, width, height, uvFunction);
    }

    // Using raster matrices find correspondance to LOD0 and calculate delta
    for (int face = 0; face < asset.NumTriangles(); ++face) {
        for (int vtx = 0; vtx < 3; ++vtx) {
            // nearest neighbor
            int u = int(width * asset.Texcoords().col(asset.TexTriangles()(vtx, face))[0]);
            int v = int(height * asset.Texcoords().col(asset.TexTriangles()(vtx, face))[1]);
            int tIndex = lod0TriIndex(u, v);

            if (tIndex == 0) {
                outputDeltas.col(asset.Triangles()(vtx, face)) = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
                continue;
            }

            Eigen::Matrix3Xf vertices = Eigen::Matrix3Xf(3, 3);
            vertices.col(0) = lod0Asset.Vertices().col(lod0Asset.Triangles()(2, tIndex));
            vertices.col(1) = lod0Asset.Vertices().col(lod0Asset.Triangles()(1, tIndex));
            vertices.col(2) = lod0Asset.Vertices().col(lod0Asset.Triangles()(0, tIndex));
            Eigen::Vector3f barycentricCoords = Eigen::Vector3f(lod0BcX(u, v), lod0BcY(u, v), lod0BcZ(u, v));
            epic::nls::BarycentricCoordinates<float> bcOut(Eigen::Vector3i(0, 1, 2), barycentricCoords);

            Eigen::Vector3f newVertexPosition = bcOut.Evaluate<3>(vertices);

            outputDeltas.col(asset.Triangles()(vtx,
                                               face)) = newVertexPosition - asset.Vertices().col(asset.Triangles()(vtx, face));
        }
    }

    return outputDeltas;
}

bool ContainsSubstring(const std::string string, const std::string substring) {
    if (string.find(substring) != std::string::npos) {
        return true;
    }
    return false;
}

void SetVertexPositionsToAsset(int assetId, Eigen::Matrix3Xf vertices, dna::StreamWriter* dna) {
    size_t numMeshVertices = vertices.cols();
    dna->setVertexPositions(uint16_t(assetId), (dna::Position*)vertices.data(), uint32_t(numMeshVertices));
}

Eigen::Matrix3Xf UpdateVerticesWithDeformationGrid(GridDeformation<float>& gridDeformation,
                                                   const Eigen::Matrix3Xf& vertices,
                                                   const Eigen::Vector3f& offset = Eigen::Vector3f::Zero(),
                                                   const float scale = 1.0f) {
    const int vertexCount = static_cast<int>(vertices.cols());
    Eigen::Matrix3Xf output = Eigen::Matrix3Xf(3,
                                               vertexCount);

    for (int i = 0; i < vertexCount; ++i) {
        output.col(i) = scale * gridDeformation.EvaluateGridPosition(Eigen::Vector3f(vertices.col(i))) + offset;
    }

    return output;
}

void TransformJoints(const std::vector<Affine<float, 3, 3> >& transforms, dna::StreamWriter* dna,
                     const std::shared_ptr<RigGeometry<float> >& rigGeometry) {
    // replace the joints
    const JointRig2<float>& jointRig = rigGeometry->GetJointRig();
    std::map<std::string, int> jointNameToIndex;
    for (const std::string& jointName : jointRig.GetJointNames()) {
        jointNameToIndex[jointName] = jointRig.GetJointIndex(jointName);
    }

    std::vector<Affine<float, 3, 3> > jointWorldTransforms;

    const std::uint16_t numJoints = std::uint16_t(rigGeometry->GetJointRig().NumJoints());
    for (std::uint16_t jointIndex = 0; jointIndex < numJoints; jointIndex++) {
        Affine<float, 3, 3> jointWorldTransform(rigGeometry->GetBindMatrix(jointIndex));
        jointWorldTransform = transforms[jointIndex] * jointWorldTransform;
        jointWorldTransforms.push_back(jointWorldTransform);
    }

    Eigen::Matrix<float, 3, -1> jointTranslations(3, numJoints);
    Eigen::Matrix<float, 3, -1> jointRotations(3, numJoints);
    for (std::uint16_t jointIndex = 0; jointIndex < numJoints; jointIndex++) {
        Affine<float, 3, 3> localTransform;
        const int parentJointIndex = jointRig.GetParentIndex(jointIndex);
        if (parentJointIndex >= 0) {
            auto parentTransform = jointWorldTransforms[parentJointIndex];
            localTransform = parentTransform.Inverse() * jointWorldTransforms[jointIndex];
        } else {
            localTransform = jointWorldTransforms[jointIndex];
        }

        jointTranslations.col(jointIndex) = localTransform.Translation().cast<float>();

        constexpr float rad2deg = float(180.0 / CARBON_PI);
        jointRotations.col(jointIndex) = rad2deg * RotationMatrixToEulerXYZ<float>(localTransform.Linear());
    }

    // Update joint translations
    dna->setNeutralJointTranslations((dna::Vector3*)jointTranslations.data(), numJoints);

    // Update joint rotations
    dna->setNeutralJointRotations((dna::Vector3*)jointRotations.data(), numJoints);
}

void TransformRig(const Affine<float, 3, 3>& transform,
                  dna::StreamWriter* dna,
                  const std::shared_ptr<RigGeometry<float> >& rigGeometry,
                  int numLODs) {

    const std::uint16_t numJoints = std::uint16_t(rigGeometry->GetJointRig().NumJoints());

    Affine<float, 3, 3> identity;
    identity.SetIdentity();

    std::vector<Affine<float, 3, 3> > jointTransforms(numJoints);

    std::fill(jointTransforms.begin(), jointTransforms.end(), transform);
    TransformJoints(jointTransforms, dna, rigGeometry);

    for (int lod = 0; lod < numLODs; ++lod) {
        std::vector<int> meshIds = rigGeometry->GetMeshIndicesForLOD(lod);

        for (int i = 0; i < int(meshIds.size()); ++i) {
            int meshId = meshIds[i];
            Mesh<float> asset = rigGeometry->GetMesh(meshId);
            Eigen::Matrix3Xf vertices;
            vertices = transform.Transform(asset.Vertices());
            SetVertexPositionsToAsset(meshId, vertices, dna);
        }
    }
}


Eigen::Matrix3Xf CalculateDeltasUsingBaseMesh(const Mesh<float>& baseMesh,
                                              const Mesh<float>& assetMesh,
                                              const Eigen::Matrix3Xf& baseDeltas) {
    Mesh<float> baseMeshTris = baseMesh;
    baseMeshTris.Triangulate();

    Mesh<float> assetMeshTris = assetMesh;
    assetMeshTris.Triangulate();

    Eigen::Matrix3Xf outputDeltas = Eigen::Matrix3Xf::Zero(3, assetMesh.Vertices().cols());

    epic::carbon::AABBTree<float> aabbTree(baseMeshTris.Vertices().transpose(), baseMeshTris.Triangles().transpose());

    // Find intersection for each asset vertex
    for (int vtx = 0; vtx < assetMeshTris.NumVertices(); vtx++) {
        Eigen::Vector3f vtxPos = assetMeshTris.Vertices().col(vtx);

        const auto [triangleIndex, closestBarycentric,
                    squaredDistance] = aabbTree.getClosestPoint(vtxPos.transpose(), float(1e9));

        Eigen::Matrix3Xf vertices = Eigen::Matrix3Xf(3, 3);
        epic::nls::BarycentricCoordinates<float> bcOut(Eigen::Vector3i(0, 1, 2), closestBarycentric);

        vertices.col(0) = baseDeltas.col(baseMeshTris.Triangles()(0, triangleIndex));
        vertices.col(1) = baseDeltas.col(baseMeshTris.Triangles()(1, triangleIndex));
        vertices.col(2) = baseDeltas.col(baseMeshTris.Triangles()(2, triangleIndex));

        outputDeltas.col(vtx) = bcOut.Evaluate<3>(vertices);
    }

    return outputDeltas;
}

Eigen::Vector3f JointTranslationFromMeshes(const Eigen::Matrix3Xf& source,
                                           const Eigen::Matrix3Xf& target,
                                           const std::shared_ptr<RigGeometry<float> >& rigGeometry,
                                           const int jointIndex) {
    const auto [scale, transform] = Procrustes<float, 3>::AlignRigidAndScale(source, target);
    Eigen::Vector3f output = transform.Transform(Affine<float, 3, 3>(scale * rigGeometry->GetBindMatrix(jointIndex)).Translation());

    return output;
}

void RigMorphModule::Init(dna::StreamReader* inputDna) {
    m_memoryStream = pma::makeScoped<dna::MemoryStream>();
    m_streamReader = pma::makeScoped<dna::StreamReader>(m_memoryStream.get());

    auto streamWriter = pma::makeScoped<dna::StreamWriter>(m_memoryStream.get());
    streamWriter->setFrom(inputDna);

    streamWriter->write();
    m_streamReader->read();

    m_lodCount = inputDna->getLODCount();
    m_jointCount = inputDna->getJointCount();
}

dna::StreamReader* RigMorphModule::GetEstimatedDna() {
    return m_streamReader.get();
}

void RigMorphModule::ApplyRigidTransform(const Affine<float, 3, 3>& rigidTransform) {
    auto streamWriter = pma::makeScoped<dna::StreamWriter>(m_memoryStream.get());
    streamWriter->setFrom(m_streamReader.get());

    std::shared_ptr<RigGeometry<float> > rigGeometry = std::make_shared<RigGeometry<float> >();
    rigGeometry->Init(m_streamReader.get());
    TransformRig(rigidTransform, streamWriter.get(), rigGeometry, m_lodCount);

    m_streamReader = pma::makeScoped<dna::StreamReader>(m_memoryStream.get());
    streamWriter->write();
    m_streamReader->read();
}

void RigMorphModule::UpdateTeeth(const Eigen::Matrix3Xf& teethMeshVertices,
                                 const epic::nls::VertexWeights<float>& mouthSocketVertices,
                                 int gridSize) {
    auto streamWriter = pma::makeScoped<dna::StreamWriter>(m_memoryStream.get());
    streamWriter->setFrom(m_streamReader.get());

    const std::uint16_t numJoints = std::uint16_t(m_jointCount);

    Affine<float, 3, 3> identity;
    identity.SetIdentity();

    std::shared_ptr<RigGeometry<float> > rigGeometry = std::make_shared<RigGeometry<float> >();
    rigGeometry->Init(GetEstimatedDna());

    std::vector<Affine<float, 3, 3> > jointTransforms(numJoints, identity);
    for (std::uint16_t jointIndex = 0; jointIndex < numJoints; jointIndex++) {
        Affine<float, 3, 3> jointWorldTransform(rigGeometry->GetBindMatrix(jointIndex));
        jointTransforms.push_back(jointWorldTransform);
    }

    Mesh<float> targetTeethMesh = rigGeometry->GetMesh(teethId);
    targetTeethMesh.SetVertices(teethMeshVertices);
    targetTeethMesh.Triangulate();
    targetTeethMesh.CalculateVertexNormals();

    Mesh<float> sourceMesh = rigGeometry->GetMesh(teethId);

    Mesh<float> headMesh = rigGeometry->GetMesh(headId);

    const int gridPtsX = gridSize;
    const int gridPtsY = gridSize;
    const int gridPtsZ = gridSize;

    GridDeformation<float> gridDeformation(gridPtsX, gridPtsY, gridPtsZ);
    gridDeformation.Init(sourceMesh.Vertices());
    gridDeformation.Solve(sourceMesh.Vertices(), targetTeethMesh.Vertices(), 10.0);

    const int vertexCount = static_cast<int>(headMesh.NumVertices());
    Eigen::Matrix3Xf outputHeadVertices = headMesh.Vertices();

    for (int i = 0; i < vertexCount; ++i) {
        if (mouthSocketVertices.Weights()[i] > 0) {
            outputHeadVertices.col(i) += gridDeformation.EvaluateGridPosition(Eigen::Vector3f(headMesh.Vertices().col(i)));
        }
    }

    headMesh.SetVertices(outputHeadVertices);

    std::string upperTeethJointName = "FACIAL_C_TeethUpper";
    std::string lowerTeethJointName = "FACIAL_C_TeethLower";

    Eigen::Matrix3Xf lod0TeethDeltas;
    Mesh<float> targetSaliva = rigGeometry->GetMesh(salivaId);
    targetSaliva.Triangulate();

    Affine<float, 3, 3> upperTeethBindMatrix, lowerTeethBindMatrix;
    upperTeethBindMatrix.SetMatrix(rigGeometry->GetBindMatrix(m_jointNameToIndex[upperTeethJointName]));
    lowerTeethBindMatrix.SetMatrix(rigGeometry->GetBindMatrix(m_jointNameToIndex[lowerTeethJointName]));

    Eigen::Vector3f upperTeethJointPosDelta = JointTranslationFromMeshes(rigGeometry->GetMesh(teethId).Vertices(),
                                                                         teethMeshVertices,
                                                                         rigGeometry,
                                                                         m_jointNameToIndex[upperTeethJointName]) - upperTeethBindMatrix.Translation();

    Eigen::Vector3f lowerTeethJointPosDelta = JointTranslationFromMeshes(rigGeometry->GetMesh(teethId).Vertices(),
                                                                         teethMeshVertices,
                                                                         rigGeometry,
                                                                         m_jointNameToIndex[lowerTeethJointName]) - lowerTeethBindMatrix.Translation();

    jointTransforms[m_jointNameToIndex[upperTeethJointName]].SetTranslation(jointTransforms[m_jointNameToIndex[upperTeethJointName]].Translation() + upperTeethJointPosDelta);
    jointTransforms[m_jointNameToIndex[lowerTeethJointName]].SetTranslation(jointTransforms[m_jointNameToIndex[lowerTeethJointName]].Translation() + lowerTeethJointPosDelta);

    TransformJoints(jointTransforms, streamWriter.get(), rigGeometry);

    int numLODs = m_lodCount;

    for (int lod = 0; lod < numLODs; ++lod) {
        std::vector<int> meshIds = rigGeometry->GetMeshIndicesForLOD(lod);

        for (int i = 0; i < int(meshIds.size()); ++i) {
            int meshId = meshIds[i];
            std::string meshName = rigGeometry->GetMeshName(meshId);
            Mesh<float> asset = rigGeometry->GetMesh(meshId);
            Eigen::Matrix3Xf vertices;
            if (ContainsSubstring(meshName, "teeth")) {
                if (lod == 0) {
                    vertices = targetTeethMesh.Vertices();
                    lod0TeethDeltas = vertices - asset.Vertices();
                } else {
                    asset.Triangulate();
                    vertices = asset.Vertices() + UpdateLowerLODVerticesRaycasting(targetTeethMesh, asset);
                }
                SetVertexPositionsToAsset(meshId, vertices, streamWriter.get());
            } else if (ContainsSubstring(meshName, "saliva")) {
                if (lod == 0) {
                    vertices = asset.Vertices() + CalculateDeltasUsingBaseMesh(rigGeometry->GetMesh(teethId), asset, lod0TeethDeltas);
                    targetSaliva.SetVertices(vertices);
                    targetSaliva.CalculateVertexNormals();
                } else {
                    asset.Triangulate();
                    vertices = asset.Vertices() + UpdateLowerLODVerticesRaycasting(targetSaliva, asset);
                }
                SetVertexPositionsToAsset(meshId, vertices, streamWriter.get());
            }
            else if (ContainsSubstring(meshName, "head")) {
                if (lod == 0) {
                    vertices = headMesh.Vertices();
                }
                else {
                    asset.Triangulate();
                    vertices = asset.Vertices() + UpdateLowerLODVerticesRaycasting(headMesh, asset);
                }
                SetVertexPositionsToAsset(meshId, vertices, streamWriter.get());
            }
        }
    }

    m_streamReader = pma::makeScoped<dna::StreamReader>(m_memoryStream.get());
    streamWriter->write();
    m_streamReader->read();
}

void RigMorphModule::Morph(const Eigen::Matrix3Xf& targetHeadMeshVertices,
                           const Eigen::Matrix3Xf& targetEyeLeftMeshVertices,
                           const Eigen::Matrix3Xf& targetEyeRightMeshVertices,
                           int gridSize) {
    auto streamWriter = pma::makeScoped<dna::StreamWriter>(m_memoryStream.get());
    streamWriter->setFrom(m_streamReader.get());

    std::shared_ptr<RigGeometry<float> > rigGeometry = std::make_shared<RigGeometry<float> >();
    rigGeometry->Init(GetEstimatedDna());

    Mesh<float> targetHeadMesh = rigGeometry->GetMesh(headId);
    targetHeadMesh.SetVertices(targetHeadMeshVertices);
    targetHeadMesh.Triangulate();
    targetHeadMesh.CalculateVertexNormals();
    Mesh<float> sourceMesh = rigGeometry->GetMesh(headId);
    bool useEyeAssets = (targetEyeLeftMeshVertices.size() == 0 || targetEyeRightMeshVertices.size() == 0) ? false : true;

    const int gridPtsX = gridSize;
    const int gridPtsY = gridSize;
    const int gridPtsZ = gridSize;

    GridDeformation<float> gridDeformation(gridPtsX, gridPtsY, gridPtsZ);
    gridDeformation.Init(sourceMesh.Vertices());
    gridDeformation.Solve(sourceMesh.Vertices(), targetHeadMesh.Vertices(), 10.0);

    // replace the joints
    const JointRig2<float>& jointRig = rigGeometry->GetJointRig();

    for (const std::string& jointName : jointRig.GetJointNames()) {
        m_jointNameToIndex[jointName] = jointRig.GetJointIndex(jointName);
    }

    std::vector<Affine<float, 3, 3> > jointWorldTransforms;

    std::string leftEyeJointName = "FACIAL_L_Eye", rightEyeJointName = "FACIAL_R_Eye", teethJointName = "FACIAL_C_TeethUpper";

    if (m_jointNameToIndex.find(leftEyeJointName) == m_jointNameToIndex.end()) {
        CARBON_CRITICAL("{} does not exist in the input DNA file.", leftEyeJointName);
    }
    if (m_jointNameToIndex.find(rightEyeJointName) == m_jointNameToIndex.end()) {
        CARBON_CRITICAL("{} does not exist in the input DNA file.", rightEyeJointName);
    }
    if (m_jointNameToIndex.find(teethJointName) == m_jointNameToIndex.end()) {
        CARBON_CRITICAL("{} does not exist in the input DNA file.", teethJointName);
    }

    const std::uint16_t numJoints = std::uint16_t(m_jointCount);
    for (std::uint16_t jointIndex = 0; jointIndex < numJoints; jointIndex++) {
        Affine<float, 3, 3> jointWorldTransform(rigGeometry->GetBindMatrix(jointIndex));

        const Eigen::Vector3<float> originalJointPosition = jointWorldTransform.Translation();
        // get the joint translation to new deformed state
        Eigen::Vector3<float> jointTranslation = gridDeformation.EvaluateGridPosition(originalJointPosition);

        // get the joint rotation from the estimate grid cell transformation, but only keep the rotational component, the joint
        // should transform optimally

        jointWorldTransform.SetTranslation(jointTranslation + originalJointPosition);
        jointWorldTransforms.push_back(jointWorldTransform);
    }

    // specific asset-based calculations
    Affine<float, 3, 3> teethRig2TargetTransform = jointWorldTransforms[m_jointNameToIndex[teethJointName]]
                                                 * Affine<float, 3, 3>(rigGeometry->GetBindMatrix(m_jointNameToIndex[teethJointName])).Inverse();

    Mesh<float> targetEyeLeftMesh = rigGeometry->GetMesh(eyeLeftId);
    Mesh<float> targetEyeRightMesh = rigGeometry->GetMesh(eyeRightId);
    targetEyeLeftMesh.Triangulate();
    targetEyeRightMesh.Triangulate();

    if (useEyeAssets) {
        targetEyeLeftMesh.SetVertices(targetEyeLeftMeshVertices);
        targetEyeRightMesh.SetVertices(targetEyeRightMeshVertices);

        jointWorldTransforms[m_jointNameToIndex[leftEyeJointName]].SetTranslation(JointTranslationFromMeshes(rigGeometry->GetMesh(eyeLeftId).Vertices(),
                                                                                                             targetEyeLeftMeshVertices,
                                                                                                             rigGeometry,
                                                                                                             m_jointNameToIndex[leftEyeJointName]));
        jointWorldTransforms[m_jointNameToIndex[rightEyeJointName]].SetTranslation(JointTranslationFromMeshes(rigGeometry->GetMesh(eyeRightId).Vertices(),
                                                                                                              targetEyeRightMeshVertices,
                                                                                                              rigGeometry,
                                                                                                              m_jointNameToIndex[rightEyeJointName]));
    }
    else {
        Affine<float, 3, 3> eyeLeftRig2TargetTransform = jointWorldTransforms[m_jointNameToIndex[leftEyeJointName]] *
                                                         Affine<float, 3, 3>(rigGeometry->GetBindMatrix(m_jointNameToIndex[leftEyeJointName])).Inverse();

        Affine<float, 3, 3> eyeRightRig2TargetTransform = jointWorldTransforms[m_jointNameToIndex[rightEyeJointName]] *
                                                          Affine<float, 3, 3>(rigGeometry->GetBindMatrix(m_jointNameToIndex[rightEyeJointName])).Inverse();

        targetEyeLeftMesh.SetVertices(eyeLeftRig2TargetTransform.Transform(targetEyeLeftMesh.Vertices()));
        targetEyeRightMesh.SetVertices(eyeRightRig2TargetTransform.Transform(targetEyeRightMesh.Vertices()));
    }

    targetEyeLeftMesh.CalculateVertexNormals();
    targetEyeRightMesh.CalculateVertexNormals();

    // local transformations
    Eigen::Matrix<float, 3, -1> jointTranslations(3, numJoints);
    Eigen::Matrix<float, 3, -1> jointRotations(3, numJoints);
    for (std::uint16_t jointIndex = 0; jointIndex < numJoints; jointIndex++) {
        Affine<float, 3, 3> localTransform;
        const int parentJointIndex = jointRig.GetParentIndex(jointIndex);
        if (parentJointIndex >= 0) {
            auto parentTransform = jointWorldTransforms[parentJointIndex];
            localTransform = parentTransform.Inverse() * jointWorldTransforms[jointIndex];
        } else {
            localTransform = jointWorldTransforms[jointIndex];
        }

        jointTranslations.col(jointIndex) = localTransform.Translation().cast<float>();

        constexpr float rad2deg = float(180.0 / CARBON_PI);
        jointRotations.col(jointIndex) = rad2deg * RotationMatrixToEulerXYZ<float>(localTransform.Linear());
    }

    // Update joint translations
    streamWriter->setNeutralJointTranslations((dna::Vector3*)jointTranslations.data(), numJoints);

    int numLODs = m_lodCount;
    Mesh<float> targetCartilage;
    Eigen::Matrix3Xf headLod0Deltas;

    for (int lod = 0; lod < numLODs; ++lod) {
        std::vector<int> meshIds = rigGeometry->GetMeshIndicesForLOD(lod);

        for (int i = 0; i < int(meshIds.size()); ++i) {
            int meshId = meshIds[i];
            std::string meshName = rigGeometry->GetMeshName(meshId);
            Mesh<float> asset = rigGeometry->GetMesh(meshId);
            Eigen::Matrix3Xf vertices;
            if (ContainsSubstring(meshName, "head")) {
                if (lod == 0) {
                    vertices = targetHeadMesh.Vertices();
                    headLod0Deltas = vertices - asset.Vertices();
                } else {
                    asset.Triangulate();
                    vertices = asset.Vertices() + UpdateLowerLODVerticesRaycasting(targetHeadMesh, asset);
                }
            } else if (ContainsSubstring(meshName, "eyeLeft")) {
                if (lod == 0) {
                    vertices = targetEyeLeftMesh.Vertices();
                } else {
                    asset.Triangulate();
                    vertices = asset.Vertices() + UpdateLowerLODVerticesRaycasting(targetEyeLeftMesh, asset);
                }
            } else if (ContainsSubstring(meshName, "eyeRight")) {
                if (lod == 0) {
                    vertices = targetEyeRightMesh.Vertices();
                } else {
                    asset.Triangulate();
                    vertices = asset.Vertices() + UpdateLowerLODVerticesRaycasting(targetEyeRightMesh, asset);
                }
            } else if (ContainsSubstring(meshName, "teeth") || ContainsSubstring(meshName, "saliva")) {
                vertices = teethRig2TargetTransform.Transform(asset.Vertices());
            } else if (ContainsSubstring(meshName, "eyeshell") || ContainsSubstring(meshName, "eyelashes")) {
                vertices = asset.Vertices() + UpdateVerticesWithDeformationGrid(gridDeformation, asset.Vertices());
            } else if (ContainsSubstring(meshName, "cartilage")) {
                if (lod == 0) {
                    targetCartilage = asset;
                    targetCartilage.Triangulate();
                    vertices = asset.Vertices() + CalculateDeltasUsingBaseMesh(rigGeometry->GetMesh(headId), targetCartilage, headLod0Deltas);
                    targetCartilage.SetVertices(vertices);
                    targetCartilage.CalculateVertexNormals();
                }
                else {
                    asset.Triangulate();
                    vertices = asset.Vertices() + UpdateLowerLODVerticesRaycasting(targetCartilage, asset);
                }
            } else if (ContainsSubstring(meshName, "eyeEdge")) {
                vertices = asset.Vertices() +
                    UpdateVerticesWithDeformationGrid(gridDeformation, asset.Vertices(), Eigen::Vector3f(0.0f, 0.0f, 0.005f));
            } else {
                printf("Mesh from DNA file not supported.");
            }
            SetVertexPositionsToAsset(meshId, vertices, streamWriter.get());
        }
    }

    m_streamReader = pma::makeScoped<dna::StreamReader>(m_memoryStream.get());
    streamWriter->write();
    m_streamReader->read();
}

void RigMorphModule::SaveDna(std::string outputDnaPath) {
    // output the dna file
    pma::ScopedPtr<dna::FileStream> outputStream = pma::makeScoped<dna::FileStream>(outputDnaPath.c_str(),
                                                                                    dna::FileStream::AccessMode::Write,
                                                                                    dna::FileStream::OpenMode::Binary);
    pma::ScopedPtr<dna::StreamWriter> writer = pma::makeScoped<dna::StreamWriter>(outputStream.get());

    writer->setFrom(GetEstimatedDna());
    writer->write();
}
