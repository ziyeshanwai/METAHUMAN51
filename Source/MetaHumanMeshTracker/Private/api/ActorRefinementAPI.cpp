// Copyright Epic Games, Inc. All Rights Reserved.

#include <ActorRefinementAPI.h>

#include <carbon/utils/TaskThreadPool.h>
#include <Common.h>
#include <nls/rig/RigLogicDNAResource.h>
#include <pma/PolyAllocator.h>
#include <pma/utils/ManagedInstance.h>
#include <conformer/RigMorphModule.h>

#include <cstring>
#include <filesystem>
#include <map>

using namespace epic::nls;
using namespace pma;

namespace titan::api {

struct ActorRefinementAPI::Private {
    std::shared_ptr<epic::carbon::TaskThreadPool> globalThreadPool = epic::carbon::TaskThreadPool::GlobalInstance(
        /*createIfNotAvailable=*/ true);
    std::map<RefinementMaskType, VertexWeights<float> > masks;
};


ActorRefinementAPI::ActorRefinementAPI()
    : m(new Private()) {
}

ActorRefinementAPI::~ActorRefinementAPI() {
    delete m;
}

bool ActorRefinementAPI::RefineTeeth(dna::StreamReader* InDnaStream,
                                     const float* InTeethMeshVertexPositions,
                                     dna::StreamWriter* OutDnaStream) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(InDnaStream, false, "input dna stream not valid");
        PolyAllocator<RigMorphModule> polyAllocator{MEM_RESOURCE};

        Eigen::Matrix3Xf verticesMap = Eigen::Map<const Eigen::Matrix<float, 3, -1, Eigen::ColMajor> >(
            (const float*)InTeethMeshVertexPositions,
            3,
            InDnaStream->getVertexPositionCount(0)).template cast<float>();

        std::shared_ptr<RigMorphModule> rigMorphing = std::allocate_shared<RigMorphModule>(polyAllocator);
        rigMorphing->Init(InDnaStream);
        epic::nls::VertexWeights<float> weights(int(verticesMap.cols()), 0.0f);
        auto it = m->masks.find(RefinementMaskType::MOUTH_SOCKET);
        if (it != m->masks.end()) {
            weights = m->masks[RefinementMaskType::MOUTH_SOCKET];
        }

        rigMorphing->UpdateTeeth(verticesMap, weights);

        OutDnaStream->setFrom(rigMorphing->GetEstimatedDna());

        return true;
    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to fit teeth: {}", e.what());
    }
}

bool ActorRefinementAPI::RefineRig(dna::StreamReader* InDnaStream,
                                   const float* InHeadMeshVertexPositions,
                                   const float* InEyeLeftMeshVertexPositions,
                                   const float* InEyeRightMeshVertexPositions,
                                   dna::StreamWriter* OutDnaStream) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(InDnaStream, false, "input dna stream not valid");
        CHECK_OR_RETURN(OutDnaStream, false, "output dna stream not valid");
        PolyAllocator<RigMorphModule> polyAllocator{MEM_RESOURCE};

        Eigen::Matrix3Xf headVerticesMap = Eigen::Map<const Eigen::Matrix<float, 3, -1, Eigen::ColMajor> >(
            (const float*)InHeadMeshVertexPositions,
            3,
            InDnaStream->getVertexPositionCount(0)).template cast<float>();

        Eigen::Matrix3Xf eyeLeftVerticesMap, eyeRightVerticesMap;
        if (InEyeLeftMeshVertexPositions && InEyeRightMeshVertexPositions) {
            eyeLeftVerticesMap = Eigen::Map<const Eigen::Matrix<float, 3, -1, Eigen::ColMajor> >(
                (const float*)InEyeLeftMeshVertexPositions,
                3,
                InDnaStream->getVertexPositionCount(3)).template cast<float>();

            eyeRightVerticesMap = Eigen::Map<const Eigen::Matrix<float, 3, -1, Eigen::ColMajor> >(
                (const float*)InEyeRightMeshVertexPositions,
                3,
                InDnaStream->getVertexPositionCount(4)).template cast<float>();
        }

        std::shared_ptr<RigMorphModule> rigMorphing = std::allocate_shared<RigMorphModule>(polyAllocator);
        rigMorphing->Init(InDnaStream);
        rigMorphing->Morph(headVerticesMap, eyeLeftVerticesMap, eyeRightVerticesMap,  /*gridSize*/ 128);

        OutDnaStream->setFrom(rigMorphing->GetEstimatedDna());

        return true;
    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to modify dna: {}", e.what());
    }
}

bool ActorRefinementAPI::TransformRigOrigin(dna::StreamReader* InDnaStream,
                                            const float* InTransformMatrix,
                                            dna::StreamWriter* OutDnaStream) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(InDnaStream, false, "input dna stream not valid");
        CHECK_OR_RETURN(OutDnaStream, false, "output dna stream not valid");
        PolyAllocator<RigMorphModule> polyAllocator{MEM_RESOURCE};

        Eigen::Matrix4f transformMap = Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> >(
            (const float*)InTransformMatrix,
            4,
            4).template cast<float>();

        Affine<float, 3, 3> rigTransformation;
        rigTransformation.SetMatrix(transformMap);

        std::shared_ptr<RigMorphModule> rigMorphing = std::allocate_shared<RigMorphModule>(polyAllocator);
        rigMorphing->Init(InDnaStream);
        rigMorphing->ApplyRigidTransform(rigTransformation);

        OutDnaStream->setFrom(rigMorphing->GetEstimatedDna());

        return true;
    } catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to modify dna: {}", e.what());
    }
}

bool ActorRefinementAPI::GetFittingMask(float* OutVertexWeights, RefinementMaskType InMaskType) {
    try {
        RESET_ERROR;
        CHECK_OR_RETURN(!m->masks.empty(), false, "frame data is empty");

        // mask
        const auto& weights = m->masks[InMaskType].Weights();

        memcpy(OutVertexWeights,
            weights.data(),
            int32_t(weights.cols() * weights.rows()) *
            sizeof(float));

        return true;
    }
    catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to get mask: {}", e.what());
    }
}

bool ActorRefinementAPI::SetFittingMask(int32_t numVertices, float* InVertexWeights, RefinementMaskType InMaskType) {
    try {
        RESET_ERROR;

        Eigen::VectorXf weightsMap = Eigen::Map<const Eigen::VectorXf >(
            (const float*)InVertexWeights,
            numVertices).template cast<float>();

        m->masks[InMaskType] = VertexWeights<float>(weightsMap);

        return true;
    }
    catch (const std::exception& e) {
        HANDLE_EXCEPTION("failure to set mask: {}", e.what());
    }

}

}
