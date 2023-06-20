// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Defs.h"

#include <map>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

namespace dna {
class StreamReader;
class StreamWriter;
}

namespace titan {
namespace api {

enum class RefinementMaskType {
    MOUTH_SOCKET
};

class TITAN_API ActorRefinementAPI {
public:
    ActorRefinementAPI();
    ~ActorRefinementAPI();
    ActorRefinementAPI(ActorRefinementAPI&&) = delete;
    ActorRefinementAPI(const ActorRefinementAPI&) = delete;
    ActorRefinementAPI& operator=(ActorRefinementAPI&&) = delete;
    ActorRefinementAPI& operator=(const ActorRefinementAPI&) = delete;

     /**
     * Get vertex weights for mask type.
     * @param[out] InVertexWeights weight values
     * @param[in] InMaskType mask type
     * @returns True if was successful.
     */
    bool GetFittingMask(float* OutVertexWeights, RefinementMaskType InMaskType);

     /**
     * Set vertex weights for mask type.
     * @param[in] Num of input weights
     * @param[in] InVertexWeights weight values
     * @param[in] InMaskType mask type
     * @returns True if was successful.
     */
    bool SetFittingMask(int32_t NumVertices, float* InVertexWeights, RefinementMaskType InMaskType);

    /**
    * Update teeth model and position in the rig given input data.
    * @param[in] InDnaStream  Input DNA stream.
    * @param[in] InTeethMeshVertexPositions  Input target vertex positions as numVertices x 3 in column major order..
    * @param[out] OutDnaStream  Updated dna stream.
    * @returns True if fitting and DNA update was successful.
    */
    bool RefineTeeth(dna::StreamReader* InDnaStream, const float* InTeethMeshVertexPositions,
                        dna::StreamWriter* OutDnaStream);

    /**
    * Update the joints and mesh assets using target head mesh vertex positions.
    * @param[in] InDnaStream  Input DNA stream.
    * @param[in] InHeadMeshVertexPositions  Input target vertex positions as numVertices x 3 in column major order.
    * @param[out] OutDnaStream  Updated dna stream.
    * @returns True if DNA update was successful.
    */
    bool RefineRig(dna::StreamReader* InDnaStream,
                    const float* InHeadMeshVertexPositions,
                    const float* InEyeLeftMeshVertexPositions,
                    const float* InEyeRightMeshVertexPositions,
                    dna::StreamWriter* OutDnaStream);

    /**
    * Transform the rig with 4x4 transform matrix.
    * @param[in] InDnaStream  Input DNA stream.
    * @param[in] InTransformMatrix  Input 4x4 transformation matrix in column major order.
    * @param[out] OutDnaStream  Updated dna stream.
    * @returns True if DNA update was successful.
    */
    bool TransformRigOrigin(dna::StreamReader* InDnaStream, const float* InTransformMatrix, dna::StreamWriter* OutDnaStream);

private:
    struct Private;
    Private* m;
};

}
}
