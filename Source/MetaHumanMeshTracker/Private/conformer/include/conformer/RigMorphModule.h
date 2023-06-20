// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <dna/StreamReader.h>
#include <dna/StreamWriter.h>
#include <nrr/DataDescription.h>
#include <nrr/TemplateDescription.h>

using namespace epic::nls;

class RigMorphModule {
    public:
        RigMorphModule() {
        }

        void Init(dna::StreamReader* inputDna);

        // perform the volumetric morph assuming that input target meshes are in "rig" space
        void Morph(const Eigen::Matrix3Xf& targetHeadMeshVertices,
                   const Eigen::Matrix3Xf& targetEyeLeftMeshVertices,
                   const Eigen::Matrix3Xf& targetEyeRightMeshVertices,
                   int gridSize = 128);

        // update teeth in dna
        // @ teethMesh is assumed to be in original space
        void UpdateTeeth(const Eigen::Matrix3Xf& teethMeshVertices,
                         const epic::nls::VertexWeights<float>& mouthSocketVertices,
                         int gridSize = 128);

        void ApplyRigidTransform(const Affine<float, 3,3>& rigidTransform);

        void SaveDna(std::string outputDnaPath);

        dna::StreamReader* GetEstimatedDna();

    private:
        pma::ScopedPtr<dna::StreamReader> m_streamReader;
        pma::ScopedPtr<dna::MemoryStream> m_memoryStream;
        std::map<std::string, int> m_jointNameToIndex;
        int m_lodCount;
        int m_jointCount;
};
