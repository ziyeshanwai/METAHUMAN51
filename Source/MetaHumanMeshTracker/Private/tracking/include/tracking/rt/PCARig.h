// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>
#include <nls/geometry/Mesh.h>
#include <tracking/rt/HeadVertexState.h>
#include <tracking/rt/LinearEyeModel.h>
#include <tracking/rt/LinearVertexModel.h>

namespace dna {
class StreamReader;
class StreamWriter;
}

namespace epic::nls::rt {

const char* EyeLeftJointName();
const char* EyeRightJointName();
const char* FacialRootJointName();

const char* HeadMeshName();
const char* TeethMeshName();
const char* EyeLeftMeshName();
const char* EyeRightMeshName();

struct PCARig
{
    //! PCA model of the face vertices
    LinearVertexModel<float> facePCA;

    //! PCA model of the teeth vertices
    LinearVertexModel<float> teethPCA;

    //! PCA model of the parameters of the left eye (rotation, translation)
    LinearEyeModel<float> eyeLeftTransformPCA;

    //! PCA model of the parameters of the right eye (rotation, translation)
    LinearEyeModel<float> eyeRightTransformPCA;

    //! global root bind pose (required to save the eye joints relative to the root)
    Eigen::Transform<float, 3, Eigen::Affine> rootBindPose;

    //! meshes of face, teeth, eye left, and eye right
    Mesh<float> meshes[4];

    int NumCoeffs() const { return static_cast<int>(facePCA.NumPCAModes()); }
    HeadVertexState<float> EvaluatePCARig(const Eigen::VectorX<float>& pcaCoeffs) const;

    Eigen::VectorX<float> Project(const HeadVertexState<float>& headVertexState, const Eigen::VectorX<float>& coeffs) const;

    //! Translates the rig globally, moving the vertices and the bind poses
    void Translate(const Eigen::Vector3f& translation);

    //! @returns the midepoint between the eyes
    Eigen::Vector3f EyesMidpoint() const;

    void SaveAsDNA(const std::string& filename) const;
    void SaveAsDNA(dna::StreamWriter* writer) const;

    //! Save the pca model as a large npy matrix including face, teeth, and eyes
    void SaveAsNpy(const std::string& filename) const;

    bool LoadFromDNA(const std::string& filename);
    bool LoadFromDNA(dna::StreamReader* reader);
};

} // namespace epic::nls::rt
