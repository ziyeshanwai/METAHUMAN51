// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include "ControlMapper.h"
#include <nrr/deformation_models/DeformationModelRigLogic.h>
#include <vector>
#include <string>
#include <random>


namespace epic
{
    namespace rigposebasedsolver
    {
        using namespace epic::carbon;
        using namespace epic::nls;

        struct FramePoseData;

        /**
         * RigGenerateTrainingData is a class which can generate synthetic (vertex) training data for a facial rig for training a
         * 3D pose-based solver based upon a set of pre-defined animation 'poses' and a set of parameters defining what vertex data to extract 
         * from the rig meshes, and how to generate/augment the training data.
         */
        class RigGenerateTrainingData
        {
        public:

            /**
             * A simple struct defining the a set of vertex ids to be extracted from a particular rig mesh, and an 'offset' which is
             * used to ensure unique ids across the whole rig.
             */
            struct MeshVertexIdData {
                std::vector<std::pair<int, std::vector<unsigned>>> vertexIdsAndShapeIndices;
                int vertexIdOutputOffset = 0;

                /*
                    friend function for serialization
                */
                friend void serialize(const MeshVertexIdData& item, std::ostream& out);

                /*
                    friend function for deserialization
                */
                friend void deserialize(MeshVertexIdData& item, std::istream& in);

                static const unsigned version;
            };

            /**
             * A simple struct defining how synthetic training data should be generated from the rig: which controls may be exercised during augmentation,
             * and a number of parameters for augmenting the training data.
             */
            struct PoseBasedSolverTrainingDataParameters
            {
                int randomSeed = 0;
                double vertexNoiseSd = 0;
                double rigAlignmentVertexSd = 0;
                unsigned numAugmentationsPerFrame = 0;
                unsigned numControlsPerAugmentation = 0;
                double fractionalAugmentationSd = 0;
                std::vector<std::string> solverControlNames;
                std::vector<ControlMapper> controlMappers;

                /*
                    friend function for serialization
                */
                friend void serialize(const PoseBasedSolverTrainingDataParameters& item, std::ostream& out);

                /*
                    friend function for deserialization
                */
                friend void deserialize(PoseBasedSolverTrainingDataParameters& item, std::istream& in);

                static const unsigned version;
            };

            /*
            * Default constructor
            */
            inline RigGenerateTrainingData()
            {
            }

            /*
            * Initialize the object with a shared ptr to the rig being used to generate the training data. Requires that IsRigGenerationDataConsistent
            * (rig, animationData, rigMeshOutputVertexIdData, poseBasedSolverTrainingDataParameters) is true
            * @param[in] rig: a shared ptr to the rig
            * @param[in] animationData: a vector of animation control data for each training set (originally a set of .qsa files); each element of the vector
            * is a map of control names to a map of int (frame number) to control value.
            * @param[in] rigMeshOutputVertexIdData: a map of rig mesh name to MeshVertexIdData (defining vertex indices of the rig to use) for each rig mesh to use
            * @param[in] poseBasedSolverTrainingDataParameters a struct containing parameters specifying how to generate and augment the rig data
            * @param[in] numShapes: the number of shapes used in the solver
            */
            void Init(std::shared_ptr< const Rig<float>> rig, const std::vector< std::map<std::string, std::map<int, double>> >& animationData,
                const std::map< std::string, MeshVertexIdData>& rigMeshOutputVertexIdData, 
                const PoseBasedSolverTrainingDataParameters& poseBasedSolverTrainingDataParameters,
                unsigned numShapes);

            /*
            * @returns true if the object has been initialized (Init() called), false otherwise
            */
            bool IsInitialized() const;

            /*
            * @returns true if the specified vertexIds are in range for the specified meshName for the rig, and also no duplicate vertex ids, 
            * each shape index must be in range 0-numShapes-1, returns false otherwise
            * @param[in] rig: a shared ptr to the rig
            * @param[in] meshName: the name of the mesh that we are checking vertex ids for
            * @param[in] vertexIdsAndShapeIndices: a vector of pairs of vertex ids and corresponding vector of shape indices that the vertex is should be in
            * @param[in] meshName: the name of the mesh that we are checking vertex ids for
            * @param[in] numShapes: the number of shapes used in the solver
            * @param[out] errorString: on return will contain a string describing any problems with the data
            */
            static bool IsValidMeshData(std::shared_ptr<const Rig<float>> rig, const std::string& meshName, 
                const std::vector<std::pair<int, std::vector<unsigned>>>& vertexIdsAndShapeIndices, unsigned numShapes, std::string& errorString);

            /*
            * @returns a vector of all meshnames which can be used to generate the training data
            */
            static const std::vector<std::string>& MeshNames();

            /*
            * Check if the supplied animation data, rigMeshOutputVertexIdData and poseSolverTrainingDataParameters are is consistent with the rig. 
            * @param[in] rig: a shared ptr to the rig
            * @param[in] animationData: a vector of animation control data for each training set (originally a set of .qsa files); each element of the vector
            * is a map of control names to a map of int (frame number) to control value.
            * @param[in] rigMeshOutputVertexIdData: a map of rig mesh name to MeshVertexIdData (defining vertex indices of the rig to use) for each rig mesh to use
            * @param[in] numShapes: the number of shapes used in the solver
            * @param[out] errorString: on return will contain a string describing any problems with the data
            * @param[in] poseBasedSolverTrainingDataParameters a struct containing parameters specifying how to generate and augment the rig data
            * @returns true if consistent, false otherwise
            */
            static bool IsRigGenerationDataConsistent(std::shared_ptr<const Rig<float>> rig, const std::vector< std::map<std::string, std::map<int, double>> >& animationData,
                const std::map< std::string, MeshVertexIdData>& rigMeshOutputVertexIdData, const PoseBasedSolverTrainingDataParameters& poseBasedSolverTrainingDataParameters,
                unsigned numShapes, std::string& errorString);

            /*
            * Generate the training data for the solver, based upon the previously supplied animationData and parameters. Requires that Init() has been called.
            * @param[out]: on return poseData contains the generated animation data and corresponding rig vertices
            * @param[in]: progressCallback a function to call to report progress
            * @param[in]: progress the current progress
            * @param[in]: progressStep how much this function contributes to overall progress
            * @param[in]: cancelledCallback a function to call to see if user has cancelled the operation
            */
            void GenerateTrainingData(std::vector<FramePoseData>& poseData, 
                std::function<void(float)> progressCallback = [](float) {}, float progress = 0.0, float progressStep = 100.0, 
                std::function<bool(void)> cancelledCallback = []() { return false; });

            /*
            *Evaluate the rig vertices for the set of control values supplied in curRigControlValues.Optionally add random perturbation to the control values
            * (which will then be modified to reflect that), noise to the vertex position, and noise to the overall rigid alignment of the rig, all as specified in the config file.
            * Requires that function Init() has been called already, and that curRigControlValues is the correct size ie the size of the number of UI controls.
            * @param[in / out] curRigControlValues: on input, should contain the values for each UI control which we want to evaluate.On return, if addControlPerturbation is
            * set to true, the values will be updated to reflect the perturbation.
            * @param[out] vertices: on return, will contain the evaluated rig vertices for each mesh specified in MeshNames(), with any specified perturbations added to the vertices.
            * @param[int] addControlPerturbation: if set to true, input rig controls will be randomly perturbed as specified in the config file, if false, no perturbation will be applied.
            * @param[in] addVertexNoise: if set to true, the evaluated rig vertices will be randomly perturbed as specified in the config file, if false, no perturbation will be applied.
            * @param[in] addRigidAlignmentNoise: if set to true, the overall rigid pose of the evaluated rig vertices will be randomly perturbed as specified in the config file, if false, no perturbation will be applied.
            */
            void EvaluateRigVerticesAndPerturbedControlValues(Eigen::VectorX<float>& curRigControlValues, std::vector<Eigen::Matrix<float, 3, -1>>& vertices,
                bool addControlPerturbation, bool addVertexNoise, bool addRigidAlignmentNoise);

            /*
            * Get the UI animation control data for all UI controls corresponding to animation subset animationIndex, frame frameNumber. Any controls
            * which don't exist in the original data-set are set to default values. Requires IsInitialized() to be true, animationIndex < the size of the original animationData vector
            * and frameNumber to be in range for the corresponding animation data sub-set
            * @param[in] animationIndex: the animation subset index
            * @param[in] frameNumber: the frame number in the animation subset
            * @param[out] rigControlValues: on return contains the UI animation control data for the specified frame
            */
            void GetRigControlData(unsigned animationIndex, unsigned frameNumber, Eigen::VectorX<float>& rigControlValues) const;

            /*
             * @returns a matrix containing the min and max control values for each UI control of the rig. Requires IsInitialized() to be true
             */
            const Eigen::Matrix<float, 2, -1>& GuiControlRanges() const;

            /*
             * @returns a vector of the UI control names of the rig. Requires IsInitialized() to be true
             */
            const std::vector<std::string>& GuiControlNames() const;

            /*
             * @returns a vector of all UI control names to be used in the pose-based solver. Requires IsInitialized() to be true
             */
            const std::vector<std::string>& SolverControlNames() const;

            /*
            * Get the min, max and default values for rig UI control controlName.  Requires IsInitialized() to be true and
            * controlName to be a valid UI control name for the rig.
            * @param[in] controlName: the name of the control we want the range for
            * @param[out] minVal: on return contains the min value for the specified UI control
            * @param[out] maxVal: on return contains the max value for the specified UI control
            * @param[out] defaultVal: on return contains the default value for the specified UI control
            */
            void GetSolverControlMinMaxDefault(const std::string& controlName, float& minVal, float& maxVal, float& defaultVal) const;

            /*
             * @returns the default control value for each UI control of the rig. Requires IsInitialized() to be true 
             */
            const Eigen::VectorX<float>& DefaultUIControlValues() const;

            /*
              * @returns the name of the meshRigIndex'th mesh in the rig. Requires IsInitialized() to be true and
              * meshRigIndex to be in range for the number of meshes associated with the rig
              * @param[in] meshRigIndex: the index of the mesh we want the mesh name for
             */
            const std::string& GetMeshName(int meshRigIndex) const;

            /*
             * @returns the vertex id offset to be applied for mesh meshName during solver training. Requires IsInitialized() to be true and
             * meshName to be a valid mesh name in the list of meshes specified in the solver rigMeshOutputVertexIdData data
             * @param[in] meshName: the name of the rig mesh which we want to return the vertex id offset
             */
            int VertexOffset(const std::string& meshName) const;

            /*
             * @returns the mesh name meshName. Requires IsInitialized() to be true and
             * meshName to be a valid mesh name for the rig
             * @param[in] meshName: the name of the rig mesh which we want to return
             */
            const Mesh<float>& GetMesh(const std::string& meshName) const;

            /*
            * @returns the number of training animation frames for animation data-set indice animationIndex. Requires IsInitialized() to be true and
            * animationIndex < the size of the original animationData vector
            * @param[in] animationIndex: the animation subset index for which we want the number of animation frames
            */
            unsigned GetNumAnimationFrames(unsigned animationIndex) const;

            /*
            * @returns a vector of vertex ids to be used in the solver from rig mesh meshName and corresponding shape indices for each vertex. 
            * If no vertices are used from mesh meshName (or the name is invalid)
            * an empty vector is returned. Requires IsInitialized() to be true
            * @param[in] meshName: the name of the rig mesh for which we want the solver vertex ids
            */
            std::vector<std::pair<int, std::vector<unsigned>>> OutputVertexIdsAndShapeIndices(const std::string& meshName) const;

            /*
            * @returns the vertex id data which is needed from the rig mesh(es) to
            * create input for the solver: a map of meshname to MeshVertexIdData (which contains a vector of the rig vertex indices to use and also 
            * an offset to use so that these have unique names within the solver).
            */
            const std::map< std::string, MeshVertexIdData>& RigMeshOutputVertexIdData() const;

        private:
            std::vector<std::string> m_guiControlNames;
            Eigen::VectorX<float> m_defaultUIControlValues;
            std::vector< std::map<std::string, std::map<int, double>> > m_animationData;
            std::mt19937 m_randomEngine;
            std::vector<int> m_meshIndices;
            static const std::vector<std::string> m_meshNames;
            unsigned m_numShapes = 0;

            std::map< std::string, MeshVertexIdData> m_rigMeshOutputVertexIdData;

            PoseBasedSolverTrainingDataParameters m_poseSolverTrainingDataParameters;

            // rig logic
            std::shared_ptr<const Rig<float>> m_rig;
            DeformationModelRigLogic<float> m_deformationModelRigLogic;

        };
    }
}

