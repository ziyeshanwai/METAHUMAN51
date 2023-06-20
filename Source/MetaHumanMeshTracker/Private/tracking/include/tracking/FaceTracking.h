// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <rigposebasedsolver/RigPoseBasedSolver.h>
#include <rigposebasedsolver/FramePoseData.h>

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>
#include <carbon/utils/TaskThreadPool.h>
#include <nls/geometry/BarycentricCoordinates.h>
#include <nls/geometry/DepthmapData.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/MetaShapeCamera.h>
#include <nls/geometry/MultiCameraTriangulation.h>
#include <nls/geometry/Procrustes.h>
#include <nls/serialization/ObjFileFormat.h>
#include <nls/utils/ConfigurationParameter.h>

#include <reconstruction/MultiCameraSetup.h>
#include <reconstruction/ReconstructionCommon.h>

#include <nrr/landmarks/LandmarkConstraints2D.h>
#include <nrr/IdentityBlendModel.h>
#include <nrr/ICPConstraints.h>
#include <nrr/MeshLandmarks.h>
#include <nrr/deformation_models/DeformationModelRigid.h>
#include <nrr/deformation_models/DeformationModelRigLogic.h>
#include <nrr/deformation_models/DeformationModelVertex.h>

#include <nrr/DataDescription.h>
#include <nrr/EyeballConstraints.h>
#include <nrr/landmarks/LandmarkSequence.h>
#include <nrr/TemplateDescription.h>
#include <nrr/TrackingDescription.h>

#include <tracking/FlowConstraints.h>
#include <tracking/PCAFaceTracking.h>
#include <tracking/TrackingRig.h>
#include <tracking/TrackingRigState.h>
#include <tracking/VisibilityCalculation.h>
#include <tracking/rt/PCARig.h>
#include <tracking/rt/PCAVertexRig.h>

namespace epic::rigposebasedsolver
{
    struct RigPoseBasedSolverData;
}

namespace epic::nls {

/**
 * Mesh flow data represents the flow from a mesh from a src camera image to
 * a target camera image. For the source camera we assume the mesh vertices are known including a visibility weight per vertex.
 * The targetCamera and targetPixels represent where a vertex that is projected into the source camera, should be located in the target camera.
 * The targetPixels have been calculated from a flow field (that is not explicitly stored here).
 */
struct MeshFlowData {
    Camera<float> m_targetCamera;
    Eigen::Matrix<float, 2, -1> m_targetPixels;
    Eigen::VectorXi m_targetVertexIDs;
    Eigen::VectorXf m_confidences;
};

//! Debug Info for current state
template <class T>
struct ConstraintDebugInfo {
    typename MeshCorrespondenceSearch<T>::Result correspondences;
    LandmarkConstraintsData<T> landmarkConstraints;
    LandmarkConstraintsData<T> curveConstraints;
    LandmarkConstraintsData<T> contourConstraints;
    LandmarkConstraintsData<T> lipConstraintsUpper;
    LandmarkConstraintsData<T> lipConstraintsLower;
    LandmarkConstraintsData<T> eyeLeftCurveConstraints;
    LandmarkConstraintsData<T> eyeRightCurveConstraints;
    LandmarkConstraintsData<T> teethLandmarkConstraints;
    Eigen::VectorX<T> projectiveStrain;
    Eigen::VectorX<T> greenStrain;
    Eigen::VectorX<T> quadraticBending;
    Eigen::VectorX<T> dihedralBending;
    // constraints per camera
    std::map<std::string, FlowConstraintsData<float>> temporalFlowConstraints;
    std::map<std::string, FlowConstraintsData<float>> modelFlowConstraints;
    std::map<std::string, FlowConstraintsData<float>> uvFlowConstraints;
};

/**
 * The available modes for tracking.
 */
enum class FaceTrackingFittingMode {
    COARSE,
    RIGID,
    PCA,
    POSEBASEDSOLVE,
    RIGLOGIC,
    FINE
};

const std::vector<std::string>& FaceTrackingFittingModeNames();

/**
 * The available modes which to output the vertices/mesh.
 */
enum class FaceTrackingOutputMode {
    NEUTRAL,  //!< output the neutral mesh
    RIGLOGIC, //!< output the evaluated rig mesh
    FINE,     //!< output the full fitted mesh
    POST,     //!< output a post-processed fitted mesh
    BEST      //!< the best fit i.e. the FINE fit, though can change in the future if we add additional modes
};

//! the frame states
struct FaceTrackingState {
    Affine<float, 3, 3> m_mesh2scan;                //!< the alginment from the mesh to the scan
    Eigen::VectorX<float> m_guiControls;            //!< the gui controls of the rig
    Eigen::Matrix<float, 3, -1> m_rigVertices;      //!< the evaluate rig vertices
    Eigen::Matrix<float, 3, -1> m_vertexOffsets;    //!< additional per vertex offsets
    Eigen::Matrix<float, 3, -1> m_vertices;         //!< the evaluated vertices (can be calculated from the data below)
    std::map<std::string, Eigen::Matrix<float, 3, -1>> m_deformedAssets; //!< additional deformed assets
    Eigen::VectorX<float> m_pcaCoeffs;              //!< the pca coeffs of the rig

    std::shared_ptr<const Eigen::VectorX<float>> m_postProcessedGuiControls; //!< post processed gui controls i.e. output of pose-based solve
};

bool SaveFaceTrackingStates(const std::string& filename, const std::map<int, std::shared_ptr<const FaceTrackingState>>& faceTrackingStates);

bool LoadFaceTrackingStates(const std::string& filename, std::map<int, std::shared_ptr<FaceTrackingState>>& faceTrackingStates, int& version);

//! @return the frame state pose, rig, and pca data as json format
epic::carbon::JsonElement FaceTrackingStateToJson(const FaceTrackingState& state, const TrackingRig<float>& trackingRig);

//! @return calculates a new face tracking state from the json data that contains head pose, rig controls, and pca coefficients
std::shared_ptr<FaceTrackingState> FaceTrackingStateFromJson(const epic::carbon::JsonElement& data, const TrackingRig<float>& trackingRig);

CARBON_SUPRESS_MS_WARNING(4324)
class FaceTracking {

    public:
        static constexpr const char* ConfigNameGeneral = "General";
        static constexpr const char* ConfigNameIcp = ICPConstraints<float>::ConfigName();
        static constexpr const char* ConfigNameLandmarks = LandmarkConstraintsBase<float>::ConfigName();
        static constexpr const char* ConfigNameFlow = "Flow Constraints Configuration";
        static constexpr const char* ConfigNameRigid = DeformationModelRigid<float>::ConfigName();
        static constexpr const char* ConfigNameRigLogic = DeformationModelRigLogic<float>::ConfigName();
        static constexpr const char* ConfigNamePerVertex = DeformationModelVertex<float>::ConfigName();

    public:
        ~FaceTracking();
        FaceTracking();

        bool Init(const TemplateDescription& templateDescription,
                  const std::string& configurationPath,
                  const std::string& controlsConfigurationPath);

        bool LoadTrackingRig(const std::string& dnaFilename, const std::string& solveDefinitionFilename);
        bool LoadTrackingRig(dna::StreamReader* dnaStream, const std::string& solveDefinitionFilename);

        /**
        * Load the PCA rig from (DNA) file pcaFilename
        * @pre must have loaded the tracking rig before loading the PCA rig
        */
        bool LoadPCARig(const std::string& pcaFilename);

        /**
        * Load the PCA rig from (DNA) streamreader dnaStream
        * @pre must have loaded the tracking rig before loading the PCA rig
        */
        bool LoadPCARig(dna::StreamReader* dnaStream);
        bool SavePCARig(dna::StreamWriter* dnaStream) const;
        void SavePCARigAsNpy(const std::string& filename) const;
        bool LoadPoseBasedSolversData(const std::vector<std::string>& poseBasedSolverFilenames);

        void SetCameraSetup(const MultiCameraSetup<float>& cameraSetup);

        void SetFaceTrackingState(const FaceTrackingState& state);
        const FaceTrackingState& GetFaceTrackingState() const { return m_currentState; }

        void SetPreviousTrackingStates(const std::vector<std::shared_ptr<const FaceTrackingState>>& previousStates) { m_previousStates = previousStates; }

        //! Resets all input data.
        void ResetInputData();

        /**
         * Set the landmark data.
         * @pre Assumes the landmarks are already undistorted.
         */
        void SetLandmarks(const std::map<std::string, std::shared_ptr<const LandmarkInstance<float, 2>>>& landmarkData);

        /**
         * Set the current scan. The scan needs to be in the scale and coordinate frame of the camera setup. If
         * the cameras are scaled e.g. using ScaleCamerasUsingFitting() then it is the callers responsibility to
         * set data of the correct scale.
         */
        void SetScan(std::shared_ptr<const Mesh<float>> scan);

        /**
         * Set the current depthap. The scan needs to be in the scale and coordinate frame of the camera setup. If
         * the cameras are scaled e.g. using ScaleCamerasUsingFitting() then it is the callers responsibility to
         * set data of the correct scale.
         * @param[in] depthmapData  The depthmap data that should be used for the face tracking.
         */
        void SetDepthmaps(const std::vector<std::shared_ptr<const DepthmapData<float>>>& depthmapData);

        void SetTemporalFlow(const std::map<std::string, std::shared_ptr<const MeshFlowData>>& flowData);
        void SetModelBaseFlow(const std::map<std::string, std::shared_ptr<const MeshFlowData>>& flowData);
        void SetUVFlow(const std::map<std::string, std::shared_ptr<const MeshFlowData>>& flowData);

        //! @returns the camera setup
        const MultiCameraSetup<float>& GetCameraSetup() const { return m_cameraSetup; }

        /**
         * @returns the triangulated landmarks based on the currently set landmarks and scan. If there
         *          are two or more cameras with landmarks, then the landmarks are triangulated by multi camera triangulation.
         *          If there is only one camera and a scan then the 3D location of the landmarks are estimated
         *          by ray-mesh intersection between landmarks and scan.
         * @pre landmarks and scan (if only one camera with landmarks) need to be set using SetLandmarks() and SetScan()
         */
        const std::map<std::string, Eigen::Vector3f>& TriangulatedLandmarks();

        /**
         * Estimates the camera ranges by projecting the current mesh into each camera
         * @param[in] rangeScaling  An additional scaling factor to increase the range above the distance that is measured from the projected mesh.
         */
        void EstimateCameraRangesUsingCurrentMesh(float rangeScaling = 1.5f);

        // ! Estimate the rigid transformation of the current mesh into the current images using triangulated landmarks only
        void EstimateRigidUsingLandmarks();

        /**
         * Scales the cameras by estimating the scale using the transformation between the mesh and the triangulated landmarks.
         * @pre Uses triangualted landmarks, so landmarks and scan (if only one camera with landmarks) need to be set using SetLandmarks() and SetScan()
         * @post The camera setup is changed and therefore all currently set input data is reset.
         */
        void ScaleCamerasUsingLandmarks();

        /**
         * Scales the cameras by jointly estimating the rigid alignment and scale of the mesh against the scan.
         * @pre Scan or depthmap data is used for the fit, so data needs to be set before using SetScan() or SetDepthmap().
         * @post The camera setup is changed and therefore all currently set input data is reset.
         */
        void ScaleCamerasUsingFitting();

        /**
         * Scales the cameras.
         * @post The camera setup is changed and therefore all currently set input data is reset.
         */
        void ScaleCameras(float scale);

        // ! @return the head vertices
        Eigen::Matrix<float, 3, -1> HeadVertices(const FaceTrackingState& state, FaceTrackingOutputMode outputMode) const;

        // ! @return the head vertices
        Eigen::Matrix<float, 3, -1> CurrentHeadVertices(FaceTrackingOutputMode outputMode) const;

        //! @returns the current vertices for mesh @p meshIndex.
        Eigen::Matrix<float, 3, -1> CurrentMeshVertices(FaceTrackingOutputMode outputMode, int meshIndex) const;

        //! @returns the vertices for mesh @p meshIndex and state @p state.
        Eigen::Matrix<float, 3, -1> MeshVertices(const FaceTrackingState& state, FaceTrackingOutputMode outputMode, int meshIndex) const;

        // ! @return the fitted mesh
        std::shared_ptr<const Mesh<float> > CurrentMesh(FaceTrackingOutputMode outputMode = FaceTrackingOutputMode::FINE);

        // ! @return the alignment of the mesh to the camera coordinate system
        const Affine<float, 3, 3>& CurrentRigid() const;

        //! @returns the template description
        const TemplateDescription& GetTemplateDescription() const { return m_templateDescription; }

        //! @return debug information to visualize the data constraints such as ICP and landmarks
        std::shared_ptr<const ConstraintDebugInfo<float>> CurrentDebugConstraints(FaceTrackingOutputMode outputMode = FaceTrackingOutputMode::FINE);

        /**
         * Fit the model to the input data using the selected fitting mode
         * @param[in] solveControlSetIndex  select the control set to optimize (only applicable for valid FaceTrackingFittingMode::)
         */
        void FitRigLogic(FaceTrackingFittingMode fittingMode, size_t solveControlSetIndex = 0);

        void FitPCA();

        /*
        * Predict the rig-control values using the pose-based solver(s). Requires HasPoseBasedSolverModel() to be true
        */
        void PerformPoseBasedSolve();

        //! @returns the mesh landmarks. they may be extend by the brow landmarks once InitializeBrows() is called
        const MeshLandmarks<float>& GetMeshLandmarks() const { return m_meshLandmarks; }

        //! Updates the mesh landmarks.
        void SetMeshLandmarks(const MeshLandmarks<float>& meshLandmarks);

        //! Return the configuration settings for the fitting mode
        const Configuration& GetFittingConfiguration(FaceTrackingFittingMode fittingMode) const;

        //! Set the configuration settings for the fitting mode
        void SetFittingConfiguration(FaceTrackingFittingMode fittingMode, const Configuration& fittingConfiguration);

        //! @returns whether a tracking rig has been set
        bool HasTrackingRig() const { return m_trackingRig.GetBaseMesh().NumVertices() > 0; }

        //! @returns the TrackingRig
        const TrackingRig<float>& GetTrackingRig() const { return m_trackingRig; }

        //! @returns the TrackingRigState
        const TrackingRigState<float>& GetTrackingRigState() const { return *m_trackingRigState; }

        //! @returns True if face tracking contains a PCA model.
        bool HasPCAModel() const { return (m_pcaFaceTracking.GetPCARig().NumCoeffs() > 0); }

        //! @returns True if face tracking contains a pose-based solver model.
        bool HasPoseBasedSolverModel() const;

        //! @returns the pose based solver model data
        const std::vector<rigposebasedsolver::RigPoseBasedSolverData>& GetPoseBasedSolversData() const;

        //! Set the pose based solver model data
        void SetPoseBasedSolversData(const std::vector<rigposebasedsolver::RigPoseBasedSolverData>& solverData);

        //! Set the current riglogic control values
        void SetRigSolveControlValues(size_t solveControlSetIndex, const Eigen::VectorXf& values);

        //! Set the current riglogic control values
        void SetRigGuiControlValues(const Eigen::VectorXf& values);

        /**
         * Reset the current riglogic control values.
         * @param[in] resetNextSets  If true then the controls for the next sets will also be reset.
         */
        void ResetRigSolveControlValues(size_t solveControlSetIndex, bool resetNextSets);

        //! Sets which rig controls to optimize
        void SetRigSolveControlsToOptimize(size_t solveControlSetIndex, const std::vector<bool>& controlsToOptimize);

        //! Reset the vertex offsets
        void ResetVertexOffsets();

        //! @return the number of meshes in the rig
        int NumMeshesInRig() const { return m_trackingRig.GetRigGeometry() ? m_trackingRig.GetRigGeometry()->NumMeshes() : 0; };

        //! @return the name of the mesh
        const std::string& RigMeshName(int meshIndex) const;

        //! @returns the mesh a index @p meshIndex of the current rig
        const Mesh<float>& RigMesh(int meshIndex) const;

        //! Load the tracking configuration from a json file
        void LoadConfiguration(const std::string& filename);

        //! Load the tracking configuration from json
        void LoadConfiguration(const carbon::JsonElement& jsonDict);

        //! Save the tracking configuration to a json file
        void SaveConfiguration(const std::string& filename) const;

        //! Save the tracking configuration to json
        void SaveConfiguration(carbon::JsonElement& jsonDict) const;

        //! Load the controls configuration from a json file i.e. what controls to optimize
        void LoadControlsConfiguration(const std::string& filename);

        //! Save the controls configuration to a json file i.e. what controls to optimize
        void SaveControlsConfiguration(const std::string& filename) const;

        //! Set the user defined landmark and curve weights
        void SetUserDefinedLandmarkAndCurveWeights(const std::map<std::string, float>& userDefinedLandmarkAndCurveWeights);

        //! @returns the user defined landmark and curve weights
        const std::map<std::string, float>& UserDefinedLandmarkAndCurveWeights() const { return m_userDefinedLandmarkAndCurveWeights; }

        //! @returns which curves in the landmarks (@see SetLandmarks) should be merged
        const std::map<std::string, std::vector<std::string>>& CurvesToMerge() const { return m_curvesToMerge; }

        //! @return the name of the ICP mask that should be used for @p fittingMode
        std::string GetICPMask(FaceTrackingFittingMode fittingMode) const;

        //! @return the name of the flow mask that should be used for @p fittingMode
        std::string GetFlowMask(FaceTrackingFittingMode fittingMode) const;

        //! @return the name of the uv flow mask that should be used for @p fittingMode
        std::string GetUVFlowMask(FaceTrackingFittingMode fittingMode) const;

        //! @return the loaded pca rig
        const rt::PCARig& GetPCARig() const { return m_pcaFaceTracking.GetPCARig(); }

    private:
        // ! @return the 3D positions of the landmarks
        std::map<std::string, Eigen::Vector3f> EvaluateMeshLandmarks(const Eigen::Matrix<float, 3, -1>& vertices) const;

        //! Flags that the mesh needs to be recalculated
        void UpdateCurrentMesh();

        //! Updates the solve control sets data
        void UpdateSolveControlSetsData();

        //! Resets the data constraints
        void ResetDataConstraints();

        //! Setup the data constraints if not yet set (ICP, landmarks)
        void SetupDataConstraints();

        void SetupICPDataConstraints();
        void SetupLandmarkDataConstraints();
        void SetupTemporalFlowDataConstraints();
        void SetupModelFlowDataConstraints();
        void SetupUVFlowDataConstraints();
        void SetupEyeballConstraints();

        //! Setup default weights for all landmarks and curves if not defined yet
        void SetupDefaultUserDefinedLandmarkAndCurveWeights();

        //! @returns the scan for the current frame scaled according to the current camera scale (@see ScaleCamerasUsingLandmarks())
        std::shared_ptr<const Mesh<float> > CurrentScan();

        //! Load the controls configuration from json i.e. what controls to optimize
        void LoadControlsConfiguration(const epic::carbon::JsonElement& json);

        //! Save the controls configuration to json i.e. what controls to optimize
        epic::carbon::JsonElement SaveControlsConfiguration() const;


    private:
        TemplateDescription m_templateDescription;

        MeshLandmarks<float> m_meshLandmarks;

        //! the tracking rig containing RigLogic, RigGeometry, and RigLogicSolveControls
        TrackingRig<float> m_trackingRig;

        //! pca-based face tracking
        PCAFaceTracking m_pcaFaceTracking;
        std::vector<PCAFaceTracking::State> m_pcaFaceTrackingStates;
        std::map<int, int> m_meshIndexToPcaRig;

        //! pose-based solver
        std::unique_ptr<std::vector<rigposebasedsolver::RigPoseBasedSolverData>> m_rigPoseBasedSolversData;
        std::vector<rigposebasedsolver::FramePoseData> m_RigPoseBasedSolverTrainingData;

        MultiCameraSetup<float> m_cameraSetup;

        //! deformation models
        std::unique_ptr<DeformationModelRigid<float>> m_defModelRigid;
        std::unique_ptr<DeformationModelRigLogic<float>> m_defModelRigLogic;
        std::unique_ptr<DeformationModelVertex<float>> m_defModelVertex;

        //! The data for the current frame which is used for the data constraints.
        struct FrameData {
            //! clear everything
            void Clear() {
                m_landmarkData.clear();
                m_scan.reset();
                m_depthmapData.clear();
                m_temporalFlow.clear();
                m_modelFlow.clear();
                m_uvFlow.clear();
                m_triangulatedLandmarks.clear();
            }

            std::map<std::string, std::shared_ptr<const LandmarkInstance<float, 2>>> m_landmarkData;
            std::shared_ptr<const Mesh<float> > m_scan;
            std::vector<std::shared_ptr<const DepthmapData<float>>> m_depthmapData;
            std::map<std::string, std::shared_ptr<const MeshFlowData>> m_temporalFlow;
            std::map<std::string, std::shared_ptr<const MeshFlowData>> m_modelFlow;
            std::map<std::string, std::shared_ptr<const MeshFlowData>> m_uvFlow;
            std::map<std::string, Eigen::Vector3f> m_triangulatedLandmarks;
        } m_currentFrameData;

        //! constraints
        std::unique_ptr<LandmarkConstraints2D<float>> m_landmarkConstraints;
        std::unique_ptr<ICPConstraints<float>> m_icpConstraints;
        std::unique_ptr<FlowConstraints<float>> m_temporalFlowConstraints;
        std::unique_ptr<FlowConstraints<float>> m_modelFlowConstraints;
        std::unique_ptr<FlowConstraints<float>> m_uvFlowConstraints;
        std::unique_ptr<EyeballConstraints<float>> m_leftEyeConstraints;
        std::unique_ptr<EyeballConstraints<float>> m_rightEyeConstraints;

        //! debug data to show constraints
        std::map<FaceTrackingOutputMode, std::shared_ptr<const ConstraintDebugInfo<float>>> m_constraintsDebugInfo;

        /**
         * The tracking state for the current frame including the rigid alignment, the riglogic parameters (gui controls), and
         * per-vertex offsets.
         */
        FaceTrackingState m_currentState;

        //! previous tracking states;
        std::vector<std::shared_ptr<const FaceTrackingState>> m_previousStates;

        /**
         * Additional tracking rig state containing for the TrackingRig. This includes
         * the solve controls for all solver sets and which controls to optimize.
         */
        std::unique_ptr<TrackingRigState<float>> m_trackingRigState;

        //! the current meshes (fully determined by currentState)
        std::map<FaceTrackingOutputMode, std::shared_ptr<const Mesh<float>>> m_currentMeshes;

        //! all settings for fitting
        std::map<FaceTrackingFittingMode, Configuration> m_fittingConfigurations;

        //! weights for landmarks and curves
        std::map<std::string, float> m_userDefinedLandmarkAndCurveWeights;

        //! the set of gui controls that are fixed.
        std::vector<std::string> m_fixedRigLogicGuiControls;

        //! the set of symmetric controls
        std::vector<std::tuple<std::string, std::string, float>> m_symmetricRigLogicGuiControls;

        //! the set of curves that should be merged before calling SetLandmarks
        std::map<std::string, std::vector<std::string>> m_curvesToMerge;

        //! default values for the UI controls
        Eigen::VectorXf m_defaultGuiControlValues;

        std::shared_ptr<epic::carbon::TaskThreadPool> m_globalThreadPool = epic::carbon::TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/ true);
};
CARBON_REENABLE_MS_WARNING

}
