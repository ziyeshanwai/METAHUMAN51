// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/geometry/BarycentricCoordinates.h>

#include <map>
#include <string>

namespace epic::nls {

/**
 * Module to align a template mesh with a high resolution 3D scan.
 *
 * Implemented for T=float and T=double
 */
template <class T>
class IDFit
{
public:
    IDFit();
    ~IDFit();
    IDFit(IDFit&& other);
    IDFit(const IDFit& other) = delete;
    IDFit& operator=(IDFit&& other);
    IDFit& operator=(const IDFit& other) = delete;

    //! @returns the configuration for rigid registration
    std::map<std::string, std::string> RigidRegistrationConfiguration() const;
    //! sets the configuration for rigid registration
    void SetRigidRegistrationConfiguration(const std::map<std::string, std::string>& config);

    //! @returns the configuration for non-rigid registration (Gauss-Newton)
    std::map<std::string, std::string> NonRigidRegistrationConfigurationGN() const;
    //! sets the configuration for non-rigid registration (Gauss-Newton)
    void SetNonRigidRegistrationConfigurationGN(const std::map<std::string, std::string>& config);

    //! @returns the configuration for non-rigid registration (NNLS)
    std::map<std::string, std::string> NonRigidRegistrationConfigurationNNLS() const;
    //! sets the configuration for non-rigid registration (NNLS)
    void SetNonRigidRegistrationConfigurationNNLS(const std::map<std::string, std::string>& config);

    //! @returns the configuration for fine fitting
    std::map<std::string, std::string> FineRegistrationConfiguration() const;
    //! sets the configuration for fine fitting
    void SetFineRegistrationConfiguration(const std::map<std::string, std::string>& config);

    //! @returns the configuration for markers
    std::map<std::string, std::string> MarkerConfiguration() const;
    //! sets the configuration for markers
    void SetMarkerConfiguration(const std::map<std::string, std::string>& config);

    //! Set the source mesh.
    void SetSourceMesh(const Eigen::VectorXi& polygons, const Eigen::VectorXi& vIDs, const Eigen::Matrix<T, 3, -1>& vertices);

    //! Set the registration vertex weights
    void SetCorrespondenceSearchVertexWeights(const Eigen::VectorX<T>& vertexWeights);

    //! Set the target mesh
    void SetTargetMesh(const Eigen::VectorXi& polygons, const Eigen::VectorXi& vIDs, const Eigen::Matrix<T, 3, -1>& vertices);

    //! Set list of region names for the region model
    void SetModelRegionNames(const std::vector<std::string>& regionNames);
    //! Set region for the region model
    void SetModelRegion(const std::string& regionName, const Vector<T>& regionData);
    //! Set all regions for the region model
    void SetModelRegions(const std::map<std::string, Vector<T>>& regions);

    //! Set list of character names for the region model
    void SetModelCharacterNames(const std::vector<std::string>& charNames);
    //! Set character for the region model
    void SetModelCharacter(const std::string& charName, const Eigen::Matrix<T, 3, -1>& charData);
    //! Set all characters for the region model
    void SetModelCharacters(const std::map<std::string, Eigen::Matrix<T, 3, -1>>& characters);

    //! Set archetype for the region model
    void SetModelArchetype(const Eigen::Matrix<T, 3, -1>& archetype);

    //! Set symmetric region list for the region model
    void SetModelSymmetricRegions(const std::vector<std::pair<std::string, std::string>>& symmetricRegions);

    //! Generate region model based on the provided data
    void GenerateModel();

    //! @return the current deformed vertices
    const Eigen::Matrix<T, 3 ,-1>& CurrentDeformedVertices() const;

    //! @return the current alignmed of the deformed vertices to the target mesh (excluding the target transformation that is passed in Register...)
    const Eigen::Matrix<T, 4, 4>& CurrentAlignment() const;

    /**
     * Run rigid registration.
     * @param sourceAffine                    The (current) affine transformation of the source mesh.
     * @param targetAffine                    The target affine transformation of the target mesh.
     * @param initialCorrespondenceVertices   The vertices to use as a starting point for finding correspondences. If not set, current deformed vertices are used.
     */
    void RegisterRigid(const Eigen::Matrix<T, 4, 4>& sourceAffine,
                       const Eigen::Matrix<T, 4, 4>& targetAffine,
                       const Eigen::Matrix<T, 3, -1>& initialCorrespondenceVertices = Eigen::Matrix<T, 3, -1>());

    //! Resets the region model parameters as well as the per vertex offsets
    void ResetNonRigid();

    /**
     * Nonrigid registration using the region model (discard the per-vertex offsets) using GaussNewton solver
     */
    void RegisterNonRigidGN(const Eigen::Matrix<T, 3, -1>& initialCorrespondenceVertices = Eigen::Matrix<T, 3, -1>());
    /**
     * Nonrigid registration using the region model (discard the per-vertex offsets) using NNLS solver
     */
    void RegisterNonRigidNNLS(const Eigen::Matrix<T, 3, -1>& initialCorrespondenceVertices = Eigen::Matrix<T, 3, -1>());

    //! Resets the fine registration (per vertex offsets)
    void ResetFine();

    /**
     * Nonrigid registration with per-vertex displacement
     */
    void RegisterFine(const Eigen::Matrix<T, 3, -1>& initialCorrespondenceVertices = Eigen::Matrix<T, 3, -1>());

    //! Get current blend model parameters
    Eigen::VectorX<T> ModelParameters() const;

    //! Get per-vertex offsets produced by fine registration
    Eigen::Matrix<T, 3, -1> FineFittingDeltas() const;

    //! Number of currently completed iterations for non-rigid registration (NNLS)
    int CompletedIterationsNNLS() const;

    //! Updates affine transformation of the source mesh.
    void UpdateSourceAffine(const Eigen::Matrix<T, 4, 4>& sourceAffine);

    //! Updates affine transformation of the target mesh.
    void UpdateTargetAffine(const Eigen::Matrix<T, 4, 4>& targetAffine);

    //! Updates barycentric coordinates for marker with specified index in the source markers vector.
    void UpdateSourceMarker(int markerID, const BarycentricCoordinates<T>& coordinates);

    //! Updates position and normal for marker with specified index in the target markers vector.
    void UpdateTargetMarker(int markerID, const Eigen::Vector3<T>& position, const Eigen::Vector3<T>& normal);

    /** Sets source and target markers.
    * @param sourceMarkers    Barycentric coordinates of source markers.
    * @param targetMarkers    Positions of target markers.
    * @param targetNormals    Normals of target markers.
    * @param markerWeights    Individual weights for each marker. If not set, uses a vector of ones.
    */
    void SetMarkers(const std::vector<BarycentricCoordinates<T>>& sourceMarkers,
                    const Eigen::Matrix<T, 3, -1>& targetMarkers,
                    const Eigen::Matrix<T, 3, -1>& targetNormals,
                    const Eigen::Vector<T, -1>& markerWeights = Eigen::Vector<T, -1>());

    //! Removes all source and target markers.
    void RemoveMarkers();

    //! Updates marker weight for marker with specified index in the markers vector.
    void UpdateIndividualMarkerWeight(int markerID, T markerWeight);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
