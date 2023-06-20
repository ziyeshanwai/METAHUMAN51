// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/rig/Joint.h>
#include <nls/rig/JointRig.h>
#include <carbon/utils/Profiler.h>

#include <vector>


namespace epic::nls {

/**
 * A JointRig represents a hierarchy of joints and the set of influence weights on how each joint influences geometry.
*/
template <class T>
class JointRig2
{
public:
	JointRig2() = default;
    ~JointRig2() = default;
    JointRig2(JointRig2&&) = default;
    JointRig2& operator=(JointRig2&&) = default;
    JointRig2(JointRig2&) = default;
    JointRig2& operator=(const JointRig2&) = default;

    void Clear();

    int NumJoints() const { return static_cast<int>(m_jointNames.size()); }

    //! Adds a new joint to the rig
    void AddJoint(const std::string& newJointName);

    const std::vector<std::string>& GetJointNames() const { return m_jointNames; }

    int GetJointIndex(const std::string& jointName) const;

    int GetParentIndex(int jointIndex) const { return m_jointParentIndices[jointIndex]; }

    //! Sets a joint to be the parent of another joint
    void AttachJointToParent(const std::string& childJointName, const std::string& parentJointName);

    //! Checks if @p jointName is child of parentJointName
    bool IsChildOf(const std::string& jointName, const std::string& parentJointName) const;

    //! @return the hierarchy level of the joint. 0 is a root joint, 1 is a joint one below the root, ...
    int HierarchyLevel(const std::string& jointName) const;
    int HierarchyLevel(int jointIndex) const;

    //! @return vector of which joints indices are at what level
    std::vector<std::vector<int>> GetJointsPerHierarchyLevel() const;

    /**
     * Add all joint influence weights for geometry @p geometryName.
     * @param[in] numVertices  If numVertices < 0 then the number of vertices are infered from the vertex indices in @p jointInfluenceWeights.
     */
    void AddInfluenceWeights(const std::string& geometryName, const std::map<std::string, InfluenceWeights<T>>& jointInfluenceWeights, int numVertices = -1);

    InfluenceWeights<T> GetInfluenceWeights(const std::string& jointName, const std::string& geometryName) const;

    std::vector<std::string> GetGeometryNames() const;

    //! Sanity checks for the rig
    void CheckValidity();

    //! JointRig only supports rigs with a single root joint
    void CheckSingleRoot();

    //! @return the skin weights matrix for geometry @p geometryName
    const SparseMatrix<T>& GetVertexInfluenceWeights(const std::string& geometryName) const;

    //! Remove skin weights for a geometry @p geometryName
    void RemoveVertexInfluenceWeights(const std::string& geometryName);

    //! Check if skinning weights exist for geometry @p geometryName
    bool HasVertexInfluenceWeights(const std::string& geometryName) const;

    //! Resample the vertex influence weights for geometry @p geometryName, where influence for the new vertex i will have the old weights of vertex newToOldMap[i].
    void Resample(const std::string& geometryName, const std::vector<int>& newToOldMap);

    /**
     * @brief Remove all unused joints.
     *
     * @return Maps new joint indices to previous joint indices.
     */
    std::vector<int> RemoveUnusedJoints();

private:
    //! names of the joints
    std::vector<std::string> m_jointNames;

    //! maps a joint to its parent (via indices)
    std::vector<int> m_jointParentIndices;

    //! map of geometry name to sparse matrix with rows: vertices, columns: joint index, values: skin weight
    std::map<std::string, SparseMatrix<T>> m_vertexInfluenceWeights;
};


} // namespace epic::nls
