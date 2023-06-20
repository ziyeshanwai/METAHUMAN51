// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/rig/JointRig2.h>

#include <carbon/utils/Timer.h>

namespace epic::nls {

template <class T>
void JointRig2<T>::Clear()
{
    m_jointNames.clear();
    m_jointParentIndices.clear();
    m_vertexInfluenceWeights.clear();
}

template <class T>
void JointRig2<T>::AddJoint(const std::string& newJointName)
{
    for (const std::string& jointName : m_jointNames) {
        if (jointName == newJointName) {
            CARBON_CRITICAL("joint with name \"{}\" already exists", newJointName);
        }
    }
    m_jointNames.push_back(newJointName);
    m_jointParentIndices.push_back(-1);
}

template <class T>
int JointRig2<T>::GetJointIndex(const std::string& jointName) const {
    for (int i = 0; i < int(m_jointNames.size()); ++i) {
        if (jointName == m_jointNames[i]) {
            return i;
        }
    }
    return -1;
}

template <class T>
void JointRig2<T>::AttachJointToParent(const std::string& childJointName, const std::string& parentJointName)
{
    const int childIndex = GetJointIndex(childJointName);
    const int parentIndex = GetJointIndex(parentJointName);
    if (childIndex < 0) {
        CARBON_CRITICAL("joint \"{}\" does not exist", childJointName);
    }
    if (parentIndex < 0) {
        CARBON_CRITICAL("joint \"{}\" does not exist", parentJointName);
    }
    if (m_jointParentIndices[childIndex] >= 0) {
        CARBON_CRITICAL("joint \"{}\" already has a parent", childJointName);
    }
    if (IsChildOf(parentJointName, childJointName)) {
        CARBON_CRITICAL("joint \"{}\" is a child of \"{}\"", parentJointName, childJointName);
    }
    if (childIndex == parentIndex) {
        CARBON_CRITICAL("cannot parent joint to itself");
    }
    if (childIndex < parentIndex) {
        CARBON_CRITICAL("cannot have child with lower index compared to parent");
    }
    m_jointParentIndices[childIndex] = parentIndex;
}

template <class T>
bool JointRig2<T>::IsChildOf(const std::string& jointName, const std::string& parentJointName) const
{
    const int jointIndex = GetJointIndex(jointName);
    const int parentIndex = GetJointIndex(parentJointName);
    if (jointIndex < 0) {
        CARBON_CRITICAL("joint \"{}\" does not exist", jointName);
    }
    if (parentIndex < 0) {
        CARBON_CRITICAL("joint \"{}\" does not exist", parentJointName);
    }
    int currentIndex = jointIndex;
    while (m_jointParentIndices[currentIndex] >= 0) {
        if (m_jointParentIndices[currentIndex] == parentIndex) return true;
        currentIndex = m_jointParentIndices[currentIndex];
    }
    return false;
}

template <class T>
int JointRig2<T>::HierarchyLevel(const std::string& jointName) const
{
    const int jointIndex = GetJointIndex(jointName);
    if (jointIndex < 0) {
        CARBON_CRITICAL("joint \"{}\" does not exist", jointName);
    }
    return HierarchyLevel(jointIndex);
}

template <class T>
int JointRig2<T>::HierarchyLevel(int jointIndex) const
{
    if (jointIndex < 0 || jointIndex >= NumJoints()) {
        CARBON_CRITICAL("invalid joint index {}", jointIndex);
    }
    int level = 0;
    int currentIndex = jointIndex;
    while (m_jointParentIndices[currentIndex] >= 0) {
        level++;
        currentIndex = m_jointParentIndices[currentIndex];
    }
    return level;
}

template <class T>
std::vector<std::vector<int>> JointRig2<T>::GetJointsPerHierarchyLevel() const
{
    int maxLevel = 0;
    std::vector<std::vector<int>> jointsPerHierarchyLevel(NumJoints());
    for (int jointIndex = 0; jointIndex < NumJoints(); ++jointIndex) {
        const int hierarchyLevel = HierarchyLevel(jointIndex);
        jointsPerHierarchyLevel[hierarchyLevel].push_back(jointIndex);
        maxLevel = std::max<int>(hierarchyLevel, maxLevel);
    }
    jointsPerHierarchyLevel.resize(maxLevel + 1);
    return jointsPerHierarchyLevel;
}

template <class T>
void JointRig2<T>::AddInfluenceWeights(const std::string& geometryName, const std::map<std::string, InfluenceWeights<T>>& jointInfluenceWeights, int numVertices)
{
    if (m_vertexInfluenceWeights.find(geometryName) != m_vertexInfluenceWeights.end()) {
        CARBON_CRITICAL("influence weights for geometry \"{}\" have already been defined", geometryName);
    }

    for (const auto& [jointName, _] : jointInfluenceWeights) {
        if (GetJointIndex(jointName) < 0) {
            CARBON_CRITICAL("no joint {} in joint rig", jointName);
        }
    }

    int maxInfluenceVertex = std::max<int>(numVertices - 1, -1);
    if (numVertices <= 0) {
        for (auto&& [_, influenceWeights] : jointInfluenceWeights) {
            maxInfluenceVertex = std::max<int>(maxInfluenceVertex, influenceWeights.indices.maxCoeff());
        }
    }

    // record if all vertices are influenced
    std::vector<T> totalVertexInfluence(maxInfluenceVertex + 1, 0);
    for (const auto& [_, influenceWeights] : jointInfluenceWeights) {
        for (int k = 0; k < influenceWeights.indices.size(); ++k) {
            totalVertexInfluence[influenceWeights.indices[k]] += influenceWeights.weights[k];
        }
    }
    int numVerticesNotFullyInfluenced = 0;
    std::vector<int> verticesToConnectToRoot;
    for (int vID = 0; vID < static_cast<int>(totalVertexInfluence.size()); ++vID) {
        if (totalVertexInfluence[vID] == T(0)) {
            LOG_WARNING("Vertex {} is not influenced by any rig joint. Connecting vertex to the root.", vID);
            verticesToConnectToRoot.push_back(vID);
        } else if (fabs(totalVertexInfluence[vID] - T(1)) > T(1e-3)) {
            LOG_WARNING("Vertex {} is not fully influenced, total weight is {}.", vID, totalVertexInfluence[vID]);
            numVerticesNotFullyInfluenced++;
        }
    }
    if (numVerticesNotFullyInfluenced > 0) {
        LOG_WARNING("Geometry \"{}\" has {} out of {} vertices where the total weights of all joints do not add up to 1.0", geometryName, numVerticesNotFullyInfluenced, numVertices);
    }
    if (verticesToConnectToRoot.size() > 0) {
        LOG_WARNING("Geometry \"{}\" has {} vertices that are not influenced by any rig joint and are therefore connected to the root.", geometryName, verticesToConnectToRoot.size());
    }

    SparseMatrix<T> mat(NumJoints(), maxInfluenceVertex + 1);
    for (int jointIndex = 0; jointIndex < NumJoints(); jointIndex++) {
        mat.startVec(jointIndex);
        auto it = jointInfluenceWeights.find(m_jointNames[jointIndex]);
        if (it != jointInfluenceWeights.end()) {
            for (int k = 0; k < int(it->second.indices.size()); k++) {
                mat.insertBackByOuterInnerUnordered(jointIndex, it->second.indices[k]) = it->second.weights[k];
            }
        }
        if (m_jointParentIndices[jointIndex] < 0) {
            for (int vID : verticesToConnectToRoot) {
                mat.insertBackByOuterInnerUnordered(jointIndex, vID) = T(1);
            }
        }
    }
    mat.finalize();
    // transpose does also an ordering
    m_vertexInfluenceWeights[geometryName] = mat.transpose();
}

template <class T>
InfluenceWeights<T> JointRig2<T>::GetInfluenceWeights(const std::string& jointName, const std::string& geometryName) const
{
    if (m_vertexInfluenceWeights.find(geometryName) == m_vertexInfluenceWeights.end()) {
        CARBON_CRITICAL("no geometry {} is influenced by the jointrig", geometryName);
    }

    const int jointIndex = GetJointIndex(jointName);
    if (jointIndex < 0) {
        CARBON_CRITICAL("no joint with name {}", jointName);
    }

    // assemble the weights for the joint
    std::vector<std::pair<int, T>> weights;
    const SparseMatrix<T>& skinningWeights = m_vertexInfluenceWeights.find(geometryName)->second;
    for (int vID = 0; vID < int(skinningWeights.rows()); vID++) {
        for (typename SparseMatrix<T>::InnerIterator it(skinningWeights, vID); it; ++it) {
            if (jointIndex == it.col()) {
                weights.push_back(std::pair<int, T>(vID, it.value()));
            }
        }
    }

    InfluenceWeights<T> influenceWeights;
    influenceWeights.indices.resize(weights.size());
    influenceWeights.weights.resize(weights.size());
    for (int k = 0; k < int(weights.size()); k++) {
        influenceWeights.indices[k] = weights[k].first;
        influenceWeights.weights[k] = weights[k].second;
    }

    return influenceWeights;
}

template <class T>
bool JointRig2<T>::HasVertexInfluenceWeights(const std::string& geometryName) const
{
    return (m_vertexInfluenceWeights.find(geometryName) != m_vertexInfluenceWeights.end());
}

template <class T>
void JointRig2<T>::RemoveVertexInfluenceWeights(const std::string& geometryName)
{
    auto it = m_vertexInfluenceWeights.find(geometryName);
    if (it != m_vertexInfluenceWeights.end()) {
        m_vertexInfluenceWeights.erase(it);
    }
}

template <class T>
std::vector<std::string> JointRig2<T>::GetGeometryNames() const
{
    std::vector<std::string> names;
    for (auto&& [name,_] : m_vertexInfluenceWeights) {
        names.push_back(name);
    }
    return names;
}

template <class T>
void JointRig2<T>::CheckValidity()
{
    CheckSingleRoot();
}

template <class T>
void JointRig2<T>::CheckSingleRoot()
{
    int numRootJoints = 0;
    for (int parentIndex : m_jointParentIndices) {
        if (parentIndex < 0) {
            numRootJoints++;
        }
    }
    if (numRootJoints != 1) {
        CARBON_CRITICAL("The rig containts {} root joints, but only a single one is supported", numRootJoints);
    }
}

template <class T>
const SparseMatrix<T>& JointRig2<T>::GetVertexInfluenceWeights(const std::string& geometryName) const
{
    auto it = m_vertexInfluenceWeights.find(geometryName);
    if (it != m_vertexInfluenceWeights.end()) {
        return it->second;
    }
    CARBON_CRITICAL("no geometry skin cluster for geometry \"{}\"", geometryName);
}

template <class T>
void JointRig2<T>::Resample(const std::string& geometryName, const std::vector<int>& newToOldMap)
{
    auto it = m_vertexInfluenceWeights.find(geometryName);
    if (it != m_vertexInfluenceWeights.end()) {
        const SparseMatrix<T>& oldInfluenceVertices = it->second;
        SparseMatrix<T> newInfluenceVertices(newToOldMap.size(), oldInfluenceVertices.cols());
        for (size_t i = 0; i < newToOldMap.size(); ++i) {
            newInfluenceVertices.row(i) = oldInfluenceVertices.row(newToOldMap[i]);
        }
        newInfluenceVertices.makeCompressed();
        it->second = newInfluenceVertices;
    }
}

template <class T>
std::vector<int> JointRig2<T>::RemoveUnusedJoints()
{
    // check which joints are influenced by any geometry
    Eigen::VectorX<T> totalInfluenceWeightsPerJoints = Eigen::VectorX<T>::Zero(NumJoints());
    for (const auto& [_, skinningWeights] : m_vertexInfluenceWeights) {
        for (int vID = 0; vID < int(skinningWeights.rows()); vID++) {
            for (typename SparseMatrix<T>::InnerIterator it(skinningWeights, vID); it; ++it) {
                totalInfluenceWeightsPerJoints[it.col()] += it.value();
            }
        }
    }
    std::vector<bool> used(NumJoints(), false);
    for (int i = 0; i < NumJoints(); ++i) {
        used[i] = (totalInfluenceWeightsPerJoints[i] > 0);
        if (used[i]) {
            LOG_VERBOSE("joint {} is used", m_jointNames[i]);
        }
    }
    for (int i = 0; i < NumJoints(); ++i) {
        if (totalInfluenceWeightsPerJoints[i] > 0) {
            int currIdx = i;
            while (currIdx >= 0) {
                if (!used[currIdx]) {
                    used[currIdx] = true;
                    LOG_VERBOSE("joint {} is used as parent", m_jointNames[currIdx]);
                }
                currIdx = m_jointParentIndices[currIdx];
            }
        }
    }

    // remove all unused joints (update skinning weights)
    std::vector<int> oldToNew(NumJoints(), -1);
    std::vector<int> newToOld;
    for (int i = 0; i < NumJoints(); ++i) {
        if (used[i]) {
            oldToNew[i] = int(newToOld.size());
            newToOld.push_back(i);
        }
    }
    const int numNewJoints = int(newToOld.size());

    // update names
    std::vector<std::string> jointNames(numNewJoints);
    for (int i = 0; i < numNewJoints; ++i) {
        jointNames[i] = m_jointNames[newToOld[i]];
    }
    m_jointNames = jointNames;

    // update parents
    std::vector<int> jointParentIndices(numNewJoints, -1);
    for (int i = 0; i < numNewJoints; ++i) {
        const int oldIndex = newToOld[i];
        const int oldParentIndex = m_jointParentIndices[oldIndex];
        if (oldParentIndex >= 0) {
            const int newParentIndex = oldToNew[oldParentIndex];
            jointParentIndices[i] = newParentIndex;
        }
    }
    m_jointParentIndices = jointParentIndices;

    // update skinning weights
    for (auto& [_, skinningWeights] : m_vertexInfluenceWeights) {
        std::vector<Eigen::Triplet<T>> triplets;
        for (int vID = 0; vID < int(skinningWeights.rows()); vID++) {
            for (typename SparseMatrix<T>::InnerIterator it(skinningWeights, vID); it; ++it) {
                const int newIdx = oldToNew[int(it.col())];
                if (newIdx >= 0) {
                    triplets.push_back(Eigen::Triplet<T>(vID, newIdx, it.value()));
                }
            }
        }
        skinningWeights.resize(skinningWeights.rows(), numNewJoints);
        skinningWeights.setFromTriplets(triplets.begin(), triplets.end());
    }

    return newToOld;
}

// explicitly instantiate the JointRig2 classes
template class JointRig2<float>;
template class JointRig2<double>;

} // namespace epic::nls
