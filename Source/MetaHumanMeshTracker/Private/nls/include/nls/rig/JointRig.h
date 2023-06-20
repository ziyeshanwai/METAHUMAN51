// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/rig/Joint.h>
#include <carbon/utils/Profiler.h>

#include <map>


namespace epic {
namespace nls {


template <class T>
struct InfluenceWeights {
public:
    //! vertex indices
    Eigen::VectorX<int> indices;
    //! vertex weights
    Eigen::VectorX<T> weights;
};


/**
 * A JointRig represents a hierarchy of joints and the set of influence weights on how each joint influences geometry.
 * The deformed geometry can be evaluated with or without Jacobians.
*/
template <class AffineType>
class JointRig
{
public:
    using T = typename JointHelper<AffineType>::SCALAR;
    using JointPtr = std::shared_ptr<Joint<AffineType>>;

public:
	JointRig() {}
    virtual ~JointRig() {}

    JointRig(JointRig&) = delete;
    JointRig& operator=(const JointRig&) = delete;

    virtual void Clear()
    {
        m_joints.clear();
        m_jointsVector.clear();
        m_vertexInfluenceWeights.clear();
    }

    //! Adds a new joint to the rig
    void AddJoint(const JointPtr& joint)
    {
        if (!joint || joint->Name().empty()) {
            CARBON_CRITICAL("invalid joint");
        }
        if (m_joints.find(joint->Name()) != m_joints.end()) {
            CARBON_CRITICAL("joint with name {} is already part of the rig", joint->Name());
        }
        m_joints[joint->Name()] = joint;
        m_jointsVector.push_back(joint);
    }

    JointPtr GetJoint(const std::string& name) const
    {
        auto it = m_joints.find(name);
        if (it != m_joints.end()) {
            return it->second;
        } else {
            return JointPtr();
        }
    }

    const std::map<std::string, JointPtr>& GetJoints() const { return m_joints; }

    std::vector<std::string> GetJointNames() const
    {
        std::vector<std::string> names;
        for (auto&& [name,_] : m_joints) {
            names.push_back(name);
        }
        return names;
    }

    /**
     * Add all joint influence weights for geometry @p geometryName.
     * @param[in] numVertices  If numVertices < 0 then the number of vertices are infered from the vertex indices in @p jointInfluenceWeights.
     */
    void AddInfluenceWeights(const std::string& geometryName, const std::map<std::string, InfluenceWeights<T>>& jointInfluenceWeights, int numVertices = -1)
    {
        if (m_vertexInfluenceWeights.find(geometryName) != m_vertexInfluenceWeights.end()) {
            CARBON_CRITICAL("influence weights for geometry \"{}\" have already been defined", geometryName);
        }
        const int numJoints = int(m_jointsVector.size());
        int maxInfluenceVertex = std::max<int>(numVertices - 1, 0);
        for (auto&& [_, influenceWeights] : jointInfluenceWeights) {
            maxInfluenceVertex = std::max<int>(maxInfluenceVertex, influenceWeights.indices.maxCoeff());
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
        for (int vID = 0; vID < int(totalVertexInfluence.size()); ++vID) {
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
            LOG_WARNING("Geometry \"{}\" has {} vertices that are not influenced by any rig joint and are therefore connected to the root.", verticesToConnectToRoot.size());
        }

        SparseMatrix<T> mat(numJoints, maxInfluenceVertex + 1);
        for (int i = 0; i < int(m_jointsVector.size()); i++) {
            mat.startVec(i);
            auto it = jointInfluenceWeights.find(m_jointsVector[i]->Name());
            if (it != jointInfluenceWeights.end()) {
                for (int k = 0; k < int(it->second.indices.size()); k++) {
                    mat.insertBackByOuterInnerUnordered(i, it->second.indices[k]) = it->second.weights[k];
                }
            }
            if (m_jointsVector[i]->IsRoot()) {
                for (int vID : verticesToConnectToRoot) {
                    mat.insertBackByOuterInnerUnordered(i, vID) = T(1);
                }
            }
        }
        mat.finalize();
        // transpose does also an ordering
        m_vertexInfluenceWeights[geometryName] = mat.transpose();
    }


    InfluenceWeights<T> GetInfluenceWeights(const std::string& jointName, const std::string& geometryName) const
    {
        if (m_vertexInfluenceWeights.find(geometryName) == m_vertexInfluenceWeights.end()) {
            CARBON_CRITICAL("no geometry {} is influenced by the jointrig", geometryName);
        }

        int jointIndex = -1;
        for (int i = 0; i < int(m_jointsVector.size()); i++) {
            if (m_jointsVector[i]->Name() == jointName) {
                jointIndex = i;
                break;
            }
        }
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

    std::vector<std::string> GetGeometryNames() const
    {
        std::vector<std::string> names;
        for (auto&& [name,_] : m_vertexInfluenceWeights) {
            names.push_back(name);
        }
        return names;
    }


    //! Returns the current state (local matrices) of all the joints
    std::map<std::string, Affine<T,3,3>> GetState() const
    {
        std::map<std::string, Affine<T,3,3>> localMattrices;
        for (auto&& [jointName, jointPtr] : m_joints) {
            localMattrices[jointName] = Affine<T, 3, 3>(jointPtr->LocalMatrix().Matrix());
        }
        return localMattrices;
    }


    //! Set the state of the joints
    virtual void SetState(const std::map<std::string, Affine<T,3,3>>& state)
    {
        for (auto&& [jointName, aff] : state) {
            if (m_joints.find(jointName) == m_joints.end()) {
                CARBON_CRITICAL("failed to find joint {}", jointName);
            }
            m_joints[jointName]->SetLocalMatrix(aff);
        }
    }


    /**
     * Basic geometry evaluation. This does not calculate any Jacobian.
     *
     * Instantiated for float and double versions of Affine<T, 3, 3> and DiffDataAffine<T, 3, 3>
     */
    Eigen::Matrix<T, 3, -1> EvaluateGeometry(const std::string& geometryName,
                                             const Eigen::Matrix<T, 3, -1>& restVertices) const;


    /**
     * Geometry evaluation with Jacobian.
     *
     * Instantiated for float and double versions of DiffDataAffine<T, 3, 3>
     */
    DiffDataMatrix<T, 3, -1> EvaluateGeometry(const std::string& geometryName,
                                              const DiffDataMatrix<T, 3, -1>& restVertices) const;

    /**
     * Geometry evaluation with a dense Jacobian as output.
     *
     * Instantiated for float and double versions of DiffDataAffine<T, 3, 3>
     */
    DiffDataMatrix<T, 3, -1> EvaluateGeometryDense(const std::string& geometryName,
                                                   const DiffDataMatrix<T, 3, -1>& restVertices) const;


    /**
     * Calculates a regularization residual of the affine transformations of the rig against the save state
     *
     * Instantiated for float and double versions of DiffDataAffine<T, 3, 3>
     */
    DiffData<T> EvaluateRegularization(const T rotationWeight,
                                       const T translationWeight,
                                       const std::map<std::string, Affine<T,3,3>>& restStates,
                                       const bool regularizeRoot = false) const;


    //! Sanity checks for the rig
    void CheckValidity()
    {
        CheckSingleRoot();
    }

    //! JointRig only supports rigs with a single root joint
    void CheckSingleRoot()
    {
        int numRootJoints = 0;
        for (auto&& [jointName, joint] : m_joints) {
            if (!joint) {
                CARBON_CRITICAL("Invalid nullptr for joint {}", jointName);
            }
            if (joint->IsRoot()) {
                numRootJoints++;
            }
        }
        if (numRootJoints != 1) {
            CARBON_CRITICAL("The rig containts {} root joints, but only a single one is supported", numRootJoints);
        }
    }

private:
    //! map of joint name to joint
    std::map<std::string, JointPtr> m_joints;

    //! vector of joints
    std::vector<JointPtr> m_jointsVector;

    //! map of geometry name to map of vertex index to list of joint indices (@see m_jointsVector) and weight
    std::map<std::string, SparseMatrix<T>> m_vertexInfluenceWeights;
};


} // namespace nls
} //namespace epic
