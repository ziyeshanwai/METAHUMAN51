// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/math/Math.h>

#include <string>
#include <vector>

namespace epic::nls {

template <class T>
class JointRigOptimization
{
public:
    JointRigOptimization();
    ~JointRigOptimization();
    JointRigOptimization(JointRigOptimization&& other);
    JointRigOptimization(const JointRigOptimization& other) = delete;
    JointRigOptimization& operator=(JointRigOptimization&& other);
    JointRigOptimization& operator=(const JointRigOptimization& other) = delete;

    /**
     * Set the joint rig using a json-based rig definition.
     * @param jointRigDefinitionJson  The definition of the joint rig in json format
     */
    void SetJointRigJsonBased(const std::string& jointRigDefinitionJson);

    /**
     * Set the mesh using a json-based mesh definition.
     * @param meshDefinitionJson  The definition of the mesh in json format
     */
    void SetMeshJsonBased(const std::string& meshDefinitionJson);

    /**
     * Json-IO based joint rig optimization.
     * @param deformedGeometryJson       The definition of the deformed geometry in json format
     * @param rotationRegularization     Regularization on the delta of the joint rotation relative to the rest rig state.
     * @param translationRegularization  Regularization of the delta of the joint translation relative to the rest rig state.
     * @param strainWeight               How much to regularize strain between the evaluated vertices and the target vertices
     * @param bendingWeight              How much to regularize bending between the evaluated vertices and the target vertices
     * @param numIterations              The number of iterations for the optimization
     * @param regularizeRoot             Whether to regularize the rotation and translation of the root node. Typically false as you may want to
     *                                   allow an arbitrary global transformation.
     * @return the jointRig state in json-format that best approximated the deformed geometry
     * @see JointRigSerialization, GeometrySerialization
     */
    std::string OptimizeJointsJsonBased(const std::string& deformedGeometryJson,
                                        const Vector<T>& splitMap,
                                        T rotationRegularization,
                                        T translationRegularization,
                                        T strainWeight,
                                        T bendingWeight,
                                        int numIterations = 20,
                                        bool regularizeRoot = false);

    /**
     * @return the current jointRig state in json-format
     * @see JointRigSerialization, GeometrySerialization
     */
    std::string RigStateJsonBased();

    //! Set the jointRig state in json format
    void SetRigStateJsonBased(const std::string& rigStateJson);

    //! Restore the rig state to the rest state
    void RestoreRigState();

    //! Sets the degrees of freedom of the joint i.e. whether to optimize rotation, translation, both, or none when calling optimizeJointsJsonBased().
    void SetJointDegreesOfFreedom(const std::string& jointName, bool rotation, bool translation);

    //! @return the names of all the joints that are part of the rig
    std::vector<std::string> GetJointNames() const;

    //! @returns the names of the geometries that are influenced by the rig
    std::vector<std::string> GetGeometryNames() const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
