// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/Context.h>
#include <nls/geometry/AffineVariable.h>
#include <nls/geometry/QuaternionVariable.h>
#include <nls/rig/Joint.h>

#include <map>


namespace epic {
namespace nls {

/**
 * A 3D JointRig with variable local matrices that can be optimized
*/
template <class T>
class JointRigVariable : public JointRig<DiffDataAffine<T, 3, 3>>
{
    using JointPtr = typename JointRig<DiffDataAffine<T, 3, 3>>::JointPtr;
public:
    virtual ~JointRigVariable() {}

    virtual void Clear() override
    {
        JointRig<DiffDataAffine<T, 3, 3>>::Clear();
        m_variables.clear();
    }

    //! Create all variables associated to the joints
    void CreateVariables()
    {
        m_variables.clear();
        for (auto&& [jointName, jointPtr] : this->GetJoints()) {
            // create the variable and set the value to the current state of the joints
            m_variables[jointName] = std::make_shared<AffineVariable<QuaternionVariable<T>>>();
            m_variables[jointName]->SetAffine(Affine<T, 3, 3>(jointPtr->LocalMatrix().Matrix()));
        }
    }

    /**
     * Evaluates the joint variables and sets the local affine transformation matrix of the rig including the Jacobians.
     */
    void EvaluateVariables(Context<T>* context)
    {
        for (auto&& [jointName, varPtr] : m_variables) {
            JointPtr jointPtr = this->GetJoint(jointName);
            CARBON_ASSERT(jointPtr, "joint ptr need to be valid");
            jointPtr->SetLocalMatrix(varPtr->EvaluateAffine(context));
        }
    }

    //! Set the joint state and update the variables
    virtual void SetState(const std::map<std::string, Affine<T,3,3>>& state) override
    {
        JointRig<DiffDataAffine<T, 3, 3>>::SetState(state);

        // update the variables that already exist (may not be available if CreateVariables had not been called yet)
        for (auto&& [jointName, jointPtr] : this->GetJoints()) {
            if (m_variables.find(jointName) != m_variables.end()) {
                m_variables[jointName]->SetAffine(Affine<T, 3, 3>(jointPtr->LocalMatrix().Matrix()));
            }
        }
    }

    /**
     * Sets the degrees of freedom of a joint
     * @param rotation     Whether rotation is a degree of freedom (3 DOFs really)
     * @param translation  Whether translation is a degree of freedom (3 DOFs really)
     */
    void SetDegreesOfFreedom(const std::string& jointName, bool rotation, bool translation)
    {
        auto iter = m_variables.find(jointName);
        if (iter == m_variables.end()) {
            throw std::runtime_error("joint rig does not contain joint " + jointName);
        }
        iter->second->MakeConstant(!rotation, !translation);
    }

private:
    //! the variables associated with the joints
    std::map<std::string, std::shared_ptr<AffineVariable<QuaternionVariable<T>>>> m_variables;
};


} // namespace nls
} //namespace epic
