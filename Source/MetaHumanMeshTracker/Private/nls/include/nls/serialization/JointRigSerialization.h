// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/io/JsonIO.h>

#include <nls/rig/JointRig.h>
#include <nls/serialization/AffineSerialization.h>

#include <carbon/utils/Timer.h>

namespace epic {
namespace nls {

/**
 * Deserializes the joint rig from json format.
 *
 * TODO: open questions: how to handle the case when the root node itself has a parent with
 *       a global transformation. For now we ignore the case and throw an exception if the local and bind pose of the root node do not match.
 *
 * Format for joints:
 * joints {
 *  “name of joint” : {
 *      “parent” : name of parent joint (not available means root node)
 *      “local”: 4x4 matrix (column major, pre multiply - should be possible to calculate it from the parent)
 *      “world”: 4x4 matrix (column major, pre multiply - this should be the bind pose)
 *      “influence”: {
 *          “name of geometry”: {
 *              “vertex indices”: []
 *              “vertex weights”: []
 *          }
 *      }
 *    },
 *    ... (next joint)
 * }
 */
template <class AffineType>
void JointRigFromJson(const carbon::JsonElement& j, JointRig<AffineType>& jointRig) {

    CARBON_PRECONDITION(j.IsObject(), "json object needs to be a dictionary");
    CARBON_PRECONDITION(j.Contains("joints"), "json objects needs to contain a \"joints\" key");
    CARBON_PRECONDITION(j["joints"].IsObject(), "json value at key \"joints\" needs to be a dictionary");

    using T = typename JointHelper<AffineType>::SCALAR;
    using Joint = epic::nls::Joint<AffineType>;
    using JointPtr = std::shared_ptr<Joint>;

    std::map<std::string, JointPtr> joints;

    // create all joints
    const carbon::JsonElement& jointsDict = j["joints"];
    for (const auto& [jointName, _] : jointsDict.Map()) {
        joints[jointName] = std::make_shared<Joint>(jointName);
    }

    // read all joints and set the local and bind matrix
    int numRootJoints = 0;
    for (const auto& [jointName, jointDict] : jointsDict.Map()) {
        CARBON_PRECONDITION(jointDict.Contains("world"), "joints dictionary needs to contain a \"world\" key");
        CARBON_PRECONDITION(jointDict.Contains("local"), "joints dictionary needs to contain a \"local\" key");
        CARBON_PRECONDITION(jointDict.Contains("influence"), "joints dictionary needs to contain a \"influence\" key");

        std::shared_ptr<Joint> joint = joints[jointName];
        joint->SetLocalMatrix(FromJson<Affine<T, 3, 3>>(jointDict["local"]));
        joint->SetBindMatrix(FromJson<Affine<T, 3, 3>>(jointDict["world"]));

        if (jointDict.Contains("parent")) {
            const std::string& parentName = jointDict["parent"].String();
            if (joints.find(parentName) == joints.end()) {
                CARBON_CRITICAL("joint {} has parent {}, but the parent is not part of the dictionary", jointName, parentName);
            }
            joints[parentName]->AddChild(joint.get());
        } else {
            // joint is a root joint
            numRootJoints++;
            if (!joint->LocalMatrix().Matrix().isApprox(joint->WorldMatrix().Matrix(), T(1e-6))) {
                CARBON_CRITICAL("we only support root joints that have a consistent Local and World matrix");
            }
        }
    }

    if (numRootJoints != 1) {
        CARBON_CRITICAL("there is more than one root joint in the rig");
    }

    // create the rig
    jointRig.Clear();
    for (const auto& nameJointPtrPair : joints) {
        jointRig.AddJoint(nameJointPtrPair.second);
    }

    Timer timer;
    // add influence weights per joint
    std::map<std::string, std::map<std::string, InfluenceWeights<T>>> allJointInfluences;
    for (const auto& [jointName, jointDict] : jointsDict.Map()) {

        const carbon::JsonElement& influencesDict = jointDict["influence"];
        CARBON_PRECONDITION(influencesDict.IsObject(), "influence json object needs to be a dictionary");
        for (const auto& [geometryName, influenceDict] : influencesDict.Map()) {
            CARBON_PRECONDITION(influenceDict.Contains("vertex indices"), "influence dictionary needs to contain a \"vertex indices\" key");
            CARBON_PRECONDITION(influenceDict.Contains("vertex weights"), "influence dictionary needs to contain a \"vertex weights\" key");
            const Eigen::VectorX<T> weights = FromJson<Eigen::VectorX<T>>(influenceDict["vertex weights"]);
            const Eigen::VectorX<int> indices = FromJson<Eigen::VectorX<int>>(influenceDict["vertex indices"]);
            allJointInfluences[geometryName][jointName] = {indices, weights};
            // jointRig.AddInfluenceWeights(jointName, geometryName, {indices, weights});
        }
    }

    for (auto && [geometryName, jointInfluences] : allJointInfluences) {
        jointRig.AddInfluenceWeights(geometryName, jointInfluences);
    }

    // make sure the joint rig is set up correctly
    jointRig.CheckValidity();
}

/**
 * Deserializes the joint state from json format i.e. it only reads the local transformation matrix per joint
 *
 * Format for joint states (note that any other entries in the dictionaries are ignored)
 * joints {
 *   “name of joint” : {
 *      “local”: 4x4 matrix (column major, pre multiply - should be possible to calculate it from the parent)
 *   },
 *   ... (next joint)
 * }
 */
template <class AffineType>
void JointRigStateFromJson(const carbon::JsonElement& j, JointRig<AffineType>& jointRig) {

    const carbon::JsonElement& jointsDict = j["joints"];

    using T = typename JointHelper<AffineType>::SCALAR;
    std::map<std::string, Affine<T,3,3>> rigState;

    for (const auto& [jointName, jointDict] : jointsDict.Map()) {
        rigState[jointName] = FromJson<Affine<T,3,3>>(jointDict["local"]);
    }

    jointRig.SetState(rigState);
}


/**
 * Serializes the joint state to json format i.e. it only writes the local transformation matrix per joint.
 * @see JointRigStateFromJson
 */
template <class AffineType>
carbon::JsonElement JointRigStateToJson(const JointRig<AffineType>& jointRig) {

    using T = typename JointHelper<AffineType>::SCALAR;
    std::map<std::string, Affine<T,3,3>> rigState = jointRig.GetState();

    carbon::JsonElement states(carbon::JsonElement::JsonType::Object);
    for (auto&& [jointName, aff] : rigState) {
        carbon::JsonElement state(carbon::JsonElement::JsonType::Object);
        state.Insert("local", ToJson2(aff));
        states.Insert(jointName, std::move(state));
    }
    carbon::JsonElement out(carbon::JsonElement::JsonType::Object);
    out.Insert("joints", std::move(states));
    return out;
}

}
}
