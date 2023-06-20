// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include <tracking/rt/HeadState.h>
#include <tracking/rt/PCARig.h>
#include <tracking/rt/PCAVertexRig.h>
#include <nls/rig/Rig.h>
#include <riglogic/RigLogic.h>
#include <set>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4324)
#endif

using namespace epic::nls;

namespace epic::nls::rt {

    //! Class to create a PCA rig from a DNA rig.
    class PCARigCreator
    {
    public:
        PCARigCreator(const std::string& dnaFilename, const std::string& configFilename);
        PCARigCreator(std::shared_ptr<const Rig<float>> rig);

        bool LoadConfigFile(const std::string& configFilename);

        /**
         * @brief Create the PCA model for the input dna and the current config.
         *
         * @param maxModes  The maximum number of modes to be used for the PCA model. If negative, then the
         *                  maximum number of modes is as defined in config file (and 100 if not defined in the config).
         * @return True is PCA model was created, false otherwise.
         */
        bool Create(int maxModes = -1);

        int RawControlIndex(const std::string& rawControlName) const;

        //! @returns the default raw controls
        Eigen::VectorXf DefaultRawControls() const;

        //! @returns all expressions for the face
        std::vector<Eigen::VectorXf> AllFaceExpressions() const;

        //! @returns all expressions for the neck
        std::vector<Eigen::VectorXf> AllNeckExpressions() const;

        //! evaluate the face on the rig
        rt::HeadVertexState<float> EvaluateExpression(const Eigen::VectorXf& rawControls) const;

        void SavePCAVertexRigAsJson(const std::string& filename) const;

        const std::shared_ptr<const Rig<float>>& GetRig() { return m_rig; }
        const rt::PCARig& GetPCARig() const { return m_pcaRig; }
        // const rt::PCAVertexRig& GetPCARig() const { return m_pcaVertexRig; }

    private:

        std::map<std::string, float> m_rawControlDefaults;
        std::set<std::string> m_fixedControls;
        std::set<std::string> m_neckControls;
        std::shared_ptr<const Rig<float>> m_rig;
        std::vector<int> m_offsetsPerModel;
        rt::PCARig m_pcaRig;
        rt::RigidBody<float> m_eyeLeftBody;
        rt::RigidBody<float> m_eyeRightBody;
        rt::PCAVertexRig m_pcaVertexRig;

        std::vector<int> m_meshIndices;
        int m_headMeshIndex;
        int m_teethMeshIndex;
        int m_eyeLeftMeshIndex;
        int m_eyeRightMeshIndex;

        int m_defaultMaxModes = 100;
    };

}

#ifdef _MSC_VER
#pragma warning(pop)
#endif
