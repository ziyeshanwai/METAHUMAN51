// Copyright Epic Games, Inc. All Rights Reserved.


#pragma once

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>

#include <nls/math/Math.h>

#include <map>
#include <string>

namespace epic::nls {

class FaceMeshSolve {
    public:
        FaceMeshSolve();
        ~FaceMeshSolve();

        //! initialize the solver with the dna file, solver defintion, template description as defined in a tracking description
        bool Init(const std::string& templateDescriptionFilename, const std::string& solveDefinitionFilename, const std::string& dnaFilename);

        //! @returns the number of solver control sets. the last one is solving against all controls.
        size_t NumSolveControlSets() const;

        //! Solve the rig against the target
        Eigen::VectorXf Solve(const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& vertices,
                              const Eigen::VectorXf& prior,
                              size_t solveControlSetIndex,
                              int iterations = 10,
                              float l1Regularization = 0.1f,
                              float wPrior = 10.0f,
                              float wPoint2Point = 1.0f) const;

        //! Solve the rig against multiple target meshes
        Eigen::VectorXf SolveMultiple(const std::map<int, Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>& vertices,
                                      const Eigen::VectorXf& prior,
                                      size_t solveControlSetIndex,
                                      int iterations = 10,
                                      float l1Regularization = 0.1f,
                                      float wPrior = 10.0f,
                                      float wPoint2Point = 1.0f) const;

        //! Evaluate the rig
        Eigen::Matrix<float, -1, 3, Eigen::RowMajor> EvaluateRig(const Eigen::VectorXf& guiControls, int meshIndex = 0) const;

        //! @return the number of gui controls
        int NumGuiControls() const;

        //! Set the map from gui control names to gui control index
        std::map<std::string, int> GuiControlNameToIndexMap() const;

        //! @return the index of the head mesh
        int HeadMeshIndex() const;

        //! @return the index of the teeth mesh
        int TeethMeshIndex() const;

        //! @return the index of the left eye mesh
        int EyeLeftMeshIndex() const;

        //! @return the index of the right eye mesh
        int EyeRightMeshIndex() const;

    private:
        struct Private;
        std::unique_ptr<Private> m;
};

} // namespace epic::nls
