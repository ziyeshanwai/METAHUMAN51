// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include <rigposebasedsolver/RigGenerateTrainingData.h>
#include <carbon/common/EigenDenseBackwardsCompatible.h>
#include <memory>
#include <vector>
#include <map>

namespace cm
{
	template <class T, int D>
	class modular_solver;

	enum class alignment3d_type;
}

namespace epic
{
    namespace rigposebasedsolver
    {

		struct FramePoseData;

		/**
		 * RigPoseBasedSolver is essentially a pose-based solver (a modular_solver) which works
		 * with data generated from a 3D face rig.
		 */
		class RigPoseBasedSolver
		{
		public:

			/*
			* Default constructor
			*/
			RigPoseBasedSolver();

			/*
			* Destructor
			*/
			~RigPoseBasedSolver();

			/*
			* Move constructor
			*/
			RigPoseBasedSolver(RigPoseBasedSolver&& other);

			/*
			* Copy constructor
			*/
			RigPoseBasedSolver(const RigPoseBasedSolver& other);

			/*
			* Assignment
			*/
			RigPoseBasedSolver& operator=(const RigPoseBasedSolver& other); 

			/*
			* Move assignment
			*/
			RigPoseBasedSolver& operator=(RigPoseBasedSolver&& other);


			/*
			* Solve for a set of rig UI control values from rig vertices. Requires IsTrained() , rigVertices.size() must be >= to the maximum mesh index needed for the solver,
			* number of vertices in element of rigVertices contains the correct number of vertices for the mesh.
			* @returns a map of UI control names to solved UI control values.
			* @param[in] rigVertices: should contain the evaluated rig vertices for each mesh specified in RigGenerateTrainingData::MeshNames().
			*/
			std::map<std::string, double> SolveFrame(const std::vector<Eigen::Matrix<float, 3, -1>>& rigVertices) const;

			/**
			* @return whether the solver has been trained
			*/
			bool IsTrained() const;

			/*
				friend function for serialization
			*/
			friend void serialize(const RigPoseBasedSolver & solver, std::ostream& out);

			/*
				friend function for deserialization
			*/
			friend void deserialize(RigPoseBasedSolver & solver, std::istream& in);

			friend class RigPoseBasedSolverTrainer;

			/*
			* Returns an MD5 checksum for the solver control ranges. Requires IsTrained()
			*/
			std::string GetControlRangesMD5() const;

			/*
			* Returns an MD5 checksum for the solver control mappers. Requires IsTrained()
			*/
			std::string GetControlMappersMD5() const;


			/*
			* Returns an MD5 checksum for the solver original training params. Requires IsTrained()
			*/
			std::string GetOrigTrainingParamsMD5() const;

			/*
			* @returns the rig mesh vertex id data associated with the solver for converting rig vertices into a form that
			* can be used with the solver: a map of meshname to MeshVertexIdData (which contains a vector of the rig vertex indices to use and also
			* an offset to use so that these have unique names within the solver).
			*/
			const std::map< std::string, RigGenerateTrainingData::MeshVertexIdData>& RigMeshVertexIdData() const;

			/*
			* @returns the control mappers for the solver which map between rig ui controls and raw controls and back
			*/
			const std::vector<ControlMapper>& ControlMappers() const;


			/*
			* @returns the shape alignment type for each shape in the solver
			*/
			std::vector<cm::alignment3d_type> ShapeAlignmentTypes() const;


		private:
			void SetSolverData(const cm::modular_solver<double, 3>& modularSolver, const std::map< std::string, RigGenerateTrainingData::MeshVertexIdData>& rigMeshVertexIdData,
				const std::vector<ControlMapper>& controlMappers);
			const cm::modular_solver<double, 3>& ModularSolver() const;

			class Impl;
			std::unique_ptr<Impl> m_impl;
		};
    }
}
