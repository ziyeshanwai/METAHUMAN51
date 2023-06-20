// Copyright Epic Games, Inc. All Rights Reserved.

#include <rigposebasedsolver/RigPoseBasedSolver.h>
#include <rigposebasedsolver/RigPoseBasedSolverTrainer.h>
#include <rigposebasedsolver/RigGenerateTrainingData.h>
#include <rigposebasedsolver/ControlMapper.h>
#include <map>
#include <vector>
#include <string>

namespace epic
{
    namespace rigposebasedsolver
    {

		class RigPoseBasedSolver::Impl
		{
		public:
			Impl()
			{
			}

			static const unsigned version;

			std::map<std::string, double> SolveFrame(const std::vector<Eigen::Matrix<float, 3, -1>>& rigVertices) const
			{
				CARBON_ASSERT(IsTrained(), "RigPoseBasedSolver must be trained before we can solve a frame");

				std::map<std::string, double> result;

				// extract the required rig vertices
				std::vector<std::map<std::string, dlib::vector<double, 3>>> viewsTrackingData(1); //  we only support one view for rig pose based solvers
				for (unsigned k = 0; k < RigGenerateTrainingData::MeshNames().size(); k++)
				{
					auto it = m_rigMeshVertexIdData.find(RigGenerateTrainingData::MeshNames()[k]);
					if (it != m_rigMeshVertexIdData.end())
					{
						CARBON_ASSERT(k < rigVertices.size(), "rigVertices must contain elements for rig index: " + std::to_string(k));
						for (const auto& vertexId : it->second.vertexIdsAndShapeIndices)
						{
							const int curVertexId = vertexId.first + it->second.vertexIdOutputOffset;
							CARBON_ASSERT(rigVertices[k].cols() > vertexId.first, "rigVertices for rig " + std::to_string(k) + " contains the wrong number of vertices");
							viewsTrackingData[0][std::to_string(curVertexId)] = dlib::vector<double, 3>
							{ rigVertices[k].col(vertexId.first).x(), rigVertices[k].col(vertexId.first).y(), rigVertices[k].col(vertexId.first).z() };
						}
					}
				}

				result = m_solver.solve_frame(viewsTrackingData, {}, 0.0);

				// finally, clip the range of the result to the control range (this is not done by default in solve_frame method)
				for (auto controlRangeIt = m_solver.get_control_ranges().begin(); controlRangeIt != m_solver.get_control_ranges().end(); ++controlRangeIt)
				{
					result.at(controlRangeIt->first) = std::min(std::max(result.at(controlRangeIt->first), controlRangeIt->second.min), controlRangeIt->second.max);
				}

				// unmap any controls we need to
				for (unsigned c = 0; c < m_controlMappers.size(); c++)
				{
					bool unMapped = m_controlMappers[c].UnMap(result);
					if (!unMapped)
					{
						CARBON_CRITICAL("Failed to unmap control mapper {}", m_controlMappers[c].toName);
					}
				}
	
				return result;
			}

			bool IsTrained() const
			{
				return m_solver.is_trained();
			}


			friend void serialize(const RigPoseBasedSolver::Impl& item, std::ostream& out)
			{
				dlib::serialize(item.version, out);
				serialize(item.m_solver, out);
				dlib::serialize(item.m_rigMeshVertexIdData, out);
				dlib::serialize(item.m_controlMappers, out);
			}

			friend void deserialize(RigPoseBasedSolver::Impl& item, std::istream& in)
			{
				// current version deserialization
				// IMPORTANT: please try and make class serialization / deserialization back-compatible
				// by supporting multiple versions if possible, and if not, function should throw an exception
				unsigned curVersion;
				dlib::deserialize(curVersion, in);
				if (curVersion == 1u)
				{
					using dlib::deserialize;
					deserialize(item.m_solver, in);
					dlib::deserialize(item.m_rigMeshVertexIdData, in);
					dlib::deserialize(item.m_controlMappers, in);
				}
				else
				{
					throw cm::serialization_error("RigPoseBasedSolver is not compatible with version : " + std::to_string(curVersion));
				}
			}

			std::string GetControlRangesMD5() const
			{
				return m_solver.get_control_ranges_md5();
			}

			std::string GetOrigTrainingParamsMD5() const
			{
				std::stringstream ss;
				serialize(m_solver.get_final_trained_params().orig_training_params, ss);
				return dlib::md5(ss);
			}

			std::string GetControlMappersMD5() const
			{
				std::stringstream ss;
				dlib::serialize(m_controlMappers, ss);
				return dlib::md5(ss);
			}

			std::vector<cm::modular_solver<double, 3>::alignmentD_type> ShapeAlignmentTypes() const
			{
				std::vector<cm::modular_solver<double, 3>::alignmentD_type> shapeAlignmentTypes(m_solver.get_n_shapes());
				for (unsigned i = 0; i < m_solver.get_n_shapes(); i++)
				{
					shapeAlignmentTypes[i] = m_solver.get_shape_alignment(i);
				}

				return shapeAlignmentTypes;
			}


			cm::modular_solver<double, 3> m_solver;
			std::map< std::string, RigGenerateTrainingData::MeshVertexIdData> m_rigMeshVertexIdData;
			std::vector<ControlMapper> m_controlMappers;
		};

		const unsigned RigPoseBasedSolver::Impl::version = 1;

		RigPoseBasedSolver::RigPoseBasedSolver():
			m_impl(new RigPoseBasedSolver::Impl())
		{
		}

		RigPoseBasedSolver::~RigPoseBasedSolver() = default;

		RigPoseBasedSolver::RigPoseBasedSolver(RigPoseBasedSolver&& other) = default;

		RigPoseBasedSolver::RigPoseBasedSolver(const RigPoseBasedSolver & other)
			: m_impl(new Impl(*other.m_impl))
		{}

		RigPoseBasedSolver& RigPoseBasedSolver::operator=(const RigPoseBasedSolver & other)
		{
			*m_impl = *other.m_impl;
			return *this;
		}

		RigPoseBasedSolver& RigPoseBasedSolver::operator=(RigPoseBasedSolver&& other) = default;


		std::map<std::string, double> RigPoseBasedSolver::SolveFrame(const std::vector<Eigen::Matrix<float, 3, -1>>& rigVertices) const
		{
			return m_impl->SolveFrame(rigVertices);
		}

		bool RigPoseBasedSolver::IsTrained() const
		{
			return m_impl->IsTrained();
		}

		void serialize(const RigPoseBasedSolver& solver, std::ostream& out)
		{
			serialize(*solver.m_impl, out);
		}

		void deserialize(RigPoseBasedSolver& solver, std::istream& in)
		{
			deserialize(*solver.m_impl, in);
		}

		void RigPoseBasedSolver::SetSolverData(const cm::modular_solver<double, 3>& modularSolver, 
			const std::map< std::string, RigGenerateTrainingData::MeshVertexIdData>& rigMeshVertexIdData,
			const std::vector<ControlMapper>& controlMappers)
		{
			m_impl->m_solver = modularSolver;
			m_impl->m_rigMeshVertexIdData = rigMeshVertexIdData;
			m_impl->m_controlMappers = controlMappers;
		}

		const cm::modular_solver<double, 3>& RigPoseBasedSolver::ModularSolver() const
		{
			return m_impl->m_solver;
		}

		std::string RigPoseBasedSolver::GetControlRangesMD5() const
		{
			CARBON_ASSERT(IsTrained(), "RigPoseBasedSolver must be trained before we can call GetControlRangesMD5()");
			return m_impl->GetControlRangesMD5();
		}

		std::string RigPoseBasedSolver::GetControlMappersMD5() const
		{
			CARBON_ASSERT(IsTrained(), "RigPoseBasedSolver must be trained before we can call GetControlMappersMD5()");
			return m_impl->GetControlMappersMD5();
		}

		std::string RigPoseBasedSolver::GetOrigTrainingParamsMD5() const
		{
			CARBON_ASSERT(IsTrained(), "RigPoseBasedSolver must be trained before we can call GetOrigTrainingParamsMD5()");
			return m_impl->GetOrigTrainingParamsMD5();
		}

		const std::map< std::string, RigGenerateTrainingData::MeshVertexIdData>& RigPoseBasedSolver::RigMeshVertexIdData() const
		{
			return m_impl->m_rigMeshVertexIdData;
		}

		const std::vector<ControlMapper>& RigPoseBasedSolver::ControlMappers() const
		{
			return m_impl->m_controlMappers;
		}


		std::vector<cm::modular_solver<double, 3>::alignmentD_type> RigPoseBasedSolver::ShapeAlignmentTypes() const
		{
			return m_impl->ShapeAlignmentTypes();
		}

    }
}
