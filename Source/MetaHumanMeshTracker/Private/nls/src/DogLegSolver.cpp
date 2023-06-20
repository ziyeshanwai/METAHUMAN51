// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/SolverDogLeg.h>
#include <nls/Context.h>
#include <nls/PatternCheck.h>
#include <nls/Variable.h>
#include <algorithm>

NLS_DISABLE_EIGEN_WARNINGS
#ifdef EIGEN_USE_MKL_ALL
#include <Eigen/PardisoSupport>
#else
// #include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#endif
NLS_RENABLE_WARNINGS

#include <chrono>

namespace epic {
namespace nls {


template <class T>
bool DogLegSolver<T>::Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
                        Context<T>& context,
                        int iterations,
                        T reg) const
{
    // TODO: export stopping criterions in API (for user to set)
    T max_trust_radius = T(100.0);  // max trust radius
    T trust_radius = T(1.0); // trust - radius for  step

#ifdef EIGEN_USE_MKL_ALL
        Eigen::PardisoLDLT<Eigen::SparseMatrix<T>> solver;
#else
        //Eigen::SimplicialLDLT<Eigen::SparseMatrix<T>> solver;
        Eigen::SparseLU<Eigen::SparseMatrix<T>> solver;
#endif

        PatternCheck<T> jtjPatternChecker;

    for (int iter = 0; iter < iterations; ++iter){

         DiffData<T> diffData = evaluationFunction(&context);
         const Vector<T> fx = diffData.Value();
         T residualError = diffData.Value().squaredNorm();
         const Vector<T> currX = context.Value();
         Eigen::SparseMatrix<T> J = *(diffData.Jacobian().AsSparseMatrix());
         const int numVariables = int(J.cols());


         // Remove empty columns i.e. variables are constant or the solution is not depending on the variables.
         // Note that not removing the variables would mean that the regularization further below gets very slow as it will
         // add new diagonal coefficients into the sparse matrix
         Eigen::Matrix<int, -1, 1> mapping;
         DiscardEmptyInnerDimensions(J, mapping);
         Eigen::SparseMatrix<T> JtJ = J.transpose() * J;


        // || fx + Jdx || = 0 , s.t. ||dx|| < trust_radius
         //Nocedal, Wright implemenation
        // if || tau * Jtb * (trust_radius/||Jtb||) ||=trust_radius, then dx_sd=tau * Jtb * (trust_radius/||Jtb||) - sd means  steepest descent, tau=min(1,||Jtb||^3/trust_radius*||JJt*fx||^2)
        // else
         //dx_dl=dx_sd+tau * ( dx_gn - dx_sd )  - tau is largest value in [0,1] such that ||dx_dl||<=trust - radius , dl means dog leg, gn means  Gauss Newton

            if (reg>0) {
                for (int i = 0; i < int(JtJ.cols()); i++) {
                    // only fast if the entry already exists, which is the case whenever the variable is used in the Jacobian (see DiscardEmptyInnerDimensions above)
                    JtJ.coeffRef(i,i) += reg;
                 }
            }

         Vector<T> Jtb = -J.transpose() * fx; // negative  gradient  i.e. steepest descent
        //  T squaredGradNorm = Jtb.squaredNorm(); // squared gradient norm
         T gradNorm = Jtb.norm(); //  gradient norm
         T tau_sd = std::min<T>(T(1.0), T(std::pow(gradNorm, 3)) / (trust_radius * Jtb.transpose() * JtJ * Jtb));
         Vector<T> dx_sd=tau_sd*(trust_radius/gradNorm)*Jtb; // steepest descent- so called Cauchy point

        if (jtjPatternChecker.checkAndUpdatePattern(JtJ)) {
            solver.analyzePattern(JtJ);
        }
        solver.factorize(JtJ);
        if (solver.info() != Eigen::Success) {
            // TODO: proper logging
            printf("failed to analyze matrix\n");
            return false;
        }

        const Vector<T> dx_gn = solver.solve(Jtb);// Gauss Newton direction
        T tau = (-dx_sd.transpose() * (dx_gn - dx_sd) +
                T(std::sqrt(T(std::pow(dx_sd.transpose() * (dx_gn - dx_sd), 2)) - (dx_gn - dx_sd).squaredNorm() * (dx_sd.squaredNorm() - T(std::pow(trust_radius, 2)))))) / (dx_gn - dx_sd).squaredNorm();
        // tau must be in [0,1]
        if (tau > 1) {
            tau = 1;
        }
        else {
            if (tau < 0) {
                tau = 0;
            }
        }
       // Selection of dx
        Vector<T> dx;
            if ( dx_sd.norm() ==trust_radius )  {
                dx = dx_sd;
            }
            else {
                dx = dx_sd + tau * (dx_gn - dx_sd);
            }
        Vector<T> finalDx = dx;
        if (numVariables > int(dx.size())) {
            // remapping - some variables are constant and not optimized
            finalDx = Vector<T>::Zero(numVariables);
            for (int i = 0; i < int(mapping.size()); i++) {
                if (mapping[i] >= 0) {
                    finalDx[i] = dx[mapping[i]];
                }
            }
        }
        context.Update(finalDx); // we have candidate for new point: x_{k+1}= x_k + dx
        T newResidualError = evaluationFunction(nullptr).Value().squaredNorm();
        T actualReduction = residualError - newResidualError;
        T predictedReduction = - finalDx.transpose() * ( -Jtb + T(0.5) * JtJ * finalDx ); //reduction m_k(0)-m_k(dx) predicted by quadratic model m_k(dx)=0.5*||fx+J*dx||^2
        //setting trust region radius
        T new_trust_radius;

            if (actualReduction / predictedReduction < 0.25) {
                new_trust_radius= T(0.25) * finalDx.norm();
            }
            else {
                if (actualReduction / predictedReduction > 0.75 &&  (std::fabs(finalDx.norm() - trust_radius) < 1e-5)) {
                    new_trust_radius = std::min(T(2.0) * trust_radius, max_trust_radius);
                }
                else {
                    new_trust_radius = trust_radius;
                }
            }
        // the criterion for rejecting a candidat for new point
        if ( actualReduction / predictedReduction < 0.15 ){

            context.Set(currX);//we don't accept new step
        }
        trust_radius = new_trust_radius;
    }

    return true;
}

template class DogLegSolver<float>;
template class DogLegSolver<double>;

} // namespace nls
} //namespace epic
