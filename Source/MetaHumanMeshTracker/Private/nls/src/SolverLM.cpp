// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/SolverLM.h>
#include <nls/Context.h>
#include <nls/PatternCheck.h>
#include <nls/Variable.h>
#include <algorithm>
#include <iostream>

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
bool LMSolver<T>::Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
                        Context<T>& context,
                        int iterations) const
{
    // TODO: export stopping criterions in API (for user to set)
    const T epsilon1 = T(1e-8);   // stopping criteria for gradient norm
    const T epsilon2 = T(1e-8); // change in step is small
    int nu = 2; // we need this to update damping parameter mu: (JtJ + mu * E) * dx = -Jt * fx

#ifdef EIGEN_USE_MKL_ALL
        Eigen::PardisoLDLT<Eigen::SparseMatrix<T>> solver;
#else
        // Eigen::SimplicialLDLT<Eigen::SparseMatrix<T>> solver;
        Eigen::SparseLU<Eigen::SparseMatrix<T>> solver;
#endif

    PatternCheck<T> jtjPatternChecker;

    DiffData<T> diffData = evaluationFunction(&context);
    Vector<T> fx = diffData.Value();
    T residualError = fx.squaredNorm();
    Vector<T> currX = context.Value(); //initial iteration
    T normCurrX = currX.norm();
    Eigen::SparseMatrix<T> J = *(diffData.Jacobian().AsSparseMatrix());
    int numVariables = int(J.cols());

    // Remove empty columns i.e. variables are constant or the solution is not depending on the variables.
    // Note that not removing the variables would mean that the regularization further below gets very slow as it will
    // add new diagonal coefficients into the sparse matrix
    Eigen::Matrix<int, -1, 1> mapping;
    DiscardEmptyInnerDimensions(J, mapping);
    //We use this reference : http://www2.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf (page 24)
    Eigen::SparseMatrix<T> JtJ = J.transpose() * J;
    const int numVariableVariables = int(JtJ.cols());
    Eigen::SparseMatrix<T> E = (Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Identity(numVariableVariables, numVariableVariables)).sparseView();
    Vector<T> Jtb = -J.transpose() * fx; //// gradient  in initial iteration x_0
    T gradNorm = Jtb.squaredNorm(); // gradient norm in initial iteration x_0
    bool found = (gradNorm <= epsilon1);
    int k = 0;
    const T maxDiagonalCoeff = JtJ.diagonal().maxCoeff();
    T mu = maxDiagonalCoeff * T(1e-3); // initial damping parameter

    while (!found && k < iterations) {
        k += 1;

        // || fx + Jdx || = 0 , s.t. ||dx|| < delta
        // (JtJ + mu * E) * dx = - Jt * fx
        // trust-region complementarity condition : mu * (delta - ||dx||) = 0

        Eigen::SparseMatrix<T> LMJtJ = JtJ + mu * E;
        if (jtjPatternChecker.checkAndUpdatePattern(LMJtJ)) {
            solver.analyzePattern(LMJtJ);
        }

        solver.factorize(LMJtJ);

        if (solver.info() != Eigen::Success) {
            // TODO: proper logging
            printf("failed to analyze matrix\n");
            return false;
        }
        const Vector<T> dx = solver.solve(Jtb);
        const T normStep = dx.norm();

        if (normStep <= epsilon2 * (normCurrX + epsilon2)) {
            // almost no further change in norm possible, so stop the optimization
            return true;
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
        context.Update(finalDx); // we have new point: x_{k+1}= x_k + dx
        T newResidualError = evaluationFunction(nullptr).Value().squaredNorm();
        T actualReduction = residualError - newResidualError;
        T predictedReduction = 0.5 * dx.transpose() * (mu * dx + Jtb); //reduction predicted by linear model l(dx) = fx+J*dx

            if (actualReduction / predictedReduction > 0) {
                // if true, then we define J and g with new point: x + dx
                diffData = evaluationFunction(&context);
                currX = context.Value();
                normCurrX = currX.norm();
                fx = diffData.Value();
                J = *(diffData.Jacobian().AsSparseMatrix());
                numVariables = int(J.cols());
                residualError = fx.squaredNorm();

                DiscardEmptyInnerDimensions(J, mapping);

                JtJ = J.transpose() * J;
                Jtb = -J.transpose() * fx;//gradient of objective function
                gradNorm = Jtb.squaredNorm(); // gradient norm
                mu = mu * T(std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * actualReduction / predictedReduction - 1.0, 3)));//if step is good, i.e. we have reduced objective function, then  we reduce mu also
                nu = 2;

                found = (gradNorm <= epsilon1);
            } else {
                mu = mu * nu; //if step is bad, then we increase mu.
                nu = 2 * nu;
                context.Set(currX);//we don't accept new step
            }
    }

    return true;
}
template class LMSolver<float>;
template class LMSolver<double>;

} // namespace nls
} //namespace epic
