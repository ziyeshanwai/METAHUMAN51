// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/Solver.h>
#include <nls/Context.h>
#include <nls/PatternCheck.h>
#include <nls/Variable.h>
#include <carbon/utils/Profiler.h>

NLS_DISABLE_EIGEN_WARNINGS
#ifdef EIGEN_USE_MKL_ALL
#include <Eigen/PardisoSupport>
#include <nls/math/MKLWrapper.h>
#else
#include <Eigen/IterativeLinearSolvers>
#endif
NLS_RENABLE_WARNINGS

#include <chrono>

namespace epic {
namespace nls {

template <> float DiagonalRegularization() { return float(1e-3); }
template <> double DiagonalRegularization() { return double(1e-7); }

template <class T> T ResidualErrorStoppingCriterion();
template <> float ResidualErrorStoppingCriterion() { return float(1e-8); }
template <> double ResidualErrorStoppingCriterion() { return double(1e-12); }

template <class T> T PredictionReductionErrorStoppingCriterion();
template <> float PredictionReductionErrorStoppingCriterion() { return float(1e-6); }
template <> double PredictionReductionErrorStoppingCriterion() { return double(1e-8); }


template <class T>
bool GaussNewtonSolver<T>::Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			                    Context<T>& context,
			                    int iterations,
                                T reg) const
{
    // stops when the residual error goes below this point
    const T residualErrorStoppingCriterion = ResidualErrorStoppingCriterion<T>();

    // stops when predicted reduction goes below this point
    const T predictionReductionStoppingCriterion = PredictionReductionErrorStoppingCriterion<T>();

    // maximum number of line search iterations
    const int maxLineSearchIterations = 10;

    PROFILING_FUNCTION(PROFILING_COLOR_BLUE);

#ifdef EIGEN_USE_MKL_ALL
    Eigen::PardisoLDLT<Eigen::SparseMatrix<T, Eigen::RowMajor>> solver;
#else
    Eigen::ConjugateGradient<Eigen::SparseMatrix<T, Eigen::RowMajor>, Eigen::Lower|Eigen::Upper> solver;
    // TODO: check if we want to switch to LeastSquaresConjugateGradient
    // Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<T, Eigen::RowMajor>> solver;
#endif

    PatternCheck<T> jtjPatternChecker;

    for (int iter = 0; iter < iterations; ++iter) {

        PROFILING_BLOCK("evaluate");
        DiffData<T> diffData = evaluationFunction(&context);
        if (!diffData.HasJacobian()) {
            throw std::runtime_error("data is missing the jacobian");
        }
        const Vector<T> fx = diffData.Value();
        const T residualError = diffData.Value().squaredNorm();
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("get current x");
        const Vector<T> currX = context.Value();
        PROFILING_END_BLOCK;

        // printf("%d: energy: %30.29f\n", iter, double(residualError));

        if (residualError < residualErrorStoppingCriterion) {
            return true;
        }

        // || fx + Jdx || = 0
        // JtJ dx = - Jt fx
        PROFILING_BLOCK("J");
        const int numTotalVariables = context.UpdateSize();
        SparseMatrix<T> Jt = diffData.Jacobian().AsSparseMatrix()->transpose();
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("Remove constant");
        // Remove empty columns of J (so empty rows of Jt). Empty columns are variables that are constant or the
        // solution is not depending on these variables.
        // Note that not removing the variables would mean that the regularization further below gets very slow as it will
        // add new diagonal coefficients into the sparse matrix
        Eigen::VectorXi mapping;
        DiscardEmptyInnerDimensions(Jt, mapping);
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("JtJ");
        Eigen::SparseMatrix<T, Eigen::RowMajor> JtJ;
#ifdef EIGEN_USE_MKL_ALL
        mkl::ComputeAAt(Jt, JtJ);
#else
        JtJ = Jt * Jt.transpose();
#endif
        PROFILING_END_BLOCK;

        if (reg > 0) {
            PROFILING_BLOCK("JtJ reg");
            for (int i = 0; i < JtJ.cols(); i++) {
                // only fast if the entry already exists, which is the case whenever the variable is used in the Jacobian (see DiscardEmptyInnerDimensions above)
                JtJ.coeffRef(i,i) += reg;
            }
            PROFILING_END_BLOCK;
        }

        PROFILING_BLOCK("Jtb");
        const Vector<T> Jtb = - Jt * fx;
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("analyze");
        if (jtjPatternChecker.checkAndUpdatePattern(JtJ)) {
            solver.analyzePattern(JtJ);
        }
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("factorize");
        solver.factorize(JtJ);
        // solver.factorize(*diffData.Jacobian().AsSparseMatrix());
        PROFILING_END_BLOCK;

        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("Gauss Newton: failed to analyze matrix - singularity?\n");
        }
        PROFILING_BLOCK("solve");
#ifndef EIGEN_USE_MKL_ALL
        // for congugate gradient we hardcode the maximum number of iterations
        solver.setMaxIterations(200);
#endif
        const Vector<T> dx = solver.solve(Jtb);
        // const Vector<T> dx = solver.solve(-fx);
        const T predictedReduction = Jtb.dot(dx);
        PROFILING_END_BLOCK;

        // a way to look at the angle between the update and the gradient descent direction
        // const T angle = acos(Jtb.normalized().dot(dx.normalized())) / T(CARBON_PI) * T(180.0);
        // printf("angle: %f\n", angle);

        // printf("%d: predicted reduction: %30.29f\n", iter, double(predictedReduction));

        if (predictedReduction / residualError < predictionReductionStoppingCriterion) {
            // almost no further change in residual possible, so stop the optimization
            return true;
        }

        PROFILING_BLOCK("update");
        Vector<T> finalDx;
        if (numTotalVariables > int(dx.size())) {
            // remapping - some variables are constant and not optimized
            finalDx = Vector<T>::Zero(numTotalVariables);
            for (int i = 0; i < int(mapping.size()); i++) {
                if (mapping[i] >= 0) {
                    finalDx[i] = dx[mapping[i]];
                }
            }
        } else {
            finalDx = dx;
        }
        context.Update(finalDx);
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("line search");
        // based on description in http://sites.science.oregonstate.edu/~gibsonn/optpart2.pdf
        T newResidualError = evaluationFunction(nullptr).Value().squaredNorm();
        T actualReduction = residualError - newResidualError;
        // printf("%d: actual reduction: %30.29f\n", iter, double(actualReduction));

        const T alpha = T(0.25);
        T currStep = T(1.0);
        int lineSearchIter = 0;
        //T armijo = 1.0;

        while (actualReduction < alpha * currStep * predictedReduction && lineSearchIter < maxLineSearchIterations) {
            // error has increased or not decreased signficantly
            lineSearchIter++;

            // Options to reduce the error:
            // 1) Armijo rules to reduce the error
            // armijo *= 0.5;
            // const T nextStep = armijo - accumulatedStep;

            // 2) Quadratric interpolation on the residuals to calculate the optimal step
            const T quadraticStep = - (-predictedReduction) / (2 *(newResidualError - residualError - (-predictedReduction)));
            if (quadraticStep <= 0 || quadraticStep >= 1.0) {
                throw std::runtime_error("Gauss Newton: quadratic interpolation has failed.\n");
            }
            // use quadratic interpolation only if interpoalted value is meaningful and not reducing the steps size too much
            const T nextStep = (quadraticStep > T(0.05)) ? quadraticStep * currStep : T(0.25) * currStep;

            // set and update to always start from the original position or otherwise there may be an accumulation of error
            context.Set(currX);
            context.Update(nextStep * finalDx);

            newResidualError = evaluationFunction(nullptr).Value().squaredNorm();
            T newActualReduction = residualError - newResidualError;
            // printf("%d:     new actual reduction: %30.29f (step %f)\n", iter, double(newActualReduction), double(nextStep));

            if (newActualReduction < actualReduction) {
                // results get worse, so we need to invert the last step and stop
                context.Set(currX);
                context.Update(currStep * finalDx);
                break;
            }
            actualReduction = newActualReduction;
            currStep = nextStep;
        }

        if (actualReduction <= 0) {
           printf("Gauss Newton: failed to further reduce the error, despite no stopping criterion was hit.\n");
           return true;
        }
        PROFILING_END_BLOCK;
    }

    return true;
}

template class GaussNewtonSolver<float>;
template class GaussNewtonSolver<double>;

} // namespace nls
} //namespace epic


//TEST(Performance, Test)
//{
//	int size = 1000000;
//	epic::nls2::SparseMatrix<double> sparseMatrix(size, size);
//	sparseMatrix.setIdentity();
//
//	for (int k = 0; k < 10; k++) {
//		auto start = std::chrono::high_resolution_clock::now();
//		sparseMatrix *= double(-12);
////		for (int j = 0; j < sparseMatrix.cols(); j++) {
////			sparseMatrix.valuePtr()[j] *= double(-12);
////		}
//		auto end = std::chrono::high_resolution_clock::now();
//
//		printf("Elapsed time in milliseconds: %f ms\n", std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1000000.0);
//	}
//
//}
//
