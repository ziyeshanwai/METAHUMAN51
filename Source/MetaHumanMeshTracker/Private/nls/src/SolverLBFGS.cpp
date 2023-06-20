// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/SolverLBFGS.h>
#include <nls/Context.h>
#include <nls/Variable.h>
#include <carbon/utils/Profiler.h>

#include <chrono>

namespace epic {
namespace nls {

template <class T> T ResidualErrorStoppingCriterion2();
template <> float ResidualErrorStoppingCriterion2() { return float(1e-8); }
template <> double ResidualErrorStoppingCriterion2() { return double(1e-12); }

template <class T> T PredictionReductionErrorStoppingCriterion2();
template <> float PredictionReductionErrorStoppingCriterion2() { return float(1e-6); }
template <> double PredictionReductionErrorStoppingCriterion2() { return double(1e-8); }

template <class T>
bool LBFGSSolver<T>::Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			                    Context<T>& context,
			                    int iterations) const
{
    // gradient lambda
    auto getEnergyAndGradient = [&]()
    {
        PROFILING_BLOCK("evaluate");
        DiffData<T> diffData = evaluationFunction(&context);
        if (!diffData.HasJacobian())
        {
            throw std::runtime_error("data is missing the jacobian");
        }
        const Vector<T> fx = diffData.Value();
        const T residualError = diffData.Value().squaredNorm();
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("J");
        const SparseMatrix<T> Jt = diffData.Jacobian().AsSparseMatrix()->transpose();
        PROFILING_END_BLOCK;

        // the gradient can be calculated as 2 Jt r
        PROFILING_BLOCK("Gradient calculation");
        const Vector<T> g = Jt * fx * 2.0;
        PROFILING_END_BLOCK;

        return std::make_pair(residualError, g);
    };

    // stops when the residual error goes below this point
    const T residualErrorStoppingCriterion = ResidualErrorStoppingCriterion2<T>();

    // stops when predicted reduction goes below this point
    const T predictionReductionStoppingCriterion = PredictionReductionErrorStoppingCriterion2<T>();

    // storage for other variables
    Vector<T> x;
    Vector<T> g;
    Vector<T> newG;
    Vector<T> dx;
    Vector<T> prevX;
    Vector<T> prevG;
    int currentHistorySize = -1;
    int currentIndex = -2;
    T residualError = T(0.0);
    T alpha = T(1.0);

    PROFILING_FUNCTION(PROFILING_COLOR_BLUE);

    for (int iter = 0; iter < iterations; ++iter) {

        // perform LBFGS update
        currentHistorySize = std::min(currentHistorySize + 1, historySize);
        currentIndex = (currentIndex + 1) % historySize;

        std::tie(residualError, g) = getEnergyAndGradient();
        x = context.Value();

        // check for termination
        if (residualError < residualErrorStoppingCriterion)
        {
            return true;
        }

        if (currentHistorySize == 0)
        {
            // first iteration, just do gradient descent
            dx = -g;

            alpha = std::min(T(0.1), T(0.1) / dx.cwiseAbs().sum());
        }
        else
        {
            // later iteration, update search direction using lbfgs updat
            s[currentIndex] = x - prevX;
            y[currentIndex] = g - prevG;
            const auto ys = y[currentIndex].dot(s[currentIndex]);

            // check if current update is in a numerically stable direction
            if (ys > T(1e-7))
            {
                p[currentIndex] = T(1.0) / ys;

                const auto yk = ys / y[currentIndex].squaredNorm();

                dx = g;
                for (int i = 0; i < currentHistorySize; ++i)
                {
                    const auto index = (currentIndex - i) % currentHistorySize;
                    a[index] = s[index].dot(dx) * p[index];
                    dx -= y[index] * a[index];
                }

                dx *= yk;

                for (int i = 0; i < currentHistorySize; ++i)
                {
                    const auto index = (currentIndex + 1 + i) % currentHistorySize;
                    const auto b = y[index].dot(dx) * p[index];
                    dx += s[index] * (a[index] - b);
                }

                dx = -dx;

                alpha = T(1.0);
            }
            else
            {
                // no, reset and clear history
                currentHistorySize = 0;
                currentIndex = -1;
                dx = -g;
                alpha = std::min(T(0.1), T(0.1) / dx.cwiseAbs().sum());
            }
        }

        prevX = x;
        prevG = g;

        auto gtd = g.dot(dx);

        if (gtd >= T(0.0)) {
            // hessian approximation failed, reset
            currentHistorySize = 0;
            currentIndex = -1;
            dx = -g;
            alpha = std::min(T(0.1), T(0.1) / dx.cwiseAbs().sum());
            gtd = g.dot(dx);
        }

        if (-gtd / residualError < predictionReductionStoppingCriterion) {
            // almost no further change in residual possible, so stop the optimization
            return true;
        }

        PROFILING_BLOCK("update");
        context.Update(alpha * dx);
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("cubic update");
        // get gradient and value at estimated step
        T newResidualError;
        std::tie(newResidualError, newG) = getEnergyAndGradient();
        auto newGtd = newG.dot(dx);

        // calculate cubic interpolation udpate
        //self._cubic_interpolate(0, f, gtd.item(), alpha, f_new, gtd_new.item())
        const auto d1 = gtd + newGtd - T(3.0) * (residualError - newResidualError) / (0.0 - alpha);
        const auto d2_square = std::max(T(0.0), T(d1 * d1 - gtd * newGtd));
        const auto d2 = std::sqrt(d2_square);
        const auto step = alpha - alpha * ((newGtd + d2 - d1) / (newGtd - gtd + T(2.0) * d2));
        const auto alpha_new = std::min(std::max(T(step), T(0.0)), T(100.0));

        if (alpha_new == T(0.0))
        {
            // almost no further change in residual possible, so stop the optimization
            return true;
        }

        PROFILING_BLOCK("update2");
        context.Update((alpha_new - alpha) * dx);
        PROFILING_END_BLOCK;

        //T res = evaluationFunction(nullptr).Value().squaredNorm();
        PROFILING_END_BLOCK;
    }

    return true;
}

template class LBFGSSolver<float>;
template class LBFGSSolver<double>;

} // namespace nls
} //namespace epic
