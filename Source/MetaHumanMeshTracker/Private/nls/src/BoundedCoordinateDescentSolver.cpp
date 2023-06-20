// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/BoundedCoordinateDescentSolver.h>
#include <nls/Context.h>
#include <nls/PatternCheck.h>
#include <nls/Variable.h>
#include <carbon/utils/Profiler.h>
#include <carbon/utils/Timer.h>


namespace epic {
namespace nls {

template <class T> T ResidualErrorStoppingCriterion();
template <class T> T PredictionReductionErrorStoppingCriterion();

template <class T>
bool BoundedCoordinateDescentSolver<T>::Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			                    Context<T>& context,
			                    int iterations,
                                BoundedVectorVariable<T>* boundedVectorVariable,
                                T l1Reg,
                                int coordinateDescentIterations) const
{
    std::vector<BoundedVectorVariable<T>*> boundedVectorVariables;
    if (boundedVectorVariable) boundedVectorVariables.emplace_back(boundedVectorVariable);
    return Solve(evaluationFunction, context, iterations, boundedVectorVariables, l1Reg, coordinateDescentIterations);
}

template <class T>
bool BoundedCoordinateDescentSolver<T>::Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
			                    Context<T>& context,
			                    int iterations,
                                const std::vector<BoundedVectorVariable<T>*>& boundedVectorVariables,
                                T l1Reg,
                                int coordinateDescentIterations) const
{
    // stops when the residual error goes below this point
    const T residualErrorStoppingCriterion = ResidualErrorStoppingCriterion<T>();

    // stops when predicted reduction goes below this point
    const T predictionReductionStoppingCriterion = PredictionReductionErrorStoppingCriterion<T>();

    // maximum number of line search iterations
    const int maxLineSearchIterations = 10;

    // bounds for each variable
    struct Bounds {
        bool valid = false;
        T lowerBound = std::numeric_limits<T>::lowest();
        T upperBound = std::numeric_limits<T>::max();
    };
    std::vector<Bounds> bounds;

    PROFILING_FUNCTION(PROFILING_COLOR_BLUE);
    for (int iter = 0; iter < iterations; ++iter) {

        PROFILING_BLOCK("evaluate");
        DiffData<T> diffData = evaluationFunction(&context);
        if (!diffData.HasJacobian()) {
            throw std::runtime_error("data is missing the jacobian");
        }
        const Vector<T> fx = diffData.Value();
        const T residualError = T(0.5) * diffData.Value().squaredNorm() + EvaluateL1(boundedVectorVariables, l1Reg);
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

        if (static_cast<int>(bounds.size()) != numTotalVariables) {
            PROFILING_BLOCK("Bounds");
            std::vector<Bounds> newBounds(numTotalVariables);
            for (const auto* boundedVectorVariable : boundedVectorVariables) {
                if (boundedVectorVariable) {
                    const std::pair<int, int> boundedVariableIndexAndSize = context.MappedVariableIndexAndSize(boundedVectorVariable);
                    if (boundedVectorVariable->BoundsAreEnforced()) {
                        // L1 and bounds are applied
                        for (int j = 0; j < boundedVariableIndexAndSize.second; ++j) {
                            newBounds[boundedVariableIndexAndSize.first + j].valid = true;
                            newBounds[boundedVariableIndexAndSize.first + j].lowerBound = boundedVectorVariable->Bounds()(0, j);
                            newBounds[boundedVariableIndexAndSize.first + j].upperBound = boundedVectorVariable->Bounds()(1, j);
                        }
                    } else {
                        // only L1 is applied
                        for (int j = 0; j < boundedVariableIndexAndSize.second; ++j) {
                            newBounds[boundedVariableIndexAndSize.first + j].valid = true;
                        }
                    }
                }
            }
            std::swap(bounds, newBounds);
            PROFILING_END_BLOCK;
        }

        PROFILING_BLOCK("JtJ");
        const Eigen::Matrix<T, -1, -1> Jtd = Jt;
        const Eigen::Matrix<T, -1, -1> JtJ = Jtd * Jtd.transpose();
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("Jtb");
        const Vector<T> Jtb = - Jt * fx;
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("solve");
        // gauss seidel iterations to solve
        Vector<T> dx = Vector<T>::Zero(Jtb.size());
        for (int gsIter = 0; gsIter < coordinateDescentIterations; ++gsIter) {
            // Vector<T> dxOld = dx;
            for (int r = 0; r < int(Jtb.size()); ++r) {
                if (JtJ(r, r) == 0) continue;
                T acc = Jtb[r];
                for (int c = 0; c < int(JtJ.cols()); ++c) {
                    if (c != r) {
                        acc -= JtJ(r, c) * dx[c];
                    }
                }
                if (bounds[r].valid) {
                    // bounded variable => apply first L1, then bounds

                    // include L1 in the optimization
                    // 0 = JtJ dx + Jt fx +/- l1Reg  (gradient)
                    // 0 = JtJ' dx' + Jt fx + JtJ(j,j) (x - prevx) +/- l1Reg
                    // - JtJ' dx' - Jt fx = JtJ(j,j) (x - prevx) +/- l1Reg
                    // acc = JtJ(j,j) (x - prevx) +/- l1Reg
                    // acc + JtJ(j,j) prevX = JtJ(j,j) x +/- l1Reg
                    // x = (acc + JtJ(j,j) prevX -/+ l1Reg) / JtJ(j,j)
                    // x = (acc2 -/+ l1Reg) / JtJ(j,j)
                    const T prevXj = currX[r];

                    T newXj = 0;
                    const T acc2 = acc + prevXj * JtJ(r, r);
                    if (acc2 > l1Reg) {
                        newXj = (acc2 - l1Reg) / JtJ(r, r);
                    } else if (acc2 < -l1Reg) {
                        newXj = (acc2 + l1Reg) / JtJ(r, r);
                    } else {
                        newXj = 0;
                    }

                    dx[r] = clamp<T>(newXj, bounds[r].lowerBound, bounds[r].upperBound) - prevXj;//variableBounds(0, mapToBoundedVariable[r]), variableBounds(1, mapToBoundedVariable[r])) - prevXj;
                } else {
                    dx[r] = acc / JtJ(r, r);
                }
            }
            // printf("max change %d: %f\n", gsIter, (dxOld-dx).cwiseAbs().maxCoeff());
        }
        const T predictedReduction = Jtb.dot(dx);
        PROFILING_END_BLOCK;

        // a way to look at the angle between the update and the gradient descent direction
        // const T angle = acos(Jtb.normalized().dot(dx.normalized())) / T(CARBON_PI) * T(180.0);
        // LOG_INFO("angle: {}", angle);

        // LOG_INFO("{}: predicted reduction: {}", iter, predictedReduction);

        if (predictedReduction / residualError < predictionReductionStoppingCriterion) {
            // almost no further change in residual possible, so stop the optimization
            return true;
        }

        PROFILING_BLOCK("update");
        context.Update(dx);
        PROFILING_END_BLOCK;

        PROFILING_BLOCK("line search");
        // based on description in http://sites.science.oregonstate.edu/~gibsonn/optpart2.pdf
        T newResidualError = Evaluate(evaluationFunction, boundedVectorVariables, l1Reg);
        T actualReduction = residualError - newResidualError;
        // LOG_INFO("{}: actual reduction: {} - {} = {}", iter, residualError, newResidualError, actualReduction);

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
                CARBON_CRITICAL("Bounded Coordinate Descent: quadratic interpolation in line search has failed.");
            }
            // use quadratic interpolation only if interpoalted value is meaningful and not reducing the steps size too much
            const T nextStep = (quadraticStep > T(0.05)) ? quadraticStep * currStep : T(0.25) * currStep;

            // set and update to always start from the original position or otherwise there may be an accumulation of error
            context.Set(currX);
            context.Update(nextStep * dx);

            newResidualError = Evaluate(evaluationFunction, boundedVectorVariables, l1Reg);
            T newActualReduction = residualError - newResidualError;
            // printf("%d:     new actual reduction: %30.29f (step %f)\n", iter, double(newActualReduction), double(nextStep));

            if (newActualReduction < actualReduction) {
                // results get worse, so we need to invert the last step and stop
                context.Set(currX);
                context.Update(currStep * dx);
                break;
            }
            actualReduction = newActualReduction;
            currStep = nextStep;
        }

        if (actualReduction <= 0) {
           LOG_INFO("Bounded Coordinate Descent: failed to further reduce the error, despite no stopping criterion was hit.");
           return true;
        }
        PROFILING_END_BLOCK;
    }

    return true;
}


template <class T>
T BoundedCoordinateDescentSolver<T>::EvaluateL1(const std::vector<BoundedVectorVariable<T>*>& boundedVectorVariables, T l1Reg) const
{
    T cost = 0;
    for (auto boundedVectorVariable : boundedVectorVariables) {
        if (boundedVectorVariable) {
            cost += l1Reg * boundedVectorVariable->Value().template lpNorm<1>();
        }
    }
    return cost;
}

template <class T>
T BoundedCoordinateDescentSolver<T>::Evaluate(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction, const std::vector<BoundedVectorVariable<T>*>& boundedVectorVariables, T l1Reg) const
{
    return  T(0.5) * evaluationFunction(nullptr).Value().squaredNorm() + EvaluateL1(boundedVectorVariables, l1Reg);
}


template class BoundedCoordinateDescentSolver<float>;
template class BoundedCoordinateDescentSolver<double>;

} // namespace nls
} //namespace epic
