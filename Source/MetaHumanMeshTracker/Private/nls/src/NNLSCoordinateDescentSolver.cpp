// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/NNLSCoordinateDescentSolver.h>
#include <nls/Context.h>
#include <nls/PatternCheck.h>
#include <nls/Variable.h>
#include <algorithm>
#include <chrono>
namespace epic {
namespace nls {
template <class T>
bool NNLSCoordinateDescentSolver<T>::Solve(const std::function<DiffData<T>(Context<T>*)>& evaluationFunction,
										   Context<T>& context,
	                                       int outerIterations ,
	                                       int innerIterations ) const
{
    const T eps = T(1e-10);   // stopping criteria for eps - KKT condition

	//We are solving min||f(x)||,x>=0, f(x)=Ax-b, J=A;
	//linearization : ||f(x+dx)||=||fx+Jdx||=||Ax-b+Jdx||=||Jx-b+Jdx||=||J(x+dx)-(Jx-fx)||, x+dx>=0


	//loop over outer iterations
	for (int iter = 0; iter < outerIterations; ++iter) {
		DiffData<T> diffData = evaluationFunction(&context);
		if (!diffData.HasJacobian()) {
			throw std::runtime_error("data is missing the jacobian");
		}
		const Vector<T> fx = diffData.Value();

		// Jacobian with both constant and non-constant variable
		const Eigen::SparseMatrix<T> J_all = *(diffData.Jacobian().AsSparseMatrix());

		// Remove empty columns i.e. variables are constant or the solution is not depending on the variables.
		Eigen::Matrix<int, -1, 1> mapping;
		Eigen::SparseMatrix<T> J_var = J_all;
		DiscardEmptyInnerDimensions(J_var, mapping);
		const Vector<T> currX = context.Value();
		const Eigen::Matrix<T, -1, -1> Jtd_var = J_var;//we need dense matrix, not sparse one
		const Eigen::Matrix<T, -1, -1> JtJ_var = Jtd_var.transpose() * Jtd_var;
		const Eigen::Matrix<T, -1, -1> Jtd_all = J_all;//we need dense matrix, not sparse one
		const Eigen::Matrix<T, -1, -1> JtJ_all = Jtd_all.transpose() * Jtd_all;
		const Vector<T> Jtb = -J_var.transpose() * (J_all * currX - fx); //negative gradient of 0.5*||Jx-fx||^2;
		Vector<T> x = Vector<T>::Zero(JtJ_var.cols());
		for (int i = 0; i < int(mapping.size()); i++) {
			if (mapping[i] >= 0) {
				x[mapping[i]] = currX[i];
			}
		}
		Vector<T> mu = Jtb;


		//loop over inner iterations
		for (int i = 0; i < innerIterations; ++i) {
			Vector<T> grad = JtJ_var * x - Jtb; //gradient of objective function 0.5*||J(x+dx)-(Jx-fx)||^2
			T min = grad[0]; //minimal element in gradient
			T max = min; //maximal element in gradient
			for (int m = 0; m < grad.size(); ++m) {
				if (min > grad[m]) {
					min = grad[m];
				}
				for (int r = 0; r < int(grad.size()); ++r) {
					if (x[r] > 0) {
						if (max < grad[m]) {
							max = grad[m];
						}
					}
				}
			}
			const bool c_1 = (min >= -eps);  //first stopping condition from paper, 1st Eq. in (7)
			const bool c_2 = (max <= eps);  //second stopping condition from paper, 2nd Eq. in (7)
			if (c_1 && c_2) {
				return true;
			}
			T x_diff;
			//loop over coordinates
			for (int k = 0; k<int(JtJ_var.cols()); ++k) {
				x_diff = -x[k];
				x[k] = std::max(T(0), T(x[k] - mu[k] / JtJ_var(k, k))); //Here we control 2 things: non-negativity of solution and  force gradient of Lagrangian function in zero, 1st and 2nd Eqs. in (6)
				x_diff += x[k];
				for (int j = 0; j < int(Jtb.size()); ++j) {
					mu[j] += x_diff * JtJ_var(j, k); //Here we force last Eq. in (6), so called complementary constraint
				}
			}
		}

		Vector<T> x_all;
		if (x.size() != currX.size()) {
			x_all = currX;
			for (int i = 0; i < int(mapping.size()); i++) {
				if (mapping[i] >= 0) {
					x_all[i] = x[mapping[i]];
				}
			}
		}
		else {
			x_all = x;
		}

		context.Update(x_all - currX);
	}
    return true;
}
template class NNLSCoordinateDescentSolver<float>;
template class NNLSCoordinateDescentSolver<double>;
} // namespace nls
} //namespace epic