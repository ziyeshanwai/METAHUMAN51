// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/TetConstraints.h>
#include <nls/math/Math.h>

#include <iostream>

namespace epic::nls {


template <class T>
void FtoR(const Eigen::Matrix<T, 3, 3>& F, Eigen::Matrix<T, 3, 3>& R, Eigen::Matrix<T, 9, 9>* dRdF)
{
	Eigen::Matrix<T, 3, 3> S;
	const Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
	const T det = F.determinant();
	if (det < 0) {
		R = svd.matrixU() * Eigen::Vector<T,3>(1,1,-1).asDiagonal() * svd.matrixV().transpose();
	} else {
		R = svd.matrixU() * svd.matrixV().transpose();
	}

	if (dRdF) {
		if (det < 0) {
			S = svd.matrixV() * Eigen::Vector<T,3>(1,1,-1).asDiagonal() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
		} else {
			S = svd.matrixV() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
		}

		const Eigen::Matrix<T, 3, 3> D = S.trace()*Eigen::Matrix<T, 3, 3>::Identity() - S;
		Eigen::Matrix<T, 3, 3> Dinv;
		if (D.determinant() != 0) {
			Dinv = D.inverse();
		} else {
			Dinv = Eigen::Matrix<T, 3, 3>::Zero();
		}

		// Eigen::Matrix<T, 3, 3> E[3];
		// E[0] = Eigen::Matrix<T, 3, 3>::Zero();
		// E[0](2, 1) = -1;
		// E[0](1, 2) = 1;
		// E[1] = Eigen::Matrix<T, 3, 3>::Zero();
		// E[1](2, 0) = 1;
		// E[1](0, 2) = -1;
		// E[2] = Eigen::Matrix<T, 3, 3>::Zero();
		// E[2](1, 0) = -1;
		// E[2](0, 1) = 1;

		// for (int d = 0; d < 3; ++d) {
		// 	for (int y = 0; y < 3; ++y) {
		// 		for (int t = 0; t < 3; ++t) {
		// 			for (int p = 0; p < 3; ++p) {
		// 				std::vector<std::string> valuesToAdd;
		// 				T deriv = 0;
		// 				for (int a = 0; a < 3; ++a) {
		// 					for (int b = 0; b < 3; ++b) {
		// 						for (int e = 0; e < 3; ++e) {
		// 							for (int s = 0; s < 3; ++s) {
		// 								const T mul = E[a](e,d) * E[b](s,t);
		// 								if (mul != 0) {
		// 									deriv += E[a](e,d) * R(y,e) * Dinv(a,b) * E[b](s,t) * R(p, s);
		// 									//LOG_INFO("({},{}) += R({},{}) * Dinv({},{} * R({},{}))", 3 * d + y, 3 * t + p, y, e, a, b, p, s);
		// 									valuesToAdd.push_back(epic::carbon::fmt::format("{} R({},{}) * Dinv({},{}) * R({},{})", mul < 0 ? " -" : " +", y, e, a, b, p, s));
		// 								}
		// 							}
		// 						}
		// 					}
		// 				}
		// 				std::string command = epic::carbon::fmt::format("(*dRdF)({},{}) =", 3 * d + y, 3 * t + p);
		// 				for (size_t k = 0; k < valuesToAdd.size(); ++k) {
		// 					command = command + valuesToAdd[k];
		// 				}
		// 				printf("%s;\n", command.c_str());
		// 				(*dRdF)(3 * d + y, 3 * t + p) = deriv;
		// 			}
		// 		}
		// 	}
		// }

		(*dRdF)(0,0) = + R(0,2) * Dinv(1,1) * R(0,2) - R(0,2) * Dinv(1,2) * R(0,1) - R(0,1) * Dinv(2,1) * R(0,2) + R(0,1) * Dinv(2,2) * R(0,1);
		(*dRdF)(0,1) = + R(0,2) * Dinv(1,1) * R(1,2) - R(0,2) * Dinv(1,2) * R(1,1) - R(0,1) * Dinv(2,1) * R(1,2) + R(0,1) * Dinv(2,2) * R(1,1);
		(*dRdF)(0,2) = + R(0,2) * Dinv(1,1) * R(2,2) - R(0,2) * Dinv(1,2) * R(2,1) - R(0,1) * Dinv(2,1) * R(2,2) + R(0,1) * Dinv(2,2) * R(2,1);
		(*dRdF)(0,3) = - R(0,2) * Dinv(1,0) * R(0,2) + R(0,2) * Dinv(1,2) * R(0,0) + R(0,1) * Dinv(2,0) * R(0,2) - R(0,1) * Dinv(2,2) * R(0,0);
		(*dRdF)(0,4) = - R(0,2) * Dinv(1,0) * R(1,2) + R(0,2) * Dinv(1,2) * R(1,0) + R(0,1) * Dinv(2,0) * R(1,2) - R(0,1) * Dinv(2,2) * R(1,0);
		(*dRdF)(0,5) = - R(0,2) * Dinv(1,0) * R(2,2) + R(0,2) * Dinv(1,2) * R(2,0) + R(0,1) * Dinv(2,0) * R(2,2) - R(0,1) * Dinv(2,2) * R(2,0);
		(*dRdF)(0,6) = + R(0,2) * Dinv(1,0) * R(0,1) - R(0,2) * Dinv(1,1) * R(0,0) - R(0,1) * Dinv(2,0) * R(0,1) + R(0,1) * Dinv(2,1) * R(0,0);
		(*dRdF)(0,7) = + R(0,2) * Dinv(1,0) * R(1,1) - R(0,2) * Dinv(1,1) * R(1,0) - R(0,1) * Dinv(2,0) * R(1,1) + R(0,1) * Dinv(2,1) * R(1,0);
		(*dRdF)(0,8) = + R(0,2) * Dinv(1,0) * R(2,1) - R(0,2) * Dinv(1,1) * R(2,0) - R(0,1) * Dinv(2,0) * R(2,1) + R(0,1) * Dinv(2,1) * R(2,0);
		(*dRdF)(1,0) = + R(1,2) * Dinv(1,1) * R(0,2) - R(1,2) * Dinv(1,2) * R(0,1) - R(1,1) * Dinv(2,1) * R(0,2) + R(1,1) * Dinv(2,2) * R(0,1);
		(*dRdF)(1,1) = + R(1,2) * Dinv(1,1) * R(1,2) - R(1,2) * Dinv(1,2) * R(1,1) - R(1,1) * Dinv(2,1) * R(1,2) + R(1,1) * Dinv(2,2) * R(1,1);
		(*dRdF)(1,2) = + R(1,2) * Dinv(1,1) * R(2,2) - R(1,2) * Dinv(1,2) * R(2,1) - R(1,1) * Dinv(2,1) * R(2,2) + R(1,1) * Dinv(2,2) * R(2,1);
		(*dRdF)(1,3) = - R(1,2) * Dinv(1,0) * R(0,2) + R(1,2) * Dinv(1,2) * R(0,0) + R(1,1) * Dinv(2,0) * R(0,2) - R(1,1) * Dinv(2,2) * R(0,0);
		(*dRdF)(1,4) = - R(1,2) * Dinv(1,0) * R(1,2) + R(1,2) * Dinv(1,2) * R(1,0) + R(1,1) * Dinv(2,0) * R(1,2) - R(1,1) * Dinv(2,2) * R(1,0);
		(*dRdF)(1,5) = - R(1,2) * Dinv(1,0) * R(2,2) + R(1,2) * Dinv(1,2) * R(2,0) + R(1,1) * Dinv(2,0) * R(2,2) - R(1,1) * Dinv(2,2) * R(2,0);
		(*dRdF)(1,6) = + R(1,2) * Dinv(1,0) * R(0,1) - R(1,2) * Dinv(1,1) * R(0,0) - R(1,1) * Dinv(2,0) * R(0,1) + R(1,1) * Dinv(2,1) * R(0,0);
		(*dRdF)(1,7) = + R(1,2) * Dinv(1,0) * R(1,1) - R(1,2) * Dinv(1,1) * R(1,0) - R(1,1) * Dinv(2,0) * R(1,1) + R(1,1) * Dinv(2,1) * R(1,0);
		(*dRdF)(1,8) = + R(1,2) * Dinv(1,0) * R(2,1) - R(1,2) * Dinv(1,1) * R(2,0) - R(1,1) * Dinv(2,0) * R(2,1) + R(1,1) * Dinv(2,1) * R(2,0);
		(*dRdF)(2,0) = + R(2,2) * Dinv(1,1) * R(0,2) - R(2,2) * Dinv(1,2) * R(0,1) - R(2,1) * Dinv(2,1) * R(0,2) + R(2,1) * Dinv(2,2) * R(0,1);
		(*dRdF)(2,1) = + R(2,2) * Dinv(1,1) * R(1,2) - R(2,2) * Dinv(1,2) * R(1,1) - R(2,1) * Dinv(2,1) * R(1,2) + R(2,1) * Dinv(2,2) * R(1,1);
		(*dRdF)(2,2) = + R(2,2) * Dinv(1,1) * R(2,2) - R(2,2) * Dinv(1,2) * R(2,1) - R(2,1) * Dinv(2,1) * R(2,2) + R(2,1) * Dinv(2,2) * R(2,1);
		(*dRdF)(2,3) = - R(2,2) * Dinv(1,0) * R(0,2) + R(2,2) * Dinv(1,2) * R(0,0) + R(2,1) * Dinv(2,0) * R(0,2) - R(2,1) * Dinv(2,2) * R(0,0);
		(*dRdF)(2,4) = - R(2,2) * Dinv(1,0) * R(1,2) + R(2,2) * Dinv(1,2) * R(1,0) + R(2,1) * Dinv(2,0) * R(1,2) - R(2,1) * Dinv(2,2) * R(1,0);
		(*dRdF)(2,5) = - R(2,2) * Dinv(1,0) * R(2,2) + R(2,2) * Dinv(1,2) * R(2,0) + R(2,1) * Dinv(2,0) * R(2,2) - R(2,1) * Dinv(2,2) * R(2,0);
		(*dRdF)(2,6) = + R(2,2) * Dinv(1,0) * R(0,1) - R(2,2) * Dinv(1,1) * R(0,0) - R(2,1) * Dinv(2,0) * R(0,1) + R(2,1) * Dinv(2,1) * R(0,0);
		(*dRdF)(2,7) = + R(2,2) * Dinv(1,0) * R(1,1) - R(2,2) * Dinv(1,1) * R(1,0) - R(2,1) * Dinv(2,0) * R(1,1) + R(2,1) * Dinv(2,1) * R(1,0);
		(*dRdF)(2,8) = + R(2,2) * Dinv(1,0) * R(2,1) - R(2,2) * Dinv(1,1) * R(2,0) - R(2,1) * Dinv(2,0) * R(2,1) + R(2,1) * Dinv(2,1) * R(2,0);
		(*dRdF)(3,0) = - R(0,2) * Dinv(0,1) * R(0,2) + R(0,2) * Dinv(0,2) * R(0,1) + R(0,0) * Dinv(2,1) * R(0,2) - R(0,0) * Dinv(2,2) * R(0,1);
		(*dRdF)(3,1) = - R(0,2) * Dinv(0,1) * R(1,2) + R(0,2) * Dinv(0,2) * R(1,1) + R(0,0) * Dinv(2,1) * R(1,2) - R(0,0) * Dinv(2,2) * R(1,1);
		(*dRdF)(3,2) = - R(0,2) * Dinv(0,1) * R(2,2) + R(0,2) * Dinv(0,2) * R(2,1) + R(0,0) * Dinv(2,1) * R(2,2) - R(0,0) * Dinv(2,2) * R(2,1);
		(*dRdF)(3,3) = + R(0,2) * Dinv(0,0) * R(0,2) - R(0,2) * Dinv(0,2) * R(0,0) - R(0,0) * Dinv(2,0) * R(0,2) + R(0,0) * Dinv(2,2) * R(0,0);
		(*dRdF)(3,4) = + R(0,2) * Dinv(0,0) * R(1,2) - R(0,2) * Dinv(0,2) * R(1,0) - R(0,0) * Dinv(2,0) * R(1,2) + R(0,0) * Dinv(2,2) * R(1,0);
		(*dRdF)(3,5) = + R(0,2) * Dinv(0,0) * R(2,2) - R(0,2) * Dinv(0,2) * R(2,0) - R(0,0) * Dinv(2,0) * R(2,2) + R(0,0) * Dinv(2,2) * R(2,0);
		(*dRdF)(3,6) = - R(0,2) * Dinv(0,0) * R(0,1) + R(0,2) * Dinv(0,1) * R(0,0) + R(0,0) * Dinv(2,0) * R(0,1) - R(0,0) * Dinv(2,1) * R(0,0);
		(*dRdF)(3,7) = - R(0,2) * Dinv(0,0) * R(1,1) + R(0,2) * Dinv(0,1) * R(1,0) + R(0,0) * Dinv(2,0) * R(1,1) - R(0,0) * Dinv(2,1) * R(1,0);
		(*dRdF)(3,8) = - R(0,2) * Dinv(0,0) * R(2,1) + R(0,2) * Dinv(0,1) * R(2,0) + R(0,0) * Dinv(2,0) * R(2,1) - R(0,0) * Dinv(2,1) * R(2,0);
		(*dRdF)(4,0) = - R(1,2) * Dinv(0,1) * R(0,2) + R(1,2) * Dinv(0,2) * R(0,1) + R(1,0) * Dinv(2,1) * R(0,2) - R(1,0) * Dinv(2,2) * R(0,1);
		(*dRdF)(4,1) = - R(1,2) * Dinv(0,1) * R(1,2) + R(1,2) * Dinv(0,2) * R(1,1) + R(1,0) * Dinv(2,1) * R(1,2) - R(1,0) * Dinv(2,2) * R(1,1);
		(*dRdF)(4,2) = - R(1,2) * Dinv(0,1) * R(2,2) + R(1,2) * Dinv(0,2) * R(2,1) + R(1,0) * Dinv(2,1) * R(2,2) - R(1,0) * Dinv(2,2) * R(2,1);
		(*dRdF)(4,3) = + R(1,2) * Dinv(0,0) * R(0,2) - R(1,2) * Dinv(0,2) * R(0,0) - R(1,0) * Dinv(2,0) * R(0,2) + R(1,0) * Dinv(2,2) * R(0,0);
		(*dRdF)(4,4) = + R(1,2) * Dinv(0,0) * R(1,2) - R(1,2) * Dinv(0,2) * R(1,0) - R(1,0) * Dinv(2,0) * R(1,2) + R(1,0) * Dinv(2,2) * R(1,0);
		(*dRdF)(4,5) = + R(1,2) * Dinv(0,0) * R(2,2) - R(1,2) * Dinv(0,2) * R(2,0) - R(1,0) * Dinv(2,0) * R(2,2) + R(1,0) * Dinv(2,2) * R(2,0);
		(*dRdF)(4,6) = - R(1,2) * Dinv(0,0) * R(0,1) + R(1,2) * Dinv(0,1) * R(0,0) + R(1,0) * Dinv(2,0) * R(0,1) - R(1,0) * Dinv(2,1) * R(0,0);
		(*dRdF)(4,7) = - R(1,2) * Dinv(0,0) * R(1,1) + R(1,2) * Dinv(0,1) * R(1,0) + R(1,0) * Dinv(2,0) * R(1,1) - R(1,0) * Dinv(2,1) * R(1,0);
		(*dRdF)(4,8) = - R(1,2) * Dinv(0,0) * R(2,1) + R(1,2) * Dinv(0,1) * R(2,0) + R(1,0) * Dinv(2,0) * R(2,1) - R(1,0) * Dinv(2,1) * R(2,0);
		(*dRdF)(5,0) = - R(2,2) * Dinv(0,1) * R(0,2) + R(2,2) * Dinv(0,2) * R(0,1) + R(2,0) * Dinv(2,1) * R(0,2) - R(2,0) * Dinv(2,2) * R(0,1);
		(*dRdF)(5,1) = - R(2,2) * Dinv(0,1) * R(1,2) + R(2,2) * Dinv(0,2) * R(1,1) + R(2,0) * Dinv(2,1) * R(1,2) - R(2,0) * Dinv(2,2) * R(1,1);
		(*dRdF)(5,2) = - R(2,2) * Dinv(0,1) * R(2,2) + R(2,2) * Dinv(0,2) * R(2,1) + R(2,0) * Dinv(2,1) * R(2,2) - R(2,0) * Dinv(2,2) * R(2,1);
		(*dRdF)(5,3) = + R(2,2) * Dinv(0,0) * R(0,2) - R(2,2) * Dinv(0,2) * R(0,0) - R(2,0) * Dinv(2,0) * R(0,2) + R(2,0) * Dinv(2,2) * R(0,0);
		(*dRdF)(5,4) = + R(2,2) * Dinv(0,0) * R(1,2) - R(2,2) * Dinv(0,2) * R(1,0) - R(2,0) * Dinv(2,0) * R(1,2) + R(2,0) * Dinv(2,2) * R(1,0);
		(*dRdF)(5,5) = + R(2,2) * Dinv(0,0) * R(2,2) - R(2,2) * Dinv(0,2) * R(2,0) - R(2,0) * Dinv(2,0) * R(2,2) + R(2,0) * Dinv(2,2) * R(2,0);
		(*dRdF)(5,6) = - R(2,2) * Dinv(0,0) * R(0,1) + R(2,2) * Dinv(0,1) * R(0,0) + R(2,0) * Dinv(2,0) * R(0,1) - R(2,0) * Dinv(2,1) * R(0,0);
		(*dRdF)(5,7) = - R(2,2) * Dinv(0,0) * R(1,1) + R(2,2) * Dinv(0,1) * R(1,0) + R(2,0) * Dinv(2,0) * R(1,1) - R(2,0) * Dinv(2,1) * R(1,0);
		(*dRdF)(5,8) = - R(2,2) * Dinv(0,0) * R(2,1) + R(2,2) * Dinv(0,1) * R(2,0) + R(2,0) * Dinv(2,0) * R(2,1) - R(2,0) * Dinv(2,1) * R(2,0);
		(*dRdF)(6,0) = + R(0,1) * Dinv(0,1) * R(0,2) - R(0,1) * Dinv(0,2) * R(0,1) - R(0,0) * Dinv(1,1) * R(0,2) + R(0,0) * Dinv(1,2) * R(0,1);
		(*dRdF)(6,1) = + R(0,1) * Dinv(0,1) * R(1,2) - R(0,1) * Dinv(0,2) * R(1,1) - R(0,0) * Dinv(1,1) * R(1,2) + R(0,0) * Dinv(1,2) * R(1,1);
		(*dRdF)(6,2) = + R(0,1) * Dinv(0,1) * R(2,2) - R(0,1) * Dinv(0,2) * R(2,1) - R(0,0) * Dinv(1,1) * R(2,2) + R(0,0) * Dinv(1,2) * R(2,1);
		(*dRdF)(6,3) = - R(0,1) * Dinv(0,0) * R(0,2) + R(0,1) * Dinv(0,2) * R(0,0) + R(0,0) * Dinv(1,0) * R(0,2) - R(0,0) * Dinv(1,2) * R(0,0);
		(*dRdF)(6,4) = - R(0,1) * Dinv(0,0) * R(1,2) + R(0,1) * Dinv(0,2) * R(1,0) + R(0,0) * Dinv(1,0) * R(1,2) - R(0,0) * Dinv(1,2) * R(1,0);
		(*dRdF)(6,5) = - R(0,1) * Dinv(0,0) * R(2,2) + R(0,1) * Dinv(0,2) * R(2,0) + R(0,0) * Dinv(1,0) * R(2,2) - R(0,0) * Dinv(1,2) * R(2,0);
		(*dRdF)(6,6) = + R(0,1) * Dinv(0,0) * R(0,1) - R(0,1) * Dinv(0,1) * R(0,0) - R(0,0) * Dinv(1,0) * R(0,1) + R(0,0) * Dinv(1,1) * R(0,0);
		(*dRdF)(6,7) = + R(0,1) * Dinv(0,0) * R(1,1) - R(0,1) * Dinv(0,1) * R(1,0) - R(0,0) * Dinv(1,0) * R(1,1) + R(0,0) * Dinv(1,1) * R(1,0);
		(*dRdF)(6,8) = + R(0,1) * Dinv(0,0) * R(2,1) - R(0,1) * Dinv(0,1) * R(2,0) - R(0,0) * Dinv(1,0) * R(2,1) + R(0,0) * Dinv(1,1) * R(2,0);
		(*dRdF)(7,0) = + R(1,1) * Dinv(0,1) * R(0,2) - R(1,1) * Dinv(0,2) * R(0,1) - R(1,0) * Dinv(1,1) * R(0,2) + R(1,0) * Dinv(1,2) * R(0,1);
		(*dRdF)(7,1) = + R(1,1) * Dinv(0,1) * R(1,2) - R(1,1) * Dinv(0,2) * R(1,1) - R(1,0) * Dinv(1,1) * R(1,2) + R(1,0) * Dinv(1,2) * R(1,1);
		(*dRdF)(7,2) = + R(1,1) * Dinv(0,1) * R(2,2) - R(1,1) * Dinv(0,2) * R(2,1) - R(1,0) * Dinv(1,1) * R(2,2) + R(1,0) * Dinv(1,2) * R(2,1);
		(*dRdF)(7,3) = - R(1,1) * Dinv(0,0) * R(0,2) + R(1,1) * Dinv(0,2) * R(0,0) + R(1,0) * Dinv(1,0) * R(0,2) - R(1,0) * Dinv(1,2) * R(0,0);
		(*dRdF)(7,4) = - R(1,1) * Dinv(0,0) * R(1,2) + R(1,1) * Dinv(0,2) * R(1,0) + R(1,0) * Dinv(1,0) * R(1,2) - R(1,0) * Dinv(1,2) * R(1,0);
		(*dRdF)(7,5) = - R(1,1) * Dinv(0,0) * R(2,2) + R(1,1) * Dinv(0,2) * R(2,0) + R(1,0) * Dinv(1,0) * R(2,2) - R(1,0) * Dinv(1,2) * R(2,0);
		(*dRdF)(7,6) = + R(1,1) * Dinv(0,0) * R(0,1) - R(1,1) * Dinv(0,1) * R(0,0) - R(1,0) * Dinv(1,0) * R(0,1) + R(1,0) * Dinv(1,1) * R(0,0);
		(*dRdF)(7,7) = + R(1,1) * Dinv(0,0) * R(1,1) - R(1,1) * Dinv(0,1) * R(1,0) - R(1,0) * Dinv(1,0) * R(1,1) + R(1,0) * Dinv(1,1) * R(1,0);
		(*dRdF)(7,8) = + R(1,1) * Dinv(0,0) * R(2,1) - R(1,1) * Dinv(0,1) * R(2,0) - R(1,0) * Dinv(1,0) * R(2,1) + R(1,0) * Dinv(1,1) * R(2,0);
		(*dRdF)(8,0) = + R(2,1) * Dinv(0,1) * R(0,2) - R(2,1) * Dinv(0,2) * R(0,1) - R(2,0) * Dinv(1,1) * R(0,2) + R(2,0) * Dinv(1,2) * R(0,1);
		(*dRdF)(8,1) = + R(2,1) * Dinv(0,1) * R(1,2) - R(2,1) * Dinv(0,2) * R(1,1) - R(2,0) * Dinv(1,1) * R(1,2) + R(2,0) * Dinv(1,2) * R(1,1);
		(*dRdF)(8,2) = + R(2,1) * Dinv(0,1) * R(2,2) - R(2,1) * Dinv(0,2) * R(2,1) - R(2,0) * Dinv(1,1) * R(2,2) + R(2,0) * Dinv(1,2) * R(2,1);
		(*dRdF)(8,3) = - R(2,1) * Dinv(0,0) * R(0,2) + R(2,1) * Dinv(0,2) * R(0,0) + R(2,0) * Dinv(1,0) * R(0,2) - R(2,0) * Dinv(1,2) * R(0,0);
		(*dRdF)(8,4) = - R(2,1) * Dinv(0,0) * R(1,2) + R(2,1) * Dinv(0,2) * R(1,0) + R(2,0) * Dinv(1,0) * R(1,2) - R(2,0) * Dinv(1,2) * R(1,0);
		(*dRdF)(8,5) = - R(2,1) * Dinv(0,0) * R(2,2) + R(2,1) * Dinv(0,2) * R(2,0) + R(2,0) * Dinv(1,0) * R(2,2) - R(2,0) * Dinv(1,2) * R(2,0);
		(*dRdF)(8,6) = + R(2,1) * Dinv(0,0) * R(0,1) - R(2,1) * Dinv(0,1) * R(0,0) - R(2,0) * Dinv(1,0) * R(0,1) + R(2,0) * Dinv(1,1) * R(0,0);
		(*dRdF)(8,7) = + R(2,1) * Dinv(0,0) * R(1,1) - R(2,1) * Dinv(0,1) * R(1,0) - R(2,0) * Dinv(1,0) * R(1,1) + R(2,0) * Dinv(1,1) * R(1,0);
		(*dRdF)(8,8) = + R(2,1) * Dinv(0,0) * R(2,1) - R(2,1) * Dinv(0,1) * R(2,0) - R(2,0) * Dinv(1,0) * R(2,1) + R(2,0) * Dinv(1,1) * R(2,0);
	}
}

template void FtoR(const Eigen::Matrix<float, 3, 3>& F, Eigen::Matrix<float, 3, 3>& R, Eigen::Matrix<float, 9, 9>* dRdF);
template void FtoR(const Eigen::Matrix<double, 3, 3>& F, Eigen::Matrix<double, 3, 3>& R, Eigen::Matrix<double, 9, 9>* dRdF);


template <class T>
void TetConstraints<T>::SetRestPose(const Eigen::Matrix<T, 3, -1>& vertices, bool allowInvertedTets)
{
	const int numTets = int(m_tets.cols());
	m_numVertices = int(vertices.cols());

	m_invRestFrame.resize(size_t(numTets));
	m_sqrtRestVolume.resize(size_t(numTets));

	for (int t = 0; t < numTets; t++) {
		const int v[4] = { m_tets(0, t), m_tets(1, t), m_tets(2, t), m_tets(3, t) };
		const Eigen::Vector3<T>& v0 = vertices.col(v[0]);
		const Eigen::Vector3<T>& v1 = vertices.col(v[1]);
		const Eigen::Vector3<T>& v2 = vertices.col(v[2]);
		const Eigen::Vector3<T>& v3 = vertices.col(v[3]);

		Eigen::Matrix<T, 3, 3> restFrame;
		restFrame.col(0) = v1 - v0;
		restFrame.col(1) = v2 - v0;
		restFrame.col(2) = v3 - v0;

		const T restDet = restFrame.determinant();
		if (!allowInvertedTets && restDet < T(1e-9)) CARBON_CRITICAL("Tet with tiny or even negative volume in the rest pose");
		m_sqrtRestVolume[t] = sqrt(abs(restDet) / T(6.0));
		if (m_sqrtRestVolume[t] > 1e-12) {
			m_invRestFrame[t] = restFrame.inverse();
		} else {
			m_invRestFrame[t].setZero();
		}
	}
}

template <class T>
DiffData<T> TetConstraints<T>::EvaluateStrain(const DiffDataMatrix<T, 3, -1>& vertices, const T strainWeight) const
{
	const int numTets = int(m_tets.cols());
	if (vertices.Cols() != m_numVertices) throw std::runtime_error("Incorrect number of vertices");
	if (int(m_invRestFrame.size()) != numTets) throw std::runtime_error("Incorrect number of tets");

	VectorPtr<T> outputValue = std::make_shared<Vector<T>>(numTets * 9);
	std::vector<Eigen::Triplet<T>> triplets;
	if (vertices.HasJacobian()) triplets.reserve(size_t(numTets) * 36);

	const T strainWeightSqrt = sqrt(strainWeight);

	for (int t = 0; t < numTets; t++) {
		const int v[4] = { m_tets(0, t), m_tets(1, t), m_tets(2, t), m_tets(3, t) };

		const Eigen::Vector3<T>& v0 = vertices.Matrix().col(v[0]);
		const Eigen::Vector3<T>& v1 = vertices.Matrix().col(v[1]);
		const Eigen::Vector3<T>& v2 = vertices.Matrix().col(v[2]);
		const Eigen::Vector3<T>& v3 = vertices.Matrix().col(v[3]);

		Eigen::Matrix<T, 3, 3> currFrame;
		currFrame.col(0) = v1 - v0;
		currFrame.col(1) = v2 - v0;
		currFrame.col(2) = v3 - v0;

		const Eigen::Matrix<T, 3, 3> F = currFrame * m_invRestFrame[t];
		Eigen::Matrix<T, 9, 9> dRdF;
		Eigen::Matrix<T, 3, 3> R;
		FtoR(F, R, vertices.HasJacobian() ? &dRdF : nullptr);

		const T coefficient = strainWeightSqrt * m_sqrtRestVolume[t];
		const Eigen::Matrix<T, 3, 3> residual = coefficient * (F - R);
		outputValue->segment(9 * t, 9) = Eigen::Map<const Eigen::Vector<T, 9>>(residual.data(), residual.size());

		if (vertices.HasJacobian()) {
			Eigen::Matrix<T, 9, 9> dResdF = Eigen::Matrix<T, 9, 9>::Identity() - dRdF;

			for (int i = 0; i < 3; i++) { // dv1, dv2, dv3
				Eigen::Matrix<T, 9, 3> dFdx = Eigen::Matrix<T, 9, 3>::Zero();
				for (int j = 0; j < 3; j++) { // x, y, z
					for (int c = 0; c < 3; c++) {
						dFdx(3 * j + c, c) = coefficient * m_invRestFrame[t](i, j);
					}
				}
				const Eigen::Matrix<T, 9, 3> dResdx = dResdF * dFdx;
				for (int k = 0; k < 9; k++) { // x, y, z
					for (int c = 0; c < 3; c++) {
						triplets.push_back(Eigen::Triplet<T>(9 * t + k, 3 * v[i + 1] + c, dResdx(k, c)));
					}
				}
			}
			// dv0 is special:
			{
				const Eigen::Vector<T, 3> sum = Eigen::Matrix<T, 1, 3>(T(-1.0), T(-1.0), T(-1.0)) * coefficient * m_invRestFrame[t];
				Eigen::Matrix<T, 9, 3> dFdx = Eigen::Matrix<T, 9, 3>::Zero();
				for (int j = 0; j < 3; j++) { // x, y, z
					for (int c = 0; c < 3; c++) {
						dFdx(3 * j + c, c) = sum[j];
					}
				}
				const Eigen::Matrix<T, 9, 3> dResdx = dResdF * dFdx;
				for (int k = 0; k < 9; k++) { // x, y, z
					for (int c = 0; c < 3; c++) {
						triplets.push_back(Eigen::Triplet<T>(9 * t + k, 3 * v[0] + c, dResdx(k, c)));
					}
				}
			}
		}
	}

	JacobianConstPtr<T> Jacobian;

	if (vertices.HasJacobian())
	{
		SparseMatrix<T> localJacobian(int(outputValue->size()), vertices.Size());
		localJacobian.setFromTriplets(triplets.begin(), triplets.end());
		Jacobian = vertices.Jacobian().Premultiply(localJacobian);
	}

	return DiffData<T>(outputValue, Jacobian);
}


template <class T>
DiffData<T> TetConstraints<T>::EvaluateStrainLinearProjective(const DiffDataMatrix<T, 3, -1>& vertices, const T strainWeight, ElasticityModel elModel) const
{
	const int numTets = int(m_tets.cols());
	if (vertices.Cols() != m_numVertices) throw std::runtime_error("Incorrect number of vertices");
	if (int(m_invRestFrame.size()) != numTets) throw std::runtime_error("Incorrect number of tets");

	VectorPtr<T> outputValue = std::make_shared<Vector<T>>(numTets * 9);
	std::vector<Eigen::Triplet<T>> triplets;
	if (vertices.HasJacobian()) triplets.reserve(size_t(numTets) * 36);

	const T strainWeightSqrt = sqrt(strainWeight);

	for (int t = 0; t < numTets; t++) {
		const int v[4] = { m_tets(0, t), m_tets(1, t), m_tets(2, t), m_tets(3, t) };

		const Eigen::Vector3<T>& v0 = vertices.Matrix().col(v[0]);
		const Eigen::Vector3<T>& v1 = vertices.Matrix().col(v[1]);
		const Eigen::Vector3<T>& v2 = vertices.Matrix().col(v[2]);
		const Eigen::Vector3<T>& v3 = vertices.Matrix().col(v[3]);

		Eigen::Matrix<T, 3, 3> currFrame;
		currFrame.col(0) = v1 - v0;
		currFrame.col(1) = v2 - v0;
		currFrame.col(2) = v3 - v0;

		const Eigen::Matrix<T, 3, 3> F = currFrame * m_invRestFrame[t];
		Eigen::Matrix<T, 3, 3> Fdash;
		if (elModel == Corotated) {
			const Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
			// Eigen::Vector3<T> S = svd.singularValues(); // => switch once we support clamping
			// constexpr T minRange = T(0.9);
			// constexpr T maxRange = T(1.1);
			// S[0] = std::clamp(S[0], minRange, maxRange);
			// S[1] = std::clamp(S[1], minRange, maxRange);
			// S[2] = std::clamp(S[2], minRange, maxRange);
			Eigen::Vector3<T> S = Eigen::Vector3<T>::Ones();
			if (F.determinant() < 0) {
				// F is a reflection, so we need to invert the matrix
				S[2] = -S[2];
			}
			Fdash = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
		}
		else if (elModel == Linear) {
			Fdash = Eigen::Matrix<T, 3, 3>::Identity();
		}

		const T coefficient = strainWeightSqrt * m_sqrtRestVolume[t];
		const Eigen::Matrix<T, 3, 3> residual = coefficient * (F - Fdash);
		outputValue->segment(9 * t, 9) = Eigen::Map<const Eigen::Vector<T, 9>>(residual.data(), residual.size());

		if (vertices.HasJacobian()) {
			for (int i = 0; i < 3; i++) { // dv1, dv2, dv3
				for (int j = 0; j < 3; j++) { // x, y, z
					for (int c = 0; c < 3; c++)
						triplets.push_back(Eigen::Triplet<T>(9 * t + 3 * j + c, 3 * v[i + 1] + c, coefficient * m_invRestFrame[t](i, j)));
				}
			}
			// dv0 is special:
			const Eigen::Vector<T, 3> sum = Eigen::Matrix<T, 1, 3>(T(-1.0), T(-1.0), T(-1.0)) * coefficient * m_invRestFrame[t];
			for (int j = 0; j < 3; j++) { // x, y, z
				for (int c = 0; c < 3; c++)
					triplets.push_back(Eigen::Triplet<T>(9 * t + 3 * j + c, 3 * v[0] + c, sum[j]));
			}
		}
	}

	JacobianConstPtr<T> Jacobian;

	if (vertices.HasJacobian())
	{
		SparseMatrix<T> localJacobian(int(outputValue->size()), vertices.Size());
		localJacobian.setFromTriplets(triplets.begin(), triplets.end());
		Jacobian = vertices.Jacobian().Premultiply(localJacobian);
	}

	return DiffData<T>(outputValue, Jacobian);
}

template <class T>
DiffData<T> TetConstraints<T>::EvaluateVolumeLoss(const DiffDataMatrix<T, 3, -1>& vertices, T volumeWeight) const
{
	const int numTets = int(m_tets.cols());
	if (vertices.Cols() != m_numVertices) throw std::runtime_error("Incorrect number of vertices");
	if (int(m_invRestFrame.size()) != numTets) throw std::runtime_error("Incorrect number of tets");

	VectorPtr<T> outputValue = std::make_shared<Vector<T>>(numTets);
	std::vector<Eigen::Triplet<T>> triplets;
	if (vertices.HasJacobian()) triplets.reserve(size_t(numTets) * 36);

	const T volumeWeightSqrt = sqrt(volumeWeight);

	for (int t = 0; t < numTets; t++) {
		const int v[4] = { m_tets(0, t), m_tets(1, t), m_tets(2, t), m_tets(3, t) };

		const Eigen::Vector3<T>& v0 = vertices.Matrix().col(v[0]);
		const Eigen::Vector3<T>& v1 = vertices.Matrix().col(v[1]);
		const Eigen::Vector3<T>& v2 = vertices.Matrix().col(v[2]);
		const Eigen::Vector3<T>& v3 = vertices.Matrix().col(v[3]);

		Eigen::Matrix<T, 3, 3> currFrame;
		currFrame.col(0) = v1 - v0;
		currFrame.col(1) = v2 - v0;
		currFrame.col(2) = v3 - v0;

		const Eigen::Matrix<T, 3, 3> F = currFrame * m_invRestFrame[t];
		const T V = F.determinant();

		const T coefficient = volumeWeightSqrt * m_sqrtRestVolume[t];
		(*outputValue)[t] = volumeWeightSqrt * m_sqrtRestVolume[t] * (V - T(1));

		if (vertices.HasJacobian()) {
			Eigen::Matrix<T, 3, 3> dVdF;
			dVdF(0, 0) = F(1, 1) * F(2, 2) - F(2, 1) * F(1, 2);
			dVdF(0, 1) = F(2, 0) * F(1, 2) - F(1, 0) * F(2, 2);
			dVdF(0, 2) = F(1, 0) * F(2, 1) - F(2, 0) * F(1, 1);
			dVdF(1, 0) = F(2, 1) * F(0, 2) - F(0, 1) * F(2, 2);
			dVdF(1, 1) = F(0, 0) * F(2, 2) - F(2, 0) * F(0, 2);
			dVdF(1, 2) = F(2, 0) * F(0, 1) - F(0, 0) * F(2, 1);
			dVdF(2, 0) = F(0, 1) * F(1, 2) - F(1, 1) * F(0, 2);
			dVdF(2, 1) = F(1, 0) * F(0, 2) - F(0, 0) * F(1, 2);
			dVdF(2, 2) = F(0, 0) * F(1, 1) - F(1, 0) * F(0, 1);

			for (int i = 0; i < 3; i++) { // dv1, dv2, dv3
				for (int j = 0; j < 3; j++) { // x, y, z
					for (int c = 0; c < 3; c++) {
						triplets.push_back(Eigen::Triplet<T>(t, 3 * v[i + 1] + c, dVdF(c, j) * coefficient * m_invRestFrame[t](i, j)));
					}
				}
			}
			// dv0 is special:
			const Eigen::Vector<T, 3> sum = Eigen::Matrix<T, 1, 3>(T(-1.0), T(-1.0), T(-1.0)) * coefficient * m_invRestFrame[t];
			for (int j = 0; j < 3; j++) { // x, y, z
				for (int c = 0; c < 3; c++) {
					triplets.push_back(Eigen::Triplet<T>(t, 3 * v[0] + c, dVdF(c, j) * sum[j]));
				}
			}
		}
	}

	JacobianConstPtr<T> Jacobian;

	if (vertices.HasJacobian())
	{
		SparseMatrix<T> localJacobian(int(outputValue->size()), vertices.Size());
		localJacobian.setFromTriplets(triplets.begin(), triplets.end());
		Jacobian = vertices.Jacobian().Premultiply(localJacobian);
	}

	return DiffData<T>(outputValue, Jacobian);
}


template <class T>
DiffData<T> TetConstraints<T>::EvaluateVolumeLossProjective(const DiffDataMatrix<T, 3, -1>& vertices, T volumeWeight) const
{
	const int numTets = int(m_tets.cols());
	if (vertices.Cols() != m_numVertices) throw std::runtime_error("Incorrect number of vertices");
	if (int(m_invRestFrame.size()) != numTets) throw std::runtime_error("Incorrect number of tets");

	VectorPtr<T> outputValue = std::make_shared<Vector<T>>(numTets * 9);
	std::vector<Eigen::Triplet<T>> triplets;
	if (vertices.HasJacobian()) triplets.reserve(size_t(numTets) * 36);

	const T volumeWeightSqrt = sqrt(volumeWeight);

	for (int t = 0; t < numTets; t++) {
		const int v[4] = { m_tets(0, t), m_tets(1, t), m_tets(2, t), m_tets(3, t) };

		const Eigen::Vector3<T>& v0 = vertices.Matrix().col(v[0]);
		const Eigen::Vector3<T>& v1 = vertices.Matrix().col(v[1]);
		const Eigen::Vector3<T>& v2 = vertices.Matrix().col(v[2]);
		const Eigen::Vector3<T>& v3 = vertices.Matrix().col(v[3]);

		Eigen::Matrix<T, 3, 3> currFrame;
		currFrame.col(0) = v1 - v0;
		currFrame.col(1) = v2 - v0;
		currFrame.col(2) = v3 - v0;

		const Eigen::Matrix<T, 3, 3> F = currFrame * m_invRestFrame[t];
        const Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Vector3<T> S = svd.singularValues();

		constexpr int innerIterations = 4;
		Eigen::Vector3<T> d = Eigen::Vector3<T>::Zero();
		for (int i = 0; i < innerIterations; ++i) {
			const T currentVolume = S[0] * S[1] * S[2];
			const T f = currentVolume - T(1);//clamp(v, rangeMin_, rangeMax_);
			Eigen::Vector3<T> g(S[1]*S[2], S[0]*S[2], S[0]*S[1]);
			d = -((f - g.dot(d)) / g.dot(g)) * g;
			S = svd.singularValues() + d;
		}
		if (F.determinant() < 0) {
			// F is a reflection, so we need to invert the matrix
			S[2] = -S[2];
		}
		const Eigen::Matrix<T, 3, 3> Fdash = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();

		const T coefficient = volumeWeightSqrt * m_sqrtRestVolume[t];
		const Eigen::Matrix<T, 3, 3> residual = coefficient * (F - Fdash);
		outputValue->segment(9 * t, 9) = Eigen::Map<const Eigen::Vector<T, 9>>(residual.data(), residual.size());

		if (vertices.HasJacobian()) {
			for (int i = 0; i < 3; i++) { // dv1, dv2, dv3
				for (int j = 0; j < 3; j++) { // x, y, z
					for (int c = 0; c < 3; c++)
						triplets.push_back(Eigen::Triplet<T>(9 * t + 3 * j + c, 3 * v[i + 1] + c, coefficient * m_invRestFrame[t](i, j)));
				}
			}
			// dv0 is special:
			const Eigen::Vector<T, 3> sum = Eigen::Matrix<T, 1, 3>(T(-1.0), T(-1.0), T(-1.0)) * coefficient * m_invRestFrame[t];
			for (int j = 0; j < 3; j++) { // x, y, z
				for (int c = 0; c < 3; c++)
					triplets.push_back(Eigen::Triplet<T>(9 * t + 3 * j + c, 3 * v[0] + c, sum[j]));
			}
		}
	}

	JacobianConstPtr<T> Jacobian;

	if (vertices.HasJacobian())
	{
		SparseMatrix<T> localJacobian(int(outputValue->size()), vertices.Size());
		localJacobian.setFromTriplets(triplets.begin(), triplets.end());
		Jacobian = vertices.Jacobian().Premultiply(localJacobian);
	}

	return DiffData<T>(outputValue, Jacobian);
}

template <class T>
DiffDataMatrix<T, 9, -1> TetConstraints<T>::EvaluateDeformationGradient(const DiffDataMatrix<T, 3, -1>& vertices, bool volumeWeighted, const std::vector<T>& perTetWeight) const
{
	const int numTets = int(m_tets.cols());
	if (vertices.Cols() != m_numVertices) CARBON_CRITICAL("Incorrect number of vertices");
	if (int(m_invRestFrame.size()) != numTets) CARBON_CRITICAL("Incorrect number of tets");

	VectorPtr<T> outputValue = std::make_shared<Vector<T>>(numTets * 9);
	std::vector<Eigen::Triplet<T>> triplets;
	if (vertices.HasJacobian()) triplets.reserve(size_t(numTets) * 36);

	for (int t = 0; t < numTets; t++) {
		const int v[4] = { m_tets(0, t), m_tets(1, t), m_tets(2, t), m_tets(3, t) };

		const Eigen::Vector3<T>& v0 = vertices.Matrix().col(v[0]);
		const Eigen::Vector3<T>& v1 = vertices.Matrix().col(v[1]);
		const Eigen::Vector3<T>& v2 = vertices.Matrix().col(v[2]);
		const Eigen::Vector3<T>& v3 = vertices.Matrix().col(v[3]);

		Eigen::Matrix<T, 3, 3> currFrame;
		currFrame.col(0) = v1 - v0;
		currFrame.col(1) = v2 - v0;
		currFrame.col(2) = v3 - v0;

		const Eigen::Matrix<T, 3, 3> F = currFrame * m_invRestFrame[t];

		T coefficient = volumeWeighted ? m_sqrtRestVolume[t] : T(1);
		if (static_cast<int>(perTetWeight.size()) == numTets) {
			coefficient *= perTetWeight[t];
		}
		const Eigen::Matrix<T, 3, 3> residual = coefficient * F;
		outputValue->segment(9 * t, 9) = Eigen::Map<const Eigen::Vector<T, 9>>(residual.data(), residual.size());

		if (vertices.HasJacobian()) {
			for (int i = 0; i < 3; i++) { // dv1, dv2, dv3
				for (int j = 0; j < 3; j++) { // x, y, z
					for (int c = 0; c < 3; c++)
						triplets.push_back(Eigen::Triplet<T>(9 * t + 3 * j + c, 3 * v[i + 1] + c, coefficient * m_invRestFrame[t](i, j)));
				}
			}
			// dv0 is special:
			const Eigen::Vector<T, 3> sum = Eigen::Matrix<T, 1, 3>(T(-1.0), T(-1.0), T(-1.0)) * coefficient * m_invRestFrame[t];
			for (int j = 0; j < 3; j++) { // x, y, z
				for (int c = 0; c < 3; c++)
					triplets.push_back(Eigen::Triplet<T>(9 * t + 3 * j + c, 3 * v[0] + c, sum[j]));
			}
		}
	}

	JacobianConstPtr<T> Jacobian;

	if (vertices.HasJacobian())
	{
		SparseMatrix<T> localJacobian(int(outputValue->size()), vertices.Size());
		localJacobian.setFromTriplets(triplets.begin(), triplets.end());
		Jacobian = vertices.Jacobian().Premultiply(localJacobian);
	}

	return DiffDataMatrix<T, 9, -1>(9, numTets, DiffData<T>(outputValue, Jacobian));
}

template class TetConstraints<float>;
template class TetConstraints<double>;

} // namespace epic::nls
