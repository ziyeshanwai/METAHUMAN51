// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffData.h>
#include <nls/math/Math.h>
#include <carbon/utils/Profiler.h>

#include <vector>

namespace epic::nls {

/**
 * Concatenation to combine a set of residuals to model the energy E = || concatenate(x) ||^2 =  w1 || r1(x) ||^2 + w2 || r2(x) ||^2 + ...
 * Therefore,
 * concatenate(x) = | sqrt(w1) r1(x) |
 * 		  	        | sqrt(w2) r2(x) |
 * 		  	        | ...      	     |
 */
template <class T>
DiffData<T> ConcatenateDiffData(const std::vector<DiffData<T>>& inputs, const std::vector<T>& weights)
{
	PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

	CARBON_PRECONDITION(inputs.size() == weights.size(), "number of inputs and weights need to match");

	if (inputs.size() == 0) {
		return DiffData<T>(Vector<T>());
	}

	if (inputs.size() == 1) {
		if (weights[0] == T(1)) {
			return inputs[0];
		}
		if (inputs[0].HasJacobian()) {
			return DiffData<T>(std::make_shared<Vector<T>>(std::sqrt(weights[0]) * inputs[0].Value()), inputs[0].Jacobian().Scale(std::sqrt(weights[0])));
		} else {
			return DiffData<T>(std::make_shared<Vector<T>>(std::sqrt(weights[0]) * inputs[0].Value()));
		}
	}

	// Get total number of rows and the maximum column size. Remember that not all Jacobians
	// must have the same column size. However each column represents the same variable assuming
	// the mapping has been done with the same Context.
	int totalRows = 0;
	int maxCols = 0;
	std::vector<int> rowOffsets;
	bool anyJacobian = false;
	for (const DiffData<T>& input : inputs) {
		rowOffsets.push_back(totalRows);
		totalRows += int(input.Value().size());
		if (input.HasJacobian()) {
			maxCols = std::max<int>(int(input.Jacobian().Cols()), maxCols);
			anyJacobian = true;
		}
	}

	// scale the values
	VectorPtr<T> values = std::make_shared<Vector<T>>(totalRows);
	for (int i = 0; i < int(inputs.size()); i++)  {
		values->segment(rowOffsets[i], inputs[i].Value().size()) = std::sqrt(weights[i]) * inputs[i].Value();
	}

	JacobianConstPtr<T> Jacobian;

	if (anyJacobian) {
		// we only need to concatenate the jacobians if there is any jacobian in the terms
		std::vector<SparseMatrixConstPtr<T>> jacobians;
		for (int i = 0; i < int(inputs.size()); i++) {
			const DiffData<T>& input = inputs[i];
			const int rows = int(input.Value().size());
			if (input.HasJacobian()) {
				SparseMatrixConstPtr<T> resizedJacobian;
				if (weights[i] == T(1)) {
					if (input.Jacobian().Cols() != maxCols) {
						// just resize
						SparseMatrixPtr<T> jacobian = std::make_shared<SparseMatrix<T>>(*input.Jacobian().AsSparseMatrix());
						// resize so that cols match for all jacobians
						jacobian->conservativeResize(rows, maxCols);
						resizedJacobian = jacobian;
					} else {
						resizedJacobian = input.Jacobian().AsSparseMatrix();
					}
				} else {
					// resize and scale
					SparseMatrixPtr<T> scaledJacobian = std::make_shared<SparseMatrix<T>>(*input.Jacobian().Scale(std::sqrt(weights[i]))->AsSparseMatrix());
					CARBON_ASSERT(scaledJacobian->cols() <= maxCols, "number of columns of resize jacobian need to be smaller or equal the total number of columns");
					// resize so that cols match for all jacobians
					scaledJacobian->conservativeResize(rows, maxCols);
					resizedJacobian = scaledJacobian;
				}
				CARBON_ASSERT(resizedJacobian->rows() == rows, "rows of resized jacobian need to match value size");
				CARBON_ASSERT(resizedJacobian->cols() == maxCols, "number of columns of resized jacobian need to be equal the total number of columns");
				jacobians.push_back(resizedJacobian);
			} else {
				jacobians.push_back(std::make_shared<SparseMatrix<T>>(rows, maxCols));
			}
		}

		// concatenate the jacobians
		std::vector<std::reference_wrapper<const SparseMatrix<T>>> jacobianRefs;
		for (int i = 0; i < int(jacobians.size()); i++) {
			jacobianRefs.push_back(*jacobians[i]);
		}
		SparseMatrixPtr<T> concatenatedJacobian = std::make_shared<SparseMatrix<T>>();
		// TODO: potential optimization - put scaling and resizing into ConcatenateSparseMatricesAlongRowDimension
		ConcatenateSparseMatricesAlongRowDimension<T>(jacobianRefs, *concatenatedJacobian);
		Jacobian = std::make_shared<SparseJacobian<T>>(concatenatedJacobian);
	}

	return DiffData<T>(values, Jacobian);
}

} //namespace epic::nls
