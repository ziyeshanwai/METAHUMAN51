// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/Cost.h>
#include <carbon/utils/Profiler.h>
#include <nls/functions/ConcatenateFunction.h>

namespace epic::nls {

template <class T>
DiffData<T> Cost<T>::CostToDiffData() const
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

	if (m_costTerms.size() == 0) {
		return DiffData<T>(Vector<T>());
	}

	if (m_costTerms.size() == 1) {
		if (m_costTerms[0].weight == T(1)) {
			return m_costTerms[0].diffdata;
		}
        const CostTerm& costTerm = m_costTerms[0];
        const T sqrtWeight = std::sqrt(costTerm.weight);
		if (costTerm.diffdata.HasJacobian()) {
			return DiffData<T>(std::make_shared<Vector<T>>(sqrtWeight * costTerm.diffdata.Value()), costTerm.diffdata.Jacobian().Scale(sqrtWeight));
		} else {
			return DiffData<T>(std::make_shared<Vector<T>>(sqrtWeight * costTerm.diffdata.Value()));
		}
	}

	// Get total number of rows and the maximum column size. Remember that not all Jacobians
	// must have the same column size. However each column represents the same variable assuming
	// the mapping has been done with the same Context.
	int totalRows = 0;
	int maxCols = 0;
	std::vector<int> rowOffsets;
	bool anyJacobian = false;
	for (const CostTerm& costTerm : m_costTerms) {
		rowOffsets.push_back(totalRows);
		totalRows += costTerm.diffdata.Size();
		if (costTerm.diffdata.HasJacobian()) {
			maxCols = std::max<int>(costTerm.diffdata.Jacobian().Cols(), maxCols);
			anyJacobian = true;
		}
	}

	// scale the values
	VectorPtr<T> values = std::make_shared<Vector<T>>(totalRows);
	for (int i = 0; i < NumTerms(); ++i)  {
		values->segment(rowOffsets[i], m_costTerms[i].diffdata.Value().size()) = std::sqrt(m_costTerms[i].weight) * m_costTerms[i].diffdata.Value();
	}

	JacobianConstPtr<T> Jacobian;

	if (anyJacobian) {
		// we only need to concatenate the jacobians if there is any jacobian in the terms
		std::vector<SparseMatrixConstPtr<T>> jacobians;
		for (int i = 0; i < NumTerms(); ++i) {
			const CostTerm& costTerm = m_costTerms[i];
			const int rows = costTerm.diffdata.Size();
			if (costTerm.diffdata.HasJacobian()) {
				SparseMatrixConstPtr<T> resizedJacobian;
				if (costTerm.weight == T(1)) {
					if (costTerm.diffdata.Jacobian().Cols() != maxCols) {
						// just resize
						SparseMatrixPtr<T> jacobian = std::make_shared<SparseMatrix<T>>(*costTerm.diffdata.Jacobian().AsSparseMatrix());
						// resize so that cols match for all jacobians
						jacobian->conservativeResize(rows, maxCols);
						resizedJacobian = jacobian;
					} else {
						resizedJacobian = costTerm.diffdata.Jacobian().AsSparseMatrix();
					}
				} else {
					// resize and scale
					SparseMatrixPtr<T> scaledJacobian = std::make_shared<SparseMatrix<T>>(*costTerm.diffdata.Jacobian().Scale(std::sqrt(costTerm.weight))->AsSparseMatrix());
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

template class Cost<float>;
template class Cost<double>;

} // namespace epic::nls
