// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/common/EigenDenseBackwardsCompatible.h>

#include <cmath>
#include <memory>

NLS_DISABLE_EIGEN_WARNINGS
#include <Eigen/Sparse>
NLS_RENABLE_WARNINGS

namespace epic {
namespace nls {

template <class T>
using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template <class T>
using VectorPtr = std::shared_ptr<Vector<T>>;

template <class T>
using VectorConstPtr = std::shared_ptr<const Vector<T>>;

template <class T>
using SparseMatrix = Eigen::SparseMatrix<T, Eigen::RowMajor>;

template <class T>
using SparseMatrixPtr = std::shared_ptr<SparseMatrix<T>>;

template <class T>
using SparseMatrixConstPtr = std::shared_ptr<const SparseMatrix<T>>;


//! basic implementation of a uniform random variable between 0 and 1
template<typename T> inline T random() { return T(std::rand())/T(RAND_MAX) ; }

//! clamp value
template <typename T> inline T clamp(T value, T minValue, T maxValue)
{
	if (value < minValue) return minValue;
	else if (value < maxValue) return value;
	return maxValue;
}

//! acos ensuring that input is clamped to -1
template <typename T> inline T clamped_acos(T value)
{
	if (value <= -1) {
		return T(CARBON_PI);
	} else if (value >= 1) {
		return 0;
	} else {
		return acos(value);
	}
}

//! cotangent between vector a and b
template <typename T>
T cotangent(const Eigen::Vector3<T>& a, const Eigen::Vector3<T>& b)
{
    const T cosAB = a.dot(b);
    const T sinAB = std::sqrt(a.dot(a) * b.dot(b) - cosAB * cosAB);
    return sinAB > 0 ? cosAB / sinAB : std::numeric_limits<T>::max();
}

//! @returns True if the sparse matrix is an identity matrix.
template <class T>
static bool isIdentityMatrix(const SparseMatrix<T>& mat)
{
    bool isIdentity = (mat.rows() == mat.cols()) && (mat.rows() == mat.nonZeros());
    for (size_t i = 0; i < int(mat.rows()) && isIdentity; i++) {
        isIdentity = (mat.valuePtr()[i] == T(1) && mat.innerIndexPtr()[i] == i);
    }
    return isIdentity;
}

/**
 * Calculates the cross product matrix such that for the cross product c = a x b,
 * we have c = CrossProductMatrix(a) * b
 *
 * CrossProductMatrix([x y z]^T) = |  0  -z   y  |
 *          			   		   |  z   0  -x  |
 *          			   		   | -y   x   0  |
 */
template <class T>
Eigen::Matrix<T,3,3> CrossProductMatrix(const Eigen::Vector<T,3>& vec)
{
	Eigen::Matrix<T,3,3> mat;
	mat.coeffRef(0,0) = 0;
	mat.coeffRef(1,0) = vec[2];
	mat.coeffRef(2,0) = -vec[1];
	mat.coeffRef(0,1) = -vec[2];
	mat.coeffRef(1,1) = 0;
	mat.coeffRef(2,1) = vec[0];
	mat.coeffRef(0,2) = vec[1];
	mat.coeffRef(1,2) = -vec[0];
	mat.coeffRef(2,2) = 0;
	return mat;
}

/**
 * Adds two compressed sparse CRS matrices. Both matrices need to have the same number of rows, but the columns may differ. In this case
 * the output matrix will have the larger of the two column dimensions and the smaller of the two matrices is considered to be
 * filled with zeros.
 * A: (rows, cols_a)
 * B: (rows, cols_b)
 * output: (rows, max(cols_a, cols_b))
 */
template <class T>
SparseMatrix<T> AddSparseMatricesAndPadColumns(const SparseMatrix<T>& A,
											   const SparseMatrix<T>& B,
											   bool squeeze)
{
	static_assert(SparseMatrix<T>::IsRowMajor);
	CARBON_PRECONDITION(A.rows() == B.rows(), "number of rows of matrix A and B need to match, but {} != {}", A.rows(), B.rows());
	CARBON_PRECONDITION(A.isCompressed(), "A needs to be compressed");
	CARBON_PRECONDITION(B.isCompressed(), "B needs to be compressed");

	const int rows = int(A.rows());
	const int cols = std::max<int>(int(A.cols()), int(B.cols()));

	// check trivial additions with empty matrices - in this case we only pad the matrix
	if (A.nonZeros() == 0) {
		SparseMatrix<T> output = B;
		output.conservativeResize(rows, cols);
		if (squeeze) output.data().squeeze();
		return output;
	}
	if (B.nonZeros() == 0) {
		SparseMatrix<T> output = A;
		output.conservativeResize(rows, cols);
		if (squeeze) output.data().squeeze();
		return output;
	}

	// estimate the maximum number of nonzeros
	const int maxNonZeros = int(std::min<uint64_t>(uint64_t(A.nonZeros() + B.nonZeros()), uint64_t(rows) * uint64_t(cols)));

	CARBON_PRECONDITION(rows > 0, "number of rows needs to be positive");
	CARBON_PRECONDITION(cols > 0, "number of columns needs to be positive");
	CARBON_PRECONDITION(maxNonZeros > 0, "number of maximum of nonzeros needs to be positive");

	SparseMatrix<T> output(rows, cols);
	output.resizeNonZeros(maxNonZeros);

	const int* aRowIndices = A.outerIndexPtr();
	const int* bRowIndices = B.outerIndexPtr();
	const int* aColIndices = A.innerIndexPtr();
	const int* bColIndices = B.innerIndexPtr();
	const T* aValues = A.valuePtr();
	const T* bValues = B.valuePtr();

	int* outputRowIndices = output.outerIndexPtr();
	int* outputColIndices = output.innerIndexPtr();
	T* outputValues = output.valuePtr();
	int outputCurrIndex = 0;

	for (int r = 0; r < rows; r++) {
		outputRowIndices[r] = outputCurrIndex;
		const int startA = aRowIndices[r];
		const int endA = aRowIndices[r+1];
		const int startB = bRowIndices[r];
		const int endB = bRowIndices[r+1];
		int currA = startA;
		int currB = startB;
		while (currA < endA && currB < endB) {
			const int aColIndex = aColIndices[currA];
			const int bColIndex = bColIndices[currB];
			if (aColIndex == bColIndex) {
				outputColIndices[outputCurrIndex] = aColIndex;
				outputValues[outputCurrIndex] = aValues[currA] + bValues[currB];
				currA++;
				currB++;
			}
			else if (aColIndex < bColIndex)
			{
				outputColIndices[outputCurrIndex] = aColIndex;
				outputValues[outputCurrIndex] = aValues[currA];
				currA++;
			}
			else // aColIndex > bColIndex
			{
				outputColIndices[outputCurrIndex] = bColIndex;
				outputValues[outputCurrIndex] = bValues[currB];
				currB++;

			}
			outputCurrIndex++;
		}
		if (currA < endA) {
			const int restOfA = endA - currA;
			memcpy(outputColIndices + outputCurrIndex, aColIndices + currA, restOfA * sizeof(int));
			memcpy(outputValues + outputCurrIndex, aValues + currA, restOfA * sizeof(T));
			outputCurrIndex += restOfA;
			currA = endA;
		}
		if (currB < endB) {
			const int restOfB = endB - currB;
			memcpy(outputColIndices + outputCurrIndex, bColIndices + currB, restOfB * sizeof(int));
			memcpy(outputValues + outputCurrIndex, bValues + currB, restOfB * sizeof(T));
			outputCurrIndex += restOfB;
			currB = endB;
		}
	}
	outputRowIndices[rows] = outputCurrIndex;

	if (squeeze) output.data().squeeze();

	return output;
}


/**
 * Create a new sparse matrix by repeating the rows of the input matrix @p n times
 * output =  | A |
 * 		  | A |
 * 		  | ... |
 * 		  | A |
 */
template <class T>
void RepeatRowsOfSparseMatrix(const SparseMatrix<T>& A,
							  SparseMatrix<T>& output,
							  int n)
{
	static_assert(SparseMatrix<T>::IsRowMajor);
	CARBON_PRECONDITION(A.isCompressed(), "A needs to be compressed");

	const int rows = int(A.rows());
	const int cols = int(A.cols());
	const int nonZeros = int(A.nonZeros());

	output.resize(rows * n, cols);
	output.setZero();
	output.makeCompressed();

	if (nonZeros == 0 || n == 0) {
		return;
	}

	output.resizeNonZeros(n * nonZeros);

	const int* aRowIndices = A.outerIndexPtr();
	const int* aColIndices = A.innerIndexPtr();
	const T* aValues = A.valuePtr();

	int* outputRowIndices = output.outerIndexPtr();
	int* outputColIndices = output.innerIndexPtr();
	T* outputValues = output.valuePtr();

	// copy more and more tiles at the same time
	if (n > 0) {
		memcpy(outputValues, aValues, sizeof(T) * nonZeros);
		memcpy(outputColIndices, aColIndices, sizeof(int) * nonZeros);

		int copiedTiles = 1;
		while (copiedTiles < n) {
			const int tilesToCopy = n - copiedTiles;
			const int toCopy = std::min<int>(tilesToCopy, copiedTiles);
			memcpy(outputValues + copiedTiles * nonZeros, outputValues, sizeof(T) * nonZeros * toCopy);
			memcpy(outputColIndices + copiedTiles * nonZeros, outputColIndices, sizeof(int) * nonZeros * toCopy);
			copiedTiles += toCopy;
		}
	}

	// copy one tile at a time
//	for (int i = 0; i < n; i++) {
//		memcpy(outputValues + i * nonZeros, aValues, sizeof(T) * nonZeros);
//		memcpy(outputColIndices + i * nonZeros, aColIndices, sizeof(int) * nonZeros);
//	}

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < rows; j++) {
			outputRowIndices[i * rows + j] = aRowIndices[j] + i * nonZeros;
		}
	}

	outputRowIndices[n * rows] = n * nonZeros;
}

/**
 * Concatenates a set of sparse matrices along the row dimension. Input matrices may be compressed or uncompressed, the output is always
 * a compressed sparse matrix.
 * Precondition: all input matrices need to be valid (no nullptr) and have the same number of cols.
*/
template <class T>
void ConcatenateSparseMatricesAlongRowDimension(const std::vector<std::reference_wrapper<const SparseMatrix<T>>>& inputs,
												SparseMatrix<T>& output)
{
	static_assert(SparseMatrix<T>::IsRowMajor);

	if (inputs.size() == 0) {
		output = SparseMatrix<T>();
		return;
	}
	else if (inputs.size() == 1)
	{
		output = inputs[0];
	}
	else
	{
		int totalRows = 0;
		int totalNonZeros = 0;
		const int cols = int(inputs.front().get().cols());
		for (int i = 0; i < int(inputs.size()); i++) {
			CARBON_PRECONDITION(inputs[i].get().cols() == cols, "all inputs need to have the same number of columns");
			totalRows += int(inputs[i].get().rows());
			totalNonZeros += int(inputs[i].get().nonZeros());
		}

		// allocate the output data structure
		output.resize(totalRows, cols);
		output.resizeNonZeros(totalNonZeros);
		CARBON_POSTCONDITION(output.isCompressed(), "output needs to be compressed");

		int* outputRowIndices = output.outerIndexPtr();
		int* outputColIndices = output.innerIndexPtr();
		T* outputValues = output.valuePtr();
		int outputCurrIndex = 0;
		int outputRowIndex = 0;

		// go input by input and then copy rows
		for (int i = 0; i < int(inputs.size()); i++) {
			const SparseMatrix<T>& input = inputs[i].get();

			const int* inputRowIndices = input.outerIndexPtr();
			const int* inputRowNumNonZeros = input.innerNonZeroPtr();
			const int* inputColIndices = input.innerIndexPtr();
			const T* inputValues = input.valuePtr();

			if (input.isCompressed())
			{
				// copy all values and row indices
				const int numElements = int(input.nonZeros());
				memcpy(outputColIndices + outputCurrIndex, inputColIndices, numElements * sizeof(int));
				memcpy(outputValues + outputCurrIndex, inputValues, numElements * sizeof(T));

				// copy the row indices offset appropriately
				for (int r = 0; r < input.rows(); r++) {
					outputRowIndices[outputRowIndex++] = outputCurrIndex + inputRowIndices[r];
				}
				outputCurrIndex += int(input.nonZeros());
			}
			else
			{
				// we need to copy the data row by row
				for (int r = 0; r < input.rows(); r++)
				{
					const int numElements = inputRowNumNonZeros[r];
					memcpy(outputColIndices + outputCurrIndex, inputColIndices + inputRowIndices[r], numElements * sizeof(int));
					memcpy(outputValues + outputCurrIndex, inputValues + inputRowIndices[r], numElements * sizeof(T));

					outputRowIndices[outputRowIndex++] = outputCurrIndex;
					outputCurrIndex += numElements;
				}
			}
		}

		CARBON_ASSERT(outputRowIndex == totalRows, "row index increment needs to match total number of rows");
		outputRowIndices[outputRowIndex] = outputCurrIndex;
		CARBON_ASSERT(outputCurrIndex == totalNonZeros, "element index increment needs to match total number of nonzeros");
	}
}


/**
 * Gathers rows from the sparse matrix.
 * @param blockIndices  The indices indicate which rows to select from the sparse matrix. rowIndex = blockIdx * blockSize
 * @param blockSize		The size of each row block i.e. for blockSize = 1, the blockIndices are equivalent to rowIndices.
 */
template <class T>
SparseMatrix<T> RowGather(const SparseMatrix<T>& inSparse, const Eigen::VectorX<int>& blockIndices, int blockSize = 1)
{
	static_assert(SparseMatrix<T>::IsRowMajor);
	CARBON_PRECONDITION(inSparse.isCompressed(), "input matrix needs to be compressed");

	const int numRows = int(blockIndices.size()) * blockSize;
	SparseMatrix<T> outSparse(numRows, inSparse.cols());
	int totalNonZeros = 0;
	if (inSparse.nonZeros() > 0) {
		// count nonZeros per row
		const int* inputRowIndices = inSparse.outerIndexPtr();
		const int* inputColIndices = inSparse.innerIndexPtr();
		const int* inputRowNumNonZeros = inSparse.innerNonZeroPtr();
		const T* inputValues = inSparse.valuePtr();

		for (int i = 0; i < int(blockIndices.size()); i++) {
            const int blockIdx = blockIndices[i];
            for (int k = 0; k < blockSize; k++) {
                const int inIdx = blockIdx * blockSize + k;
				const int nonZerosInRow = (inSparse.isCompressed() ? (inputRowIndices[inIdx + 1] - inputRowIndices[inIdx]) : inputRowNumNonZeros[inIdx]);
				totalNonZeros += nonZerosInRow;
            }
        }
		outSparse.resizeNonZeros(totalNonZeros);
		int* outputRowIndices = outSparse.outerIndexPtr();
		int* outputColIndices = outSparse.innerIndexPtr();
		T* outputValues = outSparse.valuePtr();
		int counter = 0;
		for (int i = 0; i < int(blockIndices.size()); i++) {
            const int blockIdx = blockIndices[i];
            for (int k = 0; k < blockSize; k++) {
                const int inIdx = blockIdx * blockSize + k;
				const int outIdx = i * blockSize + k;
				outputRowIndices[outIdx] = counter;
				const int nonZerosInRow = (inSparse.isCompressed() ? (inputRowIndices[inIdx + 1] - inputRowIndices[inIdx]) : inputRowNumNonZeros[inIdx]);
				memcpy(outputColIndices + counter, inputColIndices + inputRowIndices[inIdx], sizeof(int) * nonZerosInRow);
				memcpy(outputValues + counter, inputValues + inputRowIndices[inIdx], sizeof(T) * nonZerosInRow);
				counter += nonZerosInRow;
            }
        }
		CARBON_ASSERT(counter == totalNonZeros, "counter increment needs to match total number of nonzeros");
		outputRowIndices[numRows] = counter;
	}

	return outSparse;
}


/**
 * Scatters rows from the sparse matrix.
 * @param blockIndices  The indices indicate where each row of the input matrix is placed in the output matrix.
 * @param blockSize		The size of each row block i.e. for blockSize = 1, the blockIndices are equivalent to rowIndices.
 */
template <class T>
SparseMatrix<T> RowScatter(const SparseMatrix<T>& inSparse, const int outputRows, const Eigen::VectorX<int>& blockIndices, int blockSize = 1)
{
	SparseMatrix<T> outSparse(outputRows, inSparse.cols());
	const int totalNonZeros = int(inSparse.nonZeros());
	if (totalNonZeros > 0) {
		outSparse.resizeNonZeros(totalNonZeros);

		const int* inputRowIndices = inSparse.outerIndexPtr();
		const int* inputColIndices = inSparse.innerIndexPtr();
		const int* inputRowNumNonZeros = inSparse.innerNonZeroPtr();
		const T* inputValues = inSparse.valuePtr();

		int* outputRowIndices = outSparse.outerIndexPtr();
		int* outputColIndices = outSparse.innerIndexPtr();
		T* outputValues = outSparse.valuePtr();

		int counter = 0;
		int currentRowIndex = 0;
		for (int i = 0; i < int(blockIndices.size()); i++) {
            const int blockIdx = blockIndices[i];
            for (int k = 0; k < blockSize; k++) {
                const int inIdx = i * blockSize + k;
				const int outIdx = blockIdx * blockSize + k;
				while (currentRowIndex <= outIdx) {
					outputRowIndices[currentRowIndex++] = counter;
				}
				const int nonZerosInRow = (inSparse.isCompressed() ? (inputRowIndices[inIdx + 1] - inputRowIndices[inIdx]) : inputRowNumNonZeros[inIdx]);
				memcpy(outputColIndices + counter, inputColIndices + inputRowIndices[inIdx], sizeof(int) * nonZerosInRow);
				memcpy(outputValues + counter, inputValues + inputRowIndices[inIdx], sizeof(T) * nonZerosInRow);
				counter += nonZerosInRow;
            }
        }
		CARBON_ASSERT(counter == totalNonZeros, "counter increment needs to match total number of nonzeros");
		while (currentRowIndex <= outputRows) {
			outputRowIndices[currentRowIndex++] = counter;
		}
	}

	return outSparse;
}


/**
 * Removes the empty inner dimensions of a sparse matrix (removes empty rows for row major, and empty columns for column major),
 * and outputs the indices of the old rows/columns to the new rows/columns.
 */
template<typename _Scalar, int _Options, typename _StorageIndex>
void DiscardEmptyInnerDimensions(Eigen::SparseMatrix<_Scalar, _Options, _StorageIndex>& sparse, Eigen::Matrix<int, -1, 1>& indices)
{
	// TODO: create unit test
	CARBON_PRECONDITION(sparse.isCompressed(), "input matrix needs to be compressed");
	const int outerSize = int(sparse.outerSize());
	const int innerSize = int(sparse.innerSize());
	indices.resize(outerSize);

	int newOuterIndex = 0;
	for (int outerIndex = 0; outerIndex < outerSize; outerIndex++) {
		const int numInnerNonZeros = int(sparse.outerIndexPtr()[outerIndex + 1] - sparse.outerIndexPtr()[outerIndex]);
		if (numInnerNonZeros > 0) {
			indices[outerIndex] = newOuterIndex++;
		} else {
			indices[outerIndex] = -1; // will be removed from the new matrix
		}
	}
	const int newOuterSize = newOuterIndex;

	if (newOuterSize < outerSize) {
		// there are inner dimensions that have been discarded
		Eigen::SparseMatrix<_Scalar, _Options, _StorageIndex> newSparse;
		if constexpr (newSparse.IsRowMajor) {
			newSparse.resize(newOuterSize, innerSize);
		} else {
			newSparse.resize(innerSize, newOuterSize);
		}
		newSparse.resizeNonZeros(sparse.nonZeros());

		memcpy(newSparse.valuePtr(), sparse.valuePtr(), sizeof(_Scalar) * sparse.nonZeros());
		memcpy(newSparse.innerIndexPtr(), sparse.innerIndexPtr(), sizeof(_StorageIndex) * sparse.nonZeros());
		for (int outerIndex = 0; outerIndex < outerSize; outerIndex++) {
			newOuterIndex = indices[outerIndex];
			if (newOuterIndex >= 0) {
				newSparse.outerIndexPtr()[newOuterIndex] = sparse.outerIndexPtr()[outerIndex];
			}
		}
		newSparse.outerIndexPtr()[newOuterSize] = sparse.outerIndexPtr()[outerSize];

		sparse.swap(newSparse);
	}
}


/**
 * Convert a dense matrix to a sparse matrix. As opposed to dense.sparseView() it keeps zero values in the matrix.
 */
template <class T>
void Dense2Sparse(const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>& dense, SparseMatrix<T>& sparse)
{
	sparse.resize(dense.rows(), dense.cols());
	sparse.resizeNonZeros(dense.rows() * dense.cols());
	for (int i = 0; i < int(dense.rows()); ++i) {
		sparse.outerIndexPtr()[i] = i * int(dense.cols());
	}
	sparse.outerIndexPtr()[dense.rows()] = int(dense.rows() * dense.cols());
	for (int i = 0; i < int(dense.cols()); ++i) {
		sparse.innerIndexPtr()[i] = i;
	}
	for (int i = 0; i < int(dense.rows()); ++i) {
		memcpy(sparse.innerIndexPtr() + i * int(dense.cols()), sparse.innerIndexPtr(), sizeof(int) * int(dense.cols()));
	}
	memcpy(sparse.valuePtr(), dense.data(), sizeof(T) * dense.rows() * dense.cols());
}


} // namespace nls
} //namespace epic
