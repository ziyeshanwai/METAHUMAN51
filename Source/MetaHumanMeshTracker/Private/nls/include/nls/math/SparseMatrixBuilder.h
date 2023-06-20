// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>
#include <nls/math/SparseMatrixReorder.h>

namespace epic {
namespace nls {


/**
 * The SparseMatrixInnerBuilder class is used to incrementally build a sparse inner (row or column) vector
 * by keeping track of the indexed data in an remapping vector and a separate row index and value vector.
 *
 * TODO: it might be faster to use a memset to clear the data once a certain fill factor has been reached
 */
template <class T>
class SparseMatrixInnerBuilder {
public:
    SparseMatrixInnerBuilder()
    {}

    //! Create an empty sparse inner vector of size @p innerSize
    void Init(int innerSize)
    {
        m_currentCounter = 0;
        m_indexMapping = Eigen::VectorXi::Constant(innerSize, -1);
        m_innerIndices.resize(innerSize);
        m_values.resize(innerSize);
    }

    //! Add @p value at index @p innerIndex
    void Add(int innerIndex, T value)
    {
        if (m_indexMapping[innerIndex] < 0) {
            // the innerIndex was not mapped so far, so remap the index and set the value
            int newIndex = m_currentCounter++;
            m_indexMapping[innerIndex] = newIndex;
            m_innerIndices[newIndex] = innerIndex;
            m_values[newIndex] = value;
        } else {
            // the innerIndex was used before, so we can simply accumualte the value
            m_values[m_indexMapping[innerIndex]] += value;
        }
    }

    //! @return the number of elements in the sparse vector
    int Size() const
    {
        return m_currentCounter;
    }

    //! Clears the remapping by going though all current indices and setting them to -1
    void Clear()
    {
        for (int i = 0; i < m_currentCounter; i++) {
            // reset index vector to -1
            m_indexMapping[m_innerIndices[i]] = -1;
        }
        m_currentCounter = 0;
    }

    //! @return the inner indices
    typename Eigen::VectorXi::ConstSegmentReturnType InnerIndices() const { return m_innerIndices.head(Size()); }

    //! @return the values
    typename Eigen::VectorX<T>::ConstSegmentReturnType Values() const { return m_values.head(Size()); }

private:
    int m_currentCounter = 0;
    Eigen::VectorXi m_indexMapping;
    Eigen::VectorXi m_innerIndices;
    Eigen::VectorX<T> m_values;
};



/**
 * The SparseMatrixBuilder class is used to incrementally build a sparse (row or column-major) matrix
 * by filling @p BLOCK_SIZE rows or columns at a time. In the following we sparse inner vector to represent
 * the block of rows or columns that are being filled. The builder can be used for either row or column major
 * matrices, but the caller needs to make sure to use the corresponding outerIndex and innerIndex.
 *
 * Use it in the following way:
 * SparseMatrixBuilder builder(M, N)
 * for (int b = 0; b < M; b += BLOCK_SIZE)
 * {
 *      StartBlock(b);
 *      // add arbitrary number of values using
 *      // Add(outerIndex, innerIndex, value)    where outerIndex needs to be b <= outerIndex < b + BLOCK_SIZE
 *      // ...
 *      FinalizeBlock();
 * }
 * SparseMatrix<T> sMat;
 * builder.Build(sMat);
 *
 * Note that the functionality is similar to Eigen::SparseMatrix::startVec(), ::insertBackByOuterInnerUnordered(), and ::finalize()
 * but supports multiple rows at a time.
 */
template <class T, int BLOCK_SIZE>
class SparseMatrixBuilder {
public:
    SparseMatrixBuilder(int outerSize, int innerSize, T allocationRatio = T(0.1))
    {
        for (int k = 0; k < BLOCK_SIZE; k++)
        {
            m_innerBuilders[k].Init(innerSize);
        }

        m_outerSize = outerSize;
        m_innerSize = innerSize;
        m_currentOuterIndexStart = 0;

        m_allocationIncrement = std::max<int>(outerSize + 1, int(outerSize * innerSize * allocationRatio));
        m_allocatedSize = m_allocationIncrement;
        m_currentSize = 0;
        m_outerIndices = Eigen::VectorXi::Zero(outerSize + 1);
        m_innerIndices.resize(m_allocatedSize);
        m_values.resize(m_allocatedSize);
    }

    //! Start the next block of sparse inner vectors
    void StartBlock(int outerIndex)
    {
        CARBON_PRECONDITION(outerIndex == m_currentOuterIndexStart, "outer index start needs to match block boundaries and start in sequential order");
        CARBON_PRECONDITION(!m_blockInProgress, "there is currently already a block in process");

        m_currentOuterIndexStart = outerIndex;
        m_blockInProgress = true;
    }

    /**
     * Add items to the current block of sparse vectors
     * @pre m_currentOuterIndexStart <= outerIndex < m_currentOuterIndexStart + BLOCK_SIZE
     */
    void Add(int outerIndex, int innerIndex, T value)
    {
        const int builderIndex = outerIndex - m_currentOuterIndexStart;
        CARBON_PRECONDITION(builderIndex >= 0 && builderIndex < BLOCK_SIZE, "adding values needs to be within current block bounds");
		if (builderIndex >= 0 && builderIndex < BLOCK_SIZE) {
			m_innerBuilders[builderIndex].Add(innerIndex, value);
		}
		else {
			throw std::runtime_error("outerIndex is out of range");
		}
    }

    /**
     * Finalizes the current block of sparse inner vectors and copies the data to internal arrays.
     */
    void FinalizeBlock()
    {
        CARBON_PRECONDITION(m_blockInProgress, "there is currently no block being processed");

        // copy block data
        for (int k = 0; k < BLOCK_SIZE; k++)
        {
            const int nonZeros = m_innerBuilders[k].Size();
            const int requiredSize = nonZeros + m_currentSize;
            if (requiredSize >= m_allocatedSize) {
                const int additionalMemory = std::max<int>(requiredSize - m_allocatedSize, m_allocationIncrement);
                m_allocatedSize += additionalMemory;
                m_innerIndices.conservativeResize(m_allocatedSize);
                m_values.conservativeResize(m_allocatedSize);
            }
            m_innerIndices.segment(m_outerIndices[m_currentOuterIndexStart + k], nonZeros) = m_innerBuilders[k].InnerIndices();
            m_values.segment(m_outerIndices[m_currentOuterIndexStart + k], nonZeros) = m_innerBuilders[k].Values();
            m_currentSize = m_outerIndices[m_currentOuterIndexStart + k] + nonZeros;
            m_outerIndices[m_currentOuterIndexStart + k + 1] = m_currentSize;
        }

        for (int k = 0; k < BLOCK_SIZE; k++)
        {
            m_innerBuilders[k].Clear();
        }

        m_currentOuterIndexStart += BLOCK_SIZE;
        m_blockInProgress = false;
    }


    //! Builds an Eigen::SparseMatrix from the assembled sparse vector data.
    template <int Options>
    void Build(Eigen::SparseMatrix<T, Options>& m)
    {
        if (m_currentOuterIndexStart != m_outerSize) {
            throw std::runtime_error("all blocks of the matrix have to been processed before building the sparse matrix");
        }

        const int rows = (Options == Eigen::RowMajor) ? m_outerSize : m_innerSize;
        const int cols = (Options == Eigen::RowMajor) ? m_innerSize : m_outerSize;

        m.resize(rows, cols);
        m.resizeNonZeros(m_currentSize);
        memcpy(m.outerIndexPtr(), m_outerIndices.data(), (m_outerSize + 1) * sizeof(int));
        memcpy(m.innerIndexPtr(), m_innerIndices.data(), m_currentSize * sizeof(int));
        memcpy(m.valuePtr(), m_values.data(), m_currentSize * sizeof(T));

        SparseMatrixReorder(m);
    }


private:
    int m_outerSize = 0;
    int m_innerSize = 0;
    int m_currentOuterIndexStart = 0;
    SparseMatrixInnerBuilder<T> m_innerBuilders[BLOCK_SIZE];

    bool m_blockInProgress = false;

    // data for assembled matrix
    int m_allocationIncrement = 1;
    int m_allocatedSize = 0;
    int m_currentSize = 0;
    Eigen::VectorXi m_outerIndices;
    Eigen::VectorXi m_innerIndices;
    Eigen::VectorX<T> m_values;
};


} // namespace nls
} //namespace epic
