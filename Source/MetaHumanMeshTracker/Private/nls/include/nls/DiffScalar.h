// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic {
namespace nls {


/**
 * Base data representation representing a scalar and a Jacobian as a sparse vector
 * value: f(x)
 * jacobian: df/dx
 */
template <typename T>
class DiffScalar
{
public:
	DiffScalar(const T value, const Eigen::SparseVector<T>& jacobian) : m_value(value), m_jacobian(jacobian)
	{}

	explicit DiffScalar(const T value) : m_value(value), m_jacobian() {}

	const T Value() const { return m_value; }

	bool HasJacobian() const { return (m_jacobian.size() > 0); }
	const Eigen::SparseVector<T>& Jacobian() const { return m_jacobian; }

private:
	T m_value;
	Eigen::SparseVector<T> m_jacobian;
};

//! Extract the @p index row from @p diffData.
template <typename T>
DiffScalar<T> ExtractScalar(const DiffData<T>& diffData, int index)
{
	if (diffData.HasJacobian()) {
		return DiffScalar<T>(diffData.Value()[index], diffData.Jacobian().Row(index));
	} else {
		return DiffScalar<T>(diffData.Value()[index]);
	}
}


//! Assemble all DiffScalar into on DiffData
template <typename T>
DiffData<T> AssembleDiffData(const std::vector<DiffScalar<T>>& diffScalars)
{
	const int rows = int(diffScalars.size());
	int cols = 0;
	int numNonZeros = 0;
	std::shared_ptr<Vector<T>> values = std::make_shared<Vector<T>>(rows);

	for (size_t i = 0; i < diffScalars.size(); ++i) {
		const DiffScalar<T>& diffScalar = diffScalars[i];
		cols = std::max<int>(cols, static_cast<int>(diffScalar.Jacobian().size()));
		numNonZeros += static_cast<int>(diffScalar.Jacobian().nonZeros());
		(*values)[i] = diffScalar.Value();
	}

	SparseMatrixPtr<T> sparseMatrix;

	if (numNonZeros > 0) {
		sparseMatrix = std::make_shared<SparseMatrix<T>>(rows, cols);
		sparseMatrix->resizeNonZeros(numNonZeros);

		int* outputRowIndices = sparseMatrix->outerIndexPtr();
		int* outputColIndices = sparseMatrix->innerIndexPtr();
		T* outputValues = sparseMatrix->valuePtr();
		int outputCurrIndex = 0;

		for (int r = 0; r < rows; ++r) {
			outputRowIndices[r] = outputCurrIndex;
			const DiffScalar<T>& diffScalar = diffScalars[r];
			cols = std::max<int>(cols, static_cast<int>(diffScalar.Jacobian().size()));
			numNonZeros += static_cast<int>(diffScalar.Jacobian().nonZeros());
			(*values)[r] = diffScalar.Value();
		}

		for (int r = 0; r < rows; r++) {
			outputRowIndices[r] = outputCurrIndex;
			typename Eigen::SparseVector<T>::InnerIterator it(diffScalars[r].Jacobian());
			for (; it; ++it) {
				outputColIndices[outputCurrIndex] = it.index();
				outputValues[outputCurrIndex] = it.value();
				++outputCurrIndex;
			}
		}
		outputRowIndices[rows] = outputCurrIndex;
	}

	return DiffData<T>(values, sparseMatrix ? std::make_shared<SparseJacobian<T>>(sparseMatrix) : nullptr);
}


// Unary +
template <typename T>
inline DiffScalar<T> operator+(const DiffScalar<T>& f) {
	return f;
}

// Unary -
template <typename T>
inline DiffScalar<T> operator-(const DiffScalar<T>& f) {
	return DiffScalar<T>(-f.Value(), -f.Jacobian());
}

template <class T>
Eigen::SparseVector<T> AutoResizeAdd(const Eigen::SparseVector<T>& a, const Eigen::SparseVector<T>& b)
{
	if (a.size() && b.size()) {
		const typename Eigen::SparseVector<T>::Index size = std::max<typename Eigen::SparseVector<T>::Index>(a.size(), b.size());
		const typename Eigen::SparseVector<T>::Index maxNonZeros = std::min<typename Eigen::SparseVector<T>::Index>(size, a.nonZeros() + b.nonZeros());
		Eigen::SparseVector<T> vec(size);
		vec.reserve(maxNonZeros);
		typename Eigen::SparseVector<T>::InnerIterator ita(a);
		typename Eigen::SparseVector<T>::InnerIterator itb(b);
		while (true) {
			if (ita && itb && (ita.index() == itb.index())) {
				vec.insertBack(ita.index()) = ita.value() + itb.value();
				++ita;
				++itb;
			} else if (ita && (!itb || (ita.index() < itb.index()))) {
				vec.insertBack(ita.index()) = ita.value();
				++ita;
			} else if (itb && (!ita || (itb.index() < ita.index()))) {
				vec.insertBack(itb.index()) = itb.value();
				++itb;
			}
			else {
				break;
			}
		}
		return vec;
	} else if (a.size()) {
		return a;
	} else {
		return b;
	}
}

// Binary +
template <typename T>
inline DiffScalar<T> operator+(const DiffScalar<T>& f, const DiffScalar<T>& g) {
	return DiffScalar<T>(f.Value() + g.Value(), AutoResizeAdd<T>(f.Jacobian(), g.Jacobian()));
}

// Binary + with scalar f + s
template <typename T>
inline DiffScalar<T> operator+(const DiffScalar<T>& f, T s) {
	return DiffScalar<T>(f.Value() + s, f.Jacobian());
}

// Binary + with scalar s + f
template <typename T>
inline DiffScalar<T> operator+(T s, const DiffScalar<T>& f) {
	return DiffScalar<T>(f.Value() + s, f.Jacobian());
}

// Binary -
template <typename T>
inline DiffScalar<T> operator-(const DiffScalar<T>& f, const DiffScalar<T>& g) {
	return f + (-g);
}

// Binary - with scalar f - s
template <typename T>
inline DiffScalar<T> operator-(const DiffScalar<T>& f, T s) {
	return DiffScalar<T>(f.Value() - s, f.Jacobian());
}

// Binary - with scalar s - f
template <typename T>
inline DiffScalar<T> operator-(T s, const DiffScalar<T>& f) {
	return DiffScalar<T>(s - f.Value() , -f.Jacobian());
}

// Binary *
template <typename T>
inline DiffScalar<T> operator*(const DiffScalar<T>& f, const DiffScalar<T>& g) {
	return DiffScalar(f.Value() * g.Value(), AutoResizeAdd<T>(f.Value() * g.Jacobian(), f.Jacobian() * g.Value()));
}

// Binary * with scalar f * s
template <typename T>
inline DiffScalar<T> operator*(const DiffScalar<T>& f, T s) {
	return DiffScalar(f.Value() * s, f.Jacobian() * s);
}

// Binary * with scalar s * f
template <typename T>
inline DiffScalar<T> operator*(T s, const DiffScalar<T>& f) {
	return DiffScalar<T>(f.Value() * s, f.Jacobian() * s);
}

// Binary /
template <typename T>
inline DiffScalar<T> operator/(const DiffScalar<T>& f, const DiffScalar<T>& g) {
	const T invG = T(1) / g.Value();
	const T fdivG = f.Value() * invG;
	return DiffScalar<T>(fdivG, AutoResizeAdd<T>(f.Jacobian(), - fdivG * g.Jacobian()) * invG);
}

// Binary / with scalar f / s
template <typename T>
inline DiffScalar<T> operator/(const DiffScalar<T>& f, T s) {
	const T invS = T(1) / s;
	return DiffScalar(f.Value() * invS, f.Jacobian() * invS);
}

// Binary / with scalar s / f
template <typename T>
inline DiffScalar<T> operator/(T s, const DiffScalar<T>& f) {
	const T invF = T(1) / f.Value();
	return DiffScalar(s * invF, - invF * invF * s * f.Jacobian());
}

// maximum (the Jacobian is not well defined when both values are equal, so in this case we return the average Jacobian)
template <typename T>
inline DiffScalar<T> max(const DiffScalar<T> f, const DiffScalar<T>& g) {
	if (f.Value() > g.Value()) {
		return DiffScalar<T>(f.Value(), f.Jacobian());
	} else if (f.Value() < g.Value()) {
		return DiffScalar<T>(g.Value(), g.Jacobian());
	} else {
		return DiffScalar<T>(f.Value(), T(0.5) * (f.Jacobian() + g.Jacobian()));
	}
}

template <typename T>
inline DiffScalar<T> max(T s, const DiffScalar<T>& f) {
	if (s > f.Value()) {
		return DiffScalar<T>(s);
	} else {
		return DiffScalar<T>(f.Value(), f.Jacobian());
	}
}

template <typename T>
inline DiffScalar<T> max(const DiffScalar<T>& f, T s) { return max(s, f); }

// minimum (the Jacobian is not well defined when both values are equal, so in this case we return the average Jacobian)
template <typename T>
inline DiffScalar<T> min(const DiffScalar<T> f, const DiffScalar<T>& g) {
	if (f.Value() < g.Value()) {
		return DiffScalar<T>(f.Value(), f.Jacobian());
	} else if (f.Value() > g.Value()) {
		return DiffScalar<T>(g.Value(), g.Jacobian());
	} else {
		return DiffScalar<T>(f.Value(), T(0.5) * (f.Jacobian() + g.Jacobian()));
	}
}

template <typename T>
inline DiffScalar<T> min(T s, const DiffScalar<T>& f) {
	if (s < f.Value()) {
		return DiffScalar<T>(s);
	} else {
		return DiffScalar<T>(f.Value(), f.Jacobian());
	}
}

template <typename T>
inline DiffScalar<T> min(const DiffScalar<T>& f, T s) { return min(s, f); }

// clamp
template <typename T>
inline DiffScalar<T> clamp(const DiffScalar<T>& f, T a, T b) {
	return max<T>(a, min<T>(b, f));
}

} // namespace nls
} //namespace epic
