// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>


#include <vector>
#include <iostream>
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

namespace epic::nls
{
    enum class DataOrder
    {
        RowsAreExamples,
        ColsAreExamples
    };

	/**
	 * @brief Class for training a PCA model incrementally (after an initial training batch)
     * @details Uses the method of Ross et al in:
     * "Incremental Learning for Robust Visual Tracking"
     * https://www.cs.toronto.edu/~dross/ivt/RossLimLinYang_ijcv.pdf
     * @tparam T must be a floating point type such as float, double, etc.
	 */
	template<typename T>
	class IncrementalPCA
	{
        static_assert(
            std::is_floating_point<T>::value,
            "T must be a floating-point type");

		public:
            /*
            * @brief Add a set of training examples to the model
            * @details If this block is the first set of examples
            * a normal PCA is computed, otherwise the existing model
            * is updated. In the second case, the data must have the
            * same number of dimensions as the original data used to train
            * the model.
            * @param dataMatrix A matrix of training data, where each row or column is
            * a training example according to the order flag
            * @param forgetFactor Typically set to 1.0 this controls how much
            * weight to give to the existing model versus the new data. A forgot
            * factor of 0.0 gives all the weight to the new data essentially
            * completely forgetting all the previous training.
            * @param maxModes The maximum number of modes to retain in the model.
            * @param maxVariance The maximum variance to retain.
            * @param order Flag to specify whether the data is column-wise or row-wise
            * @pre dataMatrix.cols() > 0
            * @pre dataMatrix.rows() > 0
            * @pre 0<= forgetFactor <= 1
            * @pre if IsTrained() then (order==DataOrder::ColsAreExamples && dataMatrix.rows()==NumVariables())
            *                     || (order==DataOrder::RowsreExamples && dataMatrix.cols()==NumVariables())
            */
            void AddTrainingData(
                const Eigen::Matrix<T, -1, -1>& dataMatrix,
                T forgetFactor,
                int maxModes,
                T maxVariance,
                DataOrder order)
            {
                assert(dataMatrix.cols() > 0);
                assert(dataMatrix.rows() > 0);
                assert(forgetFactor >= 0);
                assert(forgetFactor <= 1);
                assert(!IsTrained()
                    || (order == DataOrder::ColsAreExamples && dataMatrix.rows() == NumVariables())
                    || (order == DataOrder::RowsAreExamples && dataMatrix.cols() == NumVariables()));

                if (!IsTrained())
                {
                    Train(dataMatrix, maxModes, maxVariance, order);
                }
                else
                {
                    Update(dataMatrix, forgetFactor, maxModes, maxVariance, order);
                }
            }

            /*
            * @brief Check if the model is trained
            */
            bool IsTrained()
            {
                return m_isTrained;
            }

            /*
            * @brief Reset the model so that the next set of data passed
            * to it is treated as the first set of data.
            */
            void Reset()
            {
                m_n = 0;
                m_isTrained = false;
            }

            /*
            * @brief Given a set of data examples, return their parameterizations
            * @param dataMatrix A matrix of training data, where each row or column is
            * a training example according to the order flag
            * @param order Flag to specify whether the data is column-wise or row-wise
            * @return A matrix of parameters where each column corresponds to each column
            * of the input data.
            * @pre (IsTrained() && (order==DataOrder::ColsAreExamples && dataMatrix.rows()==NumVariables())
            *   || (order==DataOrder::RowsAreExamples && dataMatrix.cols()==NumVariables()))
            */
            Eigen::Matrix<T, -1, -1> Parameterize(const Eigen::Matrix<T, -1, -1>& dataMatrix, DataOrder order)
            {
                assert(IsTrained() && ((order == DataOrder::ColsAreExamples && dataMatrix.rows() == NumVariables())
                    || (order == DataOrder::RowsAreExamples && dataMatrix.cols() == NumVariables())));

                if (DataOrder::ColsAreExamples == order)
                {
                    Eigen::Matrix<T, -1, -1> meanSubtractedData = dataMatrix.colwise() - m_mean;
                    return m_U.transpose() * meanSubtractedData;
                }
                else
                {
                    Eigen::Matrix<T, -1, -1> meanSubtractedData = dataMatrix.transpose().colwise() - m_mean;
                    return (m_U.transpose() * meanSubtractedData).transpose();
                }

            }

            /*
            * @brief Given a set of parameterizations, return their reconstructions
            * @param parameterMatrix A matrix of parameters, where each row or column is an example
            * according to the order flag.
            * @param order Flag to specify whether the data is column-wise or row-wise
            * @return A matrix of reconstructions where each column corresponds to each column
            * of the input data.
            * @pre (IsTrained() && (order==DataOrder::ColsAreExamples && parameterMatrix.rows()==NumParams())
            *   || (order==DataOrder::RowsAreExamples && parameterMatrix.cols()==NumParams()))
            */
            Eigen::Matrix<T, -1, -1> Reconstruct(const Eigen::Matrix<T, -1, -1>& parameterMatrix, DataOrder order)
            {
                assert(IsTrained() && ((order == DataOrder::ColsAreExamples && parameterMatrix.rows() == NumVariables()) || (order == DataOrder::RowsAreExamples && parameterMatrix.cols() == NumVariables())));

                if (DataOrder::ColsAreExamples == order)
                {
                    Eigen::Matrix<T, -1, -1> temp = (m_U * parameterMatrix);
                    return temp.colwise() + m_mean;
                }
                else
                {
                    Eigen::Matrix<T, -1, -1> temp = (m_U * parameterMatrix.transpose());
                    return (temp.colwise() + m_mean).transpose();
                }
            }

            /*
            * @brief Given a set of data, parameterize, the reconstruct their approximations
            * @param dataMatrix A matrix of training data, where each row or column is
            * a training example according to the order flag
            * @param order Flag to specify whether the data is column-wise or row-wise
            * @return A matrix of reconstructions where each column corresponds to each column
            * of the input data.
            * @pre (IsTrained() && (order==DataOrder::ColsAreExamples && dataMatrix.rows()==NumVariables())
            *   || (order==DataOrder::RowsAreExamples && dataMatrix.cols()==NumVariables()))
            */
            Eigen::Matrix<T, -1, -1> BestFit(const Eigen::Matrix<T, -1, -1>& dataMatrix, DataOrder order)
            {
                assert(IsTrained() && ((order == DataOrder::ColsAreExamples && dataMatrix.rows() == NumVariables()) || (order == DataOrder::RowsAreExamples && dataMatrix.cols() == NumVariables())));

                return Reconstruct(Parameterize(dataMatrix,order),order);
            }

            /*
            * @brief Return the number of modes in the model.
            * @return The number of modes.
            */
            int NumModes()
            {
                return m_U.cols();
            }

            /*
            * @brief Return the number of variables in the data the model encodes.
            * @return The number of variables.
            */
            int NumVariables()
            {
                return m_U.rows();
            }

            /*
            * @brief Return a vector giving the standard deviations of the modes.
            * @return Vector of standard deviations.
            */
            const Eigen::Vector<T, -1>& StandardDeviations()
            {
                return m_stds;
            }

            /*
            * @brief Return the matrix of modes.
            * @return The modes.
            */
            const Eigen::Matrix<T, -1, -1>& U()
            {
                return m_U;
            }

		private:
            bool m_isTrained = false;
            Eigen::VectorX<T> m_mean;
            Eigen::Matrix<T, -1, -1> m_U;
            Eigen::Vector<T, -1> m_singularValues;
            Eigen::Vector<T, -1> m_stds;
            T m_n = 0.f;


            void Train(const Eigen::Matrix<T, -1, -1>& dataMatrix, int maxModes, T maxVariance, DataOrder order)
            {
                assert(dataMatrix.cols() > 0);
                assert(dataMatrix.rows() > 0);
                assert(maxModes > 0);

                Eigen::Matrix<T, -1, -1> meanCenteredDataMatrix;

                if (DataOrder::ColsAreExamples == order)
                {
                    m_mean = dataMatrix.rowwise().mean();
                    meanCenteredDataMatrix = dataMatrix.colwise() - m_mean;
                }
                else
                {
                    m_mean = dataMatrix.transpose().rowwise().mean();
                    meanCenteredDataMatrix = dataMatrix.transpose().colwise() - m_mean;
                }

                Eigen::JacobiSVD<Eigen::Matrix<T, -1, -1>> svd(meanCenteredDataMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
                m_U = svd.matrixU();

                //Flip U to ensure deterministic output
                Eigen::Matrix<T, -1, -1> absU = m_U.array().abs();
                for (int c = 0; c < absU.cols(); c++)
                {
                    int i = 0;
                    absU.col(c).maxCoeff(&i);
                    if (m_U.col(c)(i) < 0)
                    {
                        m_U.col(c) *= -T(1.0f);
                    }
                }

                m_singularValues = svd.singularValues();
                m_n = static_cast<T>(meanCenteredDataMatrix.cols());
                m_stds = m_singularValues / sqrt(T(m_n - 1));

                // Prevent keeping modes with singular values which are effectively zero
                T actualMaxVariance = std::min(T(0.99999), maxVariance);
                T totalVariance = 0;
                for (int i = 0; i < int(m_stds.size()); i++)
                {
                    const T variance = m_stds[i] * m_stds[i];
                    totalVariance += variance;
                }
                T sumVariance = 0;
                int finalIndex = 0;
                for (int i = 0; i < int(m_stds.size()); i++)
                {
                    const T variance = m_stds[i] * m_stds[i];
                    sumVariance += variance;
                    if (sumVariance / totalVariance <= actualMaxVariance)
                    {
                        finalIndex = i;
                    }
                }

                int rows = static_cast<int>(meanCenteredDataMatrix.rows());
                int cols = static_cast<int>(meanCenteredDataMatrix.cols());
                int hardMax = std::min(std::min(maxModes, std::min(rows, cols) - 1), finalIndex+1);
                if (m_U.cols() > hardMax)
                {
                    m_U.conservativeResize(m_U.rows(), hardMax);
                    m_singularValues.conservativeResize(hardMax);
                }
                m_isTrained = true;
            }

            void Update(const Eigen::Matrix<T, -1, -1>& dataMatrix, T forgetFactor, int maxModes, T maxVariance, DataOrder order)
            {
                assert(dataMatrix.cols() > 0);
                assert(forgetFactor >= 0);
                assert(forgetFactor <= 1);


                Eigen::VectorX<T> meanOfNewData;
                Eigen::Matrix<T, -1, -1> bHat;
                if (DataOrder::ColsAreExamples == order)
                {
                    meanOfNewData = dataMatrix.rowwise().mean();
                    bHat = dataMatrix.colwise() - meanOfNewData;
                }
                else
                {
                    meanOfNewData = dataMatrix.transpose().rowwise().mean();
                    bHat = dataMatrix.transpose().colwise() - meanOfNewData;
                }
                T nNewExamples = static_cast<T>(bHat.cols());


                T weight1 = static_cast<T>(forgetFactor * m_n) / static_cast<T>(forgetFactor * m_n + nNewExamples);
                T weight2 = static_cast<T>(nNewExamples) / static_cast<T>(forgetFactor * m_n + nNewExamples);
                Eigen::VectorX<T> newCombinedMean = weight1 * m_mean + weight2 * meanOfNewData;

                bHat.conservativeResize(bHat.rows(), bHat.cols() + 1);
                T multiplier = std::sqrt((nNewExamples * m_n) / (nNewExamples + m_n));
                bHat.col(bHat.cols() - 1) = multiplier * (m_mean-meanOfNewData);

                Eigen::Matrix<T, -1, -1> projection = m_U.transpose() * bHat;
                Eigen::Matrix<T, -1, -1> residual = bHat - m_U * projection;

                Eigen::JacobiSVD<Eigen::Matrix<T, -1, -1>> svd_orth(residual, Eigen::ComputeThinU | Eigen::ComputeThinV);

                Eigen::Matrix<T, -1, -1> orth = svd_orth.matrixU();
                Eigen::Matrix<T, -1, -1> Q;
                Q.resize(m_U.rows(), m_U.cols() + orth.cols());
                Q << m_U, orth;

                Eigen::Matrix<T, -1, -1> topRow;
                Eigen::Matrix<T, -1, -1> diag(m_singularValues.rows(), m_singularValues.rows());
                diag.setZero();
                for (int r = 0; r < diag.rows(); ++r)
                {
                    diag(r, r) = m_singularValues(r);
                }

                topRow.resize(diag.rows(), diag.cols() + projection.cols());
                topRow << diag * forgetFactor, projection;
                Eigen::Matrix<T, -1, -1> bottomRowL;
                Eigen::Matrix<T, -1, -1> bottomRowR;
                Eigen::Matrix<T, -1, -1> bottomRow;

                bottomRowR = orth.transpose() * residual;
                bottomRowL.resize(bottomRowR.rows(), topRow.cols() - bottomRowR.cols());
                bottomRowL.setZero();
                bottomRow.resize(bottomRowL.rows(), bottomRowL.cols() + bottomRowR.cols());
                bottomRow << bottomRowL, bottomRowR;
                Eigen::Matrix<T, -1, -1> R;
                R.resize(topRow.rows() + bottomRow.rows(), topRow.cols());
                R << topRow, bottomRow;

                Eigen::JacobiSVD<Eigen::Matrix<T, -1, -1>> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
                Eigen::Matrix<T, -1, -1> newU = svd.matrixU();

                //Flip U to ensure deterministic output
                Eigen::Matrix<T, -1, -1> absU = newU.array().abs();
                for (int c = 0; c < absU.cols(); c++)
                {
                    int i = 0;
                    absU.col(c).maxCoeff(&i);
                    if (newU.col(c)(i) < 0)
                    {
                        newU.col(c) *= -T(1.0f);
                    }
                }

                m_U = Q * newU;
                m_mean = newCombinedMean;
                m_singularValues = svd.singularValues();
                m_mean = newCombinedMean;
                m_n = nNewExamples + forgetFactor * m_n;
                m_stds = m_singularValues / sqrt(T(m_n - 1));

                T totalVariance = 0;
                for (int i = 0; i < int(m_stds.size()); i++)
                {
                    const T variance = m_stds[i] * m_stds[i];
                    totalVariance += variance;
                }
                T sumVariance = 0;
                int finalIndex = 0;
                T actualMaxVariance = std::min(T(0.99999), maxVariance);
                for (int i = 0; i < int(m_stds.size()); i++)
                {
                    const T variance = m_stds[i] * m_stds[i];
                    sumVariance += variance;
                    if (sumVariance / totalVariance <= actualMaxVariance)
                    {
                        finalIndex = i;
                    }
                }

                int hardMax = std::min(maxModes, finalIndex + 1);
                if (m_U.cols() > hardMax)
                {
                    m_U.conservativeResize(m_U.rows(), hardMax);
                    m_singularValues.conservativeResize(hardMax);
                }
            }



	};

} // namespace epic::nls
