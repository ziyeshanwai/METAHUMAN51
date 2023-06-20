// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffData.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {

/**
 * Function to calculated the weighted sum of multiple vectors: f(x) = w1 f1(x) + w2 f2(x) + ...
 */
template <class T>
class WeightedSumFunction
{
public:
	WeightedSumFunction() {}

	DiffData<T> Evaluate(const std::vector<DiffData<T>>& inputs,
						 const std::vector<T>& weights) const
	{
        CARBON_PRECONDITION(inputs.size() == weights.size(), "number of inputs need to match the number weights");

        if (inputs.size() == 0) {
			return DiffData<T>(Vector<T>());
		}

		if (inputs.size() == 1) {
			if (inputs[0].HasJacobian()) {
				return DiffData<T>(std::make_shared<Vector<T>>(weights[0] * inputs[0].Value()), inputs[0].Jacobian().Scale(weights[0]));
			} else {
				return DiffData<T>(std::make_shared<Vector<T>>(weights[0] * inputs[0].Value()));
			}
		}

        const int rows = int(inputs.front().Value().size());
        bool anyJacobian = false;
        int maxCols = 0;
        for (const DiffData<T>& diffData : inputs) {
            CARBON_PRECONDITION(int(diffData.Value().size()) == rows, "all inputs need to match in size");
            if (diffData.HasJacobian()) {
                anyJacobian = true;
                maxCols = std::max<int>(maxCols, diffData.Jacobian().Cols());
            }
        }

		VectorPtr<T> result = std::make_shared<Vector<T>>(rows);
        *result = weights[0] * inputs[0].Value();
        for (int i = 1; i < int(inputs.size()); i++) {
            *result += weights[i] * inputs[i].Value();
        }

		JacobianConstPtr<T> Jacobian;
        if (anyJacobian) {
            for (int i = 0; i < int(inputs.size()); i++) {
                if (inputs[i].HasJacobian()) {
                    if (Jacobian) {
                        // TODO: make more efficient
                        Jacobian = Jacobian->Add(inputs[i].Jacobian().Scale(weights[i]));
                    } else {
                        Jacobian = inputs[i].Jacobian().Scale(weights[i]);
                    }
                }
            }
        }

		return DiffData<T>(result, Jacobian);
	}
};


} // namespace nls
} //namespace epic
