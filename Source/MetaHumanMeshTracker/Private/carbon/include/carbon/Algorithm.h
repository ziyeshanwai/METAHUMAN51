// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>

#include <vector>
#include <set>

namespace epic::carbon {

/**
 * Concatenates two vectors with undown direction along one matching end point (removing the duplicate).
 * Fails if there is no matching end point or if either vector is empty.
 */
template <class T>
bool ConcatenateVectorsWithMatchingEndPointsAndUnknownDirection(const std::vector<T>& vector1, const std::vector<T>& vector2, std::vector<T>& mergedVector)
{
    if (vector1.empty() || vector2.empty()) {
        // no matching of vectors if any is empty
        return false;
    }

    std::vector<T> newMergedVector;
    newMergedVector.reserve(vector1.size() + vector2.size() - 1);
    bool success = false;

    if (vector2.front() == vector1.front()) {
        newMergedVector.insert(newMergedVector.begin(), vector1.rbegin(), vector1.rend());
        newMergedVector.insert(newMergedVector.end(), vector2.begin() + 1, vector2.end());
        success = true;
    } else if (vector2.front() == vector1.back()) {
        newMergedVector.insert(newMergedVector.begin(), vector1.begin(), vector1.end());
        newMergedVector.insert(newMergedVector.end(), vector2.begin() + 1, vector2.end());
        success = true;
    } else if (vector2.back() == vector1.front()) {
        newMergedVector.insert(newMergedVector.begin(), vector2.begin(), vector2.end());
        newMergedVector.insert(newMergedVector.end(), vector1.begin() + 1, vector1.end());
        success = true;
    } else if (vector2.back() == vector1.back()) {
        newMergedVector.insert(newMergedVector.begin(), vector2.begin(), vector2.end());
        newMergedVector.insert(newMergedVector.end(), vector1.rbegin() + 1, vector1.rend());
        success = true;
    }

    if (success) {
        mergedVector.swap(newMergedVector);
    }
    return success;
}

}  // namespace epic::carbon
