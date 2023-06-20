// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffData.h>

#include <vector>

namespace epic::nls {

/**
 * Cost
 */
template <class T>
class Cost
{
public:
	Cost() = default;

    void Add(const DiffData<T>& diffdata, T weight, const std::string& name = std::string())
    {
        if (diffdata.Size() > 0 && weight > 0) {
            m_costTerms.push_back({weight, diffdata, name});
        }
    }

    void Add(const Cost& other)
    {
        m_costTerms.insert(m_costTerms.begin(), other.m_costTerms.begin(), other.m_costTerms.end());
    }

    void Add(const Cost& other, T weight)
    {
        if (weight > 0) {
            for (int i = 0; i < other.NumTerms(); ++i) {
                m_costTerms.push_back({weight * other.m_costTerms[i].weight, other.m_costTerms[i].diffdata, other.m_costTerms[i].name});
            }
        }
    }

    DiffData<T> CostToDiffData() const;

    int NumTerms() const { return static_cast<int>(m_costTerms.size()); }

    int Size() const {
        int size = 0;
        for (const auto& costTerm : m_costTerms) {
            size += costTerm.diffdata.Size();
        }
        return size;
    }

private:
    struct CostTerm {
        T weight;
        DiffData<T> diffdata;
        std::string name;
    };

    std::vector<CostTerm> m_costTerms;
};

} // namespace epic::nls
