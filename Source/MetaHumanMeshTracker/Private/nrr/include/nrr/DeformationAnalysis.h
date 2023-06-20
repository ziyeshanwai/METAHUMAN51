// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>

#include <string>
#include <vector>

namespace epic::nls {

template <class T>
class DeformationAnalysis
{
public:
    DeformationAnalysis();
    ~DeformationAnalysis();
    DeformationAnalysis(DeformationAnalysis&&);
    DeformationAnalysis(DeformationAnalysis&) = delete;
    DeformationAnalysis& operator=(DeformationAnalysis&&);
    DeformationAnalysis& operator=(const DeformationAnalysis&) = delete;

    /**
     * Set the mesh using a json-based mesh definition.
     * @param meshDefinitionJson  The definition of the mesh in json format
     */
    void SetMeshJsonBased(const std::string& meshDefinitionJson);

    /**
     * @return the various deformation errors (vertex distance, stretching, bending)
     */
    std::string AnalyseDeformationJsonBased(const std::string& vertices1Json,
                                            const std::string& vertices2Json);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
