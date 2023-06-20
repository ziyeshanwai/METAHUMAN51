// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>

#include <string>

namespace dna {
class StreamReader;
}

namespace epic::nls {

/**
 * Wrapper to load RigLogic DNA resources.
 */
class RigLogicDNAResource {
public:
    RigLogicDNAResource();
    ~RigLogicDNAResource();
    RigLogicDNAResource(RigLogicDNAResource&&);
    RigLogicDNAResource(const RigLogicDNAResource&) = delete;
    RigLogicDNAResource& operator=(RigLogicDNAResource&&);
    RigLogicDNAResource& operator=(const RigLogicDNAResource&) = delete;

    /**
     * Read the DNA file from \p dnaFile.
     * @param [in] retain  Cache the resource in memory for subsequent accesses. Only necessary if the same DNA file is being loaded multiple times.
     */
    static std::shared_ptr<const RigLogicDNAResource> LoadDNA(const std::string& dnaFile, bool retain);

    /**
     * Returns the pointer to the underlying RL4 StreamReader. Only valid as long as the instance is alive.
     */
    dna::StreamReader* Stream() const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
