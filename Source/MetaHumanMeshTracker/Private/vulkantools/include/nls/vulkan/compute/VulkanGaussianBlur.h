// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

#include <map>
#include <mutex>

namespace epic {
namespace nls {

class VulkanGaussianBlur : public VulkanComputeContext::Kernel {

public:
    VulkanGaussianBlur(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeGaussianBlur")
    {
        VulkanGaussianBlur::Init();
    }

    VulkanGaussianBlur(VulkanGaussianBlur&&) = default;

    bool Init() override;

    static int KernelSize(const float sigma);

    void SetupGaussianKernel(const float sigma);
    void SetupGaussianKernel(const float sigma, const int kernelSize);

    void Blur(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        std::shared_ptr<VulkanComputeBuffer> tmpBuffer,
        const float sigma,
        VulkanComputeContext::HBatch batch = nullptr
    );

    void Blur(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        std::shared_ptr<VulkanComputeBuffer> tmpBuffer,
        const float sigma,
        const int kernelSize,
        VulkanComputeContext::HBatch batch = nullptr
    );

    /**
     * @brief Runs Gaussian blur on the input buffer and stores the result in the outputBuffer. Creates or modifies an internal temporary buffer.
     * Updates the Gaussian kernel if sigma or kernelSize changed. It is recommended to use the explicit methods above to
     * set the gaussian kenrel and to pass in a valid temporary buffer.
     */
    void Blur(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        const float sigma,
        const int kernelSize,
        VulkanComputeContext::HBatch batch = nullptr
    );

    void ClearKernels() { m_kernelBuffers.clear(); }

private:
    std::shared_ptr<VulkanComputeBuffer> m_tmpBuffer;
    std::shared_ptr<VulkanComputeBuffer> m_gaussianBuffer;

    std::map<std::pair<int, float>, std::shared_ptr<VulkanComputeBuffer>> m_kernelBuffers;
};


} //namespace nls
} //namespace epic
