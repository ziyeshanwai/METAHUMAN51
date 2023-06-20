// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanGaussianBlur.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_GaussianBlur.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/GaussianBlur.comp.h"
#include "shaders/GaussianBlur.comp.refl.h"

#include <cmath>
#include <vector>

namespace epic::nls {

static void CalculateGaussianKernel(float sigma, int kernelSize, float* kernel)
{
    float totalWeight = 0;
    for (int dx = -kernelSize/2; dx <= kernelSize/2; dx++) {
        const float weight = exp(-(dx * dx) / (2 * sigma * sigma));
        kernel[kernelSize/2 + dx] = weight;
        totalWeight += weight;
    }
    for (int dx = -kernelSize/2; dx <= kernelSize/2; dx++) {
        kernel[kernelSize/2 + dx] /= totalWeight;
    }
}

bool VulkanGaussianBlur::Init()
{
    return Kernel::Init< GaussianBlurReflection, GaussianBlurPushConstants >(SPV_GaussianBlur_comp_spv, sizeof(SPV_GaussianBlur_comp_spv));
}

int VulkanGaussianBlur::KernelSize(const float sigma)
{
    return 2 * int(std::ceil(3 * sigma)) + 1;
}

void VulkanGaussianBlur::SetupGaussianKernel(const float sigma)
{
    SetupGaussianKernel(sigma, KernelSize(sigma));
}

void VulkanGaussianBlur::SetupGaussianKernel(const float sigma, const int kernelSize)
{
    CARBON_PRECONDITION(kernelSize > 0, "kernel size needs to be positive");
    CARBON_PRECONDITION(kernelSize % 2, "kernel size needs to be odd");

    auto it = m_kernelBuffers.find({kernelSize, sigma});
    if (it == m_kernelBuffers.end()) {
        auto gaussianBuffer = VulkanComputeBuffer::Create(kernelSize, 1, m_ctx);
        std::vector<float> tmpKernel(kernelSize);
        CalculateGaussianKernel(sigma, kernelSize, tmpKernel.data());
        gaussianBuffer->CopyToDevice(tmpKernel.data(), tmpKernel.size() * sizeof(float));
        m_kernelBuffers[{kernelSize, sigma}] = gaussianBuffer;
    }
}

void VulkanGaussianBlur::Blur(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    const float sigma,
    const int kernelSize,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(inputBuffer, "input buffer must be valid");
    CARBON_PRECONDITION(outputBuffer, "output buffer must be valid");
    CARBON_PRECONDITION(inputBuffer->Width() == outputBuffer->Width(), "input and output buffer must have same width");
    CARBON_PRECONDITION(inputBuffer->Height() == outputBuffer->Height(), "input and output buffer must have same height");

    SetupGaussianKernel(sigma, kernelSize);

    if (!m_tmpBuffer || m_tmpBuffer->SizeInBytes() < inputBuffer->SizeInBytes()) {
        m_tmpBuffer = VulkanComputeBuffer::Create(inputBuffer->Width(), inputBuffer->Height(), m_ctx);
    }

    Blur(inputBuffer, outputBuffer, m_tmpBuffer, sigma, kernelSize, batch);
}

void VulkanGaussianBlur::Blur(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    std::shared_ptr<VulkanComputeBuffer> tmpBuffer,
    const float sigma,
    VulkanComputeContext::HBatch batch
)
{
    Blur(inputBuffer, outputBuffer, tmpBuffer, sigma, KernelSize(sigma), batch);
}

void VulkanGaussianBlur::Blur(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    std::shared_ptr<VulkanComputeBuffer> tmpBuffer,
    const float sigma,
    const int kernelSize,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(inputBuffer, "input buffer must be valid");
    CARBON_PRECONDITION(outputBuffer, "output buffer must be valid");
    CARBON_PRECONDITION(inputBuffer->Width() == outputBuffer->Width(), "input and output buffer must have same width");
    CARBON_PRECONDITION(inputBuffer->Height() == outputBuffer->Height(), "input and output buffer must have same height");
    CARBON_PRECONDITION(tmpBuffer->SizeInBytes() >= inputBuffer->SizeInBytes(), "tmp buffer does not have sufficient size");

    auto it = m_kernelBuffers.find({kernelSize, sigma});
    if (it == m_kernelBuffers.end()) {
        CARBON_CRITICAL("gaussian kernel with sigma {} and kernel size {} has not been set up", sigma, kernelSize);
    }

    {
        GaussianBlurPushConstants ubo = { inputBuffer->Width(), inputBuffer->Height(), kernelSize / 2, 1, 0 };

        GaussianBlurReflection::DescSet0_Update   dsu(inputBuffer->ManagedBuffer()->Buffer(), tmpBuffer->ManagedBuffer()->Buffer(), it->second->ManagedBuffer()->Buffer());

        Run(
            batch,
            {
                (uint32_t)std::ceil(inputBuffer->Width() / float(GaussianBlurComputeThreadSizeX)),
                (uint32_t)std::ceil(inputBuffer->Height() / float(GaussianBlurComputeThreadSizeY)),
                1
            },
            dsu.writes, dsu.nWrites,
            &ubo
        );
    }

    {
        GaussianBlurPushConstants ubo = { inputBuffer->Width(), inputBuffer->Height(), kernelSize / 2, 0, 1 };

        GaussianBlurReflection::DescSet0_Update   dsu(tmpBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer(), it->second->ManagedBuffer()->Buffer());

        Run(
            batch,
            {
                (uint32_t)std::ceil(inputBuffer->Width() / float(GaussianBlurComputeThreadSizeX)),
                (uint32_t)std::ceil(inputBuffer->Height() / float(GaussianBlurComputeThreadSizeY)),
                1
            },
            dsu.writes, dsu.nWrites,
            &ubo
        );
    }
}

} // namespace epic::nls
