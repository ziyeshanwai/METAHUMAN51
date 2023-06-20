// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanHighPass.h>

namespace epic::nls {

void VulkanHighPass::SetupKernels(const float sigma1, const int kernelSize1, const float sigma2, const int kernelSize2)
{
    m_sigma1 = sigma1;
    m_kernelSize1 = kernelSize1;
    m_sigma2 = sigma2;
    m_kernelSize2 = kernelSize2;

    if (sigma1 != 0) {
        m_gaussianBlur->SetupGaussianKernel(sigma1, kernelSize1);
    }
    if (sigma2 != 0) {
        m_gaussianBlur->SetupGaussianKernel(sigma2, kernelSize2);
    }
}

void VulkanHighPass::HighPass(const std::shared_ptr<VulkanComputeBuffer>& input,
                              const std::shared_ptr<VulkanComputeBuffer>& output,
                              const std::shared_ptr<VulkanComputeBuffer>& tmp1,
                              const std::shared_ptr<VulkanComputeBuffer>& tmp2,
                              const float scale,
                              const float delta,
                              VulkanComputeContext::HBatch batch)
{
    if (m_sigma1 > 0) {
        m_gaussianBlur->Blur(input, tmp2, output, m_sigma1, m_kernelSize1, batch);
        m_gaussianBlur->Blur(input, output, tmp1, m_sigma2, m_kernelSize2, batch);
        m_weightedAverage->WeightedAverage(tmp2, output, output, scale, -scale, delta, batch);

    } else {
        m_gaussianBlur->Blur(input, output, tmp1, m_sigma2, m_kernelSize2, batch);
        m_weightedAverage->WeightedAverage(input, output, output, scale, -scale, delta, batch);
    }
}

} // namespace epic::nls
