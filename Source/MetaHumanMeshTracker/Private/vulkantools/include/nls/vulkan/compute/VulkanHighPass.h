// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/VulkanGaussianBlur.h>
#include <nls/vulkan/compute/VulkanWeightedAverage.h>


namespace epic::nls
{
    class VulkanHighPass
    {
    public:
        VulkanHighPass(std::shared_ptr<VulkanGaussianBlur> gaussianBlur, std::shared_ptr<VulkanWeightedAverage> weightedAverage)
            : m_gaussianBlur(gaussianBlur)
            , m_weightedAverage(weightedAverage)
        {
        }

        void SetupKernels(const float sigma1, const int kernelSize1, const float sigma2, const int kernelSize2);

        void HighPass(const std::shared_ptr<VulkanComputeBuffer>& input,
                      const std::shared_ptr<VulkanComputeBuffer>& output,
                      const std::shared_ptr<VulkanComputeBuffer>& tmp1,
                      const std::shared_ptr<VulkanComputeBuffer>& tmp2, //< only required if sigma1 != 0
                      const float scale,
                      const float delta,
                      VulkanComputeContext::HBatch batch = nullptr);

    private:
        std::shared_ptr<VulkanGaussianBlur> m_gaussianBlur;
        std::shared_ptr<VulkanWeightedAverage> m_weightedAverage;

        float m_sigma1;
        int m_kernelSize1;
        float m_sigma2;
        int m_kernelSize2;
    };
}
