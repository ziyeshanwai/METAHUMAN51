// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/common/VulkanDescriptorSetLayout.h>
#include <nls/vulkan/common/VulkanDevice.h>
#include <nls/vulkan/common/VulkanMemory.h>
#include <nls/vulkan/common/VulkanGpuTimer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

#include <carbon/Common.h>

#include<pma/PolyAllocator.h>

namespace epic {
namespace nls {

//!
class VulkanComputeContext {

public:

    enum OptionName
    {
        cOptGPUProfiling,                   //!< Enable/disable GPU profiling
        cOptDebugBatchSubmission,           //!< Every batch will be launched in sync, i.e. CPU will wait for GPU to finish

        cOptCount
    };

    class Kernel;
    class Batch;

    using HBatch = Batch*;

    //! Gpu time in range [start, start + count - 1]
    struct GpuTimeRange
    {
        int32_t    start = { -1 };
        int32_t    count = { 0 };
    };


    VulkanComputeContext(std::shared_ptr<VulkanDevice> vulkanDevice);

    ~VulkanComputeContext();

    static std::shared_ptr<VulkanComputeContext> Create(std::shared_ptr<VulkanDevice> vulkanDevice)
    {
        pma::PolyAllocator<VulkanComputeContext> polyAllocator{ MEM_RESOURCE };
        return std::allocate_shared<VulkanComputeContext>(polyAllocator, vulkanDevice);
    }

    std::shared_ptr<VulkanDevice> Device() const { return m_device; }
    std::shared_ptr<VulkanMemory> Memory() const { return m_memory; }
    std::shared_ptr<VulkanDescriptorSetLayout> BufferDescriptorSetLayout() const { return m_bufferDescriptorSetLayout; }
    std::shared_ptr<VulkanDescriptorSetLayout> ImageDescriptorSetLayout() const { return m_imageDescriptorSetLayout; }
    VulkanCommandPool* TransientCommandPool() { return m_transientCommandPool.get(); }

    void OptionSet(OptionName name, uint32_t value) { m_options[name] = value; }
    uint32_t OptionGet(OptionName name) const { return m_options[name]; }

    //! Begin new batch.
    //! If range is provieded, the gpu compute kernel execution time will be profiled and elapsed time can be queried
    //! later by TimerQueryValues().
    HBatch BatchBegin(GpuTimeRange* range = nullptr);

    //! End current submission batch, i.e. submit multiple command buffers
    bool BatchEnd(HBatch& batch);

    //! Dispatch kernel in async
    template< typename ConstantValuesT >
    void BatchKernel(
                        HBatch batch,
                        const VkExtent3D& groupCount,
                        VulkanComputeShaderPipeline* pipe,
                        const VkWriteDescriptorSet* dsets, uint32_t dsetCount,
                        const ConstantValuesT* pushConsts = nullptr,
                        const char* dbgName = "")
    {
        DispatchKernel(batch, groupCount, pipe, dsets, dsetCount, pushConsts, sizeof(ConstantValuesT), dbgName);
    }

    //! Insert host read barrier into current command dispatch
    void BatchCopyBufferToDevice(HBatch batch, VkBuffer srcBuffer, VkBuffer dstBuffer);

    //! Insert host read barrier into current command dispatch
    void BatchCopyBufferToHost(HBatch batch, VkBuffer srcBuffer, VkBuffer dstBuffer);

    //! Wait for all batches to finish
    void WaitForAll();

    //! Run kernel in sync, i.e. wait for kernel to finish
    //! NOTE: This is introducing stalls on CPU side and starvation for the GPU side, should only be used for debugging
    template< typename ConstantValuesT >
    void RunKernel(
                        const VkExtent3D& groupCount,
                        VulkanComputeShaderPipeline* pipe,
                        const VkWriteDescriptorSet* dsets, uint32_t dsetCount,
                        const ConstantValuesT* pushConsts = nullptr,
                        const char* dbgName = "")
    {
        DispatchKernel(nullptr, groupCount, pipe, dsets, dsetCount, pushConsts, sizeof(ConstantValuesT), dbgName);
    }

    //! Reset GPU timer
    void TimerReset()
    {
        m_gpuTimer->Reset();
        m_numTimestamps = 0;
    }

    //! Query elapsed time in range
    //! NOTE: Don't forget to call TimerReset() when you want to start gpu timer.
    float TimerQueryValue(const GpuTimeRange& range) const;

private:

    void CreateDescriptorSetLayouts();

    bool BatchFlush(HBatch& batch, bool wait);

    void DispatchKernel(
                        HBatch batch,
                        const VkExtent3D& groupCount,
                        VulkanComputeShaderPipeline* pipe,
                        const VkWriteDescriptorSet* dsets, uint32_t dsetCount,
                        const void* pushConsts, size_t pushConstsSize,
                        const char* dbgName);

private:

    constexpr static const uint32_t cMaxNumCommandPools = 16;

    std::shared_ptr<VulkanDevice> m_device;
    std::shared_ptr<VulkanMemory> m_memory;
    std::shared_ptr<VulkanDescriptorSetLayout> m_bufferDescriptorSetLayout;
    std::shared_ptr<VulkanDescriptorSetLayout> m_imageDescriptorSetLayout;
    std::unique_ptr<VulkanCommandPool> m_transientCommandPool;
    std::unique_ptr<VulkanGpuTimer> m_gpuTimer;

    VkCommandPool                   m_cmdPools[cMaxNumCommandPools];
    VkCommandBuffer                 m_cmdBuffs[cMaxNumCommandPools];
    VkFence                         m_cmdPoolsFences[cMaxNumCommandPools];
    std::unique_ptr< Batch[] >      m_batches;
    uint32_t                        m_numTimestamps{ 0 };

    uint32_t                        m_options[cOptCount];
};

//! Base class for Vulkan compute kernels
class VulkanComputeContext::Kernel
{
public:

    Kernel(Kernel&&) = default;

    virtual ~Kernel() = default;

    //! Initialize kernel pipeline
    virtual bool Init() = 0;

    //! Return kernel name
    const char* Name() const { return m_name; }

protected:

    //! Default constructor
    Kernel(std::shared_ptr<VulkanComputeContext>& ctx, const char* name)
        : m_ctx(ctx)
        , m_name(name)
    {}

    //! Initialize kernel
    template< typename PipelineReflectionT, typename PushConstantsT >
    bool Init(const uint8_t* spirvData, uint32_t spirvDataSize)
    {
        if (!m_pipeline)
        {
            typename PipelineReflectionT::DescSet0_Init  layoutBinds;

            std::vector< VkDescriptorSetLayoutBinding >     dsetLayoutBindings(layoutBinds.bindings, layoutBinds.bindings + layoutBinds.nBindings);

            m_dsetLayout = VulkanDescriptorSetLayout::CreatePushDescriptorSetLayout(m_ctx->Device(), dsetLayoutBindings);

            m_pipeline = VulkanComputeShaderPipeline::Create(
                                                            m_ctx,
                                                            spirvData, spirvDataSize,
                                                            {m_dsetLayout},
                                                            sizeof(PushConstantsT));
        }

        CARBON_ASSERT(m_pipeline, "Compute pipeline is not valid");

        return m_pipeline.get() != nullptr;
    }

    //! Run kernel in batch
    //! NOTE:
    //!     If batch is not provieded, the kernel execution will stall CPU and starve GPU
    //!     It should be only used for debugging purposes!
    template< typename ConstantValuesT >
    void Run(VulkanComputeContext::HBatch batch, const VkExtent3D& groupCount, const VkWriteDescriptorSet* dsets, uint32_t dsetCount, const ConstantValuesT* pushConsts = nullptr)
    {
        if (batch)
            m_ctx->BatchKernel(batch, groupCount, m_pipeline.get(), dsets, dsetCount, pushConsts, m_name);
        else
        {
            LOG_WARNING("Kernel {} is not running in batch", m_name);

            m_ctx->RunKernel(groupCount, m_pipeline.get(), dsets, dsetCount, pushConsts, m_name);
        }
    }

    //! Run kernel in synchronous mode
    //! NOTE:
    //!     If batch is not provieded, the kernel execution will stall CPU and starve GPU
    //!     It should be only used for debugging purposes!
    template< typename ConstantValuesT >
    void Run(const VkExtent3D& groupCount, const VkWriteDescriptorSet* dsets, uint32_t dsetCount, const ConstantValuesT* pushConsts = nullptr)
    {
        //LOG_WARNING("Kernel {} is not running in batch", m_name);

        m_ctx->RunKernel(groupCount, m_pipeline.get(), dsets, dsetCount, pushConsts, m_name);
    }

    // Members:
    std::shared_ptr<VulkanComputeContext> m_ctx;
    std::shared_ptr<VulkanComputeShaderPipeline> m_pipeline;
    std::shared_ptr<VulkanDescriptorSetLayout> m_dsetLayout;

    const char* m_name;
};


} //namespace nls
} //namespace epic
