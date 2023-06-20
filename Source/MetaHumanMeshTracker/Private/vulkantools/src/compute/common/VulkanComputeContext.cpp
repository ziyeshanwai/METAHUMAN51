// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <thread>

using namespace epic::nls;

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
class VulkanComputeContext::Batch
{
public:

    VkCommandBuffer cmdBuff;
    uint32_t        idx;
    GpuTimeRange*   range;
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
VulkanComputeContext::VulkanComputeContext(std::shared_ptr<VulkanDevice> vulkanDevice)
{
    m_device = vulkanDevice;
    m_memory = VulkanMemory::Create(vulkanDevice);
    m_transientCommandPool = VulkanCommandPool::Create(vulkanDevice, /*transient=*/true);
    m_gpuTimer = VulkanGpuTimer::Create(vulkanDevice, 1024);
    CreateDescriptorSetLayouts();

    m_batches.reset(new Batch[cMaxNumCommandPools]);

    for (uint32_t c = 0; c < cMaxNumCommandPools; ++c)
    {
        VkCommandPoolCreateInfo poolInfo = {};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;

        poolInfo.queueFamilyIndex = m_device->AllFamily().value();
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

        VkResult res = vkCreateCommandPool(m_device->Device(), &poolInfo, nullptr, &m_cmdPools[c]);

        if (res != VK_SUCCESS)
            CARBON_CRITICAL("Failed to create Vulkan compute context command pool");

        VkCommandBufferAllocateInfo	cbaInfo = { };

        cbaInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        cbaInfo.commandPool = m_cmdPools[c];
        cbaInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cbaInfo.commandBufferCount = 1;

        res = vkAllocateCommandBuffers(m_device->Device(), &cbaInfo, &m_cmdBuffs[c]);
        if (res != VK_SUCCESS)
            CARBON_CRITICAL("Failed to create Vulkan compute context command buffer");

        VkFenceCreateInfo   fenceInfo{};
        fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;

        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

        res = vkCreateFence(m_device->Device(), &fenceInfo, nullptr, &m_cmdPoolsFences[c]);

        if (res != VK_SUCCESS)
            CARBON_CRITICAL("Failed to create Vulkan compute context fence");
    }

    OptionSet(cOptGPUProfiling, 0);
    OptionSet(cOptDebugBatchSubmission, 0);
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
VulkanComputeContext::~VulkanComputeContext()
{
    for (uint32_t c = 0; c < cMaxNumCommandPools; ++c)
    {
        if (m_cmdPools[c])
        {
            vkFreeCommandBuffers(m_device->Device(), m_cmdPools[c], 1, &m_cmdBuffs[c]);
            m_cmdBuffs[c] = nullptr;

            vkDestroyCommandPool(m_device->Device(), m_cmdPools[c], nullptr);
            m_cmdPools[c] = nullptr;
        }

        if (m_cmdPoolsFences[c])
        {
            vkDestroyFence(m_device->Device(), m_cmdPoolsFences[c], nullptr);
            m_cmdPoolsFences[c] = nullptr;
        }
    }
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void VulkanComputeContext::CreateDescriptorSetLayouts()
{
    {
        VkDescriptorSetLayoutBinding storageLayoutBinding = {};

        // a single descriptor set layout for a buffer storage binding
        storageLayoutBinding.binding = 0;
        storageLayoutBinding.descriptorCount = 1;
        storageLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        storageLayoutBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        storageLayoutBinding.pImmutableSamplers = nullptr;

        m_bufferDescriptorSetLayout = VulkanDescriptorSetLayout::CreateDescriptorSetLayout(m_device, { storageLayoutBinding });
    }

    {
        VkDescriptorSetLayoutBinding storageLayoutBinding = {};

        // a single descriptor set layout for an image storage binding
        storageLayoutBinding.binding = 0;
        storageLayoutBinding.descriptorCount = 1;
        storageLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        storageLayoutBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        storageLayoutBinding.pImmutableSamplers = nullptr;

        m_imageDescriptorSetLayout = VulkanDescriptorSetLayout::CreateDescriptorSetLayout(m_device, { storageLayoutBinding });
    }
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
VulkanComputeContext::HBatch VulkanComputeContext::BatchBegin(VulkanComputeContext::GpuTimeRange* range)
{
    constexpr uint32_t cMaxNumTries = 10;

    HBatch  batch = nullptr;

    // Grab first available command pool and it's command buffer
    VkCommandPool cmdPool = nullptr;

	for (uint32_t tries = 0; !cmdPool && tries < cMaxNumTries; ++tries)
	{
		const uint32_t nUsedCmdPools = cMaxNumCommandPools;

		for (uint32_t c = 0; c < nUsedCmdPools; ++c)
		{
			VkResult res;

			res = vkWaitForFences(m_device->Device(), 1, m_cmdPoolsFences + c, VK_TRUE, 0);
			if (res == VK_SUCCESS)
			{
				cmdPool = m_cmdPools[c];
				vkResetCommandBuffer(m_cmdBuffs[c], 0);

				vkResetFences(m_device->Device(), 1, &m_cmdPoolsFences[c]);
				batch = &m_batches[c];
				batch->idx = c;
				batch->cmdBuff = m_cmdBuffs[c];
				batch->range = range;
				break;
			}
		}

		std::this_thread::yield();
	}

    if (!batch)
    {
        // If there is no command list available, force a wait a on the first one in the list,
        // this allows the application to submit as many batches as it wants without the risk
        // of running operations outside of a batch, which hurts performance. The option
        // cOptDebugBatchSubmission can bet set to true to force the CPU to wait after
        // every submission.

        // The first command pool is likely to be the one to finish first, so wait on it
        const int waitIdx = 0;
        VkResult res = vkWaitForFences(m_device->Device(), 1, &m_cmdPoolsFences[waitIdx], VK_TRUE, std::numeric_limits<uint64_t>::max());
        if (res == VK_SUCCESS)
        {
			vkResetCommandBuffer(m_cmdBuffs[waitIdx], 0);

			vkResetFences(m_device->Device(), 1, &m_cmdPoolsFences[waitIdx]);
			batch = &m_batches[waitIdx];
			batch->idx = waitIdx;
			batch->cmdBuff = m_cmdBuffs[waitIdx];
			batch->range = range;
        }
        else
        {
            CARBON_CRITICAL("Error on vkWaitForFences");
        }
    }

    VkResult res;

    VkCommandBufferBeginInfo cbBegin = { };

    cbBegin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    cbBegin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    res = vkBeginCommandBuffer(batch->cmdBuff, &cbBegin);

    if (res != VK_SUCCESS)
        return nullptr;

    return batch;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool VulkanComputeContext::BatchEnd(VulkanComputeContext::HBatch& batch)
{
    return BatchFlush(batch, false);
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool VulkanComputeContext::BatchFlush(VulkanComputeContext::HBatch& batch, bool wait)
{
    vkEndCommandBuffer(batch->cmdBuff);

    VkCommandBuffer     cmdBuffs[] = { batch->cmdBuff };
    VkSubmitInfo        submit = { };

    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = cmdBuffs;

    VkResult res;

    res = vkQueueSubmit(m_device->AllQueue(), 1, &submit, wait ? nullptr : m_cmdPoolsFences[batch->idx]);

    if (res == VK_SUCCESS && wait)
        vkQueueWaitIdle(m_device->AllQueue());

    batch->cmdBuff = nullptr;
    batch->idx = uint32_t(-1);
    batch->range = nullptr;

    return res == VK_SUCCESS;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void VulkanComputeContext::DispatchKernel(
                                        HBatch batch,
                                        const VkExtent3D& groupCount,
                                        VulkanComputeShaderPipeline* pipe,
                                        const VkWriteDescriptorSet* dsets, uint32_t dsetCount,
                                        const void* pushConsts, size_t pushConstsSize,
                                        const char* dbgName)
{
    if (m_options[cOptDebugBatchSubmission])
        batch = nullptr;

    VkCommandBuffer cmdBuff;

    cmdBuff = batch ? batch->cmdBuff : m_transientCommandPool->BeginSingleTimeCommands();

    if (pushConsts)
        vkCmdPushConstants(cmdBuff, pipe->PipelineLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, (uint32_t) pushConstsSize, pushConsts);

    vkCmdPushDescriptorSetKHR(cmdBuff, VK_PIPELINE_BIND_POINT_COMPUTE, pipe->PipelineLayout(), 0, dsetCount, dsets);

    vkCmdBindPipeline(cmdBuff, VK_PIPELINE_BIND_POINT_COMPUTE, pipe->Pipeline());

    if (m_options[cOptGPUProfiling])
    {
        int id = m_gpuTimer->AddTimestamp(cmdBuff, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT);

        if (batch && batch->range)
        {
            if (batch->range->start == -1)
                batch->range->start = id;
            ++batch->range->count;
        }
    }

    VulkanCommandBufferDebugLabel   dbgLabel(cmdBuff);

    if (dbgName)
        dbgLabel.Begin(dbgName);

    vkCmdDispatch(cmdBuff, groupCount.width, groupCount.height, groupCount.depth);

    // Setup memory barriers to prevent hazards
    if (batch)
    {
        VkMemoryBarrier         barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;

        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

        vkCmdPipelineBarrier(cmdBuff,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            0,
            1, &barrier,
            0, nullptr,
            0, nullptr);
    }

    if (m_options[cOptGPUProfiling])
    {
        // Wait for all compute to finish, i.e. when all compute kernels are done
        m_gpuTimer->AddTimestamp(cmdBuff, VK_PIPELINE_STAGE_TRANSFER_BIT); // VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT
    }

    if (dbgName)
        dbgLabel.End();

    if (!batch)
        m_transientCommandPool->EndSingleTimeCommands(cmdBuff);
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void VulkanComputeContext::BatchCopyBufferToHost(VulkanComputeContext::HBatch batch, VkBuffer srcBuffer, VkBuffer dstBuffer)
{
    VkMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;

    barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;

    vkCmdPipelineBarrier(batch->cmdBuff,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_HOST_BIT,
        0,
        1, &barrier,
        0, nullptr,
        0, nullptr);

    vkCmdCopyBuffer(batch->cmdBuff, srcBuffer, dstBuffer, 0, nullptr);
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void VulkanComputeContext::WaitForAll()
{
    vkWaitForFences(m_device->Device(), cMaxNumCommandPools, m_cmdPoolsFences, VK_TRUE, std::numeric_limits<uint64_t>::max());
}

//-----------------------------------------------------------------------------
// Query batch execution timer
//-----------------------------------------------------------------------------
float VulkanComputeContext::TimerQueryValue(const GpuTimeRange& range) const
{
    float elapsed = 0.0f;

    for (int curr = range.start; curr >= 0 && curr < range.start + range.count; curr += 2)
    {
        uint64_t timestamps[2] = {0, 0};

        m_gpuTimer->QueryValues(timestamps, 2, curr);

        elapsed += (timestamps[1] - timestamps[0]) * 1e-6f;
    }

    return elapsed;
}
